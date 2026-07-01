// ============================================================================
//  TVC_FlightComputer.ino
//  Thrust-vector-control flight software for a custom Teensy 4.0 rocket
//  avionics board:
//    Teensy 4.0 + GY-521 (MPU6050) + GY-BMP280 + SPI MicroSD + W25Q128 flash
//    + 4x N-MOSFET pyro channels + active buzzer + common-cathode RGB LED.
//
//  Build: Arduino IDE / arduino-cli with the Teensy core (PJRC).
//  Board: "Teensy 4.0". See README.md.
//
//  Architecture (based on proven open-source TVC computers such as
//  tomkuttler/TVC-Rocket-Flight-Code and AdamMarciniak/CygnusX1):
//
//    250 Hz loop:  MPU6050 -> Mahony quaternion -> 2x PID -> 2 servos
//     50 Hz loop:  BMP280 -> altitude -> apogee detection
//    100 Hz:       black-box logging (RAM ring buffer -> SD + flash on landing)
//
//  State machine:
//    IDLE -> (arm) -> CALIBRATING -> PAD_ARMED -> BOOST -> COAST
//         -> DESCENT (chute pyro fired) -> LANDED (log committed)
//    BOOST tilt > ABORT_TILT_DEG  =>  ABORT (servos centered, chute still
//    deploys at apogee / failsafe timer).
//
//  RGB LED status:      green slow blink  = idle, healthy
//                       blue solid        = calibrating (hold still!)
//                       red solid         = ARMED / in flight
//                       cyan              = descent
//                       green fast blink  = landed, log saved
//                       red fast blink    = sensor failure
//
//  !!! SAFETY !!!
//  * The pyro battery MUST run through a physical arming switch. This code
//    also refuses to fire pyros unless software-armed, but software is not
//    a safety device.
//  * Never connect live igniters while USB-connected or while bench testing.
// ============================================================================

#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "MPU6050.h"
#include "BMP280.h"
#include "Attitude.h"
#include "Control.h"
#include "Logger.h"

// ---------------------------------------------------------------------------
// Flight state machine
// ---------------------------------------------------------------------------
enum FlightState : uint8_t {
  STATE_IDLE = 0,       // powered, disarmed. Servos centered, pyros locked.
  STATE_CALIBRATING,    // gyro bias + attitude convergence (hold still!)
  STATE_PAD_ARMED,      // waiting for launch. Pyros hot (if switch closed).
  STATE_BOOST,          // motor burning — TVC ACTIVE
  STATE_COAST,          // burnout to apogee — servos centered
  STATE_DESCENT,        // chute fired, falling
  STATE_LANDED,         // on the ground, log saved
  STATE_ABORT,          // tilt limit exceeded during boost
  STATE_SENSOR_FAIL     // a sensor did not init — refuse to arm
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
MPU6050   imu(Wire, MPU6050_ADDR);
BMP280    baro(Wire, BMP280_ADDR);
Attitude  attitude;
TVCMount  tvc;
Logger    logger;
AxisPID   pidX(PID_KP, PID_KI, PID_KD, PID_I_LIMIT_DEG);
AxisPID   pidY(PID_KP, PID_KI, PID_KD, PID_I_LIMIT_DEG);

FlightState state = STATE_IDLE;

static const uint8_t PYRO_PINS[4] = {PIN_PYRO_1, PIN_PYRO_2,
                                     PIN_PYRO_3, PIN_PYRO_4};

float gyroBias[3] = {0, 0, 0};        // deg/s, from pad calibration
float accelBody[3], gyroBodyDps[3];   // latest IMU sample in BODY frame
float imuTempC = 0;

float groundPressurePa = 101325.0f;
float altitudeM = 0, maxAltitudeM = 0;
float baroTempC = 0;
int   descendingSamples = 0;

uint32_t launchMs = 0;                // millis() at liftoff
uint32_t stateEnteredMs = 0;
uint32_t highAccelSinceMs = 0;        // for launch detection debounce
uint32_t landedStableSinceMs = 0;
float    landedRefAltM = 0;

bool     pyroFired[4] = {false, false, false, false};
uint32_t pyroOnSinceMs[4] = {0, 0, 0, 0};

// loop scheduling
uint32_t nextControlUs = 0, nextBaroUs = 0, nextLogUs = 0, lastLoopUs = 0;

// ---------------------------------------------------------------------------
// Runtime-tunable settings, persisted in the Teensy's emulated EEPROM.
// Change over serial ("set kp 0.35"), persist with "save". Saved values
// override the config.h defaults at boot. The tuner UI uses this too.
// ---------------------------------------------------------------------------
static const uint32_t SETTINGS_MAGIC = 0x54564332;   // "TVC2"

struct __attribute__((packed)) Settings {
  uint32_t magic;
  float    kp, ki, kd;
  float    gearRatio;
  int16_t  trimXUs, trimYUs;
};

static void settingsApply(const Settings &s) {
  pidX.kp = pidY.kp = s.kp;
  pidX.ki = pidY.ki = s.ki;
  pidX.kd = pidY.kd = s.kd;
  tvc.gearRatio = s.gearRatio;
  tvc.centerXUs = s.trimXUs;
  tvc.centerYUs = s.trimYUs;
}

static Settings settingsCurrent() {
  Settings s;
  s.magic     = SETTINGS_MAGIC;
  s.kp        = pidX.kp;
  s.ki        = pidX.ki;
  s.kd        = pidX.kd;
  s.gearRatio = tvc.gearRatio;
  s.trimXUs   = tvc.centerXUs;
  s.trimYUs   = tvc.centerYUs;
  return s;
}

static void settingsLoad() {
  Settings s;
  EEPROM.get(0, s);
  if (s.magic == SETTINGS_MAGIC && isfinite(s.kp) && isfinite(s.ki) &&
      isfinite(s.kd) && isfinite(s.gearRatio)) {
    settingsApply(s);
    Serial.println("Loaded saved settings from EEPROM.");
  }
}

static void settingsSave() {
  Settings s = settingsCurrent();
  EEPROM.put(0, s);
}

static void settingsShow() {
  Serial.print("kp=");     Serial.print(pidX.kp, 4);
  Serial.print(" ki=");    Serial.print(pidX.ki, 4);
  Serial.print(" kd=");    Serial.print(pidX.kd, 4);
  Serial.print(" gear=");  Serial.print(tvc.gearRatio, 3);
  Serial.print(" trimx="); Serial.print(tvc.centerXUs);
  Serial.print(" trimy="); Serial.println(tvc.centerYUs);
}

// ---------------------------------------------------------------------------
// Hardware watchdog (i.MX RT1062 WDOG1). Once enabled it cannot be disabled;
// if the firmware hangs, the Teensy resets and the pyro pins return to their
// power-on (safe, low) state.
// ---------------------------------------------------------------------------
static void watchdogBegin() {
#if WATCHDOG_TIMEOUT_S > 0
  // WT field: timeout = 0.5 s * WT + 0.5 s
  uint16_t wt = WATCHDOG_TIMEOUT_S * 2 - 1;
  WDOG1_WCR = WDOG_WCR_WT(wt) | WDOG_WCR_WDE | WDOG_WCR_WDBG |
              WDOG_WCR_SRS | WDOG_WCR_WDA;
#endif
}

static void watchdogFeed() {
#if WATCHDOG_TIMEOUT_S > 0
  WDOG1_WSR = 0x5555;
  WDOG1_WSR = 0xAAAA;
#endif
}

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static void setState(FlightState s) {
  state = s;
  stateEnteredMs = millis();
}

static void readImuBody() {
  float aChip[3], gChip[3];
  if (!imu.read(aChip, gChip, &imuTempC)) return;
  for (int i = 0; i < 3; i++) {
    accelBody[i]   = IMU_AXIS_SIGN[i] * aChip[IMU_AXIS_MAP[i]];
    gyroBodyDps[i] = IMU_AXIS_SIGN[i] * gChip[IMU_AXIS_MAP[i]] - gyroBias[i];
  }
}

static float readBatteryV() {
#if PIN_VBAT_ADC >= 0
  return analogRead(PIN_VBAT_ADC) * (3.3f / 4095.0f) * VBAT_DIVIDER;
#else
  return 0.0f;
#endif
}

static void firePyro(int ch) {           // ch = 1..4
  int idx = ch - 1;
  if (idx < 0 || idx > 3 || pyroFired[idx]) return;
  digitalWrite(PYRO_PINS[idx], HIGH);
  pyroFired[idx] = true;
  pyroOnSinceMs[idx] = millis();
  Serial.print("PYRO "); Serial.print(ch); Serial.println(" FIRED");
}

static void servicePyros() {              // turn channels off after PYRO_FIRE_MS
  for (int i = 0; i < 4; i++) {
    if (pyroFired[i] && pyroOnSinceMs[i] != 0 &&
        millis() - pyroOnSinceMs[i] >= PYRO_FIRE_MS) {
      digitalWrite(PYRO_PINS[i], LOW);
      pyroOnSinceMs[i] = 0;
    }
  }
}

// --- RGB status LED (common cathode: HIGH = on) ----------------------------
static void setLed(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_B, b ? HIGH : LOW);
}

static void serviceLed() {
  bool slow = (millis() >> 9) & 1;   // ~1 Hz
  bool fast = (millis() >> 7) & 1;   // ~4 Hz
  switch (state) {
    case STATE_IDLE:        setLed(false, slow, false);       break;
    case STATE_CALIBRATING: setLed(false, false, true);       break;
    case STATE_PAD_ARMED:
    case STATE_BOOST:
    case STATE_COAST:       setLed(true, false, false);       break;
    case STATE_ABORT:       setLed(fast, false, false);       break;
    case STATE_DESCENT:     setLed(false, true, true);        break;
    case STATE_LANDED:      setLed(false, fast, false);       break;
    case STATE_SENSOR_FAIL: setLed(fast, false, false);       break;
  }
}

// --- non-blocking buzzer patterns -----------------------------------------
struct Beeper {
  uint32_t nextToggle = 0;
  bool on = false;
  uint16_t onMs = 0, offMs = 0;
  void set(uint16_t on_, uint16_t off_) { onMs = on_; offMs = off_; }
  void service() {
    if (onMs == 0) { out(false); return; }
    uint32_t now = millis();
    if (now >= nextToggle) {
      on = !on;
      out(on);
      nextToggle = now + (on ? onMs : offMs);
    }
  }
  void out(bool en) {
#if BUZZER_IS_ACTIVE
    digitalWrite(PIN_BUZZER, en ? HIGH : LOW);
#else
    if (en) tone(PIN_BUZZER, BUZZER_TONE_HZ); else noTone(PIN_BUZZER);
#endif
  }
} beeper;

// ---------------------------------------------------------------------------
// Calibration (rocket must be VERTICAL and STILL on the pad)
// ---------------------------------------------------------------------------
static bool calibrate() {
  Serial.println("Calibrating: keep the rocket vertical and still...");

  // 1) Gyro bias: average raw rates, reject if the rocket is moving.
  double sum[3] = {0, 0, 0};
  float minV[3] = {1e9, 1e9, 1e9}, maxV[3] = {-1e9, -1e9, -1e9};
  uint32_t n = 0, t0 = millis();
  while (millis() - t0 < GYRO_CAL_MS) {
    float a[3], g[3];
    if (imu.read(a, g)) {
      for (int i = 0; i < 3; i++) {
        sum[i] += g[i];
        minV[i] = min(minV[i], g[i]);
        maxV[i] = max(maxV[i], g[i]);
      }
      n++;
    }
    watchdogFeed();
    delay(2);
  }
  if (n < 100) return false;
  for (int i = 0; i < 3; i++) {
    if (maxV[i] - minV[i] > 4.0f) {        // >4 dps spread => it was moving
      Serial.println("Calibration FAILED: rocket was moving. Try again.");
      return false;
    }
  }
  float biasChip[3];
  for (int i = 0; i < 3; i++) biasChip[i] = (float)(sum[i] / n);
  for (int i = 0; i < 3; i++)
    gyroBias[i] = IMU_AXIS_SIGN[i] * biasChip[IMU_AXIS_MAP[i]];

  // 2) Ground pressure reference (averaged).
  double pSum = 0; int pN = 0;
  t0 = millis();
  while (millis() - t0 < 500) {
    float p, tC;
    if (baro.read(p, tC)) { pSum += p; pN++; }
    watchdogFeed();
    delay(15);
  }
  if (pN > 0) groundPressurePa = (float)(pSum / pN);
  maxAltitudeM = 0;

  // 3) Converge the attitude quaternion onto the accel gravity vector.
  attitude.reset();
  attitude.setAidingGain(MAHONY_KP_INIT);
  t0 = millis();
  uint32_t lastUs = micros();
  while (millis() - t0 < ATT_CONVERGE_MS) {
    readImuBody();
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    float gRad[3] = { (float)(gyroBodyDps[0] * DEG_TO_RAD),
                      (float)(gyroBodyDps[1] * DEG_TO_RAD),
                      (float)(gyroBodyDps[2] * DEG_TO_RAD) };
    attitude.update(gRad, accelBody, dt);
    watchdogFeed();
    delay(2);
  }

  float tilt = attitude.tiltMagnitudeDeg();
  Serial.print("Calibration OK. Pad tilt = ");
  Serial.print(tilt, 1);
  Serial.println(" deg");
  if (tilt > 15.0f) {
    Serial.println("WARNING: rocket does not look vertical!");
  }

  pidX.reset();
  pidY.reset();
  return true;
}

// ---------------------------------------------------------------------------
// Serial command console
// ---------------------------------------------------------------------------
static void handleSerial() {
  static char buf[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[len] = 0;
      len = 0;
      String cmd(buf);
      cmd.trim();
      cmd.toLowerCase();
      if (cmd.length() == 0) continue;

      if (cmd == "arm" && (state == STATE_IDLE || state == STATE_LANDED)) {
        logger.erase();                     // new flight -> fresh log
        setState(STATE_CALIBRATING);
      } else if (cmd == "disarm") {
        tvc.center();
        setState(STATE_IDLE);
        Serial.println("Disarmed.");
      } else if (cmd == "status") {
        Serial.print("state=");   Serial.print((int)state);
        Serial.print(" tilt=");   Serial.print(attitude.tiltMagnitudeDeg(), 1);
        Serial.print(" alt=");    Serial.print(altitudeM, 1);
        Serial.print(" maxAlt="); Serial.print(maxAltitudeM, 1);
        Serial.print(" vbat=");   Serial.print(readBatteryV(), 2);
        Serial.print(" imuT=");   Serial.print(imuTempC, 1);
        Serial.print(" baroT=");  Serial.print(baroTempC, 1);
        Serial.print(" sd=");     Serial.print(logger.sdPresent() ? "OK" : "NONE");
        Serial.print(" flash=");  Serial.println(logger.flashPresent() ? "OK" : "NONE");
      } else if (cmd == "dump") {
        logger.dumpCsv(Serial);
      } else if (cmd == "erase") {
        logger.erase();
        Serial.println("Log erased.");
      } else if (cmd.startsWith("set ") && state == STATE_IDLE) {
        // "set kp 0.35" | "set ki .." | "set kd .." | "set gear 4.0"
        // "set trimx 1490" | "set trimy 1510"  — then "save" to persist.
        int sp = cmd.indexOf(' ', 4);
        if (sp > 0) {
          String name = cmd.substring(4, sp);
          float val = cmd.substring(sp + 1).toFloat();
          if      (name == "kp")    pidX.kp = pidY.kp = val;
          else if (name == "ki")    pidX.ki = pidY.ki = val;
          else if (name == "kd")    pidX.kd = pidY.kd = val;
          else if (name == "gear")  tvc.gearRatio = val;
          else if (name == "trimx") { tvc.centerXUs = (int)val; tvc.center(); }
          else if (name == "trimy") { tvc.centerYUs = (int)val; tvc.center(); }
          else { Serial.println("unknown: kp ki kd gear trimx trimy"); continue; }
          settingsShow();
        }
      } else if (cmd == "show") {
        settingsShow();
      } else if (cmd == "save" && state == STATE_IDLE) {
        settingsSave();
        Serial.println("Settings saved to EEPROM.");
      } else if (cmd == "servotest" && state == STATE_IDLE) {
        // Sweep each axis so you can verify direction and linkage ratio.
        Serial.println("Servo test: X sweep, then Y sweep.");
        for (int pass = 0; pass < 2; pass++) {
          for (float d = -GIMBAL_LIMIT_DEG; d <= GIMBAL_LIMIT_DEG; d += 0.5f) {
            tvc.write(pass == 0 ? d : 0, pass == 0 ? 0 : d, 0);
            watchdogFeed();
            delay(30);
          }
          tvc.center();
          delay(300);
        }
        Serial.println("Servo test done.");
#if ENABLE_PYRO_BENCH_TEST
      } else if (cmd.startsWith("fire") && cmd.length() == 5 &&
                 state == STATE_IDLE) {
        int ch = cmd.charAt(4) - '0';
        if (ch >= 1 && ch <= 4) {
          Serial.print("BENCH FIRE CH"); Serial.print(ch);
          Serial.println(" (dummy load only!)");
          pyroFired[ch - 1] = false;
          firePyro(ch);
        }
#endif
      } else {
        Serial.println("cmds: arm | disarm | status | servotest | dump | "
                       "erase | show | set <kp|ki|kd|gear|trimx|trimy> <v> | save");
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
static void logSample() {
  float pitch, yaw;
  attitude.getTilt(pitch, yaw);
  LogRecord r;
  r.tMs        = millis();
  r.state      = (uint8_t)state;
  r.pitchCdeg  = (int16_t)constrain(pitch * 100.0f, -32000.0f, 32000.0f);
  r.yawCdeg    = (int16_t)constrain(yaw * 100.0f, -32000.0f, 32000.0f);
  r.gyroX      = (int16_t)constrain(gyroBodyDps[0] * 10.0f, -32000.0f, 32000.0f);
  r.gyroY      = (int16_t)constrain(gyroBodyDps[1] * 10.0f, -32000.0f, 32000.0f);
  r.gyroZ      = (int16_t)constrain(gyroBodyDps[2] * 10.0f, -32000.0f, 32000.0f);
  r.accelX     = (int16_t)constrain(accelBody[0] * 100.0f, -32000.0f, 32000.0f);
  r.accelY     = (int16_t)constrain(accelBody[1] * 100.0f, -32000.0f, 32000.0f);
  r.accelZ     = (int16_t)constrain(accelBody[2] * 100.0f, -32000.0f, 32000.0f);
  r.servoXCdeg = (int16_t)(tvc.lastX * 100.0f);
  r.servoYCdeg = (int16_t)(tvc.lastY * 100.0f);
  r.altDm      = (int16_t)constrain(altitudeM * 10.0f, -32000.0f, 32000.0f);
  r.vbatMv10   = (int16_t)(readBatteryV() * 100.0f);
  r.flags      = (pyroFired[0] ? 1 : 0) | (pyroFired[1] ? 2 : 0) |
                 (pyroFired[2] ? 4 : 0) | (pyroFired[3] ? 8 : 0);
  logger.record(r);
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
  for (int i = 0; i < 4; i++) {          // pyros SAFE, first thing
    pinMode(PYRO_PINS[i], OUTPUT);
    digitalWrite(PYRO_PINS[i], LOW);
  }
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  setLed(false, false, false);
#if PIN_VBAT_ADC >= 0
  analogReadResolution(12);
#endif

  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // don't block on battery power

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  tvc.begin();          // servos centered immediately
  settingsLoad();       // EEPROM-saved gains/trims override config.h defaults
  bool logOk = logger.begin();

  bool imuOk  = imu.begin();
  bool baroOk = baro.begin();
  Serial.print("MPU6050:  "); Serial.println(imuOk  ? "OK" : "FAIL");
  Serial.print("BMP280:   "); Serial.println(baroOk ? "OK" : "FAIL");
  Serial.print("SD card:  "); Serial.println(logger.sdPresent() ? "OK" : "NONE");
  Serial.print("W25Q128:  "); Serial.println(logger.flashPresent() ? "OK" : "NONE");
  if (!logOk) Serial.println("WARNING: no working log storage!");

  watchdogBegin();

  if (!imuOk || !baroOk) {
    setState(STATE_SENSOR_FAIL);
    beeper.set(100, 100);              // rapid beep = sensor failure
  } else {
    setState(STATE_IDLE);
    beeper.set(80, 3000);              // slow chirp = idle, healthy
    Serial.println("Ready. Type 'arm' to begin the launch sequence.");
  }

  lastLoopUs = nextControlUs = nextBaroUs = nextLogUs = micros();
}

void loop() {
  watchdogFeed();
  handleSerial();
  servicePyros();
  beeper.service();
  serviceLed();

  uint32_t nowUs = micros();

  // ----- calibration is a blocking one-shot, run outside the schedulers ----
  if (state == STATE_CALIBRATING) {
    if (calibrate()) {
      attitude.setAidingGain(MAHONY_KP_PAD);
      highAccelSinceMs = 0;
      launchMs = 0;
      for (int i = 0; i < 4; i++) pyroFired[i] = false;
      setState(STATE_PAD_ARMED);
      beeper.set(500, 500);            // steady beep = ARMED, stand clear
      Serial.println("ARMED. Waiting for launch.");
    } else {
      setState(STATE_IDLE);
      beeper.set(80, 3000);
    }
    return;
  }

  // ------------------------- 250 Hz control loop ---------------------------
  if ((int32_t)(nowUs - nextControlUs) >= 0) {
    nextControlUs += 1000000UL / CONTROL_RATE_HZ;
    float dt = (nowUs - lastLoopUs) * 1e-6f;
    lastLoopUs = nowUs;
    if (dt <= 0 || dt > 0.05f) dt = 1.0f / CONTROL_RATE_HZ;

    readImuBody();
    float gRad[3] = { (float)(gyroBodyDps[0] * DEG_TO_RAD),
                      (float)(gyroBodyDps[1] * DEG_TO_RAD),
                      (float)(gyroBodyDps[2] * DEG_TO_RAD) };
    attitude.update(gRad, accelBody, dt);

    switch (state) {
      case STATE_PAD_ARMED: {
        // Launch detection: sustained axial acceleration.
        if (accelBody[2] > LAUNCH_ACCEL_G) {
          if (highAccelSinceMs == 0) highAccelSinceMs = millis();
          if (millis() - highAccelSinceMs >= LAUNCH_DETECT_MS) {
            launchMs = millis();
            attitude.setAidingGain(MAHONY_KP_FLIGHT);  // gyro-only now
            pidX.reset();
            pidY.reset();
            setState(STATE_BOOST);
            beeper.set(0, 0);          // silence — every cycle counts
            Serial.println("LIFTOFF");
          }
        } else {
          highAccelSinceMs = 0;
        }
        break;
      }

      case STATE_BOOST: {
        // ---- THE ACTUAL TVC ----
        // Hold center while still on the launch rail/tube.
        bool tvcEnabled = millis() - launchMs >= TVC_ENABLE_DELAY_MS;
        float pitch, yaw;
        attitude.getTilt(pitch, yaw);              // deg away from vertical
        // setpoint = 0 (straight up); error = 0 - measured
        float cmdX = pidX.update(-pitch, gyroBodyDps[0], dt);
        float cmdY = pidY.update(-yaw,   gyroBodyDps[1], dt);
        tvc.write(tvcEnabled ? cmdX : 0.0f, tvcEnabled ? cmdY : 0.0f, dt);

        if (attitude.tiltMagnitudeDeg() > ABORT_TILT_DEG) {
          tvc.center();
          setState(STATE_ABORT);
          Serial.println("ABORT: tilt limit");
          break;
        }
        uint32_t burnMs = millis() - launchMs;
        bool accelBurnout = burnMs > MIN_BURN_MS &&
                            accelBody[2] < BURNOUT_ACCEL_G;
        if (accelBurnout || burnMs > MAX_BURN_MS) {
          tvc.center();                // no thrust => no control authority
          setState(STATE_COAST);
          Serial.println("BURNOUT -> coast");
        }
        break;
      }

      case STATE_COAST:
      case STATE_ABORT:
        // Apogee handled in the baro loop; failsafe here:
        if (launchMs && millis() - launchMs > FAILSAFE_CHUTE_MS &&
            !pyroFired[PYRO_CHUTE_CHANNEL - 1]) {
          firePyro(PYRO_CHUTE_CHANNEL);
          setState(STATE_DESCENT);
          Serial.println("FAILSAFE chute deploy");
        }
        break;

      case STATE_DESCENT: {
        // Landing detection: altitude stable for LANDED_TIME_MS.
        if (fabsf(altitudeM - landedRefAltM) > LANDED_WINDOW_M) {
          landedRefAltM = altitudeM;
          landedStableSinceMs = millis();
        } else if (millis() - landedStableSinceMs > LANDED_TIME_MS) {
          setState(STATE_LANDED);
          logger.commit();
          beeper.set(1500, 1500);      // loud locator beep
          Serial.println("LANDED. Log saved — SD card or type 'dump'.");
        }
        break;
      }

      default:
        break;
    }
  }

  // --------------------------- 50 Hz baro loop -----------------------------
  if ((int32_t)(nowUs - nextBaroUs) >= 0) {
    nextBaroUs += 1000000UL / BARO_RATE_HZ;
    float p;
    if (baro.read(p, baroTempC)) {
      float newAlt = BMP280::altitudeM(p, groundPressurePa);
      // Apogee detection (only meaningful in coast/abort)
      if (state == STATE_COAST || state == STATE_ABORT) {
        if (newAlt > maxAltitudeM) {
          maxAltitudeM = newAlt;
          descendingSamples = 0;
        } else if (newAlt < maxAltitudeM - 0.5f) {
          descendingSamples++;
        }
        bool lockoutDone = millis() - launchMs > APOGEE_MIN_FLIGHT_MS;
        if (lockoutDone && descendingSamples >= APOGEE_DESCENT_SAMPLES) {
          firePyro(PYRO_CHUTE_CHANNEL);
          landedRefAltM = newAlt;
          landedStableSinceMs = millis();
          setState(STATE_DESCENT);
          Serial.print("APOGEE @ ");
          Serial.print(maxAltitudeM, 1);
          Serial.println(" m -> chute");
        }
      } else if (state == STATE_BOOST && newAlt > maxAltitudeM) {
        maxAltitudeM = newAlt;
      }
      altitudeM = newAlt;
    }
  }

  // ----------------------------- logging -----------------------------------
  bool inFlight = state == STATE_PAD_ARMED || state == STATE_BOOST ||
                  state == STATE_COAST || state == STATE_DESCENT ||
                  state == STATE_ABORT;
  uint32_t logPeriodUs =
      1000000UL / (inFlight ? LOG_RATE_FLIGHT_HZ : LOG_RATE_IDLE_HZ);
  if ((int32_t)(nowUs - nextLogUs) >= 0) {
    nextLogUs = nowUs + logPeriodUs;
    if (state != STATE_LANDED) logSample();
  }
}
