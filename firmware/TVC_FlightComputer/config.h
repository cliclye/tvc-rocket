#pragma once
// ============================================================================
//  config.h — EVERY user-tunable setting for the TVC flight computer.
//
//  TARGET HARDWARE (custom PCB):
//    * Teensy 4.0 (IMXRT1062, 600 MHz Cortex-M7, 3.3 V logic — NOT 5V tolerant)
//    * GY-521  MPU6050  6-axis IMU        (I2C)
//    * GY-BMP280 barometer                (I2C)
//    * SPI MicroSD breakout               (primary flight log)
//    * Winbond W25Q128JVSIQ 16 MB flash   (backup flight log, LittleFS)
//    * 4x N-MOSFET pyro channels on 2-pin screw terminals
//    * 12 mm 5 V active buzzer, 5 mm common-cathode RGB LED
//    * 7805 linear regulator: battery -> 5 V (servos, buzzer, Teensy VIN)
//
//  !!! YOU MUST EDIT THE PIN ASSIGNMENTS BELOW TO MATCH YOUR PCB !!!
//  The defaults below are a sensible Teensy 4.0 layout (all on the outer
//  0-23 pins). Open your schematic and set each pin to the net it is
//  actually routed to.
// ============================================================================

// ---------------------------------------------------------------------------
// PIN MAP  (Teensy 4.0 pin numbers)
// ---------------------------------------------------------------------------
// I2C bus (Wire): GY-521 + GY-BMP280
#define PIN_I2C_SDA       18     // Teensy 4.0 Wire SDA0 (fixed)
#define PIN_I2C_SCL       19     // Teensy 4.0 Wire SCL0 (fixed)
#define I2C_CLOCK_HZ      400000

// SPI bus (fixed on Teensy 4.0): MOSI=11, MISO=12, SCK=13
#define PIN_SD_CS         10     // MicroSD breakout chip-select
#define PIN_FLASH_CS       9     // W25Q128JVSIQ chip-select
// NOTE: the Teensy onboard LED shares pin 13 (SCK) — it is NOT usable as a
// status LED here. The RGB LED below is the status indicator.

#define PIN_SERVO_X        2     // servo that gimbals the mount about body X
#define PIN_SERVO_Y        3     // servo that gimbals the mount about body Y

// Four MOSFET pyro channels (screw terminals). Gate drive pins:
#define PIN_PYRO_1         4     // parachute / main event (default chute ch)
#define PIN_PYRO_2         5     // 2nd stage / backup
#define PIN_PYRO_3         6     // spare
#define PIN_PYRO_4         7     // spare

#define PIN_BUZZER         8
#define BUZZER_IS_ACTIVE   1     // 12mm 5V ACTIVE buzzer: just drive HIGH
#define BUZZER_TONE_HZ     2700  // only used if BUZZER_IS_ACTIVE == 0

// 5 mm common-cathode RGB LED (HIGH = on). Use series resistors!
#define PIN_LED_R         14
#define PIN_LED_G         15
#define PIN_LED_B         16

#define PIN_VBAT_ADC      17     // battery voltage divider -> A3, -1 = none
#define VBAT_DIVIDER      4.0f   // (Rtop+Rbot)/Rbot of your divider
                                 // e.g. 30k/10k -> 4.0. Keep divider output
                                 // BELOW 3.3 V at max battery voltage!

// I2C addresses
#define MPU6050_ADDR      0x68   // GY-521: 0x68 (AD0 low, default) or 0x69
#define BMP280_ADDR       0x76   // GY-BMP280: usually 0x76 (SDO low), or 0x77

// ---------------------------------------------------------------------------
// IMU MOUNTING ORIENTATION
// ---------------------------------------------------------------------------
// Body frame convention used by all flight code:
//   +Z body = out the NOSE (points at the sky on the pad)
//   +X, +Y  = lateral axes, right-hand rule.
// Map the MPU6050's chip axes into the body frame:
//   body[i] = IMU_AXIS_SIGN[i] * imu[IMU_AXIS_MAP[i]]
// Default: GY-521 mounted flat with chip +Z pointing out the nose.
static const int   IMU_AXIS_MAP [3] = { 0, 1, 2 };   // which imu axis feeds body X,Y,Z
static const float IMU_AXIS_SIGN[3] = { +1.0f, +1.0f, +1.0f };

// ---------------------------------------------------------------------------
// CONTROL LOOP
// ---------------------------------------------------------------------------
#define CONTROL_RATE_HZ     250     // attitude + PID + servo update rate
#define BARO_RATE_HZ         50     // BMP280 poll rate (sensor converts ~72 Hz)

// PID gains operate on tilt error in DEGREES and output gimbal DEGREES.
// START CONSERVATIVE. Tune with tools/tvc_tuner (see README) — it can also
// write these values here and re-flash the board for you.
// NOTE: gains/trims can be changed at runtime over serial ("set kp 0.4",
// then "save" to persist in EEPROM). Saved settings override these defaults.
#define PID_KP             0.30f
#define PID_KI             0.10f
#define PID_KD             0.11f
#define PID_I_LIMIT_DEG    3.0f    // integral contribution clamp (gimbal deg)

// First-order low-pass filter on the gyro rate feeding the D term.
// Kills motor-vibration noise before it reaches the servos. 0 = off.
#define DTERM_LPF_HZ       30.0f

// Max slew rate of the NOZZLE (deg/s). Protects the servos and avoids
// exciting airframe bending modes. 0 = off.
#define NOZZLE_RATE_LIMIT_DPS  300.0f

// Keep the nozzle centered for this long after liftoff (launch rail /
// launch tube exit — steering while guided just slams the linkage).
#define TVC_ENABLE_DELAY_MS    150

// Mechanical limits / linkage
#define GIMBAL_LIMIT_DEG   5.0f    // max nozzle deflection from center
#define SERVO_GEAR_RATIO   4.0f    // servo degrees per 1 degree of nozzle
                                   // motion (linkage reduction). MEASURE THIS!
#define SERVO_X_CENTER_US  1500    // trim so the nozzle is perfectly centered
#define SERVO_Y_CENTER_US  1500
#define SERVO_US_PER_DEG   10.0f   // ~10 us/deg for a 1000-2000us/100deg servo
#define SERVO_X_DIR        +1      // flip to -1 if bench test shows reversed
#define SERVO_Y_DIR        +1
#define SERVO_MIN_US       1000    // hard clamp, never command beyond these
#define SERVO_MAX_US       2000

// ---------------------------------------------------------------------------
// FLIGHT EVENTS
// ---------------------------------------------------------------------------
#define LAUNCH_ACCEL_G        1.8f   // |accel| threshold to declare liftoff
#define LAUNCH_DETECT_MS      60     // must persist this long
#define MIN_BURN_MS           300    // ignore "burnout" earlier than this
#define BURNOUT_ACCEL_G       0.30f  // axial accel below this => burnout
#define MAX_BURN_MS           3500   // force burnout after this (motor spec!)
#define ABORT_TILT_DEG        45.0f  // kill TVC if we tip past this in boost
#define APOGEE_DESCENT_SAMPLES 5     // consecutive descending baro samples
#define APOGEE_MIN_FLIGHT_MS  1500   // apogee lockout after liftoff
#define FAILSAFE_CHUTE_MS     15000  // ALWAYS fire chute this long after liftoff
#define PYRO_FIRE_MS          1000   // how long a pyro channel stays on
#define LANDED_WINDOW_M       1.0f   // altitude must stay within this band...
#define LANDED_TIME_MS        5000   // ...for this long to declare landing

#define PYRO_CHUTE_CHANNEL    1      // which channel (1-4) deploys the chute

// ---------------------------------------------------------------------------
// CALIBRATION
// ---------------------------------------------------------------------------
#define GYRO_CAL_MS          2000    // stationary gyro-bias averaging time
#define ATT_CONVERGE_MS      1500    // accel-aided fast converge time on arm
#define MAHONY_KP_FLIGHT     0.0f    // accel aiding gain in flight (OFF —
                                     // accel = thrust, not gravity!)
#define MAHONY_KP_PAD        2.0f    // accel aiding gain while sitting on pad
#define MAHONY_KP_INIT       20.0f   // fast-converge gain during arming

// ---------------------------------------------------------------------------
// LOGGING
// ---------------------------------------------------------------------------
// Records buffer in RAM during flight, then commit to the SD card (primary)
// or the W25Q128 LittleFS partition (fallback) after landing.
#define LOG_RATE_FLIGHT_HZ   100     // record rate from ARMED until landing
#define LOG_RATE_IDLE_HZ     2       // record rate while idle/disarmed
#define LOG_CAPACITY         12000   // records held in RAM (30 B each).
                                     // 12000 = ~360 KB = 120 s @ 100 Hz
                                     // (fits easily in the Teensy's RAM2).
#define LOG_FILENAME         "flight.bin"

// ---------------------------------------------------------------------------
// SAFETY / BENCH
// ---------------------------------------------------------------------------
// Leave 0 for flight builds. When 1, the serial commands "fire1".."fire4"
// are compiled in so you can bench-test pyro channels on a DUMMY LOAD.
#define ENABLE_PYRO_BENCH_TEST  0

#define WATCHDOG_TIMEOUT_S   8       // hardware watchdog (WDOG1), 0.5s steps
