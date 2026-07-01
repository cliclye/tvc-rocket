# Teensy 4.0 TVC Flight Computer Firmware

Thrust-vector-control flight software for a custom Teensy 4.0 rocket
avionics board:

- **Teensy 4.0** (IMXRT1062, 600 MHz Cortex-M7 — 3.3 V logic, *not* 5 V tolerant)
- **GY-521 (MPU6050)** 6-axis IMU, I2C
- **GY-BMP280** barometer, I2C
- **SPI MicroSD breakout** — primary flight log (CSV, straight off the card)
- **Winbond W25Q128JVSIQ** 16 MB SPI flash — backup flight log (LittleFS)
- **4x MOSFET pyro channels** on 2-pin 3.5 mm screw terminals
- **12 mm 5 V active buzzer** and **5 mm common-cathode RGB LED**
- **7805 regulator** — battery → 5 V (servos, buzzer, Teensy VIN)

The architecture follows proven open-source TVC computers:

- [tomkuttler/TVC-Rocket-Flight-Code](https://github.com/tomkuttler/TVC-Rocket-Flight-Code) — state machine + dual PID loops (also a Teensy)
- [AdamMarciniak/CygnusX1](https://github.com/AdamMarciniak/CygnusX1) — integrated TVC flight computer (servos, pyros, arming)
- PID tuning simulators: [fszewczyk/tvc-simulator](https://github.com/fszewczyk/tvc-simulator), [GuidodiPasquo/AeroVECTOR](https://github.com/GuidodiPasquo/AeroVECTOR)

## How it works

```
250 Hz   MPU6050 gyro+accel ──> Mahony quaternion filter ──> pitch/yaw tilt
                                                     │
                                       2x PID (one per axis)
                                                     │
                                   gimbal deg -> linkage ratio -> servo µs
 50 Hz   BMP280 ──> altitude AGL ──> apogee detection ──> pyro 1 (chute)
100 Hz   black-box log (RAM ring buffer -> SD card + SPI flash on landing)
```

**State machine:**

| State | What happens |
|---|---|
| `IDLE` | Servos centered, pyros locked. Waiting for `arm` command. |
| `CALIBRATING` | 2 s gyro-bias average + ground pressure + attitude converge. Rocket must be vertical and still. |
| `PAD_ARMED` | Steady beeping. Watching for sustained axial accel > 1.8 g. |
| `BOOST` | **TVC active.** PID drives both servos to keep the nose vertical. Tilt > 45° → `ABORT`. Burnout detected by axial accel dropping (or motor-time limit). |
| `COAST` | Servos centered (no thrust = no TVC authority). Baro watches for apogee. |
| `DESCENT` | Chute pyro fired at apogee (plus a hard 15 s failsafe timer). |
| `LANDED` | Log committed to SD + flash, loud locator beep. |

**RGB LED status:** green slow blink = idle · blue solid = calibrating ·
red solid = armed/boost/coast · cyan = descent · green fast blink = landed ·
red fast blink = sensor failure or abort.

Key design choice: the accelerometer aids the attitude filter **only on the
pad** (it measures pure gravity there). Under thrust it measures
thrust-plus-gravity, so in flight the filter integrates the freshly
bias-calibrated gyros only — drift over a few-second burn is negligible.

A hardware watchdog (WDOG1) resets the Teensy if the firmware ever hangs;
pyro pins return to their power-on safe (low) state on reset.

## Repository layout

```
firmware/TVC_FlightComputer/
  TVC_FlightComputer.ino   main sketch: state machine, scheduling, events
  config.h                 ALL pins, gains, thresholds  <-- EDIT THIS FIRST
  MPU6050.h                minimal register-level IMU driver (I2C)
  BMP280.h                 baro driver with Bosch datasheet compensation
  Attitude.h               quaternion Mahony complementary filter
  Control.h                per-axis PID + gimbal->servo mapping
  Logger.h                 RAM ring buffer -> SD CSV + W25Q128 binary backup
tools/
  tvc_tuner/               web UI: simulate YOUR rocket, tune PID, upload
  plot_log.py              capture + plot flight logs (pandas/matplotlib)
```

## TVC Tuner (tuning UI)

`tools/tvc_tuner` is a local web app that simulates *your* rocket and tunes
the TVC to it:

```bash
pip install -r tools/tvc_tuner/requirements.txt
python tools/tvc_tuner/app.py        # then open http://127.0.0.1:5055
```

What it does:

- **Rocket physics input, down to the detail** — either totals (mass, CM,
  moment of inertia) or a full component table (each part's mass, position,
  length) from which mass, CM, and inertia are computed. Plus body
  diameter, motor thrust/burn/propellant (Estes presets included), TVC
  geometry (gimbal limit, linkage ratio), and servo dynamics (speed,
  latency).
- **Flight simulation** using an *exact copy of the firmware's PID loop*
  (same rates, same D-term filter, same slew limit) against a planar
  3-DOF model: thrust torque, thrust misalignment, wind, and the
  destabilizing aerodynamic moment of a finless rocket (CP vs CM).
  Charts show tilt, nozzle angle, altitude, and lateral drift; metric
  cards flag saturation, slow settling, and low thrust/weight.
- **"Suggest gains"** — pole-placement starting gains from your rocket's
  thrust, lever arm, and inertia, including extra stiffness to beat the
  aero instability at burnout speed. It warns if the design is physically
  uncontrollable (aero moment beats full gimbal authority).
- **"Write gains → config.h"** — writes Kp/Ki/Kd and the TVC parameters
  straight into the firmware config.
- **"Compile & upload to rocket"** — runs `arduino-cli` and flashes the
  connected Teensy, streaming build output into the page console.
- **"Send gains live over serial"** — updates a running board's gains via
  the `set`/`save` serial commands without reflashing (board must be
  IDLE, i.e. disarmed).

## Build & flash

1. Install the Arduino IDE and the **Teensy board package by PJRC**
   (Boards Manager URL: `https://www.pjrc.com/teensy/package_teensy_index.json`),
   or use `arduino-cli`. All libraries used (`Servo`, `SD`, `LittleFS`) ship
   with the Teensy core — nothing else to install.
2. **Edit `config.h`** — see the checklist below.
3. Board: *Teensy 4.0*, then upload (the Teensy Loader pops up automatically;
   press the button on the Teensy the first time).

With `arduino-cli`:

```bash
arduino-cli core install teensy:avr \
  --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
arduino-cli compile --fqbn teensy:avr:teensy40 firmware/TVC_FlightComputer
arduino-cli upload  --fqbn teensy:avr:teensy40 firmware/TVC_FlightComputer
```

## config.h checklist (do these before first power-up)

1. **Pins** — set every `PIN_*` to what your PCB actually routes. Defaults:
   servos 2/3, pyros 4-7, buzzer 8, flash CS 9, SD CS 10, RGB 14/15/16,
   battery ADC 17, I2C on 18/19 (Wire), SPI on 11/12/13 (fixed).
   Note: pin 13 is SCK, so the Teensy's onboard LED is not usable —
   the RGB LED is the status indicator.
2. **I2C addresses** — GY-521 is `0x68` (AD0 low) or `0x69`; GY-BMP280 is
   usually `0x76`, some modules are `0x77`.
3. **IMU orientation** — `IMU_AXIS_MAP` / `IMU_AXIS_SIGN` so body +Z points
   out the nose. Verify: with the rocket vertical, `status` should show a
   small tilt; tipping the rocket should grow it.
4. **`SERVO_GEAR_RATIO`** — servo degrees per degree of *nozzle* motion.
   Measure it on your mount: command a sweep (`servotest`), measure actual
   nozzle deflection, adjust until commanded = actual.
5. **`SERVO_X/Y_CENTER_US`** — trim until the nozzle is perfectly straight.
6. **`SERVO_X/Y_DIR`** — see "direction check" below. Getting a sign wrong
   turns the stabilizer into a destabilizer.
7. **Motor timing** — set `MAX_BURN_MS` to your motor's burn time + margin.

## Hardware notes

- **Teensy 4.0 is NOT 5 V tolerant.** Power the GY-521 and GY-BMP280 from
  the Teensy's 3.3 V pin so the I2C lines never see 5 V. Feed the 7805's
  5 V only to VIN, the servos, and the buzzer.
- A **7805 supplies ~1-1.5 A and needs ≥ 7.5 V in** (2S LiPo / 9 V works).
  Two TVC servos moving hard can transiently exceed that — give the 7805 a
  heatsink and generous output capacitance, and bench-test under real servo
  load. Cut USB power (or use the VUSB/VIN separation) when running on
  battery.
- SD card and W25Q128 share the SPI bus with separate chip-selects; the
  firmware tolerates either or both being absent (it warns at boot and in
  `status`).

## Serial console (115200 baud)

| Command | Action |
|---|---|
| `arm` | Erase flash backup log, calibrate, go to `PAD_ARMED` |
| `disarm` | Back to `IDLE`, servos centered |
| `status` | Tilt, altitude, battery, temperatures, SD/flash health |
| `servotest` | Slow sweep of each axis (IDLE only) |
| `dump` | Stream the flash backup log as CSV |
| `erase` | Delete the flash backup log |
| `show` | Print current gains/trims |
| `set <p> <v>` | Live-tune `kp ki kd gear trimx trimy` (IDLE only) |
| `save` | Persist current gains/trims to EEPROM (they survive reboot) |

## Ground testing (in order — do not skip)

1. **Sensors:** `status` while tilting the board; pitch/yaw should track.
2. **Servo direction check (critical):** arm the rocket (no motor, no
   igniters), hold it, and tilt the nose in +X. The nozzle must deflect so
   that thrust would push the nose *back toward vertical*. If an axis moves
   the wrong way, flip `SERVO_X_DIR`/`SERVO_Y_DIR`.
3. **Pyro channels:** with `ENABLE_PYRO_BENCH_TEST 1`, fire each channel
   (`fire1`..`fire4`) into a **dummy resistive load** (never an igniter)
   and scope the pulse. Rebuild with it set to `0` for flight.
4. **Simulate PID gains** before flying with the built-in tuner
   (`python tools/tvc_tuner/app.py`): enter your rocket's real mass
   properties, run "Suggest gains", stress-test with disturbances (initial
   tilt, misalignment, wind), then write the gains to `config.h` and
   upload — all from the UI. Cross-check with AeroVECTOR or tvc-simulator
   if you want a second opinion.
5. **Full dress rehearsal:** arm on the pad rig, shake-test, verify no
   pyro fires, verify the log records (`status` shows SD/flash OK).

## Safety — read this

- **The pyro battery must run through a physical arming switch** in its
  positive lead. Software arming is *not* a safety device; MOSFET gates are
  held low from the first line of `setup()` and the watchdog resets to a
  safe state, but a physical break is the only real safe.
- Never connect live igniters while USB is attached or people are near.
- TVC rockets are *unstable by design* (no fins doing the work). A software
  bug or wrong servo sign means a rocket that flies sideways. Test
  exhaustively; follow NAR/NFPA 1122 safety code; fly with a safety officer.
- The 45° abort centers the nozzle rather than fighting a lost cause, and
  the 15 s failsafe timer deploys the chute even if apogee detection fails.

## After the flight

Pop the MicroSD card into your computer — each flight is saved as
`flight000.csv`, `flight001.csv`, ... — and plot it:

```bash
python tools/plot_log.py /Volumes/SDCARD/flight000.csv
```

No SD card? Pull the backup from the W25Q128 over USB:

```bash
python tools/plot_log.py --port /dev/cu.usbmodem*   # sends 'dump', captures, plots
```

The plot shows tilt vs. TVC commands vs. acceleration vs. altitude with
BOOST/DESCENT shaded — exactly what you need for post-flight PID tuning.
