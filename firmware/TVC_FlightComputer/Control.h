#pragma once
// ============================================================================
//  Control.h — PID controllers and the gimbal->servo mapping.
//
//  Two independent PID loops (one per axis), the same architecture used by
//  tomkuttler/TVC-Rocket-Flight-Code and AdamMarciniak/CygnusX1:
//
//    tilt error (deg) -> PID -> nozzle deflection (deg)
//                     -> x linkage ratio -> servo microseconds
//
//  Control refinements over the basic loop:
//   * D term is computed on the MEASUREMENT (the gyro rate), not the error,
//     which avoids derivative kick and uses the cleanest signal we have.
//   * The gyro rate is low-pass filtered (DTERM_LPF_HZ) before the D term
//     so motor vibration doesn't buzz the servos.
//   * Nozzle motion is slew-rate limited (NOZZLE_RATE_LIMIT_DPS).
//   * Gains, servo trims, and the linkage ratio are runtime-tunable
//     (serial "set" command / tuner UI) and persisted in EEPROM.
// ============================================================================

#include <Arduino.h>
#include <Servo.h>
#include "config.h"

class AxisPID {
public:
  // Public + mutable: adjusted live via the serial console / tuner UI.
  float kp, ki, kd;

  AxisPID(float kp_, float ki_, float kd_, float iLim_)
      : kp(kp_), ki(ki_), kd(kd_), iLim(iLim_) {}

  void reset() {
    integral = 0.0f;
    rateFilt = 0.0f;
  }

  // errDeg:  desired - actual tilt (deg). Setpoint is 0 (straight up).
  // rateDps: measured body angular rate about this axis (deg/s).
  // Returns commanded nozzle deflection in degrees.
  float update(float errDeg, float rateDps, float dt) {
    if (DTERM_LPF_HZ > 0.0f) {
      float alpha = dt / (dt + 1.0f / (2.0f * (float)PI * DTERM_LPF_HZ));
      rateFilt += alpha * (rateDps - rateFilt);
    } else {
      rateFilt = rateDps;
    }
    integral += ki * errDeg * dt;
    integral = constrain(integral, -iLim, iLim);
    // Derivative-on-measurement: d(error)/dt = -d(angle)/dt = -rate
    float out = kp * errDeg + integral - kd * rateFilt;
    return constrain(out, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG);
  }

private:
  float iLim;
  float integral = 0.0f;
  float rateFilt = 0.0f;
};

class TVCMount {
public:
  // Runtime-tunable (serial "set" command / tuner UI, EEPROM-persisted):
  int   centerXUs = SERVO_X_CENTER_US;
  int   centerYUs = SERVO_Y_CENTER_US;
  float gearRatio = SERVO_GEAR_RATIO;

  void begin() {
    servoX.attach(PIN_SERVO_X, SERVO_MIN_US, SERVO_MAX_US);
    servoY.attach(PIN_SERVO_Y, SERVO_MIN_US, SERVO_MAX_US);
    center();
  }

  void center() {
    lastX = lastY = 0.0f;
    apply();
  }

  // Nozzle deflections in degrees -> servo pulse widths.
  // dt (seconds) drives the slew-rate limit; pass 0 to move immediately.
  void write(float nozzleXDeg, float nozzleYDeg, float dt) {
    float tx = constrain(nozzleXDeg, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG);
    float ty = constrain(nozzleYDeg, -GIMBAL_LIMIT_DEG, GIMBAL_LIMIT_DEG);
    if (NOZZLE_RATE_LIMIT_DPS > 0.0f && dt > 0.0f) {
      float maxStep = NOZZLE_RATE_LIMIT_DPS * dt;
      lastX += constrain(tx - lastX, -maxStep, maxStep);
      lastY += constrain(ty - lastY, -maxStep, maxStep);
    } else {
      lastX = tx;
      lastY = ty;
    }
    apply();
  }

  float lastX = 0.0f, lastY = 0.0f;   // current nozzle angles (deg)

private:
  Servo servoX, servoY;

  void apply() {
    int usX = centerXUs +
              (int)(SERVO_X_DIR * lastX * gearRatio * SERVO_US_PER_DEG);
    int usY = centerYUs +
              (int)(SERVO_Y_DIR * lastY * gearRatio * SERVO_US_PER_DEG);
    servoX.writeMicroseconds(constrain(usX, SERVO_MIN_US, SERVO_MAX_US));
    servoY.writeMicroseconds(constrain(usY, SERVO_MIN_US, SERVO_MAX_US));
  }
};
