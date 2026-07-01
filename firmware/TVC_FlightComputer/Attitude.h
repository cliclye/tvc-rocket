#pragma once
// ============================================================================
//  Attitude.h — quaternion attitude estimator (Mahony complementary filter).
//
//  Why this design (same approach as most open-source TVC computers, e.g.
//  tomkuttler/TVC-Rocket-Flight-Code and BPS.space-style boards):
//
//   * On the pad, the accelerometer measures pure gravity, so we use it to
//     find "which way is up" and to converge the quaternion (high Kp).
//   * Under thrust the accelerometer measures thrust + gravity and can no
//     longer tell us where "down" is, so accel aiding is turned OFF and we
//     integrate the (bias-calibrated) gyros only. Over a 2-4 s burn the
//     drift of a freshly-calibrated MEMS gyro is a fraction of a degree —
//     completely fine for TVC.
//
//  Frames: body +Z = out the nose. On the pad, +Z points up, so gravity
//  measured by the accel is ~(0, 0, +1) g.
//
//  Outputs: pitchDeg / yawDeg — the tilt of the nose away from vertical,
//  decomposed about the body X and Y axes. These are exactly the two angles
//  the two TVC servos must null.
// ============================================================================

#include <Arduino.h>
#include <math.h>

class Attitude {
public:
  void reset() {
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    ix = iy = iz = 0.0f;
  }

  // Accel-aiding gain. High while converging on the pad, ZERO in flight.
  void setAidingGain(float kp) { kpAid = kp; }

  // gyro in rad/s (bias already removed), accel in g, dt in seconds.
  void update(const float gyro[3], const float accel[3], float dt) {
    float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    if (kpAid > 0.0f) {
      float norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                         accel[2] * accel[2]);
      // Only trust the accel when |a| is close to 1 g (i.e. not thrusting
      // or vibrating hard) — a standard sanity gate.
      if (norm > 0.7f && norm < 1.3f) {
        float ax = accel[0] / norm, ay = accel[1] / norm, az = accel[2] / norm;

        // Estimated gravity direction in body frame from the quaternion.
        // (third row of the rotation matrix, since gravity = world +Z here)
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Error = cross(measured, estimated)
        float ex = ay * vz - az * vy;
        float ey = az * vx - ax * vz;
        float ez = ax * vy - ay * vx;

        gx += kpAid * ex;
        gy += kpAid * ey;
        gz += kpAid * ez;
      }
    }

    // Quaternion integration: q_dot = 0.5 * q ⊗ (0, gx, gy, gz)
    float halfDt = 0.5f * dt;
    float nq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfDt;
    float nq1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * halfDt;
    float nq2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * halfDt;
    float nq3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * halfDt;

    float invN = 1.0f / sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);
    q0 = nq0 * invN; q1 = nq1 * invN; q2 = nq2 * invN; q3 = nq3 * invN;
  }

  // Tilt of the body +Z (nose) axis away from world-vertical, decomposed
  // into rotations about body X and Y. Returns degrees.
  //   pitchDeg: tilt component the X servo must correct (rotation about X)
  //   yawDeg:   tilt component the Y servo must correct (rotation about Y)
  void getTilt(float &pitchDeg, float &yawDeg) const {
    // World up-vector expressed in body frame:
    float ux = 2.0f * (q1 * q3 - q0 * q2);
    float uy = 2.0f * (q0 * q1 + q2 * q3);
    float uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    // Small-angle-exact decomposition: rotation about body X moves the
    // up-vector in the Y-Z plane and vice versa.
    pitchDeg =  atan2f(uy, uz) * RAD_TO_DEG;
    yawDeg   = -atan2f(ux, uz) * RAD_TO_DEG;
  }

  // Total angle between nose and vertical (for the abort check), degrees.
  float tiltMagnitudeDeg() const {
    float uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    uz = constrain(uz, -1.0f, 1.0f);
    return acosf(uz) * RAD_TO_DEG;
  }

  // Roll rate is not controllable with a 2-axis gimbal but is logged.
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

private:
  float kpAid = 0.0f;
  float ix = 0, iy = 0, iz = 0;   // (reserved for optional Ki aiding)
};
