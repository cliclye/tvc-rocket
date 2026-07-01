#pragma once
// ============================================================================
//  MPU6050.h — minimal register-level I2C driver for the InvenSense MPU-6050
//  (GY-521 breakout module).
//
//  Configuration used here:
//    Gyro : ±2000 dps  (16.4 LSB per dps)
//    Accel: ±16 g      (2048 LSB per g)
//    DLPF : 42 Hz (config 3) — good vibration rejection for a control loop,
//           sample rate 1 kHz internally.
//
//  Register map: InvenSense RM-MPU-6000A-00.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>

class MPU6050 {
public:
  static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
  static constexpr uint8_t REG_CONFIG       = 0x1A;
  static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
  static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
  static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;  // start of 14-byte burst
  static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
  static constexpr uint8_t REG_WHO_AM_I     = 0x75;  // reads 0x68

  static constexpr float GYRO_LSB_PER_DPS = 16.4f;   // at ±2000 dps
  static constexpr float ACCEL_LSB_PER_G  = 2048.0f; // at ±16 g

  MPU6050(TwoWire &wire, uint8_t addr) : _wire(wire), _addr(addr) {}

  // Returns true on success.
  bool begin() {
    // WHO_AM_I returns the 7-bit address bits [6:1] = 0x68 regardless of AD0
    if ((readReg(REG_WHO_AM_I) & 0x7E) != 0x68) return false;

    writeReg(REG_PWR_MGMT_1, 0x80);       // device reset
    delay(100);
    writeReg(REG_PWR_MGMT_1, 0x01);       // wake, clock = PLL w/ X gyro ref
    delay(10);

    writeReg(REG_SMPLRT_DIV, 0x00);       // sample rate = gyro rate / 1
    writeReg(REG_CONFIG, 0x03);           // DLPF 42 Hz (gyro rate 1 kHz)
    writeReg(REG_GYRO_CONFIG,  0b11 << 3); // FS_SEL  3: ±2000 dps
    writeReg(REG_ACCEL_CONFIG, 0b11 << 3); // AFS_SEL 3: ±16 g
    delay(50);                            // let everything settle
    return true;
  }

  // Burst-reads accel + temp + gyro. Outputs: g's and deg/s in CHIP frame.
  bool read(float accelG[3], float gyroDps[3], float *tempC = nullptr) {
    uint8_t b[14];
    if (!readRegs(REG_ACCEL_XOUT_H, b, 14)) return false;

    int16_t rawA[3], rawG[3];
    for (int i = 0; i < 3; i++)
      rawA[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    int16_t rawT = (int16_t)((b[6] << 8) | b[7]);
    for (int i = 0; i < 3; i++)
      rawG[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]);

    for (int i = 0; i < 3; i++) {
      accelG[i]  = rawA[i] / ACCEL_LSB_PER_G;
      gyroDps[i] = rawG[i] / GYRO_LSB_PER_DPS;
    }
    if (tempC) *tempC = rawT / 340.0f + 36.53f;   // datasheet formula
    return true;
  }

private:
  TwoWire &_wire;
  uint8_t _addr;

  void writeReg(uint8_t reg, uint8_t val) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    _wire.write(val);
    _wire.endTransmission();
  }

  uint8_t readReg(uint8_t reg) {
    uint8_t v = 0;
    readRegs(reg, &v, 1);
    return v;
  }

  bool readRegs(uint8_t reg, uint8_t *buf, size_t len) {
    _wire.beginTransmission(_addr);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom(_addr, (uint8_t)len) != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = _wire.read();
    return true;
  }
};
