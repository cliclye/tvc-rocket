#pragma once
// ============================================================================
//  BMP280.h — minimal register-level I2C driver for the Bosch BMP280
//  (GY-BMP280 breakout), with the official compensation math from the
//  Bosch datasheet (section 3.11.3, 32-bit fixed point variant converted
//  to float for clarity — the Teensy 4.0 has a hardware FPU).
//
//  Configuration: normal mode, pressure oversampling x8, temp x1,
//  IIR filter coefficient 4, standby 0.5 ms  =>  ~14 ms per sample,
//  comfortably faster than our 50 Hz poll rate.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class BMP280 {
public:
  static constexpr uint8_t REG_CALIB    = 0x88;   // 24 bytes of trim
  static constexpr uint8_t REG_CHIP_ID  = 0xD0;   // reads 0x58
  static constexpr uint8_t REG_RESET    = 0xE0;
  static constexpr uint8_t REG_CTRL_MEAS = 0xF4;
  static constexpr uint8_t REG_CONFIG   = 0xF5;
  static constexpr uint8_t REG_DATA     = 0xF7;   // press[3] then temp[3]

  BMP280(TwoWire &wire, uint8_t addr) : _wire(wire), _addr(addr) {}

  bool begin() {
    uint8_t id = readReg(REG_CHIP_ID);
    if (id != 0x58 && id != 0x60) return false;   // 0x60 = BME280, also fine

    writeReg(REG_RESET, 0xB6);                    // soft reset
    delay(5);
    if (!readCalibration()) return false;

    // config: t_sb=0.5ms (000), IIR filter x4 (010), spi3w off
    writeReg(REG_CONFIG, (0b000 << 5) | (0b010 << 2));
    // ctrl_meas: osrs_t x1 (001), osrs_p x8 (100), normal mode (11)
    writeReg(REG_CTRL_MEAS, (0b001 << 5) | (0b100 << 2) | 0b11);
    delay(50);
    return true;
  }

  // pressure in Pa, temperature in degC
  bool read(float &pressurePa, float &tempC) {
    uint8_t b[6];
    if (!readRegs(REG_DATA, b, 6)) return false;
    int32_t up = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    int32_t ut = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);

    // --- temperature compensation (datasheet, float variant) ---
    float v1 = (ut / 16384.0f - T1 / 1024.0f) * T2;
    float v2 = (ut / 131072.0f - T1 / 8192.0f);
    v2 = v2 * v2 * T3;
    float tFine = v1 + v2;
    tempC = tFine / 5120.0f;

    // --- pressure compensation (datasheet, float variant) ---
    float p1 = tFine / 2.0f - 64000.0f;
    float p2 = p1 * p1 * P6 / 32768.0f;
    p2 = p2 + p1 * P5 * 2.0f;
    p2 = p2 / 4.0f + P4 * 65536.0f;
    p1 = (P3 * p1 * p1 / 524288.0f + P2 * p1) / 524288.0f;
    p1 = (1.0f + p1 / 32768.0f) * P1;
    if (p1 == 0.0f) return false;                 // avoid div by zero
    float p = 1048576.0f - up;
    p = (p - p2 / 4096.0f) * 6250.0f / p1;
    p1 = P9 * p * p / 2147483648.0f;
    p2 = p * P8 / 32768.0f;
    pressurePa = p + (p1 + p2 + P7) / 16.0f;
    return true;
  }

  // Barometric altitude above the reference pressure, in meters.
  static float altitudeM(float pressurePa, float refPa) {
    return 44330.0f * (1.0f - powf(pressurePa / refPa, 0.190284f));
  }

private:
  TwoWire &_wire;
  uint8_t _addr;
  // Trim coefficients (names per datasheet dig_T*/dig_P*)
  float T1, T2, T3;
  float P1, P2, P3, P4, P5, P6, P7, P8, P9;

  bool readCalibration() {
    uint8_t c[24];
    if (!readRegs(REG_CALIB, c, 24)) return false;
    // All little-endian; T1/P1 unsigned, the rest signed.
    T1 = (uint16_t)(c[0]  | (c[1] << 8));
    T2 = (int16_t) (c[2]  | (c[3] << 8));
    T3 = (int16_t) (c[4]  | (c[5] << 8));
    P1 = (uint16_t)(c[6]  | (c[7] << 8));
    P2 = (int16_t) (c[8]  | (c[9] << 8));
    P3 = (int16_t) (c[10] | (c[11] << 8));
    P4 = (int16_t) (c[12] | (c[13] << 8));
    P5 = (int16_t) (c[14] | (c[15] << 8));
    P6 = (int16_t) (c[16] | (c[17] << 8));
    P7 = (int16_t) (c[18] | (c[19] << 8));
    P8 = (int16_t) (c[20] | (c[21] << 8));
    P9 = (int16_t) (c[22] | (c[23] << 8));
    return true;
  }

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
