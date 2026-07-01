#pragma once
// ============================================================================
//  Logger.h — black-box flight recorder (Teensy 4.0).
//
//  Records fixed-size binary frames into a RAM ring buffer during flight
//  (writing to SD/flash mid-flight can stall for milliseconds and glitch
//  the control loop), then after landing commits to:
//
//    * MicroSD card (primary): human-readable CSV, auto-numbered
//      flight000.csv, flight001.csv, ... so old flights are never lost.
//      Pop the card out and open it directly with tools/plot_log.py.
//    * W25Q128JVSIQ SPI flash (backup): binary LOG_FILENAME via LittleFS.
//      Retrieved over USB with the "dump" serial command even if the SD
//      card was missing or ejected on impact.
// ============================================================================

#include <Arduino.h>
#include <SD.h>
#include <LittleFS.h>
#include "config.h"

struct __attribute__((packed)) LogRecord {
  uint32_t tMs;        // millis since boot
  uint8_t  state;      // FlightState enum
  int16_t  pitchCdeg;  // tilt about X, centi-degrees
  int16_t  yawCdeg;    // tilt about Y, centi-degrees
  int16_t  gyroX, gyroY, gyroZ;    // deg/s * 10
  int16_t  accelX, accelY, accelZ; // g * 100
  int16_t  servoXCdeg; // commanded nozzle X, centi-degrees
  int16_t  servoYCdeg; // commanded nozzle Y, centi-degrees
  int16_t  altDm;      // baro altitude AGL, decimeters
  int16_t  vbatMv10;   // battery, 10 mV units
  uint8_t  flags;      // bits 0-3: pyro 1-4 fired
};  // 30 bytes

static const char *LOG_CSV_HEADER =
    "t_ms,state,pitch_deg,yaw_deg,gx_dps,gy_dps,gz_dps,"
    "ax_g,ay_g,az_g,tvc_x_deg,tvc_y_deg,alt_m,vbat_v,flags";

class Logger {
public:
  // Returns true if at least one storage medium is usable.
  bool begin() {
    sdOk    = SD.begin(PIN_SD_CS);
    flashOk = flashFs.begin(PIN_FLASH_CS, SPI);
    buf = (LogRecord *)malloc(sizeof(LogRecord) * LOG_CAPACITY);
    return (sdOk || flashOk) && buf != nullptr;
  }

  bool sdPresent() const    { return sdOk; }
  bool flashPresent() const { return flashOk; }

  void record(const LogRecord &r) {
    if (!buf) return;
    buf[head] = r;
    head = (head + 1) % LOG_CAPACITY;
    if (count < LOG_CAPACITY) count++;
  }

  // Write the ring buffer out (call once, after landing).
  bool commit() {
    if (!buf || committed || count == 0) return false;
    size_t start = (count == LOG_CAPACITY) ? head : 0;
    bool anyOk = false;

    // --- SD card: numbered CSV ---
    if (sdOk) {
      char name[24];
      for (int n = 0; n < 1000; n++) {
        snprintf(name, sizeof(name), "flight%03d.csv", n);
        if (!SD.exists(name)) break;
      }
      File f = SD.open(name, FILE_WRITE);
      if (f) {
        f.println(LOG_CSV_HEADER);
        for (size_t i = 0; i < count; i++)
          printCsvRecord(f, buf[(start + i) % LOG_CAPACITY]);
        f.close();
        anyOk = true;
      }
    }

    // --- SPI flash: binary backup ---
    if (flashOk) {
      flashFs.remove(LOG_FILENAME);
      File f = flashFs.open(LOG_FILENAME, FILE_WRITE);
      if (f) {
        for (size_t i = 0; i < count; i++) {
          const LogRecord &r = buf[(start + i) % LOG_CAPACITY];
          f.write((const uint8_t *)&r, sizeof(LogRecord));
        }
        f.close();
        anyOk = true;
      }
    }

    committed = anyOk;
    return anyOk;
  }

  // Stream the backup log from SPI flash over serial as CSV.
  void dumpCsv(Stream &out) {
    if (!flashOk) { out.println("ERR: no SPI flash"); return; }
    File f = flashFs.open(LOG_FILENAME, FILE_READ);
    if (!f) { out.println("ERR: no log file"); return; }
    out.println(LOG_CSV_HEADER);
    LogRecord r;
    while (f.read((uint8_t *)&r, sizeof(LogRecord)) == sizeof(LogRecord))
      printCsvRecord(out, r);
    f.close();
    out.println("EOF");
  }

  // New flight: clear RAM buffer and the flash backup (SD history is kept).
  void erase() {
    if (flashOk) flashFs.remove(LOG_FILENAME);
    head = count = 0;
    committed = false;
  }

private:
  LittleFS_SPIFlash flashFs;
  LogRecord *buf = nullptr;
  size_t head = 0, count = 0;
  bool sdOk = false, flashOk = false;
  bool committed = false;

  static void printCsvRecord(Print &out, const LogRecord &r) {
    out.print(r.tMs);                    out.print(',');
    out.print(r.state);                  out.print(',');
    out.print(r.pitchCdeg / 100.0f, 2);  out.print(',');
    out.print(r.yawCdeg / 100.0f, 2);    out.print(',');
    out.print(r.gyroX / 10.0f, 1);       out.print(',');
    out.print(r.gyroY / 10.0f, 1);       out.print(',');
    out.print(r.gyroZ / 10.0f, 1);       out.print(',');
    out.print(r.accelX / 100.0f, 2);     out.print(',');
    out.print(r.accelY / 100.0f, 2);     out.print(',');
    out.print(r.accelZ / 100.0f, 2);     out.print(',');
    out.print(r.servoXCdeg / 100.0f, 2); out.print(',');
    out.print(r.servoYCdeg / 100.0f, 2); out.print(',');
    out.print(r.altDm / 10.0f, 1);       out.print(',');
    out.print(r.vbatMv10 / 100.0f, 2);   out.print(',');
    out.println(r.flags);
  }
};
