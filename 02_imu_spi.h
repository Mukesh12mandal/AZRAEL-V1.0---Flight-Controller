// ============================================================
//  BLOCK 2 — GY-91 / MPU-9250  SPI DRIVER
//  Accelerometer + Gyroscope only (BMP280 & Mag ignored)
//  SPI1 : SCK=PA5  MISO=PA6  MOSI=PA7  CS=PB0
//
//  FIXES FROM v1:
//  FIX-1 : Gyro offset sign bug — offset must be subtracted from
//           raw value BEFORE applying axis inversion, not after.
//           Old (wrong):  gy = -gy_raw + gyro_offset[1]
//           New (correct): gy = -(gy_raw - (int16_t)gyro_offset[1])
//           Mathematically:  -raw + offset  ≠  -(raw - offset)
//           when offset ≠ 0.  The corrected form ensures the offset
//           measured on the raw axis is consistently removed.
//  FIX-2 : Global variables declared here are externed in main.ino
//           — no re-declaration needed in other blocks that include
//           this header (all other blocks must include this file).
// ============================================================

#pragma once
#include "01_pins_and_includes.h"

// ── Raw sensor output (global) ──────────────────────────────
// Used by: block 6 (gyro cal samples gx_raw/gy_raw/gz_raw)
//          block 7 (madgwick reads ax/ay/az, gx/gy/gz)
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

// ── Orientation-corrected + offset-removed values ───────────
// These are what the flight controller and Madgwick use
int16_t ax, ay, az;
int16_t gx, gy, gz;

// ── Gyro offsets — filled by block 6 gyro calibration ───────
float gyro_offset[3] = {0.0f, 0.0f, 0.0f};

// ── SPI chip-select helpers ─────────────────────────────────
static inline void imu_cs_low()  { digitalWrite(IMU_CS, LOW);  }
static inline void imu_cs_high() { digitalWrite(IMU_CS, HIGH); }

// ── Low-level register access ───────────────────────────────
uint8_t mpuReadReg(uint8_t reg) {
  imu_cs_low();
  SPI.transfer(reg | MPU_SPI_READ);
  uint8_t val = SPI.transfer(0x00);
  imu_cs_high();
  return val;
}

void mpuWriteReg(uint8_t reg, uint8_t data) {
  imu_cs_low();
  SPI.transfer(reg & ~MPU_SPI_READ);
  SPI.transfer(data);
  imu_cs_high();
}

// ── Initialise MPU-9250 via SPI ─────────────────────────────
bool initSensors() {
  pinMode(IMU_CS, OUTPUT);
  imu_cs_high();

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // 4.5 MHz — safe for config
  SPI.setDataMode(SPI_MODE3);            // CPOL=1 CPHA=1 for MPU-9250
  SPI.setBitOrder(MSBFIRST);
  delay(100);

  // Who am I check - MPU-9250 = 0x71 / MPU-6500 variant = 0x70
  uint8_t whoami = mpuReadReg(MPU_WHO_AM_I);
  if (whoami != 0x71 && whoami != 0x70) {
    DEBUG_SERIAL.print("!! MPU WHO_AM_I FAIL: 0x");
    DEBUG_SERIAL.println(whoami, HEX);
    return false;
  }
  DEBUG_SERIAL.print("✓ MPU-9250  WHO_AM_I=0x");
  DEBUG_SERIAL.println(whoami, HEX);

  mpuWriteReg(MPU_PWR_MGMT_1,   0x01);   // Wake, PLL gyro clock
  delay(50);
  mpuWriteReg(MPU_GYRO_CONFIG,  0x08);   // ±500 °/s
  mpuWriteReg(MPU_ACCEL_CONFIG, 0x00);   // ±2 g

  SPI.setClockDivider(SPI_CLOCK_DIV4);   // 18 MHz — fast data reads
  return true;
}

// ── Burst-read 6-axis (accel + gyro) ────────────────────────
void readMPU() {
  // Read 14 bytes: ACCEL(6) + TEMP(2) + GYRO(6)
  imu_cs_low();
  SPI.transfer(MPU_ACCEL_XOUT_H | MPU_SPI_READ);

  ax_raw = (int16_t)((SPI.transfer(0) << 8) | SPI.transfer(0));
  ay_raw = (int16_t)((SPI.transfer(0) << 8) | SPI.transfer(0));
  az_raw = (int16_t)((SPI.transfer(0) << 8) | SPI.transfer(0));
  SPI.transfer(0); SPI.transfer(0);   // skip TEMP_OUT (2 bytes)
  gx_raw = (int16_t)((SPI.transfer(0) << 8) | SPI.transfer(0));
  gy_raw = (int16_t)((SPI.transfer(0) << 8) | SPI.transfer(0));
  gz_raw = (int16_t)((SPI.transfer(0) << 8) | SPI.transfer(0));

  imu_cs_high();

  // ── Axis orientation: X=forward  Y=right  Z=up ──────────
  ax =  ax_raw;                                          // accel not offset-calibrated
  ay = -ay_raw;
  az = -az_raw;

  gx =  (gx_raw - (int16_t)gyro_offset[0]);             // no axis flip
  gy = -(gy_raw - (int16_t)gyro_offset[1]);             // FIX-1 applied
  gz = -(gz_raw - (int16_t)gyro_offset[2]);             // FIX-1 applied
}