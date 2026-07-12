// ============================================================
//  BLOCK 1 — PINS & INCLUDES
//  Board    : STM32F103C8T6 Blue Pill
//  Toolchain: Arduino + STM32duino
//


#pragma once
#include <SPI.h>
#include <Arduino.h>

// ── ESC PWM  (TIM2_CH1-4) ───────────────────────────────────
#define ESC1_PIN   PA0    // TIM2_CH1  Motor1 CW  Front-Right
#define ESC2_PIN   PA1    // TIM2_CH2  Motor2 CCW Front-Left
#define ESC3_PIN   PA2    // TIM2_CH3  Motor3 CW  Rear-Left
#define ESC4_PIN   PA3    // TIM2_CH4  Motor4 CCW Rear-Right

// ── GY-91 SPI1 ──────────────────────────────────────────────
#define IMU_SCK    PA5    // SPI1_SCK  (SCL on GY-91)
#define IMU_MISO   PA6    // SPI1_MISO (SDO on GY-91)
#define IMU_MOSI   PA7    // SPI1_MOSI (SDA on GY-91)
#define IMU_CS     PB0    // Chip-Select (NCS on GY-91)

// ── Serial ports ────────────────────────────────────────────
// USART1 : FTDI debug  TX=PA9  RX=PA10  → Serial1
// USART3 : i-BUS RX   RX=PB11          → Serial3
#define IBUS_SERIAL   Serial3
#define DEBUG_SERIAL  Serial1

// ── Status LEDs ─────────────────────────────────────────────
#define LED_RED    PB12   // HIGH = ON
#define LED_GREEN  PB13   // HIGH = ON

// ── MPU-9250 SPI registers ──────────────────────────────────
#define MPU_WHO_AM_I      0x75
#define MPU_PWR_MGMT_1    0x6B
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_GYRO_XOUT_H   0x43
#define MPU_SPI_READ      0x80

// ── i-BUS protocol ──────────────────────────────────────────
#define IBUS_BUFFSIZE     32
#define IBUS_SYNCBYTE1    0x20
#define IBUS_SYNCBYTE2    0x40
#define IBUS_NUM_CH       10

// ── Calibration constants ───────────────────────────────────
#define GYRO_CAL_SAMPLES  2000
#define GYRO_STILL_LIMIT  3000   // raw LSB — restart if exceeded

// ── Loop timing ─────────────────────────────────────────────
#define FLIGHT_LOOP_US    2500   // 400 Hz flight controller
#define CAL_LOOP_MS         20  //  50 Hz calibration tick

// ── Sensor scaling ──────────────────────────────────────────
// Defined here so blocks 2 and 7 share the same values
#define ACCEL_SCALE_INV   (1.0f / 16384.0f)  // ±2 g  → 1 g = 16384 LSB
#define GYRO_SCALE_DEG    (1.0f / 65.5f)     // ±500°/s → 65.5 LSB/°/s

// ── Flight limits ───────────────────────────────────────────
// Defined here so blocks 6 and 7 can both access them
#define MAX_ANGLE_DEG     30.0f   // max stick-commanded lean angle
#define MAX_YAW_RATE      150.0f  // max stick-commanded yaw rate °/s

// ── Madgwick beta ───────────────────────────────────────────
#define MADGWICK_BETA     0.04f   // 0.01 smoother, 0.1 faster
#define DEG_TO_RAD        (3.14159265f / 180.0f)
