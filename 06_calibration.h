// ============================================================
//  BLOCK 6 — CALIBRATION STATE MACHINE
//  States: IDLE → GYRO_CAL / STICK_CAL / ESC_CAL → DONE
// ============================================================

#pragma once
#include "01_pins_and_includes.h"
#include "02_imu_spi.h"
#include "03_ibus_receiver.h"   
#include "04_esc_pwm.h"
#include "05_leds.h"

// ── Calibration state enum ───────────────────────────────────
enum CalState { CAL_IDLE, GYRO_CAL, STICK_CAL, ESC_CAL, CAL_DONE };
CalState calState = CAL_IDLE;

// ── Calibration data ─────────────────────────────────────────
struct CalibData {
  // Gyro
  int      gyro_samples  = 0;
  long     gyro_sum[3]   = {0, 0, 0};

  // Stick
  uint16_t s_min[4]      = {2000, 2000, 2000, 2000};
  uint16_t s_max[4]      = {1000, 1000, 1000, 1000};
  uint16_t s_center[4]   = {1500, 1500, 1500, 1500};
  uint8_t  stick_phase   = 0;

  // ESC
  uint8_t  esc_phase     = 0;

  // Generic timer
  uint32_t start         = 0;
} calib;

void resetCalib() { calib = CalibData(); }

// ── Normalised stick value with deadzone ─────────────────────
// Returns float 1000.0–2000.0
float getStickValue(uint8_t chan) {
  if (!ibusValid() || chan >= 4) {
    return (chan == 2) ? 1000.0f : 1500.0f;
  }

  uint16_t raw = (uint16_t)constrain(
    (int)ibus_ch[chan],
    (int)calib.s_min[chan],
    (int)calib.s_max[chan]
  );

  float normalized = (float)map(
    (long)raw,
    (long)calib.s_min[chan],
    (long)calib.s_max[chan],
    1000L, 2000L
  );

  // 3% deadzone around recorded center
  float dz = (calib.s_max[chan] - calib.s_min[chan]) * 0.03f;
  if (fabsf((float)raw - (float)calib.s_center[chan]) < dz) {
    return 1500.0f;
  }

  return constrain(normalized, 1000.0f, 2000.0f);
}

// ════════════════════════════════════════════════════════════
//  GYRO CALIBRATION
//  Place quad flat and perfectly still.
//  Collects 2000 samples and averages them for offsets.
// ════════════════════════════════════════════════════════════
void updateGyroCal() {
  if (calib.gyro_samples == 0) {
    ledGYROCAL();
    memset(calib.gyro_sum, 0, sizeof(calib.gyro_sum));
    calib.start = millis();
    DEBUG_SERIAL.println(">> GYRO CAL: Keep quad PERFECTLY STILL...");
  }

  readMPU();

  // Restart if quad is moved during sampling
  if (abs(gx_raw) > GYRO_STILL_LIMIT ||
      abs(gy_raw) > GYRO_STILL_LIMIT ||
      abs(gz_raw) > GYRO_STILL_LIMIT) {
    DEBUG_SERIAL.println("!! MOVEMENT DETECTED — restarting gyro cal");
    calib.gyro_samples = 0;
    memset(calib.gyro_sum, 0, sizeof(calib.gyro_sum));
    return;
  }

  // Accumulate raw (pre-flip) values — offsets live in raw domain
  calib.gyro_sum[0] += gx_raw;
  calib.gyro_sum[1] += gy_raw;
  calib.gyro_sum[2] += gz_raw;
  calib.gyro_samples++;

  if (calib.gyro_samples % 500 == 0) {
    DEBUG_SERIAL.print("  Progress: ");
    DEBUG_SERIAL.print(calib.gyro_samples);
    DEBUG_SERIAL.print(" / ");
    DEBUG_SERIAL.println(GYRO_CAL_SAMPLES);
  }

  if (calib.gyro_samples >= GYRO_CAL_SAMPLES) {
    gyro_offset[0] = calib.gyro_sum[0] / (float)GYRO_CAL_SAMPLES;
    gyro_offset[1] = calib.gyro_sum[1] / (float)GYRO_CAL_SAMPLES;
    gyro_offset[2] = calib.gyro_sum[2] / (float)GYRO_CAL_SAMPLES;
    DEBUG_SERIAL.print("✓ GYRO OFFSETS  X:");
    DEBUG_SERIAL.print(gyro_offset[0], 2);
    DEBUG_SERIAL.print("  Y:");
    DEBUG_SERIAL.print(gyro_offset[1], 2);
    DEBUG_SERIAL.print("  Z:");
    DEBUG_SERIAL.println(gyro_offset[2], 2);
    calState = CAL_DONE;
  }
}

// ════════════════════════════════════════════════════════════
//  STICK CALIBRATION
//  Phase 0→1 : record center position (3 s)
//  Phase 1→2 : sweep full travel on all sticks (10 s)
// ════════════════════════════════════════════════════════════
void updateStickCal() {
  switch (calib.stick_phase) {

    case 0:
      ledSTICKCAL();
      DEBUG_SERIAL.println(">> STICK CAL 1/2: Hold sticks at CENTER for 3 s...");
      calib.start = millis();
      calib.stick_phase = 1;
      break;

    case 1:
      if (millis() - calib.start > 3000 && ibusValid()) {
        for (int i = 0; i < 4; i++) {
          calib.s_center[i] = (uint16_t)ibus_ch[i];
          calib.s_min[i]    = (uint16_t)ibus_ch[i];
          calib.s_max[i]    = (uint16_t)ibus_ch[i];
        }
        DEBUG_SERIAL.println(">> STICK CAL 2/2: SWEEP ALL STICKS full range for 10 s!");
        calib.start = millis();
        calib.stick_phase = 2;
      }
      break;

    case 2: {
      for (int i = 0; i < 4; i++) {
        uint16_t v = (uint16_t)ibus_ch[i];
        if (v < calib.s_min[i]) calib.s_min[i] = v;
        if (v > calib.s_max[i]) calib.s_max[i] = v;
      }

      // Live feedback every 500 ms
      static uint32_t fb = 0;
      if (millis() - fb > 500) {
        fb = millis();
        DEBUG_SERIAL.print("  R[");  DEBUG_SERIAL.print(calib.s_min[0]);
        DEBUG_SERIAL.print("-");     DEBUG_SERIAL.print(calib.s_max[0]);
        DEBUG_SERIAL.print("] P[");  DEBUG_SERIAL.print(calib.s_min[1]);
        DEBUG_SERIAL.print("-");     DEBUG_SERIAL.print(calib.s_max[1]);
        DEBUG_SERIAL.print("] T[");  DEBUG_SERIAL.print(calib.s_min[2]);
        DEBUG_SERIAL.print("-");     DEBUG_SERIAL.print(calib.s_max[2]);
        DEBUG_SERIAL.print("] Y[");  DEBUG_SERIAL.print(calib.s_min[3]);
        DEBUG_SERIAL.print("-");     DEBUG_SERIAL.print(calib.s_max[3]);
        DEBUG_SERIAL.println("]");
      }

      if (millis() - calib.start > 10000) {
        DEBUG_SERIAL.println("✓ STICK CALIBRATION COMPLETE");
        for (int i = 0; i < 4; i++) {
          DEBUG_SERIAL.print("  CH"); DEBUG_SERIAL.print(i);
          DEBUG_SERIAL.print("  min=");    DEBUG_SERIAL.print(calib.s_min[i]);
          DEBUG_SERIAL.print("  max=");    DEBUG_SERIAL.print(calib.s_max[i]);
          DEBUG_SERIAL.print("  center="); DEBUG_SERIAL.println(calib.s_center[i]);
        }
        calState = CAL_DONE;
        return;
      }

      // 15 s global timeout guard
      if (millis() - calib.start > 15000) {
        DEBUG_SERIAL.println("!! STICK CAL TIMEOUT — type 's' to retry");
        calState = CAL_IDLE;
      }
      break;
    }
  }
}

// ════════════════════════════════════════════════════════════
// ════════════════════════════════════════════════════════════
//  ESC CALIBRATION
//  !! REMOVE PROPELLERS BEFORE RUNNING !!
//  Triggered instantly by typing 'e' via serial monitor.
// ════════════════════════════════════════════════════════════
void updateEscCal() {
  switch (calib.esc_phase) {
    
    case 0:
      // NEW ENTRY LOGIC: Trigger MAX throttle instantly when 'e' is pressed!
      // This forces the high-signal out BEFORE checking for your receiver.
      ledESCCAL();
      DEBUG_SERIAL.println(">> ESC CAL: Sending MAX throttle (3 s) — PROPS OFF!");
      DEBUG_SERIAL.println(">> ESC CAL: Perform heartbeat tap to synchronize receiver NOW...");
      setAllMotors(2000);
      calib.start     = millis();
      calib.esc_phase = 1;
      break;

    case 1:
      // HOLD MAX signal until your transmitter syncs AND 3 seconds pass.
      // This gives you all the time in the world to do your heartbeat tap.
      if (millis() - calib.start > 3000) {
        if (!ibusValid()) {
          // If 3 seconds passed but you haven't tapped yet, hold max throttle safely
          static uint32_t tWait = 0;
          if (millis() - tWait > 1500) {
            tWait = millis();
            DEBUG_SERIAL.println("  [Holding MAX] Waiting for heartbeat tap to verify i-BUS stream...");
          }
          return; 
        }
        
        // Once receiver is fully active, drop to minimum
        DEBUG_SERIAL.println(">> ESC CAL: Receiver verified! Sending MIN throttle (3 s)...");
        setAllMotors(1000);
        calib.start     = millis();
        calib.esc_phase = 2;
      }
      break;

    case 2:
      if (millis() - calib.start > 3000) {
        motorsOff();
        calib.esc_phase = 0;
        DEBUG_SERIAL.println("✓ ESC CALIBRATION DONE");
        calState = CAL_DONE;
      }
      break;
  }
}

// ── Calibration dispatcher — call from main loop at 50 Hz ───
void runCalibration() {
  switch (calState) {
    case GYRO_CAL:  updateGyroCal();  break;
    case STICK_CAL: updateStickCal(); break;
    case ESC_CAL:   updateEscCal();   break;
    case CAL_DONE:  ledDONE();        break;
    default:        ledIDLE();        break;
  }
}

// ── Serial command parser ────────────────────────────────────
bool parseCalibCommand() {
  if (!DEBUG_SERIAL.available()) return false;
  char cmd = DEBUG_SERIAL.read();
  switch (cmd) {
    case 'g':
      calState = GYRO_CAL;  resetCalib();
      DEBUG_SERIAL.println(">> CMD: Gyro calibration — keep still");
      break;
    case 's':
      calState = STICK_CAL; resetCalib();
      DEBUG_SERIAL.println(">> CMD: Stick calibration");
      break;
    case 'e':
      calState = ESC_CAL;   resetCalib();
      DEBUG_SERIAL.println(">> CMD: ESC calibration  !! REMOVE PROPS !!");
      break;
    case 'r':
      calState = CAL_IDLE;  resetCalib(); motorsOff();
      setLED(false, false);
      DEBUG_SERIAL.println(">> CMD: Reset");
      break;
    case 'f':
      // Signal main.ino to switch to flight mode
      return true;
    case '?':
      DEBUG_SERIAL.println("─────────────────────────────────");
      DEBUG_SERIAL.println(" g = Gyro cal    (keep still)");
      DEBUG_SERIAL.println(" s = Stick cal   (follow prompts)");
      DEBUG_SERIAL.println(" e = ESC cal     (PROPS OFF!)");
      DEBUG_SERIAL.println(" f = Switch to FLIGHT mode");
      DEBUG_SERIAL.println(" r = Reset");
      DEBUG_SERIAL.println("─────────────────────────────────");
      break;
  }
  return false;
}