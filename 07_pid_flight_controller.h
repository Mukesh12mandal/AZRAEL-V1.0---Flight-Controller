// ============================================================
//  BLOCK 7 — MADGWICK AHRS + PID + FLIGHT CONTROLLER
//  Madgwick: accel + gyro only (no magnetometer)
//  PID: Roll (angle), Pitch (angle), Yaw (rate)
//  Motor mixing: X-frame quadcopter
//
//  Motor layout (top view):
//        FRONT
//    M2(CCW)  M1(CW)
//    M3(CW)   M4(CCW)
//        REAR
//
//  i-BUS channel map:
//    CH0=Roll  CH1=Pitch  CH2=Throttle  CH3=Yaw
//
//  Telemetry commands (flight mode only):
//    'a' = toggle Euler angle stream  (Roll Pitch Yaw)
//    'd' = toggle raw IMU stream      (AX AY AZ GX GY GZ)
//    't' = stop active stream
//    'i' = single iBUS channel snapshot
//    'c' = run flat-level calibration  (keep drone still!)
//    'p' = open PID live tuner menu
//    'v' = save current gains + offsets to Flash
//    '?' = help
// ============================================================

#pragma once
#include "01_pins_and_includes.h"
#include "02_imu_spi.h"
#include "03_ibus_receiver.h"
#include "04_esc_pwm.h"
#include "05_leds.h"
#include "06_calibration.h"
#include <EEPROM.h>

// ════════════════════════════════════════════════════════════
//  EEPROM LAYOUT  (STM32duino emulated Flash, 4 bytes/float)
//
//  Addr  Content
//   0    roll_offset
//   4    pitch_offset
//   8    pid_roll.kp
//  12    pid_roll.ki
//  16    pid_roll.kd
//  20    pid_pitch.kp
//  24    pid_pitch.ki
//  28    pid_pitch.kd
//  32    pid_yaw.kp
//  36    pid_yaw.ki
//  40    pid_yaw.kd
//  44    magic = 0xDEADBEEF  (marks valid save)
// ════════════════════════════════════════════════════════════
#define EEPROM_MAGIC      0xDEADBEEF
#define EEPROM_ADDR_ROLLOFF   0
#define EEPROM_ADDR_PITCHOFF  4
#define EEPROM_ADDR_RKP       8
#define EEPROM_ADDR_RKI      12
#define EEPROM_ADDR_RKD      16
#define EEPROM_ADDR_PKP      20
#define EEPROM_ADDR_PKI      24
#define EEPROM_ADDR_PKD      28
#define EEPROM_ADDR_YKP      32
#define EEPROM_ADDR_YKI      36
#define EEPROM_ADDR_YKD      40
#define EEPROM_ADDR_MAGIC    44

// ════════════════════════════════════════════════════════════
//  MADGWICK AHRS
//  Reference: S. Madgwick, "An efficient orientation filter
//  for inertial and inertial/magnetic sensor arrays", 2010.
//
//  MADGWICK_BETA tradeoff (defined in block 1):
//    lower  (0.01) = smoother angles, slow to correct drift
//    higher (0.1)  = faster correction, more accel noise
//    recommended start: 0.04
// ════════════════════════════════════════════════════════════

// Quaternion state — unit quaternion representing orientation
// Initialised to identity (no rotation)
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Euler output in degrees — written by madgwickUpdate()
float roll_angle  = 0.0f;
float pitch_angle = 0.0f;
float yaw_angle   = 0.0f;

// ── Dynamic flat-level calibration offsets ───────────────────
// Computed at runtime by runLevelCal() — subtracted from
// Madgwick output before PID so drone reads 0/0 when flat.
// Loaded from Flash on boot if a valid save exists.
float roll_offset  = 0.0f;
float pitch_offset = 0.0f;

// ── Fast inverse square root ─────────────────────────────────
// Quake III method with 2 Newton-Raphson iterations.
// Guard added: returns 0 for near-zero input to prevent the
// quaternion or gradient vector from exploding to infinity
static float invSqrt(float x) {
  if (x < 1e-10f) return 0.0f;   // safety guard for near-zero
  float halfx = 0.5f * x;
  union { float f; uint32_t i; } conv = {x};
  conv.i = 0x5f3759df - (conv.i >> 1);
  conv.f *= (1.5f - halfx * conv.f * conv.f);
  conv.f *= (1.5f - halfx * conv.f * conv.f);
  return conv.f;
}

// ── Madgwick filter update ───────────────────────────────────
//   gxd/gyd/gzd : gyro degrees/s  (block 2 corrected values)
//   axr/ayr/azr : accel raw LSB   (block 2 corrected values)
//   dt          : loop period in seconds
void madgwickUpdate(float gxd, float gyd, float gzd,
                    float axr, float ayr, float azr,
                    float dt) {
  float gx_r = gxd * DEG_TO_RAD;
  float gy_r = gyd * DEG_TO_RAD;
  float gz_r = gzd * DEG_TO_RAD;
  float ax_g = axr * ACCEL_SCALE_INV;
  float ay_g = ayr * ACCEL_SCALE_INV;
  float az_g = azr * ACCEL_SCALE_INV;

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3;
  float _4q0, _4q1, _4q2, _8q1, _8q2;
  float q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1*gx_r - q2*gy_r - q3*gz_r);
  qDot2 = 0.5f * ( q0*gx_r + q2*gz_r - q3*gy_r);
  qDot3 = 0.5f * ( q0*gy_r - q1*gz_r + q3*gx_r);
  qDot4 = 0.5f * ( q0*gz_r + q1*gy_r - q2*gx_r);

  // Apply accelerometer correction only when accel magnitude is
  // close to 1g (0.7–1.58g). Values outside this window indicate
  // a vibration spike or free-fall — skip correction to avoid
  // feeding bad data into the gradient descent step
  float accel_mag = ax_g*ax_g + ay_g*ay_g + az_g*az_g;
  if (accel_mag > 0.5f && accel_mag < 2.5f) {

    // Normalise accelerometer vector
    recipNorm = invSqrt(accel_mag);
    ax_g *= recipNorm; ay_g *= recipNorm; az_g *= recipNorm;

    _2q0=2.0f*q0; _2q1=2.0f*q1; _2q2=2.0f*q2; 
    _4q0=4.0f*q0; _4q1=4.0f*q1; _4q2=4.0f*q2;
    _8q1=8.0f*q1; _8q2=8.0f*q2;
    q0q0=q0*q0; q1q1=q1*q1; q2q2=q2*q2; q3q3=q3*q3;

    // Gradient of objective function (Madgwick eq. 25)
    s0 = _4q0*q2q2 + _2q2*ax_g + _4q0*q1q1 - _2q1*ay_g;
    s1 = _4q1*q3q3 - _2q3*ax_g + 4.0f*q0q0*q1 - _2q0*ay_g
       - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az_g;
    s2 = 4.0f*q0q0*q2 + _2q0*ax_g + _4q2*q3q3 - _2q3*ay_g
       - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az_g;
    s3 = 4.0f*q1q1*q3 - _2q1*ax_g + 4.0f*q2q2*q3 - _2q2*ay_g;

    // Normalise gradient
    recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (recipNorm != 0.0f) {
      s0 *= recipNorm; s1 *= recipNorm;
      s2 *= recipNorm; s3 *= recipNorm;

      // Apply feedback step
      qDot1 -= MADGWICK_BETA * s0;
      qDot2 -= MADGWICK_BETA * s1;
      qDot3 -= MADGWICK_BETA * s2;
      qDot4 -= MADGWICK_BETA * s3;
    }
  }

  // Integrate rate of change to yield new quaternion
  q0 += qDot1 * dt; q1 += qDot2 * dt;
  q2 += qDot3 * dt; q3 += qDot4 * dt;

  // Normalise quaternion — guard against all-zero quaternion
  // which would make invSqrt return garbage and corrupt state
  float qnorm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
  if (qnorm < 1e-10f) {
    // Quaternion has collapsed — reset to identity and return
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    return;
  }
  recipNorm = invSqrt(qnorm);
  q0 *= recipNorm; q1 *= recipNorm;
  q2 *= recipNorm; q3 *= recipNorm;

  // Convert quaternion to Euler angles in degrees.
  // The pitch asinf() argument is theoretically bounded to +-1.0
  // by the unit quaternion constraint, but floating point
  // accumulation over many steps can push it slightly outside
  // (e.g. 1.0000002f). asinf() returns NaN for |x| > 1 on
  // ARM Cortex-M3, which then propagates through the entire PID
  // pipeline producing garbage PWM values. Clamp to +-1 first.
  float sinp  = constrain(2.0f*(q1*q3 - q0*q2), -1.0f, 1.0f);

  roll_angle  =  atan2f(2.0f*(q0*q1 + q2*q3),
                        1.0f - 2.0f*(q1*q1 + q2*q2)) / DEG_TO_RAD;
  pitch_angle = -asinf(sinp)                          / DEG_TO_RAD;
  yaw_angle   =  atan2f(2.0f*(q0*q3 + q1*q2),
                        1.0f - 2.0f*(q2*q2 + q3*q3)) / DEG_TO_RAD;
}

// ════════════════════════════════════════════════════════════
//  FLAT-LEVEL CALIBRATION
//
//  Triggered by 'c' command in parseFlightCommand().
//  Drone must be perfectly flat and still.
//  Runs 500 samples at 4 ms each (~2 seconds).
//  Computes average roll/pitch and stores as offsets.
//  These offsets are subtracted in flightControllerUpdate()
//  so PID always sees 0/0 when the drone is level.
//
//  Can be re-run any time without rebooting — useful when
//  the FC is remounted at a different angle in the frame.
// ════════════════════════════════════════════════════════════
void runLevelCal() {
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
  DEBUG_SERIAL.println(F("  FLAT-LEVEL CALIBRATION"));
  DEBUG_SERIAL.println(F("  Place drone on flat surface."));
  DEBUG_SERIAL.println(F("  Do NOT touch it. Starting in 2s..."));
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
  delay(2000);

  // Warm up Madgwick — discard first 200 samples so the
  // filter has fully converged before we start averaging
  DEBUG_SERIAL.println(F("  Warming up filter..."));
  for (int i = 0; i < 200; i++) {
    updateIBUS();
    readMPU();
    float gx_dps = (float)gx * GYRO_SCALE_DEG;
    float gy_dps = (float)gy * GYRO_SCALE_DEG;
    float gz_dps = (float)gz * GYRO_SCALE_DEG;
    madgwickUpdate(gx_dps, gy_dps, gz_dps,
                   (float)ax, (float)ay, (float)az,
                   0.004f);
    delay(4);
  }

  // Collect 500 samples and average them
  DEBUG_SERIAL.println(F("  Sampling... (keep still)"));
  float roll_sum  = 0.0f;
  float pitch_sum = 0.0f;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    updateIBUS();
    readMPU();
    float gx_dps = (float)gx * GYRO_SCALE_DEG;
    float gy_dps = (float)gy * GYRO_SCALE_DEG;
    float gz_dps = (float)gz * GYRO_SCALE_DEG;
    madgwickUpdate(gx_dps, gy_dps, gz_dps,
                   (float)ax, (float)ay, (float)az,
                   0.004f);
    roll_sum  += roll_angle;
    pitch_sum += pitch_angle;
    delay(4);
  }

  // Store computed offsets in global variables
  roll_offset  = roll_sum  / (float)samples;
  pitch_offset = pitch_sum / (float)samples;

  DEBUG_SERIAL.println(F("  ✓ LEVEL CAL COMPLETE!"));
  DEBUG_SERIAL.print(F("  Roll offset  = ")); DEBUG_SERIAL.println(roll_offset,  3);
  DEBUG_SERIAL.print(F("  Pitch offset = ")); DEBUG_SERIAL.println(pitch_offset, 3);
  DEBUG_SERIAL.println(F("  Send 'v' to save offsets to Flash."));
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
}

// ════════════════════════════════════════════════════════════
//  PID CONTROLLER
//
//  Implements:
//    P  — proportional on error
//    I  — integral with anti-windup clamp
//    D  — derivative on measurement to eliminate derivative
//         kick when setpoint steps (e.g. stick snap)
//    LP — first-order low-pass filter on the D term to
//         suppress high-frequency sensor noise
//
//  Tuning order (always with props OFF first):
//    1. Set ki=0, kd=0. Raise kp until slow oscillation, halve it.
//    2. Raise kd until oscillations damp within 1-2 cycles.
//    3. Add small ki (start 0.01) to remove steady lean.
// ════════════════════════════════════════════════════════════

struct PID {
  float kp, ki, kd;
  float i_limit;   // anti-windup clamp — same units as output
  float out_limit; // maximum output magnitude
  float d_filter;  // D low-pass: 0.0 = off, 0.7 = moderate

  // Controller state — call reset() whenever motors are killed
  float integral      = 0.0f;
  float prev_error    = 0.0f;
  float prev_measured = 0.0f;   // used for D-on-measurement
  float prev_d        = 0.0f;   // previous RAW derivative sample

  // C++ constructor — compatible with all STM32duino versions
  PID(float _kp, float _ki, float _kd,
      float _ilim, float _olim, float _df = 0.7f)
    : kp(_kp), ki(_ki), kd(_kd),
      i_limit(_ilim), out_limit(_olim), d_filter(_df),
      integral(0), prev_error(0), prev_measured(0), prev_d(0) {}

  float compute(float setpoint, float measured, float dt) {
    // Guard against zero or negative dt — would cause divide-by-zero
    // in the D term. This should never happen if main.ino clamps dt,
    // but the PID must be self-protecting regardless of caller
    if (dt < 1e-6f) dt = 1e-6f;

    float error = setpoint - measured;

    // Proportional term
    float p_out = kp * error;

    // Integral term with anti-windup clamp
    integral = constrain(integral + error * dt, -i_limit, i_limit);
    float i_out = ki * integral;

    // Derivative on measurement — avoids derivative kick when
    // setpoint changes suddenly (stick snap). The derivative of
    // (setpoint - measured) with constant setpoint equals the
    // negative derivative of measured, so we compute:
    //   raw_d = -(measured - prev_measured) / dt
    // This gives the same damping effect without the kick.
    float raw_d = -(measured - prev_measured) / dt;
    float d_out = kd * (d_filter * prev_d + (1.0f - d_filter) * raw_d);
    prev_d        = raw_d;
    prev_measured = measured;
    prev_error    = error;

    return constrain(p_out + i_out + d_out, -out_limit, out_limit);
  }

  void reset() {
    integral      = 0.0f;
    prev_error    = 0.0f;
    prev_measured = 0.0f;
    prev_d        = 0.0f;
  }
};

// ── PID instances ────────────────────────────────────────────
// Roll  — angle mode  (setpoint = degrees)
// Pitch — angle mode  (setpoint = degrees)
// Yaw   — rate  mode  (setpoint = degrees/s), no D needed
PID pid_roll ( 1.2f, 0.02f, 0.5f, 200.0f, 400.0f, 0.7f);
PID pid_pitch( 1.2f, 0.02f, 0.5f, 200.0f, 400.0f, 0.7f);
PID pid_yaw  ( 2.5f, 0.05f, 0.0f, 200.0f, 400.0f, 0.0f);

// ════════════════════════════════════════════════════════════
//  EEPROM SAVE / LOAD
//  Saves: roll_offset, pitch_offset, all 9 PID gains.
//  A magic number (0xDEADBEEF) marks a valid save so we
//  never accidentally load uninitialised Flash garbage.
// ════════════════════════════════════════════════════════════

void saveToFlash() {
  EEPROM.put(EEPROM_ADDR_ROLLOFF,  roll_offset);
  EEPROM.put(EEPROM_ADDR_PITCHOFF, pitch_offset);
  EEPROM.put(EEPROM_ADDR_RKP,      pid_roll.kp);
  EEPROM.put(EEPROM_ADDR_RKI,      pid_roll.ki);
  EEPROM.put(EEPROM_ADDR_RKD,      pid_roll.kd);
  EEPROM.put(EEPROM_ADDR_PKP,      pid_pitch.kp);
  EEPROM.put(EEPROM_ADDR_PKI,      pid_pitch.ki);
  EEPROM.put(EEPROM_ADDR_PKD,      pid_pitch.kd);
  EEPROM.put(EEPROM_ADDR_YKP,      pid_yaw.kp);
  EEPROM.put(EEPROM_ADDR_YKI,      pid_yaw.ki);
  EEPROM.put(EEPROM_ADDR_YKD,      pid_yaw.kd);
  uint32_t magic = EEPROM_MAGIC;
  EEPROM.put(EEPROM_ADDR_MAGIC,    magic);

  DEBUG_SERIAL.println(F("════════════════════════════════════"));
  DEBUG_SERIAL.println(F("  ✓ SAVED TO FLASH"));
  DEBUG_SERIAL.print(F("  Roll offset  = ")); DEBUG_SERIAL.println(roll_offset,  3);
  DEBUG_SERIAL.print(F("  Pitch offset = ")); DEBUG_SERIAL.println(pitch_offset, 3);
  DEBUG_SERIAL.print(F("  Roll  KP=")); DEBUG_SERIAL.print(pid_roll.kp, 3);
  DEBUG_SERIAL.print(F(" KI="));        DEBUG_SERIAL.print(pid_roll.ki, 3);
  DEBUG_SERIAL.print(F(" KD="));        DEBUG_SERIAL.println(pid_roll.kd, 3);
  DEBUG_SERIAL.print(F("  Pitch KP=")); DEBUG_SERIAL.print(pid_pitch.kp, 3);
  DEBUG_SERIAL.print(F(" KI="));        DEBUG_SERIAL.print(pid_pitch.ki, 3);
  DEBUG_SERIAL.print(F(" KD="));        DEBUG_SERIAL.println(pid_pitch.kd, 3);
  DEBUG_SERIAL.print(F("  Yaw   KP=")); DEBUG_SERIAL.print(pid_yaw.kp,   3);
  DEBUG_SERIAL.print(F(" KI="));        DEBUG_SERIAL.print(pid_yaw.ki,   3);
  DEBUG_SERIAL.print(F(" KD="));        DEBUG_SERIAL.println(pid_yaw.kd,   3);
  DEBUG_SERIAL.println(F("  Will survive power cycle."));
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
}

void loadFromFlash() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);

  if (magic != EEPROM_MAGIC) {
    DEBUG_SERIAL.println(F("  No saved data in Flash — using defaults."));
    return;
  }

  EEPROM.get(EEPROM_ADDR_ROLLOFF,  roll_offset);
  EEPROM.get(EEPROM_ADDR_PITCHOFF, pitch_offset);
  EEPROM.get(EEPROM_ADDR_RKP,      pid_roll.kp);
  EEPROM.get(EEPROM_ADDR_RKI,      pid_roll.ki);
  EEPROM.get(EEPROM_ADDR_RKD,      pid_roll.kd);
  EEPROM.get(EEPROM_ADDR_PKP,      pid_pitch.kp);
  EEPROM.get(EEPROM_ADDR_PKI,      pid_pitch.ki);
  EEPROM.get(EEPROM_ADDR_PKD,      pid_pitch.kd);
  EEPROM.get(EEPROM_ADDR_YKP,      pid_yaw.kp);
  EEPROM.get(EEPROM_ADDR_YKI,      pid_yaw.ki);
  EEPROM.get(EEPROM_ADDR_YKD,      pid_yaw.kd);

  DEBUG_SERIAL.println(F("  ✓ Loaded from Flash:"));
  DEBUG_SERIAL.print(F("    Roll offset=")); DEBUG_SERIAL.print(roll_offset, 3);
  DEBUG_SERIAL.print(F("  Pitch offset=")); DEBUG_SERIAL.println(pitch_offset, 3);
  DEBUG_SERIAL.print(F("    Roll  KP=")); DEBUG_SERIAL.print(pid_roll.kp, 3);
  DEBUG_SERIAL.print(F(" KI="));          DEBUG_SERIAL.print(pid_roll.ki, 3);
  DEBUG_SERIAL.print(F(" KD="));          DEBUG_SERIAL.println(pid_roll.kd, 3);
  DEBUG_SERIAL.print(F("    Pitch KP=")); DEBUG_SERIAL.print(pid_pitch.kp, 3);
  DEBUG_SERIAL.print(F(" KI="));          DEBUG_SERIAL.print(pid_pitch.ki, 3);
  DEBUG_SERIAL.print(F(" KD="));          DEBUG_SERIAL.println(pid_pitch.kd, 3);
  DEBUG_SERIAL.print(F("    Yaw   KP=")); DEBUG_SERIAL.print(pid_yaw.kp,   3);
  DEBUG_SERIAL.print(F(" KI="));          DEBUG_SERIAL.print(pid_yaw.ki,   3);
  DEBUG_SERIAL.print(F(" KD="));          DEBUG_SERIAL.println(pid_yaw.kd,   3);
}

// ════════════════════════════════════════════════════════════
//  FLIGHT CONTROLLER STATE
// ════════════════════════════════════════════════════════════

bool armed = false;
static uint32_t arm_timer    = 0;
static uint32_t disarm_timer = 0;

// ── Arm / Disarm gesture detection ───────────────────────────
//   ARM    : Throttle < 1100  AND  Yaw > 1800  held 2 s
//   DISARM : Throttle < 1100  AND  Yaw < 1200  held 2 s
void checkArmDisarm(float thr, float yaw) {
  if (!armed) {
    if (thr < 1100.0f && yaw > 1800.0f) {
      if (arm_timer == 0) arm_timer = millis();
      if (millis() - arm_timer > 2000) {
        armed     = true;
        arm_timer = 0;
        DEBUG_SERIAL.println(">> ARMED");
        ledARMED();
      }
    } else {
      arm_timer = 0;
    }
  } else {
    if (thr < 1100.0f && yaw < 1200.0f) {
      if (disarm_timer == 0) disarm_timer = millis();
      if (millis() - disarm_timer > 2000) {
        armed        = false;
        disarm_timer = 0;
        motorsOff();
        pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset();
        DEBUG_SERIAL.println(">> DISARMED");
        ledDISARMED();
      }
    } else {
      disarm_timer = 0;
    }
  }
}

// ── Throttle channel read without center deadzone ────────────
// getStickValue() applies a 3% center deadzone to all channels.
// For throttle this is wrong — center (1500) = 50% power.
// Throttle needs only range-mapping with no center snap.
// This function maps raw ibus throttle to 1000-2000 directly.
float getThrottle() {
  if (!ibusValid()) return 1000.0f;
  uint16_t raw = (uint16_t)constrain(
    (int)ibus_ch[2],
    (int)calib.s_min[2],
    (int)calib.s_max[2]
  );
  return constrain(
    (float)map((long)raw,
               (long)calib.s_min[2],
               (long)calib.s_max[2],
               1000L, 2000L),
    1000.0f, 2000.0f
  );
}

// ── Motor mixing — X-frame ───────────────────────────────────
//   M1 FR CW  : +T  -R  +P  -Y
//   M2 FL CCW : +T  +R  +P  +Y
//   M3 RL CW  : +T  +R  -P  -Y
//   M4 RR CCW : +T  -R  -P  +Y
void mixAndWrite(float thr, float r, float p, float y) {
  if (!armed) { motorsOff(); return; }

  // Safety floor: below minimum throttle kill motors and
  // reset PID integrators to prevent integral windup
  if (thr < 1100.0f) {
    motorsOff();
    pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset();
    return;
  }

  float t  = thr - 1000.0f;   // rescale to 0-1000
  float m1 = t - r + p - y;   // Front-Right CW
  float m2 = t + r + p + y;   // Front-Left  CCW
  float m3 = t + r - p - y;   // Rear-Left   CW
  float m4 = t - r - p + y;   // Rear-Right  CCW

  // If any motor exceeds maximum, scale all down proportionally
  // to preserve attitude authority at high throttle
  float mx = max(max(m1, m2), max(m3, m4));
  if (mx > 1000.0f) {
    float sc = 1000.0f / mx;
    m1 *= sc; m2 *= sc; m3 *= sc; m4 *= sc;
  }

  // Floor at minimum idle to prevent motor stall and
  // eliminate re-spool lag on rapid throttle inputs
  m1 = max(m1, 100.0f); m2 = max(m2, 100.0f);
  m3 = max(m3, 100.0f); m4 = max(m4, 100.0f);

  setMotor(1, (uint16_t)(m1 + 1000));
  setMotor(2, (uint16_t)(m2 + 1000));
  setMotor(3, (uint16_t)(m3 + 1000));
  setMotor(4, (uint16_t)(m4 + 1000));
}

// ── Main flight controller tick ──────────────────────────────
//   Called from main loop at fixed dt (FLIGHT_LOOP_US period)
void flightControllerUpdate(float dt) {

  // If RC signal is lost: kill motors, disarm, reset PIDs.
  // Message is rate-limited to once per second to prevent
  // serial flooding at 400 Hz
  if (!ibusValid()) {
    static uint32_t tWarn = 0;
    if (millis() - tWarn > 1000) {
      tWarn = millis();
      DEBUG_SERIAL.println("!! RC SIGNAL LOST — motors killed");
    }
    motorsOff();
    pid_roll.reset(); pid_pitch.reset(); pid_yaw.reset();
    armed = false;
    return;
  }

  // Read throttle without center deadzone (see getThrottle())
  // Read roll/pitch/yaw with center deadzone via getStickValue()
  float raw_thr = getThrottle();
  float raw_rol = getStickValue(0);
  float raw_pit = getStickValue(1);
  float raw_yaw = getStickValue(3);

  // Check arm/disarm stick gesture
  checkArmDisarm(raw_thr, raw_yaw);

  // Convert block-2 gyro LSB to degrees/s for Madgwick and yaw PID
  float gx_dps = (float)gx * GYRO_SCALE_DEG;
  float gy_dps = (float)gy * GYRO_SCALE_DEG;
  float gz_dps = (float)gz * GYRO_SCALE_DEG;

  // Run Madgwick filter — updates roll_angle, pitch_angle, yaw_angle
  madgwickUpdate(gx_dps, gy_dps, gz_dps,
                 (float)ax, (float)ay, (float)az,
                 dt);

  // Apply dynamic flat-level offsets — these are computed by
  // runLevelCal() and represent whatever tilt the IMU had when
  // the drone was sitting flat. Subtracting them here means the
  // PID always sees 0/0 as the true level reference, regardless
  // of how the FC is physically mounted in the frame.
  float corrected_roll  = roll_angle  - roll_offset;
  float corrected_pitch = pitch_angle - pitch_offset;

  // Map stick inputs to physical setpoints
  // Roll/Pitch: angle mode — stick deflection = desired lean angle
  // Yaw       : rate  mode — stick deflection = desired rotation rate
  float sp_roll  = ((raw_rol - 1500.0f) / 500.0f) * MAX_ANGLE_DEG;
  float sp_pitch = ((raw_pit - 1500.0f) / 500.0f) * MAX_ANGLE_DEG;
  float sp_yaw   = ((raw_yaw - 1500.0f) / 500.0f) * MAX_YAW_RATE;

  // Compute PID outputs — feed corrected angles, not raw ones
  float out_roll  = pid_roll.compute( sp_roll,  corrected_roll,  dt);
  float out_pitch = pid_pitch.compute(sp_pitch, corrected_pitch, dt);
  float out_yaw   = pid_yaw.compute(  sp_yaw,   gz_dps,          dt);

  // Mix PID outputs with throttle and write to motors
  mixAndWrite(raw_thr, out_roll, out_pitch, out_yaw);
}

// ════════════════════════════════════════════════════════════
//  IN-FLIGHT TELEMETRY + PID TUNER PARSER
//  Non-blocking — call every main loop iteration.
//
//  STREAM COMMANDS:
//    'a' = toggle Euler angle stream  (Roll Pitch Yaw corrected)
//    'd' = toggle raw IMU stream      (AX AY AZ | GX GY GZ)
//    't' = stop active stream
//    'i' = single iBUS channel snapshot
//
//  CALIBRATION COMMANDS:
//    'c' = run flat-level calibration (keep drone still!)
//    'v' = save offsets + PID gains to Flash
//
//  PID LIVE TUNER:
//    'p'      = show PID menu + current gains + safe ranges
//    rp<val>  = set Roll  KP  e.g. rp1.5
//    ri<val>  = set Roll  KI  e.g. ri0.02
//    rd<val>  = set Roll  KD  e.g. rd0.4
//    pp<val>  = set Pitch KP
//    pi<val>  = set Pitch KI
//    pd<val>  = set Pitch KD
//    yp<val>  = set Yaw   KP
//    yi<val>  = set Yaw   KI
//    yd<val>  = set Yaw   KD (not recommended, leave 0)
//
//  Only one stream runs at a time. Starting one stops the other.
//  Stream rate = STREAM_INTERVAL_MS.
// ════════════════════════════════════════════════════════════

static bool     stream_imu    = false;
static bool     stream_angles = false;
static uint32_t tStream       = 0;
#define STREAM_INTERVAL_MS  100   // ms between stream prints

// ── Helper: print all current gains ──────────────────────────
static void printGains() {
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
  DEBUG_SERIAL.println(F("   CURRENT PID GAINS"));
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
  DEBUG_SERIAL.print(F("  Roll  KP=")); DEBUG_SERIAL.print(pid_roll.kp,  3);
  DEBUG_SERIAL.print(F("  KI="));       DEBUG_SERIAL.print(pid_roll.ki,  3);
  DEBUG_SERIAL.print(F("  KD="));       DEBUG_SERIAL.println(pid_roll.kd, 3);
  DEBUG_SERIAL.print(F("  Pitch KP=")); DEBUG_SERIAL.print(pid_pitch.kp, 3);
  DEBUG_SERIAL.print(F("  KI="));       DEBUG_SERIAL.print(pid_pitch.ki, 3);
  DEBUG_SERIAL.print(F("  KD="));       DEBUG_SERIAL.println(pid_pitch.kd,3);
  DEBUG_SERIAL.print(F("  Yaw   KP=")); DEBUG_SERIAL.print(pid_yaw.kp,   3);
  DEBUG_SERIAL.print(F("  KI="));       DEBUG_SERIAL.print(pid_yaw.ki,   3);
  DEBUG_SERIAL.print(F("  KD="));       DEBUG_SERIAL.println(pid_yaw.kd,  3);
  DEBUG_SERIAL.println(F("────────────────────────────────────"));
  DEBUG_SERIAL.print(F("  Level offsets: Roll="));
  DEBUG_SERIAL.print(roll_offset, 3);
  DEBUG_SERIAL.print(F("  Pitch="));
  DEBUG_SERIAL.println(pitch_offset, 3);
  DEBUG_SERIAL.println(F("════════════════════════════════════"));
}

void parseFlightCommand() {

  // ── Process incoming serial character ───────────────────
  if (DEBUG_SERIAL.available()) {
    char cmd = DEBUG_SERIAL.read();
    switch (cmd) {

      // ── Stream toggles ─────────────────────────────────
      case 'd':
        stream_angles = false;
        stream_imu    = !stream_imu;
        DEBUG_SERIAL.println(stream_imu
          ? F(">> IMU stream ON   (send 't' to stop)")
          : F(">> IMU stream OFF"));
        break;

      case 'a':
        stream_imu    = false;
        stream_angles = !stream_angles;
        DEBUG_SERIAL.println(stream_angles
          ? F(">> Angle stream ON  (send 't' to stop)")
          : F(">> Angle stream OFF"));
        break;

      case 't':
        if (stream_imu || stream_angles) {
          stream_imu    = false;
          stream_angles = false;
          DEBUG_SERIAL.println(F(">> Stream STOPPED"));
        }
        break;

      case 'i':
        DEBUG_SERIAL.print(F("iBUS CH0-3:  "));
        for (int i = 0; i < 4; i++) {
          DEBUG_SERIAL.print(ibus_ch[i]);
          DEBUG_SERIAL.print(F("  "));
        }
        DEBUG_SERIAL.println(ibusValid() ? F("[OK]") : F("[NO SIGNAL]"));
        break;

      // ── Flat-level calibration ──────────────────────────
      // Disarms first for safety, then runs 500-sample average.
      // Safe to run at any time on the bench.
      case 'c':
        if (armed) {
          DEBUG_SERIAL.println(F("!! DISARM before running level cal!"));
          break;
        }
        runLevelCal();
        break;

      // ── Save to Flash ───────────────────────────────────
      case 'v':
        saveToFlash();
        break;

      // ── PID tuner menu ──────────────────────────────────
      case 'p':
        DEBUG_SERIAL.println(F("════════════════════════════════════"));
        DEBUG_SERIAL.println(F("   PID LIVE TUNER"));
        DEBUG_SERIAL.println(F("════════════════════════════════════"));
        printGains();
        DEBUG_SERIAL.println(F("  HOW TO SET A GAIN:"));
        DEBUG_SERIAL.println(F("  [axis][term][value]  — no spaces"));
        DEBUG_SERIAL.println(F("  axis: r=Roll  p=Pitch  y=Yaw"));
        DEBUG_SERIAL.println(F("  term: p=KP    i=KI     d=KD"));
        DEBUG_SERIAL.println(F("  e.g.  rp1.8   ri0.02   rd0.5"));
        DEBUG_SERIAL.println(F("────────────────────────────────────"));
        DEBUG_SERIAL.println(F("  SAFE RANGES:"));
        DEBUG_SERIAL.println(F("  KP : 0.5 – 4.0   (default 1.2)"));
        DEBUG_SERIAL.println(F("  KI : 0.0 – 0.10  (default 0.02)"));
        DEBUG_SERIAL.println(F("  KD : 0.0 – 1.0   (default 0.5 )"));
        DEBUG_SERIAL.println(F("  !! Values outside range are BLOCKED"));
        DEBUG_SERIAL.println(F("────────────────────────────────────"));
        DEBUG_SERIAL.println(F("  'p'  = show this menu again"));
        DEBUG_SERIAL.println(F("  'v'  = save current gains to Flash"));
        DEBUG_SERIAL.println(F("  'c'  = redo flat-level calibration"));
        DEBUG_SERIAL.println(F("════════════════════════════════════"));
        break;

      // ── Live gain entry: r/p/y followed by p/i/d + value ─
      // Format: rp1.8  ri0.02  rd0.5  pp1.5  yp2.0 etc.
      // 'r' and 'y' are not used by other commands so safe
      // to catch here. 'p' is the PID menu above — but when
      // followed by a digit or dot it becomes a Pitch gain entry.
      // We read the rest of the buffer to disambiguate.
      case 'r':
      case 'y': {
        // Wait briefly for the rest of the command to arrive
        delay(2);
        String line = "";
        line += cmd;
        while (DEBUG_SERIAL.available())
          line += (char)DEBUG_SERIAL.read();
        line.trim();

        if (line.length() < 3) {
          DEBUG_SERIAL.println(F("!! Incomplete. Format: rp1.8 ri0.02 rd0.5"));
          break;
        }

        char axis = line[0];
        char term = line[1];
        float val = line.substring(2).toFloat();

        // Validate term character
        if (term != 'p' && term != 'i' && term != 'd') {
          DEBUG_SERIAL.println(F("!! Unknown term. Use p / i / d"));
          break;
        }

        // Range check — block unsafe values entirely
        bool blocked = false;
        if (term == 'p' && (val < 0.5f || val > 4.0f))  blocked = true;
        if (term == 'i' && (val < 0.0f || val > 0.10f)) blocked = true;
        if (term == 'd' && (val < 0.0f || val > 1.0f))  blocked = true;

        if (blocked) {
          DEBUG_SERIAL.print(F("!! BLOCKED — value out of safe range: "));
          DEBUG_SERIAL.println(val, 3);
          DEBUG_SERIAL.println(F("   KP: 0.5-4.0   KI: 0.0-0.10   KD: 0.0-1.0"));
          break;
        }

        // Apply to correct PID axis
        PID* target = (axis == 'r') ? &pid_roll
                    : (axis == 'y') ? &pid_yaw
                    :                  &pid_pitch;

        if      (term == 'p') target->kp = val;
        else if (term == 'i') target->ki = val;
        else if (term == 'd') target->kd = val;

        // Reset integrator — stale integral from old gain
        // would cause a bump/lurch immediately after change
        target->reset();

        DEBUG_SERIAL.print(F(">> APPLIED  "));
        DEBUG_SERIAL.print(axis); DEBUG_SERIAL.print(term);
        DEBUG_SERIAL.print(F(" = ")); DEBUG_SERIAL.println(val, 3);
        DEBUG_SERIAL.println(F("   Send 'p' to review all gains."));
        DEBUG_SERIAL.println(F("   Send 'v' to save when happy."));
        break;
      }

      // ── Pitch gain entry — pp / pi / pd ────────────────
      // 'p' alone = menu (handled above).
      // 'p' + p/i/d + number = pitch gain change.
      // We peek at the buffer to decide which it is.
      // Note: this runs as a SECOND check inside the same
      // switch — we re-enter with a full string read.
      // Handled by falling into default and re-reading:
      // Actually we handle it by checking inside 'p' case
      // if next char is p/i/d digit.  Done below in default.

      // ── Help ───────────────────────────────────────────
      case '?':
        DEBUG_SERIAL.println(F("─────────────────────────────────────"));
        DEBUG_SERIAL.println(F(" FLIGHT MODE COMMANDS:"));
        DEBUG_SERIAL.println(F("  a  = Roll/Pitch/Yaw angle stream (toggle)"));
        DEBUG_SERIAL.println(F("  d  = Raw IMU stream (toggle)"));
        DEBUG_SERIAL.println(F("  t  = Stop active stream"));
        DEBUG_SERIAL.println(F("  i  = iBUS channel snapshot"));
        DEBUG_SERIAL.println(F("  c  = Flat-level calibration"));
        DEBUG_SERIAL.println(F("  p  = PID live tuner menu"));
        DEBUG_SERIAL.println(F("  v  = Save gains + offsets to Flash"));
        DEBUG_SERIAL.println(F("  ?  = This help menu"));
        DEBUG_SERIAL.println(F("─────────────────────────────────────"));
        break;

      default:
        // Catch pitch gain commands: pp / pi / pd + value
        // These start with 'p' followed immediately by p/i/d
        // We check the buffer for that pattern here.
        if (cmd == 'p' && DEBUG_SERIAL.available()) {
          delay(2);
          String line = "p";
          while (DEBUG_SERIAL.available())
            line += (char)DEBUG_SERIAL.read();
          line.trim();

          // If second char is p/i/d and there are digits after — it's a gain
          if (line.length() >= 3 &&
              (line[1] == 'p' || line[1] == 'i' || line[1] == 'd')) {

            char term = line[1];
            float val = line.substring(2).toFloat();

            bool blocked = false;
            if (term == 'p' && (val < 0.5f || val > 4.0f))  blocked = true;
            if (term == 'i' && (val < 0.0f || val > 0.10f)) blocked = true;
            if (term == 'd' && (val < 0.0f || val > 1.0f))  blocked = true;

            if (blocked) {
              DEBUG_SERIAL.print(F("!! BLOCKED — value out of safe range: "));
              DEBUG_SERIAL.println(val, 3);
              DEBUG_SERIAL.println(F("   KP: 0.5-4.0   KI: 0.0-0.10   KD: 0.0-1.0"));
            } else {
              if      (term == 'p') pid_pitch.kp = val;
              else if (term == 'i') pid_pitch.ki = val;
              else if (term == 'd') pid_pitch.kd = val;
              pid_pitch.reset();
              DEBUG_SERIAL.print(F(">> APPLIED  p"));
              DEBUG_SERIAL.print(term);
              DEBUG_SERIAL.print(F(" = ")); DEBUG_SERIAL.println(val, 3);
              DEBUG_SERIAL.println(F("   Send 'p' to review all gains."));
              DEBUG_SERIAL.println(F("   Send 'v' to save when happy."));
            }
          } else {
            // No buffer content after 'p' — show PID menu
            DEBUG_SERIAL.println(F("════════════════════════════════════"));
            DEBUG_SERIAL.println(F("   PID LIVE TUNER"));
            DEBUG_SERIAL.println(F("════════════════════════════════════"));
            printGains();
            DEBUG_SERIAL.println(F("  HOW TO SET A GAIN:"));
            DEBUG_SERIAL.println(F("  [axis][term][value]  — no spaces"));
            DEBUG_SERIAL.println(F("  axis: r=Roll  p=Pitch  y=Yaw"));
            DEBUG_SERIAL.println(F("  term: p=KP    i=KI     d=KD"));
            DEBUG_SERIAL.println(F("  e.g.  rp1.8   ri0.02   rd0.5"));
            DEBUG_SERIAL.println(F("────────────────────────────────────"));
            DEBUG_SERIAL.println(F("  SAFE RANGES:"));
            DEBUG_SERIAL.println(F("  KP : 0.5 – 4.0   (default 1.2)"));
            DEBUG_SERIAL.println(F("  KI : 0.0 – 0.10  (default 0.02)"));
            DEBUG_SERIAL.println(F("  KD : 0.0 – 1.0   (default 0.5 )"));
            DEBUG_SERIAL.println(F("  !! Values outside range are BLOCKED"));
            DEBUG_SERIAL.println(F("────────────────────────────────────"));
            DEBUG_SERIAL.println(F("  'p'  = show this menu again"));
            DEBUG_SERIAL.println(F("  'v'  = save current gains to Flash"));
            DEBUG_SERIAL.println(F("  'c'  = redo flat-level calibration"));
            DEBUG_SERIAL.println(F("════════════════════════════════════"));
          }
        }
        break;
    }
  }

  // ── Continuous stream output at fixed rate ───────────────
  if (millis() - tStream < STREAM_INTERVAL_MS) return;
  tStream = millis();

  if (stream_imu) {
    DEBUG_SERIAL.print(F("AX:")); DEBUG_SERIAL.print(ax);
    DEBUG_SERIAL.print(F("  AY:")); DEBUG_SERIAL.print(ay);
    DEBUG_SERIAL.print(F("  AZ:")); DEBUG_SERIAL.print(az);
    DEBUG_SERIAL.print(F("  |  GX:")); DEBUG_SERIAL.print(gx);
    DEBUG_SERIAL.print(F("  GY:")); DEBUG_SERIAL.print(gy);
    DEBUG_SERIAL.print(F("  GZ:")); DEBUG_SERIAL.println(gz);
  }

  if (stream_angles) {
    // Stream corrected angles so the plotter shows 0/0 when flat
    DEBUG_SERIAL.print(F("Roll:"));   DEBUG_SERIAL.print(roll_angle  - roll_offset,  3);
    DEBUG_SERIAL.print(F("  Pitch:")); DEBUG_SERIAL.print(pitch_angle - pitch_offset, 3);
    DEBUG_SERIAL.print(F("  Yaw:"));  DEBUG_SERIAL.println(yaw_angle, 3);
  }
}