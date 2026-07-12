// ============================================================
//  MASTER FLIGHT CONTROLLER — main.ino
//  Board    : STM32F103C8T6 Blue Pill
//  Toolchain: Arduino IDE + STM32duino Core

#include "01_pins_and_includes.h"
#include "02_imu_spi.h"
#include "03_ibus_receiver.h"
#include "04_esc_pwm.h"
#include "05_leds.h"
#include "06_calibration.h"
#include "07_pid_flight_controller.h"

// ═══════════════════════════════════════════════════════════════════
// HARDWARE INSTANTIATION — Forces the core to map Serial3 to USART3
// ═══════════════════════════════════════════════════════════════════
HardwareSerial Serial3(PB11, PB10); // RX pin = PB11, TX pin = PB10

// ── Loop timing trackers ─────────────────────────────────────
uint32_t last_flight_loop_time = 0;
uint32_t last_calib_loop_time  = 0;

// ── System mode flag ─────────────────────────────────────────
// false = Calibration / configuration menu  (default on boot)
// true  = Real-time 400 Hz flight engine active
bool systemInFlightMode = false;

void setup() {

  // 1. Initialise LEDs first so they give visual feedback
  //    throughout the entire boot sequence
  initLEDs();

  // 2. Start debug serial port before any other block
  //    uses it for printing — all init functions below
  //    send confirmation messages over DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
  delay(500);

  // 3. Initialise i-BUS receiver on USART3 PB11
  //    Starts IBUS_SERIAL at 115200 baud and prepares
  //    the 32-byte frame parser and channel buffer
  initIBUS();

  // 4. Initialise ESC PWM output on TIM2 PA0-PA3
  //    Configures bare-metal timer registers for
  //    400 Hz hardware PWM generation, then immediately
  //    holds all 4 channels at 1000 us (motors disarmed)
  initESC();
  motorsOff();

  // 5. Initialise MPU-9250 over SPI1
  //    Starts the SPI bus, verifies WHO_AM_I register,
  //    configures gyro to ±500 deg/s and accel to ±2 g.
  //    If sensor is not detected, trap execution and
  //    flash red LED as a visible fault indicator
  if (!initSensors()) {
    DEBUG_SERIAL.println(F("!! FATAL: IMU init failed"));
    DEBUG_SERIAL.println(F("!! Check wiring: SCK=PA5  MISO=PA6  MOSI=PA7  CS=PB0"));
    while (1) {
      ledERROR();
      delay(10);
    }
  }

  // 6. All hardware initialised successfully
  //    Run startup blink sequence then hold red LED
  //    Red = booted, in calibration mode, not yet armed
  ledStartupBlink();
  ledDISARMED();

  // 6a. Load saved PID gains + level offsets from Flash.
  //     Block 7 stores these via 'v' command. On first boot
  //     the magic number won't match and defaults are used.
  //     Must run AFTER initSensors() so EEPROM is ready.
  DEBUG_SERIAL.println(F("  Checking Flash for saved data..."));
  loadFromFlash();

  // 7. Print calibration mode command menu
  DEBUG_SERIAL.println(F("\n============================================="));
  DEBUG_SERIAL.println(F("    STM32 BLUE PILL FLIGHT CONTROLLER v5     "));
  DEBUG_SERIAL.println(F("============================================="));
  DEBUG_SERIAL.println(F(" CALIBRATION MODE COMMANDS:"));
  DEBUG_SERIAL.println(F("  'g'  Gyroscope calibration  (keep quad still)"));
  DEBUG_SERIAL.println(F("  's'  Stick calibration       (follow prompts)"));
  DEBUG_SERIAL.println(F("  'e'  ESC throttle range sync (REMOVE PROPS!)"));
  DEBUG_SERIAL.println(F("  'r'  Reset all calibration + cut motors"));
  DEBUG_SERIAL.println(F("  '?'  Reprint this menu"));
  DEBUG_SERIAL.println(F("---------------------------------------------"));
  DEBUG_SERIAL.println(F("  Recommended sequence before first flight:"));
  DEBUG_SERIAL.println(F("  1. g  2. s  3. e  4. f"));
  DEBUG_SERIAL.println(F("---------------------------------------------"));
  DEBUG_SERIAL.println(F(" !! SEND 'f' TO ENTER FLIGHT MODE !!"));
  DEBUG_SERIAL.println(F("=============================================\n"));

  // 8. Seed both loop timers so the first tick of each
  //    mode executes at the correct interval immediately
  last_flight_loop_time = micros();
  last_calib_loop_time  = millis();
}

// ════════════════════════════════════════════════════════════
//  MAIN LOOP
// ════════════════════════════════════════════════════════════
void loop() {

  // i-BUS frame parser — must run every single iteration
  // regardless of operating mode to continuously drain the
  // USART3 hardware buffer. Skipping even one call can
  // corrupt the next frame and cause RC signal loss
  updateIBUS();

  // ══════════════════════════════════════════════════════
  //  MODE A — CALIBRATION / CONFIGURATION
  //  Runs at 50 Hz. Accepts serial commands to perform
  //  gyro, stick and ESC calibration. Stays here until
  //  the user sends 'f' to commit and enter flight mode.
  // ══════════════════════════════════════════════════════
  if (!systemInFlightMode) {

    if (millis() - last_calib_loop_time >= CAL_LOOP_MS) {
      last_calib_loop_time = millis();

      // Keep IMU data fresh so sensor values are always
      // current for debug dumps and the gyro cal routine
      readMPU();

      // Advance the active calibration state machine:
      // GYRO_CAL  → samples 2000 readings for offsets
      // STICK_CAL → records center then sweeps full range
      // ESC_CAL   → sends max then min throttle sequence
      runCalibration();
    }

    // Check for incoming serial character each iteration.
    // Returns true only when 'f' is received, signalling
    // the user is satisfied with calibration and ready to fly
    if (parseCalibCommand()) {
      systemInFlightMode    = true;
      last_flight_loop_time = micros();

      // Red LED: flight mode active but quad is not yet
      // armed — user must perform the arm stick gesture
      ledDISARMED();

      DEBUG_SERIAL.println(F("\n============================================="));
      DEBUG_SERIAL.println(F("  >>>  FLIGHT MODE ACTIVE  <<<"));
      DEBUG_SERIAL.println(F("============================================="));
      DEBUG_SERIAL.println(F(" ARM   : Throttle LOW + Yaw RIGHT  hold 2 s"));
      DEBUG_SERIAL.println(F(" DISARM: Throttle LOW + Yaw LEFT   hold 2 s"));
      DEBUG_SERIAL.println(F("---------------------------------------------"));
      DEBUG_SERIAL.println(F(" TELEMETRY COMMANDS:"));
      DEBUG_SERIAL.println(F("  'a'  Toggle Roll/Pitch/Yaw angle stream"));
      DEBUG_SERIAL.println(F("  'd'  Toggle raw Accel/Gyro data stream"));
      DEBUG_SERIAL.println(F("  't'  Stop active telemetry stream"));
      DEBUG_SERIAL.println(F("  'i'  Single iBUS channel snapshot"));
      DEBUG_SERIAL.println(F("---------------------------------------------"));
      DEBUG_SERIAL.println(F(" TUNING COMMANDS:"));
      DEBUG_SERIAL.println(F("  'c'  Flat-level calibration (keep still!)"));
      DEBUG_SERIAL.println(F("  'p'  PID live tuner menu"));
      DEBUG_SERIAL.println(F("  'v'  Save gains + offsets to Flash"));
      DEBUG_SERIAL.println(F("---------------------------------------------"));
      DEBUG_SERIAL.println(F("  '?'  Reprint this menu"));
      DEBUG_SERIAL.println(F("=============================================\n"));
    }
  }

  // ══════════════════════════════════════════════════════
  //  MODE B — REAL-TIME FLIGHT ENGINE  (400 Hz)
  //  Executes a strict timed loop every 2500 us.
  //  Each tick: reads IMU → runs Madgwick → applies level
  //  offsets → runs PIDs → mixes outputs → writes TIM2 CCRs
  // ══════════════════════════════════════════════════════
  else {
    uint32_t now_us = micros();

    if (now_us - last_flight_loop_time >= FLIGHT_LOOP_US) {

      // Compute exact elapsed time since last tick in seconds.
      // Clamped to a safe range so the Madgwick integrator
      // and PID integral terms never receive an extreme dt
      // value on the very first tick or after any stall
      float dt = (float)(now_us - last_flight_loop_time) * 1e-6f;
      last_flight_loop_time = now_us;
      dt = constrain(dt, 0.0005f, 0.02f);

      // Burst-read 14 bytes from MPU-9250 over 18 MHz SPI:
      // Accel XYZ (6 bytes) + Temp (2 bytes skipped) + Gyro XYZ (6 bytes)
      // Applies axis orientation correction and gyro offset removal
      readMPU();

      // Run full flight control pipeline:
      //   1. Madgwick AHRS     → quaternion → Roll/Pitch/Yaw angles
      //   2. Arm/disarm gesture detection
      //   3. Level offsets     → corrected_roll / corrected_pitch
      //   4. PID compute       → Roll/Pitch (angle mode), Yaw (rate mode)
      //   5. Motor mixing      → X-frame layout
      //   6. TIM2 CCR write    → PA0-PA3 PWM pulse widths
      flightControllerUpdate(dt);
    }

    // Non-blocking telemetry + PID tuner parser — runs every
    // iteration so serial characters are never missed between
    // flight ticks. Handles streams, level cal, PID tuning,
    // and Flash save without blocking the 400 Hz loop
    parseFlightCommand();
  }
}
