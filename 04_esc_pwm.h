// ============================================================
//  BLOCK 4 — ESC PWM OUTPUT  (TIM2  CH1-4 = PA0-PA3)
//  400 Hz update rate (2500 µs period), 1 µs resolution
//  Range: 1000 µs (armed/min) → 2000 µs (full throttle)
//
//  Motor assignment:
//    TIM2_CH1  PA0  Motor1 CW  Front-Right
//    TIM2_CH2  PA1  Motor2 CCW Front-Left
//    TIM2_CH3  PA2  Motor3 CW  Rear-Left
//    TIM2_CH4  PA3  Motor4 CCW Rear-Right
//
//  CHANGES FROM v1:
//  - No logic changes. Block was correct.
//  - Added explicit include guard comment explaining why
//    DEBUG_SERIAL.println in initESC() requires Serial1 to be
//    started before initESC() is called in setup().
// ============================================================

#pragma once
#include "01_pins_and_includes.h"

// ── Initialise TIM2 for 4-channel PWM ───────────────────────
// NOTE: DEBUG_SERIAL (Serial1) must be started before calling
//       initESC() or the println inside will silently fail.
void initESC() {
  // Enable TIM2 and GPIOA clocks
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  // PA0-PA3: Alternate Function Push-Pull 50 MHz
  // CRL bits [15:0] control PA0-PA3, each pin = 4 bits (CNF:MODE)
  // CNF=10 (AF PP), MODE=11 (50 MHz) → nibble = 0xB
  GPIOA->CRL = (GPIOA->CRL & ~0x0000FFFF) | 0x0000BBBB;

  // 72 MHz / PSC+1 = 1 MHz → 1 µs per tick
  TIM2->PSC = 72 - 1;

  // ARR = 2500 - 1 → period = 2500 µs = 400 Hz
  // Change to 20000 - 1 for standard 50 Hz ESCs if needed
  TIM2->ARR = 2500 - 1;

  // All channels start at 1000 µs (motors disarmed)
  TIM2->CCR1 = 1000;
  TIM2->CCR2 = 1000;
  TIM2->CCR3 = 1000;
  TIM2->CCR4 = 1000;

  // PWM Mode 1 + output compare preload on all 4 channels
  TIM2->CCMR1 = (6 << 4)  | TIM_CCMR1_OC1PE    // CH1 PWM1 + preload
              | (6 << 12) | TIM_CCMR1_OC2PE;    // CH2 PWM1 + preload
  TIM2->CCMR2 = (6 << 4)  | TIM_CCMR2_OC3PE    // CH3 PWM1 + preload
              | (6 << 12) | TIM_CCMR2_OC4PE;    // CH4 PWM1 + preload

  // Enable all 4 channel outputs (active high polarity)
  TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E |
                TIM_CCER_CC3E | TIM_CCER_CC4E;

  // Enable auto-reload preload and start the counter
  TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;

  DEBUG_SERIAL.println("✓ ESC TIM2 PA0-PA3 ready @ 400 Hz");
}

// ── Write a single motor (1-indexed: 1-4) ───────────────────
void setMotor(uint8_t m, uint16_t val) {
  val = constrain(val, 1000, 2000);
  switch (m) {
    case 1: TIM2->CCR1 = val; break;
    case 2: TIM2->CCR2 = val; break;
    case 3: TIM2->CCR3 = val; break;
    case 4: TIM2->CCR4 = val; break;
  }
}

// ── Write all motors to the same value ──────────────────────
void setAllMotors(uint16_t val) {
  val = constrain(val, 1000, 2000);
  TIM2->CCR1 = val;
  TIM2->CCR2 = val;
  TIM2->CCR3 = val;
  TIM2->CCR4 = val;
}

// ── Safety: set all motors to minimum (disarmed) ────────────
void motorsOff() {
  setAllMotors(1000);
}