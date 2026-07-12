// ============================================================
//  BLOCK 5 — STATUS LEDs
//  RED   = PB12  (GPIO output, HIGH = ON)
//  GREEN = PB13  (GPIO output, HIGH = ON)
// ============================================================

#pragma once
#include "01_pins_and_includes.h"

void initLEDs() {
  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED,   LOW);
  digitalWrite(LED_GREEN, LOW);
}

// ── Set both LEDs in one call ────────────────────────────────
void setLED(bool red, bool green) {
  digitalWrite(LED_RED,   red   ? HIGH : LOW);
  digitalWrite(LED_GREEN, green ? HIGH : LOW);
}

// ── Named states used by calibration state machine ──────────
void ledIDLE()     { setLED(false, false); }   // both off
void ledGYROCAL()  { setLED(true,  false); }   // red  = gyro cal
void ledSTICKCAL() { setLED(true,  true);  }   // both = stick cal
void ledESCCAL()   { setLED(false, true);  }   // green = ESC cal
void ledDONE()     { setLED(false, true);  }   // green = success
void ledARMED()    { setLED(false, true);  }   // green = flying
void ledDISARMED() { setLED(true,  false); }   // red   = disarmed
void ledERROR()    {                           // fast red blink
  static uint32_t t = 0;
  if (millis() - t > 200) {
    t = millis();
    digitalWrite(LED_RED, !digitalRead(LED_RED));
    digitalWrite(LED_GREEN, LOW);
  }
}

// ── Boot blink — alternates red/green 3× ────────────────────
void ledStartupBlink() {
  for (uint8_t i = 0; i < 3; i++) {
    setLED(true,  false); delay(120);
    setLED(false, true);  delay(120);
  }
  setLED(false, false);
}
