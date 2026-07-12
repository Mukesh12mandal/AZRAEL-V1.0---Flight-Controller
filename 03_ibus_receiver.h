// ============================================================
//  BLOCK 3 — i-BUS RECEIVER  (USART3  RX = PB11)
//  FlySky i-BUS: 32-byte frame @ 115200 baud
//  Channels 0-9 decoded, values 1000-2000
// ============================================================

#pragma once
#include "01_pins_and_includes.h"

// ── Channel values (1000–2000) — read by blocks 6 and 7 ─────
// volatile because updateIBUS() may run from a tight polling loop
volatile uint16_t ibus_ch[IBUS_NUM_CH] = {
  1500, 1500, 1000, 1500,   // CH0=Roll CH1=Pitch CH2=Throttle CH3=Yaw
  1000, 1000, 1000, 1000,   // CH4-7 AUX
  1000, 1000                // CH8-9 AUX
};
volatile bool ibus_fresh = false;   // true for one loop after new frame

// ── Internal parser state (private to this block) ───────────
static uint8_t  ibus_buf[IBUS_BUFFSIZE];
static uint8_t  ibus_idx  = 0;
static uint32_t ibus_last = 0;

// ── Non-blocking parser — call every loop iteration ─────────
void updateIBUS() {
  while (IBUS_SERIAL.available()) {
    uint8_t  b   = IBUS_SERIAL.read();
    uint32_t now = millis();

    // Inter-frame gap > 3 ms signals start of new frame
    if (now - ibus_last > 3) ibus_idx = 0;
    ibus_last = now;

    if (ibus_idx < IBUS_BUFFSIZE) ibus_buf[ibus_idx++] = b;

    if (ibus_idx == 32) {
      // ── Header check ──────────────────────────────────────
      if (ibus_buf[0] != IBUS_SYNCBYTE1 || ibus_buf[1] != IBUS_SYNCBYTE2) {
        ibus_idx = 0;
        return;
      }

      // ── Checksum validation ───────────────────────────────
      // i-BUS checksum = 0xFFFF minus sum of bytes [0..29]
      // Stored little-endian: LSB at byte[30], MSB at byte[31]
      uint16_t chksum = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) chksum -= ibus_buf[i];
      uint16_t rx_chk = (uint16_t)ibus_buf[30] | ((uint16_t)ibus_buf[31] << 8);

      if (chksum != rx_chk) {
        ibus_idx = 0;
        return;   // corrupt frame — discard
      }

      // ── Decode channels (2 bytes each, little-endian) ─────
      // Channel data starts at byte 2
      // byte[2+i*2] = LSB,  byte[3+i*2] = MSB
      for (uint8_t i = 0; i < IBUS_NUM_CH; i++) {
        uint16_t val = (uint16_t)ibus_buf[2 + i*2]
                     | ((uint16_t)ibus_buf[3 + i*2] << 8);
        if (val >= 900 && val <= 2100) ibus_ch[i] = val;
      }

      ibus_fresh = true;
      ibus_idx   = 0;
    }
  }
}

// ── Signal validity check — used by blocks 6 and 7 ──────────
bool ibusValid() {
  return (ibus_ch[0] > 900 && ibus_ch[0] < 2100 &&
          ibus_ch[1] > 900 && ibus_ch[1] < 2100 &&
          ibus_ch[2] > 900 && ibus_ch[2] < 2100 &&
          ibus_ch[3] > 900 && ibus_ch[3] < 2100);
}

// ── Raw channel read (renamed from getStickRaw to avoid
//    confusion with getStickValue in block 6) ────────────────
uint16_t ibusGetRaw(uint8_t ch) {
  if (ch >= IBUS_NUM_CH) return 1500;
  return (uint16_t)ibus_ch[ch];
}

void initIBUS() {
  IBUS_SERIAL.begin(115200);
  DEBUG_SERIAL.println("✓ i-BUS USART3 PB11 @ 115200");
}
