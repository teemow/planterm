// The timing-critical pLAN bus path -- uart_reg_update, tx_9bit and the RX
// ISR -- lives ALONE in this translation unit, on purpose. Do not add code
// here and do not merge it back into plan_bridge.cpp.
//
// Why: the ISR answers the controller's poll inside a microsecond-scale
// window, and enrollment has broken three times (claim_mask_, ack-always,
// W3/PLANCAP 2026-07-09) after changes near -- not to -- this code. Keeping
// it in its own minimal TU makes the compiler's output for it independent
// of unrelated edits, verified 2026-07-10: growing plan_bridge.cpp with
// real code leaves this file's object byte-identical and the linked
// uart_isr disassembly and IRAM address unchanged. The "frozen shape" rule
// becomes a structural guarantee instead of a comment.
//
// (Honest caveat from the same disassembly work: W2 vs W3 uart_isr differed
// only in PlanBridge member offsets, so codegen drift alone does NOT
// explain the W3 regression. The split still removes the whole class of
// "unrelated growth shifted the ISR" suspects from every future incident.)
//
// The function bodies below are copied VERBATIM from pre-split
// plan_bridge.cpp; any edit to them needs live A/B validation on the bus
// (see the 2026-07-03 note inside uart_isr).

#include "plan_bridge.h"

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <hal/gpio_ll.h>
#include <soc/gpio_struct.h>

namespace esphome {
namespace plan_bridge {

// pLAN is a 9-bit multidrop protocol: every frame's leading address byte is
// sent with a 9th "address" bit; payload bytes are not. The wire format is
// start + 8 data + bit9 + stop. We receive it as 8E1 -- the 9th bit lands in
// the parity flag: bit9 = even_parity(data) XOR parity_error. We transmit it
// by choosing EVEN/ODD parity per byte so the emitted parity bit equals the
// desired 9th bit. This is why byte-perfect 8N2 injections were ignored: the
// controller's UART filters on the address bit, our frames never reached its
// firmware.
// The ESP32-C3 UART core latches conf0 (parity/stop) writes only when the
// UART_ID_REG.reg_update handshake bit is pulsed; without it the per-byte
// parity switch silently never reaches the shifter (loopback-verified: every
// byte went out with the boot-time EVEN parity). IDF 5.5's C3 uart_ll has no
// wrapper for it, so poke the bit directly.
static inline void IRAM_ATTR uart_reg_update(uart_dev_t *hw) {
  hw->id.update = 1;
  while (hw->id.update) {
  }
}

// Transmit with 2 stop bits: the pGD's own bytes are start+8+bit9+2 stop
// (Phase 0 read the bus cleanly as 8N2 = 12 bit-slots); a wider inter-byte
// idle can only help the controller's sampler, RX ignores extra idle.
static void IRAM_ATTR tx_9bit(uart_dev_t *hw, const uint8_t *f, size_t len, uint32_t bit9_mask) {
  uart_ll_set_stop_bits(hw, UART_STOP_BITS_2);
  uart_reg_update(hw);
  for (size_t i = 0; i < len; i++) {
    bool bit9 = (bit9_mask >> i) & 1;
    bool ones = __builtin_parity(f[i]) != 0;
    // EVEN parity emits the data-bit XOR; ODD emits its complement.
    uart_ll_set_parity(hw, (ones == bit9) ? UART_PARITY_EVEN : UART_PARITY_ODD);
    uart_reg_update(hw);
    uint8_t b = f[i];
    uart_ll_write_txfifo(hw, &b, 1);
    // The parity register must not change while a byte is shifting out, so
    // wait for TX idle between bytes (~192 us each at 62500 baud).
    while (!uart_ll_is_tx_idle(hw)) {
    }
  }
  // Restore RX framing (8E1: parity slot = bit9 detector).
  uart_ll_set_parity(hw, UART_PARITY_EVEN);
  uart_ll_set_stop_bits(hw, UART_STOP_BITS_1);
  uart_reg_update(hw);
}

// UART RX interrupt. Runs on every received byte (RX-FIFO threshold 1,
// ~6250 IRQ/s at 62500 baud -- negligible). Feeds each byte to the pure
// PlanTerminal state machine (plan_terminal.h -- the same code the host
// integration tests run against the mock controller) and, when it returns a
// transmit action, answers the response slot from inside the ISR: check the
// FIFO is still quiet, wait the turnaround, raise DE, 9-bit-transmit, drop
// DE. Reaction time is single-digit microseconds -- well ahead of the pGD's
// own ~0.4 ms turnaround -- which the buffered-UART driver path could never
// achieve.
void IRAM_ATTR PlanBridge::uart_isr(void *arg) {
  auto *self = static_cast<PlanBridge *>(arg);
  uart_dev_t *hw = self->hw_;
  uint32_t status = uart_ll_get_intsts_mask(hw);

  BaseType_t hpw = pdFALSE;
  for (;;) {
    uint32_t pending = uart_ll_get_rxfifo_len(hw);
    if (pending == 0)
      break;
    // bit9 attribution is only exact while each ISR pass finds exactly one
    // byte queued: the parity flag below is read once at entry and applied
    // to every byte of a backlog. Count backlogs so any misattribution
    // window is visible in bus10s (measured 0 in every window so far, idle
    // AND enrolled -- at 62500 baud bytes are ~176 us apart vs
    // single-digit-us ISR latency).
    if (pending > 1) {
      self->isr_multi_drain_ = self->isr_multi_drain_ + 1;
      if (pending > self->isr_drain_max_)
        self->isr_drain_max_ = pending;
    }
    // A parity "error" under 8E1 means the wire's 9th bit differs from even
    // parity of the data bits, i.e. this is an address byte (or vice versa).
    // The flag comes from the ENTRY status snapshot. Do NOT restructure this
    // (per-byte snapshot-and-clear of UART_INTR_PARITY_ERR, or hoisting perr
    // out of the loop with a post-loop drain counter): both variants
    // live-tested (2026-07-03) as a deterministic enroll->link-reset storm
    // -- the controller rejected every roll-call reply -- while this exact
    // shape enrolls cleanly. Same unexplained ISR-shape sensitivity as the
    // claim_mask_ and ack-always incidents; the multi_drain counter above is
    // the guard that the entry-snapshot simplification stays valid.
    bool perr = (status & UART_INTR_PARITY_ERR) != 0;
    uint8_t b;
    uart_ll_read_rxfifo(hw, &b, 1);
    uint8_t bit9 = static_cast<uint8_t>((__builtin_parity(b) != 0) ^ perr);

    plan::TxAction act = self->term_.on_byte(b, bit9, esp_timer_get_time());
    if (act.kind != plan::TxAction::NONE) {
      // The response slot only belongs to us if no further byte has arrived
      // behind the match (a stale match means the controller moved on) --
      // checked before AND after the turnaround delay.
      if (uart_ll_get_rxfifo_len(hw) == 0) {
        esp_rom_delay_us(self->turnaround_us_);
        if (uart_ll_get_rxfifo_len(hw) == 0) {
          gpio_ll_set_level(&GPIO, static_cast<gpio_num_t>(self->de_pin_), 1);
          uint8_t f[sizeof(act.frame)];
          for (size_t i = 0; i < act.len; i++)
            f[i] = act.frame[i];
          tx_9bit(hw, f, act.len, act.bit9_mask);
          gpio_ll_set_level(&GPIO, static_cast<gpio_num_t>(self->de_pin_), 0);
          self->term_.tx_sent(act, esp_timer_get_time());
        } else {
          self->term_.tx_not_sent(act);
        }
      } else {
        self->term_.tx_not_sent(act);
      }
    }

    uint8_t pair[2] = {b, bit9};
    // The ISR is IRAM-resident so it keeps answering polls while the flash
    // cache is disabled (WiFi/NVS/OTA writes). That is only safe if every
    // function it calls is IRAM too: the device YAML must set
    // CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH=n, otherwise this
    // stream-buffer call panics the moment the ISR fires during a flash
    // write (crash-looped v1 of the IRAM fix straight into an OTA rollback).
    if (self->stream_ != nullptr) {
      // Drop whole pairs, never half: a partial send would shift the
      // (byte, bit9) pairing for everything after it and garble the rest of
      // the capture. The only other party is the reader task, which can only
      // FREE space, so a >=2 check here guarantees the full send succeeds.
      if (xStreamBufferSpacesAvailable(self->stream_) >= 2)
        xStreamBufferSendFromISR(self->stream_, pair, 2, &hpw);
      else
        self->cap_drop_bytes_ = self->cap_drop_bytes_ + 2;
    }
  }

  uart_ll_clr_intsts_mask(hw, status);
  if (hpw == pdTRUE)
    portYIELD_FROM_ISR();
}

}  // namespace plan_bridge
}  // namespace esphome
