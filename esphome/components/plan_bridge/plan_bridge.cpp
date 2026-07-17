#include "plan_bridge.h"
#include <plan_frame.h>
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/components/network/util.h"

#include <driver/gpio.h>
#include <soc/uart_periph.h>

#include <lwip/sockets.h>

#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace esphome {
namespace plan_bridge {

static const char *const TAG = "plan_bridge";

// TxAction::Kind names for the tx diag lines (order matches the enum).
static const char *txkind_name(uint8_t k) {
  static const char *const NAMES[] = {"none", "rollcall",     "session_ack", "ident",
                                      "link", "key",          "burst_report", "race_key"};
  return k < sizeof(NAMES) / sizeof(NAMES[0]) ? NAMES[k] : "?";
}

// Depth of the pending-key-press queue. A human pressing menu buttons never
// outruns this; extra presses are dropped rather than queued unboundedly.
static const int QUEUE_DEPTH = 8;
// ISR -> logging-task stream of (byte, bit9) pairs. Sized for bursts (display
// redraws are a few hundred bytes); overflow only drops *log* bytes, never
// affects injection.
static const size_t STREAM_BUF_SIZE = 4096;
// The PLANCAP server: TCP one port above the native API. The ESPHome logger
// proved unusable as a byte transport (it drops whole lines under burst load
// and truncates long ones -- detectable, never repairable), so the raw
// (byte, bit9) stream goes out ONLY here; the logger carries nothing but
// prose for humans. Protocol: docs/capture-protocol.md (banner, Noise
// handshake / plaintext hello, record layouts); the transport session lives
// in planterm's plan_cap.h, record kinds (EV_*) in plan_bridge.h.
static const uint16_t CAP_PORT = 6054;
// Per-client backlog cap: a client that stops reading gets cut instead of
// stalling the bus task (or the other clients); the records queued-but-
// unsent consume seq numbers, so the loss is declared as a seq gap
// downstream, exactly like a dropped log line. Worst-case capture RAM is
// CAP_MAX_CLIENTS x this, and only while every client stalls at once.
static const size_t CAP_BACKLOG_MAX = 32 * 1024;
// Hold counter of a "fresh" press. Ground truth (capture 2026-07-02): a real
// pGD tap is exactly ONE keypad report with NN=0x01; NN only ramps while a
// key is held.
static const uint8_t HOLD_BASE = 0x01;
// Whether we win the response slot depends on how busy the real pGD is: while
// it is redrawing it answers polls slowly (>2 ms) and our burst fits; on a
// static idle screen it answers in ~0.4 ms and collides with us every time.
// A rejected injection resets the link, whose full-screen redraw (~2 s later)
// makes the pGD slow again -- pacing retries at 600 ms reaches into that
// window, so the first (sacrificial) attempt bootstraps acceptance.
static const int MAX_RETRIES = 10;
static const uint32_t RETRY_SPACING_MS = 600;

struct KeyReq {
  uint8_t keycode;
  bool internal;  // observe navigation: bypasses the armed gate
};

// uart_reg_update, tx_9bit and PlanBridge::uart_isr live in
// plan_bridge_isr.cpp -- ALONE, on purpose (see the header comment there).
// Keep this file ISR-free so growth here can never shift the ISR's codegen.

void PlanBridge::setup() {
  uart_config_t cfg = {};
  cfg.baud_rate = static_cast<int>(baud_rate_);
  cfg.data_bits = UART_DATA_8_BITS;
  // The wire frame is start + 8 data + address bit + stop: exactly 8E1 with
  // the address bit in the parity slot. stop_bits_ from YAML is ignored.
  cfg.parity = UART_PARITY_EVEN;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  cfg.source_clk = UART_SCLK_DEFAULT;

  // No uart_driver_install: we own the interrupt. uart_param_config still
  // enables the peripheral clock and sets baud/format.
  esp_err_t err = uart_param_config(uart_num_, &cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  int tx = (tx_pin_ >= 0) ? tx_pin_ : UART_PIN_NO_CHANGE;
  err = uart_set_pin(uart_num_, tx, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  // MAX3485 DE+RE tied, driven directly by the ISR around the 9-bit transmit.
  // RE low keeps the receiver live while idle; it is muted during our own TX,
  // so we never capture our own frame.
  gpio_reset_pin(static_cast<gpio_num_t>(de_pin_));
  gpio_set_direction(static_cast<gpio_num_t>(de_pin_), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(de_pin_), 0);

  queue_ = xQueueCreate(QUEUE_DEPTH, sizeof(KeyReq));
  stream_ = xStreamBufferCreate(STREAM_BUF_SIZE, 1);
  if (queue_ == nullptr || stream_ == nullptr) {
    ESP_LOGE(TAG, "queue/stream alloc failed");
    this->mark_failed();
    return;
  }
  // Client command records surface from each slot's sess.feed(), i.e. from
  // capture_poll_() on the bus task -- the same context every other
  // capture_* runs in. The slot index routes the ack back to the sender.
  for (size_t i = 0; i < CAP_MAX_CLIENTS; i++) {
    cap_clients_[i].sess.on_command = [this, i](uint8_t id, uint8_t op, uint8_t arg) {
      this->capture_command_(i, id, op, arg);
    };
  }

  hw_ = UART_LL_GET_HW(uart_num_);
  uart_ll_rxfifo_rst(hw_);
  uart_ll_txfifo_rst(hw_);
  // Interrupt on every byte: the poll hunt must track the bus in real time.
  uart_ll_set_rxfifo_full_thr(hw_, 1);
  uart_ll_clr_intsts_mask(hw_, UART_LL_INTR_MASK);
  // ESP_INTR_FLAG_IRAM keeps the handler runnable while the flash cache is
  // disabled (WiFi/NVS/OTA writes). Without it the ISR is blocked for
  // milliseconds at a time, we miss polls to our enrolled terminal address,
  // and the controller resets the pLAN link ("no link" flicker on the pGD).
  err = esp_intr_alloc(uart_periph_signal[uart_num_].irq, ESP_INTR_FLAG_IRAM,
                       &PlanBridge::uart_isr, this, &intr_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_intr_alloc failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  uart_ll_ena_intr_mask(hw_, UART_INTR_RXFIFO_FULL | UART_INTR_PARITY_ERR);

  // Logging/injection-bookkeeping task; all hard timing lives in the ISR.
  BaseType_t ok = xTaskCreatePinnedToCore(task_trampoline, "plan_ctrl", 4096, this,
                                          configMAX_PRIORITIES - 3, &task_, 0);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "task create failed");
    this->mark_failed();
    return;
  }
  ready_ = true;
}

void PlanBridge::press_key(uint8_t keycode) {
  if (!ready_ || queue_ == nullptr)
    return;
  KeyReq r{keycode, false};
  if (xQueueSend(queue_, &r, 0) != pdTRUE)
    ESP_LOGW(TAG, "key queue full, press 0x%02X dropped", keycode);
}

void PlanBridge::press_key_internal(uint8_t keycode) {
  if (!ready_ || queue_ == nullptr)
    return;
  KeyReq r{keycode, true};
  if (xQueueSend(queue_, &r, 0) != pdTRUE)
    ESP_LOGW(TAG, "key queue full, internal press 0x%02X dropped", keycode);
}

// Logger-only on purpose: it runs on the main loop task too (set_armed /
// set_enroll), where the bus-task-owned capture backlog is off limits --
// and the machine-readable truth is the EV_STATE event anyway.
void PlanBridge::log_state_() {
  ESP_LOGI(TAG, "state: armed=%s enroll=%s tx_mode=%d", armed_ ? "yes" : "no",
           term_.drain_ ? "drain" : (term_.enroll_ ? "yes" : "no"),
           static_cast<int>(term_.tx_mode_));
}

// Emit the current state as an EV_STATE event record. Bus task only (the
// backlog vector is single-owner); other tasks set state_dirty_ instead.
void PlanBridge::capture_event_state_() {
  uint8_t enroll = term_.drain_ ? 2 : (term_.enroll_ ? 1 : 0);
  uint8_t a = static_cast<uint8_t>((armed_ ? 1 : 0) | (enroll << 1));
  capture_event_(EV_STATE, a, static_cast<uint8_t>(term_.tx_mode_));
}

void PlanBridge::set_armed(bool a) {
  bool changed = armed_ != a;
  armed_ = a;
  // Codegen calls this before setup(); only log runtime changes.
  if (changed && ready_) {
    log_state_();
    state_dirty_ = true;  // event goes out from the bus task
  }
}

void PlanBridge::set_enroll(bool e) {
  if (e) {
    // Re-arm the join event: on a fresh enroll it fires after the roll-call
    // walk adopts us; when already on the link, the next answered poll
    // (tens of ms) re-fires it -- either way it certifies a live poll slot.
    join_polls_base_ = term_.enroll_polls_;
    join_logged_ = false;
    term_.drain_ = false;
    term_.drain_replied_ = false;
    term_.claim_mask_ = plan::OWN_BIT;
    term_.enroll_ = true;
  } else if (term_.enroll_ && !term_.drain_) {
    // Graceful leave: stay fully on the link but renounce our bit in the next
    // periodic roll-call walk (~12 s cadence). task_main finishes the leave on
    // drain_replied_ or at the deadline.
    term_.drain_replied_ = false;
    term_.claim_mask_ = 0;
    term_.drain_ = true;
    drain_deadline_ms_ = millis() + 15000;
  } else {
    term_.drain_ = false;
    term_.claim_mask_ = plan::OWN_BIT;
    term_.enroll_ = false;
  }
  if (ready_) {
    log_state_();
    state_dirty_ = true;  // event goes out from the bus task
  }
}

void PlanBridge::task_trampoline(void *arg) { static_cast<PlanBridge *>(arg)->task_main(); }

void PlanBridge::arm_isr_tx_() {
  uint8_t f[plan::REPLY9_LEN];
  plan::encode_reply9(pending_key_, hold_, f);
  for (size_t i = 0; i < plan::REPLY9_LEN; i++)
    term_.tx_frame_[i] = f[i];
  term_.tx_fired_ = false;
  term_.tx_pending_ = true;  // last: publishes the frame to the ISR
}

// Slow path only: drain the ISR's byte stream for frame logging and run the
// press bookkeeping (arm the ISR, ramp the hold counter, deadline aborts).
void PlanBridge::task_main() {
  uint8_t tmp[128];
  uint32_t last_gap_report_ = 0;
  for (;;) {
    // (The old boot-time loopback self-test is gone: with TX and RX parity
    // coupled in loopback it could only ever show the data bytes' own parity,
    // proving nothing -- and it made us deaf right when enrollment needs us
    // answering every poll.)
    // Periodic pGD-turnaround report: how fast the real display answers its
    // polls right now (drifts with its rendering workload; drives whether our
    // injected replies win the slot).
    // Finish a graceful drain: the renouncing roll-call reply went out (or
    // the walk never came before the deadline) -> actually leave the link.
    if (term_.drain_ && (term_.drain_replied_ || millis() > drain_deadline_ms_)) {
      if (term_.drain_replied_)
        vTaskDelay(pdMS_TO_TICKS(200));  // let the controller finish processing the walk
      term_.enroll_ = false;
      term_.drain_ = false;
      capture_diag_(plan::CAP_DIAG_INFO, "drain done (%s)",
                    term_.drain_replied_ ? "renounced in roll-call" : "deadline, hard stop");
      log_state_();
      state_dirty_ = true;
    }

    // State changes from other tasks (set_armed/set_enroll run on the main
    // loop task) are emitted here: the capture backlog is bus-task-only.
    if (state_dirty_) {
      state_dirty_ = false;
      capture_event_state_();
    }
    if (hold_pending_ != 0) {
      uint8_t h = hold_pending_;
      hold_pending_ = 0;
      capture_event_(EV_HOLD, static_cast<uint8_t>(h - 1), 0);
    }

    // Explicit join event (see join_logged_ in the header): the first poll
    // to our address answered since set_enroll(true) proves the controller
    // actually polls us -- the join the host waits on instead of the state
    // line's wish flag.
    if (!join_logged_ && term_.enroll_ && term_.enroll_polls_ != join_polls_base_) {
      join_logged_ = true;
      capture_diag_(plan::CAP_DIAG_INFO, "enrolled: first poll answered");
      capture_event_(EV_JOIN, 0, 0);
    }

    if (millis() - last_gap_report_ > 10000) {
      last_gap_report_ = millis();
      log_state_();
      capture_event_state_();  // periodic truth: the host watchdog's heal path
      // Phase 0 telemetry summary: per-address frame counts, checksum
      // failures (the objective garble detector), and the smallest gap
      // between our TX end and the next RX byte (collision indicator).
      // Read-then-reset per window; a byte landing between the reads is
      // counted in the next window (ponytail: no ISR lock, off-by-one-frame
      // per 10 s window is irrelevant at ~30 frames/s).
      // cap_drop_bytes_ stays MONOTONIC (the capture stream declares it
      // in-band, a reset would look like time travel); bus10s shows the
      // per-window delta like the other counters.
      uint32_t drop_now = cap_drop_bytes_;
      capture_diag_(plan::CAP_DIAG_INFO,
                    "bus10s: ctrl=%u pgd=%u us=%u other=%u cksum_fail=%u post_tx_gap_min=%uus "
                    "cap_drop=%u multi_drain=%u drain_max=%u cap_seq=%u",
                    static_cast<unsigned>(term_.tel_frames_ctrl_),
                    static_cast<unsigned>(term_.tel_frames_pgd_),
                    static_cast<unsigned>(term_.tel_frames_us_),
                    static_cast<unsigned>(term_.tel_frames_other_),
                    static_cast<unsigned>(term_.tel_cksum_fail_),
                    static_cast<unsigned>(term_.tel_post_tx_gap_min_us_),
                    static_cast<unsigned>(drop_now - drop_last_window_),
                    static_cast<unsigned>(isr_multi_drain_), static_cast<unsigned>(isr_drain_max_),
                    static_cast<unsigned>(cap_seq_));
      drop_last_window_ = drop_now;
      term_.tel_frames_ctrl_ = 0;
      term_.tel_frames_pgd_ = 0;
      term_.tel_frames_us_ = 0;
      term_.tel_frames_other_ = 0;
      term_.tel_cksum_fail_ = 0;
      term_.tel_post_tx_gap_min_us_ = 0;
      isr_multi_drain_ = 0;
      isr_drain_max_ = 0;
      if (term_.enroll_ || term_.enroll_replies_ > 0)
        capture_diag_(plan::CAP_DIAG_INFO,
                      "enroll(addr 0x%02X): %u roll-call replies, %u polls, %u session acks, "
                      "%u ident replies, %u acks withheld (cksum fail)",
                      plan::ENROLL_ADDR, static_cast<unsigned>(term_.enroll_replies_),
                      static_cast<unsigned>(term_.enroll_polls_),
                      static_cast<unsigned>(term_.session_acks_),
                      static_cast<unsigned>(term_.ident_replies_),
                      static_cast<unsigned>(term_.ack_ck_fail_));
      if (term_.gap_n_ > 0) {
        char buf[160];
        int p = 0;
        uint32_t n = term_.gap_n_ < 16 ? term_.gap_n_ : 16;
        for (uint32_t i = 0; i < n && p < (int) sizeof(buf) - 12; i++)
          p += snprintf(buf + p, sizeof(buf) - p, "%u ", static_cast<unsigned>(term_.gap_ring_[i]));
        capture_diag_(plan::CAP_DIAG_DEBUG, "pGD turnaround (us, last %u): %s",
                      static_cast<unsigned>(n), buf);
      }
    }

    // TX visibility (heatpump-firmware#14 T1b): drain the ISR's transmit log
    // into diag records. The RX-only capture never sees our own frames
    // (DE/RE tied), so this is the only record of what we put on the wire --
    // including actions we DECIDED but skipped (stale slot). DEBUG severity:
    // lands in every PLANCAP client, stays off the serial log at the default
    // level (an enrolled, focused terminal answers ~40 polls/s).
    while (txlog_r_ != term_.txlog_w_) {
      if (term_.txlog_w_ - txlog_r_ > plan::PlanTerminal::TXLOG_N) {
        capture_diag_(plan::CAP_DIAG_WARNING, "txlog: %u entries overwritten",
                      static_cast<unsigned>(term_.txlog_w_ - txlog_r_ - plan::PlanTerminal::TXLOG_N));
        txlog_r_ = term_.txlog_w_ - plan::PlanTerminal::TXLOG_N;
      }
      plan::PlanTerminal::TxLog e = term_.txlog_[txlog_r_ & (plan::PlanTerminal::TXLOG_N - 1)];
      if (term_.txlog_w_ - txlog_r_ > plan::PlanTerminal::TXLOG_N)
        continue;  // writer lapped us mid-copy; the clamp above resyncs
      txlog_r_++;
      char hex[sizeof(e.frame) * 4 + 1];
      int p = 0;
      for (uint8_t i = 0; i < e.len && i < sizeof(e.frame); i++)
        p += snprintf(hex + p, sizeof(hex) - static_cast<size_t>(p), "%02X%s ", e.frame[i],
                      ((e.bit9 >> i) & 1) ? "'" : "");
      capture_diag_(plan::CAP_DIAG_DEBUG, "tx %s %s @%lluus: %s", e.sent ? "sent" : "skip",
                    txkind_name(e.kind), static_cast<unsigned long long>(e.us), hex);
    }

    capture_poll_();
    size_t n = xStreamBufferReceive(stream_, tmp, sizeof(tmp), pdMS_TO_TICKS(gap_ms_));
    if (n > 0) {
      capture_send_(tmp, n);
      // Screen-snapshot cache (slow path, never the ISR): remember the
      // latest valid display frame per (terminal, row) for replay to the
      // next capture client. The observe listener rides the same drain.
      uint32_t now = millis();
      for (size_t i = 0; i + 1 < n; i += 2) {
        snap_.feed(tmp[i], tmp[i + 1], now);
        if (stream_listener_)
          stream_listener_(tmp[i], tmp[i + 1], now);
      }
    }
    capture_flush_();

    uint32_t now = millis();
    if (pending_frames_ == 0) {
      KeyReq req;
      if (xQueueReceive(queue_, &req, 0) != pdTRUE)
        continue;
      if (!armed_ && !req.internal) {
        capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X ignored: not armed", req.keycode);
        continue;
      }
      capture_diag_(plan::CAP_DIAG_INFO, "injecting key 0x%02X: %d poll-slot(s)", req.keycode,
                    repeat_);
      pending_key_ = req.keycode;
      pending_frames_ = repeat_;
      hold_ = HOLD_BASE;
      term_.isr_stale_ = 0;
      retries_ = 0;
      term_.tx_rejected_ = false;
      term_.link_reset_ = false;
      inject_deadline_ms_ = now + 2000;
      arm_isr_tx_();
      continue;
    }

    if (term_.tx_fired_) {
      term_.tx_fired_ = false;
      capture_diag_(
          plan::CAP_DIAG_DEBUG,
          "TX(9bit) %02X' %02X %02X %02X %02X %02X %02X  %02X' %02X %02X %02X (stale polls: "
          "%u, attempt %d)",
          term_.tx_frame_[0], term_.tx_frame_[1], term_.tx_frame_[2], term_.tx_frame_[3],
          term_.tx_frame_[4], term_.tx_frame_[5], term_.tx_frame_[6], term_.tx_frame_[7],
          term_.tx_frame_[8], term_.tx_frame_[9], term_.tx_frame_[10],
          static_cast<unsigned>(term_.isr_stale_), retries_);
      capture_event_(EV_TX_FIRED, pending_key_, static_cast<uint8_t>(retries_));
      // Verdict phase. Two rejection signatures exist: an immediate re-poll
      // (tx_rejected_, within ms) and the silent discard, which surfaces only
      // as the controller's link-reset FF-walk ~2 s later. Only 2.5 s of
      // quiet after the TX means the key was accepted.
      //
      // In tx_mode 2 the report rides in OUR OWN poll slot: no pGD to collide
      // with and (measured 2026-07-02, dozens of injections) no silent
      // discards -- every key was accepted on attempt 0. The rejection
      // re-poll lands within ~5 ms of our TX (the ISR flags it), so two
      // 20 ms ticks cover it with margin and the accepted verdict logs
      // ~40 ms after the TX instead of 300 ms.
      bool failed = false;
      int verdict_ticks = (term_.tx_mode_ == 2) ? 2 : 25;
      uint32_t tick_ms = (term_.tx_mode_ == 2) ? 20 : 100;
      for (int i = 0; i < verdict_ticks; i++) {
        vTaskDelay(pdMS_TO_TICKS(tick_ms));
        if (term_.tx_rejected_ || term_.link_reset_) {
          failed = true;
          break;
        }
      }
      if (failed) {
        bool was_reset = term_.link_reset_;
        term_.tx_rejected_ = false;
        term_.link_reset_ = false;
        if (++retries_ <= MAX_RETRIES) {
          capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X: %s, retry %d/%d", pending_key_,
                        was_reset ? "link reset (silent discard)" : "collision with pGD reply",
                        retries_, MAX_RETRIES);
          // A link reset is followed by a full-screen redraw that keeps the
          // pGD busy (slow poll replies) -- the retry lands in that window.
          vTaskDelay(pdMS_TO_TICKS(was_reset ? 400 : RETRY_SPACING_MS));
          inject_deadline_ms_ = millis() + 2000;
          arm_isr_tx_();
          continue;
        }
        capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X: gave up after %d attempts",
                      pending_key_, MAX_RETRIES);
        pending_frames_ = 0;
        continue;
      }
      hold_ = (hold_ + 2 > 0xC8) ? 0xC8 : static_cast<uint8_t>(hold_ + 2);
      if (--pending_frames_ == 0) {
        capture_diag_(plan::CAP_DIAG_INFO, "key 0x%02X: accepted (attempt %d)", pending_key_,
                      retries_);
        capture_event_(EV_KEY_ACCEPTED, pending_key_, static_cast<uint8_t>(retries_));
      } else {
        // Space repeats at roughly the pGD's cadence, then re-arm.
        vTaskDelay(pdMS_TO_TICKS(repeat_interval_ms_));
        arm_isr_tx_();
      }
    } else if ((int32_t) (now - inject_deadline_ms_) >= 0) {
      // No poll slot could be filled (the pGD beat us into every one).
      // That costs nothing on the bus, so retry on the same budget.
      if (++retries_ <= MAX_RETRIES) {
        capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X: no clean slot, retry %d/%d",
                      pending_key_, retries_, MAX_RETRIES);
        inject_deadline_ms_ = now + 2000;
        continue;
      }
      term_.tx_pending_ = false;
      capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X: aborted (%d slot(s) unfilled, stale polls: %u)",
                    pending_key_, pending_frames_, static_cast<unsigned>(term_.isr_stale_));
      pending_frames_ = 0;
    }
  }
}

// --- the PLANCAP server (TCP 6054) -------------------------------------------
//
// Wire protocol: docs/capture-protocol.md. plan::PlanCapSession handles the
// transport (banner, mode selection, Noise NNpsk0 handshake or the keyless
// plaintext hello, frame + record encryption); this section owns the
// sockets, the record CONTENT (bus bytes, events, diagnostics, acks) and
// the backlog policy.
//
// A fresh client gets the screen-snapshot replay plus one EV_STATE as soon
// as its session is established, so it starts with truth instead of waiting
// for the 10 s periodic state event.
//
// Up to CAP_MAX_CLIENTS concurrent clients (the ekobeescope TUI plus a
// one-shot command, with a slot to spare), each with its own handshake,
// snapshot replay and backlog; records fan out to all of them. All slots
// taken -> the oldest client is evicted (the old single-client "newest
// wins" recovery property, per slot). All sockets are non-blocking: the bus
// task must never wait on the network. Everything here runs on the bus task
// ONLY (the client slots and cap_rec_ are single-owner); other tasks signal
// via state_dirty_/hold_pending_.

static void cap_set_nonblock(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }

void PlanBridge::capture_close_client_(CapClient &c) {
  if (c.fd >= 0) {
    close(c.fd);
    c.fd = -1;
  }
  c.kick = false;
  c.sess.reset();
  c.backlog.clear();
  c.backlog.shrink_to_fit();  // a stall grows this to ~32 KiB; give it back
}

bool PlanBridge::capture_established_() const {
  for (const auto &c : cap_clients_)
    if (c.fd >= 0 && !c.kick && c.sess.established())
      return true;
  return false;
}

// Enqueue the record built in cap_rec_ as one framed -- and on a keyed
// session encrypted -- unit of wire bytes for one client. No-op until that
// session is established (the session refuses records mid-handshake).
bool PlanBridge::capture_out_(CapClient &c) {
  return c.sess.send_record(cap_rec_.data(), cap_rec_.size(), c.backlog);
}

// cap_rec_ (one typed record: event, diagnostic or ack) to every
// established client. Typed records are never dropped (spec 6.3): a client
// whose backlog is past the cap is kicked instead -- deferred to
// capture_poll_, because this fan-out can run inside a client's own
// sess.feed() (command -> diag) where an inline close would reset the
// session mid-parse.
void PlanBridge::capture_fanout_typed_() {
  for (auto &c : cap_clients_) {
    if (c.fd < 0 || c.kick || !c.sess.established())
      continue;
    if (c.backlog.size() > CAP_BACKLOG_MAX) {
      ESP_LOGW(TAG, "capture client stalled (backlog %u), dropping client",
               static_cast<unsigned>(c.backlog.size()));
      c.kick = true;
      continue;
    }
    capture_out_(c);
  }
}

void PlanBridge::capture_poll_() {
  if (cap_listen_fd_ < 0) {
    // The bus task starts at HARDWARE setup priority, long before WiFi:
    // touching lwIP sockets before the TCP/IP stack is up crashes the boot
    // (measured: illegal-instruction panic + OTA rollback). Wait for an IP.
    if (!network::is_connected())
      return;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0)
      return;  // out of sockets right now; retried next loop
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(CAP_PORT);
    if (bind(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0 ||
        listen(fd, 2) < 0) {
      close(fd);
      return;
    }
    cap_set_nonblock(fd);
    cap_listen_fd_ = fd;
    ESP_LOGI(TAG, "capture stream listening on tcp/%u", CAP_PORT);
  }
  // Deferred closes first: kicks set by the typed fan-out, possibly from
  // inside a client's own feed() last round.
  for (auto &c : cap_clients_) {
    if (c.kick)
      capture_close_client_(c);
  }
  int fd = accept(cap_listen_fd_, nullptr, nullptr);
  if (fd >= 0) {
    // A free slot, or evict the oldest client when all are taken -- a
    // wedged client can never lock up the port.
    CapClient *slot = nullptr;
    for (auto &c : cap_clients_) {
      if (c.fd < 0) {
        slot = &c;
        break;
      }
    }
    if (slot == nullptr) {
      slot = &cap_clients_[0];
      for (auto &c : cap_clients_) {
        if (static_cast<int32_t>(c.opened_ms - slot->opened_ms) < 0)
          slot = &c;
      }
      ESP_LOGW(TAG, "capture slots full, evicting the oldest client");
      capture_close_client_(*slot);
    }
    cap_set_nonblock(fd);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    slot->fd = fd;
    slot->opened_ms = millis();
    // Banner out; the client's first frame selects the mode (Noise
    // handshake on a keyed server, plaintext hello on a keyless one).
    slot->sess.begin(cap_has_psk_ ? cap_psk_.data() : nullptr, slot->backlog);
    ESP_LOGI(TAG, "capture client connected (%s)",
             cap_has_psk_ ? "awaiting noise handshake" : "awaiting plaintext hello");
  }
  // Drain each client's bytes: handshake frames and command records. EOF or
  // a read error means that client is gone.
  uint8_t buf[128];
  for (auto &c : cap_clients_) {
    if (c.fd < 0)
      continue;
    for (;;) {
      ssize_t r = recv(c.fd, buf, sizeof(buf), MSG_DONTWAIT);
      if (r > 0) {
        bool was_established = c.sess.established();
        if (!c.sess.feed(buf, static_cast<size_t>(r), c.backlog)) {
          // Fatal protocol/handshake error (spec section 7): best-effort
          // flush of the queued explicit handshake-error frame, then close.
          capture_flush_client_(c);
          ESP_LOGW(TAG, "capture client rejected (handshake/protocol error), dropping");
          capture_close_client_(c);
          break;
        }
        if (!was_established && c.sess.established()) {
          capture_send_snapshot_(c);
          // State truth up front, not 10 s later. Broadcast: the other
          // clients just see one extra periodic-style state event.
          capture_event_state_();
          ESP_LOGI(TAG, "capture client established (%s, screen snapshot replayed)",
                   c.sess.keyed() ? "encrypted" : "plaintext");
        }
        if (r == static_cast<ssize_t>(sizeof(buf)))
          continue;  // more may be buffered
        break;
      }
      if (r == 0 || (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) {
        ESP_LOGI(TAG, "capture client gone");
        capture_close_client_(c);
      }
      break;
    }
  }
}

// Replay the screen-snapshot cache right after the banner: one seq-0 record
// per cached frame (seq 0 marks "replayed state, not live traffic" -- live
// seq starts at 1, so the client's gap accounting is untouched). ts_ms
// carries when the frame was actually seen. 0x20 frames get the constant
// pGD-ack trailer appended, which the client's frame parser anchors on.
void PlanBridge::capture_send_snapshot_(CapClient &c) {
  snap_.visit([this, &c](const uint8_t *bytes, size_t len, uint32_t at_ms, bool trailer) {
    size_t np = len + (trailer ? sizeof(plan::SNAP_TRAILER) : 0);
    cap_rec_.clear();
    plan::cap_rec_pairs_begin(cap_rec_, 0, at_ms, cap_drop_bytes_, static_cast<uint16_t>(np));
    for (size_t i = 0; i < len; i++) {
      cap_rec_.push_back(bytes[i]);
      cap_rec_.push_back(i == 0 ? 1 : 0);  // bit9 on the address byte
    }
    if (trailer) {
      for (size_t i = 0; i < sizeof(plan::SNAP_TRAILER); i++) {
        cap_rec_.push_back(plan::SNAP_TRAILER[i]);
        cap_rec_.push_back(i == 0 ? 1 : 0);
      }
    }
    capture_out_(c);
  });
}

void PlanBridge::capture_send_(const uint8_t *pairs, size_t n) {
  if (!capture_established_())
    return;
  size_t npairs = n / 2;
  if (npairs == 0)
    return;
  cap_seq_++;  // one number per record, shared by every client
  cap_rec_.clear();
  plan::cap_rec_pairs_begin(cap_rec_, cap_seq_, millis(), cap_drop_reported_ = cap_drop_bytes_,
                            static_cast<uint16_t>(npairs));
  cap_rec_.insert(cap_rec_.end(), pairs, pairs + 2 * npairs);
  for (auto &c : cap_clients_) {
    if (c.fd < 0 || c.kick || !c.sess.established())
      continue;
    if (c.backlog.size() > CAP_BACKLOG_MAX) {
      // This client stalled: skip its copy but seq counted the record, so
      // it sees an honest gap instead of silently thinned data. The other
      // clients keep receiving -- one slow reader stalls only itself.
      continue;
    }
    capture_out_(c);
  }
}

// One typed event record, fanned out. Bus task only. A backlog past the cap
// means that client stalled long ago (pair records stopped at the cap);
// with the device scraping 24/7 the per-keypress events would then grow the
// heap without bound, so cut the client instead -- a reconnect replays the
// snapshot plus one EV_STATE, so no state truth is lost.
void PlanBridge::capture_event_(uint8_t kind, uint8_t a, uint8_t b) {
  if (!capture_established_())
    return;
  cap_rec_.clear();
  plan::cap_rec_event(cap_rec_, millis(), kind, a, b);
  capture_fanout_typed_();
}

// Plan-related prose: to the logger for humans, and as a PLANCAP diagnostic
// record for every capture client (severity plan::CAP_DIAG_*). Diagnostics
// are advisory prose -- anything a machine must react to is an event or an
// ack, never parsed out of this text. Bus task only (same single-owner
// rule as capture_event_, same stalled-client policy).
void PlanBridge::capture_diag_(uint8_t severity, const char *fmt, ...) {
  char buf[192];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0)
    return;
  size_t len = (n < static_cast<int>(sizeof(buf))) ? static_cast<size_t>(n) : sizeof(buf) - 1;
  switch (severity) {
    case plan::CAP_DIAG_ERROR:
      ESP_LOGE(TAG, "%s", buf);
      break;
    case plan::CAP_DIAG_WARNING:
      ESP_LOGW(TAG, "%s", buf);
      break;
    case plan::CAP_DIAG_DEBUG:
      ESP_LOGD(TAG, "%s", buf);
      break;
    default:
      ESP_LOGI(TAG, "%s", buf);
      break;
  }
  if (!capture_established_())
    return;
  cap_rec_.clear();
  plan::cap_rec_diag(cap_rec_, millis(), severity, buf, len);
  capture_fanout_typed_();
}

// A client command record: dispatch into the same internals the ESPHome
// services call, then ack exactly once (spec section 5.5) -- to the issuing
// client only. Runs inside that client's sess.feed(), i.e. on the bus task.
// Commands from concurrent clients need no extra serialization: they all
// arrive on the one bus task in socket-read order, arm/enroll are
// last-writer-wins with the resulting EV_STATE broadcast to everyone, and
// key injections funnel into the existing bounded FreeRTOS queue (a full
// queue is an honest rejected ack).
void PlanBridge::capture_command_(size_t ci, uint8_t id, uint8_t op, uint8_t arg) {
  uint8_t status = plan::CAP_ACK_OK;
  switch (op) {
    case plan::CAP_CMD_ARM:
      set_armed(arg != 0);
      break;
    case plan::CAP_CMD_ENROLL:
      set_enroll(arg != 0);
      break;
    case plan::CAP_CMD_INJECT_KEY: {
      // The same gates press_key() + task_main enforce, but with the
      // verdict known synchronously so the ack can be honest.
      if (!armed_) {
        status = plan::CAP_ACK_REJECTED;
        capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X rejected: not armed", arg);
        break;
      }
      KeyReq r{arg, false};
      if (!ready_ || queue_ == nullptr || xQueueSend(queue_, &r, 0) != pdTRUE) {
        status = plan::CAP_ACK_REJECTED;
        capture_diag_(plan::CAP_DIAG_WARNING, "key 0x%02X rejected: queue full", arg);
      }
      break;
    }
    default:
      status = plan::CAP_ACK_UNKNOWN_OP;
      break;
  }
  // capture_diag_ above may have kicked the issuing client (stalled); the
  // ack is then moot -- the client is about to be dropped.
  CapClient &c = cap_clients_[ci];
  if (c.kick)
    return;
  cap_rec_.clear();
  plan::cap_rec_ack(cap_rec_, millis(), id, status);
  capture_out_(c);
}

bool PlanBridge::capture_flush_client_(CapClient &c) {
  if (c.fd < 0 || c.backlog.empty())
    return true;
  ssize_t w = send(c.fd, c.backlog.data(), c.backlog.size(), MSG_DONTWAIT);
  if (w > 0) {
    c.backlog.erase(c.backlog.begin(), c.backlog.begin() + w);
    return true;
  }
  if (w < 0 && (errno == EWOULDBLOCK || errno == EAGAIN))
    return true;
  ESP_LOGW(TAG, "capture stream send failed (errno %d), dropping client", errno);
  capture_close_client_(c);
  return false;
}

void PlanBridge::capture_flush_() {
  for (auto &c : cap_clients_) {
    if (!c.kick)  // a kicked client is closed on the next capture_poll_
      capture_flush_client_(c);
  }
}

void PlanBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "pLAN control (9-bit ISR key-press injection):");
  ESP_LOGCONFIG(TAG, "  UART%d  RX=%d  TX=%d  DE/RE(GPIO)=%d", static_cast<int>(uart_num_),
                rx_pin_, tx_pin_, de_pin_);
  ESP_LOGCONFIG(TAG, "  %u baud, 8+bit9 (as 8E1), gap %ums", baud_rate_, gap_ms_);
  ESP_LOGCONFIG(TAG, "  repeat %dx @ %ums, armed=%s, PLANCAP tcp/%u (%s)", repeat_,
                repeat_interval_ms_, armed_ ? "yes" : "no", CAP_PORT,
                cap_has_psk_ ? "encrypted" : "plaintext: no key configured");
  ESP_LOGCONFIG(TAG, "  enroll=%s (terminal 0x%02X), tx_mode=%d", term_.enroll_ ? "yes" : "no",
                plan::ENROLL_ADDR, static_cast<int>(term_.tx_mode_));
}

#if defined(PLAN_CAP_NOISE) && !defined(USE_API_NOISE)
// noise-c's RNG hook. The encrypted ESPHome API component provides it when
// present; a keyed bridge on a device without an encrypted API supplies its
// own (same implementation, HWRNG-backed).
extern "C" void noise_rand_bytes(void *output, size_t len) {
  if (!random_bytes(reinterpret_cast<uint8_t *>(output), len)) {
    ESP_LOGE(TAG, "Acquiring random bytes failed; rebooting");
    arch_restart();
  }
}
#endif

}  // namespace plan_bridge
}  // namespace esphome
