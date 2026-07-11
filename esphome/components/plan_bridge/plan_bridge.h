#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

// The pure pLAN protocol logic comes from the planterm library
// (github.com/teemow/planterm, pinned in __init__.py). PLAN_CAP_NOISE is a
// global build flag set by codegen when a PLANCAP key is configured; it
// turns on the Noise responder inside plan_cap.h (noise-c arrives via
// add_library).
#include <plan_cap.h>
#include <plan_snapshot.h>
#include <plan_terminal.h>

#include <driver/uart.h>
#include <hal/uart_ll.h>
#include <esp_intr_alloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <freertos/task.h>
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace esphome {
namespace plan_bridge {

// Typed device events on the capture stream (PLANCAP record type 0x01,
// docs/capture-protocol.md section 5.2) -- the machine interface host tools
// consume instead of regex-parsing log prose. The ESP_LOG lines stay for
// humans; these values are the wire protocol.
static const uint8_t EV_STATE = 1;         // a = armed | enroll<<1 (0 no, 1 yes, 2 drain), b = tx_mode
static const uint8_t EV_JOIN = 2;          // first poll to our address answered since set_enroll(true)
static const uint8_t EV_TX_FIRED = 3;      // a = keycode, b = attempt
static const uint8_t EV_KEY_ACCEPTED = 4;  // a = keycode, b = attempt
static const uint8_t EV_HOLD = 5;          // plan_observe's walk yielded; a = 1 paused, 0 armed

// Phase 3 bus participation + key-press injection for the CAREL pLAN /pGD
// terminal-emulation project. Unlike the Phase 0/2 read-only components, this
// one *transmits*: it puts a 6-byte keypad-report frame (1E 07 20 KK NN CC) on
// the bus to impersonate the pGD's keypad, so a menu key-press can be driven
// from Home Assistant. Pressing a key on any shared CAREL terminal moves the
// menu on all of them, so a single injected frame moves the real pGD's screen.
//
// It also serves the raw RX stream on a TCP port (the lossless capture
// stream, see plan_bridge.cpp) so the very redraw a key-press triggers can
// be captured on the same device and reconstructed into the on-screen text
// -- a self-contained closed loop: inject a key, watch the screen change,
// no second device needed.
//
// Transmit path (v2: interrupt-driven, no IDF UART driver):
//   The pGD is a bit-synchronous slave: the controller sends its 5-byte poll
//   (20 01 01 DD 01) and the terminal must start its keypad reply within
//   ~1 byte-time (~176 us at 62500 baud) or the controller fills the slot with
//   its own idle tail. Hardware tests proved the buffered-UART path (driver
//   ISR -> ring buffer -> FreeRTOS task -> uart_write_bytes) cannot make that
//   window even with the RX-FIFO threshold at 1 byte -- every injection landed
//   on the controller's tail and only disrupted the link. So this component
//   bypasses the driver entirely:
//   * uart_param_config/uart_set_pin do the pin/baud setup, then a custom RX
//     interrupt handler (RX-FIFO threshold = 1) matches the poll pattern
//     byte-by-byte as each byte arrives.
//   * On match with a press pending, the ISR itself raises the MAX3485 DE
//     GPIO and writes the 6-byte frame straight into the TX FIFO -- single-
//     digit-microsecond reaction, inside the grant window. TX_DONE drops DE.
//     DE+RE are tied, so our receiver is muted during TX (no self-echo).
//   * Received bytes stream to the logging task through a FreeRTOS stream
//     buffer; frame reassembly/logging is unchanged from the capture harness.
//
// Safety: transmit is gated behind an `armed` flag (default false); a key
// press requested while disarmed is dropped. A press responds to at most
// `repeat` polls (spaced by repeat_interval_ms) and self-aborts after a
// deadline if polls stop, so it can never hold the bus indefinitely.
class PlanBridge : public Component {
 public:
  void set_tx_pin(int p) { tx_pin_ = p; }
  void set_rx_pin(int p) { rx_pin_ = p; }
  void set_de_pin(int p) { de_pin_ = p; }
  void set_baud_rate(uint32_t b) { baud_rate_ = b; }
  void set_uart_num(int n) { uart_num_ = static_cast<uart_port_t>(n); }
  void set_stop_bits(int s) { stop_bits_ = s; }
  void set_gap_ms(uint32_t g) { gap_ms_ = g; }
  void set_repeat(int r) { repeat_ = r; }
  void set_repeat_interval_ms(uint32_t m) { repeat_interval_ms_ = m; }
  void set_armed(bool a);
  bool armed() const { return armed_; }
  bool enrolled() const { return term_.enroll_; }
  // Actual link join: a poll to our address was answered since the last
  // set_enroll(true) (the "enrolled: first poll answered" event) -- keys
  // have a live poll slot to ride in. enrolled() is only the wish flag.
  bool joined() const { return term_.enroll_ && join_logged_; }
  // Controller FF-walk recovery seen (observe uses it to re-enroll). Sticky
  // until the next injection attempt clears it in task_main.
  bool link_reset() const { return term_.link_reset_; }
  // Observe hook: a second consumer of the drained (byte, bit9) stream,
  // called from the bus task at the same site that feeds the screen-snapshot
  // cache (task context, never the ISR -- no IRAM constraints).
  void set_stream_listener(std::function<void(uint8_t byte, uint8_t bit9, uint32_t at_ms)> l) {
    stream_listener_ = std::move(l);
  }
  // plan_observe's walk-yield handshake as an EV_HOLD event. Any task: the
  // bus task drains the flag into the stream (coalescing back-to-back posts
  // is harmless -- the host waits for "one more hold event").
  void post_hold_event(bool paused) { hold_pending_ = paused ? 2 : 1; }
  // Poll-end -> reply-start delay. Runtime-tunable to probe the acceptance
  // window (controller listening threshold vs. the pGD's own reply time).
  void set_turnaround_us(uint32_t us) { turnaround_us_ = us; }
  // Enable/disable answering the controller's roll-call for a free terminal
  // address. Disabling starts the graceful drain (see PlanTerminal::drain_);
  // task_main finishes the leave.
  void set_enroll(bool e);
  // 0 = race the pGD for the poll response slot (11-byte full reply),
  // 1 = send the 7-byte keypad report after the pGD's own poll exchange,
  // 2 = inject in our enrolled terminal's poll slot (needs enroll).
  void set_tx_mode(int m) { term_.tx_mode_ = m; }
  // The PLANCAP PSK (raw 32 bytes, from base64 in YAML -- on ESPHome devices
  // codegen defaults it to api.encryption.key). Called by codegen only when
  // a key is configured (which also sets the PLAN_CAP_NOISE build flag);
  // without a key the capture socket serves the keyless plaintext mode.
  void set_capture_psk(std::array<uint8_t, 32> psk) {
    cap_psk_ = psk;
    cap_has_psk_ = true;
  }

  // Enqueue a single key press. Safe to call from loop()/the API/a YAML
  // lambda -- it just hands a keycode to the bus task via a FreeRTOS queue and
  // returns immediately.
  void press_key(uint8_t keycode);
  // Same, but bypasses the armed gate: observe's scrape navigation presses
  // keys 24/7 while the "write enable" switch keeps gating only HA/API
  // presses (which go through press_key).
  void press_key_internal(uint8_t keycode);

  void setup() override;
  void dump_config() override;
  // Bring the UART up before sensors/api.
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  // Defined in plan_bridge_isr.cpp, a translation unit deliberately holding
  // ONLY the timing-critical path, so edits to the capture/session code can
  // never shift the ISR's codegen (see the header comment there). Do not
  // move it back into plan_bridge.cpp.
  static void uart_isr(void *arg);
  static void task_trampoline(void *arg);
  void task_main();
  void arm_isr_tx_();    // encode pending_key_/hold_ into tx_frame_ and hand it to the ISR
  void log_state_();     // one "state: armed=... enroll=..." line (human prose)
  // The PLANCAP server (TCP 6054, docs/capture-protocol.md): accept the
  // single client, run the session (handshake, record framing/encryption
  // via plan::PlanCapSession), flush non-blocking. All bus task only.
  void capture_poll_();                                  // listen/accept/read/handshake
  void capture_send_(const uint8_t *pairs, size_t n);    // one bus-bytes record
  bool capture_flush_();                                 // false = client died
  void capture_close_client_();
  void capture_send_snapshot_();  // replay the screen cache as seq-0 records
  bool capture_out_();  // enqueue cap_rec_ as one framed (encrypted) record
  // Typed event records (PLANCAP type 0x01). Bus task only.
  void capture_event_(uint8_t kind, uint8_t a, uint8_t b);
  void capture_event_state_();  // EV_STATE from the current armed/enroll/tx_mode
  // Plan-related prose: one line to the logger (for humans at `esphome
  // logs`) AND, when a capture client is established, as a PLANCAP
  // diagnostic record (type 0x02) -- host tools never parse the logger.
  // Severity = plan::CAP_DIAG_* (mapped to the matching log level).
  void capture_diag_(uint8_t severity, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
  // A client command record (type 0x03): dispatch into the same internals
  // the ESPHome services call, then ack (type 0x04).
  void capture_command_(uint8_t id, uint8_t op, uint8_t arg);

  int tx_pin_{-1};
  int rx_pin_{-1};
  int de_pin_{-1};  // MAX3485 DE+RE tied; plain GPIO, driven by the ISR
  uint32_t baud_rate_{62500};
  uart_port_t uart_num_{UART_NUM_1};
  int stop_bits_{2};
  uint32_t gap_ms_{3};                // stream-drain tick: capture record boundary
  int repeat_{3};                     // keypad frames sent per single press
  uint32_t repeat_interval_ms_{60};   // spacing between those frames
  volatile bool armed_{false};

  QueueHandle_t queue_{nullptr};
  TaskHandle_t task_{nullptr};
  StreamBufferHandle_t stream_{nullptr};
  intr_handle_t intr_{nullptr};
  uart_dev_t *hw_{nullptr};
  bool ready_{false};

  // The PLANCAP server: a single-client TCP server (port 6054), the ONLY
  // transport for the raw (byte, bit9) stream -- the logger carries prose,
  // never bytes. Every drained chunk goes out as one record with a sequence
  // number and the in-band ISR-drop count: TCP is reliable, so the only
  // loss modes left (stream-buffer overflow, a stalled client cut off) are
  // declared in the data instead of vanishing. Wire bytes (framed, and on a
  // keyed session Noise-encrypted, by cap_sess_) queue in cap_backlog_ and
  // are flushed non-blocking so a stalled client can never stall the bus
  // task.
  int cap_listen_fd_{-1};
  int cap_client_fd_{-1};
  uint32_t cap_seq_{0};
  volatile uint32_t cap_drop_bytes_{0};  // bytes the ISR could not stream (buffer full)
  uint32_t cap_drop_reported_{0};        // cap_drop_bytes_ already declared in-band
  uint32_t drop_last_window_{0};         // cap_drop_bytes_ at the last bus10s report
  std::vector<uint8_t> cap_backlog_;     // encoded records awaiting send
  // bit9-attribution instrumentation: the entry-status parity flag is only
  // exact while each ISR pass finds exactly one byte queued (see uart_isr).
  // These count loop iterations that found a >=2-byte backlog (attribution
  // ambiguous there) and the deepest backlog, reported per bus10s window.
  volatile uint32_t isr_multi_drain_{0};
  volatile uint32_t isr_drain_max_{0};
  // set_armed/set_enroll run on the main loop task; the capture backlog is
  // bus-task-only, so they raise this flag and task_main emits the EV_STATE.
  // Coalescing is fine: the event carries the CURRENT state.
  volatile bool state_dirty_{false};
  // Pending EV_HOLD posted by plan_observe (0 none, 1 armed, 2 paused).
  volatile uint8_t hold_pending_{0};

  // The pure pLAN terminal-side protocol state machine (poll matching,
  // enrollment, session acks, rejection/link-reset detection). ISR <-> task
  // handshake happens through its volatile fields (single-core: the task
  // fully writes term_.tx_frame_ before setting term_.tx_pending_; the ISR
  // consumes the frame on the poll match and flips tx_pending_ -> tx_fired_).
  // Integration-tested on the host against a mock controller in planterm.
  plan::PlanTerminal term_;

  // Screen-snapshot cache: latest valid display frame per (terminal, row),
  // fed by the capture task and replayed to a fresh capture client so
  // planscope never starts from a blank screen (planterm plan_snapshot.h).
  plan::PlanSnapshot snap_;

  // Second consumer of the drained stream (plan_observe's screen model).
  std::function<void(uint8_t, uint8_t, uint32_t)> stream_listener_;

  volatile uint32_t turnaround_us_{250};  // delay from poll end to reply start

  // Active injection state (owned by the task): a press responds to up to
  // `pending_frames_` polls, ramping the hold counter, spaced by
  // repeat_interval_ms, and aborts at inject_deadline_ms.
  int pending_frames_{0};
  uint8_t pending_key_{0};
  uint8_t hold_{0};
  int retries_{0};  // collision retries used for the current press
  uint32_t inject_deadline_ms_{0};
  // Hard stop for the graceful drain: if no roll-call walk renounced us by
  // then (walks come ~every 12 s), drop the link the old abrupt way.
  uint32_t drain_deadline_ms_{0};
  // Explicit join event: log "enrolled: first poll answered" once per
  // set_enroll(true), when the first poll to our address after it was
  // answered -- the ACTUAL link join (planscope's arm()/refresh() wait on
  // it), as opposed to the enroll=yes wish flag in the state line. Written
  // by set_enroll (main loop task) and task_main; a race only duplicates or
  // pre-fires the log line, which the host treats as "link live" either way.
  volatile uint32_t join_polls_base_{0};
  volatile bool join_logged_{true};

  // LAYOUT RULE (live-bisected 2026-07-11): new PlanBridge members go HERE,
  // at the END of the class -- never above. Inserting members above term_
  // (shifting the offsets of cap_seq_..join_logged_) deterministically
  // breaks pLAN enrollment even with byte-identical code (bisect-pad: 80 B
  // of dead padding above cap_seq_ broke it; bisect-endpad: the same 80 B
  // here was fine). Mechanism still unidentified -- treat the offsets of
  // everything above this line as frozen, like the ISR's shape.
  plan::PlanCapSession cap_sess_;
  std::vector<uint8_t> cap_rec_;  // scratch: one plaintext record being built
  std::array<uint8_t, 32> cap_psk_{};
  bool cap_has_psk_{false};
};

}  // namespace plan_bridge
}  // namespace esphome
