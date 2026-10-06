#pragma once

// Pure, ESP-free pLAN terminal-side protocol state machine (Phase 3).
//
// This is the byte-driven logic that used to live inline in PlanControl's UART
// RX interrupt: poll matching, roll-call enrollment, session-frame acking,
// link-reset / rejection detection, and the decision *what* to transmit into a
// response slot. Extracted here so the exact code the ISR runs can be
// integration-tested on the host against a mock controller
// (test/mock_controller.h, test/test_integration.cpp) built from the real bus
// captures.
//
// Split of responsibilities:
//   PlanTerminal (this file, pure)     -- byte stream in, TxAction out, flags.
//   PlanControl ISR (plan_control.cpp) -- hardware glue only: FIFO-empty
//     checks, the turnaround delay, the DE GPIO, the 9-bit transmit, and
//     reporting back via tx_sent()/tx_not_sent().
//
// Time is injected (now_us) so the rejection window and the pGD-turnaround
// probe stay testable. All task-visible fields are volatile, matching the
// single-core ISR<->task handshake the component always used.

#include "plan_frame.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>

// The ISR calling into this code is IRAM-resident so it keeps running while
// the flash cache is disabled (WiFi/NVS/OTA writes). Everything it touches --
// these methods and the poll constant -- must therefore live in IRAM/DRAM on
// the ESP. On the host the attributes compile away.
#ifdef ESP_PLATFORM
#include <esp_attr.h>
#define PLAN_IRAM IRAM_ATTR
#define PLAN_DRAM DRAM_ATTR
#else
#define PLAN_IRAM
#define PLAN_DRAM
#endif

namespace plan {

// pLAN terminal address we enroll at (31; the real pGD sits at 32). Proven
// live 2026-07-02 12:20: answering the roll-call with the membership bitmap
// bit for 31 set made the controller rebroadcast the map as C0 00 00 01 ...
// and start polling address 1F' 01 01 DE.
static constexpr uint8_t ENROLL_ADDR = 0x1F;
// Our bit in the roll-call membership bitmaps (address 32 = bit7 of byte 0
// down to address 1; 31 -> bit6 of byte 0 = 0x40) and its byte index.
static constexpr uint8_t OWN_BYTE_I = (32 - ENROLL_ADDR) / 8;
static constexpr uint8_t OWN_BIT = 1u << (7 - ((32 - ENROLL_ADDR) % 8));

// The controller's poll to the pGD is the 4 bytes 20' 01 01 DD; the bus then
// goes silent for the response slot and the terminal answers.
static PLAN_DRAM const uint8_t POLL4[4] = {0x20, 0x01, 0x01, 0xDD};

// CRC-16/Modbus, one byte at a time (reflected poly 0xA001, init 0xFFFF, no
// final xor). Its residue property: running it over a whole frame INCLUDING
// the little-endian trailer yields 0 -- the streaming check the session-ack
// gate uses. Bit-banged so there is no flash-resident lookup table to fault
// the IRAM ISR; 8 shifts/byte is nothing at 6250 bytes/s.
static inline uint16_t PLAN_IRAM crc16_modbus_step(uint16_t crc, uint8_t b) {
  crc = static_cast<uint16_t>(crc ^ b);
  for (int i = 0; i < 8; i++)
    crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
  return crc;
}

// The pGD's answer to the controller's type-identification request (ground
// truth, cold-boot attach capture 2026-07-02: 20' 50 05 01 89 -> 01' 51 07 20
// 0A 17 65). The 2-byte payload is presumably terminal type + version; we
// echo the pGD's exact bytes because a pCO only sessions terminals of the
// SAME type (CAREL: "the controller cannot manage different kinds of
// terminals at the same time").
static constexpr uint8_t IDENT_REQ_TYPE = 0x50;
static PLAN_DRAM const uint8_t PGD_IDENT[2] = {0x0A, 0x17};

// Graphic/session frame types whose trailer is CRC-16/Modbus little-endian
// over the rest of the frame, NOT the classic sum-to-0xFF check byte
// (ground truth: planscope's parser, brute-forced from live captures).
static constexpr uint8_t GRAPHIC_TYPE = 0x64;
static constexpr uint8_t SESSION_INIT_TYPE = 0x65;
static constexpr uint8_t SESSION_CTL_TYPE = 0x66;

// What the ISR should put on the wire right now (into the response slot that
// just opened). len == 0 means nothing to send for this byte.
//
// Deliberately NO default member initializers / aggregate zero-init: on_byte
// constructs one of these per received byte inside the IRAM ISR, and GCC
// lowers a 16-byte {}-init into a memcpy from a .rodata zero block -- a
// flash-cache access that faults the moment the ISR fires during a WiFi/NVS/
// OTA flash write (crash + OTA rollback, seen live 2026-07-02). on_byte sets
// kind/len/bit9_mask explicitly; frame[] is only read up to len.
struct TxAction {
  enum Kind : uint8_t {
    NONE = 0,
    ROLLCALL_REPLY,    // enrollment: echo the roll-call with our bit claimed
    SESSION_ACK,       // enrollment: ack a session frame (01' 03 ADDR CK)
    IDENT_REPLY,       // enrollment: answer a 0x50 ident request (01' 51 ...)
    ENROLL_LINK_REPLY, // enrollment: idle answer to our own poll slot
    ENROLL_KEY_REPLY,  // enrollment: keypad report + link reply in our slot
    AFTER_BURST_REPORT,// tx_mode 1: standalone report after the pGD's burst
    RACE_KEY_REPLY,    // tx_mode 0: race the pGD for the 0x20 response slot
    FORWARD_POLL,      // dual-terminal: hand our poll token on to the pGD@32
    LINK_RESEND,       // A6 T8 / R-LL-05: identical resend of an unacked link reply (on_tick)
  };
  Kind kind;
  uint8_t len;
  uint8_t frame[12];
  uint16_t bit9_mask;  // bit i set => frame[i] carries the 9th/address bit
};

// --- Wave E5: bus-health telemetry --------------------------------------
// One bus10s window, latched by the task (PlanTerminal::take_window) so
// plan_bridge prints it AND keeps it for the Home Assistant getters. Every
// field is measured while passive too (the bridge only listens), except
// ack1f/ra1f and tx/hold_dev/tx_early_idle, which need our own transmissions.
// Healthy reads (July / A8 sect. 4 healthy reference) in the comments.
struct BusWindow {
  uint32_t walks;            // FF-walks (= the old walks=), healthy 0
  uint32_t walks_warm;       // ... of them warm (claims carried); cold = walks - walks_warm
  uint32_t ack20, ra20;      // controller session frames the pGD acked / then resent anyway
  uint32_t ack1f, ra1f;      // the same for our own acks (0 while passive)
  uint32_t pgd_rc;           // pGD-as-master roll-call frames XX' 02 20 (it lost the controller), 0
  uint32_t ctrl_frames;      // frames the controller sent; 0 = a silent controller
  uint32_t ctrl_sil;         // controller silences >= 1 s that ended in the window, 0
  uint32_t ctrl_gap_max_ms;  // longest controller silence, the still-open one included, ~25-50
  uint32_t paint_age_s;      // since the controller last painted 0x20, < 60
  uint32_t joins, joins_ok;  // our joins opened / closed (as rollcall10s)
  uint32_t addr_bad;         // 9th-bit bytes > 0x20 (no such address): parity garble, 0
  uint32_t uart_frm;         // UART receive errors (raw ISR status): framing, 0
  uint32_t uart_brk;         // ... break detect, 0
  uint32_t uart_glitch;      // ... glitch detect, 0
  uint32_t tx;               // our frames on the wire (uart_isr path)
  int32_t hold_dev_min_us;   // DE hold minus (len x char + overhead + de_tail); 0 if tx == 0
  int32_t hold_dev_max_us;   // ... healthy about 0 .. +20 (A7 3.2); < -96 = a frame cut short
  uint32_t tx_early_idle;    // bytes whose TX-idle read came before one char time (A7 F1), 0
  uint8_t health;            // BUS_HEALTHY / BUS_DEGRADED / BUS_LOOP
};
enum : uint8_t { BUS_HEALTHY = 0, BUS_DEGRADED = 1, BUS_LOOP = 2 };

// `ekobeescope health` thresholds (ekobeescope#46, internal/plan/bushealth.go):
// LOOP at >= 3 FF-walks at >= 1 per 10 s, or a controller that sent nothing;
// DEGRADED at any pGD roll-call, a controller silence >= 1 s, > 15 % resent
// after the ack (over >= 30 acked frames), >= 3 own joins with none closed, or
// a pGD not painted for a minute. The two rate rules run over the trailing
// HEALTH_SPAN windows (30 s): 3 walks in 30 s IS ">= 3 at >= 1 per 10 s", and
// a 2-4 walks/10 s loop then reads LOOP in every window instead of flapping.
static constexpr uint32_t HEALTH_LOOP_WALKS = 3;
static constexpr uint32_t HEALTH_SILENCE_MS = 1000;
static constexpr uint32_t HEALTH_MIN_ACKED = 30;
static constexpr uint32_t HEALTH_MAX_RESEND_PCT = 15;
static constexpr uint32_t HEALTH_MIN_JOINS = 3;
static constexpr uint32_t HEALTH_PAINT_S = 60;
static constexpr int HEALTH_SPAN = 3;
// DE hold of an n-byte frame = n x 192 us + 16 us (A7 1.4: 2320 us for every
// 12-byte frame in 65 windows): the overhead is DE lead + tail + timer reads.
static constexpr uint32_t TX_HOLD_OVERHEAD_US = 16;

// The trailing-span sums the rate rules need.
struct BusSpan {
  uint32_t walks, ack20, ra20, ack1f, ra1f;
};

inline bool resend_over(uint32_t ra, uint32_t acked) {
  return acked >= HEALTH_MIN_ACKED && ra * 100u > acked * HEALTH_MAX_RESEND_PCT;
}

inline uint8_t bus_health(const BusWindow &w, const BusSpan &s) {
  if (s.walks >= HEALTH_LOOP_WALKS || w.ctrl_frames == 0)
    return BUS_LOOP;
  if (w.pgd_rc > 0 || w.ctrl_gap_max_ms >= HEALTH_SILENCE_MS || resend_over(s.ra20, s.ack20) ||
      resend_over(s.ra1f, s.ack1f) || (w.joins >= HEALTH_MIN_JOINS && w.joins_ok == 0) ||
      w.paint_age_s >= HEALTH_PAINT_S)
    return BUS_DEGRADED;
  return BUS_HEALTHY;
}

// The bus-health fields appended to bus10s (leading space included). One
// formatter for the firmware and the host tests, which pin its worst-case
// length against plan_bridge's diag buffer. Returns snprintf's length.
inline int format_bus_window(char *out, size_t n, const BusWindow &w) {
  // health first: should a line ever be cut at the buffer, the tail goes.
  return snprintf(out, n,
                  " health=%u ra20=%u ack20=%u ra1f=%u ack1f=%u pgd_rc=%u ctrl_tx=%u ctrl_sil=%u "
                  "ctrl_gap=%ums paint_age=%us warm=%u frm=%u brk=%u glitch=%u addr_bad=%u tx=%u "
                  "hold_dev_min=%dus hold_dev_max=%dus early_idle=%u",
                  static_cast<unsigned>(w.health), static_cast<unsigned>(w.ra20),
                  static_cast<unsigned>(w.ack20), static_cast<unsigned>(w.ra1f), static_cast<unsigned>(w.ack1f),
                  static_cast<unsigned>(w.pgd_rc), static_cast<unsigned>(w.ctrl_frames),
                  static_cast<unsigned>(w.ctrl_sil), static_cast<unsigned>(w.ctrl_gap_max_ms),
                  static_cast<unsigned>(w.paint_age_s), static_cast<unsigned>(w.walks_warm),
                  static_cast<unsigned>(w.uart_frm), static_cast<unsigned>(w.uart_brk),
                  static_cast<unsigned>(w.uart_glitch), static_cast<unsigned>(w.addr_bad),
                  static_cast<unsigned>(w.tx), static_cast<int>(w.hold_dev_min_us),
                  static_cast<int>(w.hold_dev_max_us), static_cast<unsigned>(w.tx_early_idle));
}

// Frame types a terminal acks with 01' 03 T CK (ekobeescope sessionType).
static inline bool PLAN_IRAM session_type(uint8_t t) {
  return (t >= 0x0A && t <= 0x0F) || (t >= 0x60 && t <= 0x6F);
}

class PlanTerminal {
 public:
  // --- task -> state machine (mirrors the old PlanControl volatiles) ---
  volatile bool enroll_{false};        // answer the roll-call for ENROLL_ADDR
  // Graceful-disenroll drain: keep the link fully alive (polls answered,
  // sessions acked -- the poll path is untouched by design; gating it was in
  // the 2026-07-03 00:11 deploy that broke enrollment outright) but RENOUNCE
  // our membership bit in both halves of the next periodic roll-call walk
  // (~12 s cadence), so the controller removes us without the missed-poll
  // link fault -> FF-walk -> pGD "NO LINK" flash of a hard stop. The task
  // finishes the leave on drain_replied_ or at a deadline.
  volatile bool drain_{false};
  volatile bool drain_replied_{false}; // renouncing roll-call reply went out
  // Our membership bit for the roll-call reply, precomputed by the TASK
  // (set_enroll): OWN_BIT normally, 0x00 while draining. The ISR applies it
  // branch-free: frame = (frame & ~OWN_BIT) | claim_mask_. Live bisect
  // 2026-07-03 08:44-09:05: an if(drain_) branch in the claim path stormed
  // the bus on every enroll (controller adopts us, then 2 s silence +
  // FF-walk loop) even though the emitted bytes were identical, while the
  // branchless builds enrolled fine. Cause not understood (the disassembly
  // diff is only the 4-instruction branch); shape kept as close as possible
  // to the proven builds.
  volatile uint8_t claim_mask_{OWN_BIT};
  // 0 = race the pGD's 0x20 slot, 1 = report after the pGD's burst,
  // 2 = inject in our enrolled terminal's poll slot (needs enroll).
  volatile int tx_mode_{2};
  volatile bool tx_pending_{false};    // task set a frame to inject
  volatile uint8_t tx_frame_[REPLY9_LEN]{0};

  // --- state machine -> task ---
  volatile bool tx_fired_{false};
  volatile bool tx_rejected_{false};   // controller re-polled right after our TX
  volatile bool link_reset_{false};    // controller's FF-walk recovery seen
  volatile uint32_t enroll_replies_{0};
  volatile uint32_t enroll_polls_{0};
  volatile uint32_t session_acks_{0};
  volatile uint32_t ident_replies_{0};
  // Session frames addressed to us that completed byte-count-wise but FAILED
  // their checksum: we stay silent instead of acking. Live A/B 2026-07-03
  // REFUTED the resend hypothesis: the controller does not resend an unacked
  // frame, it goes ~2 s silent and FF-walk-resets the link (fault-injection
  // build withholding every 5th ack looped enroll->reset continuously). The
  // gate is kept anyway: (a) real garble is measured-zero on a healthy bus
  // (tel_cksum_fail_ = 0 across all soak windows), (b) a single garbled
  // frame -> one reset -> the recovery walk re-enrolls us and the session
  // re-init repaints the full screen, which IS the self-heal, just heavier,
  // and (c) the ack-always restructure inexplicably broke enrollment on a
  // healthy bus -- same unexplained ISR-shape sensitivity as claim_mask_
  // (byte-identical output, different outcome), so the live-proven shape
  // stays. This counter attributes any such reset to wire garble.
  volatile uint32_t ack_ck_fail_{0};
  volatile uint32_t isr_stale_{0};     // poll matched but bytes were already behind it

  // pGD turnaround probe: poll-end -> first-reply-byte gaps (us), ring of 16.
  volatile uint32_t gap_ring_[16]{0};
  volatile uint32_t gap_n_{0};

  // --- Phase 0 telemetry (ISR increments, task reads + resets per report) ---
  // A frame is one bit9-delimited run of bytes. Classic multi-byte pLAN
  // frames byte-sum to 0xFF (single-byte acks carry no checksum; CRC-16
  // types 0x64/65/66 are exempt -- see on_byte), so a failing sum means
  // corruption on the wire -- the objective "garble" detector. Frames
  // are classified by their leading address byte: controller (0x01), the pGD
  // (0x20), our enrolled address, everything else.
  volatile uint32_t tel_frames_ctrl_{0};
  volatile uint32_t tel_frames_pgd_{0};
  volatile uint32_t tel_frames_us_{0};
  volatile uint32_t tel_frames_other_{0};
  volatile uint32_t tel_cksum_fail_{0};
  // Minimum gap (us) between the end of OUR transmission and the next RX
  // byte in this window: a near-zero minimum means someone transmitted on
  // top of / hard against our frame (collision indicator). 0 = no TX seen.
  volatile uint32_t tel_post_tx_gap_min_us_{0};

  // Feed one received byte (with its recovered 9th bit). Returns the transmit
  // action this byte triggers, if any. The caller is responsible for the
  // FIFO-empty / turnaround-delay gating and must report the outcome via
  // tx_sent() or tx_not_sent().
  // ponytail: at most one action is returned per byte; the old ISR could in
  // principle fire two handlers on one byte, but only on garbage input that
  // matches two frame grammars at once -- never on a real bus.
  TxAction PLAN_IRAM on_byte(uint8_t b, uint8_t bit9, int64_t now_us) {
    // Field-by-field init, NOT `TxAction act{}`: the aggregate zero-init of
    // the 16-byte struct compiles to a memcpy from .rodata (flash), which
    // faults in this IRAM ISR during flash-cache-off windows. See TxAction.
    TxAction act;
    act.kind = TxAction::NONE;
    act.len = 0;
    act.bit9_mask = 0;

    // Phase 0 telemetry -- collision indicator: gap between the end of our
    // own transmission and the very next byte someone else puts on the wire.
    if (tel_last_tx_us_ != 0) {
      uint32_t g = static_cast<uint32_t>(now_us - tel_last_tx_us_);
      if (tel_post_tx_gap_min_us_ == 0 || g < tel_post_tx_gap_min_us_)
        tel_post_tx_gap_min_us_ = g;
      // planterm#47 reject signal: the MAX gap in the window and a count of our
      // transmissions the controller did not follow within 100 ms. In steady
      // enrolled operation every reply is acked within ~0.4 ms and the next
      // poll is ~25 ms out, so any gap past 100 ms means the controller went
      // silent right after our TX -- a rejected reply heading into the 2 s link
      // timeout and FF-walk. Our own wire is invisible (RE is muted while we
      // drive DE), so this ack/no-ack gap is the only on-device accept signal.
      if (g > tel_post_tx_gap_max_us_)
        tel_post_tx_gap_max_us_ = g;
      if (g > 100000u)
        tel_tx_unacked_ = tel_tx_unacked_ + 1;
      tel_last_tx_us_ = 0;
    }

    // Link-reply resend (wave E2, A6 T8 / R-LL-05): any byte on the wire
    // after our link reply ends its resend window -- the controller's 01'
    // accepted it, or the bus moved on (its re-poll) and a copy would collide.
    if (lr_rs_n_ != 0)
      lr_rs_n_ = 0;

    // Key fate (A5 R-KP-11/12): the FIRST byte after our key burst decides
    // the exchange. The controller acks with a bare 01' ~0.4 ms after the
    // burst; a re-poll of us (1F') or anything else first means it did not
    // take the burst, and silence is judged by the task (KEY_VERDICT_MS). A
    // walk marker seen before the burst no longer counts against it: the
    // verdict is per TX, never a sticky flag.
    if (key_await_) {
      key_await_ = false;
      key_ack_ = (bit9 != 0 && b == 0x01 && now_us - tx_done_us_ < 5000) ? 1 : 2;
    }

    // Phase 0 telemetry -- frame accounting. Frames are delimited by the
    // bit9/address byte: a new one closes the previous run, which is then
    // classified by destination address and checksum-validated (classic
    // multi-byte pLAN runs byte-sum to 0xFF; the keypad reply too, since the
    // leading 01' plus the sum-to-0xFE report gives 0xFF; the single-byte 01'
    // ack carries no checksum). Graphic/session types 0x64/65/66 carry a
    // CRC-16 trailer instead and never sum to 0xFF -- they are skipped here,
    // NOT counted as garble (proven 2026-07-16: 331 phantom "failures" on a
    // host-verified 0-failure bus, all repaint traffic; the ack gate below
    // has always known both grammars). A failing sum on the remaining types =
    // corruption on the wire, the objective "garble" detector. Our own TX
    // never reaches this path (RE is muted while we drive DE), so these
    // counters see only the others.
    if (bit9 != 0) {
      if (tel_active_) {
        if (tel_len_ > 1 && tel_sum_ != 0xFF && tel_type_ != GRAPHIC_TYPE &&
            tel_type_ != SESSION_INIT_TYPE && tel_type_ != SESSION_CTL_TYPE)
          tel_cksum_fail_ = tel_cksum_fail_ + 1;
        // pGD-liveness probe: any terminal->controller frame from 0x20
        // (link reply 01 01 20 DD, ring reply 01 02 20 .., ack 01 03 20 DB)
        // proves the pGD@32 transmitted -- gates the ring-token forward.
        if (tel_addr_ == 0x01 && tel_len_ >= 4 && tel_b3_ == 0x20)
          t_pgd_alive_us_ = now_us;
        if (tel_addr_ == 0x01)
          tel_frames_ctrl_ = tel_frames_ctrl_ + 1;
        else if (tel_addr_ == 0x20)
          tel_frames_pgd_ = tel_frames_pgd_ + 1;
        else if (tel_addr_ == ENROLL_ADDR)
          tel_frames_us_ = tel_frames_us_ + 1;
        else
          tel_frames_other_ = tel_frames_other_ + 1;
        rc_run_end_(now_us);
        hl_run_end_();
      } else if (!rc_init_) {
        rc_init_ = true;  // boot: presume a pGD may exist (see pgd_absent_now_)
        t_pgd_tx_us_ = now_us;
      }
      if (lr_ack_wait_ == 1)
        lr_ack_wait_ = 2;  // this run is the first one after our link reply
      tel_active_ = true;  // (joining mid-frame at boot: first partial run is skipped)
      tel_addr_ = b;
      tel_sum_ = b;
      tel_len_ = 1;
      // Wave E5: a 9th bit on a byte that cannot be an address (pLAN uses
      // 0x00-0x20) is a parity-flipped data byte -- the one kind of parity
      // garble the 9th-bit scheme can tell from a real address byte.
      if (b > 0x20)
        tel_addr_bad_ = tel_addr_bad_ + 1;
      tel_hash_ = (2166136261u ^ b) * 16777619u;  // FNV-1a over the run (resend match)
      tel_start_us_ = now_us;
      tel_run_sacked_ = false;
    } else if (tel_active_) {
      if (tel_len_ == 1)
        tel_type_ = b;  // frame type = first byte after the address byte
      else if (tel_len_ == 2)
        tel_b3_ = b;  // third byte: the sender in terminal->controller frames
      else if (tel_len_ == 3)
        tel_b4_ = b;  // fourth byte: the sender in LEN (session) frames
      else if (tel_len_ == 7)
        tel_c1_ = b;  // roll-call: first CLAIMS byte (bit7 = address 32)
      tel_sum_ = static_cast<uint8_t>(tel_sum_ + b);
      tel_len_ = tel_len_ + 1;
      tel_hash_ = (tel_hash_ ^ b) * 16777619u;
    }

    // Rolling windows.
    isr_win_[0] = isr_win_[1];
    isr_win_[1] = isr_win_[2];
    isr_win_[2] = isr_win_[3];
    isr_win_[3] = isr_win_[4];
    isr_win_[4] = b;

    // Poll-chain completion: after our FORWARD_POLL, the pGD answers the
    // token AS ITSELF -- 01' 01 20 DD, the mirror of a poll to 20, exactly
    // as if the controller had polled it directly (live 09:37:17-09:39:00,
    // 89 forwards -> 89 replies, zero faults; fwd-experiment-0937.txt).
    // The mirror-of-original variant (01' 01 1F DE) is kept accepted too.
    // Byte-pattern match like the other window matchers; only armed for the
    // ~one poll cycle after a forward, so paint-data false positives are
    // irrelevant.
    if (fwd_awaiting_ && isr_win_[1] == 0x01 && isr_win_[2] == 0x01 &&
        ((isr_win_[3] == 0x20 && isr_win_[4] == 0xDD) ||
         (isr_win_[3] == ENROLL_ADDR &&
          isr_win_[4] == static_cast<uint8_t>(0xFF - 0x01 - 0x01 - ENROLL_ADDR)))) {
      fwd_awaiting_ = false;
      fwd_ok_ = fwd_ok_ + 1;
    }

    // FF-walk detector: the first frame of the controller's recovery walk is
    // 02' 02 01 FF FF FF FF C1 C2 C3 C4 CC. Cold walks carry claims 00..,
    // WARM walks the claims carried over from a pGD master walk (80 ..; A1
    // R-RC-07, A2 R-LL-11: 57 of 120 walks on 10-05). rc_walk_any_claims_ = 1
    // (default) matches any claims at the C1 byte, anchored on the 02'
    // address byte (no pixel false positive: data bytes never carry bit9),
    // and FOLDS the lone announce probe into its restart 1.76-2.2 s later
    // (R-RC-06: an opener whose previous frame was an opener is one recovery)
    // -- the same detector as ekobeescope's plan.WalkCounter (#48).
    // rc_walk_any_claims_ = 0: the legacy cold-only match (A2 D8 / A1 V1).
    reset_win_[0] = reset_win_[1];
    reset_win_[1] = reset_win_[2];
    reset_win_[2] = reset_win_[3];
    reset_win_[3] = reset_win_[4];
    reset_win_[4] = reset_win_[5];
    reset_win_[5] = reset_win_[6];
    reset_win_[6] = reset_win_[7];
    reset_win_[7] = b;
    const bool opener =
        rc_walk_any_claims_
            ? (tel_active_ && tel_addr_ == 0x02 && tel_len_ == 8 && reset_win_[1] == 0x02 &&
               reset_win_[2] == 0x01 && reset_win_[3] == 0xFF && reset_win_[4] == 0xFF &&
               reset_win_[5] == 0xFF && reset_win_[6] == 0xFF)
            : (reset_win_[0] == 0x02 && reset_win_[1] == 0x01 && reset_win_[2] == 0xFF &&
               reset_win_[3] == 0xFF && reset_win_[4] == 0xFF && reset_win_[5] == 0xFF &&
               reset_win_[6] == 0x00 && reset_win_[7] == 0x00);
    if (opener) {
      link_reset_ = true;
      const bool fold = rc_walk_any_claims_ && walk_prev_ &&
                        static_cast<uint64_t>(now_us - t_link_reset_us_) <= 3'000'000ull;
      t_link_reset_us_ = now_us;  // ring-forwarding lockout window
      walk_run_ = true;
      pgd_sess_ = false;  // every walk rebuilds the sessions (R-SE-10)
      rc_last_ok_ = false;  // a roll-call after this is a probe, never a confirm
      if (fold) {
        tel_walks_folded_ = tel_walks_folded_ + 1;
      } else {
        tel_walks_ = tel_walks_ + 1;  // planterm#47: FF-walks seen this window
        if (rc_walk_any_claims_ && b != 0x00)
          tel_walks_warm_ = tel_walks_warm_ + 1;
        if (join_open_)
          rc_join_failed_(now_us);  // our join died in this walk (R-RC-13/R-LL-16)
      }
    }

    // Session-ready gate (A5 R-KP-07, F-03): a key sent in the first polls
    // after a walk dies with the session (7 of 9 measured). Any recovery
    // walk -- cold or warm, i.e. 02' 02 01 FF FF FF FF whatever claims it
    // carries -- closes the gate; so do the controller's session-init frames
    // to us (below); the first display row of the fresh session opens it.
    // Address-bit anchored, so pixel data cannot match.
    if (bit9 != 0)
      walk_m_ = (b == 0x02) ? 1 : 0;
    else if (walk_m_ != 0)
      walk_m_ = (b == (walk_m_ == 1 ? 0x02 : walk_m_ == 2 ? 0x01 : 0xFF)) ? walk_m_ + 1 : 0;
    if (walk_m_ == 7) {
      walk_m_ = 0;
      sess_ready_ = false;
    }

    // Terminal enrollment: the roll-call is a TOKEN RING, not a
    // controller-polls-everyone sweep (ground truth 2026-07-16, live ring
    // with a pGD at 0x1E: 1E' 02 01 -> 1F' 02 1E -> 20' 02 1E -> 01' 02 1E).
    // Frame format <to>' 02 <from> <8-byte payload> CK: each live member
    // receives the token, claims its bit, and FORWARDS it to the next ring
    // member -- back to the controller (0x01) only when nothing live sits
    // above it. The old matcher required <from> == 0x01 and so dropped every
    // token forwarded by another terminal (83 ignored invitations in one
    // 15-min capture = the un-rejoinable "zombie" state); the old reply
    // always terminated the ring at us, cutting members above us (the pGD
    // at 0x20) out of every walk. Accept any sender, forward per the map.
    if (enroll_) {
      if (bit9 != 0) {
        rc_state_ = (b == ENROLL_ADDR) ? 1 : 0;
      } else if (rc_state_ == 1) {
        rc_state_ = (b == 0x02) ? 2 : 0;
      } else if (rc_state_ == 2) {
        rc_from_ = b;  // token sender: the controller or any ring member
        rc_state_ = 3;
      } else if (rc_state_ >= 3 && rc_state_ <= 10) {
        rc_payload_[rc_state_ - 3] = b;
        rc_state_ = rc_state_ + 1;
      } else if (rc_state_ == 11) {
        rc_state_ = 0;
        uint8_t s = static_cast<uint8_t>(ENROLL_ADDR + 0x02 + rc_from_);
        for (int i = 0; i < 8; i++)
          s += rc_payload_[i];
        if (static_cast<uint8_t>(s + b) == 0xFF && rc_from_ != 0x01 && !rc_member_tokens_) {
          // A2 D7 / A6 T13 (wave E2, parent decision): a real terminal ignores
          // member-forwarded tokens -- the pGD ignored our 20' 02 1F (GT
          // 07-17 07:43) and every `1F' 02 20 ..` reaching us is a probe of
          // the pGD's own master walk (R-LL-18), not a ring the controller
          // runs. Answering it puts our claim into a frame the controller
          // never asked for and keeps the pGD's walk alive. Silent; the
          // controller's own walk (or its take-over of the pGD's token, A6
          // T17) is where we join. rc_member_tokens_ = 1: the legacy answer.
          tel_rc_member_ign_ = tel_rc_member_ign_ + 1;
        } else if (static_cast<uint8_t>(s + b) == 0xFF) {
          // Which controller frame is this? A CONFIRM re-sends our own last
          // reply's masks (R-RC-09/13) with no FF-walk in between; anything
          // else from 0x01 is a PROBE that would open a join. (Bytes, not
          // time: a warm walk can carry exactly our last masks, C0/C0, but
          // always after its opener.) An FF-walk probe carries the walker's
          // optimistic MAP: our own bit AND 32's (not yet probed). A gap-walk
          // probe never carries our bit (members are never probed, R-RC-03),
          // nor does the member-loss re-probe carry a dropped 32.
          const bool ctrl = rc_from_ == 0x01;
          bool confirm = rc_last_ok_;
          for (int i = 0; i < 8; i++)
            confirm = confirm && rc_payload_[i] == rc_last_[i];
          const bool absent = pgd_absent_now_(now_us);
          if (ctrl && !confirm) {
            // R-RC-30 (A1 V3): never end an FF-walk at 31 while a pGD may
            // exist -- the first accepted answer ends the walk (R-RC-09), so
            // our answer would leave CLAIMS = {31} and 0x20 unprobed. Stay
            // silent: the walk reaches 0x20, the pGD claims, and we join on
            // the controller's next gap walk (<= 12 s, C0/C0, R-RC-14), the
            // path of every July dual join. Exception: the walker's own
            // probe of 0x20 went unanswered and 0x20 stayed silent for
            // pgd_absent_us_ (no pGD): then answer, or the walk loops forever.
            if (rc_ff_silent_ && (rc_payload_[0] & OWN_BIT) && (rc_payload_[0] & 0x80) &&
                !absent) {
              tel_rc_silent_ = tel_rc_silent_ + 1;
              return act;
            }
            // Join back-off (root cause 3): after rc_backoff_after_ own joins
            // in a row died before their first link reply was acked, stay
            // silent on join-opening probes for a growing interval -- but
            // only while the loop keeps running (an FF-walk within one
            // gap-walk period): on a calm bus the next gap-walk probe is
            // answered again. Polls, acks and confirms are never gated.
            if (rc_backoff_ && now_us < rc_backoff_until_us_ &&
                static_cast<uint64_t>(now_us - t_link_reset_us_) < 12'000'000ull) {
              tel_rc_backoff_ = tel_rc_backoff_ + 1;
              return act;
            }
          }
          rc_opens_join_ = ctrl && !confirm;
          act.kind = TxAction::ROLLCALL_REPLY;
          act.len = 12;
          // Forward the token to a LIVE member above us (a pGD at 0x20),
          // else return it to the controller (0x01). The gate is TRUE
          // liveness -- the pGD transmitted within 15 s (t_pgd_alive_us_,
          // fed by the telemetry probe) -- not walk type: an FF-walk-based
          // lockout dead-locked the live-32 topology (2026-07-17 00:48:
          // recovery walks NEED the forward when 32 is real; returning to
          // 0x01 reports a cut ring, the controller faults and FF-walks
          // forever, the lockout never expires). An EMPTY 32 never
          // transmits, so the blind-forward loop of 2026-07-16 stays
          // impossible.
          // GROUND TRUTH 2026-07-17 07:43 (real pGD downstream of a forward,
          // first ever observation): a real terminal IGNORES member-forwarded
          // tokens -- it ignored our 20' 02 1F and answered the controller's
          // own 20' 02 01 eight ms later. Forwarding a controller token
          // therefore THROWS OUR CLAIM AWAY (the controller never hears it,
          // re-walks 20 itself, adopts the pGD, drops us). Tokens ALWAYS
          // return to their sender's side: controller tokens to 0x01 (the
          // proven 21:21:05 join), member tokens to 0x01 as before.
          bool pgd_live = t_pgd_alive_us_ != 0 &&
                          static_cast<uint64_t>(now_us - t_pgd_alive_us_) < 15'000'000ull;
          act.frame[0] = 0x01;
          // 32's presence bit. A real terminal echoes it verbatim (A2 D2);
          // only a walker clears the bit of an address that did not answer.
          // Default (rc_honest_skip_ = 0): clear it only when the WALKER did
          // exactly that -- its probe of 0x20 went unanswered and 0x20 stayed
          // silent for pgd_absent_us_ -- so our reply reports what the walk
          // itself established, and never because of a liveness timeout: the
          // old 15 s honest skip cut a present pGD out of MAP, and nothing
          // re-offers 32 once it is out (R-RC-22/R-RC-40, the 10-02 latch).
          // rc_honest_skip_ = 1: the legacy liveness skip (CONTEXT finding 5).
          if (rc_honest_skip_ ? !pgd_live : absent)
            rc_payload_[0] = static_cast<uint8_t>(rc_payload_[0] & ~0x80);
          act.frame[1] = 0x02;
          act.frame[2] = ENROLL_ADDR;
          for (int i = 0; i < 8; i++)
            act.frame[3 + i] = rc_payload_[i];
          // The 8-byte payload is TWO 4-byte fields (ground truth 2026-07-02
          // evening, live A/B tested): the established/probe map first, the
          // CLAIMS field second (address 32 = bit7 of byte 0 down to address
          // 1 = bit0 of byte 3 in each half; the pGD claims 80 00 00 00).
          // Halves are SENDER-CONDITIONAL (live 2026-07-16 evening):
          // - Controller-sent token (rc_from_ == 0x01): assert our bit in
          //   BOTH halves -- the 07-02 A/B ground truth (map-half only ->
          //   polled but never sessioned; claims-half only -> accumulates
          //   but never polled) and the accepted 00:58:55 join.
          // - Member-forwarded token: presence VERBATIM, claim CLAIMS-half
          //   only -- the pGD's own observed cold join (19:39:02: presence
          //   80 echoed untouched, claims 40->C0). Asserting ourselves into
          //   the presence half of a member-forwarded token made the
          //   controller link-fault on every reply (2 s silence -> FF-walk,
          //   43 resets in 3 min) even with the reply returned to 0x01: the
          //   forwarding chain owns presence accounting, a joiner may only
          //   add its claim.
          // While DRAINING, the task sets claim_mask_ = 0 and these lines
          // RENOUNCE instead. Branch shape kept minimal; see claim_mask_.
          uint8_t m = claim_mask_;
          if (rc_from_ == 0x01)
            act.frame[3 + OWN_BYTE_I] =
                static_cast<uint8_t>((act.frame[3 + OWN_BYTE_I] & ~OWN_BIT) | m);
          act.frame[3 + 4 + OWN_BYTE_I] =
              static_cast<uint8_t>((act.frame[3 + 4 + OWN_BYTE_I] & ~OWN_BIT) | m);
          uint8_t rs = 0;
          for (int i = 0; i < 11; i++)
            rs += act.frame[i];
          act.frame[11] = static_cast<uint8_t>(0xFF - rs);
          act.bit9_mask = 0x01;
          return act;
        }
      }
    }

    // Session-frame acknowledgement for the enrolled terminal. Every frame
    // the controller sends a terminal (session init 65/66/0D/0E/0F, text 0B,
    // graphics 64, ...) is acked by the terminal with 01' 03 <own-addr> CK --
    // ground truth: the pGD acks everything with 01' 03 20 DB. Frame format:
    // ADDR' TT LL ... where LL is the TOTAL frame length; track byte count
    // and ack when the frame completes. Types 01 (poll) and 02 (roll-call)
    // have their own handlers. Exception: the type-identification request
    // (0x50) is answered with an ident REPLY, not an ack -- ground truth: the
    // pGD answers 20' 50 05 01 89 with 01' 51 07 20 0A 17 65 and no ack.
    //
    // The ack is CHECKSUM-GATED: a frame that completes byte-count-wise but
    // fails its checksum (classic sum-to-0xFF, or CRC-16/Modbus LE residue
    // for 0x64/65/66) gets NO reply -- counted in ack_ck_fail_ instead.
    // The live A/B refuted resend-on-silence (the controller FF-walk-resets
    // after ~2 s instead); see ack_ck_fail_ for why the gate stays.
    if (enroll_) {
      if (bit9 != 0) {
        fa_count_ = (b == ENROLL_ADDR) ? 1 : 0;
        fa_len_ = 0;
        fa_sum_ = b;
        fa_crc_ = crc16_modbus_step(0xFFFF, b);
      } else if (fa_count_ > 0) {
        fa_sum_ = static_cast<uint8_t>(fa_sum_ + b);
        fa_crc_ = crc16_modbus_step(fa_crc_, b);
        if (fa_count_ == 1) {
          // type byte: only track session frames, not poll/roll-call
          fa_type_ = b;
          fa_count_ = (b >= 0x03) ? 2 : 0;
        } else if (fa_count_ == 2) {
          // length byte = total frame length; sanity-cap against corruption
          if (b >= 5 && b <= 200) {
            fa_len_ = b;
            fa_count_ = 3;
          } else {
            fa_count_ = 0;
          }
        } else {
          if (fa_count_ == 3)
            fa_src_ = b;  // byte 3: the sender (0x01 in every controller frame)
          fa_count_ = fa_count_ + 1;
          if (fa_count_ >= fa_len_) {
            fa_count_ = 0;
            // Two checksum grammars (ground truth: planscope's parser,
            // brute-forced from captures): graphic/session-mgmt types carry
            // a CRC-16/Modbus LE trailer -- running the CRC over the WHOLE
            // frame including the trailer leaves residue 0 -- everything
            // else byte-sums to 0xFF.
            bool crc_type = fa_type_ == GRAPHIC_TYPE || fa_type_ == SESSION_INIT_TYPE ||
                            fa_type_ == SESSION_CTL_TYPE;
            bool ck_ok = crc_type ? (fa_crc_ == 0) : (fa_sum_ == 0xFF);
            // A2 D4 / A3 DV-3 (wave E2): only the controller's frames are
            // acked. A terminal's own request to us -- since 09-30 the pGD's
            // `1F' 1F 07 20 20 04 76` (R-LL-20, R-SE-19) -- is no session
            // frame: our `01' 03 1F DC` went to the controller for a frame it
            // never sent. sack_ctrl_only_ = 0: the legacy ack-anything.
            const bool foreign = sack_ctrl_only_ && fa_src_ != 0x01;
            if (ck_ok && foreign)
              tel_sack_foreign_ = tel_sack_foreign_ + 1;
            if (ck_ok && !foreign) {
              // Session-ready gate: the session (re)init (0A/50/66/65)
              // closes it, the first text row (0B/0C) after it opens it.
              if (fa_type_ == 0x0A || fa_type_ == IDENT_REQ_TYPE ||
                  fa_type_ == SESSION_CTL_TYPE || fa_type_ == SESSION_INIT_TYPE)
                sess_ready_ = false;
              else if (fa_type_ == 0x0B || fa_type_ == 0x0C)
                sess_ready_ = true;
            }
            if (!ck_ok) {
              ack_ck_fail_ = ack_ck_fail_ + 1;  // stay silent on garble
            } else if (foreign) {
              // not ours to answer (counted above)
            } else if (fa_type_ == IDENT_REQ_TYPE) {
              act.kind = TxAction::IDENT_REPLY;
              act.len = 7;
              act.frame[0] = 0x01;
              act.frame[1] = 0x51;
              act.frame[2] = 0x07;  // total reply length
              act.frame[3] = ENROLL_ADDR;
              act.frame[4] = PGD_IDENT[0];
              act.frame[5] = PGD_IDENT[1];
              uint8_t is = 0;
              for (int i = 0; i < 6; i++)
                is += act.frame[i];
              act.frame[6] = static_cast<uint8_t>(0xFF - is);
              act.bit9_mask = 0x01;
              return act;
            } else {
              act.kind = TxAction::SESSION_ACK;
              act.len = 4;
              act.frame[0] = 0x01;
              act.frame[1] = 0x03;
              act.frame[2] = ENROLL_ADDR;
              act.frame[3] = static_cast<uint8_t>(0xFF - 0x01 - 0x03 - ENROLL_ADDR);
              act.bit9_mask = 0x01;
              return act;
            }
          }
        }
      }
    }

    // Poll service for the enrolled address: ENROLL_ADDR' 01 <from> CK.
    // The poll is a member-chained token exactly like the roll-call (ground
    // truth 2026-07-16 21:21:17: with two established terminals the pGD@1E
    // handed us the poll token as 1F' 01 1E C1, THREE retries, and our
    // from-01-only matcher ignored every one -- the controller then declared
    // us dead and consolidated the focus; that WAS dual-terminal operation
    // being offered). Accept the token from the controller (0x01) or any
    // ring member (address-range-checked, checksum computed with the real
    // sender). Answer with the pending keypad report plus our link reply,
    // or the bare link reply -- returned to the controller, per the
    // no-blind-forward rule (nothing live sits above us; see ROLLCALL_REPLY).
    if (enroll_ && isr_win_[1] == ENROLL_ADDR && isr_win_[2] == 0x01 && isr_win_[3] >= 0x01 &&
        isr_win_[3] <= 0x20 &&
        isr_win_[4] ==
            static_cast<uint8_t>(0xFF - ENROLL_ADDR - 0x01 - isr_win_[3])) {
      // The link reply returns ALONG THE CHAIN: to the controller for a
      // direct poll (byte-identical legacy), back to the FORWARDING member
      // for a chained poll token -- the forwarder aggregates and produces
      // its own return, which is what paces the controller. Live evidence
      // 2026-07-17 00:04: replying to 0x01 while the pGD@1E forwarded us
      // the token left the pGD without closure (0 replies from it) and the
      // controller re-polled instantly -- a ~500 frames/s storm; the cycle
      // ran but never completed.
      // Chain DIRECTION (2026-07-17): a token from BELOW us is the outbound
      // leg (the focus forwarding upward, 1F' 01 1E C1) -- return to the
      // forwarder. A token from ABOVE us (0x20 answering OUR forward) is
      // the RETURN leg -- produce the controller's completion, the mirror
      // of its original poll (from = the focus = us).
      // A1 V4 / R-RC-33 (wave E2): leaving. Members are never probed, so a
      // renounce in a roll-call reply can never be sent; the controller's own
      // leave path is member loss (R-RC-21c, w7 07-06 10:49:02.615): three
      // unanswered polls, then it drops us from MAP and re-probes and re-joins
      // the remaining member -- no link fault. So the drain stops answering
      // polls at once but keeps acking the controller's session frames: an
      // unacked session frame is a link fault (R-LL-08), an unanswered poll
      // is not while another member remains. drain_replied_ (the task ends
      // the leave) once the controller has let us go: at least 3 polls left
      // unanswered and then a controller roll-call to another address (its
      // re-probe, or the walk after the loss of a lone member).
      // drain_quiet_ = 0: the legacy drain (polls answered until the task's
      // deadline, then a hard stop mid-exchange).
      if (drain_ && drain_quiet_) {
        drain_polls_ = drain_polls_ + 1;
        return act;
      }
      const uint8_t from = isr_win_[3];
      const uint8_t ret = (from > ENROLL_ADDR) ? 0x01 : from;
      lr_direct_ = from == 0x01;  // D5: only a reply to the controller's own poll is resent
      if (from == 0x01) {
        if (fwd_awaiting_) {
          // Our forward never completed and the controller re-polls us
          // directly: count it, back off, answer this one normally.
          fwd_awaiting_ = false;
          fwd_fail_ = fwd_fail_ + 1;
          fwd_backoff_until_us_ = now_us + 1'000'000;
        } else if (fwd_polls_ != 0 && !fwd_just_ && !tx_pending_ &&
                   now_us >= fwd_backoff_until_us_) {
          // ALTERNATE, never forward twice in a row: the controller wants
          // OUR reply for its poll regardless -- after the pGD answers the
          // forwarded token it re-polls us within ~3.5 ms, and that one is
          // ours to answer directly. Forwarding the re-poll too is the
          // 00:04 ~500 f/s storm (the pGD-as-focus made exactly that
          // mistake toward us). One forward, one direct reply, repeat:
          // both terminals answer every ~28 ms macro-cycle.
          //
          // BOOTSTRAP PROBE (2026-07-17 NO LINK #3): liveness resets to 0 on
          // every reboot, and an unpolled pGD transmits NOTHING -- gating the
          // forward on liveness alone deadlocks after any boot where the pGD
          // missed the OTA focus window (worse: the roll-call honest skip
          // then reports 32 dead and the controller drops it entirely --
          // bus10s pgd=0, fwd enabled, ok=0 fail=0, forever). So when
          // liveness is unarmed/stale, PROBE: forward the poll token anyway,
          // at most once per 2 s. A present pGD answers a forwarded token
          // exactly like a direct poll (07-16: first poll out of weeks-deep
          // NO LINK answered in 86 ms) -- its reply arms t_pgd_alive_us_ and
          // normal alternation resumes. An absent pGD fails the probe down
          // the EXISTING benign path (controller re-polls us ~ms later,
          // fwd_fail_ + 1 s backoff). Ring-token forwarding stays strictly
          // liveness-gated -- that is where the 07-16 FF-walk loops lived.
          bool pgd_live =
              t_pgd_alive_us_ != 0 &&
              static_cast<uint64_t>(now_us - t_pgd_alive_us_) < 15'000'000ull;
          bool probe_due = !pgd_live && now_us >= fwd_probe_next_us_;
          if (fwd_gate_) {
            // A2 D3 / R-LL-17: the first poll after OUR join is ours -- the
            // joiner's own link reply completes the join; a forward there
            // kills it (FC_JOIN -> 2 s -> FF-walk). join_open_ holds until
            // the controller acked one of our link replies. And forward only
            // to a pGD that is provably SERVED: 32 in the CLAIMS of the last
            // controller roll-call and a session frame acked since the last
            // FF-walk (every walk rebuilds the sessions). Transmit liveness
            // alone (a member-token answer, a type-1F request) is not service,
            // and a recency window would deadlock: a served pGD that is not
            // the poll focus transmits nothing until it is forwarded a token.
            // The bootstrap probe survives only as the gap owner's offer
            // (R-RC-32/R-RC-27): while 32 is outside the ring because the
            // walker proved it absent, offer 0x20 a token at most every 2 s.
            const bool served32 = rc_claims32_ && pgd_sess_;
            pgd_live = !join_open_ && served32;
            probe_due = !join_open_ && !served32 && pgd_absent_now_(now_us) &&
                        now_us >= fwd_probe_next_us_;
          }
          if (pgd_live || probe_due) {
            if (probe_due)
              fwd_probe_next_us_ = now_us + 2'000'000;
            act.kind = TxAction::FORWARD_POLL;
            act.len = 4;
            act.frame[0] = 0x20;
            act.frame[1] = 0x01;
            act.frame[2] = ENROLL_ADDR;
            act.frame[3] = static_cast<uint8_t>(0xFF - 0x20 - 0x01 - ENROLL_ADDR);
            act.bit9_mask = 0x01;
            return act;
          }
        }
      } else if (from > ENROLL_ADDR && fwd_awaiting_) {
        fwd_awaiting_ = false;
        fwd_ok_ = fwd_ok_ + 1;  // completion variant (b): returned via us
      }
      if (from == 0x01)
        fwd_just_ = false;  // a direct reply re-arms the forward alternation
      const uint8_t lr[4] = {ret, 0x01, ENROLL_ADDR,
                             static_cast<uint8_t>(0xFF - ret - 0x01 - ENROLL_ADDR)};
      // A pending key rides only a session-ready slot (R-KP-07); until then
      // the bare link reply keeps the link while the controller re-sessions.
      if (tx_pending_ && tx_mode_ == 2 && sess_ready_) {
        act.kind = TxAction::ENROLL_KEY_REPLY;
        act.len = REPLY9_LEN;
        for (size_t i = 0; i < KEYPAD_LEN + 1; i++)
          act.frame[i] = tx_frame_[i];
        // Rewrite the report's terminal-address byte (0x20 = the pGD) to our
        // enrolled address and fix the sum-to-0xFE check byte, so the key
        // press is attributed to the terminal the poll addressed.
        act.frame[3] = ENROLL_ADDR;
        act.frame[6] = static_cast<uint8_t>(
            0xFE - (act.frame[1] + act.frame[2] + act.frame[3] + act.frame[4] + act.frame[5]));
        act.frame[7] = lr[0];
        act.frame[8] = lr[1];
        act.frame[9] = lr[2];
        act.frame[10] = lr[3];
        act.bit9_mask = REPLY9_BIT9_MASK;
      } else {
        act.kind = TxAction::ENROLL_LINK_REPLY;
        act.len = 4;
        for (int i = 0; i < 4; i++)
          act.frame[i] = lr[i];
        act.bit9_mask = 0x01;
      }
      return act;
    }

    // pGD turnaround probe: measure poll-end -> next-byte gaps (the pGD's
    // reply latency drifts with its workload, and acceptance of an injected
    // reply depends on beating it). Ring of the last 16 gaps.
    if (t_poll_end_ != 0) {
      uint32_t gap = static_cast<uint32_t>(now_us - t_poll_end_);
      gap_ring_[gap_n_ % 16] = gap;
      gap_n_ = gap_n_ + 1;
      t_poll_end_ = 0;
    }

    // tx_mode 1: transmit the keypad report AFTER the pGD's own link reply
    // and the controller's ack byte (idle burst tail: 01' 01 20 DD 01'),
    // instead of racing the pGD for the poll response slot. The report goes
    // out standalone; the pGD has already satisfied the poll.
    if (tx_mode_ == 1 && tx_pending_ && isr_win_[0] == 0x01 && isr_win_[1] == 0x01 &&
        isr_win_[2] == 0x20 && isr_win_[3] == 0xDD && isr_win_[4] == 0x01 && bit9 != 0) {
      act.kind = TxAction::AFTER_BURST_REPORT;
      act.len = KEYPAD_LEN + 1;
      for (size_t i = 0; i < KEYPAD_LEN + 1; i++)
        act.frame[i] = tx_frame_[i];
      act.bit9_mask = 0x01;  // bit9 on the address byte only
      return act;
    }

    bool poll_match = isr_win_[1] == POLL4[0] && isr_win_[2] == POLL4[1] &&
                      isr_win_[3] == POLL4[2] && isr_win_[4] == POLL4[3];
    if (tx_mode_ == 0 && tx_pending_ && poll_match) {
      // The response slot opens now. The real pGD also answers this poll --
      // its 0.37-2.5 ms turnaround jitters into our burst often enough that
      // acceptance is probabilistic; the rejection detection below and the
      // task's retry pacing handle that.
      act.kind = TxAction::RACE_KEY_REPLY;
      act.len = REPLY9_LEN;
      for (size_t i = 0; i < REPLY9_LEN; i++)
        act.frame[i] = tx_frame_[i];
      act.bit9_mask = REPLY9_BIT9_MASK;
      return act;
    } else if (tx_done_us_ != 0 && poll_match) {
      // A poll arriving hard on the heels of our reply is the controller
      // RE-polling: it heard our transmission but did not accept it
      // (typically the real pGD's reply landed on top of ours). The press
      // was NOT registered, so the task may safely retry. Regular polls are
      // tens of ms apart; the re-poll follows within ~1 ms.
      if (now_us - tx_done_us_ < 5000)
        tx_rejected_ = true;
      tx_done_us_ = 0;
      t_poll_end_ = now_us;
    } else if (poll_match) {
      t_poll_end_ = now_us;  // arm the turnaround measurement
    }

    return act;
  }

  // The caller actually put the action's frame on the wire.
  void PLAN_IRAM tx_sent(const TxAction &act, int64_t now_us) {
    txlog_push_(act, now_us, 1);
    tel_last_tx_us_ = now_us;  // arms the post-TX-gap measurement
    switch (act.kind) {
      case TxAction::ROLLCALL_REPLY:
        enroll_replies_ = enroll_replies_ + 1;
        if (drain_)
          drain_replied_ = true;  // the renounce went out; task finishes the leave
        for (int i = 0; i < 8; i++)
          rc_last_[i] = act.frame[3 + i];  // what a confirm of it carries
        rc_last_ok_ = true;
        if (rc_opens_join_) {
          if (join_open_)
            rc_join_failed_(now_us);  // re-probed: the previous join never closed
          join_open_ = true;
          pgd_sess_ = false;  // our adoption re-inits every served terminal (R-SE-11)
          tel_joins_ = tel_joins_ + 1;
        }
        break;
      case TxAction::SESSION_ACK:
        session_acks_ = session_acks_ + 1;
        tel_run_sacked_ = true;  // E5: the run on the wire (its last byte) is acked by us
        break;
      case TxAction::IDENT_REPLY:
        ident_replies_ = ident_replies_ + 1;
        break;
      case TxAction::ENROLL_LINK_REPLY:
        enroll_polls_ = enroll_polls_ + 1;
        lr_ack_wait_ = 1;  // a bare 01' as the next run = accepted (rc_run_end_)
        // D5: a reply to the controller's own poll is resent if unacked
        // (on_tick); replies to member-forwarded tokens never are (A6 T12).
        if (lr_resend_ && lr_direct_) {
          for (int i = 0; i < 4; i++)
            lr_rs_frame_[i] = act.frame[i];
          lr_rs_n_ = 1;
          lr_rs_t_ = now_us;
        }
        break;
      case TxAction::LINK_RESEND:
        tel_lr_resends_ = tel_lr_resends_ + 1;
        lr_ack_wait_ = 1;  // an ack of the copy closes a join just the same
        if (lr_rs_n_ != 0)
          lr_rs_n_ = static_cast<uint8_t>(lr_rs_n_ + 1);
        lr_rs_t_ = now_us;
        break;
      case TxAction::ENROLL_KEY_REPLY:
        enroll_polls_ = enroll_polls_ + 1;
        lr_ack_wait_ = 1;
        tx_pending_ = false;
        tx_done_us_ = now_us;
        key_ack_ = 0;
        key_await_ = true;  // the next byte heard decides the fate
        tx_fired_ = true;
        break;
      case TxAction::AFTER_BURST_REPORT:
      case TxAction::RACE_KEY_REPLY:
        tx_pending_ = false;
        tx_done_us_ = now_us;
        key_ack_ = 0;
        key_await_ = true;
        tx_fired_ = true;
        break;
      case TxAction::FORWARD_POLL:
        fwd_awaiting_ = true;  // completion tracked by the window matcher
        fwd_just_ = true;      // next direct poll is ours to answer
        break;
      default:
        break;
    }
  }

  // The caller skipped the action (RX FIFO was not empty: the match is stale,
  // the controller has moved on). Only the slot-race path counts these.
  void PLAN_IRAM tx_not_sent(const TxAction &act) {
    txlog_push_(act, 0, 0);
    if (act.kind == TxAction::RACE_KEY_REPLY)
      isr_stale_ = isr_stale_ + 1;
    if (act.kind == TxAction::LINK_RESEND)
      lr_rs_n_ = 0;  // the bus was busy: the exchange moved on without us
  }

  // Timer entry (wave E2, D5): call every <= 1 ms while the bus is quiet (the
  // firmware's tick ISR; the simulator schedules it). Returns the identical
  // resend of a link reply the controller left unacked -- the pGD's REPLY_RETRY
  // (A6 T8, R-LL-05): 3 transmissions in all, 14 and 15 ms apart start to start
  // (LR_GAP*_US from the end of the previous copy), then nothing: no own walk
  // (we never act as master), and no timeout anywhere is extended. Any received
  // byte cancels it (on_byte), so a resend never lands in the controller's
  // 19 ms re-poll (R-LL-06); a tick later than LR_LATE_US (flash-cache stall)
  // gives the slot up instead of transmitting late. Same caller contract as
  // on_byte: report tx_sent / tx_not_sent.
  static constexpr int64_t LR_GAP1_US = 13200;  // 14 ms start to start (4-byte reply)
  static constexpr int64_t LR_GAP2_US = 14200;  // 15 ms
  static constexpr int64_t LR_LATE_US = 1500;
  TxAction PLAN_IRAM on_tick(int64_t now_us) {
    TxAction act;
    act.kind = TxAction::NONE;
    act.len = 0;
    act.bit9_mask = 0;
    if (lr_rs_n_ == 0)
      return act;
    const int64_t due = lr_rs_t_ + (lr_rs_n_ == 1 ? LR_GAP1_US : LR_GAP2_US);
    if (now_us < due)
      return act;
    if (lr_rs_n_ >= 3 || !enroll_ || drain_ || !lr_resend_ || now_us - due > LR_LATE_US) {
      if (lr_rs_n_ >= 3)
        tel_lr_lost_ = tel_lr_lost_ + 1;  // 3 transmissions, none acked
      lr_rs_n_ = 0;
      return act;
    }
    lr_rs_t_ = now_us;  // in flight: no second copy before tx_sent / tx_not_sent
    act.kind = TxAction::LINK_RESEND;
    act.len = 4;
    for (int i = 0; i < 4; i++)
      act.frame[i] = lr_rs_frame_[i];
    act.bit9_mask = 0x01;
    return act;
  }

 protected:
  // ISR-internal state (single writer).
  volatile uint8_t isr_win_[5]{0};    // rolling window to match the poll token
  volatile uint8_t reset_win_[8]{0};  // rolling window for the FF-walk marker
  volatile int rc_state_{0};          // roll-call capture progress
  volatile uint8_t rc_payload_[8]{0}; // captured roll-call payload
  volatile int fa_count_{0};          // session-frame ack: bytes seen
  volatile int fa_len_{0};            // session-frame ack: total frame length
  volatile uint8_t fa_type_{0};       // session-frame ack: frame type byte
  volatile uint8_t fa_sum_{0};        // session-frame ack: running byte sum
  volatile uint16_t fa_crc_{0};       // session-frame ack: running CRC-16/Modbus
  volatile int64_t tx_done_us_{0};    // time our last TX finished
  volatile int64_t t_poll_end_{0};    // pending turnaround measurement

  // Phase 0 telemetry: current bit9-delimited run being accumulated.
  volatile bool tel_active_{false};    // false until the first address byte
  volatile uint8_t tel_addr_{0};       // leading address byte of the run
  volatile uint8_t tel_sum_{0};        // running byte sum (mod 256)
  volatile uint32_t tel_len_{0};       // bytes in the run so far
  volatile int64_t tel_last_tx_us_{0}; // pending post-TX-gap measurement
  // Layout rule (W3 regression, 2026-07-10): new members go at the CLASS END
  // only -- inserting above shifts member offsets and has broken the ISR.
  volatile uint8_t tel_type_{0};       // frame type byte of the current run
  volatile uint8_t rc_from_{0};        // roll-call token sender (ring member or 0x01)
  volatile int64_t t_link_reset_us_{0}; // last FF-walk marker (telemetry)
  volatile uint8_t tel_b3_{0};          // third byte of the current run
  volatile int64_t t_pgd_alive_us_{0};  // last transmission seen FROM the pGD@32

  // --- TX capture ring (heatpump-firmware#14 T1b) ------------------------
  // Every transmit decision the ISR made, sent AND skipped, for the task to
  // surface into the PLANCAP stream: our own frames never reach the RX path
  // (DE/RE tied mute the receiver during TX), so without this half of every
  // exchange is invisible and "we didn't transmit" is never provable.
  // Single writer (ISR), single reader (bus task): the reader chases
  // txlog_w_ with its own index and treats lag > TXLOG_N as overwritten
  // entries. Entries are plain storage; on the single-core C3 the volatile
  // index publish orders the accesses -- same idiom as tx_frame_/tx_pending_.
 public:
  static constexpr uint32_t TXLOG_N = 16;  // power of two
  struct TxLog {
    int64_t us;     // esp_timer at tx_sent; 0 for a skipped action
    uint8_t sent;   // 1 = frame went on the wire, 0 = slot skipped (stale)
    uint8_t kind;   // TxAction::Kind
    uint8_t len;
    uint8_t frame[12];
    uint16_t bit9;  // bit i set => frame[i] carried the 9th/address bit
  };
  TxLog txlog_[TXLOG_N]{};
  volatile uint32_t txlog_w_{0};

 protected:
  void PLAN_IRAM txlog_push_(const TxAction &act, int64_t us, uint8_t sent) {
    TxLog &e = txlog_[txlog_w_ & (TXLOG_N - 1)];
    e.us = us;
    e.sent = sent;
    e.kind = static_cast<uint8_t>(act.kind);
    e.len = act.len;
    e.bit9 = act.bit9_mask;
    for (uint8_t i = 0; i < act.len && i < sizeof(e.frame); i++)
      e.frame[i] = act.frame[i];
    txlog_w_ = txlog_w_ + 1;  // publish last
  }

  // --- poll-token chain forwarding (heatpump-firmware#14, dual-terminal) ---
  // When WE hold the poll focus, hand the poll token on to the live pGD@32
  // the way the pGD did for us. OFF by default (one experiment at a time);
  // runtime-switched via the set_poll_fwd service.
 public:
  volatile int fwd_polls_{0};
  volatile uint32_t fwd_ok_{0};    // forwards that saw a completion
  volatile uint32_t fwd_fail_{0};  // forwards the controller had to re-poll past

 protected:
  volatile bool fwd_awaiting_{false};    // forward sent, completion not yet heard
  volatile int64_t fwd_backoff_until_us_{0};  // no forwards until then after a fail
  volatile int64_t fwd_probe_next_us_{0};     // liveness-unarmed bootstrap probe pacing
  volatile bool fwd_just_{false};        // last poll was forwarded: alternate

  // --- planterm#47 reject instrumentation (read-and-reset per bus10s window) --
  // The controller silently discards ~1 in 150 of our in-cadence, well-formed
  // link replies: no ack, then ~2 s of silence, then the FF-walk whose return
  // drops the pGD. Our RE is muted while we drive DE, so the discarded frame is
  // invisible in the capture; these three surface it on-device. gap_max/unacked
  // use the post-TX-gap measurement above (our-TX-end -> next-RX-byte); walks
  // counts the FF-walk marker. The coincidence of unacked and walks over a long
  // enrolled window settles whether every walk follows a rejected reply
  // (reject-driven) or walks also happen on acked replies (controller-side).
  // LAYOUT RULE (see tel_type_): new members at the class END only.
 public:
  volatile uint32_t tel_post_tx_gap_max_us_{0};  // max our-TX-end -> next-byte gap
  volatile uint32_t tel_tx_unacked_{0};          // our TX not followed within 100 ms
  volatile uint32_t tel_walks_{0};               // FF-walk markers this window

  // --- key fate + session-ready gate (A5 R-KP-07/11/12, wave E3) -----------
  // LAYOUT RULE (see tel_type_): new members at the class END only.
  // key_ack_: verdict on our last key burst, from the first byte heard after
  // it: 0 = none yet (task: silence after KEY_VERDICT_MS), 1 = acked by the
  // controller, 2 = anything else first (re-poll, another frame). Task resets.
  volatile uint8_t key_ack_{0};
  volatile bool key_await_{false};   // key burst sent, first byte after it pending
  // Keys may ride our poll slot (R-KP-07). Closed by any recovery walk, the
  // session-init frames to us and set_enroll(true); opened by the first text
  // row of the fresh session. A fresh state machine (host tests) starts open.
  volatile bool sess_ready_{true};
  volatile uint8_t walk_m_{0};       // walk-header matcher progress (02' 02 01 FF FF FF FF)

  // --- wave E1: roll-call / join rework (A1 R-RC-30/32, A2 D2/D3/D8) --------
  // Knobs (task -> ISR, runtime-switchable via PlanBridge setters). Defaults
  // are the fixed behaviour; 0 restores the pre-E1 behaviour of that change.
  volatile uint8_t rc_ff_silent_{1};       // R-RC-30: silent on FF-walk probes while a pGD may exist
  volatile uint8_t rc_honest_skip_{0};     // 1 = legacy D2 skip (clear 32 after 15 s pGD silence)
  volatile uint8_t fwd_gate_{1};           // D3: no forward before our join closed / to an unserved pGD
  volatile uint8_t rc_walk_any_claims_{1}; // D8/V1: warm walks count, lone announce probe folded
  volatile uint8_t rc_backoff_{1};         // root cause 3: back off our joins while they keep failing
  volatile uint8_t rc_backoff_after_{2};   // consecutive failed own joins before the back-off
  volatile uint32_t pgd_absent_us_{40'000'000};  // 0x20 silent this long at its walk probe = no pGD
  // Telemetry (ISR increments; the task reads and resets per bus10s window,
  // except the state fields).
  volatile uint32_t tel_walks_warm_{0};    // of tel_walks_: warm (claims carried)
  volatile uint32_t tel_walks_folded_{0};  // lone announce probes folded into their restart
  volatile uint32_t tel_rc_silent_{0};     // FF-walk probes left to 0x20 (R-RC-30)
  volatile uint32_t tel_rc_backoff_{0};    // join probes skipped by the back-off
  volatile uint32_t tel_joins_{0};         // joins opened (reply to a controller probe)
  volatile uint32_t tel_joins_ok_{0};      // ... closed: our link reply acked
  volatile uint32_t tel_joins_failed_{0};  // ... died before that (walk / re-probe)
  volatile uint32_t rc_join_streak_{0};    // state: consecutive failed own joins
  volatile int64_t rc_backoff_until_us_{0};// state: back-off end (0 = none)
  bool pgd_absent(int64_t now_us) const { return pgd_absent_now_(now_us); }
  bool join_open() const { return join_open_; }

 protected:
  volatile bool rc_init_{false};       // first byte seen (boot time stamped)
  volatile uint8_t tel_c1_{0};         // first CLAIMS byte of the current run
  volatile int64_t t_pgd_tx_us_{0};    // last frame FROM 0x20 (types 01-03), boot if none
  volatile bool rc20_open_{false};     // the previous run was the controller's probe of 0x20
  volatile bool rc20_unans_{false};    // ... and nothing from 0x20 answered it (sticky)
  volatile bool rc_claims32_{true};    // 32 in the CLAIMS of the last controller roll-call
                                       // (true until one is seen: unknown does not block)
  volatile bool pgd_sess_{false};      // the pGD acked a session frame since the last walk
  volatile bool walk_run_{false};      // the current run is an FF-walk opener
  volatile bool walk_prev_{false};     // the previous run was one
  volatile uint8_t rc_last_[8]{0};     // our last roll-call reply's masks ...
  volatile bool rc_last_ok_{false};    // ... valid until the next FF-walk opener
  volatile bool rc_opens_join_{false}; // the pending roll-call reply answers a probe
  volatile bool join_open_{false};     // our join is open: no link reply acked yet
  volatile uint8_t lr_ack_wait_{0};    // 1: link reply sent, 2: its next run is being read

  // No pGD: the controller's own probe of 0x20 went unanswered and 0x20 has
  // not transmitted for pgd_absent_us_ (since boot if never). The window
  // covers a pGD reboot (A6: 17-20 s silent) so a power cycle never opens
  // the gap above 31.
  bool PLAN_IRAM pgd_absent_now_(int64_t now_us) const {
    return rc20_unans_ && static_cast<uint64_t>(now_us - t_pgd_tx_us_) >= pgd_absent_us_;
  }

  // Run boundary: the run in tel_addr_/tel_type_/tel_b3_/tel_len_ completed.
  void PLAN_IRAM rc_run_end_(int64_t now_us) {
    // A frame sent BY 0x20 (link reply 01 01 20, roll-call answer or master
    // token NN 02 20, ack 01 03 20, poll token 1F 01 20): the pGD exists.
    // Types 01-03 only -- a session frame's third byte is its length.
    const bool from_pgd = tel_addr_ != 0x20 && tel_len_ >= 4 && tel_b3_ == 0x20 &&
                          tel_type_ >= 0x01 && tel_type_ <= 0x03;
    if (from_pgd) {
      t_pgd_tx_us_ = now_us;
      rc20_unans_ = false;
      if (tel_addr_ == 0x01 && tel_type_ == 0x03)
        pgd_sess_ = true;  // it acked a controller session frame
    } else if (rc20_open_) {
      rc20_unans_ = true;
    }
    const bool ctrl_rc = tel_type_ == 0x02 && tel_b3_ == 0x01 && tel_len_ == 12;
    // V4: the controller has let us go -- our polls went unanswered and it
    // runs a roll-call elsewhere (its member-loss re-probe, or a walk).
    if (drain_ && drain_quiet_ && ctrl_rc && tel_addr_ != ENROLL_ADDR && drain_polls_ >= 3)
      drain_replied_ = true;
    rc20_open_ = ctrl_rc && tel_addr_ == 0x20;
    if (ctrl_rc)
      rc_claims32_ = (tel_c1_ & 0x80) != 0;
    if (lr_ack_wait_ == 2) {
      if (tel_addr_ == 0x01 && tel_len_ == 1 && join_open_) {
        join_open_ = false;  // the controller accepted our first link reply
        tel_joins_ok_ = tel_joins_ok_ + 1;
        rc_join_streak_ = 0;
        rc_backoff_until_us_ = 0;
      }
      lr_ack_wait_ = 0;
    }
    walk_prev_ = walk_run_;
    walk_run_ = false;
  }

  // --- wave E2: link / session fixes (A2 D4/D5/D7, A1 V4) ------------------
  // LAYOUT RULE (see tel_type_): new members at the class END only.
 public:
  // Knobs (task -> ISR, runtime-switchable via PlanBridge setters). Defaults
  // are the fixed behaviour; the other value restores the pre-E2 behaviour.
  // EXCEPT D5: default OFF -- its copies need a timer entry (on_tick), i.e. a
  // new tick ISR in the firmware, which waits for a live A/B (parent decision).
  volatile uint8_t rc_member_tokens_{0};  // D7: 1 = answer member-forwarded roll-call tokens
  volatile uint8_t sack_ctrl_only_{1};    // D4: ack only frames whose sender is 0x01
  volatile uint8_t lr_resend_{0};         // D5: 1 = resend an unacked link reply (on_tick)
  volatile uint8_t drain_quiet_{1};       // V4: leave by member loss (polls unanswered, sessions acked)
  // Telemetry (ISR increments; the task reads and resets per bus10s window).
  volatile uint32_t tel_rc_member_ign_{0};  // member-forwarded tokens ignored (D7)
  volatile uint32_t tel_sack_foreign_{0};   // terminal frames to us not acked (D4)
  volatile uint32_t tel_lr_resends_{0};     // link-reply copies sent (D5)
  volatile uint32_t tel_lr_lost_{0};        // replies unacked after all 3 transmissions
  volatile uint32_t drain_polls_{0};        // state: polls left unanswered in this drain

 protected:
  volatile uint8_t fa_src_{0};          // session-frame ack: sender byte (byte 3)
  volatile bool lr_direct_{false};      // the pending link reply answers a poll from 0x01
  volatile uint8_t lr_rs_n_{0};         // transmissions of the unacked reply (0 = none pending)
  volatile int64_t lr_rs_t_{0};         // end of its latest copy
  volatile uint8_t lr_rs_frame_[4]{0};  // the reply, byte-identical for the copies

  void PLAN_IRAM rc_join_failed_(int64_t now_us) {
    join_open_ = false;
    tel_joins_failed_ = tel_joins_failed_ + 1;
    rc_join_streak_ = rc_join_streak_ + 1;
    if (rc_join_streak_ >= rc_backoff_after_) {
      // one gap-walk period, doubling per further failure, capped at 16x
      uint32_t k = rc_join_streak_ - rc_backoff_after_;
      if (k > 4)
        k = 4;
      rc_backoff_until_us_ = now_us + (static_cast<int64_t>(12'000'000) << k);
    }
  }

  // --- wave E5: bus-health telemetry (A16 sect. 7, A7 sect. 3, A2 D11) ------
  // LAYOUT RULE (see tel_type_): new members at the class END only. Counting
  // only: nothing here feeds a transmit decision.
 public:
  // ISR-fed counters; take_window() reads and resets them per bus10s window.
  volatile uint32_t tel_ack20_{0}, tel_ra20_{0}, tel_ack1f_{0}, tel_ra1f_{0};
  volatile uint32_t tel_pgd_rc_{0};
  volatile uint32_t tel_ctrl_src_{0};
  volatile uint32_t tel_ctrl_sil_{0};
  volatile uint32_t tel_ctrl_gap_max_us_{0};
  volatile uint32_t tel_addr_bad_{0};
  volatile uint32_t tel_uart_frm_{0}, tel_uart_brk_{0}, tel_uart_glitch_{0};
  volatile uint32_t tel_tx_frames_{0};
  volatile int32_t tel_hold_dev_min_{0}, tel_hold_dev_max_{0};
  volatile uint32_t tel_tx_early_idle_{0};  // incremented inside tx_9bit (plan_bridge_isr.cpp)

  // uart_isr, once per pass: the UART receive-error bits of its raw status.
  void PLAN_IRAM uart_errs(bool frm, bool brk, bool glitch) {
    if (frm)
      tel_uart_frm_ = tel_uart_frm_ + 1;
    if (brk)
      tel_uart_brk_ = tel_uart_brk_ + 1;
    if (glitch)
      tel_uart_glitch_ = tel_uart_glitch_ + 1;
  }

  // uart_isr, after each frame we put on the wire: DE hold vs the frame's
  // wire length (A7 F2). Any length: a cut 12-byte frame can no longer hide
  // above the 4-byte minimum like it did in de_hold_max.
  void PLAN_IRAM tx_hold(uint32_t len, uint32_t hold_us, uint32_t char_us, uint32_t tail_us) {
    const int32_t dev = static_cast<int32_t>(hold_us) -
                        static_cast<int32_t>(len * char_us + TX_HOLD_OVERHEAD_US + tail_us);
    if (tel_tx_frames_ == 0 || dev < tel_hold_dev_min_)
      tel_hold_dev_min_ = dev;
    if (tel_tx_frames_ == 0 || dev > tel_hold_dev_max_)
      tel_hold_dev_max_ = dev;
    tel_tx_frames_ = tel_tx_frames_ + 1;
  }

  // Task, once per bus10s window, BEFORE rollcall10s resets tel_walks_warm_ /
  // tel_joins_*: latch the window, reset its counters (tel_walks_ included),
  // judge bus_health. Read-then-reset without a lock like the other bus10s
  // counters (an increment in between lands in the next window).
  BusWindow take_window(int64_t now_us) {
    BusWindow w{};
    w.walks = tel_walks_;
    tel_walks_ = 0;
    w.walks_warm = tel_walks_warm_;
    w.joins = tel_joins_;
    w.joins_ok = tel_joins_ok_;
    w.ack20 = tel_ack20_;
    tel_ack20_ = 0;
    w.ra20 = tel_ra20_;
    tel_ra20_ = 0;
    w.ack1f = tel_ack1f_;
    tel_ack1f_ = 0;
    w.ra1f = tel_ra1f_;
    tel_ra1f_ = 0;
    w.pgd_rc = tel_pgd_rc_;
    tel_pgd_rc_ = 0;
    w.ctrl_frames = tel_ctrl_src_;
    tel_ctrl_src_ = 0;
    w.ctrl_sil = tel_ctrl_sil_;
    tel_ctrl_sil_ = 0;
    uint32_t gap = tel_ctrl_gap_max_us_;
    tel_ctrl_gap_max_us_ = 0;
    const int64_t t_ctrl = load64_(t_ctrl_us_);
    if (t_ctrl != 0 && sat32_(now_us - t_ctrl) > gap)
      gap = sat32_(now_us - t_ctrl);  // the silence still open at the window's end
    w.ctrl_gap_max_ms = gap / 1000u;
    const int64_t t_paint = load64_(t_paint20_us_);
    w.paint_age_s = t_paint != 0 ? sat32_(now_us - t_paint) / 1000000u : 0;
    w.addr_bad = tel_addr_bad_;
    tel_addr_bad_ = 0;
    w.uart_frm = tel_uart_frm_;
    tel_uart_frm_ = 0;
    w.uart_brk = tel_uart_brk_;
    tel_uart_brk_ = 0;
    w.uart_glitch = tel_uart_glitch_;
    tel_uart_glitch_ = 0;
    w.tx = tel_tx_frames_;
    w.hold_dev_min_us = w.tx != 0 ? tel_hold_dev_min_ : 0;
    w.hold_dev_max_us = w.tx != 0 ? tel_hold_dev_max_ : 0;
    tel_tx_frames_ = 0;
    w.tx_early_idle = tel_tx_early_idle_;
    tel_tx_early_idle_ = 0;
    BusSpan s{w.walks, w.ack20, w.ra20, w.ack1f, w.ra1f};
    for (int i = 0; i < HEALTH_SPAN - 1; i++) {
      s.walks += hl_hist_[i].walks;
      s.ack20 += hl_hist_[i].ack20;
      s.ra20 += hl_hist_[i].ra20;
      s.ack1f += hl_hist_[i].ack1f;
      s.ra1f += hl_hist_[i].ra1f;
    }
    for (int i = HEALTH_SPAN - 2; i > 0; i--)
      hl_hist_[i] = hl_hist_[i - 1];
    hl_hist_[0] = BusSpan{w.walks, w.ack20, w.ra20, w.ack1f, w.ra1f};
    w.health = bus_health(w, s);
    return w;
  }

 protected:
  volatile uint8_t tel_b4_{0};          // fourth byte of the current run
  volatile uint32_t tel_hash_{0};       // FNV-1a of the current run
  volatile int64_t tel_start_us_{0};    // its address byte's time
  volatile bool tel_run_sacked_{false}; // we sent a session ack for it
  volatile uint8_t ss_state_{0};        // resend watch: 0 off, 1 awaiting the pGD ack, 2 acked
  volatile uint8_t ss_to_{0};           // ... the watched frame's terminal
  volatile uint32_t ss_len_{0};         // ... its length
  volatile uint32_t ss_hash_{0};        // ... its FNV-1a
  volatile int64_t t_ctrl_us_{0};       // start of the controller's last frame (0 = none yet)
  volatile int64_t t_paint20_us_{0};    // ... of its last session frame to 0x20 (first run if none)
  BusSpan hl_hist_[HEALTH_SPAN - 1]{};  // task only: the previous windows' rate inputs

  static uint32_t PLAN_IRAM sat32_(int64_t d) {
    return d <= 0 ? 0u : d >= 0xFFFFFFFFll ? 0xFFFFFFFFu : static_cast<uint32_t>(d);
  }
  // A 64-bit ISR-written time read from the task (two 32-bit loads on the C3).
  static int64_t load64_(const volatile int64_t &v) {
    int64_t a = v;
    while (a != v)
      a = v;
    return a;
  }

  // Run boundary (after rc_run_end_): the bus-health view of the run in
  // tel_addr_/tel_type_/tel_b3_/tel_b4_/tel_len_/tel_hash_. Same frame rules
  // as `ekobeescope health` (frameSrc / trackSession / ctrlSeen).
  void PLAN_IRAM hl_run_end_() {
    const uint8_t to = tel_addr_, ty = tel_type_;
    const uint32_t len = tel_len_;
    const uint8_t src = (ty >= 0x01 && ty <= 0x03) ? tel_b3_ : tel_b4_;
    const bool from_ctrl = len >= 4 && to != 0x01 && src == 0x01;

    // Resend after ack (A16 7.1, A8 "ra"): a controller session frame, the
    // terminal's ack, then the identical frame again = the controller did
    // not hear the ack. Measured for the pGD from its own `01' 03 20 DB`
    // (works passive: no TX of ours involved) and for us from our session
    // ack's tx_sent (invisible on the wire: RE is muted while we drive).
    if (ss_state_ == 1 && len == 4 && to == 0x01 && ty == 0x03 && tel_b3_ == ss_to_) {
      ss_state_ = 2;
      tel_ack20_ = tel_ack20_ + 1;
    } else {
      if (ss_state_ == 2 && len == ss_len_ && tel_hash_ == ss_hash_) {
        if (ss_to_ == 0x20)
          tel_ra20_ = tel_ra20_ + 1;
        else
          tel_ra1f_ = tel_ra1f_ + 1;
      }
      ss_state_ = 0;
      if (from_ctrl && len >= 5 && session_type(ty)) {
        if (to == 0x20) {
          ss_state_ = 1;
        } else if (to == ENROLL_ADDR && tel_run_sacked_) {
          ss_state_ = 2;
          tel_ack1f_ = tel_ack1f_ + 1;
        }
        ss_to_ = to;
        ss_len_ = len;
        ss_hash_ = tel_hash_;
      }
    }

    // The pGD running the roll-call itself (A8 pGD-master, A6 TOKEN_LOST).
    if (to != 0x01 && ty == 0x02 && len == 12 && tel_b3_ == 0x20)
      tel_pgd_rc_ = tel_pgd_rc_ + 1;

    // Controller silences: gaps between the STARTS of its frames.
    if (from_ctrl) {
      tel_ctrl_src_ = tel_ctrl_src_ + 1;
      if (t_ctrl_us_ != 0) {
        const uint32_t gap = sat32_(tel_start_us_ - t_ctrl_us_);
        if (gap > tel_ctrl_gap_max_us_)
          tel_ctrl_gap_max_us_ = gap;
        if (gap >= HEALTH_SILENCE_MS * 1000u)
          tel_ctrl_sil_ = tel_ctrl_sil_ + 1;
      }
      t_ctrl_us_ = tel_start_us_;
      if (to == 0x20 && session_type(ty))
        t_paint20_us_ = tel_start_us_;
    }
    if (t_paint20_us_ == 0)
      t_paint20_us_ = tel_start_us_;  // the paint age counts from the first frame heard
  }
};

}  // namespace plan
