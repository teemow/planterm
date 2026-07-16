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
  };
  Kind kind;
  uint8_t len;
  uint8_t frame[12];
  uint16_t bit9_mask;  // bit i set => frame[i] carries the 9th/address bit
};

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
      tel_last_tx_us_ = 0;
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
        if (tel_addr_ == 0x01)
          tel_frames_ctrl_ = tel_frames_ctrl_ + 1;
        else if (tel_addr_ == 0x20)
          tel_frames_pgd_ = tel_frames_pgd_ + 1;
        else if (tel_addr_ == ENROLL_ADDR)
          tel_frames_us_ = tel_frames_us_ + 1;
        else
          tel_frames_other_ = tel_frames_other_ + 1;
      }
      tel_active_ = true;  // (joining mid-frame at boot: first partial run is skipped)
      tel_addr_ = b;
      tel_sum_ = b;
      tel_len_ = 1;
    } else if (tel_active_) {
      if (tel_len_ == 1)
        tel_type_ = b;  // frame type = first byte after the address byte
      tel_sum_ = static_cast<uint8_t>(tel_sum_ + b);
      tel_len_ = tel_len_ + 1;
    }

    // Rolling windows.
    isr_win_[0] = isr_win_[1];
    isr_win_[1] = isr_win_[2];
    isr_win_[2] = isr_win_[3];
    isr_win_[3] = isr_win_[4];
    isr_win_[4] = b;

    // Link-reset detector: the first frame of the controller's recovery walk
    // is SS' 02 01 FF FF FF FF 00 00 00 00 CC. Eight bytes of context make a
    // pixel-data false positive practically impossible.
    reset_win_[0] = reset_win_[1];
    reset_win_[1] = reset_win_[2];
    reset_win_[2] = reset_win_[3];
    reset_win_[3] = reset_win_[4];
    reset_win_[4] = reset_win_[5];
    reset_win_[5] = reset_win_[6];
    reset_win_[6] = reset_win_[7];
    reset_win_[7] = b;
    if (reset_win_[0] == 0x02 && reset_win_[1] == 0x01 && reset_win_[2] == 0xFF &&
        reset_win_[3] == 0xFF && reset_win_[4] == 0xFF && reset_win_[5] == 0xFF &&
        reset_win_[6] == 0x00 && reset_win_[7] == 0x00)
      link_reset_ = true;

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
        if (static_cast<uint8_t>(s + b) == 0xFF) {
          act.kind = TxAction::ROLLCALL_REPLY;
          act.len = 12;
          // Return the token to the controller (0x01) -- NEVER forward to
          // 0x20 blind. Forwarding per the presence map looped the bus live
          // (2026-07-16 20:15-26): FF-walk recovery initializes presence to
          // assume-all-alive, so bit7 pointed at the EMPTY address 32; a
          // ring member that forwards must also timeout-and-skip a silent
          // next hop (the pGD does), which this byte-driven ISR cannot do
          // without a timer. The token dying at 32 link-faulted the
          // controller into 84 consecutive FF-walks with polls stopped for
          // everyone. Returning to 0x01 is correct while nothing live sits
          // between us and 0x20; if a terminal ever moves back above us,
          // implement the timeout on the bus task before forwarding.
          act.frame[0] = 0x01;
          act.frame[1] = 0x02;
          act.frame[2] = ENROLL_ADDR;
          for (int i = 0; i < 8; i++)
            act.frame[3 + i] = rc_payload_[i];
          // The 8-byte payload is TWO 4-byte fields (ground truth 2026-07-02
          // evening, live A/B tested): the established/probe map first, the
          // CLAIMS field second (address 32 = bit7 of byte 0 down to address
          // 1 = bit0 of byte 3 in each half; the pGD claims 80 00 00 00).
          // Assert our bit in BOTH halves. Measured on the real controller:
          // map-half only -> BIOS establishes + polls us but the app never
          // sees us (never ident-queried/sessioned); claims-half only -> the
          // claim accumulates in the broadcast (claims C0 00 00 00) but we
          // are never established/polled. The pGD's replies are consistent
          // with both-halves (its map bit is always already present, so
          // verbatim-echo vs. echo+bit is indistinguishable in captures).
          // While DRAINING, the task sets claim_mask_ = 0 and this same line
          // RENOUNCES instead (bit cleared in both halves), so the controller
          // removes us cleanly. Branch-free; see claim_mask_.
          uint8_t m = claim_mask_;
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
            if (!ck_ok) {
              ack_ck_fail_ = ack_ck_fail_ + 1;  // stay silent on garble
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

    // Poll service for the enrolled address: ENROLL_ADDR' 01 01 CK. Answer
    // with the pending keypad report (body from tx_frame_, terminal-address
    // byte rewritten to ours) plus our link reply, or the bare link reply.
    if (enroll_ && isr_win_[1] == ENROLL_ADDR && isr_win_[2] == 0x01 && isr_win_[3] == 0x01 &&
        isr_win_[4] == static_cast<uint8_t>(0xFF - 0x02 - ENROLL_ADDR)) {
      const uint8_t lr[4] = {0x01, 0x01, ENROLL_ADDR, static_cast<uint8_t>(0xFD - ENROLL_ADDR)};
      if (tx_pending_ && tx_mode_ == 2) {
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
    tel_last_tx_us_ = now_us;  // arms the post-TX-gap measurement
    switch (act.kind) {
      case TxAction::ROLLCALL_REPLY:
        enroll_replies_ = enroll_replies_ + 1;
        if (drain_)
          drain_replied_ = true;  // the renounce went out; task finishes the leave
        break;
      case TxAction::SESSION_ACK:
        session_acks_ = session_acks_ + 1;
        break;
      case TxAction::IDENT_REPLY:
        ident_replies_ = ident_replies_ + 1;
        break;
      case TxAction::ENROLL_LINK_REPLY:
        enroll_polls_ = enroll_polls_ + 1;
        break;
      case TxAction::ENROLL_KEY_REPLY:
        enroll_polls_ = enroll_polls_ + 1;
        tx_pending_ = false;
        tx_done_us_ = now_us;
        tx_fired_ = true;
        break;
      case TxAction::AFTER_BURST_REPORT:
      case TxAction::RACE_KEY_REPLY:
        tx_pending_ = false;
        tx_done_us_ = now_us;
        tx_fired_ = true;
        break;
      default:
        break;
    }
  }

  // The caller skipped the action (RX FIFO was not empty: the match is stale,
  // the controller has moved on). Only the slot-race path counts these.
  void PLAN_IRAM tx_not_sent(const TxAction &act) {
    if (act.kind == TxAction::RACE_KEY_REPLY)
      isr_stale_ = isr_stale_ + 1;
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
};

}  // namespace plan
