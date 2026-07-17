// Host integration test: the firmware's PlanTerminal state machine (the exact
// code the UART RX ISR runs, plan_terminal.h) driven against a mock CAREL µPC
// controller (mock_controller.h) built from the live-bus captures. No hardware
// or esp-idf needed. Run with:
//   c++ -std=c++17 test/test_integration.cpp -o /tmp/t && /tmp/t
//
// Covered end-to-end (all byte-level, both directions checksum-validated):
//   * idle silence: an un-enrolled, idle terminal never transmits
//   * roll-call enrollment at address 31 (membership bit claimed, map updated)
//   * enrolled link maintenance (polls answered, zero resets over many cycles)
//   * key-press injection in the enrolled slot (report re-addressed to 31)
//   * session-frame acknowledgement (the pGD-style 01' 03 ADDR CK ack)
//   * checksum-gated acks: garbled frames (both grammars: sum-to-0xFF and
//     CRC-16/Modbus for 0x64/65/66) get silence, clean frames get acked
//   * slot-racing mode (tx_mode 0) acceptance and the re-poll rejection signal
//   * the report-without-link-reply failure -> FF-walk -> link_reset_ detection

#include "../src/plan_terminal.h"
#include "mock_controller.h"

#include <cassert>
#include <cstdio>

using namespace plan;
using mock::B;
using mock::Bytes;
using mock::MockController;

// Simulated half-duplex bus: delivers a controller frame to the terminal byte
// by byte (advancing wire time ~192 us per byte = 12 bit-slots at 62500 baud)
// and collects whatever the terminal transmits into the response slot.
struct Bus {
  PlanTerminal term;
  int64_t now = 1'000'000;

  Bytes feed(const Bytes &frame) {
    Bytes reply;
    for (const auto &b : frame) {
      now += 192;
      TxAction act = term.on_byte(b.v, b.bit9, now);
      if (act.kind != TxAction::NONE) {
        now += 250;  // the firmware's turnaround delay before raising DE
        for (size_t i = 0; i < act.len; i++)
          reply.push_back({act.frame[i], static_cast<uint8_t>((act.bit9_mask >> i) & 1)});
        now += 192 * act.len;
        term.tx_sent(act, now);
      }
    }
    return reply;
  }
};

static void arm_key(PlanTerminal &t, uint8_t keycode) {
  uint8_t f[REPLY9_LEN];
  encode_reply9(keycode, 0x01, f);
  for (size_t i = 0; i < REPLY9_LEN; i++)
    t.tx_frame_[i] = f[i];
  t.tx_fired_ = false;
  t.tx_pending_ = true;
}

int main() {
  // --- an un-enrolled, idle terminal never transmits ---
  {
    Bus bus;
    MockController ctl;
    assert(bus.feed(ctl.emit_poll(0x20)).empty());
    assert(bus.feed(ctl.emit_rollcall(0x1F)).empty());
    assert(bus.feed(ctl.emit_ack()).empty());
  }

  // --- TX log ring: every sent/skipped action is recorded verbatim ---
  {
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;
    assert(bus.term.txlog_w_ == 0);
    Bytes reply = bus.feed(ctl.emit_rollcall(0x1F));
    assert(!reply.empty());
    assert(bus.term.txlog_w_ == 1);
    const PlanTerminal::TxLog &e = bus.term.txlog_[0];
    assert(e.sent == 1);
    assert(e.kind == TxAction::ROLLCALL_REPLY);
    assert(e.us != 0);
    assert(e.len == reply.size());
    for (size_t i = 0; i < reply.size(); i++) {
      assert(e.frame[i] == reply[i].v);
      assert(((e.bit9 >> i) & 1) == reply[i].bit9);
    }
    // A skipped action lands too, flagged unsent.
    TxAction skipped;
    skipped.kind = TxAction::RACE_KEY_REPLY;
    skipped.len = 2;
    skipped.frame[0] = 0x01;
    skipped.frame[1] = 0x02;
    skipped.bit9_mask = 0x1;
    bus.term.tx_not_sent(skipped);
    assert(bus.term.txlog_w_ == 2);
    assert(bus.term.txlog_[1].sent == 0);
    assert(bus.term.txlog_[1].us == 0);
    assert(bus.term.txlog_[1].kind == TxAction::RACE_KEY_REPLY);
  }

  // --- poll-token chain forwarding (dual-terminal, heatpump-firmware#14) ---
  {
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;
    bus.term.fwd_polls_ = 1;

    // pGD not live yet: polls get the normal link reply, no forward.
    Bytes r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r.size() == 4 && r[0].v == 0x01 && r[2].v == ENROLL_ADDR);

    // Arm the liveness probe: a pGD session ack (01' 03 20 DB) + a closing
    // address byte (frame classification happens on the next bit9 byte).
    bus.feed({{0x01, 1}, {0x03, 0}, {0x20, 0}, {0xDB, 0}});
    bus.feed(ctl.emit_ack());

    // Now the poll token is handed on to the pGD: 20' 01 1F BF.
    r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r.size() == 4);
    assert(r[0].v == 0x20 && r[0].bit9 == 1 && r[1].v == 0x01 && r[2].v == ENROLL_ADDR);
    assert(r[3].v == static_cast<uint8_t>(0xFF - 0x20 - 0x01 - ENROLL_ADDR));

    // Completion: the pGD answers the token as itself (01' 01 20 DD -- the
    // live-verified variant, fwd-experiment-0937.txt).
    bus.feed({{0x01, 1}, {0x01, 0}, {0x20, 0}, {0xDD, 0}});
    assert(bus.term.fwd_ok_ == 1);

    // Alternation: the controller re-polls right after the pGD's reply and
    // that one is ours to answer directly -- never two forwards in a row.
    r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r.size() == 4 && r[0].v == 0x01 && r[2].v == ENROLL_ADDR);  // direct
    r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r[0].v == 0x20);  // forwarded again

    // No completion this time -> the re-poll gets the normal link reply and
    // the failure backs forwarding off for 1 s.
    r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r.size() == 4 && r[0].v == 0x01 && r[2].v == ENROLL_ADDR);  // normal reply
    assert(bus.term.fwd_fail_ == 1);
    r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r[0].v == 0x01);  // still backing off

    // Backoff expires -> forwards resume; completion variant (b): the pGD
    // returns the token to US (1F' 01 20 CK) and we produce the controller's
    // completion, the mirror of its original poll.
    bus.now += 2'000'000;
    r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(r[0].v == 0x20);  // forwarded
    r = bus.feed({{ENROLL_ADDR, 1},
                  {0x01, 0},
                  {0x20, 0},
                  {static_cast<uint8_t>(0xFF - ENROLL_ADDR - 0x01 - 0x20), 0}});
    assert(r.size() == 4 && r[0].v == 0x01 && r[1].v == 0x01 && r[2].v == ENROLL_ADDR);
    assert(bus.term.fwd_ok_ == 2);
  }

  // --- enrollment, link maintenance, key injection, session ack ---
  {
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;
    bus.term.tx_mode_ = 2;

    // Roll-calls for other addresses are ignored.
    assert(bus.feed(ctl.emit_rollcall(0x0B)).empty());

    // A pGD@32 link reply on the bus arms the liveness probe (gates the
    // ring-token forward; the mock map carries the pGD's bit).
    Bytes pgd_beacon{{0x01, 1}, {0x01, 0}, {0x20, 0}, {0xDD, 0}};
    bus.feed(pgd_beacon);

    // Our roll-call: the reply asserts our bit in BOTH halves (live A/B:
    // map-half alone = polled but app-invisible, claims-half alone = claimed
    // but never established).
    Bytes rc_reply = bus.feed(ctl.emit_rollcall(ENROLL_ADDR));
    assert(!rc_reply.empty());
    assert(rc_reply[3].v == 0xC0);  // map half: pGD's 80 + our 40
    assert(rc_reply[7].v == 0xC0);  // claims half: pGD's 80 + our 40
    assert(ctl.handle_rollcall_reply(ENROLL_ADDR, rc_reply));
    assert(ctl.enrolled().size() == 1 && ctl.enrolled()[0] == ENROLL_ADDR);
    assert(bus.term.enroll_replies_ == 1);
    // Live-bus ground truth: enrolling 31 turns the broadcast map 80 00 00 01
    // into C0 00 00 01.
    assert(ctl.emit_rollcall(ENROLL_ADDR)[3].v == 0xC0);

    // Idle polls to our address: bare link reply, exactly 01' 01 1F DE.
    Bytes lr = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(lr.size() == 4);
    assert(lr[0].v == 0x01 && lr[0].bit9 == 1 && lr[1].v == 0x01 && lr[2].v == 0x1F &&
           lr[3].v == 0xDE);
    assert(ctl.handle_reply(ENROLL_ADDR, lr) == MockController::LINK_OK);
    bus.feed(ctl.emit_ack());

    // Stable for many cycles: zero contention, zero resets.
    for (int i = 0; i < 100; i++) {
      Bytes r = bus.feed(ctl.emit_poll(ENROLL_ADDR));
      assert(ctl.handle_reply(ENROLL_ADDR, r) == MockController::LINK_OK);
      bus.feed(ctl.emit_ack());
      bus.now += 30'000;  // ~30 polls/s as measured live
    }
    assert(!ctl.needs_reset() && ctl.link_resets() == 0);
    assert(bus.term.enroll_polls_ == 101);

    // A poll to the pGD's address must not tempt us, even with a key pending.
    arm_key(bus.term, KEY_DOWN);
    assert(bus.feed(ctl.emit_poll(0x20)).empty());
    assert(bus.term.tx_pending_);

    // Our own slot: keypad report (re-addressed to 31) + link reply in one
    // burst; the mock validates both and records the keycode.
    Bytes kr = bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(kr.size() == REPLY9_LEN);
    assert(kr[3].v == ENROLL_ADDR);  // report attributed to our terminal
    assert(ctl.handle_reply(ENROLL_ADDR, kr) == MockController::KEY_ACCEPTED);
    assert(ctl.keys().size() == 1 && ctl.keys()[0] == KEY_DOWN);
    assert(bus.term.tx_fired_ && !bus.term.tx_pending_);
    bus.feed(ctl.emit_ack());

    // The next poll goes back to the bare link reply (a tap is ONE report).
    bus.term.tx_fired_ = false;
    assert(ctl.handle_reply(ENROLL_ADDR, bus.feed(ctl.emit_poll(ENROLL_ADDR))) ==
           MockController::LINK_OK);

    // Session frame (a 0x0B text row, the Phase 2 display grammar) is acked
    // with 01' 03 1F CK -- the shape the pGD acks everything with.
    std::vector<uint8_t> row = {0x02, ' ', ' ', ' ', ' ', 'H', 'o', 't', 'w', 'a', 't',
                                'e',  'r', ':', ' ', ' ', ' ', '3', '8', '.', '0', 0xDF,
                                'C'};
    Bytes ack = bus.feed(ctl.emit_session(ENROLL_ADDR, 0x0B, row));
    assert(ctl.handle_session_ack(ENROLL_ADDR, ack));
    assert(bus.term.session_acks_ == 1);
    // Session frames for the pGD are not ours to ack.
    assert(bus.feed(ctl.emit_session(0x20, 0x0B, row)).empty());

    // The type-identification request (0x50) gets the ident REPLY -- the
    // pGD's exact type bytes re-addressed to us (01' 51 07 1F 0A 17 66),
    // NOT a bare session ack. This is what the controller sends before
    // opening a session once terminal 31 is in its Trm list.
    Bytes ident = bus.feed(ctl.emit_ident(ENROLL_ADDR));
    assert(ident.size() == 7);
    assert(ident[6].v == 0x66);  // 0xFF - (01+51+07+1F+0A+17)
    assert(ctl.handle_ident_reply(ENROLL_ADDR, ident));
    assert(bus.term.ident_replies_ == 1);
    assert(bus.term.session_acks_ == 1);  // unchanged: no ack for 0x50
    // The pGD's own ident request is not ours to answer.
    assert(bus.feed(ctl.emit_ident(0x20)).empty());

    // --- checksum-gated session acks ---
    // Graphic/session-mgmt frames (0x64/0x65/0x66) carry a CRC-16/Modbus LE
    // trailer instead of the sum-to-0xFF check byte; a valid one is acked
    // like any session frame (the mock's CRC is an independent
    // implementation, so this cross-checks the firmware's).
    std::vector<uint8_t> gfx = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    Bytes gack = bus.feed(ctl.emit_session_crc(ENROLL_ADDR, 0x64, gfx));
    assert(ctl.handle_session_ack(ENROLL_ADDR, gack));
    assert(bus.term.session_acks_ == 2);
    assert(bus.term.ack_ck_fail_ == 0);

    // A garbled classic frame (payload bit flip -> sum != 0xFF) completes
    // byte-count-wise but gets NO ack: silence, counted in ack_ck_fail_.
    // (Live A/B 2026-07-03: the controller answers that silence with an
    // FF-walk link reset after ~2 s, NOT a resend -- the recovery walk
    // re-enrolls us and the session re-init repaints the screen, which is
    // the heavier form of self-healing. See ack_ck_fail_ in plan_terminal.h
    // for why the gate stays.)
    Bytes bad_row = ctl.emit_session(ENROLL_ADDR, 0x0B, row);
    bad_row[6].v ^= 0x40;
    assert(bus.feed(bad_row).empty());
    assert(bus.term.session_acks_ == 2);
    assert(bus.term.ack_ck_fail_ == 1);

    // Same for a garbled CRC-type frame (trailer no longer matches).
    Bytes bad_gfx = ctl.emit_session_crc(ENROLL_ADDR, 0x65, gfx);
    bad_gfx[5].v ^= 0x01;
    assert(bus.feed(bad_gfx).empty());
    assert(bus.term.ack_ck_fail_ == 2);

    // A garbled ident request is not answered either.
    Bytes bad_ident = ctl.emit_ident(ENROLL_ADDR);
    bad_ident[3].v ^= 0x08;
    assert(bus.feed(bad_ident).empty());
    assert(bus.term.ident_replies_ == 1);
    assert(bus.term.ack_ck_fail_ == 3);

    // A clean frame after garble is acked normally: the gate recovers
    // per-frame, no stuck state.
    Bytes resent = bus.feed(ctl.emit_session(ENROLL_ADDR, 0x0B, row));
    assert(ctl.handle_session_ack(ENROLL_ADDR, resent));
    assert(bus.term.session_acks_ == 3);
    assert(bus.term.ack_ck_fail_ == 3);
  }

  // --- slot-racing mode (tx_mode 0): acceptance and the re-poll rejection ---
  {
    Bus bus;
    MockController ctl;
    bus.term.tx_mode_ = 0;

    // pGD silent (post-link-reset backoff): our raced reply is accepted.
    arm_key(bus.term, KEY_PRG);
    Bytes r = bus.feed(ctl.emit_poll(0x20));
    assert(r.size() == REPLY9_LEN);
    assert(r[3].v == 0x20);  // in this mode the report keeps the pGD's address
    assert(ctl.handle_reply(0x20, r) == MockController::KEY_ACCEPTED);
    assert(ctl.keys().back() == KEY_PRG);
    assert(bus.term.tx_fired_);

    // A poll hard on the heels of our TX = the controller heard garbage
    // (collision with the pGD's reply): the rejection signal the task's
    // retry logic keys off.
    assert(!bus.term.tx_rejected_);
    bus.feed(ctl.emit_poll(0x20));
    assert(bus.term.tx_rejected_);

    // A poll long after our TX is NOT a rejection.
    bus.term.tx_rejected_ = false;
    arm_key(bus.term, KEY_UP);
    bus.feed(ctl.emit_poll(0x20));
    bus.now += 40'000;
    bus.feed(ctl.emit_poll(0x20));
    assert(!bus.term.tx_rejected_);
  }

  // --- failure modes the captures pinned down ---
  {
    Bus bus;
    MockController ctl;

    // Keypad report WITHOUT the link reply: received, but the controller
    // still expects the link reply for that poll -> link reset ~2 s later.
    uint8_t rep[REPLY9_LEN];
    encode_reply9(KEY_ESC, 0x01, rep);
    Bytes report_only;
    for (size_t i = 0; i < KEYPAD_LEN + 1; i++)
      report_only.push_back({rep[i], static_cast<uint8_t>(i == 0)});
    assert(ctl.handle_reply(0x20, report_only) == MockController::LINK_LOST);
    assert(ctl.needs_reset());

    // The FF-walk the controller then runs is detected by the terminal.
    assert(!bus.term.link_reset_);
    bus.feed(ctl.emit_link_reset());
    assert(bus.term.link_reset_);
    assert(ctl.link_resets() == 1);

    // A corrupted reply (bad check byte) -> immediate re-poll, not a reset.
    Bytes bad;
    for (size_t i = 0; i < REPLY9_LEN; i++)
      bad.push_back({rep[i], static_cast<uint8_t>(((REPLY9_BIT9_MASK >> i) & 1))});
    bad[5].v ^= 0x01;
    assert(ctl.handle_reply(0x20, bad) == MockController::REPOLL);

    // No reply at all -> link lost.
    assert(ctl.handle_reply(0x20, {}) == MockController::LINK_LOST);
  }

  // --- Phase 0 telemetry: per-address frame counts, checksum failures,
  // --- post-TX gap (the objective garble/collision instrumentation) ---
  {
    Bus bus;
    MockController ctl;

    // A run is closed (classified + checksum-checked) by the NEXT address
    // byte, so after 10 polls only 9 are counted; the 10th run is still open.
    for (int i = 0; i < 10; i++)
      bus.feed(ctl.emit_poll(0x20));
    assert(bus.term.tel_frames_pgd_ == 9);
    assert(bus.term.tel_cksum_fail_ == 0);

    // A corrupted frame on the wire (bad byte -> sum != 0xFF) is the garble
    // detector's positive case.
    Bytes bad = ctl.emit_poll(0x20);
    bad[3].v ^= 0x01;
    bus.feed(bad);
    bus.feed(ctl.emit_ack());  // closes the corrupt run
    assert(bus.term.tel_frames_pgd_ == 11);
    assert(bus.term.tel_cksum_fail_ == 1);

    // The single-byte 01' ack carries no checksum: counted (as a frame to the
    // controller), never flagged.
    bus.feed(ctl.emit_ack());
    assert(bus.term.tel_frames_ctrl_ == 1);
    assert(bus.term.tel_cksum_fail_ == 1);

    // Frames to other addresses land in the catch-all bucket.
    bus.feed(ctl.emit_rollcall(0x0B));
    bus.feed(ctl.emit_ack());
    assert(bus.term.tel_frames_other_ == 1);
    assert(bus.term.tel_cksum_fail_ == 1);

    // CRC-16-trailer types (0x64/65/66) never byte-sum to 0xFF and must NOT
    // be flagged (live 2026-07-16: 331 phantom failures on a host-verified
    // 0-failure bus, all repaint traffic). Real session-init frame from that
    // capture: 20' 65 0F 01 01 00*8 9C 46.
    Bytes init{{0x20, 1}, {0x65, 0}, {0x0F, 0}, {0x01, 0}, {0x01, 0},
               {0x00, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0},
               {0x00, 0}, {0x00, 0}, {0x00, 0}, {0x9C, 0}, {0x46, 0}};
    assert(sum8v(init, 0, init.size()) != 0xFF);  // would false-positive
    bus.feed(init);
    bus.feed(ctl.emit_ack());  // closes the run
    assert(bus.term.tel_frames_pgd_ == 12);
    assert(bus.term.tel_cksum_fail_ == 1);  // unchanged: not garble

    // A garbled CLASSIC frame after a CRC-type run still counts (tel_type_
    // is re-captured per run, no stale exemption).
    Bytes bad2 = ctl.emit_poll(0x20);
    bad2[2].v ^= 0x04;
    bus.feed(bad2);
    bus.feed(ctl.emit_ack());
    assert(bus.term.tel_cksum_fail_ == 2);
  }
  {
    // Post-TX gap: armed after our own transmission, measured on the next RX
    // byte. Our enrolled link replies (and the polls to us) also count into
    // the per-address buckets.
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;
    bus.feed(ctl.emit_rollcall(ENROLL_ADDR));
    assert(bus.term.tel_post_tx_gap_min_us_ == 0);  // nothing after our TX yet
    bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(bus.term.tel_post_tx_gap_min_us_ > 0);  // poll followed our reply
    uint32_t first = bus.term.tel_post_tx_gap_min_us_;
    bus.now += 30'000;
    bus.feed(ctl.emit_poll(ENROLL_ADDR));
    assert(bus.term.tel_post_tx_gap_min_us_ == first);  // min is sticky
    bus.feed(ctl.emit_ack());
    assert(bus.term.tel_frames_us_ >= 1);
    assert(bus.term.tel_cksum_fail_ == 0);  // clean bus: zero failures
  }

  {
    // Graceful disenroll (drain): the link stays fully alive -- polls are
    // still answered -- but the roll-call reply RENOUNCES our membership bit
    // in both payload halves (claim_mask_ = 0, set by the task), so the
    // controller removes us without a missed-poll link fault. NOTE (live
    // 2026-07-03): the controller never roll-calls ESTABLISHED members, so
    // on a healthy bus this only triggers during FF-walk recovery; the task
    // falls back to a deadline hard stop otherwise.
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;
    Bytes beacon{{0x01, 1}, {0x01, 0}, {0x20, 0}, {0xDD, 0}};
    bus.feed(beacon);  // arm the pGD liveness probe
    assert(ctl.handle_rollcall_reply(ENROLL_ADDR, bus.feed(ctl.emit_rollcall(ENROLL_ADDR))));
    assert(ctl.handle_reply(ENROLL_ADDR, bus.feed(ctl.emit_poll(ENROLL_ADDR))) ==
           MockController::LINK_OK);

    bus.term.claim_mask_ = 0;  // task side of the drain (set_enroll false)
    bus.term.drain_ = true;
    assert(ctl.handle_reply(ENROLL_ADDR, bus.feed(ctl.emit_poll(ENROLL_ADDR))) ==
           MockController::LINK_OK);  // still answering
    assert(!bus.term.drain_replied_);

    // Roll-call while draining: echo with our bit CLEARED in both halves
    // (ctl's map/claims carry our bit from the enrollment above).
    Bytes renounce = bus.feed(ctl.emit_rollcall(ENROLL_ADDR));
    assert(renounce.size() == 12 && renounce[0].v == 0x01 && renounce[0].bit9 == 1 &&
           renounce[1].v == 0x02 && renounce[2].v == ENROLL_ADDR);
    assert((renounce[3 + OWN_BYTE_I].v & OWN_BIT) == 0);  // map half renounced
    assert((renounce[7 + OWN_BYTE_I].v & OWN_BIT) == 0);  // claims half renounced
    assert(sum8v(renounce, 0, 12) == 0xFF);               // checksum still valid
    assert(bus.term.drain_replied_);                      // task finishes the leave

    // Task-side finalize (mirrors task_main): enroll off, polls go dark.
    bus.term.enroll_ = false;
    bus.term.drain_ = false;
    assert(bus.feed(ctl.emit_poll(ENROLL_ADDR)).empty());
  }

  // --- Roll-call token ring (ground truth 2026-07-16): tokens arrive from
  // --- ANY ring member, and the reply forwards to the next live member ---
  {
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;

    // Token forwarded by a pGD at 0x1E (exact live frame, ck 0x7F): presence
    // echoed VERBATIM (the forwarding chain owns it -- the pGD's own cold
    // join behaves this way), our bit claimed in the CLAIMS half only,
    // token returned to 0x01.
    Bytes token{{0x1F, 1}, {0x02, 0}, {0x1E, 0}, {0x20, 0}, {0x00, 0}, {0x00, 0},
                {0x01, 0}, {0x20, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0}, {0x7F, 0}};
    Bytes r = bus.feed(token);
    assert(r.size() == 12 && r[0].v == 0x01 && r[0].bit9 == 1 && r[1].v == 0x02 &&
           r[2].v == ENROLL_ADDR);
    assert(r[3].v == 0x20 && r[7].v == (0x20 | OWN_BIT));
    assert(sum8v(r, 0, 12) == 0xFF);

    // Presence-bit7 token WITHOUT any pGD transmission ever seen: the 32 is
    // presumed EMPTY (assume-all-alive FF-walk presence) -- return to 0x01;
    // blind-forwarding into an empty 32 looped the live bus 2026-07-16.
    Bytes token32{{0x1F, 1}, {0x02, 0}, {0x1E, 0}, {0xE0, 0}, {0x00, 0}, {0x00, 0},
                  {0x01, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0}, {0xFF, 0}};
    uint8_t s = 0;
    for (size_t i = 0; i < 11; i++)
      s += token32[i].v;
    token32[11].v = static_cast<uint8_t>(0xFF - s);
    r = bus.feed(token32);
    assert(r.size() == 12 && r[0].v == 0x01);
    // Honest skip: the dead 32's presence bit is CLEARED in the return
    // (E0 -> 60), exactly like the pGD's own skip frames -- returning it
    // intact link-faulted the controller (01:04 loop).
    assert(r[3].v == 0x60 && r[7].v == OWN_BIT);
    assert(sum8v(r, 0, 12) == 0xFF);

    // The pGD transmits (link reply from 0x20) -> liveness armed -> the
    // token STILL returns to 0x01 (a real terminal ignores member-forwarded
    // tokens; forwarding throws the claim away -- ground truth 07:43), but
    // the LIVE 32 keeps its presence bit intact.
    Bytes beacon{{0x01, 1}, {0x01, 0}, {0x20, 0}, {0xDD, 0}};
    bus.feed(beacon);
    r = bus.feed(token32);
    assert(r.size() == 12 && r[0].v == 0x01 && r[0].bit9 == 1);
    assert(r[3].v == 0xE0 && r[7].v == OWN_BIT);  // presence intact, claims-only
    assert(sum8v(r, 0, 12) == 0xFF);

    // Liveness expires (15 s without a pGD transmission): return + clear.
    bus.now += 16'000'000;
    r = bus.feed(token32);
    assert(r.size() == 12 && r[0].v == 0x01 && r[3].v == 0x60);

    // Corrupted token (checksum no longer matches the sender byte): silence.
    Bytes bad = token;
    bad[2].v = 0x1D;  // sender byte garbled, ck now wrong
    assert(bus.feed(bad).empty());
  }

  // --- Poll token ring (ground truth 2026-07-16 21:21:17): with two
  // --- established terminals the polls chain member-to-member too ---
  {
    Bus bus;
    MockController ctl;
    bus.term.enroll_ = true;

    // The exact captured frame: pGD@1E hands us the poll token (ck C1).
    // The reply returns ALONG THE CHAIN -- back to the forwarder (0x1E),
    // which aggregates and produces its own return to the controller.
    Bytes poll1e{{0x1F, 1}, {0x01, 0}, {0x1E, 0}, {0xC1, 0}};
    Bytes r = bus.feed(poll1e);
    assert(r.size() == 4 && r[0].v == 0x1E && r[0].bit9 == 1 && r[1].v == 0x01 &&
           r[2].v == ENROLL_ADDR);
    assert(r[3].v == 0xC1);  // 0xFF - 1E - 01 - 1F (sum-symmetric with the token)

    // A pending key report rides a member-forwarded poll token like any poll.
    bus.term.tx_frame_[0] = 0x01;
    bus.term.tx_frame_[1] = 0x1E;
    bus.term.tx_frame_[2] = 0x07;
    bus.term.tx_frame_[3] = 0x20;
    bus.term.tx_frame_[4] = 0x10;
    bus.term.tx_frame_[5] = 0x01;
    bus.term.tx_frame_[6] = 0xA8;
    bus.term.tx_pending_ = true;
    r = bus.feed(poll1e);
    assert(r.size() == REPLY9_LEN && r[3].v == ENROLL_ADDR);  // report, addr rewritten
    assert(r[7].v == 0x1E);  // appended link reply also returns via the forwarder

    // Garbled sender byte (checksum mismatch): ignored.
    Bytes badp{{0x1F, 1}, {0x01, 0}, {0x1D, 0}, {0xC1, 0}};
    assert(bus.feed(badp).empty());
  }

  std::printf("ok\n");
  return 0;
}
