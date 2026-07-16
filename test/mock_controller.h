#pragma once

// Host-only mock of the CAREL µPC controller's pLAN link layer, built from the
// ground truth captured off the live bus (see docs/protocol.md for the
// captured behaviors it cites). It emits the controller-side byte
// streams -- poll, roll-call, session frames, the FF-walk link reset -- and
// validates terminal replies exactly the way the real controller was observed
// to: checksums, 9th/address bits, report structure, and the
// report-without-link-reply failure mode. Integration tests run the firmware's
// PlanTerminal state machine against this mock (test_integration.cpp).
//
// Everything structural here cites a captured behavior; nothing is invented:
//   poll            20' 01 01 DD           (sum-to-0xFF; 1F' 01 01 DE for addr 31)
//   link reply      01' 01 ADDR CK         (sum-to-0xFF)
//   key reply       01' 1E 07 ADDR KK NN CC 01' 01 ADDR CK   (report sums to 0xFE)
//   controller ack  01'                    (the idle burst's trailing byte)
//   roll-call       AA' 02 01 <8B bitmap> CK  and its echoed reply with the
//                   terminal's own membership bit claimed
//   session frame   ADDR' TYPE LEN 01 <payload> CK, acked with 01' 03 ADDR CK
//   ident request   ADDR' 50 05 01 CK, answered 01' 51 07 ADDR 0A 17 CK
//                   (boot-attach capture: 20' 50 05 01 89 -> 01' 51 07 20 0A 17 65)
//   link reset      SS' 02 01 FF FF FF FF 00 00 00 00 CK  (the FF-walk)

#include <cstdint>
#include <vector>

namespace plan {
namespace mock {

// One wire byte with its 9th/address bit, i.e. what a 9-bit multidrop UART
// actually exchanges.
struct B {
  uint8_t v;
  uint8_t bit9;
};
using Bytes = std::vector<B>;

inline uint8_t sum8v(const Bytes &f, size_t from, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; i++)
    s += f[from + i].v;
  return s;
}

class MockController {
 public:
  // What handle_reply concluded about a terminal's response-slot reply.
  enum Verdict {
    LINK_OK,       // valid link reply, link stays up
    KEY_ACCEPTED,  // valid keypad report + link reply; keycode recorded
    REPOLL,        // heard garbage -> controller re-polls immediately
    LINK_LOST,     // no/incomplete reply or report without link reply ->
                   // controller resets the link (FF-walk) shortly after
  };

  // --- controller -> bus ---

  // Link-layer poll: ADDR' 01 01 CK, sum-to-0xFF (20' 01 01 DD / 1F' 01 01 DE).
  Bytes emit_poll(uint8_t addr) const {
    return {{addr, 1}, {0x01, 0}, {0x01, 0}, {static_cast<uint8_t>(0xFF - addr - 0x02), 0}};
  }

  // The single ack byte the controller appends after a valid exchange
  // (the idle burst 20' 01 01 DD | 01' 01 20 DD | 01').
  Bytes emit_ack() const { return {{0x01, 1}}; }

  // Membership roll-call: ADDR' 02 01 <8-byte payload> CK, sum-to-0xFF.
  // The payload is TWO 4-byte fields (ground truth 2026-07-02 evening): the
  // established/probe map first, the accumulated terminal CLAIMS second.
  // Idle payload 80 00 00 01 80 00 00 00 = map {controller@1, pGD@32} +
  // claims {pGD@32} (bitmaps MSB-first from address 32 down to address 1).
  Bytes emit_rollcall(uint8_t addr) const {
    Bytes f = {{addr, 1}, {0x02, 0}, {0x01, 0}};
    for (int i = 0; i < 4; i++)
      f.push_back({map_[i], 0});
    for (int i = 0; i < 4; i++)
      f.push_back({claims_[i], 0});
    uint8_t s = 0;
    for (auto &b : f)
      s += b.v;
    f.push_back({static_cast<uint8_t>(0xFF - s), 0});
    return f;
  }

  // Application session frame: ADDR' TYPE LEN 01 <payload> CK where LEN is the
  // total length including CK and the frame sums to 0xFF (the general
  // controller->terminal envelope, e.g. TYPE 0x0B = one text row).
  Bytes emit_session(uint8_t addr, uint8_t type, const std::vector<uint8_t> &payload) const {
    Bytes f = {{addr, 1}, {type, 0}, {static_cast<uint8_t>(payload.size() + 5), 0}, {0x01, 0}};
    for (uint8_t p : payload)
      f.push_back({p, 0});
    uint8_t s = 0;
    for (auto &b : f)
      s += b.v;
    f.push_back({static_cast<uint8_t>(0xFF - s), 0});
    return f;
  }

  // Graphic/session-management frame (types 0x64/0x65/0x66): ADDR' TYPE LEN
  // 01 <payload> CRClo CRChi. These do NOT use the classic sum-to-0xFF check
  // byte -- their trailer is CRC-16/Modbus (poly 0xA001 reflected, init
  // 0xFFFF) over the frame up to the trailer, little-endian (ground truth:
  // planscope's checksum brute-force over the live captures). Deliberately an
  // independent CRC implementation from the firmware's, so the test
  // cross-checks it.
  Bytes emit_session_crc(uint8_t addr, uint8_t type, const std::vector<uint8_t> &payload) const {
    Bytes f = {{addr, 1}, {type, 0}, {static_cast<uint8_t>(payload.size() + 6), 0}, {0x01, 0}};
    for (uint8_t p : payload)
      f.push_back({p, 0});
    uint16_t crc = 0xFFFF;
    for (auto &b : f) {
      crc = static_cast<uint16_t>(crc ^ b.v);
      for (int i = 0; i < 8; i++)
        crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                        : static_cast<uint16_t>(crc >> 1);
    }
    f.push_back({static_cast<uint8_t>(crc & 0xFF), 0});
    f.push_back({static_cast<uint8_t>(crc >> 8), 0});
    return f;
  }

  // Terminal-type identification request: ADDR' 50 05 01 CK (sum-to-0xFF).
  // The controller sends it right before opening a terminal's session (and
  // rarely in steady state); a terminal must answer with its ident REPLY,
  // not a bare ack.
  Bytes emit_ident(uint8_t addr) const {
    Bytes f = {{addr, 1}, {0x50, 0}, {0x05, 0}, {0x01, 0}};
    uint8_t s = 0;
    for (auto &b : f)
      s += b.v;
    f.push_back({static_cast<uint8_t>(0xFF - s), 0});
    return f;
  }

  // The first frame of the link-reset recovery walk:
  // SS' 02 01 FF FF FF FF 00 00 00 00 CK.
  Bytes emit_link_reset() {
    needs_reset_ = false;
    Bytes f = {{0x02, 1}, {0x02, 0}, {0x01, 0}, {0xFF, 0}, {0xFF, 0}, {0xFF, 0},
               {0xFF, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0}, {0x00, 0}};
    uint8_t s = 0;
    for (auto &b : f)
      s += b.v;
    f.push_back({static_cast<uint8_t>(0xFF - s), 0});
    link_resets_++;
    return f;
  }

  // --- bus -> controller (validate the terminal's reply) ---

  // Validate the reply to emit_poll(addr). Ground truth: the idle reply is the
  // 4-byte link reply 01' 01 ADDR CK; a key press prepends the 7-byte keypad
  // report 01' 1E 07 ADDR KK NN CC (report bytes sum to 0xFE). A keypad report
  // WITHOUT the trailing link reply is received but the controller still
  // expects the link reply for that poll -> link reset ~2 s later.
  Verdict handle_reply(uint8_t addr, const Bytes &r) {
    if (r.empty()) {
      needs_reset_ = true;
      return LINK_LOST;
    }
    size_t i = 0;
    bool got_key = false;
    // Optional keypad report: 01' 1E 07 ADDR KK NN CC.
    if (r.size() >= 7 && r[0].v == 0x01 && r[0].bit9 == 1 && r[1].v == 0x1E) {
      if (r.size() < 7 || r[1].bit9 != 0 || r[2].v != 0x07 || r[3].v != addr ||
          sum8v(r, 1, 6) != 0xFE)
        return REPOLL;
      keys_.push_back(r[4].v);
      got_key = true;
      i = 7;
    }
    // Mandatory link reply: 01' 01 ADDR CK, sum-to-0xFF.
    if (r.size() - i != 4 || r[i].v != 0x01 || r[i].bit9 != 1 || r[i + 1].v != 0x01 ||
        r[i + 2].v != addr || sum8v(r, i, 4) != 0xFF) {
      needs_reset_ = true;
      return LINK_LOST;
    }
    return got_key ? KEY_ACCEPTED : LINK_OK;
  }

  // Validate the reply to emit_rollcall(addr): the terminal echoes the frame
  // back as 01' 02 ADDR <map | own bit> <claims | own bit> CK (sum-to-0xFF).
  // Live A/B (2026-07-02): the bit must be asserted in BOTH halves -- map
  // half alone gets BIOS polling without app visibility, claims half alone
  // gets the claim broadcast without establishment/polls. On success the
  // controller adopts the address (starts polling it) and rebroadcasts with
  // the bit in both halves -- e.g. enrolling 31 turned the map 80 00 00 01
  // into C0 00 00 01 on the live bus.
  bool handle_rollcall_reply(uint8_t addr, const Bytes &r) {
    // Ring semantics: the terminal always returns the token to the
    // controller (0x01) -- blind-forwarding to a possibly-dead 0x20 looped
    // the live bus (see ROLLCALL_REPLY in plan_terminal.h).
    if (r.size() != 12 || r[0].v != 0x01 || r[0].bit9 != 1 || r[1].v != 0x02 || r[2].v != addr)
      return false;
    if (sum8v(r, 0, 12) != 0xFF)  // sum-to-0xFF over the whole 12-byte frame
      return false;
    uint8_t own_byte = (32 - addr) / 8;
    uint8_t own_bit = static_cast<uint8_t>(1u << (7 - ((32 - addr) % 8)));
    for (int i = 0; i < 4; i++) {
      uint8_t expect_map = map_[i];
      uint8_t expect_claims = claims_[i];
      if (i == own_byte) {
        expect_map |= own_bit;
        expect_claims |= own_bit;
      }
      if (r[3 + i].v != expect_map || r[7 + i].v != expect_claims)
        return false;
    }
    map_[own_byte] |= own_bit;
    claims_[own_byte] |= own_bit;
    enrolled_.push_back(addr);
    return true;
  }

  // Validate the ack to emit_session(addr, ...): 01' 03 ADDR CK, sum-to-0xFF
  // (the pGD acks everything with 01' 03 20 DB).
  bool handle_session_ack(uint8_t addr, const Bytes &r) const {
    return r.size() == 4 && r[0].v == 0x01 && r[0].bit9 == 1 && r[1].v == 0x03 &&
           r[2].v == addr && sum8v(r, 0, 4) == 0xFF;
  }

  // Validate the reply to emit_ident(addr): 01' 51 07 ADDR TT VV CK,
  // sum-to-0xFF, where TT VV must equal the pGD's own identification (0A 17)
  // -- a pCO only sessions terminals of the same type.
  bool handle_ident_reply(uint8_t addr, const Bytes &r) const {
    return r.size() == 7 && r[0].v == 0x01 && r[0].bit9 == 1 && r[1].v == 0x51 &&
           r[2].v == 0x07 && r[3].v == addr && r[4].v == 0x0A && r[5].v == 0x17 &&
           sum8v(r, 0, 7) == 0xFF;
  }

  // --- observable state ---
  const std::vector<uint8_t> &keys() const { return keys_; }
  const std::vector<uint8_t> &enrolled() const { return enrolled_; }
  bool needs_reset() const { return needs_reset_; }
  int link_resets() const { return link_resets_; }

 private:
  // Idle state from the live capture: established map {pGD@32, controller@1},
  // claims field {pGD@32} (the controller itself never claims there).
  uint8_t map_[4] = {0x80, 0x00, 0x00, 0x01};
  uint8_t claims_[4] = {0x80, 0x00, 0x00, 0x00};
  std::vector<uint8_t> keys_;      // keycodes of accepted key presses
  std::vector<uint8_t> enrolled_;  // addresses adopted via roll-call
  bool needs_reset_{false};
  int link_resets_{0};
};

}  // namespace mock
}  // namespace plan
