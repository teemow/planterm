#pragma once

// The conformance reference: what a pGD-faithful terminal at address `addr`
// answers to each received frame. It is wave row A6's behavioural model of
// the real pGD1 (pgd-model.md: state machine T1-T20, receive decision table
// section 4) plus one normative rule of the roll-call chapter (A1
// roll-call.md R-RC-30: a bridge stays silent on an FF-walk probe while a pGD
// may exist, and joins on the controller's gap walk instead).
//
// It is evaluated SLOT BY SLOT against the recorded input: it does not
// re-simulate the bus, so after a divergence it keeps answering the frames
// the real bus sent (which only exist because the bridge answered the way
// it did). Each answer names the rule it comes from.
//
// Run at addr 0x20 without R-RC-30 it predicts the real pGD, which the
// harness checks against the pGD's own frames in every capture (calibration).

#include "../../src/plan_terminal.h"
#include "frames.h"

#include <cstdint>
#include <string>
#include <vector>

namespace replay {

inline uint8_t sum8(const std::vector<uint8_t> &b, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n && i < b.size(); i++)
    s = static_cast<uint8_t>(s + b[i]);
  return s;
}

inline bool crc_type(uint8_t t) {
  return t == plan::GRAPHIC_TYPE || t == plan::SESSION_INIT_TYPE || t == plan::SESSION_CTL_TYPE;
}

// Short frames (types 01 poll/link reply, 02 roll-call, 03 ack, 05) carry the
// sender in byte 2; long frames carry LEN in byte 2 and the sender in byte 3.
inline bool short_type(uint8_t t) { return t == 0x01 || t == 0x02 || t == 0x03 || t == 0x05; }

inline int frame_type(const Frame &f) { return f.bit9 && f.bytes.size() >= 2 ? f.bytes[1] : -1; }

inline int frame_sender(const Frame &f) {
  int t = frame_type(f);
  if (t < 0)
    return -1;
  if (short_type(static_cast<uint8_t>(t)))
    return f.bytes.size() >= 3 ? f.bytes[2] : -1;
  return f.bytes.size() >= 4 ? f.bytes[3] : -1;
}

// Grammar length of the frame (extra tail bytes excluded), 0 if unknown.
inline size_t grammar_len(const Frame &f) {
  int t = frame_type(f);
  if (t == 0x01 || t == 0x03 || t == 0x05)
    return 4;
  if (t == 0x02)
    return 12;
  if (t >= 0 && f.bytes.size() >= 3)
    return f.bytes[2];
  return 0;
}

inline bool frame_valid(const Frame &f) {
  size_t n = grammar_len(f);
  if (n < 4 || n > f.bytes.size())
    return false;
  if (crc_type(f.bytes[1])) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++)
      crc = plan::crc16_modbus_step(crc, f.bytes[i]);
    return crc == 0;
  }
  return sum8(f.bytes, n) == 0xFF;
}

inline bool is_rollcall(const Frame &f) { return frame_type(f) == 0x02 && f.bytes.size() >= 12; }

// The controller's FF-walk (recovery walk) start: 02' 02 01 FF FF FF FF <any claims>.
// Matching on the presence field only also catches the "warm" walks that
// carry the pGD's claim (A1 R-RC-07, deviation V1/D8).
inline bool is_ff_walk_start(const Frame &f) {
  return is_rollcall(f) && f.to == 0x02 && f.bytes[2] == 0x01 && f.bytes[3] == 0xFF &&
         f.bytes[4] == 0xFF && f.bytes[5] == 0xFF && f.bytes[6] == 0xFF;
}

// A frame put on the wire by the controller (0x01). The bare one-byte ack
// 01' is always the controller's (the pGD never sends it, A6 section 1).
inline bool from_controller(const Frame &f) {
  return f.bit9 && (f.from == 0x01 || (f.bytes.size() == 1 && f.bytes[0] == 0x01));
}

inline uint8_t own_byte(uint8_t addr) { return static_cast<uint8_t>((32 - addr) / 8); }
inline uint8_t own_bit(uint8_t addr) { return static_cast<uint8_t>(1u << (7 - ((32 - addr) % 8))); }

inline std::vector<uint8_t> with_ck(std::vector<uint8_t> f) {
  f.push_back(static_cast<uint8_t>(0xFF - sum8(f, f.size())));
  return f;
}

struct RefOut {
  std::vector<uint8_t> bytes;  // empty = stays silent
  std::string rule;            // why (A6 transition, R-* rule, deviation id)
  std::vector<uint8_t> alt;    // R-RC-30 silence: the echo it would otherwise send
};

class RefTerminal {
 public:
  RefTerminal(uint8_t addr, bool rule_rc30) : addr_(addr), rc30_(rule_rc30) {}

  bool enrolled = true;        // task state: the terminal takes part at all
  bool pgd_may_exist = false;  // R-RC-30 input (fixture header `pgd: present`)
  std::vector<uint8_t> key;    // pending keypad report (7 bytes), sent with the next poll answer

  RefOut on_frame(const Frame &f) {
    // Controller-side sequence tracking, independent of addressing.
    const bool ctrl = from_controller(f);
    const bool rc_to_me = ctrl && is_rollcall(f) && f.to == addr_ && f.bytes[2] == 0x01;
    if (ctrl) {
      if (is_ff_walk_start(f)) {
        in_ff_ = true;
        probed_in_walk_ = false;
      } else if (!is_rollcall(f)) {
        in_ff_ = false;  // polls/display resumed: the walk is over
      }
      run_ = rc_to_me ? (last_ctrl_rc_to_me_ ? run_ + 1 : 1) : 0;
      last_ctrl_rc_to_me_ = rc_to_me;
    }

    RefOut out;
    if (!enrolled || !f.bit9 || f.to != addr_ || f.bytes.size() < 2)
      return out;
    const uint8_t type = f.bytes[1];
    const int sender = frame_sender(f);

    if (type == 0x01) {
      if (!frame_valid(f)) {
        out.rule = "A6:sec4 garbled";
        return out;
      }
      // T7: a poll gets the link reply (keypad report first if pending).
      // T12: a member-forwarded poll is answered AS ITSELF to the controller.
      out.rule = sender == 0x01 ? "A6:T7" : "A6:T12";
      if (!key.empty()) {
        out.bytes = key;
        key.clear();
      }
      auto lr = with_ck({0x01, 0x01, addr_});
      out.bytes.insert(out.bytes.end(), lr.begin(), lr.end());
      return out;
    }

    if (type == 0x02) {
      if (!frame_valid(f)) {
        out.rule = "A6:sec4 garbled";
        return out;
      }
      if (sender != 0x01) {
        out.rule = "A6:T13";  // member-forwarded token: ignored
        return out;
      }
      const bool first_ff_probe = in_ff_ && !probed_in_walk_;
      if (in_ff_)
        probed_in_walk_ = true;
      if (rc30_ && pgd_may_exist && addr_ != 0x20 && first_ff_probe) {
        out.rule = "R-RC-30";  // silent on the FF-walk probe while a pGD may exist
        out.alt = echo(f);     // what R-RC-12 would have answered (tags D2 behind R-RC-30)
        return out;
      }
      if (run_ >= 3) {
        // T5: the third consecutive controller roll-call gets the link reply.
        run_ = 0;
        last_ctrl_rc_to_me_ = false;
        out.rule = "A6:T5";
        out.bytes = with_ck({0x01, 0x01, addr_});
        return out;
      }
      out.rule = run_ == 2 ? "A6:T4" : "A6:T3";
      out.bytes = echo(f);
      return out;
    }

    if (type == 0x05) {
      // R-LL-20: the controller's 20' 05 01 D9 (only after a type-0x1F request) gets a link reply.
      if (sender == 0x01 && frame_valid(f)) {
        out.rule = "R-LL-20";
        out.bytes = with_ck({0x01, 0x01, addr_});
      }
      return out;
    }

    if (sender != 0x01) {
      out.rule = "DV-3";  // a terminal-originated frame: a real terminal never acks it
      return out;
    }
    const bool display = (type >= 0x0A && type <= 0x0F) || crc_type(type);
    if (type != plan::IDENT_REQ_TYPE && !display) {
      out.rule = "R-SE-02 unknown type";
      return out;
    }
    if (!frame_valid(f)) {
      out.rule = "A6:sec4 garbled";  // checksum/CRC bad: no ack
      return out;
    }
    if (type == plan::IDENT_REQ_TYPE) {
      out.rule = "A6:T11";
      out.bytes = with_ck({0x01, 0x51, 0x07, addr_, plan::PGD_IDENT[0], plan::PGD_IDENT[1]});
      return out;
    }
    out.rule = "A6:T10";
    out.bytes = with_ck({0x01, 0x03, addr_});
    return out;
  }

 private:
  // T3/T4 + R-RC-12: echo a controller probe with every received bit kept and
  // the own bit asserted in both halves (MAP and CLAIMS). The pGD's probes
  // already carry its MAP bit, so its echo looks verbatim there; a gap-walk
  // probe to 1F (MAP 80 00 00 01) needs the bit added: all July joins.
  std::vector<uint8_t> echo(const Frame &f) const {
    std::vector<uint8_t> e = {0x01, 0x02, addr_};
    for (int i = 3; i < 11; i++)
      e.push_back(f.bytes[i]);
    e[3 + own_byte(addr_)] = static_cast<uint8_t>(e[3 + own_byte(addr_)] | own_bit(addr_));
    e[3 + 4 + own_byte(addr_)] = static_cast<uint8_t>(e[3 + 4 + own_byte(addr_)] | own_bit(addr_));
    return with_ck(e);
  }

  uint8_t addr_;
  bool rc30_;
  bool in_ff_ = false;
  bool probed_in_walk_ = false;
  bool last_ctrl_rc_to_me_ = false;
  int run_ = 0;
};

}  // namespace replay
