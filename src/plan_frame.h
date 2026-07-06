#pragma once

// Pure, platform-free CAREL pLAN frame grammar: encode/validate the
// terminal->controller keypad report, match the controller's poll token,
// re-split the raw RS-485 byte stream into atomic frames, and decode the
// 12-byte display/measurement record.
//
// The grammar was reverse-engineered from captures of one uPC controller +
// pGD terminal pair (see docs/protocol.md). Everything here is bytes in,
// bytes out: no UART, no interrupts, no board support. Host-testable with a
// plain C++17 compiler (test/test_plan_frame.cpp, test/test_plan_decode.cpp).

#include <cstddef>
#include <cstdint>

namespace plan {

inline uint8_t sum8(const uint8_t *d, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; i++)
    s += d[i];
  return s;
}

// ---------------------------------------------------------------------------
// Frame-boundary oracle
// ---------------------------------------------------------------------------

// Atomic pLAN frame lengths on the observed bus:
//   6  = keypad report (terminal->controller), whole-frame byte-sum == 0xFE
//   9  = link-layer poll token, sum == 0xFF
//   12 = data frame (sequence keepalive sum 0xFF; display/measurement sum 0xFE)
// Every valid frame's 8-bit byte-sum lands on 0xFE or 0xFF, which is the
// boundary oracle used to re-split the continuous RS485 byte stream.
//
// Returns the length (6/9/12) of a valid atomic frame starting at d[0], or 0 if
// none of the candidate lengths checksums out within the available bytes.
// Shortest valid length wins -- this matches the offline `planscope regroup`
// tool, which re-split a real 17.5 KB capture into 1909 frames with zero
// leftover bytes.
// ponytail: greedy shortest-match split can in theory mis-frame on a
// coincidental sum; validated zero-junk on the reference capture. If a bad
// bus ever mis-frames, the upgrade path is to anchor on the 0x20/0x1E address
// byte before accepting a length.
inline size_t frame_len_at(const uint8_t *d, size_t avail) {
  static const size_t kLens[3] = {6, 9, 12};
  for (size_t k = 0; k < 3; k++) {
    size_t len = kLens[k];
    if (len <= avail) {
      uint8_t s = sum8(d, len);
      if (s == 0xFE || s == 0xFF)
        return len;
    }
  }
  return 0;
}

// ---------------------------------------------------------------------------
// Keypad report (terminal -> controller)
// ---------------------------------------------------------------------------

// Keypad report frame:
//   1E 07 20 KK NN CC        6 bytes, whole-frame byte-sum == 0xFE (mod 256)
//   │  │  │  │  │  └ check byte: make-weight so the 6 bytes sum to 0xFE
//   │  │  │  │  └─── NN  hold/repeat counter (ramps while a key is held)
//   │  │  │  └────── KK  keycode (see PlanKey)
//   │  │  └───────── 0x20 constant
//   │  └──────────── 0x07 keypad-report frame type
//   └─────────────── 0x1E terminal source address (silent at idle)
static constexpr uint8_t KEYPAD_ADDR = 0x1E;
static constexpr uint8_t KEYPAD_TYPE = 0x07;
static constexpr uint8_t KEYPAD_CONST = 0x20;
static constexpr uint8_t KEYPAD_SUM = 0xFE;
static constexpr size_t KEYPAD_LEN = 6;

// The six physical pGD buttons, keycodes confirmed by idle-vs-held capture
// correlation.
enum PlanKey : uint8_t {
  KEY_ESC = 0x01,
  KEY_PRG = 0x06,
  KEY_ALARM = 0x0D,
  KEY_ENTER = 0x0E,
  KEY_UP = 0x0F,
  KEY_DOWN = 0x10,
};

// The controller's 9-byte link-layer poll token, ~93% of idle bus traffic.
// A run of these, or the inter-token silence around them, marks the window a
// terminal transmits into.
static constexpr size_t POLL_LEN = 9;
inline bool is_poll_token(const uint8_t *d, size_t n) {
  static const uint8_t POLL[POLL_LEN] = {0x20, 0x01, 0x01, 0xDD, 0x01, 0x01, 0x20, 0xDD, 0x01};
  if (n < POLL_LEN)
    return false;
  for (size_t i = 0; i < POLL_LEN; i++)
    if (d[i] != POLL[i])
      return false;
  return true;
}

// Build a keypad report for `keycode` with hold counter `hold` into out[6].
// The check byte is derived so the frame sums to 0xFE, exactly matching frames
// captured from the real pGD. Returns the frame length (6).
inline size_t encode_keypad(uint8_t keycode, uint8_t hold, uint8_t out[KEYPAD_LEN]) {
  out[0] = KEYPAD_ADDR;
  out[1] = KEYPAD_TYPE;
  out[2] = KEYPAD_CONST;
  out[3] = keycode;
  out[4] = hold;
  out[5] = static_cast<uint8_t>(KEYPAD_SUM - sum8(out, 5));
  return KEYPAD_LEN;
}

// True iff `f` is a structurally valid keypad report (address/type/const bytes
// and the sum-to-0xFE check). Used by the self-check and to sanity-verify a
// frame before it is put on the wire.
inline bool keypad_frame_ok(const uint8_t *f, size_t len) {
  return len == KEYPAD_LEN && f[0] == KEYPAD_ADDR && f[1] == KEYPAD_TYPE && f[2] == KEYPAD_CONST &&
         sum8(f, KEYPAD_LEN) == KEYPAD_SUM;
}

// The terminal's response-slot reply, ground truth from a 9-bit capture of
// real pGD key presses (' marks the 9th/address bit):
//
//   20' 01 01 DD | ~0.4ms | 01' 1E 07 20 KK NN CC  01' 01 20 DD | 01'
//   controller poll         pGD: keypad report + link reply       ctrl ack
//
// The pGD answers its poll with BOTH frames back-to-back: the 7-byte keypad
// report (addr 01' = controller, then the 6-byte sum-to-0xFE report) and its
// regular 4-byte link reply 01' 01 20 DD (the only thing it sends when idle).
// Sending the keypad report alone is received but resets the link 2 s later:
// the controller still expects the link reply for that poll.
static constexpr size_t REPLY9_LEN = KEYPAD_LEN + 1 + 4;
// bit i set => byte i carries the 9th/address bit (bytes 0 and 7).
static constexpr uint32_t REPLY9_BIT9_MASK = 0x81;
inline size_t encode_reply9(uint8_t keycode, uint8_t hold, uint8_t out[REPLY9_LEN]) {
  out[0] = 0x01;
  encode_keypad(keycode, hold, out + 1);
  out[7] = 0x01;
  out[8] = 0x01;
  out[9] = 0x20;
  out[10] = 0xDD;
  return REPLY9_LEN;
}

// ---------------------------------------------------------------------------
// Display/measurement record decode
// ---------------------------------------------------------------------------

struct DisplayRecord {
  uint8_t selector;  // which display field this value belongs to
  uint16_t value;    // big-endian 16-bit raw reading
};

// A display/measurement frame is exactly:
//   20 0C 08 01 | SEL | HH LL | CK | 01 03 20 DB
// where header+SEL+HH+LL+CK sum to 0xFF (CK is the per-record check byte) and
// the trailer 01 03 20 DB is constant (and itself sums to 0xFF). The reading is
// the big-endian 16-bit HH<<8 | LL. Returns true and fills `out` iff the frame
// is a structurally valid display record.
inline bool decode_display_record(const uint8_t *f, size_t len, DisplayRecord &out) {
  if (len != 12)
    return false;
  if (!(f[0] == 0x20 && f[1] == 0x0C && f[2] == 0x08 && f[3] == 0x01))
    return false;
  if (!(f[8] == 0x01 && f[9] == 0x03 && f[10] == 0x20 && f[11] == 0xDB))
    return false;
  if (sum8(f, 8) != 0xFF)  // header + SEL + HH + LL + CK check byte
    return false;
  out.selector = f[4];
  out.value = (static_cast<uint16_t>(f[5]) << 8) | f[6];
  return true;
}

}  // namespace plan
