#pragma once

// PLANCAP transport session, server side (docs/capture-protocol.md).
//
// Sans-IO: the caller owns the socket and the outgoing byte queue. This
// class turns received bytes into parsed command records (via on_command)
// and turns records into framed -- and, on a keyed session, Noise-encrypted
// -- wire bytes appended to the output vector. It covers sections 2-4 and
// the client->server half of section 5 of the spec:
//
//   banner "PLANCAP" -> mode selection by the client's first frame
//   (Noise NNpsk0_25519_ChaChaPoly_SHA256 handshake, prologue "PLANCAP",
//   or the plaintext hello on a keyless server) -> record frames
//   0x01/0x00 + u16 BE length.
//
// The Noise implementation is the same noise-c the ESPHome native API
// bundles; compile with PLAN_CAP_NOISE defined to enable the encrypted
// mode (without it only the keyless plaintext mode exists and begin()
// with a key fails). Record LAYOUTS (little-endian, spec section 5) are
// encoded by the cap_rec_* helpers below so both the firmware and the
// host tests share one definition.
//
// Host-tested against a noise-c initiator in test/test_plan_cap.cpp.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

#ifdef PLAN_CAP_NOISE
#include <noise/protocol.h>
#endif

namespace plan {

// Banner: 7 ASCII bytes, unframed, also the Noise prologue (spec 2 + 4).
static constexpr uint8_t CAP_BANNER[7] = {'P', 'L', 'A', 'N', 'C', 'A', 'P'};

// Record types (spec 5). 0x00-0x02 server->client, 0x03 client->server,
// 0x04 server->client.
static constexpr uint8_t CAP_REC_PAIRS = 0x00;
static constexpr uint8_t CAP_REC_EVENT = 0x01;
static constexpr uint8_t CAP_REC_DIAG = 0x02;
static constexpr uint8_t CAP_REC_CMD = 0x03;
static constexpr uint8_t CAP_REC_ACK = 0x04;

// Diagnostic severities (spec 5.3).
static constexpr uint8_t CAP_DIAG_ERROR = 1;
static constexpr uint8_t CAP_DIAG_WARNING = 2;
static constexpr uint8_t CAP_DIAG_INFO = 3;
static constexpr uint8_t CAP_DIAG_DEBUG = 4;

// Command ops (spec 5.4).
static constexpr uint8_t CAP_CMD_ARM = 1;
static constexpr uint8_t CAP_CMD_ENROLL = 2;
static constexpr uint8_t CAP_CMD_INJECT_KEY = 3;

// Ack statuses (spec 5.5).
static constexpr uint8_t CAP_ACK_OK = 0;
static constexpr uint8_t CAP_ACK_REJECTED = 1;
static constexpr uint8_t CAP_ACK_UNKNOWN_OP = 2;

// Largest incoming frame body we accept. Nothing legitimate is bigger: the
// handshake message is 49 bytes (status byte + NNpsk0 message 1), a command
// record is 4 bytes plaintext / 20 bytes ciphertext. Trust boundary: the
// pre-handshake peer is unauthenticated, so an absurd length claim must not
// buffer 64 KiB -- it fails here instead.
static constexpr uint16_t CAP_MAX_IN_FRAME = 128;

static constexpr size_t CAP_NOISE_MAC = 16;  // ChaChaPoly tag per record

// --- record encoding helpers (all integers little-endian, spec 5) ----------

inline void cap_put_u32(std::vector<uint8_t> &v, uint32_t x) {
  v.push_back(static_cast<uint8_t>(x));
  v.push_back(static_cast<uint8_t>(x >> 8));
  v.push_back(static_cast<uint8_t>(x >> 16));
  v.push_back(static_cast<uint8_t>(x >> 24));
}

// Type 0x00 header; the caller appends the n (byte, bit9) pairs.
inline void cap_rec_pairs_begin(std::vector<uint8_t> &rec, uint32_t seq, uint32_t ts_ms,
                                uint32_t drops, uint16_t n) {
  rec.push_back(CAP_REC_PAIRS);
  cap_put_u32(rec, seq);
  cap_put_u32(rec, ts_ms);
  cap_put_u32(rec, drops);
  rec.push_back(static_cast<uint8_t>(n));
  rec.push_back(static_cast<uint8_t>(n >> 8));
}

inline void cap_rec_event(std::vector<uint8_t> &rec, uint32_t ts_ms, uint8_t kind, uint8_t a,
                          uint8_t b) {
  rec.push_back(CAP_REC_EVENT);
  cap_put_u32(rec, ts_ms);
  rec.push_back(kind);
  rec.push_back(a);
  rec.push_back(b);
}

inline void cap_rec_diag(std::vector<uint8_t> &rec, uint32_t ts_ms, uint8_t severity,
                         const char *text, size_t len) {
  rec.push_back(CAP_REC_DIAG);
  cap_put_u32(rec, ts_ms);
  rec.push_back(severity);
  rec.push_back(static_cast<uint8_t>(len));
  rec.push_back(static_cast<uint8_t>(len >> 8));
  rec.insert(rec.end(), reinterpret_cast<const uint8_t *>(text),
             reinterpret_cast<const uint8_t *>(text) + len);
}

inline void cap_rec_ack(std::vector<uint8_t> &rec, uint32_t ts_ms, uint8_t id, uint8_t status) {
  rec.push_back(CAP_REC_ACK);
  cap_put_u32(rec, ts_ms);
  rec.push_back(id);
  rec.push_back(status);
}

// --- the session ------------------------------------------------------------

class PlanCapSession {
 public:
  PlanCapSession() = default;
  ~PlanCapSession() { reset(); }
  PlanCapSession(const PlanCapSession &) = delete;
  PlanCapSession &operator=(const PlanCapSession &) = delete;

  // Parsed client command records (id, op, arg) surface here, from inside
  // feed(). Unknown ops surface too -- the spec wants them acked with
  // status 2, which is the caller's record to send.
  std::function<void(uint8_t id, uint8_t op, uint8_t arg)> on_command;

  // Start a session on a freshly accepted connection: appends the banner to
  // `out`. psk32 = the raw 32-byte key (keyed session) or nullptr (keyless
  // plaintext session). A key without PLAN_CAP_NOISE compiled in fails the
  // session immediately.
  void begin(const uint8_t *psk32, std::vector<uint8_t> &out) {
    reset();
    out.insert(out.end(), CAP_BANNER, CAP_BANNER + sizeof(CAP_BANNER));
#ifdef PLAN_CAP_NOISE
    if (psk32 != nullptr && !init_handshake_(psk32))
      state_ = St::FAILED;
#else
    if (psk32 != nullptr)
      state_ = St::FAILED;
#endif
  }

  bool established() const { return state_ == St::ESTABLISHED; }
  bool failed() const { return state_ == St::FAILED; }
  bool keyed() const { return keyed_; }

  // Feed received client bytes. May append output (the handshake reply or
  // an explicit handshake error frame). Returns false on a fatal protocol /
  // handshake error (spec 7): flush what is queued, then close the socket.
  bool feed(const uint8_t *data, size_t len, std::vector<uint8_t> &out) {
    if (state_ == St::FAILED)
      return false;
    in_.insert(in_.end(), data, data + len);
    for (;;) {
      if (in_.size() < 3)
        return true;
      uint8_t marker = in_[0];
      uint16_t blen = static_cast<uint16_t>((in_[1] << 8) | in_[2]);
      if (marker > 0x01 || blen > CAP_MAX_IN_FRAME)
        return fail_();
      if (in_.size() < static_cast<size_t>(3) + blen)
        return true;
      uint8_t *body = in_.data() + 3;
      bool ok = (state_ == St::WAIT_MODE) ? on_mode_frame_(marker, body, blen, out)
                                          : on_data_frame_(marker, body, blen);
      in_.erase(in_.begin(), in_.begin() + 3 + blen);
      if (!ok)
        return false;
    }
  }

  // Frame (and on a keyed session encrypt) exactly one record into `out`.
  // Only valid once established.
  bool send_record(const uint8_t *rec, size_t len, std::vector<uint8_t> &out) {
    if (state_ != St::ESTABLISHED)
      return false;
    if (!keyed_) {
      out.push_back(0x00);
      out.push_back(static_cast<uint8_t>(len >> 8));
      out.push_back(static_cast<uint8_t>(len));
      out.insert(out.end(), rec, rec + len);
      return true;
    }
#ifdef PLAN_CAP_NOISE
    out.push_back(0x01);
    size_t lpos = out.size();
    out.push_back(0);
    out.push_back(0);
    size_t dpos = out.size();
    out.insert(out.end(), rec, rec + len);
    out.resize(out.size() + CAP_NOISE_MAC);
    NoiseBuffer nb;
    noise_buffer_set_inout(nb, out.data() + dpos, len, len + CAP_NOISE_MAC);
    if (noise_cipherstate_encrypt(tx_, &nb) != NOISE_ERROR_NONE) {
      state_ = St::FAILED;
      return false;
    }
    out[lpos] = static_cast<uint8_t>(nb.size >> 8);
    out[lpos + 1] = static_cast<uint8_t>(nb.size);
    return true;
#else
    return false;  // keyed_ is never true without PLAN_CAP_NOISE
#endif
  }

  // Free crypto state and buffered input; the session is inert until the
  // next begin().
  void reset() {
    state_ = St::WAIT_MODE;
    keyed_ = false;
    in_.clear();
#ifdef PLAN_CAP_NOISE
    if (hs_ != nullptr) {
      noise_handshakestate_free(hs_);
      hs_ = nullptr;
    }
    if (tx_ != nullptr) {
      noise_cipherstate_free(tx_);
      tx_ = nullptr;
    }
    if (rx_ != nullptr) {
      noise_cipherstate_free(rx_);
      rx_ = nullptr;
    }
#endif
  }

 private:
  enum class St : uint8_t { WAIT_MODE, ESTABLISHED, FAILED };

  bool fail_() {
    state_ = St::FAILED;
    return false;
  }

  // The client's first frame selects the mode for the connection (spec 3).
  bool on_mode_frame_(uint8_t marker, uint8_t *body, uint16_t blen, std::vector<uint8_t> &out) {
    (void) out;  // only written on the PLAN_CAP_NOISE path
    if (marker == 0x00) {
      // Plaintext hello: empty body, keyless server only.
      if (keyed_ || blen != 0)
        return fail_();
      state_ = St::ESTABLISHED;
      return true;
    }
    // Noise handshake message 1: keyed server only.
    if (!keyed_ || blen < 1 || body[0] != 0x00)
      return fail_();
#ifdef PLAN_CAP_NOISE
    NoiseBuffer nb;
    noise_buffer_set_input(nb, body + 1, blen - 1);
    int err = noise_handshakestate_read_message(hs_, &nb, nullptr);
    if (err != NOISE_ERROR_NONE) {
      send_handshake_error_(
          err == NOISE_ERROR_MAC_FAILURE ? "Handshake MAC failure" : "Handshake error", out);
      return fail_();
    }
    if (noise_handshakestate_get_action(hs_) != NOISE_ACTION_WRITE_MESSAGE)
      return fail_();
    uint8_t msg[96];
    msg[0] = 0x00;  // success status byte
    noise_buffer_set_output(nb, msg + 1, sizeof(msg) - 1);
    if (noise_handshakestate_write_message(hs_, &nb, nullptr) != NOISE_ERROR_NONE)
      return fail_();
    out.push_back(0x01);
    out.push_back(static_cast<uint8_t>((nb.size + 1) >> 8));
    out.push_back(static_cast<uint8_t>(nb.size + 1));
    out.insert(out.end(), msg, msg + nb.size + 1);
    // NNpsk0 completes in two messages: split into the transport ciphers.
    if (noise_handshakestate_get_action(hs_) != NOISE_ACTION_SPLIT)
      return fail_();
    if (noise_handshakestate_split(hs_, &tx_, &rx_) != NOISE_ERROR_NONE)
      return fail_();
    noise_handshakestate_free(hs_);
    hs_ = nullptr;
    state_ = St::ESTABLISHED;
    return true;
#else
    return fail_();  // unreachable: keyed_ requires PLAN_CAP_NOISE
#endif
  }

  // Established stream: the client may only send command records (spec 5).
  bool on_data_frame_(uint8_t marker, uint8_t *body, uint16_t blen) {
    uint8_t *rec = body;
    size_t rlen = blen;
    if (keyed_) {
#ifdef PLAN_CAP_NOISE
      if (marker != 0x01)
        return fail_();
      NoiseBuffer nb;
      noise_buffer_set_inout(nb, body, blen, blen);
      if (noise_cipherstate_decrypt(rx_, &nb) != NOISE_ERROR_NONE)
        return fail_();
      rlen = nb.size;
#else
      return fail_();
#endif
    } else if (marker != 0x00) {
      return fail_();
    }
    // type u8 + id + op + arg, exactly. Unknown ops are the caller's ack
    // status 2; unknown TYPES and malformed lengths are fatal (spec 7).
    if (rlen != 4 || rec[0] != CAP_REC_CMD)
      return fail_();
    if (on_command)
      on_command(rec[1], rec[2], rec[3]);
    return true;
  }

#ifdef PLAN_CAP_NOISE
  bool init_handshake_(const uint8_t *psk32) {
    NoiseProtocolId nid;
    std::memset(&nid, 0, sizeof(nid));
    // Noise_NNpsk0_25519_ChaChaPoly_SHA256 -- same suite as the ESPHome
    // native API; the prologue "PLANCAP" (vs the API's NoiseAPIInit\0\0)
    // domain-separates the two protocols sharing one PSK.
    nid.pattern_id = NOISE_PATTERN_NN;
    nid.cipher_id = NOISE_CIPHER_CHACHAPOLY;
    nid.dh_id = NOISE_DH_CURVE25519;
    nid.prefix_id = NOISE_PREFIX_STANDARD;
    nid.hash_id = NOISE_HASH_SHA256;
    nid.modifier_ids[0] = NOISE_MODIFIER_PSK0;
    if (noise_handshakestate_new_by_id(&hs_, &nid, NOISE_ROLE_RESPONDER) != NOISE_ERROR_NONE)
      return false;
    if (noise_handshakestate_set_pre_shared_key(hs_, psk32, 32) != NOISE_ERROR_NONE)
      return false;
    if (noise_handshakestate_set_prologue(hs_, CAP_BANNER, sizeof(CAP_BANNER)) != NOISE_ERROR_NONE)
      return false;
    if (noise_handshakestate_start(hs_) != NOISE_ERROR_NONE)
      return false;
    keyed_ = true;
    return true;
  }

  void send_handshake_error_(const char *text, std::vector<uint8_t> &out) {
    size_t tlen = std::strlen(text);
    out.push_back(0x01);
    out.push_back(static_cast<uint8_t>((tlen + 1) >> 8));
    out.push_back(static_cast<uint8_t>(tlen + 1));
    out.push_back(0x01);  // failure status byte
    out.insert(out.end(), reinterpret_cast<const uint8_t *>(text),
               reinterpret_cast<const uint8_t *>(text) + tlen);
  }

  NoiseHandshakeState *hs_{nullptr};
  NoiseCipherState *tx_{nullptr};
  NoiseCipherState *rx_{nullptr};
#endif

  St state_{St::WAIT_MODE};
  bool keyed_{false};
  std::vector<uint8_t> in_;  // unparsed client bytes (bounded by CAP_MAX_IN_FRAME + 3)
};

}  // namespace plan
