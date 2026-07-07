// Host self-check for the PLANCAP server session (src/plan_cap.h) against a
// real noise-c INITIATOR -- the same exchange planscope's Go client performs
// (docs/capture-protocol.md): banner, Noise NNpsk0 handshake with prologue
// "PLANCAP", encrypted record framing, command dispatch, the explicit
// "Handshake MAC failure" reject, and the keyless plaintext mode.
//
// Needs noise-c (the esphome fork) compiled for the host; see the
// host-tests job in .github/workflows/test.yaml for the exact recipe
// (clone https://github.com/esphome/noise-c, compile the ref backend with
// -DNOISE_USE_REFERENCE_BACKEND=1 -DNOISE_USE_LIBSODIUM=0
// -DNOISE_USE_CUSTOM_RAND=0, then link this file against the objects with
// -DPLAN_CAP_NOISE).

#include "../src/plan_cap.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace plan;

// --- a minimal PLANCAP client (the initiator side), for the test only ------

struct Client {
  NoiseHandshakeState *hs{nullptr};
  NoiseCipherState *tx{nullptr}, *rx{nullptr};

  ~Client() {
    if (hs != nullptr)
      noise_handshakestate_free(hs);
    if (tx != nullptr)
      noise_cipherstate_free(tx);
    if (rx != nullptr)
      noise_cipherstate_free(rx);
  }

  // Handshake message 1, framed: 0x01 <len> 0x00 <noise msg>.
  std::vector<uint8_t> hello(const uint8_t *psk) {
    NoiseProtocolId nid;
    memset(&nid, 0, sizeof(nid));
    nid.pattern_id = NOISE_PATTERN_NN;
    nid.cipher_id = NOISE_CIPHER_CHACHAPOLY;
    nid.dh_id = NOISE_DH_CURVE25519;
    nid.prefix_id = NOISE_PREFIX_STANDARD;
    nid.hash_id = NOISE_HASH_SHA256;
    nid.modifier_ids[0] = NOISE_MODIFIER_PSK0;
    assert(noise_handshakestate_new_by_id(&hs, &nid, NOISE_ROLE_INITIATOR) == NOISE_ERROR_NONE);
    assert(noise_handshakestate_set_pre_shared_key(hs, psk, 32) == NOISE_ERROR_NONE);
    assert(noise_handshakestate_set_prologue(hs, CAP_BANNER, sizeof(CAP_BANNER)) ==
           NOISE_ERROR_NONE);
    assert(noise_handshakestate_start(hs) == NOISE_ERROR_NONE);
    uint8_t msg[96];
    NoiseBuffer nb;
    noise_buffer_set_output(nb, msg, sizeof(msg));
    assert(noise_handshakestate_write_message(hs, &nb, nullptr) == NOISE_ERROR_NONE);
    std::vector<uint8_t> f = {0x01, static_cast<uint8_t>((nb.size + 1) >> 8),
                              static_cast<uint8_t>(nb.size + 1), 0x00};
    f.insert(f.end(), msg, msg + nb.size);
    return f;
  }

  // Consume the server's handshake reply frame (must be success) and split.
  void finish(const std::vector<uint8_t> &frame) {
    assert(frame.size() >= 4);
    assert(frame[0] == 0x01);
    size_t blen = (frame[1] << 8) | frame[2];
    assert(frame.size() == 3 + blen);
    assert(frame[3] == 0x00);  // success status byte
    NoiseBuffer nb;
    noise_buffer_set_input(nb, const_cast<uint8_t *>(frame.data()) + 4, blen - 1);
    assert(noise_handshakestate_read_message(hs, &nb, nullptr) == NOISE_ERROR_NONE);
    assert(noise_handshakestate_get_action(hs) == NOISE_ACTION_SPLIT);
    assert(noise_handshakestate_split(hs, &tx, &rx) == NOISE_ERROR_NONE);
    noise_handshakestate_free(hs);
    hs = nullptr;
  }

  // Encrypt one record into a 0x01-framed wire chunk.
  std::vector<uint8_t> seal(const std::vector<uint8_t> &rec) {
    std::vector<uint8_t> buf = rec;
    buf.resize(buf.size() + CAP_NOISE_MAC);
    NoiseBuffer nb;
    noise_buffer_set_inout(nb, buf.data(), rec.size(), buf.size());
    assert(noise_cipherstate_encrypt(tx, &nb) == NOISE_ERROR_NONE);
    std::vector<uint8_t> f = {0x01, static_cast<uint8_t>(nb.size >> 8),
                              static_cast<uint8_t>(nb.size)};
    f.insert(f.end(), buf.data(), buf.data() + nb.size);
    return f;
  }

  // Decrypt one 0x01-framed chunk back into a record.
  std::vector<uint8_t> open(const uint8_t *frame, size_t flen, size_t *consumed) {
    assert(flen >= 3);
    assert(frame[0] == 0x01);
    size_t blen = (frame[1] << 8) | frame[2];
    assert(flen >= 3 + blen);
    std::vector<uint8_t> buf(frame + 3, frame + 3 + blen);
    NoiseBuffer nb;
    noise_buffer_set_inout(nb, buf.data(), blen, blen);
    assert(noise_cipherstate_decrypt(rx, &nb) == NOISE_ERROR_NONE);
    buf.resize(nb.size);
    *consumed = 3 + blen;
    return buf;
  }
};

// Split a plaintext (0x00-framed) chunk.
static std::vector<uint8_t> open_plain(const uint8_t *frame, size_t flen, size_t *consumed) {
  assert(flen >= 3);
  assert(frame[0] == 0x00);
  size_t blen = (frame[1] << 8) | frame[2];
  assert(flen >= 3 + blen);
  *consumed = 3 + blen;
  return std::vector<uint8_t>(frame + 3, frame + 3 + blen);
}

struct Cmd {
  uint8_t id, op, arg;
};

int main() {
  const uint8_t psk[32] = {0x0a, 0x0b, 0x0c, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                           10,   11,   12,   13, 14, 15, 16, 17, 18, 19, 20, 21,
                           22,   23,   24,   25, 26, 27, 28, 29};

  // --- keyed session: banner, handshake, records both ways ---------------
  {
    PlanCapSession s;
    std::vector<Cmd> cmds;
    s.on_command = [&](uint8_t id, uint8_t op, uint8_t arg) { cmds.push_back({id, op, arg}); };

    std::vector<uint8_t> out;
    s.begin(psk, out);
    assert(out.size() == 7 && memcmp(out.data(), "PLANCAP", 7) == 0);
    assert(!s.established());
    out.clear();

    Client c;
    auto m1 = c.hello(psk);
    // Dribble the handshake in one-byte feeds: framing must reassemble.
    for (uint8_t b : m1)
      assert(s.feed(&b, 1, out));
    assert(s.established() && s.keyed());
    c.finish(out);
    out.clear();

    // Server -> client: a pairs record and an event record, encrypted.
    std::vector<uint8_t> rec;
    cap_rec_pairs_begin(rec, 7, 1234, 0, 2);
    rec.insert(rec.end(), {0x20, 0x01, 0x0B, 0x00});
    assert(s.send_record(rec.data(), rec.size(), out));
    rec.clear();
    cap_rec_event(rec, 1250, 1 /* state */, 0x03, 2);
    assert(s.send_record(rec.data(), rec.size(), out));

    size_t used = 0;
    auto r1 = c.open(out.data(), out.size(), &used);
    assert(r1.size() == 19 && r1[0] == CAP_REC_PAIRS);
    assert(r1[1] == 7 && r1[2] == 0 && r1[3] == 0 && r1[4] == 0);      // seq LE
    assert(r1[5] == 0xD2 && r1[6] == 0x04);                            // ts 1234 LE
    assert(r1[13] == 2 && r1[14] == 0);                                // n
    assert(r1[15] == 0x20 && r1[16] == 0x01 && r1[17] == 0x0B);        // pairs
    auto r2 = c.open(out.data() + used, out.size() - used, &used);
    assert(r2.size() == 8 && r2[0] == CAP_REC_EVENT && r2[5] == 1 && r2[6] == 0x03 && r2[7] == 2);
    out.clear();

    // Client -> server: arm command; then an unknown op (surfaces too, the
    // caller acks it with status 2).
    auto cf = c.seal({CAP_REC_CMD, 0x11, CAP_CMD_ARM, 1});
    assert(s.feed(cf.data(), cf.size(), out));
    cf = c.seal({CAP_REC_CMD, 0x12, 0x63, 0});
    assert(s.feed(cf.data(), cf.size(), out));
    assert(cmds.size() == 2);
    assert(cmds[0].id == 0x11 && cmds[0].op == CAP_CMD_ARM && cmds[0].arg == 1);
    assert(cmds[1].id == 0x12 && cmds[1].op == 0x63);

    // The ack the caller would send round-trips.
    rec.clear();
    cap_rec_ack(rec, 1300, 0x11, CAP_ACK_OK);
    assert(s.send_record(rec.data(), rec.size(), out));
    auto ra = c.open(out.data(), out.size(), &used);
    assert(ra.size() == 7 && ra[0] == CAP_REC_ACK && ra[5] == 0x11 && ra[6] == CAP_ACK_OK);
    out.clear();

    // A diagnostic record round-trips with its exact layout.
    rec.clear();
    cap_rec_diag(rec, 1400, CAP_DIAG_WARNING, "key 0x0F rejected: not armed", 28);
    assert(s.send_record(rec.data(), rec.size(), out));
    auto rd = c.open(out.data(), out.size(), &used);
    assert(rd.size() == 8 + 28 && rd[0] == CAP_REC_DIAG && rd[5] == CAP_DIAG_WARNING);
    assert(rd[6] == 28 && rd[7] == 0);
    assert(memcmp(rd.data() + 8, "key 0x0F rejected: not armed", 28) == 0);
    out.clear();

    // A non-command record from the client is fatal (wrong direction).
    cf = c.seal({CAP_REC_EVENT, 0, 0, 0});
    assert(!s.feed(cf.data(), cf.size(), out));
    assert(s.failed());
  }

  // --- wrong PSK: explicit "Handshake MAC failure" reject ----------------
  {
    PlanCapSession s;
    std::vector<uint8_t> out;
    s.begin(psk, out);
    out.clear();

    uint8_t wrong[32];
    memcpy(wrong, psk, 32);
    wrong[0] ^= 0xFF;
    Client c;
    auto m1 = c.hello(wrong);
    assert(!s.feed(m1.data(), m1.size(), out));
    assert(s.failed());
    // The reject frame: 0x01 <len> 0x01 "Handshake MAC failure".
    assert(out.size() >= 4 && out[0] == 0x01 && out[3] == 0x01);
    std::string text(out.begin() + 4, out.end());
    assert(text == "Handshake MAC failure");
  }

  // --- a keyed server rejects the plaintext hello ------------------------
  {
    PlanCapSession s;
    std::vector<uint8_t> out;
    s.begin(psk, out);
    const uint8_t hello[3] = {0x00, 0x00, 0x00};
    assert(!s.feed(hello, 3, out));
  }

  // --- keyless plaintext session ------------------------------------------
  {
    PlanCapSession s;
    std::vector<Cmd> cmds;
    s.on_command = [&](uint8_t id, uint8_t op, uint8_t arg) { cmds.push_back({id, op, arg}); };
    std::vector<uint8_t> out;
    s.begin(nullptr, out);
    assert(out.size() == 7);
    out.clear();

    const uint8_t hello[3] = {0x00, 0x00, 0x00};
    assert(s.feed(hello, 3, out));
    assert(s.established() && !s.keyed());

    std::vector<uint8_t> rec;
    cap_rec_event(rec, 99, 2 /* join */, 0, 0);
    assert(s.send_record(rec.data(), rec.size(), out));
    size_t used = 0;
    auto r = open_plain(out.data(), out.size(), &used);
    assert(r.size() == 8 && r[0] == CAP_REC_EVENT && r[5] == 2);
    out.clear();

    const uint8_t cmd[7] = {0x00, 0x00, 0x04, CAP_REC_CMD, 0x21, CAP_CMD_INJECT_KEY, 0x0F};
    assert(s.feed(cmd, 7, out));
    assert(cmds.size() == 1 && cmds[0].id == 0x21 && cmds[0].arg == 0x0F);

    // A 0x01 frame on an established plaintext session is fatal.
    const uint8_t noise_frame[4] = {0x01, 0x00, 0x01, 0x00};
    assert(!s.feed(noise_frame, 4, out));
  }

  // --- a keyless server rejects a Noise handshake -------------------------
  {
    PlanCapSession s;
    std::vector<uint8_t> out;
    s.begin(nullptr, out);
    out.clear();
    Client c;
    auto m1 = c.hello(psk);
    assert(!s.feed(m1.data(), m1.size(), out));
  }

  // --- framing garbage is fatal -------------------------------------------
  {
    PlanCapSession s;
    std::vector<uint8_t> out;
    s.begin(nullptr, out);
    const uint8_t bad_marker[3] = {0x42, 0x00, 0x00};
    assert(!s.feed(bad_marker, 3, out));
  }
  {
    PlanCapSession s;
    std::vector<uint8_t> out;
    s.begin(nullptr, out);
    const uint8_t oversize[3] = {0x00, 0xFF, 0xFF};  // body length beyond CAP_MAX_IN_FRAME
    assert(!s.feed(oversize, 3, out));
  }
  {
    // A malformed (short) command record is fatal, not acked.
    PlanCapSession s;
    bool got = false;
    s.on_command = [&](uint8_t, uint8_t, uint8_t) { got = true; };
    std::vector<uint8_t> out;
    s.begin(nullptr, out);
    const uint8_t hello[3] = {0x00, 0x00, 0x00};
    assert(s.feed(hello, 3, out));
    const uint8_t shortcmd[6] = {0x00, 0x00, 0x03, CAP_REC_CMD, 0x01, CAP_CMD_ARM};
    assert(!s.feed(shortcmd, 6, out));
    assert(!got);
  }

  printf("test_plan_cap: all assertions passed\n");
  return 0;
}
