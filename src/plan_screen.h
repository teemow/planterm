#pragma once

// Pure, platform-free pGD screen reconstructor.
//
// Accumulates the display frames the controller sends into the current
// screen state of BOTH terminals on the bus: the physical pGD (0x20) and
// your own enrolled session (0x1F) -- the latter is what injected keys
// navigate, so it is the screen an automated reader works from.
//
// Feed it the received (byte, bit9) pairs from your transport, one at a
// time, from ordinary task context (no allocation, but no ISR guarantees);
// frames are delimited at the bit9 address marks, so this code sees the
// raw stream and needs none of the lossy-log salvage the planscope
// original carries.
//
// Frame handling (grammar per docs/protocol.md):
//   0x0B  text row      payload = ROW + chars; replaces the whole row
//   0x0C  single cell   payload = ROW COL CHAR (drifting digits, edit mode)
//   0x64  graphic band  inverse-video detection for menu-cursor verification
//   0x65  page sync     clears body bands; rows survive (delta repaint)
//   0x66  session init  every row must be repainted before a read (R-SE-05)
//   0x0D  edit cursor   ROW COL FLAG; ROW > 0 + FLAG 1 = a field has focus
// Classic frames byte-sum to 0xFF; 0x64/0x65/0x66 carry a CRC-16/Modbus LE
// trailer instead.
//
// Trust (wave A4, R-DI-22..26, W-01/W-02/W-04/W-06). The controller never
// resends what only our receiver missed, and its deltas are computed against
// ITS shadow of the terminal, so a row is only as good as its last full
// repaint. row() therefore returns "" (exactly like a never-painted row) for
// a row that is not trusted:
//   - every row after a session init (0x66) until each one is repainted, and
//     after a controller roll-call whose claims no longer hold the terminal
//     (its link was dropped: FF-walk or lockout; the screen is frozen);
//   - the row of a lost frame (bad check or misframe that still names the
//     row) until a 0x0B repaints it -- a later 0x0C cannot heal it (35.9 +
//     lost 36.0 + '1' would read 35.1); a lost frame whose row is unknown
//     distrusts every row; lost_bytes() does the same for transport drops;
//   - a 20-column row (LEN 0x1A, R-DI-07): columns 20-21 are unknown, the
//     padded text would read "Alarm-O" or "6" for 6.1.
// raw_row() keeps the untrusted text for diagnostics.
// settled() is the read gate: quiet, no session init still painting, and --
// once the stream has shown the link layer at all -- the controller still
// transmitting (a link fault is ~2 s of controller silence before the FF-walk,
// which used to pass as 600 ms of quiet).
//
// Host-tested in test/test_plan_screen.cpp by replaying an archived
// menu-walk capture.

#include "plan_frame.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace plan {

// The two terminals whose screens the controller paints.
static constexpr uint8_t SCR_TERM_PGD = 0x20;
static constexpr uint8_t SCR_TERM_ESP = 0x1F;

static constexpr uint8_t SCR_TYPE_TEXT = 0x0B;
static constexpr uint8_t SCR_TYPE_CELL = 0x0C;
static constexpr uint8_t SCR_TYPE_GRAPHIC = 0x64;
static constexpr uint8_t SCR_TYPE_INIT = 0x65;
static constexpr uint8_t SCR_TYPE_CTL = 0x66;
static constexpr uint8_t SCR_TYPE_CURSOR = 0x0D;
static constexpr uint8_t SCR_TYPE_LED = 0x0E;
// Controller address and the link-layer frame types the screen watches.
static constexpr uint8_t SCR_CTRL = 0x01;
static constexpr uint8_t SCR_LL_POLL = 0x01;
static constexpr uint8_t SCR_LL_ROLLCALL = 0x02;
// Row frame lengths: 22 columns (0x1C) or the clipped 20-column mode (0x1A).
static constexpr uint8_t SCR_ROW_LEN = 0x1C;
static constexpr uint8_t SCR_ROW_LEN_CLIPPED = 0x1A;
// The controller transmits every ~24 ms while its links live (poll period
// 24.07 ms, re-poll 19-20 ms, R-LL-02/06); a link fault is 2.00 s of silence
// before the FF-walk (R-LL / A2 T1). Longer than any healthy gap, shorter
// than the 600 ms settle quiet: a settle inside a link fault always fails.
static constexpr uint32_t SCR_CTRL_LIVE_MS = 400;

static constexpr size_t SCR_ROWS = 8;   // pGD text mode is 8 rows...
static constexpr size_t SCR_COLS = 22;  // ...of 22 columns
// CAREL's degree glyph. Kept verbatim in the row text (single byte, so the
// grid stays plain char[]); a field-extraction layer matches 0xDF where a
// renderer would show the UTF-8 '°'.
static constexpr uint8_t SCR_DEGREE = 0xDF;
// Graphics bands at pixel y < 8 are the title bar; y >= 8 is the body.
static constexpr int SCR_TITLE_BAR_PX = 8;
// LEN is a single byte < 250 (planscope's parser bound), so 256 holds any
// valid display frame, graphics included.
static constexpr size_t SCR_FRAME_MAX = 256;
// feed_graphic_ rejects bands with y > 64, so band state is indexed by y.
static constexpr int SCR_BAND_MAX_Y = 64;

// Charset mapping (planscope rowText): printable ASCII and the degree glyph
// pass through, everything else renders as '.'.
static inline char scr_char(uint8_t c) {
  if (c == SCR_DEGREE)
    return static_cast<char>(SCR_DEGREE);
  if (c >= 0x20 && c < 0x7F)
    return static_cast<char>(c);
  return '.';
}

class PlanScreen {
 public:
  // Terminal index for an address byte: 0 = pGD, 1 = own session, -1 = other.
  static int term_index(uint8_t addr) {
    if (addr == SCR_TERM_PGD)
      return 0;
    if (addr == SCR_TERM_ESP)
      return 1;
    return -1;
  }

  // Feed one received (byte, bit9) pair; now_ms stamps completed frames.
  // Returns true when the visible screen changed: a text-row repaint, a
  // graphics band, or a page sync. Single-cell updates (0x0C) do NOT count
  // -- they are value drift, and live pages emit them at ~1 Hz forever, so
  // counting them would make change-waiters never see a quiet page (they
  // still land in the rows and still advance painted_ms for settle).
  bool feed(uint8_t b, uint8_t bit9, uint32_t now_ms) {
    bool changed = false;
    if (bit9 != 0) {
      changed = finish_(now_ms);
      cur_len_ = 0;
      done_ = 0;
      cur_open_ = true;  // (joining mid-frame at boot: first partial run is skipped)
    }
    if (cur_open_) {
      if (cur_len_ >= SCR_FRAME_MAX)
        cur_open_ = false;  // overlong: not a valid display frame
      else
        cur_[cur_len_++] = b;
    }
    // A frame whose length is known (its LEN byte, or a link frame's fixed
    // size) is digested at its last byte, not at the next frame's address
    // mark: the quiet timer starts up to one poll period (~24 ms) earlier.
    if (cur_open_ && done_ == 0 && cur_len_ == expected_len_()) {
      done_ = cur_len_;
      done_ok_ = true;
      changed |= close_(cur_, cur_len_, now_ms);
    }
    return changed;
  }

  // Row text, NUL-terminated, exactly SCR_COLS chars once painted ("" while
  // never painted -- matching planscope's absent-row semantics -- and while
  // the row is not trusted, see the header).
  const char *row(uint8_t addr, int r) const {
    return row_trusted(addr, r) ? raw_row(addr, r) : "";
  }

  // The model's text regardless of trust (diagnostics, screen dumps).
  const char *raw_row(uint8_t addr, int r) const {
    int t = term_index(addr);
    if (t < 0 || r < 0 || r >= static_cast<int>(SCR_ROWS) || !row_set_[t][r])
      return "";
    return rows_[t][r];
  }

  bool row_trusted(uint8_t addr, int r) const {
    int t = term_index(addr);
    return t >= 0 && r >= 0 && r < static_cast<int>(SCR_ROWS) && row_set_[t][r] &&
           (trusted_[t] >> r & 1) != 0;
  }

  // Rows still owed by a session init or a dropped link (bit r = row r);
  // settled() is false while any is open.
  uint8_t pending(uint8_t addr) const {
    int t = term_index(addr);
    return t < 0 ? 0 : pending_[t];
  }

  // The row holding an open edit focus (0x0D ROW>0 FLAG 1, R-DI-13): its
  // value is a candidate that Esc may restore; -1 = no focus.
  int focus_row(uint8_t addr) const {
    int t = term_index(addr);
    return t < 0 ? -1 : focus_[t];
  }

  // The transport dropped bytes (ring overflow): whatever was lost may have
  // repainted any row of either terminal.
  void lost_bytes() {
    trusted_[0] = trusted_[1] = 0;
    lost_ += 2;
    cur_open_ = false;
  }

  // Whether a text row is covered by an inverse-video graphics band (the pGD
  // paints menu selection and title bars this way).
  bool row_inverse(uint8_t addr, int r) const {
    int t = term_index(addr);
    if (t < 0)
      return false;
    for (int y = 0; y <= SCR_BAND_MAX_Y; y++) {
      const Band &bd = bands_[t][y];
      if (bd.set && bd.inverse && r >= y / 8 && r < (y + bd.h + 7) / 8)
        return true;
    }
    return false;
  }

  // Last display frame for the terminal (ms clock of feed()); 0 = never.
  uint32_t painted_ms(uint8_t addr) const {
    int t = term_index(addr);
    return t < 0 ? 0 : painted_[t];
  }

  // Settle tracking (R-DI-25/26): quiet on the terminal's display session
  // for at least quiet_ms (planscope's settleQuiet is 600 ms) AND no session
  // init still painting AND, once the stream has carried a controller poll
  // or roll-call, the controller still transmitting (silence = link fault,
  // the repaint was cut, not finished). False while never painted -- there
  // is nothing settled to read.
  bool settled(uint8_t addr, uint32_t now_ms, uint32_t quiet_ms) const {
    int t = term_index(addr);
    return t >= 0 && painted_[t] != 0 && now_ms - painted_[t] >= quiet_ms && pending_[t] == 0 &&
           ctrl_live(now_ms);
  }

  // The controller transmitted within SCR_CTRL_LIVE_MS. Always true on a
  // stream that never carried the link layer (display-only replays).
  bool ctrl_live(uint32_t now_ms) const { return !link_seen_ || now_ms - ctrl_ms_ < SCR_CTRL_LIVE_MS; }

  uint32_t frames() const { return frames_; }
  uint32_t bad() const { return bad_; }
  // Display frames lost for a terminal (bad check or misframe of a frame
  // the controller sent to it) plus transport drops.
  uint32_t lost() const { return lost_; }
  // Session inits (0x66) and dropped links seen, per terminal (DV-6).
  uint32_t resessions(uint8_t addr) const {
    int t = term_index(addr);
    return t < 0 ? 0 : resessions_[t];
  }

 protected:
  struct Band {
    uint8_t h{0};
    bool inverse{false};
    bool set{false};
  };

  // CRC-16/Modbus (reflected poly 0xA001, init 0xFFFF). Residue property:
  // over a whole frame INCLUDING its little-endian trailer it yields 0.
  static uint16_t crc16_(const uint8_t *d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
      crc = static_cast<uint16_t>(crc ^ d[i]);
      for (int k = 0; k < 8; k++)
        crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
  }

  // Validate and digest one completed bit9-delimited run:
  //   ADDR TYPE LEN 01 <payload> CK    display frame, LEN = total length
  //   ADDR 01 01 CK                    controller poll (link liveness)
  //   ADDR 02 01 MAP(4) CLAIMS(4) CK   controller roll-call (claims)
  // (For 0x20 frames the pGD's ack trailer 01' 03 20 DB is its own run, so
  // the run is exactly the frame; your own acks never appear in your RX.)
  // Length of the open run once its header shows it (0 = not yet known).
  size_t expected_len_() const {
    if (cur_len_ < 3)
      return 0;
    if (cur_[2] == SCR_CTRL && cur_[1] == SCR_LL_POLL)
      return 4;
    if (cur_[2] == SCR_CTRL && cur_[1] == SCR_LL_ROLLCALL)
      return 12;
    if (cur_len_ >= 4 && cur_[3] == SCR_CTRL && cur_[2] >= 6)
      return cur_[2];
    return 0;
  }

  // The run ended at the next address mark. Not yet digested: digest it now
  // (misframes and frames of unknown length). Digested early but longer:
  // the next frame's address mark was lost, so that frame merged into this
  // run -- it is lost to us (only when the head itself was intact; a tail
  // behind a garbled head is the same garbage).
  bool finish_(uint32_t now_ms) {
    if (!cur_open_)
      return false;
    if (done_ == 0)
      return close_(cur_, cur_len_, now_ms);
    if (done_ok_ && cur_len_ - done_ >= 2) {
      int t = term_index(cur_[done_]);
      if (t >= 0 && cur_[done_ + 1] > 0x03)
        lose_(t, cur_ + done_, cur_len_ - done_);
    }
    return false;
  }

  bool close_(const uint8_t *f, size_t n, uint32_t now_ms) {
    done_ok_ = false;
    if (n < 4)
      return false;
    if (f[2] == SCR_CTRL && ((f[1] == SCR_LL_POLL && n == 4) || (f[1] == SCR_LL_ROLLCALL && n == 12))) {
      if (sum8(f, n) != 0xFF)
        return false;
      link_seen_ = true;
      ctrl_ms_ = now_ms;
      done_ok_ = true;
      if (f[1] == SCR_LL_ROLLCALL)
        rollcall_(f + 7);
      return false;
    }
    int t = term_index(f[0]);
    if (t < 0 || n < 6 || f[1] <= 0x03 || !(f[3] == SCR_CTRL || display_type_(f[1])))
      return false;  // link layer, or a terminal's request (the pGD's type 0x1F)
    uint8_t typ = f[1];
    bool crc_type = typ == SCR_TYPE_GRAPHIC || typ == SCR_TYPE_INIT || typ == SCR_TYPE_CTL;
    if (f[3] != SCR_CTRL || f[2] != n) {
      lose_(t, f, n);  // misframe: a bit9 mark lost or gained mid-frame
      if (n > f[2] && f[2] >= 4 && term_index(f[f[2]]) >= 0 && n - f[2] >= 2)
        lose_(term_index(f[f[2]]), f + f[2], n - f[2]);  // the merged next frame
      return false;
    }
    frames_++;
    bool ck_ok = crc_type ? (crc16_(f, n) == 0) : (sum8(f, n) == 0xFF);
    if (!ck_ok) {
      bad_++;
      lose_(t, f, n);
      return false;
    }
    ctrl_ms_ = now_ms;
    done_ok_ = true;
    // 0x0E (LED/indicator state, blinking every 0.6-1.8 s while an alarm is
    // pending) carries no text: it does not restart the quiet timer, or a
    // blink would starve every settle (R-SE-07).
    if (typ != SCR_TYPE_LED)
      painted_[t] = now_ms;
    switch (typ) {
      case SCR_TYPE_TEXT:
        // payload = ROW + chars (frame minus ADDR TYPE LEN 01 ... CK)
        return set_row_(t, f[4], f + 5, n - 6);
      case SCR_TYPE_CELL:
        // single-cell update ROW COL CHAR: how the controller repaints one
        // drifting digit, and the ONLY repaint an edit-mode value change or
        // a PIN digit gets (planscope ground truth 2026-07-03)
        if (n == 8)
          set_cell_(t, f[4], f[5], f[6]);
        return false;
      case SCR_TYPE_GRAPHIC:
        feed_graphic_(t, f + 4, n - 6);
        return true;
      case SCR_TYPE_INIT:
        // 0x65 is a page-sync marker, NOT a blank-slate init: the burst after
        // it is a DELTA against the current screen (live-hardware ground
        // truth 2026-07-03), so rows survive; body bands clear like on a
        // page turn.
        clear_body_bands_(t);
        return true;
      case SCR_TYPE_CTL:
        // 0x66 only ever opens a session init (R-SE-06): 65 0D 0E 0F and all
        // eight rows follow (R-SE-05, R-DI-15). Until each row is repainted
        // the screen is not readable -- an init cut by a link fault leaves
        // the previous page's rows standing (W-01).
        resession_(t);
        return true;
      case SCR_TYPE_CURSOR:
        if (n == 8)
          focus_[t] = (f[4] > 0 && f[4] < SCR_ROWS && (f[6] & 1) != 0) ? f[4] : -1;
        return false;
      default:
        return false;
    }
  }

  static bool display_type_(uint8_t typ) {
    switch (typ) {
      case 0x0A: case SCR_TYPE_TEXT: case SCR_TYPE_CELL: case SCR_TYPE_CURSOR: case SCR_TYPE_LED:
      case 0x0F: case 0x50: case SCR_TYPE_GRAPHIC: case SCR_TYPE_INIT: case SCR_TYPE_CTL:
        return true;
      default:
        return false;
    }
  }

  // A display frame for terminal t did not arrive intact. Nothing resends
  // what only our receiver missed (R-DI-23), and the controller's next delta
  // assumes the terminal has it: distrust what it could have painted.
  void lose_(int t, const uint8_t *f, size_t n) {
    lost_++;
    switch (f[1]) {
      case SCR_TYPE_TEXT:
      case SCR_TYPE_CELL:
        if (n >= 5 && f[4] < SCR_ROWS) {
          trusted_[t] = static_cast<uint8_t>(trusted_[t] & ~(1u << f[4]));
          return;
        }
        break;
      case SCR_TYPE_CTL:
        resession_(t);  // a session init we did not see
        return;
      case SCR_TYPE_INIT:
      case SCR_TYPE_GRAPHIC:
        clear_body_bands_(t);
        return;
      case SCR_TYPE_CURSOR:
        focus_[t] = -1;
        return;
      case 0x0A: case SCR_TYPE_LED: case 0x0F: case 0x50:
        return;  // no text
      default:
        break;
    }
    trusted_[t] = 0;  // row unknown: any row may have changed
  }

  // Session (re)init or a dropped link: every row is owed a repaint.
  void resession_(int t) {
    if (pending_[t] != 0xFF)
      resessions_[t]++;
    pending_[t] = 0xFF;
    trusted_[t] = 0;
    focus_[t] = -1;
    clear_body_bands_(t);
  }

  // A controller roll-call carries its claims (MSB of byte 0 = address 32
  // ... LSB of byte 3 = address 1). A terminal missing from them has lost
  // its link (an FF-walk starts with no claims, a locked-out pGD stays out):
  // the controller stops painting it and re-inits it on re-adoption
  // (R-SE-10, R-DI-24), so its screen is frozen until then.
  void rollcall_(const uint8_t *claims) {
    static constexpr uint8_t ADDR[2] = {SCR_TERM_PGD, SCR_TERM_ESP};
    for (int t = 0; t < 2; t++) {
      unsigned k = ADDR[t] - 1u;
      if ((claims[3 - k / 8] >> (k % 8) & 1) == 0)
        resession_(t);
    }
  }

  // Replace one row's text (a 0x0B frame repaints the whole row). Reports
  // whether the visible text changed; a row-0 change is a page transition,
  // which clears the body bands so the old page's selection band cannot
  // haunt the new page as a phantom highlight. A 20-column row (LEN 0x1A,
  // R-DI-07) pays the repaint debt but stays untrusted: its columns 20-21
  // are unknown, not spaces.
  bool set_row_(int t, uint8_t r, const uint8_t *chars, size_t n) {
    if (r >= SCR_ROWS)
      return false;
    char text[SCR_COLS + 1];
    for (size_t c = 0; c < SCR_COLS; c++)
      text[c] = c < n ? scr_char(chars[c]) : ' ';
    // ponytail: payloads beyond 22 columns are truncated; the pGD text mode
    // has no wider rows, and planscope has never rendered one.
    text[SCR_COLS] = '\0';
    uint8_t bit = static_cast<uint8_t>(1u << r);
    bool was_trusted = (trusted_[t] & bit) != 0;
    bool trusted = n >= SCR_COLS;
    pending_[t] = static_cast<uint8_t>(pending_[t] & ~bit);
    trusted_[t] = static_cast<uint8_t>(trusted ? (trusted_[t] | bit) : (trusted_[t] & ~bit));
    if (row_set_[t][r] && std::memcmp(rows_[t][r], text, SCR_COLS) == 0)
      return trusted && !was_trusted;
    if (r == 0)
      clear_body_bands_(t);
    std::memcpy(rows_[t][r], text, SCR_COLS + 1);
    row_set_[t][r] = true;
    return true;
  }

  // Paint one character cell. A cell never makes a row trusted: on a row
  // that missed a repaint it would forge a well-formed wrong value (W-02),
  // and a cell into a never-painted row lands on spaces of unknown text.
  bool set_cell_(int t, uint8_t r, uint8_t col, uint8_t ch) {
    if (r >= SCR_ROWS || col >= SCR_COLS)
      return false;
    if (!row_set_[t][r]) {
      std::memset(rows_[t][r], ' ', SCR_COLS);
      rows_[t][r][SCR_COLS] = '\0';
      row_set_[t][r] = true;
    }
    char c = scr_char(ch);
    if (rows_[t][r][col] == c)
      return false;
    rows_[t][r][col] = c;
    return true;
  }

  // Digest one 0x64 graphic-bitmap payload. Ground truth (menu navigation
  // capture 2026-07-02): 7 unknown/fragmentation bytes, then x,y,w,h as
  // 16-bit BE, then vertical-byte pixel data. The pGD renders menu selection
  // (and title bars) as inverse-video bands: a band painted mostly-lit is
  // inverse, mostly-dark is normal.
  // ponytail: fragment 2+ of a split band carries the same x/y/w/h, so
  // last-writer-wins per y is fine -- fragments of an inverse band are all
  // FF-heavy anyway.
  void feed_graphic_(int t, const uint8_t *p, size_t n) {
    if (n < 16)
      return;
    int y = (p[9] << 8) | p[10];
    int w = (p[11] << 8) | p[12];
    int h = (p[13] << 8) | p[14];
    if (w < 40 || h < 8 || h > 32 || y > SCR_BAND_MAX_Y)
      return;  // icons / large-font fragments, not a row band
    size_t px = n - 15;
    size_t set = 0;
    for (size_t i = 15; i < n; i++)
      for (uint8_t v = p[i]; v != 0; v &= v - 1)
        set++;
    Band b;
    b.h = static_cast<uint8_t>(h);
    b.inverse = set * 2 > px * 8;
    b.set = true;
    // Menu invariant: below the title bar at most one band is lit -- the
    // selection. The controller moves it by painting the old band dark and
    // the new one lit; newest lit band wins so a missed dark repaint cannot
    // leave two selections standing.
    if (b.inverse && y >= SCR_TITLE_BAR_PX) {
      for (int y2 = SCR_TITLE_BAR_PX; y2 <= SCR_BAND_MAX_Y; y2++)
        if (y2 != y && bands_[t][y2].inverse)
          bands_[t][y2].set = false;
    }
    bands_[t][y] = b;
  }

  // Drop every graphics band below the title bar (page transition / page
  // sync). Title-bar bands (y < 8) stay -- the status screen repaints row 0
  // every minute without touching its title band.
  void clear_body_bands_(int t) {
    for (int y = SCR_TITLE_BAR_PX; y <= SCR_BAND_MAX_Y; y++)
      bands_[t][y].set = false;
  }

  char rows_[2][SCR_ROWS][SCR_COLS + 1]{};  // [0] = pGD 0x20, [1] = own 0x1F
  bool row_set_[2][SCR_ROWS]{};
  Band bands_[2][SCR_BAND_MAX_Y + 1];
  uint32_t painted_[2]{0, 0};
  uint32_t frames_{0};
  uint32_t bad_{0};

  uint8_t cur_[SCR_FRAME_MAX];
  size_t cur_len_{0};
  bool cur_open_{false};  // false: mid-frame at start, or frame ran overlong

  // Trust state (class END per the W3 layout rule).
  uint8_t trusted_[2]{0, 0};  // bit r: row r repainted intact since its last loss
  uint8_t pending_[2]{0, 0};  // bit r: row r owed by a session init / dropped link
  int8_t focus_[2]{-1, -1};   // row with an open edit focus (0x0D), -1 = none
  uint32_t resessions_[2]{0, 0};
  uint32_t lost_{0};
  uint32_t ctrl_ms_{0};      // last intact controller frame
  bool link_seen_{false};    // the stream carries controller polls/roll-calls
  size_t done_{0};           // bytes of the open run already digested
  bool done_ok_{false};      // ...and they were an intact frame
};

}  // namespace plan
