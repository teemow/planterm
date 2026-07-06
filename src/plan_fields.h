#pragma once

// Table-driven extraction of named values from reconstructed pGD screens:
// the matcher engine. One FieldSpec row = (page ID, screen row, matcher,
// name, unit). The spec table itself is DEVICE APPLICATION DATA -- it
// encodes what one controller application paints where -- and lives with
// the consumer; this header ships the matcher kinds, the tiny scanners
// behind them, page identity, and the extraction runners.
//
// The matchers are the C++ port of planscope's fields.go regexes.
// std::regex is too heavy for small targets, and every planscope pattern is
// an "anchored label + signed float" or "token" shape, so the regexes
// became a matcher kind per shape (FxKind) plus a few tiny scanners.
// Divergences from the Go regexes are all in the "cannot appear on a real
// screen" corner (e.g. a float parses greedily past one decimal digit) and
// are noted inline.
//
// Rows come from PlanScreen: 22-char NUL-terminated strings ("" while never
// painted), with CAREL's degree glyph kept verbatim as byte 0xDF -- the
// matchers here match 0xDF where planscope's regexes match the UTF-8 '°' it
// renders it as. Units you emit for a consumer like Home Assistant stay
// UTF-8 ("°C").
//
// Page identity (page_of) is the app's own page ID in row 0 (A01, B01, D14,
// ...); the status anchor and the alarm pages carry none and get the
// synthetic IDs "status" and "alarm".
//
// Pages whose layout is NOT fixed-position -- the same page ID showing
// analogue-input label/value row pairs and digital-output rows at varying
// positions as you page through -- are matched by row content instead
// (extract_io). Whether a screen is such a page is application knowledge
// (its row-0 title), so the consumer triggers extract_io itself.
//
// No allocation, no exceptions; host-tested in test/test_plan_fields.cpp.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace plan {

static constexpr size_t FIELDS_ROWS = 8;      // = SCR_ROWS (plan_screen.h)
static constexpr char FIELDS_DEGREE = '\xDF'; // = SCR_DEGREE, kept verbatim in row text
static constexpr size_t FIELDS_VAL_MAX = 23;  // one 22-col row + NUL
static constexpr size_t FIELDS_NAME_MAX = 32; // snake_cased label / probe id + suffix
static constexpr size_t FIELDS_PAGE_MAX = 8;  // "status" + NUL

// --- matcher kinds (one per regex shape in fields.go) -----------------------

enum FxKind : uint8_t {
  FX_LABEL_NUM,   // label, optional spaces, number, required suffix
  FX_LABEL_NUM2,  // label at col 0, then the SECOND number on the row
  FX_NUM,         // first number followed by suffix, anywhere on the row
  FX_NUM_LEAD,    // leading spaces only, then number + suffix
  FX_WORD,        // ^\s*([A-Za-z]\S*)\s*$   -- the row is one word
  FX_WORD_PREFIX, // ^\s*([A-Za-z][A-Za-z-]*) -- leading letters/hyphens token
  FX_TEXT,        // trimmed row text, non-empty; anchored = no leading spaces
  // settings pages (all anchored at column 0, like the Go patterns):
  FX_LABEL_TOK,      // label, spaces, the next non-space token
  FX_LABEL_SEP_WORD, // label, \s*:\s*, a letters-only word
  FX_TIMER_STATE,    // label, hh:mm, spaces, trimmed remainder (scheduler rows)
  FX_TIMER_HH,       // label, hh:mm -- the hh digits (scheduler slot sub-field)
  FX_TIMER_MI,       // label, hh:mm -- the mm digits (scheduler slot sub-field)
};

// One fixed-position field on one page. label/suffix are "" when the kind
// does not use them; anchored means the label must sit at column 0 (the Go
// patterns' `^`). suffix "\xDF" is the on-screen degree glyph.
struct FieldSpec {
  const char *page;
  uint8_t row;
  FxKind kind;
  bool anchored;
  const char *label;
  const char *suffix;
  const char *name;
  const char *unit;
};

// Look a spec up by field name in a consumer's table; nullptr when unknown.
inline const FieldSpec *find_spec(const FieldSpec *specs, size_t nspecs, const char *name) {
  for (size_t i = 0; i < nspecs; i++)
    if (std::strcmp(specs[i].name, name) == 0)
      return &specs[i];
  return nullptr;
}

// --- tiny scanners -----------------------------------------------------------

inline bool fx_digit(char c) { return c >= '0' && c <= '9'; }
inline bool fx_alpha(char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'); }
inline const char *fx_skip_sp(const char *s) {
  while (*s == ' ')
    s++;
  return s;
}

inline void fx_copy(const char *b, const char *e, char *out, size_t outsz) {
  size_t n = static_cast<size_t>(e - b);
  if (n >= outsz)
    n = outsz - 1;  // cannot trigger: rows are 22 chars, buffers 23+
  std::memcpy(out, b, n);
  out[n] = '\0';
}

// Parse a signed decimal number ("-3.7", "157") starting exactly at s; on
// success copy its text to out and return the char right after it.
// ponytail: two divergences from Go's `-?\d+\.\d`, both unpaintable by the
// pGD -- greedy past one decimal digit, and a bare integer accepted where
// the pattern demands a decimal (temps always carry one).
inline const char *fx_num_at(const char *s, char *out, size_t outsz) {
  const char *p = s;
  if (*p == '-')
    p++;
  if (!fx_digit(*p))
    return nullptr;
  while (fx_digit(*p))
    p++;
  if (*p == '.' && fx_digit(p[1])) {
    p++;
    while (fx_digit(*p))
      p++;
  }
  fx_copy(s, p, out, outsz);
  return p;
}

// First number in s immediately followed by suffix ("" = any); returns the
// char right after the suffix.
inline const char *fx_find_num(const char *s, const char *suffix, char *out, size_t outsz) {
  size_t sl = std::strlen(suffix);
  for (; *s; s++) {
    if (!fx_digit(*s) && !(*s == '-' && fx_digit(s[1])))
      continue;
    const char *end = fx_num_at(s, out, outsz);
    if (end != nullptr && std::strncmp(end, suffix, sl) == 0)
      return end + sl;
  }
  return nullptr;
}

// number right after the label (spaces allowed between), suffix required
inline bool fx_label_num(const char *row, const FieldSpec &sp, char *val, size_t valsz) {
  size_t ll = std::strlen(sp.label);
  size_t sl = std::strlen(sp.suffix);
  if (sp.anchored) {
    if (std::strncmp(row, sp.label, ll) != 0)
      return false;
    const char *end = fx_num_at(fx_skip_sp(row + ll), val, valsz);
    return end != nullptr && std::strncmp(end, sp.suffix, sl) == 0;
  }
  for (const char *p = row; (p = std::strstr(p, sp.label)) != nullptr; p++) {
    const char *end = fx_num_at(fx_skip_sp(p + ll), val, valsz);
    if (end != nullptr && std::strncmp(end, sp.suffix, sl) == 0)
      return true;
  }
  return false;
}

inline bool fx_match(const FieldSpec &sp, const char *row, char *val, size_t valsz) {
  switch (sp.kind) {
    case FX_LABEL_NUM:
      return fx_label_num(row, sp, val, valsz);
    case FX_LABEL_NUM2: {
      // Go's `^Plant:.*°C?\s+(-?\d+\.\d)` resolves to the second number on
      // the row; port it as exactly that.
      size_t ll = std::strlen(sp.label);
      if (std::strncmp(row, sp.label, ll) != 0)
        return false;
      char first[FIELDS_VAL_MAX];
      const char *after = fx_find_num(row + ll, "", first, sizeof first);
      return after != nullptr && fx_find_num(after, "", val, valsz) != nullptr;
    }
    case FX_NUM:
      return fx_find_num(row, sp.suffix, val, valsz) != nullptr;
    case FX_NUM_LEAD: {
      const char *end = fx_num_at(fx_skip_sp(row), val, valsz);
      return end != nullptr && std::strncmp(end, sp.suffix, std::strlen(sp.suffix)) == 0;
    }
    case FX_WORD: {
      const char *p = fx_skip_sp(row);
      if (!fx_alpha(*p))
        return false;
      const char *q = p;
      while (*q != '\0' && *q != ' ')
        q++;
      if (*fx_skip_sp(q) != '\0')
        return false;  // only trailing spaces may follow the word
      fx_copy(p, q, val, valsz);
      return true;
    }
    case FX_WORD_PREFIX: {
      const char *p = fx_skip_sp(row);
      if (!fx_alpha(*p))
        return false;
      const char *q = p;
      while (fx_alpha(*q) || *q == '-')
        q++;
      fx_copy(p, q, val, valsz);
      return true;
    }
    case FX_TEXT: {
      const char *p = fx_skip_sp(row);
      if (*p == '\0' || (sp.anchored && p != row))
        return false;
      const char *q = row + std::strlen(row);
      while (q > p && q[-1] == ' ')
        q--;
      fx_copy(p, q, val, valsz);
      return true;
    }
    case FX_LABEL_TOK: {
      // Go's `^Label:\s*(\S+)`-family: the next token after the label.
      // ponytail: the Go patterns constrain the token's charset ([A-Za-z]+,
      // [\d/]+, ...); any non-space token accepted here -- on the real pages
      // those cells only ever carry the constrained shapes.
      if (std::strncmp(row, sp.label, std::strlen(sp.label)) != 0)
        return false;
      const char *p = fx_skip_sp(row + std::strlen(sp.label));
      if (*p == '\0')
        return false;
      const char *q = p;
      while (*q != '\0' && *q != ' ')
        q++;
      fx_copy(p, q, val, valsz);
      return true;
    }
    case FX_LABEL_SEP_WORD: {
      // Go's `^Label\s*:\s*([A-Za-z]+)` ("NO2 Heatsourcepump:AUT")
      if (std::strncmp(row, sp.label, std::strlen(sp.label)) != 0)
        return false;
      const char *p = fx_skip_sp(row + std::strlen(sp.label));
      if (*p != ':')
        return false;
      p = fx_skip_sp(p + 1);
      if (!fx_alpha(*p))
        return false;
      const char *q = p;
      while (fx_alpha(*q))
        q++;
      fx_copy(p, q, val, valsz);
      return true;
    }
    case FX_TIMER_STATE: {
      // Go's `^F1 \d\d:\d\d\s+(\S.*?)\s*$`: label, hh:mm, then the trimmed
      // remainder ("ENERGY SAVE" carries a space, so not a token match)
      size_t ll = std::strlen(sp.label);
      if (std::strncmp(row, sp.label, ll) != 0)
        return false;
      const char *p = row + ll;
      if (!(fx_digit(p[0]) && fx_digit(p[1]) && p[2] == ':' && fx_digit(p[3]) && fx_digit(p[4])))
        return false;
      p += 5;
      if (*p != ' ')
        return false;
      p = fx_skip_sp(p);
      if (*p == '\0')
        return false;
      const char *q = row + std::strlen(row);
      while (q > p && q[-1] == ' ')
        q--;
      fx_copy(p, q, val, valsz);
      return true;
    }
    case FX_TIMER_HH:
    case FX_TIMER_MI: {
      // One component of a scheduler slot time ("F2 08:30   OFF" ->
      // "08"/"30"): the read-back target of per-sub-field schedule edits,
      // where the edit focus steps hh and mm separately.
      size_t ll = std::strlen(sp.label);
      if (std::strncmp(row, sp.label, ll) != 0)
        return false;
      const char *p = row + ll;
      if (!(fx_digit(p[0]) && fx_digit(p[1]) && p[2] == ':' && fx_digit(p[3]) && fx_digit(p[4])))
        return false;
      const char *b = sp.kind == FX_TIMER_HH ? p : p + 3;
      fx_copy(b, b + 2, val, valsz);
      return true;
    }
  }
  return false;
}

// --- page identity -----------------------------------------------------------

// planscope's pageIDRe: ^[A-Z][A-Za-z]{0,2}\d{2}$ (A01, D14, Gd01, EcH01...)
inline bool fx_page_id(const char *tok, size_t n) {
  if (n < 3 || !(tok[0] >= 'A' && tok[0] <= 'Z'))
    return false;
  size_t i = 1;
  while (i < n && fx_alpha(tok[i]))
    i++;
  return i <= 3 && n - i == 2 && fx_digit(tok[i]) && fx_digit(tok[i + 1]);
}

// planscope's clockRe: ^\d{2}:\d{2} \d{2}/\d{2}/\d{2} (the status anchor row 0)
inline bool fx_clock(const char *r) {
  const char *pat = "dd:dd dd/dd/dd";
  for (int i = 0; pat[i] != '\0'; i++) {
    if (pat[i] == 'd' ? !fx_digit(r[i]) : r[i] != pat[i])
      return false;
  }
  return true;
}

// page_of identifies a screen: the app's page ID from row 0 when present,
// else the synthetic anchors ("status" = clock row, "alarm" = Alarms), else
// "" (menus, unknown pages -- nothing to extract). out must hold
// FIELDS_PAGE_MAX bytes.
inline void page_of(const char *const rows[FIELDS_ROWS], char *out) {
  const char *r0 = rows[0];
  const char *last = nullptr;
  size_t lastn = 0;
  for (const char *p = fx_skip_sp(r0); *p != '\0'; p = fx_skip_sp(p)) {
    const char *q = p;
    while (*q != '\0' && *q != ' ')
      q++;
    last = p;
    lastn = static_cast<size_t>(q - p);
    p = q;
  }
  if (last != nullptr && lastn < FIELDS_PAGE_MAX && fx_page_id(last, lastn)) {
    fx_copy(last, last + lastn, out, FIELDS_PAGE_MAX);
    return;
  }
  if (fx_clock(r0)) {
    std::strcpy(out, "status");
    return;
  }
  const char *p = fx_skip_sp(r0);
  if (std::strncmp(p, "Alarms", 6) == 0 && *fx_skip_sp(p + 6) == '\0') {
    std::strcpy(out, "alarm");
    return;
  }
  out[0] = '\0';
}

// --- content-driven Input/Output scan ------------------------------------------

// snake turns a display label into a stable field name:
// "Compres.1" -> "compres_1", "DHW pump" -> "dhw_pump".
inline void fx_snake(const char *s, const char *end, char *out, size_t outsz) {
  size_t n = 0;
  bool gap = true;  // swallow leading separators
  for (const char *p = s; p < end && n + 1 < outsz; p++) {
    char c = *p;
    if (c >= 'A' && c <= 'Z')
      c += 'a' - 'A';
    if ((c >= 'a' && c <= 'z') || fx_digit(c)) {
      out[n++] = c;
      gap = false;
    } else if (!gap) {
      out[n++] = '_';
      gap = true;
    }
  }
  while (n > 0 && out[n - 1] == '_')
    n--;
  out[n] = '\0';
}

// Digital output, one row: "01=Compres.1:      Off" (Go dout regex
// `^(\d{2})=(.+?)\s*:\s*([A-Za-z]+)\s*$`). Parsed from the right -- state
// word, spaces, the LAST colon -- which is exactly where Go's backtracking
// lands when the label itself contains a colon.
inline bool fx_dout(const char *row, char *name, size_t namesz, char *val, size_t valsz) {
  if (!(fx_digit(row[0]) && fx_digit(row[1]) && row[2] == '='))
    return false;
  const char *end = row + std::strlen(row);
  while (end > row && end[-1] == ' ')
    end--;
  const char *w = end;
  while (w > row && fx_alpha(w[-1]))
    w--;
  if (w == end)
    return false;  // no state word
  const char *c = w;
  while (c > row && c[-1] == ' ')
    c--;
  if (c == row || c[-1] != ':')
    return false;
  const char *lbl = row + 3;
  const char *lble = c - 1;
  while (lble > lbl && lble[-1] == ' ')
    lble--;
  if (lble == lbl)
    return false;  // empty label
  fx_snake(lbl, lble, name, namesz);
  fx_copy(w, end, val, valsz);
  return true;
}

// extract_io scans an Input/Output screen by row content: analogue-probe
// label/value pairs and digital-output rows appear at varying positions
// (and even mixed mid-repaint), so position-keyed extraction cannot work
// here. Probes are named by their probe ID (b1..b12; pressure pages emit
// bNN_press + bNN_temp), digital outputs by their snake_cased label. The
// consumer decides which screens get this scan (their row-0 title is
// application data).
template<typename Emit>
inline void extract_io(const char *page, const char *const rows[FIELDS_ROWS], Emit &&emit) {
  char name[FIELDS_NAME_MAX], val[FIELDS_VAL_MAX], val2[FIELDS_VAL_MAX];
  for (size_t r = 1; r < FIELDS_ROWS; r++) {
    const char *row = rows[r];
    if (fx_dout(row, name, sizeof name, val, sizeof val)) {
      emit(page, name, val, "");
      continue;
    }
    // analogue input: label row "B1 =Heat source out" (Go `^(B\d+)\s*=`),
    // value on the NEXT row -- either "    temp.:      23.3°", a bare
    // "        16.8°", or a pressure+temp pair "    9.7barg .   26.7°"
    // (condensation/evaporation)
    if (row[0] != 'B' || !fx_digit(row[1]))
      continue;
    const char *p = row + 1;
    while (fx_digit(*p))
      p++;
    if (*fx_skip_sp(p) != '=')
      continue;
    char id[8];   // "b" + digits; real probes are B1..B12
    id[0] = 'b';  // ToLower of the "B\d+" probe ID
    fx_copy(row + 1, p, id + 1, sizeof id - 1);
    const char *vrow = r + 1 < FIELDS_ROWS ? rows[r + 1] : "";
    // ponytail: Go allows exactly `\s*.\s*` between the barg and the temp;
    // any gap accepted here -- the pGD only ever paints " . ".
    const char *after = fx_find_num(vrow, "barg", val, sizeof val);
    if (after != nullptr && fx_find_num(after, "\xDF", val2, sizeof val2) != nullptr) {
      std::snprintf(name, sizeof name, "%s_press", id);
      emit(page, name, val, "barg");
      std::snprintf(name, sizeof name, "%s_temp", id);
      emit(page, name, val2, "°C");
    } else if (fx_find_num(vrow, "\xDF", val, sizeof val) != nullptr) {
      emit(page, id, val, "°C");
    }
  }
}

// --- entry point ---------------------------------------------------------------

// extract_fields runs a consumer's spec table over one reconstructed screen
// and emits each match as emit(page, name, value, unit) -- all pointers
// valid only for the duration of the call. Output order is deterministic:
// table order. Content-driven pages additionally go through extract_io,
// triggered by the consumer.
template<typename Emit>
inline void extract_fields(const FieldSpec *specs, size_t nspecs,
                           const char *const rows[FIELDS_ROWS], Emit &&emit) {
  char page[FIELDS_PAGE_MAX];
  page_of(rows, page);
  char val[FIELDS_VAL_MAX];
  for (size_t i = 0; i < nspecs; i++) {
    const FieldSpec &sp = specs[i];
    if (std::strcmp(sp.page, page) != 0)
      continue;
    if (fx_match(sp, rows[sp.row], val, sizeof val))
      emit(page, sp.name, val, sp.unit);
  }
}

}  // namespace plan
