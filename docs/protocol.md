# The pLAN protocol — a terminal's-eye reference

CAREL's **pLAN** is the proprietary RS-485 network that connects pCO/µPC
family HVAC controllers to their pGD display terminals (and to each other).
There is no public specification. Everything in this document was
reverse-engineered from live bus captures of **one** unit — a µPC controller
running a heat-pump application, with a pGD graphic terminal attached — with
the pGD itself as ground truth throughout. Claims below trace to those
captures; where behavior is inferred rather than observed, it says so.
The protocol is *expected* but **unverified** to generalize across the pGD
line and other pLAN devices.

This is a *terminal-role* reference: it describes what a device must send and
understand to behave like a pGD — decode the screen, join the bus as an
additional terminal, press keys. Controller-to-controller pLAN traffic
(multi-pCO networks) is out of scope and was never observed on this bus.

Notation: bytes are hex. A trailing apostrophe marks a byte transmitted with
the 9th bit set (`20'` = byte 0x20, bit9 = 1). `CK` is a checksum byte.

## 1. Physical layer

- **RS-485 half-duplex multidrop**, CAREL multimaster, up to 32 devices.
- **62500 baud** (a divisor-friendly rate; e.g. an ESP32 generates it
  exactly as 80 MHz / 1280).
- **9-bit characters**: start bit + 8 data bits + a 9th bit + stop bit(s).
  The reference captures read the bus as 8N2 (later 8E1 for bit9 recovery),
  and transmitting with 2 stop bits is proven accepted.
- **The 9th bit is the address mark**: bit9 = 1 on the *first* byte of every
  frame (the destination address byte), bit9 = 0 on every payload byte.
  This is classic 9-bit multidrop framing — receiver UARTs filter on the
  address bit, so byte-perfect frames transmitted as plain 8N1/8N2 are
  **silently ignored** by the controller. Any transmit path must be able to
  set bit9 per byte; any receive path that wants to delimit frames reliably
  must recover it.
- One 9-bit character is 11 bit times ≈ **176 µs** at 62500 baud.

Practical transport notes (platform-specific, outside this library's core):

- *Receive*: configure the UART as 8E1 and recover
  `bit9 = even_parity(data) XOR parity_error_flag`. Passive capture also
  works with plain 8N2 — the 9th bit then surfaces as occasional parity or
  framing noise you can ignore, and frames can still be split with the
  byte-sum boundary oracle (`plan::frame_len_at`).
- *Transmit*: select EVEN/ODD parity per byte so the emitted parity bit
  equals the desired bit9.

## 2. Who is who — addresses

The first byte of every frame is the **destination** address (with bit9 set).

| Address | Role |
|---------|------|
| `0x01`  | the pCO/µPC controller (bus master) |
| `0x20`  | terminal 32 — the default pLAN terminal address; the physical pGD in all reference captures |
| `0x1F`  | terminal 31 — the address this project enrolls a second terminal at |
| `0x02`–`0x1E` | remaining pLAN addresses, scanned by the controller's roll-call |

Terminals are addressable 1–32; the roll-call membership bitmap (section 5)
covers exactly 32 addresses. CAREL's own multi-terminal examples use 32 and
31, matching what worked here. (OEM conventions differ; if an application
rejects an address, the pGD's terminal-configuration screen shows what the
controller expects.)

The controller is the master: it polls, roll-calls, and pushes display
content. A terminal is **purely reactive** — the cold-boot capture of the
real pGD shows it transmitting nothing at all until the controller's
roll-call reaches its address (section 8).

## 3. Frame delimiting and checksums

Frames are delimited by the bit9 address marks: everything from one marked
byte up to (not including) the next marked byte is one frame. A single
marked byte with no payload following is a one-byte **ack** (`01'`).

Two checksum grammars coexist on the bus (both verified against captures —
one full-day capture: 5766 display frames, 99.8 % passing):

1. **Classic frames — additive make-weight.** The whole frame byte-sums to
   `0xFF` (mod 256); the last byte is `0xFF - sum(preceding bytes)`. This
   covers polls, link replies, roll-calls, session acks, ident frames, and
   display types other than the three below. One deliberate exception: the
   keypad report *body* sums to `0xFE` (section 6) — its wire form with the
   leading `01'` address byte sums to `0xFF` again.
2. **Graphic/session frames — CRC-16/Modbus.** Display types `0x64`, `0x65`,
   `0x66` end in a CRC-16/Modbus (reflected poly `0xA001`, init `0xFFFF`,
   no final XOR) computed over the whole frame body, appended
   **little-endian**. Residue property: running the CRC over the whole frame
   *including* the trailer yields 0 — convenient for streaming validation.

A checksum failure on a physically sound bus is measured-zero over hours of
captures (tens of thousands of frames), so a failing check is a reliable
wire-corruption detector — provided the checker knows both grammars. A
sum-only checker false-positives on every `0x64/0x65/0x66` burst.

For capture streams without bit9 visibility there is a second, weaker
delimiter: at idle the bus consists of back-to-back atomic frames of a few
fixed sizes, and every valid frame's byte-sum lands on `0xFE` or `0xFF`.
Greedy shortest-match on that property re-split a real 17.5 KB capture into
1909 frames with zero leftover bytes (`plan::frame_len_at`).

## 4. Link layer — the poll/reply/ack heartbeat

Idle bus traffic (~93 % of bytes, ~124 frames/s on the reference unit) is the
controller keeping its terminal link alive:

```
20' 01 01 DD          controller polls terminal 0x20  ("alive?")
      ~420 µs response slot
01' 01 20 DD          terminal's link reply (src 0x20 embedded)
01'                   controller's one-byte ack
```

General forms (both sum to `0xFF`):

- poll: `ADDR' 01 01 CK` — e.g. `1F' 01 01 DE` once terminal 31 is enrolled.
- link reply: `01' 01 SRC CK` — the terminal answers *to* the controller,
  with its own address as the third byte.

Measured properties (idle reference unit):

- The controller polls its terminal ≈ **40×/s**.
- The pGD answers **~420 µs** after the poll's last byte (1568 samples: mean
  421 µs, σ 18 µs, min 384 µs; > 2 ms only while redrawing).
- The controller is **not listening immediately**: replies fired ~64 µs after
  poll end were consistently answered with an immediate re-poll; ~100 µs is
  the observed lower bound, and both ~250 µs and the pGD-identical ~420 µs
  are proven reliable.
- A terminal that misses its polls gets its **link reset ~2 s later** (the
  FF-walk, section 5); the pGD shows "NO LINK".
- The pGD does **not** carrier-sense. It answers its poll at its fixed
  turnaround regardless of what anyone else is transmitting. Racing another
  terminal's response slot therefore collides probabilistically — the reason
  a second terminal must enroll at its own address instead of spoofing
  replies for an existing one.

Failure signatures a transmitting terminal can detect:

- **Immediate re-poll** (same poll again within ~1 ms of your TX): the
  controller heard garbage — usually a collision.
- **Silent discard + FF-walk**: ~2 s of nothing after a clean-looking TX,
  then the recovery walk (section 5). The controller received but rejected
  the exchange. One specific cause, proven by capture: sending a keypad
  report *without* the accompanying link reply (section 6).

## 5. Roll-call — membership, enrollment, link reset

### The roll-call frame

The controller periodically probes pLAN addresses with a 12-byte broadcast
(sum-to-`0xFF`):

```
ADDR' 02 01 <MAP: 4 bytes> <CLAIMS: 4 bytes> CK
```

At idle the controller runs one full walk of the unenrolled addresses about
every 12 s (~2.4 roll-call frames/s on the reference unit). The 8-byte
payload is **two independent 4-byte bitmaps**, MSB-first from address 32
down to address 1
(address 32 = bit 7 of byte 0, address 31 = bit 6 of byte 0, … address 1 =
bit 0 of byte 3):

- **MAP** — the established/probe map. In steady state it holds the current
  membership (`80 00 00 01` = terminal 32 + controller at 1). During a
  recovery walk it is the shrinking probe pattern (below), so it is *not* a
  plain membership list.
- **CLAIMS** — the field each live terminal ORs its own bit into when
  replying. The pGD's claims half is always `80 00 00 00`.

### Joining

A terminal joins by answering the roll-call for its own address, echoing the
payload back with source/destination swapped and **its bit asserted in BOTH
bitmaps** (sum-to-`0xFF`):

```
20' 02 01 80 00 00 01 00 00 00 00 5B     controller probes 0x20
01' 02 20 80 00 00 01 80 00 00 00 DB     terminal echoes, bit claimed
```

The both-halves rule is load-bearing and was established by live A/B:

- claim in the MAP half only → the controller's BIOS establishes and polls
  the terminal forever, but the *application* never sessions it (no ident
  query, no display frames);
- claim in the CLAIMS half only → the claim accumulates in the broadcast but
  the terminal is never established or polled;
- claim in **both** → full membership: within seconds the controller
  rebroadcasts the updated map (`C0 00 00 01` with 31 joined), starts
  polling the new address ~40×/s, ident-queries it (section 7), and opens a
  display session (section 8).

Adoption takes three roll-call rounds inside one burst (probe → echo →
rebroadcast → echo → confirm), after which the terminal falls into the
normal poll/reply cycle.

### Link reset — the FF-walk

When a link faults (a terminal stops answering, or an exchange is rejected),
the controller runs a **recovery walk**: back-to-back roll-calls of every
address, starting from an all-optimistic probe map and clearing the bit of
each address that fails to answer:

```
02' 02 01 FF FF FF FF 00 00 00 00 FE     first frame of the walk
03' 02 01 <shrinking map> ...            ... one probe per address ...
```

With **no** terminal on the bus at all, the controller stops polling
entirely and loops this walk every ~2.3 s. The walk's distinctive prefix
(`.. 02 01 FF FF FF FF 00 00`) is a reliable link-reset detector for a
listening terminal.

### Membership changes trigger a net rebuild

Every membership change (join, leave, rejoin) is followed ~4.5–6.1 s later
by a controller-initiated walk that re-inits **all** terminal sessions:
screens repaint from scratch, in-flight key presses are discarded. A fresh
join is itself a membership change, so one more walk follows the join.
Automation that navigates menus must wait these rebuilds out.

### Leaving

Symmetrically to joining, a terminal renounces by answering its roll-call
with its bit *cleared* in both halves. In practice the opportunity rarely
comes: the controller was never observed roll-calling an
established-and-polled terminal, so a graceful renounce waits indefinitely.
The observed leave path is simply to stop answering polls and let the ~2 s
link fault + FF-walk remove the terminal — the cost is one full-screen
repaint on the surviving terminals.

## 6. Keypad — how key presses travel

Keypad frames do not exist on the bus at idle; a terminal only emits them
while a button is down. The 6-byte report body:

```
1E 07 20 KK NN CC
│  │  │  │  │  └ check byte: make-weight so the 6 bytes sum to 0xFE
│  │  │  │  └─── NN  hold/auto-repeat counter: 0x01 on a tap, ramps while
│  │  │  │            held, saturates at 0xC8
│  │  │  └────── KK  keycode (table below)
│  │  └───────── terminal address (0x20 for the pGD)
│  └──────────── 0x07 keypad-report frame type
└─────────────── 0x1E constant lead byte
```

The `0x1E` lead is constant in every capture; the byte that identifies
*which* terminal pressed the key is the terminal address in the third
position — a terminal enrolled at 31 sends `1E 07 1F KK NN CC` (check byte
re-derived), proven app-accepted live.

| Button | `KK` |
|--------|------|
| Esc    | `0x01` |
| Prg    | `0x06` |
| Alarm  | `0x0D` |
| Enter  | `0x0E` |
| Up     | `0x0F` |
| Down   | `0x10` |

On the wire the report rides **inside the terminal's poll response slot,
back-to-back with the regular link reply**, as one burst with two address
marks (captured from the real pGD with 9-bit visibility):

```
20' 01 01 DD | ~0.4 ms | 01' 1E 07 20 KK NN CC  01' 01 20 DD | 01'
controller poll          keypad report + link reply             ack
```

Sending the keypad report *alone* is received but the controller still
expects the link reply for that poll — it silently discards the exchange and
resets the link ~2 s later. Report + link reply together in the slot is the
only accepted form. An enrolled terminal injecting a key rewrites the
report's `0x20` constant to its own terminal address and re-derives `CC`
(sum-to-`0xFE`), so the press is attributed to the terminal the poll
addressed.

Key presses from a properly enrolled, sessioned terminal are accepted at the
application level — menus navigate, and the resulting screen update is
pushed to *all* sessioned terminals. Median press-to-repaint latency on the
reference unit: ~250 ms (the controller's own repaint time).

## 7. Identification — the 0x50/0x51 exchange

Shortly after adopting a terminal (and rarely in steady state — twice in
~48 min on the reference unit), the controller sends a type-identification
request; the terminal answers with a 2-byte identity (both sum-to-`0xFF`):

```
20' 50 05 01 89              ident request (LEN=05)
01' 51 07 20 0A 17 65        ident reply: src 0x20, payload 0A 17
```

The payload is presumably terminal type + version; the pGD in the reference
captures reports `0A 17`. This exchange matters because CAREL documents that
a controller only serves terminals of the **same type** ("the controller
cannot manage different kinds of terminals at the same time") — a second
terminal that wants a session should echo the resident pGD's exact identity
bytes, re-addressed to itself. It is also the only frame besides link
replies, roll-call echoes, and acks in which a terminal sends *content*.

The ident request is answered with the ident reply **instead of** the usual
session ack (section 8) — ground truth: the pGD sends no ack for it.

## 8. Display session — how the screen gets painted

### Envelope

Every controller→terminal display frame shares one envelope:

```
ADDR' TYPE LEN 01 <payload...> CK
```

- `ADDR` — destination terminal (`0x20`, or `0x1F` for an enrolled second
  terminal, each getting its **own private session**).
- `LEN` — total frame length in bytes, i.e. everything from `ADDR` through
  the checksum (observed < 250).
- `CK` — sum-to-`0xFF` check byte, **except** types `0x64/0x65/0x66`, which
  carry a 2-byte CRC-16/Modbus LE trailer instead (section 3).

The terminal acknowledges **every** display frame with a 4-byte session ack
(sum-to-`0xFF`):

```
01' 03 SRC CK        the pGD's: 01' 03 20 DB
```

In a capture, this ack appears on the wire immediately after every display
frame to the pGD, so without bit9 visibility it looks like a constant
4-byte frame *trailer* — an easy misreading. Frames to your **own** enrolled
address have no such "trailer" in your receive stream, because the ack there
is your own transmission.

Exception: the `0x50` ident request gets the ident reply instead (section 7).
A frame that fails its checksum should get **no** reply; the controller does
not resend — it link-resets ~2 s later, and the recovery walk plus session
re-init is the (heavier) self-heal.

### Frame types

| TYPE | Meaning | Payload | Check |
|------|---------|---------|-------|
| `0x0B` | text row | `ROW` + character bytes; repaints the whole row | sum |
| `0x0C` | single-cell update | `ROW COL CHAR` (LEN = 8) | sum |
| `0x0D` | cursor | `ROW COL FLAG` — edit focus position | sum |
| `0x0E` | session setup (attach sequence; semantics unknown) | 3 bytes | sum |
| `0x0F` | session setup (attach sequence; semantics unknown) | 1 byte | sum |
| `0x0A` | session setup (attach sequence; semantics unknown) | 1 byte | sum |
| `0x50` | ident request (section 7) | none (LEN = 5) | sum |
| `0x64` | graphic bitmap | see below | CRC-16 |
| `0x65` | page sync / session init | opaque | CRC-16 |
| `0x66` | session control | opaque | CRC-16 |

### Text model

The pGD is a 132×64-pixel display, but the controller does not stream a
framebuffer — in text mode it drives the screen as **8 rows × 22 columns**
of characters:

- A `0x0B` frame replaces one whole row: payload byte 0 is the row index
  (0–7), the rest are the characters. The charset is printable ASCII plus
  CAREL glyphs; the one confirmed non-ASCII glyph is `0xDF` = the degree
  sign.
- A `0x0C` frame updates a single cell: `ROW COL CHAR`. This is how the
  controller repaints one drifting digit on a live value, and the **only**
  repaint an edit-mode value change or a PIN-entry digit gets. Live pages
  emit these at ~1 Hz indefinitely, so screen-settle detection must not
  count them as "the page is still changing".
  *(Historical note: early analysis of this project misread the `0x0C`
  frame as a "selector + big-endian 16-bit value" internal-variable record.
  The frame bytes are the same; the `ROW COL CHAR` cell-update reading is
  the one that matches the visible screen and survived live verification.)*
- The controller repaints **only rows that changed** (delta repaints); a
  full redraw happens on session init and page transitions.

### Graphics and inverse-video

`0x64` frames carry bitmap segments (icons, large-font digits, and the
inverse-video bands the pGD uses for title bars and the menu selection
cursor). Observed payload layout: 7 fragmentation/unknown bytes, then
`x, y, w, h` as 16-bit big-endian, then vertical-byte pixel data. A band
painted mostly-lit is inverse video; bands at pixel `y < 8` are the title
bar, `y ≥ 8` the body. Fragments of a split band repeat the same
`x/y/w/h`. For menu-following purposes the useful invariant is: below the
title bar at most one band is lit — the selection — and the controller
moves it by painting the old band dark and the new one lit.

`0x65` marks a **page sync**: the burst after it is a *delta* against the
current screen (rows survive; verified live — a page turn repainted only the
changed rows), so treating it as a blank-slate init is wrong. `0x66` appears
once at session open.

## 9. The attach sequence — ground truth

Captured by unplugging and replugging the real pGD while a fully passive tap
listened. While the terminal is absent, the controller polls nothing and
loops the FF-walk every ~2.3 s. After replug the terminal stays silent until
the walk reaches its address; adoption then proceeds:

```
20' 02 01 80 00 00 01 00 00 00 00 5B    roll-call probe for 0x20 (claims empty)
01' 02 20 80 00 00 01 80 00 00 00 DB    pGD echoes, claims its bit
20' 02 01 80 00 00 01 80 00 00 00 DB    controller rebroadcasts updated map
01' 02 20 80 00 00 01 80 00 00 00 DB    pGD echoes again
20' 02 01 80 00 00 01 80 00 00 00 DB    controller confirms once more
01' 01 20 DD  01'                       plain link reply; ack
```

~86 ms later the controller pushes the whole session unilaterally (fully
redrawn ~0.9 s after adoption), each frame acked by the terminal with
`01' 03 20 DB`:

```
20' 0A 06 01 01 CD                      type 0A, value 01
20' 50 05 01 89                         ident request
01' 51 07 20 0A 17 65                   ident reply (0A 17)
20' 66 08 01 00 01 9D 13                session control     (CRC-16 LE)
20' 65 0F 01 01 00 00 00 00 00 00 00 00 9C 46               (CRC-16 LE)
20' 0D 08 01 00 00 00 C9                cursor
20' 0E 08 01 80 00 00 48
20' 0F 06 01 00 C9
20' 0B ...  × 8                         all eight text rows (full redraw)
20' 64 ...  × 6                         graphic/icon segments
20' 0C ...                              steady-state cell updates resume
```

The same `66/65/0D/0E/0F` + full-redraw sequence is replayed to the
surviving terminal(s) at **every** enroll and disenroll of any terminal —
one visible repaint per membership change.

## 10. Multi-terminal behavior

CAREL documents up to 3 terminals per controller, served "as if the keypads
and the displays were connected in parallel". Observed reality on the
reference unit, with the physical pGD at 32 and a second terminal enrolled
at 31:

- **The terminal list gates the application session.** The controller's BIOS
  adopts any terminal that answers its roll-call correctly, but the
  *application* only ident-queries and sessions terminals configured in the
  controller's terminal list (edited from any pGD: hold Up+Enter+Down ~5 s →
  address/terminal-config screens). An adopted-but-unlisted terminal gets
  polls forever and nothing else.
- **Each sessioned terminal gets its own private display session** — its own
  text rows, graphics, and cursor frames, addressed to it individually. Key
  presses from either terminal act on the shared application state, and
  resulting updates are pushed to all sessioned terminals.
- **There is only ONE keypad poll slot**, and it follows the most recently
  joined terminal. While the second terminal is enrolled, the controller
  keeps pushing display updates to the pGD but stops polling it — the pGD's
  keypad is dead until the second terminal leaves, at which point normal
  service resumes within one poll interval. This is controller scheduling,
  not a timing artifact (verified at two different reply-turnaround
  settings).
- Enrollment corrupts nothing at the wire level: across >70 k frames of
  enrolled-idle soak the checksum-failure count was zero and nobody ever
  transmitted over anybody.

## 11. Timing reference

All values measured on the reference unit; treat as typical, not spec.

| Quantity | Value |
|----------|-------|
| Byte time (11 bits @ 62500) | ~176 µs |
| Terminal poll rate | ~40 /s |
| pGD poll→reply turnaround | 421 µs mean, σ 18 µs (idle); >2 ms while redrawing |
| Reply timing after poll end | 64 µs too early; ~100 µs lower bound; 250–420 µs proven |
| Missed reply → link reset | ~2 s |
| Idle roll-call cadence | one full walk per ~12 s |
| FF-walk loop, empty bus | every ~2.3 s |
| Adoption → session push | ~86 ms |
| Adoption → full redraw done | ~0.9 s |
| Membership change → net-rebuild walk | ~4.5–6.1 s |
| Key press → repaint | ~250 ms median |
| Idle bus load | ~124 frames/s (~93 % poll cycle) |

## 12. Cautions

- **Do not leave a second terminal enrolled across a controller power
  cycle.** On the reference unit, one cold boot with terminal 31 enrolled
  coincided with an abnormal machine start (a skipped startup interlock).
  Capture evidence rules out live bus traffic from the second terminal as
  the trigger; a controller-side state mechanism is unproven but cannot be
  excluded. Until understood: disenroll before any power cycle.
- Passive capture is risk-free (the reference captures show zero corruption
  from a high-impedance tap), but **transmitting is not**: a malformed or
  half-answered exchange resets the link and repaints every terminal, and
  key presses act on real machinery. Gate transmit paths off by default.
- The single keypad poll slot (section 10) means enrolling a second terminal
  silently disables the physical terminal's keypad for the duration. Prefer
  transactional enroll → act → disenroll if humans share the unit.

## 13. Known unknowns

- The semantics of attach-sequence types `0x0A`, `0x0E`, `0x0F` and the
  payloads of `0x65`/`0x66` beyond "session sync/control".
- The exact meaning of the ident payload bytes (`0A 17` here) — presumed
  type + firmware version.
- The `0x64` graphic payload's leading 7 bytes (fragmentation counters?).
- Whether any of the above varies across CAREL BIOS versions or terminal
  models — everything here comes from one µPC + one pGD.
