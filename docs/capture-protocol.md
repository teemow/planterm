# PLANCAP — the bridge capture-and-control protocol

PLANCAP is the wire protocol between a **pLAN bridge device** (a
microcontroller on the RS-485 bus, typically running a planterm-based
firmware) and a **host tool** (e.g.
[planscope](https://github.com/teemow/planscope)). One TCP connection
carries everything the tool needs:

- **capture** — every byte on the pLAN, with its 9th bit, lossless;
- **device events** — the bridge's state and transmit verdicts, typed;
- **diagnostics** — the bridge's plan-related prose, typed and ordered
  with the very bus bytes it refers to;
- **commands** — arm/disarm, enroll/disenroll, key injection, with acks.

The channel is authenticated and encrypted with a pre-shared key (Noise,
same suite as the ESPHome native API — details below); a plaintext mode
exists only for devices configured without a key.

This document is normative for both ends. It describes the protocol
as-is; there is no version negotiation and no compatibility machinery —
the bridge firmware and the host tools change in lockstep. For the pLAN
bus protocol itself (what the captured bytes *mean*) see
[protocol.md](protocol.md).

Notation: `u8`/`u16`/`u32` are unsigned integers. **Frame lengths are
big-endian; every integer inside a record is little-endian.** (The frame
layer mirrors the ESPHome native API's framing so implementations reuse
their existing Noise plumbing; the record layer keeps the little-endian
layout of the capture records.) The device is the **server**, the tool
is the **client**.

## 1. Transport

TCP. The server never initiates a connection.

- On an ESPHome device the port is **the native-API port + 1** (default
  API 6053 → PLANCAP 6054), so one configured address serves both
  channels.
- A standalone (non-ESPHome) bridge may serve any port; 6054 is the
  convention.

**Concurrent clients, bounded slots.** The server serves a small fixed
number of clients at once (implementation-defined; the reference server
has 3 slots — an interactive TUI plus a one-shot command, with one to
spare). Every server→client record (§5) fans out to all established
clients; each connection runs its own handshake (§3–4), gets its own
snapshot replay (§6.1) and has its own backlog (§6.3), so one slow
client never stalls the others. When all slots are taken, a new
connection evicts the **oldest** client: that socket is closed without
notice, so a wedged client can never lock up the port. A client whose
*established* stream drops should assume it was evicted (or the device
rebooted) and must not silently retry-loop into an eviction fight —
surface it.

The server must never block on the network: a slow client gets a bounded
per-client backlog (see §6), never a stalled bus task.

## 2. Banner

Immediately after accepting a connection the server sends the 7 ASCII
bytes

```
50 4C 41 4E 43 41 50            "PLANCAP"
```

unframed and unencrypted. The banner identifies the protocol; anything
else means the client dialed the wrong port or firmware. The client
verifies it byte-exact and disconnects on mismatch. Nothing else is sent
until the client's first frame arrives.

## 3. Framing

After the banner, everything in both directions travels in frames:

```
u8  marker      0x01 = Noise (handshake message or transport ciphertext)
                0x00 = plaintext
u16 len         big-endian body length
len bytes       body
```

Any other marker is a protocol error (§7). A frame body carries exactly
**one** unit: one Noise handshake message, or one record (§5) —
optionally Noise-encrypted. Records are never split across frames and
never share a frame.

The client's **first frame selects the mode** for the whole connection:

- marker `0x01` with a Noise handshake message → encrypted session (§4);
- marker `0x00` with an **empty body** (`00 00 00` on the wire, the
  *plaintext hello*) → plaintext session, only on a keyless server.

A server configured with a key **must** reject a plaintext hello, and a
keyless server **must** reject a Noise handshake, in both cases by
closing the connection. A client that sent a Noise handshake and got the
connection closed without any response frame should suspect a key
configuration mismatch.

## 4. Encryption

### The key

The PSK is a **32-byte key, configured as base64** — on an ESPHome
device it is the same value as `api.encryption.key`, so one key secures
both the API and PLANCAP; a standalone bridge just has "the capture
key". The key never travels on the wire. Both ends decode the base64
and use the raw 32 bytes.

### Handshake

Noise protocol name:

```
Noise_NNpsk0_25519_ChaChaPoly_SHA256
```

with **prologue = the 7 ASCII bytes `PLANCAP`**. This is the same
pattern, DH, cipher and hash as the ESPHome native API, so the firmware
reuses its bundled noise-c and Go clients reuse flynn/noise; the
*different* prologue (the API uses `NoiseAPIInit\x00\x00`) domain-
separates the two protocols, which share one PSK — a handshake message
built for one channel can never complete on the other.

The client is the Noise initiator. Handshake messages ride in
`0x01`-marked frames whose body is prefixed with one status byte:

```
client → server:   0x01 <len> 0x00 <noise handshake message 1>
server → client:   0x01 <len> 0x00 <noise handshake message 2>     success
                   0x01 <len> 0x01 <UTF-8 error text>              failure, then close
```

On a PSK mismatch the error text is exactly `Handshake MAC failure`
(clients special-case it into "wrong key"). Any other failure text is
free-form.

`NNpsk0` completes in these two messages; both ends then hold a pair of
transport cipher states.

### Transport

Every subsequent frame body is one Noise transport ciphertext
(ChaCha20-Poly1305, implicit nonces per the Noise spec, no rekey); the
decrypted plaintext is exactly one record. Client→server and
server→client use their own cipher states, each with its own nonce
sequence. A failed decryption is unrecoverable: close the connection.

The `u16` frame length caps a ciphertext at 65535 bytes, i.e. a record
at 65519 bytes. Implementations keep records far smaller (the reference
server drains at most 64 byte-pairs per bus-bytes record).

### Plaintext mode

On a keyless server, after the plaintext hello every frame in both
directions is `0x00 <len> <record>` — same records, no protection.
This mirrors the ESPHome API's plaintext fallback and exists for the
same reason: a device that has no key configured. Everything in §5–§7
applies unchanged.

## 5. Records

Every record starts with a `u8 type`; all integers little-endian.
Types `0x00`–`0x02` flow server→client only, `0x03` client→server only,
`0x04` server→client only. Receiving a record from the wrong direction
or with an unknown type is a protocol error (§7).

`ts_ms` fields carry the device's millisecond uptime clock; it wraps at
2³² (~49.7 days) and is a device-relative timeline, not wall time.

### 5.1 Type 0x00 — raw bus bytes

```
u8  type = 0x00
u32 seq        monotonic record counter; 0 = snapshot replay (§6.1)
u32 ts_ms      device clock at drain time
u32 drops      cumulative count of bus bytes lost device-side
               (receive ISR → stream buffer overflow), declared in-band
u16 n          pair count
n × (u8 byte, u8 bit9)
```

Each pair is one byte as seen on the pLAN plus its 9th bit (`bit9`:
`0x00` clear, `0x01` set; other values are invalid). The pairs are the
raw bus stream in arrival order — record boundaries are **drain ticks,
not frame boundaries**. Clients reassemble pLAN frames from the bit9
address marks (see protocol.md §3) across record boundaries.

`seq` starts at 1 when the server boots and increments by one for every
live record — **including records skipped because a client stalled**
(§6.3), so a gap in `seq` is an exact count of lost records, never
silently thinned data. `seq` is shared by all clients: the same record
carries the same `seq` to everyone, and a stalled client's skipped
records are gaps in its own stream only. `seq` is not reset per
connection; a `seq` at or below the last one seen (across a reconnect)
means the device rebooted.

`drops` is monotonic over the device's uptime. A change between
consecutive records means bus bytes vanished device-side at that point
in the stream; the delta is the byte count.

### 5.2 Type 0x01 — device event

```
u8  type = 0x01
u32 ts_ms
u8  kind
u8  a
u8  b          kind-specific, unused fields are 0
```

| kind | meaning | `a` | `b` |
|------|---------|-----|-----|
| 1 | **state** — the device's current truth | bit 0 = armed; bits 1+ = enroll: 0 no, 1 yes, 2 drain (graceful leave in progress) | transmit mode |
| 2 | **join** — first poll to the bridge's enrolled address answered since enrollment was requested; the poll slot is live | 0 | 0 |
| 3 | **TX fired** — a keypad report went out | keycode | attempt number |
| 4 | **key accepted** — no rejection signature followed the injection | keycode | attempt number |
| 5 | **hold** — a background menu walker on the device yielded the terminal session | 1 = explicitly paused, 0 = yielded to the armed write path | 0 |

The state event is the device's authoritative state: clients trust it
over any assumption about their own commands' effects. The server emits
one state event at least every 10 seconds even when nothing changes —
it doubles as the in-band keepalive (§6.4) — and immediately after
every state change.

Kinds are a closed set: an unknown kind is not an error (log and skip
the event), because events are advisory, not framing-relevant.

### 5.3 Type 0x02 — diagnostic

```
u8  type = 0x02
u32 ts_ms
u8  severity   1 error, 2 warning, 3 info, 4 debug
u16 len
len bytes      UTF-8 text, one line, no trailing newline, no NUL
```

The bridge's plan-related diagnostics — what a human used to read off
the device's serial/log console — as typed records, lossless and
ordered with the bus bytes they refer to. Host tools render them in
logs, tee files and status lines; **no tool ever consumes a device's
log console programmatically** — a logger may drop lines under load,
so nothing data-bearing travels there.

The text is free-form prose for humans. Anything a machine must react
to is an event (type `0x01`) or an ack (type `0x04`), never a
diagnostic; clients must not parse diagnostic text.

### 5.4 Type 0x03 — command (client → server)

```
u8  type = 0x03
u8  id         client-chosen, echoed in the ack
u8  op
u8  arg
```

| op | command | `arg` | server internals |
|----|---------|-------|------------------|
| 1 | **arm** | 0 disarm, 1 arm | the transmit gate: while disarmed, injected keys are dropped |
| 2 | **enroll** | 0 leave (graceful drain), 1 join | roll-call enrollment as an additional pLAN terminal |
| 3 | **inject key** | keycode (protocol.md §6) | enqueue one key press for injection into the enrolled poll slot |

Commands dispatch into the same internals as the device's other control
surfaces (on an ESPHome bridge, the `set_armed`/`set_enroll`/
`inject_key` user-defined API services — which remain available for
home-automation use); PLANCAP is the interface host tools use, so a
tool needs **no other channel** for its live features.

`id` is an opaque echo token. Clients that want to correlate acks use
distinct ids for in-flight commands (a wrapping counter is fine);
uniqueness is not the server's concern. Ids are scoped per connection:
the ack for a command goes **only to the client that sent it**.

**Concurrent commanders.** There is one control plane; commands from all
clients dispatch on the device's single control task in socket-read
order, with no locking or per-client ownership. `arm`/`enroll` are
last-writer-wins — the resulting state event broadcasts to every client,
so all of them learn the new truth, whoever set it. Key injections from
all clients funnel into the device's one bounded press queue and execute
serially in arrival order; a full queue is an honest `rejected` ack.
Clients that drive the shared terminal session concurrently will
interleave on the shared screen — coordinating *that* is the operator's
problem, not the protocol's.

### 5.5 Type 0x04 — command ack

```
u8  type = 0x04
u32 ts_ms
u8  id         echoed from the command
u8  status     0 accepted, 1 rejected, 2 unknown op
```

The server acks **every** command record exactly once, in order.
`accepted` means the command was dispatched (state set, or key press
enqueued) — the *outcome* arrives as events: a state event follows every
arm/enroll change, a join event certifies the live poll slot, and TX
fired / key accepted events report each injection attempt's verdict.
`rejected` means the command was valid but refused by a gate — e.g. a
key injection while disarmed, or a full injection queue; the server
should say why in a diagnostic record. `unknown op` means the op byte
is outside the table above (the record was still well-formed, so the
stream stays in sync).

## 6. Session behavior

### 6.1 Connection start: snapshot replay

Immediately after the mode is established (handshake complete, or the
plaintext hello on a keyless server), the server sends, before any live
record:

1. **The screen-snapshot replay** — its cache of the latest
   checksum-valid display frame per (terminal, row), one complete pLAN
   frame per type-`0x00` record, each with **`seq` = 0** and `ts_ms` =
   when that frame was actually captured. Display frames addressed to
   the primary terminal carry that terminal's constant 4-byte session
   ack appended (`01' 03 20 DB` for the pGD at `0x20` — see protocol.md
   §8), because clients anchor their frame parsing on it.
2. **One state event** (§5.2).

So a connecting client starts with a fully painted screen and the
device's current truth instead of a blank screen and a wait. `seq` 0
marks the records as replayed state, not live traffic: clients prime
their screen reconstruction from them but keep them out of traffic and
loss accounting. Live records (`seq` ≥ 1) follow.

### 6.2 Record cadence and idle detection

A record boundary is a server drain tick (the reference server drains
every few milliseconds), so on a live pLAN — idle traffic is ~40
polls/s — records flow essentially continuously. A client that has a
partial pLAN frame pending and sees **no record for ≥ 250 ms** may
treat the bus as idle and the pending tail as a complete frame (the
same boundary reasoning as protocol.md §3). The server performs no
frame-level batching: it forwards what the bus delivers, when it
delivers it.

### 6.3 Backlog and the stalled client

The server buffers each client's unsent output in its own bounded
backlog (implementation-defined; the reference server uses 32 KiB per
client) and writes it out non-blocking. When a client's backlog is full:

- new **bus-bytes records are skipped for that client** but `seq` keeps
  counting, so it later sees an honest gap (§5.1) — the other clients
  keep receiving everything;
- if the backlog is still full when an **event, diagnostic or ack** must
  go out, the server **disconnects that client** instead of dropping
  typed records or growing without bound. A reconnect replays the
  snapshot and state (§6.1), so no state truth is lost.

### 6.4 Liveness

The 10-second periodic state event (§5.2) is the in-band keepalive.
A client seeing no record at all for **90 seconds** should consider the
link dead and reconnect. The client sends nothing unprompted; commands
are its only traffic, and the server does not expect any.

## 7. Errors

The record stream has no resynchronization: framing (and in encrypted
mode, the AEAD) is the only integrity boundary. Fatal — close the
connection:

- an unknown frame marker;
- a failed decryption or handshake;
- an unknown **record type**, a record from the wrong direction, or a
  record whose length contradicts its type's layout (a length-bearing
  record shorter than its `len` claims, trailing bytes after a
  fixed-size record, an odd pair region).

Non-fatal — skip and continue: an unknown *event kind* (§5.2), an
unknown *command op* (§5.4, acked with status 2). These are
value-range, not framing, and the stream stays in sync.

There are no protocol-level retries. TCP delivers or the connection
dies; a client reconnects and starts fresh from the snapshot replay.

## 8. Security properties and limits

- Both directions are encrypted and authenticated with the PSK
  (`NNpsk0`): a passive observer learns traffic timing and sizes, not
  content; an active attacker without the key can complete no handshake
  and inject nothing.
- There is no identity beyond the PSK — anyone holding the key is "the
  client". Key distribution and rotation are the operator's problem
  (on ESPHome devices: rotate `api.encryption.key`, reflash, update the
  tools' config).
- `NN` has no static keys: each connection's ephemeral DH plus the PSK
  give per-session keys, but there is no server authentication beyond
  the PSK either — a MITM *with* the key is out of scope.
- **Plaintext mode has none of these properties.** A keyless bridge
  exposes the full display content, every key pressed (service PINs
  included) and the control surface to whoever reaches the port.
  Run keyless bridges on trusted networks only — or better, configure
  a key.
