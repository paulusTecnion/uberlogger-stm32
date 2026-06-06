# Uberlogger Phase 2 — Sub-project A: Protocol v2 + Per-Transaction Timestamp — Design Spec

- **Date:** 2026-06-06
- **Status:** Draft for review
- **Target chip:** STM32G030C6 — 64 MHz Cortex-M0+ (no hardware divide), **8 KB SRAM**, 32 KB Flash
- **Companion:** `uberlogger-esp32` (ESP32-S2), the SPI **master** (this round changes both chips)
- **Builds on:** Phase 1 (structural refactor, complete + hardware-verified) and its design spec
  `2026-06-05-uberlogger-stm32-refactor-design.md` (esp. §11 Phase-2 sketch, §5.1 shared `ul_protocol.h`).

---

## 1. Where this fits — Phase 2 is decomposed

The north-star goal is sustained logging beyond 250 Hz toward **1 kHz** (8 channels). Phase 2 is built
as **incremental sub-projects**, each its own spec → plan → bench-verified increment (the same discipline
as Phase 1's 10 tasks), each shipping working firmware:

- **Sub-project A (THIS spec):** redesign the SPI wire protocol (`UL_PROTOCOL_VERSION` 1→2) around a
  **per-transaction base timestamp**, shrink the line 29→17 B, make buffer size/depth a tunable, and add
  **500 Hz / 1 kHz RAW logging** on the *existing* free-run+decimate acquisition.
- **Sub-project B (later):** dumb-pipe — STM ships raw ADC, ESP32 owns all LUT + IIR; resolves the
  calibration-LUT-location question.
- **Sub-project C (later):** triggered acquisition — TIM3-`T3_TRGO`-coherent conversions + DMA-direct
  framing for true high-rate; and the 16-bit-high-rate IIR retuning.

A's wire format is deliberately designed to **anticipate B and C** (a `flags` byte with reserved bits) so
those stages are a flag, not another wire relayout.

---

## 2. Goal & supported matrix (sub-project A)

Deliver protocol v2 + per-transaction timestamps + the buffer tunable, and expose new high rates:

| Rate | 12-bit | 16-bit | CSV | RAW |
|---|---|---|---|---|
| 1–250 Hz | ✅ | ✅ | ✅ | ✅ |
| **500, 1000 Hz** | ✅ | ❌ (IIR retune deferred → C) | ❌ (ESP32 formatting cost) | ✅ |

**Above 250 Hz = 12-bit RAW only.** Rationale: the 16-bit path runs the rate-tuned IIR filter
(retuning is real DSP work, deferred); CSV at 1 kHz means ~1000 float→ASCII rows/sec on the single-core
ESP32-S2 while it also services SPI/SD — RAW is a cheap `memcpy`. The 12-bit high-rate path is LUT-only
(no IIR), which also aligns with the eventual dumb-pipe.

**500/1000 Hz are RELEASE-GATED on zero buffer overrun (§10).** A sample rate is shippable **only if it
sustains a long soak with `overrun == 0`** on the bench reference setup. If a rate overruns even with the
largest practical ring depth, **it is unacceptable and is NOT released** in A — it is dropped from the
supported set (and becomes a sub-project-C/triggered target). So 500 Hz and 1 kHz here are *candidate*
rates, confirmed for release individually by the gate; A is still a success delivering 500 Hz even if
1 kHz must wait for C.

---

## 3. Architecture & module touch points (builds on Phase-1 modules)

- **`framing` (STM, keystone)** — owns the v2 frame: captures the base timestamp once per frame, lays out
  17 B lines, manages the buffer ring. Nearly all STM change concentrates here (Phase 1 quarantined wire
  layout into this module exactly so this would be local).
- **`acquisition` (STM)** — minor: extend the TIM3 reload table with 500 Hz / 1 kHz (starting from the
  §10 Phase-2 prescaler hints preserved in `config.c`). **No acquisition-model change** — still
  free-run + decimate. The TIM3 decimation ISR gains the "first-line-of-frame ⇒ read RTC base" branch.
- **`config` (STM)** — accept the new rates; enforce the >250 Hz ⇒ 12-bit/RAW rule on the STM side.
- **`ul_protocol.h` (both repos)** — v2 frame structs, `UL_PROTOCOL_VERSION` = 2, format marker, the
  `flags` definitions, runtime-version command. Vendored byte-identically (Phase-1 mechanism).
- **ESP32 `logger.c` / parser** — parse v2 frames; reconstruct `t_i = base + i·period`; write CSV
  (≤250 Hz) or RAW v2; the runtime version handshake; the >250 Hz reject.
- **ESP32 `rest_server.c` + frontend** — the >250 Hz guard (backend NACK + frontend disable of 16-bit/CSV).
- **`tools/bench/ul_verify.py` / `ul_matrix.py`** — v2 RAW structural checks + v1/v2 auto-detect +
  the new semantic-equivalence and new-rate validation modes.

---

## 4. The v2 wire frame

A transaction is one **frame** = a fixed header + N fixed 17 B lines (+ resync markers). The two
mirror-image structs (`spi_msg_1_t`/`spi_msg_2_t`) collapse into **one parametric frame struct**; the
ping-pong is two instances (extendable to an N-deep ring — §6).

**Frame header** (indicative; exact offsets pinned by `_Static_assert` at implementation):

| field | bytes | notes |
|---|---|---|
| start marker | 2 | e.g. `FA FB`, kept for stream resync |
| `protocol_version` | 1 | = 2; per-frame detection / robustness guard |
| `flags` | 1 | bit0 = resolution (0=12b,1=16b); **reserved bits for B/C** (raw-ADC/DMA-direct, triggered-coherent) |
| `base_epoch` | 4 | Unix seconds, RTC read once at frame start |
| `base_subsec` | 2 | **Q16 fractional second** (units of 1/65536 s, ~15 µs resolution — finer than the RTC's 0.49 ms source); computed correctly at capture (fixes the §7 subsecond smell) |
| `fs_code` | 1 | the `LOG_SAMPLE_RATE` enum index; maps to the exact per-line `period` via a **shared table in `ul_protocol.h`** (avoids putting a wide period value on the wire — a u16 µs can't span 1 Hz…1 kHz). ESP32 reconstructs `t_i = base + i·period(fs_code)`. |
| `line_count` | 1 | lines actually present (handles partial final frame) |
| `pad` | 2 | alignment / reserved for B/C |
| **header total** | **14** | |

**Per line:**

| field | bytes |
|---|---|
| GPIO byte | 1 |
| 8 × ADC `uint16_t` (12-bit values right-aligned, as today) | 16 |
| **line total** | **17** |

(vs today's 29 B line — a ~40 % shrink, the SRAM enabler.)

---

## 5. Timestamp model — base + reconstruct

**Capture:** at each frame's first line the STM reads the RTC **once** into `base_epoch`/`base_subsec`
and stamps `fs_code` (the active rate). Subsequent lines carry no time.

**Reconstruct (ESP32):** `t_i = base_epoch + base_subsec + i·period(fs_code)`, where `period(fs_code)` comes
from the shared rate table in `ul_protocol.h`.

**Why this is more accurate, not less** (verified on this board):
- The RTC is the **LSE 32.768 kHz crystal** with `SynchPrediv = 2047` ⇒ subsecond resolution **≈ 0.49 ms**.
- TIM3 runs off the **64 MHz HSE-PLL clock** and *is* what triggers each sample ⇒ inter-sample spacing is
  crystal-accurate to ~ppm with microsecond resolution.
- **Absolute** time comes from the RTC in both old and new schemes (new re-anchors every frame ⇒ identical
  absolute accuracy). **Inter-sample spacing** is *better* under reconstruction: the old per-line RTC read
  quantizes to 0.49 ms (± half a sample at 1 kHz) plus ISR jitter; at 1 kHz the RTC physically cannot
  resolve 1 ms samples. TIM3 reconstruction has neither problem.
- Re-anchoring every frame bounds any HSE/LSE drift to one frame (~2 µs over a 119 ms frame) and prevents
  long-term accumulation.

**Caveat & mitigation:** reconstruction assumes the frame's lines are **contiguous** (no internal gaps).
If samples are dropped, post-gap times would shift. Mitigation: the buffer ring + overrun handling (§6)
keep frames gap-free; on overrun the base is re-anchored on the next frame and the discontinuity is flagged
so tooling shows a gap rather than mis-timed samples.

---

## 6. Buffer model & overrun (the tunable)

- The single parametric frame struct replaces the twin structs. Two named knobs:
  - **`LINES_PER_FRAME`** (replaces the hard-coded 70).
  - **`FRAME_DEPTH`** (2 = classic ping-pong; **N = ring** for stall slack).
- With 17 B lines the same ~4 KB that held 2×70 lines holds ~2×119 — so either more buffering in the same
  RAM or less RAM. Final sizes are chosen against the **measured** live SRAM budget (stack + working
  buffers) once the layout compiles, staying within the 8 KB ceiling.
- **Why depth matters:** the STM fills at the sample rate; the ESP32 must read a full buffer over SPI *and*
  write it to SD before the STM reuses it. SD cards stall tens of ms for housekeeping. At 250 Hz a 70-line
  buffer ≈ 280 ms of slack (invisible); at 1 kHz a ~119-line buffer ≈ 119 ms — an SD stall can exceed it. A
  ring of several buffers lets the STM coast through a stall without dropping data.
- **Revive `overrun` — and treat it as a release gate, not graceful degradation.** The dead `overrun`
  flag (read but writes commented out — found in Phase-1 cleanup) is implemented: set when no free buffer
  exists, reported to the ESP32. Its primary purpose is the **acceptance gate** (§10): a rate that overruns
  on the bench reference setup is **not acceptable for release** at all — overrun is a disqualifier, not a
  condition we ship and tolerate. The ring depth is sized so that *released* rates never overrun. (Runtime
  reporting remains as a field safety net for marginal/foreign SD cards, but the design target is zero.)

---

## 7. Two-layer data flow (and where the RTC goes)

There are **two buffer layers**; in sub-project A the ADC DMA never touches the frame buffers.

```
Layer 1 (ADC, DMA-written): ADC free-runs → DMA-circular into adc12Buffer[64]/adc16bBuffer[16];
        DMA half/full ISR runs LUT (adc_comp) + (16b) IIR → latest per-channel in iirFilter[8].   [unchanged in A]

Layer 2 (frame ring, CPU-written): the TIM3 decimation ISR bridges Layer 1 → Layer 2:

  on TIM3 tick:
    if (line_idx == 0):                       # first line of a fresh frame
        read RTC once → header.base_epoch/base_subsec
        stamp header.fs_code/version/flags/...
    copy GPIO + iirFilter[8] snapshot → frame.lines[line_idx]    # from Layer 1
    line_idx++
    if (line_idx == LINES_PER_FRAME):
        mark frame ready (DATA_RDY path); advance ring to next free buffer; line_idx = 0
        if no free buffer → set overrun (drop + flag)
```

So the RTC base is a plain CPU write into the frame **header** (Layer 2), once per `LINES_PER_FRAME` ticks —
no DMA/RTC contention because DMA (Layer 1) and the header/lines (Layer 2) are different memory & writers.

**Forward-compat for C (recorded, not built in A):** the dumb-pipe/triggered model writes ADC line data via
DMA *directly* into the frame's line region. The header is at the **front** precisely so the layout is
`[ CPU-written header | DMA-target line region ]`; DMA targets `frame + sizeof(header)`, the CPU fills the
header, and the RTC base is read at the buffer's fill-start (in the DMA block-complete ISR of the previous
buffer). The reserved `flags` bit marks "raw-ADC / DMA-direct", so C needs no new wire relayout.

---

## 8. Output formats & tooling

- **CSV (≤250 Hz only):** ESP32 reconstructs times and writes the **same columns as today** (timestamp,
  DIO, 8× AIN); users see only more-uniform timestamps. STM-side filtering placement unchanged in A.
- **RAW (all rates; only option >250 Hz):** the EXISTING `.dat` **container is kept** —
  `[uint32 header_length][9 settings bytes][8×int32 adc_offsets][channel labels…]`, frames, then a
  trailing `uint64` total-row count (this is the format `uberlogger-esp32/front/www/convert_raw.py`
  already reads; its `adc_offsets`/labels/ranges are required by the offline count→voltage conversion).
  v2 changes only: the container's **file-format-version field `settings[0]` is bumped 2→3** to signal the
  new frame body, and the frame body becomes concatenated v2 frames (header + adc block + gpio block)
  instead of the alternating `spi_msg_1/2`. Old-vs-new detection is `settings[0]` (2 vs 3); the per-frame
  `protocol_version` byte is a second SPI-stream guard. **`convert_raw.py` (the production offline
  converter) and `ul_verify.py` branch on `settings[0]`** — old captures still parse. NOTE: because CSV is
  disabled >250 Hz, `convert_raw.py` is the **only** way to get CSV for high-rate logs, so it is a
  first-class deliverable of A (reconstructs per-line time `base + i·period`; reuses the unchanged
  `convert_adc`).
- **>250 Hz guard:** frontend greys out 16-bit & CSV when rate >250 Hz; backend NACKs with a specific
  message if requested anyway (intent never silently changed).

---

## 9. Cross-repo protocol & versioning

- `UL_PROTOCOL_VERSION` 1 → 2 in the shared, vendored `ul_protocol.h`; `_Static_assert(==2)` in a
  build-visible `.c` of *each* repo catches a stale copy at build (Phase-1 mechanism).
- **Runtime version handshake:** a small command (new `STM32_CMD_GET_PROTOCOL_VERSION` or piggy-backed on
  status) lets the ESP32 verify at boot that the STM speaks v2; on mismatch it warns/refuses rather than
  producing garbage frames — cheap insurance against a half-updated (partial-OTA) device.
- **Lockstep:** both chips flashed together. **This round reflashes the ESP32** (its parser genuinely
  changes) — a new bench prerequisite: ESP32 flash over USB-serial (`build.sh` → `idf.py ... -p /dev/ttyACM0`).

---

## 10. Verification (philosophy changes from Phase 1)

Phase 1's oracle was "diff against a byte-identical baseline." A **intentionally changes the wire**, so the
oracle becomes **semantic equivalence + intrinsic validation**:

1. **Equivalence vs the v1 baseline** (overlapping rates 1/25/100/250 Hz × 12/16-bit): same physical signal
   on old vs new firmware; compare *decoded* output — ADC values within the existing tolerance, reconstructed
   timestamps vs old per-line timestamps within the RTC's ~0.49 ms resolution.
2. **New-rate validation + the overrun release gate** (500/1000 Hz, no baseline): intrinsic checks —
   row count ≈ rate×duration, reconstructed timestamps uniform at the expected period, ADC in-range/not-stuck.
   **The hard gate: a long sustained soak (e.g. ≥10 min continuous logging on the bench reference SD card,
   long enough to hit the card's housekeeping stalls) must finish with `overrun == 0`.** This is **pass/fail
   for releasing that rate** — any overrun, even one, disqualifies the rate (it is dropped from the supported
   set / deferred to sub-project C). This is also how we discover whether free-run+decimate sustains 1 kHz:
   a quantified release decision, not a surprise. Where practical, run the soak on the slowest SD card we
   intend to support (worst case), not just a fast one.

   **The soak MUST run under realistic concurrent API load**, not in an idle state — that load is what
   provokes the ESP32-side stalls that cause overrun. While logging at the rate under test, the bench
   harness concurrently exercises the HTTP API the way real use does: repeated `getStatus` polling, the
   **live-view `getValues`/`getRawAdc` single-shot poll** (a known logger-task contention source from the
   Wi-Fi/NTP work), `getConfig`, `getFileList`, and at least one heavier action (e.g. a settings read or a
   file-list during active logging). A rate passes the gate only with `overrun == 0` *under* this load.
   This makes the gate represent a user driving the web UI mid-log, not a quiet bench.
3. **v2 structural checks** in `ul_verify.py` (header sanity, 17 B stride, monotonic reconstructed time,
   inferred rate = setting).
4. **Guard + stress:** confirm the >250 Hz CSV/16-bit rejection; long 1 kHz soak and/or induced SD stalls to
   confirm the ring absorbs them and any genuine drop sets+reports `overrun` (never silent).
5. **Per-step gates:** clean build of *both* repos; v2 layout `_Static_assert`s hold; SRAM ≤ 8 KB; lockstep
   flash of both chips on the bench.

---

## 11. Error handling / edge cases

- **Partial final frame** at logging-stop: flushed with its real `line_count` (in the header) — no padding garbage.
- **Rate change:** routed through CONFIG state (logging stops first), so a frame never mixes two periods; the
  new period is stamped on the next frame.
- **Overrun mid-frame:** set `overrun`, re-anchor `base` next frame, flag the discontinuity for tooling.
- **RTC not NTP-synced:** `base_epoch` only as good as the sync — same as today, not worse; optionally surface
  a "time not synced" note (existing behavior).
- **iirFilter[8] snapshot race:** the TIM3 ISR reads `iirFilter[8]` while the ADC DMA callback may write it —
  today's `busy`/`Error_Handler` guard. A preserves it and we **measure** whether it holds at 1 kHz; B removes
  the intermediate entirely.

---

## 12. Constraints (carried from Phase 1, adjusted)

- **C1 — wire format is now CHANGED (no longer frozen).** Phase 1's freeze was a verification anchor; A is
  the planned redesign. New anchor = semantic equivalence (§10). The change is centralized in `ul_protocol.h`.
- **C2 — timing must hold or improve, measured.** A *reduces* ISR cost (RTC once/frame, no per-line timestamp
  copy). The free-run/decimate path and the snapshot race are measured at the new rates.
- **C3 — CubeMX-friendly.** Application code stays in `USER CODE` / module `.c/.h`; `.ioc`/HAL preserved.
- **C4 — fits 8 KB SRAM / 32 KB Flash** at all times (buffer sizing is explicitly budget-driven).
- **C5 — verified on hardware** at every increment; this round flashes **both** chips.

---

## 13. Known smells — A fixes vs leaves

- **Fixed by A** (framing is rewritten, not preserved): the `subseconds` "not 100 % correct" computation
  (done correctly at base capture); the twin-struct mirror layout + manual padding (collapsed to one struct);
  the dead `overrun` flag (implemented).
- **Left for later:** the `busy`/`Error_Handler` ADC-reentrancy guard (stays in `acquisition`, untouched);
  `adc_resolution`/`tim3_counter` extern-shared hot-ISR globals (Phase-2 ownership, candidate for B/C);
  16-bit > 250 Hz IIR retuning (C); calibration-LUT relocation (B).

---

## 14. Out of scope (sub-project A)

- Dumb-pipe / raw-ADC-on-wire and moving LUT/IIR to the ESP32 (sub-project B).
- Triggered (`T3_TRGO`-coherent) acquisition + DMA-direct framing (sub-project C).
- 16-bit logging above 250 Hz / IIR coefficient retuning (C).
- Channel-mask on the wire (dropped entirely, per Phase-1 §11).
- Host-side unit tests (revisit if C needs them).

---

## 15. Open questions / risks

- **1 kHz free-run+decimate sustainability** is the headline unknown, decided by the §10.2 **overrun release
  gate** (zero overrun over a long soak on the reference, ideally worst-case, SD card). A released rate must
  pass it; a rate that overruns is unacceptable and is **not shipped** in A (deferred to C). A still succeeds
  delivering the protocol/timestamp/buffer foundation + 500 Hz even if 1 kHz fails the gate.
- Final `LINES_PER_FRAME` / `FRAME_DEPTH` values — chosen against measured SRAM, not fixed here.
- SD sustained-write headroom at 1 kHz RAW (~17 KB/s payload) — expected fine; confirmed by the soak test.

---

## 16. Implementation sequencing (each = one buildable, bench-verified increment → feeds the plan)

1. **`ul_protocol.h` v2** — frame structs, version 2, flags, fs_code rate table, RAW container
   file-format-version constant (3), runtime-version command; bump + build guard in both repos.
   (No behavior yet; both repos compile.)
2. **STM `framing` v2** — single parametric frame struct, base-timestamp capture, 17 B lines,
   `LINES_PER_FRAME`/`FRAME_DEPTH` ring + `overrun`. Layout `_Static_assert`s.
3. **STM `acquisition`/`config`** — TIM3 base hook; 500/1000 Hz rates; >250 Hz ⇒ 12-bit/RAW enforcement.
4. **ESP32 parser + reconstruction** — v2 frame decode, `t_i = base + i·period`, runtime-version handshake.
5. **ESP32 output** — CSV (≤250 Hz, reconstructed) + RAW v2 (container kept, `settings[0]` 2→3, v2 frame body); >250 Hz reject; frontend disable.
6. **Tooling** — `ul_verify.py`/`ul_matrix.py` v2 structural checks + v1/v2 auto-detect + equivalence/new-rate modes; `ul_soak.py` overrun-under-load; **`convert_raw.py` v3 (the production offline RAW→CSV, only CSV path >250 Hz)**.
7. **Bench** — equivalence vs v1 baseline; 500/1000 Hz intrinsic validation + overrun soak; both-chip flash.
8. **Docs** — protocol doc bumped to v2; README rate table; record sizes + 1 kHz sustainability result.
