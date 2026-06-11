# Uberlogger STM32 Firmware Refactor — Design Spec

- **Date:** 2026-06-05
- **Status:** Draft for review
- **Target chip:** STM32G030C6 — 64 MHz Cortex-M0+ (no hardware divide), **8 KB SRAM**, 32 KB Flash
- **Companion:** `uberlogger-esp32` (ESP32-S2), which is the SPI **master**
- **Scope of this spec:** Phase 1 (structural refactor). Phase 2 (the actual >250 Hz capability) is sketched only.

---

## 1. Background & goal

The STM32 is a pure data-acquisition slave hanging off the ESP32-S2. The north-star
goal is to **raise the sustained output rate beyond the current 250 Hz toward 1 kHz+**.
The README blames "small buffer memory on the STM32 for sending SPI data"; investigation
confirms the ceiling is **how many sample-lines fit in 8 KB SRAM between SPI transactions**,
*not* raw SPI bandwidth (~29 KB/s needed at 1 kHz vs a 10 MHz link) and *not* core MHz
(actually 64 MHz, not the 16 MHz the README states).

This work is split into two sub-projects so that "things are not allowed to break" stays true:

- **Phase 1 (this spec):** Reorganize the firmware into auditable modules with explicit
  timing boundaries. The SPI wire protocol stays byte-identical **as a verification strategy,
  not because it cannot change** — frozen bytes let every step be diffed against a known-good
  baseline, so a behavior change can only be the refactor. Phase 1 also centralizes the
  hand-duplicated protocol definitions into one **canonical, vendored `ul_protocol.h`** shared
  by both repos (byte-identical, so still verifiable). Every step compiles, flashes, and is
  bench-verified.
- **Phase 2 (separate, later spec):** **Opens with the wire-protocol redesign** (per-transaction
  timestamp + a `PROTOCOL_VERSION` bump in the shared header) on the clean refactored base, then
  the higher-rate capability (bigger/N-deep buffers, dumb-pipe, triggered acquisition), updating
  both chips in lockstep. Doing the protocol change first, alone, means never debugging the
  refactor and the protocol at the same time.

---

## 2. Current architecture (as-built)

### 2.1 Acquisition — free-run + decimate

The ADC is **free-running, not TIM3-triggered**. `Config_Set_Resolution` sets the trigger to
`T3_TRGO`, but `Config_Set_Sample_freq` runs *after* it (the ESP32 sequences
resolution → sample-rate → measure-mode, see `logger.c:751-770`) and overwrites the trigger to
`ADC_SOFTWARE_START` + `ContinuousConvMode = ENABLE` for **both** 12- and 16-bit. So:

- The ADC converts continuously, DMA-circular into a small ring:
  `adc12Buffer[64]` (8 ch × 8) or `adc16bBuffer[16]` (8 ch × 2).
- The DMA **half/full callbacks** run `adc_comp()` (LUT non-linearity correction) and,
  for 16-bit, `iir_filter()`, depositing the *latest processed value per channel* into
  `iirFilter[8]`. (In 12-bit, the IIR is **not** applied — only the LUT.)
- **TIM3's period ISR is the decimation + timestamp clock.** At the desired output rate it
  reads the RTC (`HAL_RTC_GetTime` + `HAL_RTC_GetDate`), reads the GPIO byte (`GPIOB->IDR >> 8`),
  and `memcpy`s the current `iirFilter[8]` snapshot into the active frame as one line.

This is an **oversample-and-decimate** design: ADC+IIR smooth underneath, TIM3 samples the top.
RTC cannot be read by DMA, so TIM3's update IRQ is the mechanism that pairs a timestamp with
each ADC sample-set — this is *why* the code is shaped the way it is.

### 2.2 Framing — twin ping-pong structs

Data is assembled into two near-mirror-image structs, `spi_msg_1_t` / `spi_msg_2_t`
(`main.h`), each holding up to `DATA_LINES_PER_SPI_TRANSACTION = 70` lines. A line is:

| field | bytes |
|---|---|
| `s_date_time_t` timestamp | **12** |
| GPIO byte | 1 |
| 8 × ADC `uint16_t` | 16 |
| **total** | **29** |

The 12-byte timestamp is **41 % of every line** — the dominant memory waste. The two structs,
their manual `padding[]` fields, and the hand-computed `memcpy` offsets are fragile by design
(`main.h` carries a "DO NOT CHANGE … BYTES NEED TO BE 4 BYTES ALIGNED" warning). Two buffers
consume ~4 KB of the 8 KB SRAM. `gpio_is_half` / `adc_is_half` / `adc_16b_is_half` toggle which
half is filled vs. sent.

### 2.3 Transport — SPI slave + DATA_RDY handshake

STM32 is **SPI slave, mode 0 (CPOL=0/CPHA=0), MSB-first, 8-bit, 10 MHz**. `STM_DATA_RDY` (PB6)
is the handshake line the ESP32 polls/interrupts on. `spi_ctrl.c` is a small state machine
(`IDLE/SENDING/RECEIVING`) over DMA send/receive; **TIM14/TIM16 are software watchdogs** that
abort a stuck TX/RX DMA after a timeout. Logging-data transfers raise DATA_RDY and the ESP32
reacts via a GPIO ISR; command transfers use a polled handshake.

### 2.4 Control — `main.c` state machine

A `while(1)` switch in `main.c` runs `MAIN_IDLE → CONFIG / LOGGING / SINGLE_SHOT /
WAIT_FOR_TRIGGER`, processing a 13-command protocol (`STM32_CMD_*`, identical 8-byte `spi_cmd_t`
on both chips). IDLE dispatches commands; `config.c` handles the settings sub-protocol and
ADC/timer reconfiguration.

### 2.5 Two facts that specifically constrain high rates

1. **Channel-disable is a no-op on the wire.** `STM32_CMD_SET_ADC_CHANNELS_ENABLED` is
   commented out on *both* sides (`config.c:114`, `logger.c:734`); both the STM framing and the
   ESP32 parser hard-code 8 channels (`dataLen * 2 * 8`, stride `i*8`). 16 ADC bytes/line ship
   regardless of how many channels the user wants. Channel selection is only an ESP32 CSV-column
   choice.
2. **Filtering placement (corrected understanding).** At **≥ 1 Hz** the ESP32 does **not**
   filter — it writes the STM samples straight to SD; only the STM filters (LUT always, IIR for
   16-bit). The ESP32's `applyFilter`/`iir_filter_12b/16b` runs **only at sub-1 Hz rates**, where
   the STM is clamped to a 25 Hz base and the ESP32 IIR-averages 25 Hz → one row per N seconds
   (avg via `y_state` if `averageSample` is set, else last value). The live web view is a
   separate path (`live_data_buffer`). So there is **no double-filtering of normal logged data.**

---

## 3. Constraints

- **C1 — Wire format frozen through Phase 1, as a verification anchor.** The SPI wire format,
  command IDs, DATA_RDY handshake, and 10 MHz timing stay byte-/bit-identical through Phase 1 so
  each step is diffable against a baseline. This is a deliberate strategy, **not** a permanent
  constraint — the protocol is redesigned at the start of Phase 2. Centralizing the definitions
  into the shared `ul_protocol.h` must preserve the exact bytes.
- **C2 — Timing is sacred.** ISR cost and ordering on the acquisition/transport path must not
  regress. Relocations are verbatim, not rewrites.
- **C3 — CubeMX-friendly.** Keep the `.ioc` / STM32CubeIDE project and HAL. Application code
  lives in `USER CODE` sections and new `.c/.h` modules so future CubeMX regeneration is safe.
- **C4 — Fits in 8 KB SRAM / 32 KB Flash** at all times.
- **C5 — Verified on hardware** at every increment (ST-Link + a real ESP32 on the bench).

---

## 4. Approach (chosen: A — incremental in-place)

Carve the application logic out of `main.c` into focused modules, one **flash-and-test
increment** at a time, leaving CubeMX-generated init untouched. The wire format is unchanged in
Phase 1; the protocol/timestamp redesign is Phase 2 and isolated to the `framing` module + the
ESP32 parser. Rejected alternatives: **B** (context-struct/host-tests — more churn than needed
now) and **C** (protocol-first — highest risk, two codebases moving at once; reserved for Phase 2).

---

## 5. Target module structure

| Module | Owns | Public interface (indicative) |
|---|---|---|
| **`framing.c/.h`** *(keystone)* | The SPI message buffers, **all** layout/offset/padding knowledge, the ping-pong/half bookkeeping, **and the timestamp-coupling strategy** | `frame_reset()`, `frame_append_line(ts, gpio, adc16[8])`, `frame_take_ready(&ptr, &len)` |
| **`acquisition.c/.h`** | ADC + TIM3 sample path; the ADC DMA half/full + TIM3 period ISR bodies; `Adc_start`; **the free-run-vs-triggered decision and the LUT/IIR call sites** | `acq_init()`, `acq_start()`, `acq_stop()` |
| **`app.c/.h`** | The `while(1)` state machine, trigger debounce, IDLE command dispatch | `app_init()`, `app_run_once()` |
| `spi_ctrl.c` *(kept)* | DMA transport + TIM14/16 watchdogs | unchanged API; dead-code trim only |
| `config.c` *(kept)* | Settings sub-protocol + ADC/timer reconfig | unchanged API; cleanup only |
| `iirfilter.c`, `adc_comp_lut.c` *(kept)* | Pure DSP (IIR, LUT interpolation) | unchanged API; cleanup only |

After extraction, `main.c` is just:
`HAL_Init → SystemClock_Config → MX_*_Init → app_init(); while(1) app_run_once();` + `Error_Handler`.
HAL's weak callbacks (`HAL_ADC_ConvCpltCallback`, `HAL_TIM_PeriodElapsedCallback`, SPI callbacks)
are **defined inside the owning module** — no shims in `main.c`.

**Seam notes for Phase 2 (drawn deliberately, not acted on now):**
- `framing` hides *more than byte layout* — it owns whether timestamps are per-line or
  per-transaction, so the Phase-2 timestamp change is `framing`-local.
- `acquisition` owns filtering *placement*; the seam is documented as "filtering is STM-side
  today; Phase 2 may move it to the ESP32 (dumb-pipe)" so the extraction does not bake the
  filter location in.

Globals are pulled `static` into their owning module with accessors only where genuinely
cross-module (reducing `extern` sprawl) — this is *consolidation*, not the full
context-struct rework of Approach B.

### 5.1 Cross-repo protocol — single source of truth (Phase 1 deliverable)

The wire contract is currently **hand-duplicated** across repos: `spi_cmd_t`, the `STM32_CMD_*`
command enum, the message structs (`spi_msg_1_t`/`spi_msg_2_t`), `s_date_time_t`, and
`DATA_LINES_PER_SPI_TRANSACTION` all exist twice (STM `esp32_interface.h` / `main.h` vs ESP32
`spi_control.h` / `logger.c`) and are kept in sync by hand — a latent drift hazard.

Phase 1 introduces **one canonical `ul_protocol.h`** holding exactly these shared definitions,
plus a written **protocol spec** (handshake sequence, byte layout, timing, command semantics).
The header is **vendored** (copied) into both repos and guarded by a compile-time
`#define UL_PROTOCOL_VERSION n` + `_Static_assert`, so a mismatch between the two copies is
caught at build, not on the wire. In Phase 1 the centralization is **byte-identical** (pure
de-duplication, fully verifiable). In Phase 2 this single file is where the wire redesign lands
and the version is bumped — the cross-repo analogue of the `framing` keystone.

---

## 6. Data flow (preserved exactly)

```
TIM3 period-ISR      -> read RTC + GPIO; framing.append_line(current iirFilter[8])
                        -> on half full: mark ready, toggle ping-pong
ADC DMA half/full-ISR -> adc_comp() LUT correct; (16-bit) iir_filter() into work buf
app_run_once(LOGGING) -> if framing has a ready half: spi_ctrl_send(it)
```

The subtle ordering between `adc_is_half` / `adc_16b_is_half` / `gpio_is_half` and which struct
(`msg_1` vs `msg_2`) is transmitted is **relocated verbatim, not rewritten.**

---

## 7. Behavior preservation & known smells (document, do not fix in Phase 1)

These are preserved intact, with a comment + spec reference, flagged for deliberate Phase-2
review:

- `adc_16b_is_half = ~adc_16b_is_half` — bitwise-NOT (not logical `!`) on a flag.
- `busy` → `Error_Handler()` reentrancy guard inside the TIM3 ISR.
- `subseconds` "not 100 % correct" computation in the TIM3 ISR.
- The free-run + decimate behavior (§2.1) and the dead channel-disable path (§2.5) are
  **documented only** — no functional change in Phase 1.

---

## 8. Static safety net

Because Phase 1 (plain Approach A) is not host-unit-tested, the cheap compiler-level guard for
the wire format is mandatory: add `_Static_assert` on `sizeof(spi_msg_1_t)`, `sizeof(spi_msg_2_t)`,
the combined `data_buffer` size, and the byte **offset of every field**, pinned to today's
values. Any accidental layout change during the `framing` extraction becomes a **compile error**,
not a field bug found on the bench.

---

## 9. Verification (hardware in the loop)

Define the test matrix once, capture a **pre-refactor baseline**, and re-run after **every**
increment:

- **Matrix:** rates {1, 25, 100, 250 Hz} × {12-bit, 16-bit} × {CSV, raw} ×
  {continuous, external-trigger, single-shot}, against a real ESP32; confirm output files match
  the baseline. Include at least one sub-1 Hz averaging rate to exercise the ESP32 IIR path.
- **Per step:** clean compile; unchanged `arm-none-eabi-size`; all `_Static_assert`s hold.
- **Rollback:** any step that regresses on the bench is reverted in isolation (one step = one
  revertable commit).

---

## 10. Sequencing (each step = one flashable, bench-verified increment)

1. Dead-code / commented-block purge in place (no structural change) — **capture baseline matrix.**
2. Add the `_Static_assert` layout pins (§8).
3. Tidy `iirfilter` + `adc_comp_lut` (pure DSP).
4. Tidy `spi_ctrl` transport (drop unused `spi_ctrl_msg_sent`, etc.).
5. **Extract `framing.c/.h`** — biggest/keystone step; most careful bench test.
6. Extract `acquisition.c/.h` (ADC/TIM3 ISRs, `Adc_start`).
7. Extract `app.c/.h` (state machine, trigger/debounce, command dispatch).
8. Consolidate globals into owning modules; trim `extern`s.
9. **Introduce the canonical `ul_protocol.h` + version guard (§5.1), byte-identical**, and replace
   the duplicated definitions in *both* repos with it; write the protocol spec doc. Verify the
   `_Static_assert`s and the full bench matrix still pass on both chips.
10. Final pass + update README dev notes.

---

## 11. Phase 2 sketch (recorded target — NOT built in this spec)

Primary lever, then stacked roadmap. Channel-mask is **explicitly dropped** (variable-width
frames complicate the keystone module and the ESP32 parser for a gain that only helps
low-channel jobs and never the 8 ch @ 1 kHz worst case).

0. **Protocol redesign opens Phase 2.** The first Phase-2 step is the wire change *alone* —
   land it in the shared `ul_protocol.h`, bump `UL_PROTOCOL_VERSION`, update both repos, verify.
   Only then build the rate features on top. This keeps the refactor and the protocol change as
   separate, independently-verifiable variables.

1. **Per-transaction timestamp (primary).** Read the RTC **once per frame** (every N lines),
   store a base epoch + subsecond in the header, and reconstruct each line's time on the ESP32
   from the crystal-stable TIM3 period (`t = base + i / fs`). Effects: ~999/1000 fewer RTC reads;
   line shrinks 29 → 17 bytes (~40 % more lines per KB); timestamps become *more* accurate than
   today's jittery per-sample read. Localized to `framing` + a contained ESP32 parser change.

   **Buffer sizing is a tunable that stacks with this.** `DATA_LINES_PER_SPI_TRANSACTION`
   (today 70, 2-deep ping-pong, ~4 KB of 8 KB) becomes a named parameter; buffer depth
   (2-deep vs N-deep ring) is also a knob. After the 29 → 17 B line shrink the SRAM budget is
   friendly — e.g. ~150 lines × 2 ≈ 5 KB, or 3–4 shallower buffers — yielding fewer transactions
   and less DATA_RDY handshake overhead. Bigger buffers do **not** complicate timestamps here:
   each line is `base + index/fs`, costing one base capture per buffer and negligible crystal
   drift over the longer window. Pick the size against the live SRAM budget (stack + working
   buffers) once the line layout is fixed.
2. **Dumb-pipe (architectural).** STM ships **raw** ADC; ESP32 owns all LUT + IIR. Trivial STM
   ISR / near-zero-copy DMA→DMA → highest rate ceiling; raw-on-wire is re-calibratable; single
   place correction lives. Feasible because the ESP32 **already** has the filter machinery
   (`iir_filter_12b/16b`, `y_state`, `sharedBuffer`) — it is an *extension to all rates*, not a
   rebuild. **Open question (deferred):** where the per-unit calibration LUTs live — keep on STM
   and transmit at boot, or move to ESP32 settings/NVS. (`adc_comp_lut.c` currently compiles the
   per-unit "R04" tables into STM firmware.)
   - *Cheaper middle ground if Dumb-pipe is later declined:* apply the LUT only to the samples
     TIM3 keeps (the LUT is memoryless; the 16-bit IIR is recursive and must stay per-sample).
     This matters because `q_div` uses **64-bit software division on the divide-less M0+**, run
     8× per ADC callback at the free-run rate — a real hidden cost.
3. **Triggered acquisition (true high-rate).** Above ~250 Hz, switch to TIM3-`T3_TRGO`-triggered
   conversions (the original intent) so each line is a coherent fresh 8-channel set, with
   circular / N-deep DMA double-buffering and SPI sending the raw DMA half directly.

All Phase-2 work updates **both chips in lockstep** and should add a **protocol version byte**.

---

## 12. Out of scope (Phase 1)

- **Changing the SPI wire *bytes*.** Phase 1 may *relocate and centralize* the protocol
  definitions into `ul_protocol.h`, but the on-wire layout/handshake/timing stay identical;
  the redesign is the Phase-2 opener (§11.0).
- Moving filtering or calibration to the ESP32 (Phase 2 dumb-pipe).
- Bigger / N-deep transaction buffers (Phase 2 tunable, §11.1).
- Channel-mask on the wire (dropped entirely).
- Eliminating all globals via a context struct (Approach B).
- Host-side unit tests (may be revisited if Phase 2 needs them).

## 13. In scope, newly added (Phase 1)

- Canonical, vendored **`ul_protocol.h`** + `UL_PROTOCOL_VERSION` guard, replacing the
  hand-duplicated definitions in both repos, byte-identical (§5.1, step 9).
- A written **SPI protocol spec** documenting handshake, byte layout, timing, and command
  semantics — the clearer cross-repo interface description.
