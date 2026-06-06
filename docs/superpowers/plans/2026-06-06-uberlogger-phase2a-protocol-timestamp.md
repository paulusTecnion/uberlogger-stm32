# Phase 2A — Protocol v2 + Per-Transaction Timestamp — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Redesign the STM32↔ESP32 SPI wire protocol (`UL_PROTOCOL_VERSION` 1→2) around a per-transaction base timestamp (17 B/line, ESP32 reconstructs `t_i = base + i·period`), make buffer size/depth a tunable with an N-deep ring + real overrun reporting, and add 500 Hz / 1 kHz 12-bit-RAW logging on the existing free-run+decimate path.

**Architecture:** Sub-project A of the Phase-2 1 kHz arc (B=dumb-pipe, C=triggered follow as their own cycles). The wire change is centralized in the shared `ul_protocol.h`; almost all STM change concentrates in the `framing` module (Phase 1 quarantined wire layout there). Verification shifts from Phase-1's byte-identical diff to **semantic equivalence vs the v1 baseline + intrinsic high-rate validation + a zero-overrun release gate under concurrent API load**.

**Tech Stack:** C (GNU18) on STM32G030C6 (Cortex-M0+, 8 KB SRAM), STM32CubeIDE headless builder + STM32CubeProgrammer CLI; ESP-IDF (ESP32-S2) `idf.py`; Python 3 bench harness over the ESP32 HTTP API.

**Spec:** `docs/superpowers/specs/2026-06-06-uberlogger-phase2a-protocol-timestamp-design.md`

---

## Conventions & shared commands (read once)

Work happens on branch **`feature/phase2a-protocol-timestamp`** (STM repo) and **`refactor/ul-protocol-shared`** (ESP32 repo — already exists from Phase-1 Task 9; rebase/branch a fresh `feature/phase2a-protocol-timestamp` off it for this work). One commit per task. Never push.

### STM BUILD (machine-verifiable, ~2 s)
```bash
/opt/st/stm32cubeide_2.1.1/headless-build.sh \
  -data "$HOME/.stm32cubeide_ul_ws" \
  -import /home/paulus-potter/dev/uberlogger-stm32 \
  -build stm32g030c6/Debug 2>&1 | tail -12
```
- **Expected on success:** a `size` line + `Build Finished. 0 errors`.
- **Phase-1 end size (reference):** `text 21140  data 288  bss 6952`. v2 changes `bss` (buffers resized — that is EXPECTED here, unlike Phase 1). The hard rule: **`bss` must stay ≤ ~7600 B** (keep margin under the 8 KB / 8192 B ceiling for stack + working buffers); the implementer reports the new `bss` and we confirm headroom. `arm-none-eabi-size`:
  `find /opt/st/stm32cubeide_2.1.1 -name arm-none-eabi-size | head -1` then run it on `Debug/stm32g030c6.elf`.

### ESP32 BUILD (~minutes)
```bash
cd /home/paulus-potter/dev/uberlogger-esp32 && . /home/paulus-potter/esp-idf/export.sh >/dev/null 2>&1 && idf.py build 2>&1 | tail -25
```
- **Expected:** `Project build complete`, 0 errors, the `_Static_assert(UL_PROTOCOL_VERSION==2)` satisfied.

### FLASH (this round flashes BOTH chips)
- STM: `~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=SWD -w /home/paulus-potter/dev/uberlogger-stm32/Debug/stm32g030c6.elf -rst`
- ESP32 (USB-serial): `cd /home/paulus-potter/dev/uberlogger-esp32 && . /home/paulus-potter/esp-idf/export.sh && idf.py -p /dev/ttyACM0 flash`
- **Controller-coordinated; confirm prerequisites with the user before either flash** (ST-Link attached; ESP32 on USB-serial; device on its SoftAP at http://192.168.4.1; SD card inserted).

### v1 BASELINE (the equivalence oracle)
The Phase-1 metrics baseline `/tmp/ul_metrics_baseline.json` is **v1** data. Keep it. Equivalence (Task 7) compares v2 decoded output against it at the overlapping rates.

### VENDOR RULE (shared header)
`Core/Inc/ul_protocol.h` and `../uberlogger-esp32/main/ul_protocol.h` must stay **byte-identical** (`diff` clean). Edit the STM copy, then copy it verbatim to the ESP32 copy in the same task. The `UL_PROTOCOL_VERSION` build guard in each repo catches a stale copy.

### SEQUENCING NOTE (why bench is one integrated gate)
The wire changes atomically: the STM and ESP32 must BOTH speak v2 to exchange data. So Tasks 1–6 are **build-verified only** (each repo compiles; no mixed-version bench is meaningful). The **hardware bench gate is Task 7**, after both sides are v2-complete — flash both, then run equivalence + the overrun soak. Plan accordingly; do not attempt a hardware bench between Tasks 2–6.

---

## Task 1: Shared `ul_protocol.h` v2 (both repos)

Introduce the v2 frame contract + rate table + version command, byte-identical in both repos. No behavior yet; both repos must still compile.

**Files:**
- Modify: `Core/Inc/ul_protocol.h`, then copy verbatim to `../uberlogger-esp32/main/ul_protocol.h`
- (v1 structs `spi_msg_1_t`/`spi_msg_2_t`/`s_date_time_t` stay for now — Task 2 removes the STM use; ESP32 Task 4/5 removes its use. Keep them this task so both repos still build.)

- [ ] **Step 1: Bump the version and add the v2 contract**

In `Core/Inc/ul_protocol.h`, change the version and its assert:
```c
#define UL_PROTOCOL_VERSION 2
_Static_assert(UL_PROTOCOL_VERSION == 2, "ul_protocol.h: unexpected UL_PROTOCOL_VERSION");
```

Add a new command to the `stm32cmd_t` enum (before `CMD_UNKNOWN`), for the runtime version handshake:
```c
    STM32_CMD_SET_TRIGGER_MODE,
    STM32_CMD_GET_PROTOCOL_VERSION,   /* v2: ESP32 reads STM's UL_PROTOCOL_VERSION at boot */
    CMD_UNKNOWN
```

Append the v2 streaming contract after the v1 structs/asserts (keep v1 for now), guarded by a clear banner:
```c
/* ===========================================================================
 * v2 STREAMING FRAME (UL_PROTOCOL_VERSION 2) — per-transaction base timestamp.
 *
 * One SPI transaction = one frame: a 14-byte header + a fixed-stride payload of
 *   adc[capacity * 8] (uint16) then gpio[capacity] (uint8).
 * Only the first `line_count` lines are valid; `capacity` is the per-session
 * stride (LINES_PER_FRAME). Block layout (adc block, then gpio block) keeps the
 * uint16 ADC block 2-byte aligned (M0+ friendly) and is what the ADC DMA
 * produces (sub-project C ready). Each line's time is reconstructed on the
 * ESP32 as t_i = base_epoch + base_subsec/65536 + i * period_us(fs_code).
 * ===========================================================================*/
#define UL_ADC_CH                 8
#define UL_FRAME_START0           0xFA
#define UL_FRAME_START1           0xFB

/* RAW .dat CONTAINER versioning. The existing .dat container is KEPT as-is (see
 * uberlogger-esp32 front/www/convert_raw.py — the inverse reader): a 4-byte
 * header_length, then 9 settings bytes where settings[0] = "File format version",
 * then 8 x int32 adc_offsets, then channel labels; frames follow; a trailing
 * uint64 total-row-count ends the file. The adc_offsets + labels + ranges in that
 * header are REQUIRED by the offline count->voltage conversion and are unchanged.
 * v2 bumps ONLY settings[0] from 2 -> 3 to signal the new frame body inside;
 * convert_raw.py detects 2 (old spi_msg_1/2 frames) vs 3 (new v2 frames). No new
 * magic is introduced. (The per-frame start markers + protocol_version below are
 * the SPI-stream resync/guard, independent of the file container.) */
#define UL_RAW_FILE_FORMAT_VERSION   3

/* flags byte */
#define UL_FLAG_RES16             (1u << 0)     /* 0 = 12-bit, 1 = 16-bit */
/* bits 1..7 reserved for sub-projects B/C (raw-ADC/DMA-direct, triggered) */

/* v2 frame header — exactly 14 bytes, 4-byte aligned. */
typedef struct {
    uint8_t  start[2];          /* UL_FRAME_START0, UL_FRAME_START1 */
    uint8_t  protocol_version;  /* = UL_PROTOCOL_VERSION (2) */
    uint8_t  flags;             /* UL_FLAG_RES16 | reserved */
    uint32_t base_epoch;        /* unix seconds at line 0 (RTC, once per frame) */
    uint16_t base_subsec;       /* Q16 fractional second (units of 1/65536 s) */
    uint8_t  fs_code;           /* LOG_SAMPLE_RATE enum index -> ul_period_us() */
    uint8_t  line_count;        /* valid lines in this frame (<= capacity) */
    uint8_t  capacity;          /* per-session stride = LINES_PER_FRAME */
    uint8_t  pad;               /* alignment / reserved */
} ul_frame_hdr_t;

_Static_assert(sizeof(ul_frame_hdr_t) == 14, "ul_frame_hdr_t must be 14 bytes on the wire");
_Static_assert(offsetof(ul_frame_hdr_t, base_epoch) == 4,  "ul_frame_hdr_t.base_epoch moved");
_Static_assert(offsetof(ul_frame_hdr_t, base_subsec) == 8, "ul_frame_hdr_t.base_subsec moved");
_Static_assert(offsetof(ul_frame_hdr_t, fs_code) == 10,    "ul_frame_hdr_t.fs_code moved");

/* Per-line wire size: 8 ADC uint16 (16 B, in the adc block) + 1 GPIO byte. */
#define UL_LINE_BYTES   (UL_ADC_CH * 2 + 1)   /* = 17 */

/* fs_code (== LOG_SAMPLE_RATE enum) -> per-line period in microseconds.
 * 0 = sub-1Hz averaging path / unknown (no fixed period). Shared by both repos
 * so the ESP32 reconstruction and the STM >250Hz guard agree. */
static inline uint32_t ul_period_us(uint8_t fs_code) {
    switch (fs_code) {
        case 5:  return 1000000u; /* 1 Hz   */
        case 6:  return 500000u;  /* 2 Hz   */
        case 7:  return 200000u;  /* 5 Hz   */
        case 8:  return 100000u;  /* 10 Hz  */
        case 9:  return 40000u;   /* 25 Hz  */
        case 10: return 20000u;   /* 50 Hz  */
        case 11: return 10000u;   /* 100 Hz */
        case 12: return 4000u;    /* 250 Hz */
        case 13: return 2000u;    /* 500 Hz */
        case 14: return 1000u;    /* 1000 Hz*/
        default: return 0u;       /* averaging / unknown */
    }
}

/* True for rates that are 12-bit-RAW-only (CSV + 16-bit disabled). */
static inline int ul_is_high_rate(uint8_t fs_code) { return fs_code >= 13; }
```

> Note: `static inline` in a shared header is fine for both toolchains (GNU18 / IDF gcc) and was the approach validated in Phase 1. `offsetof` needs `<stddef.h>` (already included).

- [ ] **Step 2: Vendor the identical copy to the ESP32 repo**

```bash
cp /home/paulus-potter/dev/uberlogger-stm32/Core/Inc/ul_protocol.h /home/paulus-potter/dev/uberlogger-esp32/main/ul_protocol.h
diff /home/paulus-potter/dev/uberlogger-stm32/Core/Inc/ul_protocol.h /home/paulus-potter/dev/uberlogger-esp32/main/ul_protocol.h && echo IDENTICAL
```
Expected: `IDENTICAL`. The ESP32's existing `_Static_assert(UL_PROTOCOL_VERSION==…)` (in `main/logger.c`, from Phase-1 Task 9) updates automatically to ==2 because it reads the macro — confirm it isn't hard-coded to 1 (if it is, it lives in the header guard region; the header carries the canonical assert).

- [ ] **Step 3: Build BOTH repos**

Run STM BUILD → `0 errors` (v1 + v2 both defined; the new `ul_frame_hdr_t` asserts hold). Run ESP32 BUILD → `Project build complete`, version guard now ==2.
Expected STM size: ~unchanged from `21140/288/6952` (only added types + inline fns; no instances yet).

- [ ] **Step 4: Commit (both repos)**

```bash
cd /home/paulus-potter/dev/uberlogger-stm32 && git add Core/Inc/ul_protocol.h \
  && git commit -m "feat(phase2a): ul_protocol.h v2 — frame header, rate table, version cmd"
cd /home/paulus-potter/dev/uberlogger-esp32 && git checkout -b feature/phase2a-protocol-timestamp \
  && git add main/ul_protocol.h \
  && git commit -m "feat(phase2a): vendor ul_protocol.h v2 (byte-identical)"
```

---

## Task 2: STM `framing` v2 — single frame struct, base capture, ring + overrun

Replace the twin-struct/half-bookkeeping `framing` with the v2 single parametric frame struct, an N-deep ring, per-frame base-timestamp capture, and real overrun reporting.

**Files:**
- Modify: `Core/Inc/framing.h`, `Core/Src/framing.c`
- Test (static): the layout `_Static_assert`s in `ul_protocol.h` (Task 1) + new ones in `framing.c`.

- [ ] **Step 1: Rewrite `framing.h` (the v2 contract)**

```c
/* framing.h — v2 SPI framing: owns the frame ring, the per-transaction base
 * timestamp, and the 17 B/line block layout (refactor + phase2a specs).
 * Phase-2A: per-transaction base timestamp; ESP32 reconstructs per-line time. */
#ifndef _FRAMING_H
#define _FRAMING_H
#include "stdint.h"
#include "ul_protocol.h"      /* ul_frame_hdr_t, UL_ADC_CH, flags, period table */
#include "esp32_interface.h"  /* adc_resolution_t */

/* Compile-time max lines per frame; runtime LINES_PER_FRAME (capacity) <= this.
 * Sized against the SRAM budget — see Step 4. */
#define UL_LINES_MAX     128
/* Number of frame buffers in the ring (>=2). Tunable; see Step 4. */
#define UL_FRAME_DEPTH   4

void     frame_init(void);                         /* boot: zero ring, set markers */
void     frame_reset(void);                        /* logging start: reset ring/ptrs/overrun */
void     frame_set_lines_per_transaction(uint8_t n);   /* config: set capacity (<= UL_LINES_MAX) */

/* Begin a new frame if needed and append one sample line. Called from the TIM3
 * sample-tick ISR. On the first line of a frame, captures base via the supplied
 * epoch/subsec. gpio = GPIOB high byte; adc = 8 corrected/filtered u16; res
 * selects the flags bit. Returns 1 when a frame just became ready, else 0.
 * If no free ring slot exists, sets the overrun flag and drops the line. */
uint8_t  frame_append_line(uint32_t base_epoch, uint16_t base_subsec, uint8_t fs_code,
                           uint8_t gpio, const uint16_t *adc, adc_resolution_t res);

/* Hand back the oldest ready frame's buffer + on-wire byte length; advance the
 * read cursor and free the slot. Returns 1 if one was pending, else 0. */
uint8_t  frame_take_ready(uint8_t **buf, uint16_t *len);

/* For SEND_LAST/single-shot: hand back the current (partially) filled frame. */
void     frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot, adc_resolution_t res);

/* Overrun flag (set when the ring was full at append time). Cleared by frame_reset(). */
uint8_t  frame_overrun(void);
void     frame_clear_overrun(void);

/* True iff the NEXT frame_append_line() will start a new frame (line position 0).
 * The TIM3 ISR uses this to read the RTC base exactly once per frame (single
 * source of truth — no separate mirror counter that could desync on overrun). */
uint8_t  frame_at_line_zero(void);

#endif
```

- [ ] **Step 2: Rewrite `framing.c` — the v2 frame, ring, and overrun**

Replace the whole body (keep the MIT header). The on-wire length for a frame is `sizeof(ul_frame_hdr_t) + capacity*UL_LINE_BYTES` (fixed stride; gpio block starts at `hdr + capacity*16`).

```c
#include "framing.h"
#include "string.h"

/* One ring slot: a max-sized frame. Only [0..line_count) lines are valid; the
 * gpio block sits at the fixed offset capacity*16 after the header so the layout
 * is stride-stable (ESP32 finds gpio without per-frame offset math). */
typedef struct {
    ul_frame_hdr_t hdr;
    uint16_t adc[UL_LINES_MAX * UL_ADC_CH];   /* aligned: hdr is 14 (even) */
    uint8_t  gpio[UL_LINES_MAX];
} ul_frame_buf_t;

_Static_assert(offsetof(ul_frame_buf_t, adc) == 14, "v2 adc block must follow the 14B header");

static ul_frame_buf_t ring[UL_FRAME_DEPTH];
static volatile uint8_t  wr_slot   = 0;   /* slot being filled */
static volatile uint8_t  rd_slot   = 0;   /* oldest ready slot */
static volatile uint8_t  ready_cnt = 0;   /* frames ready to send */
static volatile uint8_t  line_idx  = 0;   /* line within wr_slot */
static volatile uint8_t  overrun   = 0;
static uint8_t capacity = DATA_LINES_PER_SPI_TRANSACTION;  /* default 70; reset in init */

/* On-wire bytes for a frame of the current capacity. */
static inline uint16_t frame_wire_len(void) {
    return (uint16_t)(sizeof(ul_frame_hdr_t) + (uint32_t)capacity * UL_LINE_BYTES);
}

void frame_init(void) {
    memset(ring, 0, sizeof(ring));
    wr_slot = rd_slot = ready_cnt = line_idx = overrun = 0;
    if (capacity == 0 || capacity > UL_LINES_MAX) capacity = UL_LINES_MAX;
}

void frame_reset(void) {
    wr_slot = rd_slot = ready_cnt = line_idx = overrun = 0;
}

void frame_set_lines_per_transaction(uint8_t n) {
    if (n == 0) n = 1;
    if (n > UL_LINES_MAX) n = UL_LINES_MAX;
    capacity = n;
}

uint8_t frame_overrun(void)        { return overrun; }
void    frame_clear_overrun(void)  { overrun = 0; }
uint8_t frame_at_line_zero(void)   { return (uint8_t)(line_idx == 0); }

uint8_t frame_append_line(uint32_t base_epoch, uint16_t base_subsec, uint8_t fs_code,
                          uint8_t gpio, const uint16_t *adc, adc_resolution_t res) {
    /* If the current slot is full of unsent frames, we cannot start/continue: overrun. */
    if (line_idx == 0) {
        if (ready_cnt >= UL_FRAME_DEPTH) { overrun = 1; return 0; }  /* ring full: drop */
        ul_frame_hdr_t *h = &ring[wr_slot].hdr;
        h->start[0] = UL_FRAME_START0; h->start[1] = UL_FRAME_START1;
        h->protocol_version = UL_PROTOCOL_VERSION;
        h->flags    = (res == ADC_16_BITS) ? UL_FLAG_RES16 : 0;
        h->base_epoch  = base_epoch;
        h->base_subsec = base_subsec;
        h->fs_code   = fs_code;
        h->capacity  = capacity;
        h->line_count = 0;
        h->pad = 0;
    }
    ul_frame_buf_t *f = &ring[wr_slot];
    memcpy(&f->adc[line_idx * UL_ADC_CH], adc, UL_ADC_CH * 2);  /* 8 x u16 */
    f->gpio[line_idx] = gpio;
    line_idx++;
    f->hdr.line_count = line_idx;

    if (line_idx >= capacity) {           /* frame full -> mark ready, advance ring */
        ready_cnt++;
        wr_slot = (uint8_t)((wr_slot + 1) % UL_FRAME_DEPTH);
        line_idx = 0;
        return 1;
    }
    return 0;
}

uint8_t frame_take_ready(uint8_t **buf, uint16_t *len) {
    if (ready_cnt == 0) return 0;
    *buf = (uint8_t*)&ring[rd_slot];
    *len = frame_wire_len();
    rd_slot = (uint8_t)((rd_slot + 1) % UL_FRAME_DEPTH);
    ready_cnt--;
    return 1;
}

void frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot, adc_resolution_t res) {
    (void)singleshot; (void)res;
    /* Hand back the in-progress slot with its real line_count (partial frame). */
    ul_frame_buf_t *f = &ring[wr_slot];
    f->hdr.line_count = line_idx;
    *buf = (uint8_t*)f;
    *len = (uint16_t)(sizeof(ul_frame_hdr_t) + (uint32_t)capacity * UL_LINE_BYTES);
}
```

> The legacy `~adc_16b_is_half` toggle and twin-struct selection are GONE (the §7 smells they embodied are resolved by the single-ring design). `frame_take_last` always sends a fixed-stride buffer with the real `line_count`; the ESP32 reads only valid lines.

- [ ] **Step 3: Build**

Run STM BUILD. Expected `0 errors`; the new `_Static_assert`s hold. `bss` changes (the ring is `UL_FRAME_DEPTH × sizeof(ul_frame_buf_t)`). Compute and report it.

- [ ] **Step 4: Size the ring against the SRAM budget**

`sizeof(ul_frame_buf_t) = 14 + UL_LINES_MAX*17` (rounded for alignment). With `UL_LINES_MAX=128` → ~2190 B/slot; `UL_FRAME_DEPTH=4` → ~8.7 KB — **too big for 8 KB**. Tune so the ring fits with margin: report the measured `bss`, then adjust `UL_LINES_MAX`/`UL_FRAME_DEPTH` so total `bss ≤ ~7600 B`. Suggested starting point to evaluate: `UL_LINES_MAX=70`, `UL_FRAME_DEPTH=4` (~4 slots × ~1204 B ≈ 4.8 KB) — leaves room and gives 4× the stall slack of ping-pong. Record the chosen values + the resulting `bss` in the commit message. (Final tuning is confirmed by the Task-7 overrun soak.)

- [ ] **Step 5: Commit**

```bash
git add Core/Inc/framing.h Core/Src/framing.c
git commit -m "feat(phase2a): framing v2 — single frame ring, base timestamp, overrun"
```

---

## Task 3: STM `acquisition` + `config` — base capture hook, 500/1000 Hz, >250 guard

Wire the new framing into the TIM3 ISR (capture base + new append signature), add the high rates, and enforce 12-bit-RAW-only above 250 Hz.

**Files:**
- Modify: `Core/Src/acquisition.c` (the TIM3 period ISR + base capture), `Core/Src/config.c` (`Config_Set_Sample_freq` rate table + the >250 guard), `Core/Inc/esp32_interface.h` (rate enum values 500/1000 if not present).

- [ ] **Step 1: Add the 500/1000 Hz rates to the enum**

In `Core/Inc/esp32_interface.h`, ensure the `LOG_SAMPLE_RATE`/`adc_sample_rate_t` enum has codes 13 (500 Hz) and 14 (1000 Hz) consistent with `ul_period_us()` (Task 1). If the enum uses named members, add `ADC_SAMPLE_RATE_500Hz = 13, ADC_SAMPLE_RATE_1000Hz = 14`. (Grep the current enum first; match its style.)

- [ ] **Step 2: Un-comment + set the 500/1000 Hz TIM3 prescaler/period in `config.c`**

In `Config_Set_Sample_freq` (`config.c:353`), the Phase-2 reference cases are commented at `config.c:486-497`. At 64 MHz timer clock, for 500 Hz: `Prescaler = 64-1, Period = 2000` (64e6/64/2000 = 500); for 1000 Hz: `Prescaler = 64-1, Period = 1000` (= 1000 Hz). Add live `case ADC_SAMPLE_RATE_500Hz:` / `case ADC_SAMPLE_RATE_1000Hz:` setting `htim3.Init.Prescaler`/`Period` accordingly, mirroring the existing live cases' surrounding code (the `HAL_TIM_Base_Init(&htim3)` / re-init call that the other cases use). Verify the exact reload math against an existing known case (e.g. 250 Hz) before committing the numbers.

- [ ] **Step 3: Enforce the >250 Hz guard (STM side)**

In `config.c`, where resolution and logmode are set (`STM32_CMD_SET_RESOLUTION`, `STM32_CMD_SET_LOGMODE`, and `Config_Set_Sample_freq`), add: when the active `fs_code` satisfies `ul_is_high_rate(fs_code)` (≥500 Hz), force 12-bit resolution and reject 16-bit / CSV (return the NOK response the command path already uses). This is the STM backstop; the ESP32 (Task 5) is the primary guard. Keep it minimal — set resolution to `ADC_12_BITS` and refuse a CSV/16-bit selection at high rate.

- [ ] **Step 4: Capture base + call the new append in the TIM3 ISR**

In `acquisition.c`'s `HAL_TIM_PeriodElapsedCallback` TIM3 branch, replace the `frame_append_line(&current_date_time, gpio, iirFilter, adc_resolution)` call with the v2 signature. On the first line of a frame the framing reads the base from the args, so compute `base_epoch`/`base_subsec` here **once per frame**. Since framing tracks `line_idx` internally, compute the base every tick but framing only uses it on line 0 — OR expose a cheap "is this the first line" via the return path. Simplest correct approach: read the RTC every tick into a local and pass it; framing only stores it on the first line. To avoid an RTC read every tick at 1 kHz (the cost we are removing), instead gate the RTC read in the ISR:

```c
/* in acquisition.c, TIM3 branch */
uint32_t epoch = 0; uint16_t subsec = 0; uint8_t fs = current_fs_code();  /* fs_code accessor from config */
if (frame_at_line_zero()) {                               /* single source of truth (Task 2) */
    RTC_TimeTypeDef t; RTC_DateTypeDef d;
    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);           /* MUST read date after time to unlock shadow regs */
    epoch  = ul_make_epoch(&d, &t);                        /* helper: y/m/d h:m:s -> unix seconds */
    subsec = (uint16_t)(((uint32_t)(t.SecondFraction - t.SubSeconds) << 16) / (t.SecondFraction + 1)); /* Q16 frac */
}                                                          /* else epoch/subsec ignored by framing (not line 0) */
uint8_t ready = frame_append_line(epoch, subsec, fs, (uint8_t)(GPIOB->IDR >> 8), iirFilter, adc_resolution);
```

`frame_at_line_zero()` (Task 2) is the single line-position source — no mirror counter, so this stays correct across overrun-drops (when the ring is full, framing doesn't advance and `frame_at_line_zero()` stays true, so the next recovered frame gets a fresh base). Add the small helpers: `ul_make_epoch()` (a date/time→unix-seconds converter; place in `acquisition.c` or a tiny util — implement with a standard days-since-epoch calc, no libc `mktime` to keep it ISR-cheap) and `current_fs_code()` (config owns the active rate code). Keep the RTC `GetTime`→`GetDate` ordering (HAL requires reading date to unlock the shadow registers).

> The §7 `busy`/`Error_Handler` guard stays as-is around this branch.

- [ ] **Step 5: Build + size**

Run STM BUILD → `0 errors`, asserts hold, report `bss` (within budget). Fix any signature mismatch.

- [ ] **Step 6: Commit**

```bash
git add Core/Src/acquisition.c Core/Src/config.c Core/Inc/esp32_interface.h Core/Inc/framing.h
git commit -m "feat(phase2a): TIM3 base-timestamp capture; 500/1000Hz rates; >250Hz 12b/RAW guard"
```

---

## Task 4: ESP32 — v2 frame parser, timestamp reconstruction, version handshake

Teach the ESP32 to consume v2 frames and reconstruct per-line timestamps; add the boot-time version handshake.

**Files:**
- Modify: `main/logger.c` (frame consumption + reconstruction), `main/spi_control.*` if the command path needs the new `STM32_CMD_GET_PROTOCOL_VERSION`.

> The implementer MUST first read the current v1 frame-consumption path in `main/logger.c` (where it interprets `spi_msg_1_t`/`spi_msg_2_t`, reads `timeData[i]`, `gpioData[i]`, `adcData16[]`) — locate it, then replace it with the v2 decode below. Follow existing buffer/threading patterns.

- [ ] **Step 1: v2 frame decode + reconstruction helper**

Add a decoder that, given a received frame buffer, validates `start`/`protocol_version==2`, reads the header, and for each valid line `i` (`0..line_count`) computes:
```c
/* time of line i, in microseconds since epoch */
uint64_t line_us(const ul_frame_hdr_t *h, uint32_t i) {
    uint64_t base = (uint64_t)h->base_epoch * 1000000ull
                  + ((uint64_t)h->base_subsec * 1000000ull >> 16);   /* Q16 frac -> us */
    return base + (uint64_t)i * ul_period_us(h->fs_code);
}
/* adc block at (uint16*)(frame + 14); gpio block at (uint8*)(frame + 14 + capacity*16) */
const uint16_t *adc  = (const uint16_t*)((const uint8_t*)frame + sizeof(ul_frame_hdr_t));
const uint8_t  *gpio = (const uint8_t*)frame + sizeof(ul_frame_hdr_t) + (uint32_t)h->capacity * UL_ADC_CH * 2;
/* line i: adc[i*8 + ch], gpio[i] */
```
Feed each reconstructed line into the existing CSV/RAW writer path (the writer changes are Task 5; here, produce the decoded `(timestamp, gpio, adc[8])` per line the way the v1 path did).

- [ ] **Step 2: Runtime version handshake**

At logger startup / first STM contact, issue `STM32_CMD_GET_PROTOCOL_VERSION` via the existing `spi_ctrl_cmd()` path; the STM responds with its `UL_PROTOCOL_VERSION` in the response byte. If it != `UL_PROTOCOL_VERSION` (==2 here), log an error and refuse to start logging (set an error code the UI surfaces) rather than parsing mismatched frames. Add the matching responder on the STM side in `config.c`'s command dispatch (a one-line case returning `UL_PROTOCOL_VERSION`) — **note:** this STM responder belongs in Task 3's command handling; if not added there, add it here and amend. Keep both in sync.

- [ ] **Step 3: Build ESP32**

Run ESP32 BUILD → `Project build complete`, 0 errors.

- [ ] **Step 4: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-esp32
git add main/logger.c main/spi_control.c main/spi_control.h
git commit -m "feat(phase2a): ESP32 v2 frame decode + timestamp reconstruction + version handshake"
```

---

## Task 5: ESP32 — CSV (≤250 Hz) / RAW v2 output + >250 Hz reject + frontend disable

Write the reconstructed data: CSV unchanged (≤250 Hz) and RAW v2 (file header + frames); enforce the 12-bit-RAW-only rule at >250 Hz in the API and UI.

**Files:**
- Modify: `main/logger.c` (CSV writer uses reconstructed times; RAW writer emits v2 file header + raw frames), `main/rest_server.c` (`setConfig` guard), `front/www/js/config.js` + `config.html` (disable 16-bit/CSV when rate >250 Hz).

- [ ] **Step 1: CSV writer (≤250 Hz) — reconstructed timestamps, same columns**

In the CSV path, format each line's timestamp from `line_us()` (Task 4) into the SAME column format the v1 CSV used (locate the existing `strftime`/snprintf of the timestamp column and feed it the reconstructed time). Columns, order, and number formatting stay identical. CSV is only reachable at ≤250 Hz (guard below).

- [ ] **Step 2: RAW v2 writer — keep the container, bump the version, swap the frame body**

The ESP32 already writes the `.dat` container (locate the current RAW file-creation + append in `main/logger.c`; its format is the exact inverse of `front/www/convert_raw.py`'s reader: `[4B header_length][9 settings bytes][8×int32 adc_offsets][labels…]`, frames, then a trailing `uint64` total-row count). **Keep all of that** — the `adc_offsets`/labels/ranges are needed by the offline converter. Two changes only:
1. Set the container's **"File format version" field (settings[0]) to `UL_RAW_FILE_FORMAT_VERSION` (3)** instead of 2.
2. Replace the frame body: instead of appending alternating `spi_msg_1_t`/`spi_msg_2_t` structs, append each received **v2 frame's on-wire bytes verbatim** (header + adc block + gpio block; length `14 + capacity*17`). The trailing total-row count still accumulates `Σ line_count`.

So the `.dat` becomes `[container header, version=3][v2 frame][v2 frame]…[uint64 total_rows]`. Old `.dat`s keep version=2; `convert_raw.py` (Task 6) branches on settings[0].

- [ ] **Step 3: `setConfig` >250 Hz guard (backend)**

In `main/rest_server.c` `setConfig`, after parsing `LOG_SAMPLE_RATE`/`ADC_RESOLUTION`/`LOG_MODE`: if `ul_is_high_rate(fs_code)` and (`ADC_RESOLUTION==16` or `LOG_MODE==CSV`), `json_send_resp(..., ENDPOINT_RESP_NACK, "Above 250 Hz only 12-bit RAW logging is supported", HTTPD_400_BAD_REQUEST)` and do not apply. (Match the existing NACK helper + status code style in this file.)

- [ ] **Step 4: Frontend disable**

In `front/www/js/config.js` (+ `config.html`), when the selected sample rate is >250 Hz, disable/grey the 16-bit resolution option and the CSV log-mode option (force 12-bit + RAW), with a short note ("12-bit RAW only above 250 Hz"). Mirror the reachability-aware UX pattern already used for the Wi-Fi/NTP notices. Rebuild the web assets if the build bundles them (`www.bin`).

- [ ] **Step 5: Build ESP32**

Run ESP32 BUILD → `Project build complete`, 0 errors. (If `www.bin` is built from `front/`, confirm the asset build step ran.)

- [ ] **Step 6: Commit**

```bash
git add main/logger.c main/rest_server.c front/www/js/config.js front/www/config.html
git commit -m "feat(phase2a): ESP32 CSV(reconstructed)/RAW v2 output + >250Hz 12b-RAW guard + UI"
```

---

## Task 6: Bench tooling — v2 structural checks, auto-detect, equivalence, overrun soak

Extend the Python harness for v2 and add the equivalence + overrun-soak-under-load drivers.

**Files:**
- Modify: `tools/bench/ul_verify.py` (v2 RAW parse + v1/v2 auto-detect; CSV equivalence helpers). Mirror the copy into `/tmp/ul_verify.py` (the runner imports from `/tmp`).
- Create: `tools/bench/ul_soak.py` (overrun soak under concurrent API load).
- Modify: `../uberlogger-esp32/front/www/convert_raw.py` (production offline RAW→CSV; v3 container detect + v2-frame parse + timestamp reconstruction). **Critical: the only CSV path for >250 Hz logs.**

- [ ] **Step 1: v2 RAW parse + auto-detect in `ul_verify.py`**

A device-captured `.dat` is the full container: `[uint32 header_length][9 settings bytes …]`. Detect via the container's **file-format-version field** = `settings[0]`, the byte at offset 4: `>=3` → v2 frames, else v1. Add `analyze_raw_v2(data)`: skip the container header (`header_length` = first uint32 LE), then iterate v2 frames — validate `start == FA FB`, `protocol_version==2`, read `base_epoch/base_subsec/fs_code/line_count/capacity`; reconstruct each line's time; assert monotonic timestamps, inferred period == `ul_period_us(fs_code)`, frame stride == `14 + capacity*17`. Keep the existing v1 `analyze_raw` for old files. Return a metrics dict comparable across builds.

```python
PERIOD_US = {5:1000000,6:500000,7:200000,8:100000,9:40000,10:20000,11:10000,12:4000,13:2000,14:1000}
def detect_format(data):
    # .dat container: [uint32 header_length][settings[0] = file format version, ...]
    fmt_ver = data[4] if len(data) > 4 else 0
    return "v2" if fmt_ver >= 3 else "v1"   # 2 = old spi_msg_1/2 frames, 3 = v2 frames
```

- [ ] **Step 2: CSV equivalence helper**

Add `compare_csv_equiv(v1_metrics, v2_metrics)`: for an overlapping config, assert v2 `inferred_hz == v1 inferred_hz`, column layout identical, ADC ranges within the existing tolerance, and reconstructed timestamp spacing within the RTC resolution (~0.49 ms) of v1's. Return pass/fail + detail.

- [ ] **Step 3: `ul_soak.py` — overrun soak under concurrent API load**

```python
# Drives logging at a target rate while hammering the HTTP API; passes iff overrun==0.
# Usage: python3 tools/bench/ul_soak.py --rate 14 --dur 600   (rate idx, seconds)
# - starts RAW 12-bit logging at the rate
# - spawns concurrent threads polling /ajax/getStatus, /ajax/getValues (live single-shot),
#   /ajax/getRawAdc, /ajax/getConfig, /ajax/getFileList throughout the soak
# - after stop, checks the device's overrun status (see Step 4) and the captured file's
#   frame continuity (no gaps, line_counts consistent). PASS iff overrun==0 AND continuous.
```
Implement the threads with `concurrent.futures`/`threading`; reuse the `_req` helper pattern from `ul_verify.py`. The pass criterion is **zero overrun**.

- [ ] **Step 4: Expose overrun status to the harness**

The soak needs to read the device's overrun flag. Plumb `frame_overrun()` (STM, Task 2) → an ESP32 status field (e.g. add `"OVERRUN"` to `/ajax/getStatus` JSON, fed by a `STM32_CMD`/status byte the STM already reports, or a dedicated query). Add the minimal STM→ESP32→JSON path. (If this requires STM/ESP32 firmware edits, make them here and amend Tasks 3/4/5 commits or add a small follow-up commit — keep `OVERRUN` in `getStatus`.)

- [ ] **Step 5: Update `convert_raw.py` — the production offline RAW→CSV converter (FIRST-CLASS: it is the ONLY CSV path for >250 Hz logs)**

File: `uberlogger-esp32/front/www/convert_raw.py` (canonical; the `release/.../Converter-tool/` copies are downstream — leave them, they ship per-release). Read the current reader first; **keep `read_file_header`/`decode_file_header`/`convert_adc`/`csv_write`/`NTC_table` unchanged** (the container header, calibration offsets, and count→voltage/temperature math do not change in A). Changes:
1. Accept both file format versions: `2` (old) and `3` (v2). Branch on `decoded_settings[0]`.
2. For version 3, replace `read_spi_msg_1`/`read_spi_msg_2` with a `read_v2_frame(file)` that parses the 14-byte header, the `adc[capacity*8]` block, and the `gpio[capacity]` block, returning the first `line_count` lines.
3. **Reconstruct per-line timestamps** into the `date_time_list` tuples `csv_write()` expects — `(year, month, day, hours, minutes, seconds, _, _, subseconds_ms)`:
```python
import datetime
PERIOD_US = {5:1000000,6:500000,7:200000,8:100000,9:40000,10:20000,11:10000,12:4000,13:2000,14:1000}
def reconstruct_times(base_epoch, base_subsec_q16, fs_code, line_count):
    base_us = base_epoch*1_000_000 + (base_subsec_q16*1_000_000)//65536
    per = PERIOD_US.get(fs_code, 0)
    out = []
    for i in range(line_count):
        t_us = base_us + i*per
        dt = datetime.datetime.utcfromtimestamp(t_us/1_000_000)
        ms = int((t_us % 1_000_000)/1000)        # csv_write formats subseconds as :03d ms
        out.append((dt.year-2000, dt.month, dt.day, dt.hour, dt.minute, dt.second, 0, 0, ms))
    return out
```
   Feed `convert_adc()` the `adc` list and `csv_write()` the reconstructed `date_time_list` exactly as the v1 loop does, so the produced CSV is column-identical to today. Bump `VERSION_STRING` to `"V2.0"`.

- [ ] **Step 6: Commit (both repos)**

```bash
cd /home/paulus-potter/dev/uberlogger-stm32
git add tools/bench/ul_verify.py tools/bench/ul_soak.py
git commit -m "test(phase2a): bench harness v2 — RAW v2 parse, auto-detect, equivalence, overrun soak"
cp tools/bench/ul_verify.py /tmp/ul_verify.py
cd /home/paulus-potter/dev/uberlogger-esp32
git add front/www/convert_raw.py
git commit -m "feat(phase2a): convert_raw.py reads v2 RAW (file format v3) + reconstructs timestamps"
```

---

## Task 7: Integrated hardware gate — flash both, equivalence + overrun release gate

The single hardware gate for A. **Confirm prerequisites with the user before flashing.**

**Files:** none (verification task; results recorded in the commit message of Task 8).

- [ ] **Step 1: Flash both chips**

Confirm with the user (ST-Link + ESP32 USB-serial + SoftAP + SD). Flash STM, then ESP32 (commands in Conventions). Confirm the device boots, the version handshake passes (no mismatch error), and `http://192.168.4.1/ajax/getStatus` responds.

- [ ] **Step 2: Equivalence vs v1 baseline (≤250 Hz)**

For each overlapping config (1/25/100/250 Hz × 12/16-bit, CSV + RAW), capture with `ul_matrix.py task7a` (or per-config `ul_verify.py`) and compare to `/tmp/ul_metrics_baseline.json` (v1) using the equivalence helper (Task 6): inferred_hz equal, columns identical, ADC in tolerance, reconstructed timestamps within ~0.49 ms of v1. Record PASS/FAIL per config.

- [ ] **Step 3: New-rate intrinsic validation (500/1000 Hz, RAW 12-bit)**

Capture 500 Hz and 1000 Hz RAW; validate via `analyze_raw_v2`: monotonic reconstructed time, inferred period == expected, ADC in-range/not-stuck, frame continuity. Record metrics.

- [ ] **Step 4: THE OVERRUN RELEASE GATE (under concurrent API load)**

For 500 Hz and 1000 Hz separately:
```bash
python3 tools/bench/ul_soak.py --rate 13 --dur 600   # 500 Hz, 10 min, API load on
python3 tools/bench/ul_soak.py --rate 14 --dur 600   # 1000 Hz
```
**PASS iff `overrun == 0` for the full soak.** Run on the slowest SD card we intend to support if available. **A rate that overruns is NOT released** — drop it from the supported set (and from the frontend/backend allow-list) and record it as deferred to sub-project C. If a rate fails, optionally re-tune `UL_FRAME_DEPTH`/`UL_LINES_MAX` (Task 2) within the SRAM budget and re-soak once; if it still overruns, defer.

- [ ] **Step 5: Record the release decision**

Write down: which rates passed the gate (released) and which were deferred; the final `bss`; the equivalence results. These feed Task 8's commit message and the README rate table.

---

## Task 8: Docs — protocol v2 spec, README rate table, results

**Files:**
- Modify: `docs/protocol/uberlogger-spi-protocol.md` (bump to v2), `README.md`.

- [ ] **Step 1: Protocol doc → v2**

Update `docs/protocol/uberlogger-spi-protocol.md`: document `UL_PROTOCOL_VERSION 2`, the v2 frame header (the 14-byte table from Task 1), the block payload layout (adc block then gpio block; stride `14 + capacity*17`), the reconstruction formula `t_i = base + i·period(fs_code)` with the rate table, the RAW `.dat` **container** (the existing `[header_length][settings][adc_offsets][labels]…frames…[uint64 total_rows]` format is kept; the **file-format-version `settings[0]` is bumped 2→3** to signal v2 frames; `convert_raw.py` reads both), the `STM32_CMD_GET_PROTOCOL_VERSION` handshake, and the >250 Hz 12-bit-RAW-only rule. Note `ul_protocol.h` remains the machine-readable source of truth.

- [ ] **Step 2: README rate table + results**

Update `README.md`: the supported matrix (≤250 Hz: 12/16-bit, CSV/RAW; **500/1000 Hz: 12-bit RAW only**, with the released subset from Task 7), the v2 protocol note, and the both-chip lockstep-flash requirement. Record the final `bss` and which high rates passed the overrun gate.

- [ ] **Step 3: Commit (record Task-7 results in the message)**

```bash
cd /home/paulus-potter/dev/uberlogger-stm32
git add docs/protocol/uberlogger-spi-protocol.md README.md
git commit -m "docs(phase2a): protocol v2 spec + README rate table

Released high rates: <fill from Task 7>. Final bss <fill>. Equivalence vs v1: PASS."
```

---

## Self-review notes (spec coverage)

- Spec §2 matrix (>250 = 12b RAW only) → Tasks 3 (STM guard), 5 (ESP32 guard + UI).
- Spec §4 v2 frame → Task 1 (contract) + Task 2 (STM instances).
- Spec §5 timestamp reconstruct → Task 3 (base capture) + Task 4 (ESP32 reconstruct). Q16 subsec + fs_code period table in Task 1.
- Spec §6 buffer tunable + overrun → Task 2 (ring + overrun) + Task 7 Step 4 (release gate).
- Spec §7 two-layer data flow → Task 3 Step 4 (TIM3 ISR bridges Layer 1→2; ADC DMA untouched).
- Spec §8 output/tooling → Task 5 (ESP32 CSV/RAW, container version bump) + Task 6 (ul_verify v2 parse/auto-detect, ul_soak, **convert_raw.py — the production offline RAW→CSV, the only CSV path >250 Hz**).
- Spec §9 version/lockstep → Task 1 (version) + Task 4 (handshake) + Task 7 (both-chip flash).
- Spec §10 verification (equivalence + new-rate + overrun-under-load gate) → Tasks 6 + 7.
- Spec §11 error/edge (partial frame, overrun re-anchor) → Task 2 (`frame_take_last` line_count; overrun flag).
- Spec §13 smells fixed → Task 2 (twin-struct + subsecond resolved).
- Spec §16 sequencing → Tasks 1–8 map 1:1.
