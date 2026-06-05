# Uberlogger STM32 Phase 1 Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure the STM32G030 firmware into auditable modules (`framing`, `acquisition`, `app`) and a shared cross-repo protocol header, with **zero change to the SPI wire bytes**, so the codebase is ready for the Phase-2 higher-sample-rate work.

**Architecture:** Incremental in-place refactor (spec Approach A). CubeMX-generated init stays in `main.c`'s `USER CODE` sections; application logic is extracted, one flash-and-bench-verified increment per task. The `framing` module quarantines all wire-layout knowledge; `_Static_assert`s pin the byte layout so any accidental change is a compile error.

**Tech Stack:** C (GNU18), STM32 HAL, Cortex-M0+ @ 64 MHz, STM32CubeIDE 2.1.1 headless builder, STM32CubeProgrammer CLI, ST-Link.

**Spec:** `docs/superpowers/specs/2026-06-05-uberlogger-stm32-refactor-design.md`

---

## Conventions & shared commands (read once)

All work happens on the current git branch (`refactor/phase1-spec`). One commit per task (or per sub-group where noted). Never push.

### BUILD (machine-verifiable, ~2 s incremental)

```bash
/opt/st/stm32cubeide_2.1.1/headless-build.sh \
  -data "$HOME/.stm32cubeide_ul_ws" \
  -import /home/paulus-potter/dev/uberlogger-stm32 \
  -build stm32g030c6/Debug 2>&1 | tail -8
```

- `-import` is idempotent (refreshes an already-imported project), so this command is safe to run every time.
- **Expected on success:** a `size` line and `Build Finished. 0 errors`.
- **Baseline size (untouched tree) — the reference for size-delta checks:**
  ```
  text 21056   data 296   bss 6960   dec 28312
  ```
  `bss` already uses 6960 of 8192 B SRAM. **Any task that changes `bss` by more than a few bytes is a red flag** (Phase 1 moves code, it does not allocate new buffers).

### FLASH (after a green build)

```bash
~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
  -c port=SWD -w /home/paulus-potter/dev/uberlogger-stm32/Debug/stm32g030c6.elf -rst
```

### BENCH MATRIX (human-in-the-loop — the behavior oracle)

Run against a real ESP32 + SD card. Defined **once here**; tasks reference it as "run the Bench Matrix" or a named subset.

| Axis | Values |
|---|---|
| Sample rate | 1 Hz, 25 Hz, 100 Hz, 250 Hz, **one sub-1 Hz averaging rate** (e.g. EVERY_10S) |
| Resolution | 12-bit, 16-bit |
| Log mode | CSV, RAW |
| Trigger | continuous, external-trigger, single-shot |

**Procedure:** For a representative sweep (at minimum: 250 Hz/16-bit/CSV/continuous, 100 Hz/12-bit/RAW/continuous, 1 Hz/12-bit/CSV/external-trigger, single-shot, and the sub-1 Hz averaging rate), log ~30 s, pull the file, and **diff against the Task-1 baseline capture** for the same settings. Pass = identical structure and values (timestamps/ADC/GPIO) within the same tolerance the device already exhibits.

- **Smoke subset** (fast, for low-risk tasks): 250 Hz/16-bit/CSV/continuous + 100 Hz/12-bit/CSV/continuous + single-shot.
- **Full matrix:** required after Task 5, 6, 7, 9.

### VERBATIM-MOVE CONVENTION

Several steps move existing functions/blocks **verbatim** to a new file. "Move verbatim from `main.c:218-336`" means cut those exact lines unchanged and paste them into the named location; only the explicitly listed edits (e.g. swapping a global for an `extern` or an accessor) may differ. For a behavior-preserving refactor this is *safer* than re-typing, and the build + static-asserts + bench matrix catch any slip. Line numbers are against the original files and will drift as you go — match on the function name/signature, not the absolute line.

---

## Task 1: Baseline capture + dead-code purge

**Files:**
- Modify: `Core/Src/main.c`, `Core/Src/config.c`, `Core/Src/adc_comp_lut.c`, `Core/Src/spi_ctrl.c`, `Core/Src/iirfilter.c` (comment removal only)

- [ ] **Step 1: Capture the behavior baseline on hardware**

Flash the **current** firmware (build + flash commands above) and run the **full Bench Matrix**, saving each output file as `baseline/<rate>-<res>-<mode>-<trigger>.csv|.raw` somewhere outside the repo (e.g. `~/ul_baseline/`). This is the oracle every later task is checked against. Do this before touching any code.

- [ ] **Step 2: Remove clearly-dead commented code**

Delete these superseded commented blocks (they are dead history, not reference):
- `main.c`: commented struct typedefs (`spi_msg_1_t`/`spi_msg_2_t`/`spi_msg_slow_freq_t` mock-ups, ~lines 122-160), commented `spi_msg_*_ptr` aliases (~165-167), commented `adc_comp_12b` loops inside the ADC callbacks, commented `is16bitmode` blocks in the state machine, commented `Send_OK`/`Send_NOK` (~1397-1432).
- `config.c`: the commented Julian-date conversion block in `Config_Set_Time` (~280-329), the commented `STM32_CMD_SET_ADC_CHANNELS_ENABLED` case (~114-127).
- `adc_comp_lut.c`: the commented "Calibration Bas" and "Calibration R02" LUT tables (~54-136); keep only the active "R04" tables.
- `iirfilter.c`: the commented 10-coefficient block and `x_state` (~99-109, 40).

- [ ] **Step 3: PRESERVE Phase-2 reference — do NOT delete**

Keep the commented higher-rate prescaler cases in `config.c` `Config_Set_Sample_freq` (500 Hz … 1 MHz). They are Phase-2 hints, not dead code. Consolidate them under a single clearly-labeled header comment: `/* --- Phase 2 reference: candidate prescaler/period values for >250 Hz (see refactor spec §11) --- */` so their intent is unambiguous.

- [ ] **Step 4: Build**

Run BUILD. Expected: `0 errors`; size **essentially unchanged** from baseline (comment removal does not change codegen — `text/data/bss` should match `21056/296/6960` exactly).

- [ ] **Step 5: Bench-verify (Smoke subset)**

Flash; run the **Smoke subset** of the Bench Matrix; confirm files match baseline. (Comment removal cannot change behavior, but this proves the toolchain loop end-to-end before real edits.)

- [ ] **Step 6: Commit**

```bash
git add Core/Src/main.c Core/Src/config.c Core/Src/adc_comp_lut.c Core/Src/spi_ctrl.c Core/Src/iirfilter.c
git commit -m "refactor(stm32): remove dead commented code; keep Phase-2 rate refs"
```

---

## Task 2: Pin the wire layout with `_Static_assert`

This is the safety net for every later task. Confirmed exact values (host-GCC, identical to ARM ABI for these `uint8/16/32` structs).

**Files:**
- Modify: `Core/Inc/main.h` (add asserts after the `spi_msg_2_t` definition)

- [ ] **Step 1: Add the layout assertions**

Append immediately after the `spi_msg_2_t` struct definition in `main.h` (inside `USER CODE Private defines`):

```c
/* --- Phase 1 wire-layout pins (refactor spec §8). DO NOT change these numbers.
 * If a build fails here, a struct layout changed = the SPI wire format changed. --- */
#include <stddef.h>
_Static_assert(sizeof(s_date_time_t) == 12, "s_date_time_t layout changed");

_Static_assert(sizeof(spi_msg_1_t) == 2048, "spi_msg_1_t size changed");
_Static_assert(offsetof(spi_msg_1_t, timeData) == 16,  "spi_msg_1_t.timeData moved");
_Static_assert(offsetof(spi_msg_1_t, gpioData) == 856, "spi_msg_1_t.gpioData moved");
_Static_assert(offsetof(spi_msg_1_t, padding1) == 926, "spi_msg_1_t.padding1 moved");
_Static_assert(offsetof(spi_msg_1_t, adcData)  == 928, "spi_msg_1_t.adcData moved");

_Static_assert(sizeof(spi_msg_2_t) == 2048, "spi_msg_2_t size changed");
_Static_assert(offsetof(spi_msg_2_t, adcData)  == 0,    "spi_msg_2_t.adcData moved");
_Static_assert(offsetof(spi_msg_2_t, gpioData) == 1122, "spi_msg_2_t.gpioData moved");
_Static_assert(offsetof(spi_msg_2_t, timeData) == 1192, "spi_msg_2_t.timeData moved");
_Static_assert(offsetof(spi_msg_2_t, padding0) == 2032, "spi_msg_2_t.padding0 moved");
_Static_assert(offsetof(spi_msg_2_t, dataLen)  == 2044, "spi_msg_2_t.dataLen moved");
_Static_assert(offsetof(spi_msg_2_t, stopByte) == 2046, "spi_msg_2_t.stopByte moved");
```

- [ ] **Step 2: Build**

Run BUILD. Expected: `0 errors` (all asserts hold against the current layout). Size unchanged (`_Static_assert` emits no code).

> If any assert fails: my computed value differs from this compiler. Read the failing assert's actual size from the error, **verify the struct definition is unchanged from `main`**, and only then update the literal to the compiler's value. A failure here on an *unmodified* struct means recompute, not "adjust to make it pass."

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/main.h
git commit -m "refactor(stm32): pin SPI wire layout with _Static_assert"
```

---

## Task 3: Tidy the pure-DSP modules (`iirfilter`, `adc_comp_lut`)

Low risk — no interface change, no call-site change. Pure readability.

**Files:**
- Modify: `Core/Src/iirfilter.c`, `Core/Inc/iirfilter.h`, `Core/Src/adc_comp_lut.c`, `Core/Inc/adc_comp_lut.h`

- [ ] **Step 1: Document, do not change, the two known smells**

In `iirfilter.c` above `iir_filter`, add a comment noting the error-feedback fixed-point design is intentional (cite the dsp.stackexchange link already present). Do **not** alter the math. In `adc_comp_lut.c` above `q_div`, add a comment: `/* 64-bit software divide: expensive on the divide-less M0+; runs per-sample. See spec §11 (Phase 2: apply LUT only to decimated samples). */`. No code change.

- [ ] **Step 2: Normalize headers**

Ensure every public function in `iirfilter.h` / `adc_comp_lut.h` has a one-line doc comment (purpose, units). Remove duplicate `#define NUM_ADC_CHANNELS` if it is defined in both the `.c` and `esp32_interface.h` — keep the single source in `esp32_interface.h`. Make no behavioral change.

- [ ] **Step 3: Build + smoke**

Run BUILD (expect `0 errors`, size delta ≈ 0). Flash + Smoke subset. Confirm match.

- [ ] **Step 4: Commit**

```bash
git add Core/Src/iirfilter.c Core/Inc/iirfilter.h Core/Src/adc_comp_lut.c Core/Inc/adc_comp_lut.h
git commit -m "refactor(stm32): document DSP modules; dedupe NUM_ADC_CHANNELS"
```

---

## Task 4: Tidy the SPI transport (`spi_ctrl`)

Low risk — remove genuinely unused API, no behavior change.

**Files:**
- Modify: `Core/Src/spi_ctrl.c`, `Core/Inc/spi_ctrl.h`

- [ ] **Step 1: Remove the unused `spi_ctrl_msg_sent`**

It is marked "don't use this function" and has no callers (verify with `grep -rn spi_ctrl_msg_sent Core ../uberlogger-esp32` → only its own def/decl). Delete the function from `spi_ctrl.c` and its prototype from `spi_ctrl.h`. Remove the dead commented blocks inside `spi_ctrl_loop` (the `SPI_CTRL_MSG_SENT` branch) and the commented `HAL_TIM_Base_Stop_IT` lines.

- [ ] **Step 2: Document the watchdog timers**

Add a header comment in `spi_ctrl.c` explaining TIM14 = TX timeout watchdog, TIM16 = RX timeout watchdog, and that `HAL_SPI_TxCpltCallback`/`RxCpltCallback` clear them. No behavior change.

- [ ] **Step 3: Build + smoke**

Run BUILD (expect `0 errors`; `text` may shrink slightly from removing the unused function — that is fine and expected; `bss` unchanged). Flash + Smoke subset incl. a config round-trip (change a setting from the web UI so a command/response transacts).

- [ ] **Step 4: Commit**

```bash
git add Core/Src/spi_ctrl.c Core/Inc/spi_ctrl.h
git commit -m "refactor(stm32): drop unused spi_ctrl_msg_sent; document watchdogs"
```

---

## Task 5: Extract `framing` (KEYSTONE — full Bench Matrix)

Move all SPI-buffer + layout + ping-pong knowledge out of `main.c` behind one interface. Highest-risk task: do it as small sub-commits and run the **full Bench Matrix** at the end.

**Files:**
- Create: `Core/Inc/framing.h`, `Core/Src/framing.c`
- Modify: `Core/Src/main.c` (call into framing), `Core/Inc/main.h` (move struct defs + asserts into framing.h), `Core/Src/config.c` (`spi_lines_per_transaction` setter)

- [ ] **Step 1: Create `framing.h` (the contract)**

```c
/* framing.h — owns the SPI message buffers, their exact byte layout, the
 * ping-pong/half bookkeeping, and the timestamp-coupling strategy.
 * Phase 1: byte-identical to the legacy twin-struct layout.
 * Phase 2: the ONLY place the wire layout changes (refactor spec §5.1, §11). */
#ifndef _FRAMING_H
#define _FRAMING_H

#include "stdint.h"
#include "main.h"            /* s_date_time_t, spi_msg_1_t, spi_msg_2_t live here for now */
#include "esp32_interface.h" /* adc_resolution_t, NUM_ADC_CHANNELS */

void     frame_init(void);                 /* set start/stop bytes, zero buffers (boot) */
void     frame_reset(void);                /* reset write ptrs / halves / ready (logging start) */
void     frame_set_lines_per_transaction(uint8_t n);  /* called by config on sample-rate change */
uint8_t  frame_lines_per_transaction(void);

/* Deposit one sample line. Called from the TIM3 sample-tick ISR.
 *  ts   : current timestamp snapshot
 *  gpio : GPIOB high-byte digital inputs
 *  adc  : NUM_ADC_CHANNELS corrected/filtered values (the iirFilter[] snapshot)
 *  res  : current resolution (selects the 12- vs 16-bit half bookkeeping)
 * Returns 1 when a half just became ready to transmit, else 0. */
uint8_t  frame_append_line(const s_date_time_t *ts, uint8_t gpio,
                           const uint16_t *adc, adc_resolution_t res);

/* If a filled half is pending, hand back its buffer + byte length and clear the flag. */
uint8_t  frame_take_ready(uint8_t **buf, uint16_t *len);

/* For STM32_CMD_SEND_LAST_ADC_BYTES / single-shot: hand back the half currently
 * being written (or msg_1 when singleshot). Mirrors the legacy selection exactly. */
void     frame_take_last(uint8_t **buf, uint16_t *len, uint8_t singleshot,
                         adc_resolution_t res);

#endif
```

- [ ] **Step 2: Create `framing.c` — move state + logic verbatim**

Move from `main.c` into `framing.c` as `static` module state: `data_buffer[]`, `spi_msg_slow_freq_1`, `spi_msg_slow_freq_2`, `gpio_result_write_ptr`, `gpio_is_half`, `adc_is_half`, `adc_16b_is_half`, `gpio_ready`, `adc_ready`, `spi_lines_per_transaction`. Implement:
- `frame_init()` ← the `memset` + `startByte`/`stopByte` init currently in `main()` (main.c ~505-514).
- `frame_reset()` ← the pointer/half/ready resets repeated in the state machine (main.c ~592-599, 687-694, 828-834).
- `frame_append_line()` ← the body of the TIM3 callback that writes gpio/time/adc and does the half toggle + ready logic, **moved verbatim** from `HAL_TIM_PeriodElapsedCallback` (main.c ~269-331), with the RTC read and `iirFilter` source passed in as `ts`/`adc` parameters instead of read globally. Preserve the `~` toggle on `adc_16b_is_half` and the 12-bit-vs-16-bit `memcpy` split **exactly** — document each with a `/* spec §7: preserved smell */` comment.
- `frame_take_ready()` / `frame_take_last()` ← the msg_1-vs-msg_2 selection from main.c LOGGING (~627-649) and SEND_LAST/single-shot (~743-766), moved verbatim.

- [ ] **Step 3: Move the struct defs + asserts into `framing.h`**

Cut `s_date_time_t`, `spi_msg_1_t`, `spi_msg_2_t`, the layout `#define`s they need, and the Task-2 `_Static_assert` block from `main.h` into `framing.h`. Have `main.h` `#include "framing.h"` is **wrong** (circular — framing.h includes main.h); instead leave `s_date_time_t`/struct defs in `main.h` and have `framing.h` rely on `main.h` (as written in Step 1). **Decision: keep struct defs in `main.h` for Phase 1** to avoid the include cycle; framing.c owns the *instances* and *logic*, main.h owns the *type*. (Phase 2 can introduce `ul_protocol.h` — Task 9 — as the type home.) Skip moving the defs; keep the asserts in `main.h`.

- [ ] **Step 4: Rewire `main.c` call sites**

- Replace the buffer-init in `main()` with `frame_init();`.
- In `HAL_TIM_PeriodElapsedCallback`, after reading RTC into `current_date_time` and computing the gpio byte, call `frame_append_line(&current_date_time, (GPIOB->IDR >> 8), iirFilter, adc_resolution)` and use its return as the `adc_ready/gpio_ready` trigger. (Until Task 6 moves this ISR, it stays in main.c but now delegates the buffer work.)
- Replace the LOGGING send-selection with `if (frame_take_ready(&buf,&len)) spi_ctrl_send(buf,len);`.
- Replace SEND_LAST/single-shot selection with `frame_take_last(&buf,&len,_singleshot,adc_resolution);`.
- Replace the inline pointer/half resets with `frame_reset();`.
- Remove the now-duplicated globals from `main.c` (they live in `framing.c`); add `#include "framing.h"`.

- [ ] **Step 5: `config.c` setter**

Replace `extern uint8_t spi_lines_per_transaction;` + direct writes in `Config_Set_Sample_freq` with `frame_set_lines_per_transaction(n);` (add `#include "framing.h"`). Each `spi_lines_per_transaction = N;` becomes `frame_set_lines_per_transaction(N);`.

- [ ] **Step 6: Build**

Run BUILD. Expected: `0 errors`, **all `_Static_assert`s still hold**, `bss` unchanged at 6960 (buffers moved, not added). If `bss` grew, a buffer got duplicated — fix before proceeding.

- [ ] **Step 7: Bench-verify (FULL Matrix)**

Flash; run the **full Bench Matrix**; diff every file against baseline. This is the critical gate. Any mismatch → revert this task's commits and re-extract more carefully.

- [ ] **Step 8: Commit**

```bash
git add Core/Inc/framing.h Core/Src/framing.c Core/Src/main.c Core/Inc/main.h Core/Src/config.c
git commit -m "refactor(stm32): extract framing module (wire layout behind one interface)"
```

---

## Task 6: Extract `acquisition` (ADC + TIM3 + sample-tick ISR)

Move the ADC/TIM3 sample path and its ISR bodies out of `main.c`. Defines the HAL weak callbacks itself.

**Files:**
- Create: `Core/Inc/acquisition.h`, `Core/Src/acquisition.c`
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Create `acquisition.h`**

```c
/* acquisition.h — owns the ADC + TIM3 sample path and the per-sample ISRs.
 * Phase 1: free-run continuous ADC + TIM3 decimation/timestamp tick (spec §2.1).
 * Phase 2 seam: free-run-vs-triggered and LUT/IIR placement live here (spec §5, §11). */
#ifndef _ACQUISITION_H
#define _ACQUISITION_H

#include "stdint.h"
#include "esp32_interface.h"

void acq_init(void);   /* calibrate ADC, start it (the boot-time Adc_start path) */
void acq_start(void);  /* (re)start ADC DMA per current resolution */
void acq_stop(void);   /* HAL_ADC_Stop_DMA */

#endif
```

- [ ] **Step 2: Move state + ISRs verbatim into `acquisition.c`**

Move as `static` state: `adc12Buffer[64]`, `adc16bBuffer[16]`, `iirFilter[8]`, `correctedAdc`, `current_date_time`, `current_time`, `current_date`, `busy`. Move **verbatim** the function bodies of `HAL_ADC_ConvHalfCpltCallback`, `HAL_ADC_ConvCpltCallback`, `HAL_ADC_ErrorCallback`, `HAL_TIM_PeriodElapsedCallback` (the TIM3 branch — keep the TIM14/TIM16 branches wherever they currently resolve; if they must stay with the SPI watchdogs, leave those two branches and move only the TIM3 branch into an `acq` helper the callback calls), and `Adc_start` (renamed `acq_start`). `acquisition.c` `#include`s `framing.h`, `iirfilter.h`, `adc_comp_lut.h`. The TIM3 ISR keeps calling `frame_append_line(...)` from Task 5. Expose `acq_init()` wrapping `HAL_ADCEx_Calibration_Start` + `acq_start` + `iir_init` ordering from `main()`.

> Note on HAL weak callbacks: `HAL_TIM_PeriodElapsedCallback` is a single weak symbol shared by TIM3/14/16. If splitting branches across files is awkward, keep the **one** callback in `acquisition.c` and have it call a small `spi_ctrl_on_timeout_tick(htim)` for the TIM14/TIM16 cases (declared in `spi_ctrl.h`). Choose whichever keeps each branch with its owning module; document the choice.

- [ ] **Step 3: Rewire `main.c`**

Remove the moved globals/functions; `#include "acquisition.h"`. Replace `Adc_start()` calls with `acq_start()`, the boot init block with `acq_init()`, and `HAL_ADC_Stop_DMA(&hadc1)` (the SETTINGS_MODE case) with `acq_stop()`. `hadc1`/`htim3` remain owned by CubeMX in `main.c` and are reached via `extern` from `acquisition.c` (declare them `extern` there, matching the existing pattern in `config.c`).

- [ ] **Step 4: Build**

Run BUILD. Expected: `0 errors`, asserts hold, `bss` unchanged (6960). Watch for duplicate-symbol errors on the HAL weak callbacks — there must be exactly one definition of each.

- [ ] **Step 5: Bench-verify (FULL Matrix)**

Flash; full Bench Matrix; diff vs baseline. The ISR timing path moved — verify especially 250 Hz/16-bit (heaviest ISR load) and single-shot.

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/acquisition.h Core/Src/acquisition.c Core/Src/main.c
git commit -m "refactor(stm32): extract acquisition module (ADC/TIM3 + sample ISRs)"
```

---

## Task 7: Extract `app` (state machine)

Move the `while(1)` state machine out of `main.c`.

**Files:**
- Create: `Core/Inc/app.h`, `Core/Src/app.c`
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Create `app.h`**

```c
/* app.h — the application state machine (IDLE/CONFIG/LOGGING/SINGLE_SHOT/
 * WAIT_FOR_TRIGGER), trigger debounce, and IDLE command dispatch. */
#ifndef _APP_H
#define _APP_H

void app_init(void);      /* one-time state init (called after MX_*_Init) */
void app_run_once(void);  /* one iteration of the former while(1) body */

#endif
```

- [ ] **Step 2: Move the state machine verbatim into `app.c`**

Move into `app.c`: the `MainState`/`NextState`, `logging_en`, `cmd_buffer`, `_singleshot`, `_trigger_mode`, debounce vars, `ext_trigger_input*`, and the entire `while(1)` body (main.c ~530-896) as `app_run_once()` (drop the `while(1)`/braces; one pass). `app.c` `#include`s `framing.h`, `acquisition.h`, `spi_ctrl.h`, `config.h`. The EXTI callbacks (`HAL_GPIO_EXTI_Rising/Falling_Callback`) move here too (they set `logging_en`). `app_init()` holds any pre-loop state setup. Keep `HAL_Delay(50)` and all ordering verbatim.

- [ ] **Step 3: Reduce `main.c` to a thin shell**

`main()` becomes: `HAL_Init(); SystemClock_Config(); MX_*_Init(); acq_init(); app_init(); while(1) app_run_once();`. The `MX_*_Init`, `SystemClock_Config`, `Error_Handler`, and the `GPIOB->OSPEEDR` MISO drive-strength line stay in `main.c`. Add `#include "app.h"`. Verify no application globals remain in `main.c` except the CubeMX HAL handles (`hadc1`, `hspi1`, `htim3/14/16`, `hrtc`, DMA handles).

- [ ] **Step 4: Build**

Run BUILD. Expected `0 errors`, asserts hold, `bss` unchanged. `main.c` should now be ~250 lines.

- [ ] **Step 5: Bench-verify (FULL Matrix)**

Flash; full Bench Matrix incl. external-trigger and single-shot (this task owns that logic); diff vs baseline.

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/app.h Core/Src/app.c Core/Src/main.c
git commit -m "refactor(stm32): extract app state machine; main.c is now a thin shell"
```

---

## Task 8: Consolidate globals & trim `extern`s

Cleanup pass — no new behavior. Reduce cross-file `extern` sprawl now that ownership is clear.

**Files:**
- Modify: `Core/Src/*.c`, `Core/Inc/*.h` as needed

- [ ] **Step 1: Audit externs**

`grep -rn "^extern\|	extern\| extern " Core/Src Core/Inc`. For each cross-module global, confirm it now lives `static` in exactly one owning module with an accessor, **except** the CubeMX HAL handles (`hadc1`, `hspi1`, `htim3/14/16`, `hrtc`, `hdma_*`) which remain `extern`-shared by design.

- [ ] **Step 2: Replace stray externs with accessors**

For any remaining app-level `extern` (e.g. `adc_resolution`, `adc_voltage_range_g`, `logMode`, `_trigger_mode`, `ext_trigger_input`, `_debounce_time_ext_input`) used across modules, give it a single owner (likely `config.c` for settings, `app.c` for runtime state) and a small getter/setter; replace the `extern` declarations with the header include. Make no logic change.

- [ ] **Step 3: Build + FULL Matrix subset**

Run BUILD (`0 errors`, asserts hold, size delta ≈ 0). Flash; run Smoke subset + one external-trigger run + one config round-trip.

- [ ] **Step 4: Commit**

```bash
git add -A Core
git commit -m "refactor(stm32): consolidate globals into owning modules; trim externs"
```

---

## Task 9: Canonical shared `ul_protocol.h` (byte-identical) + protocol spec

Create the cross-repo single source of truth (spec §5.1). **Byte-identical** — pure de-duplication.

**Files:**
- Create: `Core/Inc/ul_protocol.h` (STM copy), `../uberlogger-esp32/main/ul_protocol.h` (vendored copy, identical), `docs/protocol/uberlogger-spi-protocol.md`
- Modify: STM `esp32_interface.h`, `main.h`; ESP32 `spi_control.h`, `logger.c` (include the header instead of redefining)

- [ ] **Step 1: Author `ul_protocol.h`**

Put the shared contract in one header: `UL_PROTOCOL_VERSION` (e.g. `1`), the `STM32_CMD_*` command enum, `spi_cmd_t` (8 bytes), the response enums, `s_date_time_t`, the layout `#define`s (`DATA_LINES_PER_SPI_TRANSACTION`, `ADC_*_PER_SPI_TRANSACTION`, `START_STOP_NUM_BYTES`), and `spi_msg_1_t`/`spi_msg_2_t`. Copy the **exact** current definitions (names, order, padding) from the STM headers so nothing on the wire changes. Add a `_Static_assert(UL_PROTOCOL_VERSION == 1, ...)` placeholder and the Task-2 layout asserts (now living in this shared header).

- [ ] **Step 2: Vendor into both repos with a version guard**

Copy the identical file to both `Core/Inc/ul_protocol.h` and `../uberlogger-esp32/main/ul_protocol.h`. In each repo add, at a single build-visible point, `_Static_assert(UL_PROTOCOL_VERSION == 1, "ul_protocol.h copies are out of sync");` so a future mismatch is caught at compile.

- [ ] **Step 3: Replace duplicated defs with the include**

STM: `esp32_interface.h` and `main.h` drop the now-shared definitions and `#include "ul_protocol.h"`. ESP32: `spi_control.h` and `logger.c` drop their hand-copied `spi_cmd_t`/`stm32cmd_t`/struct defs and `#include "ul_protocol.h"`. Keep enum value names compatible (the ESP32 used `stm32cmd_t`/`STM32_RESP_OK` — add matching aliases in `ul_protocol.h` if needed so neither repo's call sites change).

- [ ] **Step 4: Build BOTH repos**

STM: run BUILD (`0 errors`, layout asserts hold, `bss` unchanged). ESP32: build via its normal `idf.py build` / `build.sh`. Both must compile with the version guard satisfied.

- [ ] **Step 5: Bench-verify (FULL Matrix)**

Flash STM (and ESP32 if its build changed). Run the **full Bench Matrix** — both firmwares now consume the shared header; confirm the wire is unchanged.

- [ ] **Step 6: Write the protocol spec doc**

`docs/protocol/uberlogger-spi-protocol.md`: document the DATA_RDY handshake (command path vs streaming path), SPI mode/clock (slave, mode 0, MSB, 8-bit, 10 MHz), the `spi_cmd_t` command set + responses, the `spi_msg_1/2` byte layout (with the offset table from Task 2), and the watchdog/timeout behavior. State that `ul_protocol.h` is the machine-readable source of truth and this doc is its prose companion.

- [ ] **Step 7: Commit (both repos)**

```bash
# STM repo
git add Core/Inc/ul_protocol.h Core/Inc/esp32_interface.h Core/Inc/main.h docs/protocol/uberlogger-spi-protocol.md
git commit -m "refactor: introduce shared ul_protocol.h (byte-identical) + protocol spec"
# ESP32 repo (separate commit, separate repo)
cd ../uberlogger-esp32 && git add main/ul_protocol.h main/spi_control.h main/logger.c \
  && git commit -m "refactor: consume shared ul_protocol.h (byte-identical)"
```

---

## Task 10: Final pass + README dev notes

**Files:**
- Modify: `README.md`, any stragglers

- [ ] **Step 1: Update README**

Fix the stale "16 MHz internal oscillator" (actually 64 MHz HSE+PLL). Add a "Module map" section (`main` = boot/CubeMX, `app` = state machine, `acquisition` = ADC/TIM3/ISRs, `framing` = wire buffers/layout, `spi_ctrl` = transport, `config` = settings, `ul_protocol.h` = shared contract). Document the headless build command and the bench-matrix verification approach.

- [ ] **Step 2: Final full build + FULL Matrix**

Run BUILD (`0 errors`, all asserts hold, final size recorded). Flash; run the **complete Bench Matrix** one last time; diff vs baseline. Record the final `size` line in the commit message.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs(stm32): module map, correct clock, headless build + bench verification"
```

---

## Self-review notes (spec coverage)

- Spec §2 (as-built understanding) → documented in Tasks 3, 4, 6 comments + Task 9 protocol doc.
- Spec §3 constraints C1–C5 → C1 (wire frozen) enforced by Task 2 asserts + bench diffs; C3 (CubeMX-safe) by keeping init in `main.c`; C5 (HW verify) by the Bench Matrix gates.
- Spec §5 module structure → Tasks 5/6/7 (framing/acquisition/app); §5.1 shared header → Task 9.
- Spec §6 data flow preserved → verbatim-move convention + full-matrix gates after Tasks 5/6/7.
- Spec §7 known smells documented-not-fixed → Tasks 3 & 5 comments.
- Spec §8 static net → Task 2.
- Spec §9 verification → the BUILD + BENCH MATRIX primitives, used every task.
- Spec §10 sequencing → Tasks 1–10 map 1:1.
- Spec §11/§12/§13 (Phase 2, scope) → out of scope here; Phase-2 reference hints preserved (Task 1 Step 3) and the shared-header seam created (Task 9).
