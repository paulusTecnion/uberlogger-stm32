# STM32→ESP32 Out-of-Band Fault Line — Design

**Date:** 2026-06-07
**Status:** Approved (design); pending implementation plan
**Repos:** `uberlogger-stm32` (fault detection + drive) and `uberlogger-esp32` (ISR + reaction)
**Relation:** Extends Phase 2A (`2026-06-06-uberlogger-phase2a-protocol-timestamp`). No wire-format change; `UL_PROTOCOL_VERSION` stays 2.

## Problem

At 500/1000 Hz under concurrent web/API load the SPI handoff can tear. Observed in the
1000 Hz release soak: the ESP latched `ERR_LOGGER_STM32_FAULTY_DATA` (a `decodeV2Frame`
marker/count rejection) with `OVERRUN=0` — i.e. the STM ring did **not** overrun; a frame
was torn in transit and the session hard-aborted. 500 Hz passed clean.

Root mechanism: the STM raises `STM_DATA_RDY` when a frame's TX DMA is armed and drops it
on TX-complete, but a TX-timeout (TIM14) aborts the DMA mid-transfer if the ESP doesn't
clock it out in time. Under 4-worker API load the ESP logger task is delayed servicing the
SPI read → the STM aborts → the ESP reads a partial frame.

Current overrun signaling is **in-band** (a `UL_FLAG_OVERRUN` header bit per frame + an
end-of-session `STM32_CMD_GET_OVERRUN` SPI query). Both ride the SPI data path — the very
path that saturates under load — so the signal is least trustworthy exactly when overrun
matters most.

## Goal

A dedicated **out-of-band STM→ESP "fault" line** that asserts on **ring overrun OR SPI
TX-timeout/tear**, letting the ESP recover gracefully (discard torn frame → resync, or
finalize cleanly on overrun) instead of hard-aborting with `FAULTY_DATA`. Robust even when
the SPI link is saturated.

Non-goal: increasing raw 1000 Hz throughput. This makes failures clean, detectable, and
recoverable; it does not by itself guarantee 1000 Hz is a sustainable continuous rate.

## Hardware: the line

**Net `STM_USART_RX` = STM `PA10` ↔ ESP `IO4`.** Connectivity is proven (firmware flashing
already uses this UART). The line has two mutually-exclusive modes:

| Mode | STM PA10 | ESP IO4 | Line semantics |
|---|---|---|---|
| **Logging** | GPIO push-pull **output, idle LOW** | **input, pull-down, rising-edge ISR** | HIGH = fault asserted |
| **Firmware update** | ROM bootloader USART1 RX | UART TX (`uart_set_pin`) | UART (no fault use) |

Why RX not TX: `STM_USART_TX`/PA9 is also `EXT_PIN_VALUE`, driven every app loop to forward
the external-trigger state — occupied during logging. PA10 has no app function (USART is
ROM-bootloader-only; `.ioc` never configures it) and ESP IO4 is untouched during logging.

**Mode handoff (clean by construction):** Firmware update is always preceded by an STM reset
(BOOT0/NRESET), so PA10 powers up as bootloader-RX automatically; the STM app only switches
PA10→output once it is running in logging. The ESP reclaims IO4→TX on entering flash mode
(its existing `uart_set_pin` call) and returns it to input+ISR when logging resumes. Idle-low
drive + ESP pull-down means a reset or dead STM reads LOW → no false fault.

## STM firmware — detect & drive

A small fault-line helper in the STM (e.g. `fault_line.{c,h}` or folded into `spi_ctrl`):

- `fault_line_init_output(void)` — configure PA10 as push-pull output, drive LOW. Called at
  logging init.
- `fault_line_assert(void)` — drive PA10 HIGH. Latched (stays HIGH until cleared).
- `fault_line_clear(void)` — drive PA10 LOW. Called from **two** sites:
  1. `spi_ctrl.c` `HAL_SPI_TxCpltCallback` (every successful frame TX, alongside the existing
     `STM_DATA_RDY` reset), so each tear-after-a-good-frame is a **fresh rising edge** the ESP
     edge-ISR can see.
  2. `app.c` at **every** session-start site — co-located with each `frame_reset()` that begins
     acquisition (WAIT_FOR_TRIGGER→LOGGING, the IDLE→LOGGING continuous path, and SINGLE_SHOT) —
     to drop any latched fault from the prior session. Invariant: a new session always starts
     with the line LOW. (Not the read-only `STM32_CMD_GET_OVERRUN` handler, which only reads the flag.)

**Edge semantics (why clear-on-TX-complete):** the ESP IO4 ISR is rising-edge-triggered. If
the line merely latched until session reset, only the *first* tear would produce an edge and
every later resync would be missed. Clearing on each good frame's TX-complete means: normal
operation holds the line LOW; a tear pulses it HIGH until the next good frame; an **overrun
latches HIGH naturally** because the STM stops sending (no further TX-complete to clear it),
so the ESP still gets that edge. Consecutive tears with no good frame between them coalesce to
one edge — acceptable, since a link that can't land a single good frame is a dead link the
ESP's no-data/timeout path already covers. `GET_OVERRUN` (sticky on the STM until
`frame_reset`) remains the authoritative disambiguator regardless of the line's level.

**Assert sites:**
1. **Ring overrun:** in `app.c` `MAIN_LOGGING`, immediately before the overrun gate
   (`app.c:172`), guarded by `frame_overrun()`. Kept in `app.c` (not inside `framing.c`'s
   `frame_append_line`) so `framing.c` carries no dependency on the fault-line module.
2. **SPI tear:** in `spi_ctrl.c` `SPI_CTRL_TX_TIMEOUT` handling, where the DMA is aborted
   mid-transfer (`HAL_SPI_DMAStop`).

No explicit revert for flashing is needed — the pre-flash STM reset reverts PA10 to
bootloader RX.

## ESP firmware — react & disambiguate

IO4 rising-edge ISR (mirrors the existing `gpio_handshake_isr_handler`): set a volatile
`fault_pending` flag and `vTaskNotifyGiveFromISR` the logger task.

On `fault_pending`, the logger task:
1. Aborts/ignores the in-flight SPI read and discards any partial frame.
2. Queries `STM32_CMD_GET_OVERRUN` (up to **3 attempts**, since the query is itself a SPI
   transaction that may tear under the same load):
   - **overrun == 1** → STM stopped on ring overrun → **finalize the session cleanly**, set
     `ERR_LOGGER_DATA_OVERRUN`, expose `OVERRUN=1`.
   - **overrun == 0** → SPI tear → **discard & resync** (wait for next `DATA_RDY`), continue
     logging, increment a resync counter.
   - **query unreadable after retries** → treat as tear (discard & resync).

`decodeV2Frame`'s marker/count validation remains as a backstop: a torn frame that slips
through is still caught, but the fault path now lets the ESP *expect* the garbage and recover
rather than latch `FAULTY_DATA`.

The ESP must manage IO4 mode: configure input+ISR at logging start, remove the ISR and hand
IO4 back to the UART driver when entering firmware-update, and restore input+ISR afterward.

## Error semantics

- **Recovered tear:** does **not** set `ERR_LOGGER_STM32_FAULTY_DATA`. Increments
  `RESYNC_COUNT` (new field in `/ajax/getStatus`) for soak visibility.
- **Ring overrun:** `ERR_LOGGER_DATA_OVERRUN` + `OVERRUN=1`, clean finalize — same outcome as
  today, but now reliably triggered out-of-band.
- **Tear storm:** if resyncs exceed **10 within any rolling 1-second window**, escalate to a
  hard error (`ERR_LOGGER_STM32_FAULTY_DATA`) + finalize, so a genuinely unusable link fails
  loudly rather than resyncing forever. (Default; tune at the HW gate.)
- **`/ajax/getStatus`** gains `RESYNC_COUNT` (cumulative resync events for the session).
- **No wire-format change.** `UL_PROTOCOL_VERSION` stays 2.

**Firmware compatibility (graceful degradation, no version gate):**
- Old STM + new ESP: STM never drives PA10 → ESP sees IO4 low → no fault edge → falls back to
  today's `FAULTY_DATA`/`OVERRUN` behavior.
- New STM + old ESP: STM drives PA10 → ESP ignores IO4 → harmless.

## Testing / validation (hardware gate)

1. **1000 Hz soak (rate 14)** under 4-worker API load → tears become graceful resyncs (no
   `ERR 8` abort); session completes; `RESYNC_COUNT` reflects tears; `OVERRUN=0`.
2. **Forced ring overrun** (starve the ESP) → `OVERRUN=1`, clean finalize,
   `ERR_LOGGER_DATA_OVERRUN`, **no** `FAULTY_DATA`.
3. **500 Hz regression** (rate 13) → still PASS (zero overrun, no dropped frames, monotonic).
4. **Firmware-update regression** — flash the STM via the ESP after the change, proving the
   PA10/IO4 direction handoff did not break the bootloader UART. *(Riskiest interaction —
   explicitly gated.)*

Each end's pin-mode transitions (logging ↔ firmware-update) are the primary risk; test 4 is
the gate for that.

## Scope

Cross-repo (STM + ESP), one focused feature, single implementation plan. Confirm the
prerequisites with the user before flashing either chip.
