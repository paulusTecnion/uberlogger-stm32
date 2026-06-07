# STM32→ESP32 Out-of-Band Fault Line Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a dedicated STM→ESP "fault" GPIO (STM_USART_RX, PA10↔IO4) so the STM signals ring-overrun OR SPI tear out-of-band, letting the ESP recover gracefully (discard+resync, or clean overrun finalize) instead of hard-aborting with `FAULTY_DATA`.

**Architecture:** During logging the STM drives PA10 push-pull (idle LOW, HIGH=fault); the ESP reads IO4 via an edge ISR. On a fault edge the ESP discards any partial frame and queries the existing `STM32_CMD_GET_OVERRUN` (up to 3 tries) to disambiguate: overrun→finalize, tear→resync. The line reverts to its bootloader UART role during firmware update (a mode always preceded by an STM reset). No wire-format change; `UL_PROTOCOL_VERSION` stays 2.

**Tech Stack:** STM32G030C6 C/HAL (STM32CubeIDE headless build); ESP32-S2 ESP-IDF v5.2 (`idf.py`); two repos: `uberlogger-stm32`, `uberlogger-esp32`.

**Spec:** `uberlogger-stm32/docs/superpowers/specs/2026-06-07-stm-esp-fault-line-design.md`

---

## Testing strategy (read first)

This is firmware with **no unit-test harness** on either side (the STM has only python *bench* fixtures; the ESP has none). The established Phase 1/2A discipline — which this plan follows — is:

- **Per code task:** the gate is a **clean build**. Exact build command + expected output are given in each task. A task is not done until it compiles with zero errors and is committed.
- **Behavioral validation:** the **integrated hardware gate (Task 7)** — flash both chips and run the soak/overrun/regression/firmware-update checks. This is where the design's behavior is actually proven.

Do not invent a unit-test framework. The build is the fast feedback loop; the HW gate is the behavioral proof.

**Build commands (memorize):**

- **STM:** `/opt/st/stm32cubeide_2.1.1/headless-build.sh -data "$HOME/.stm32cubeide_ul_ws" -import /home/paulus-potter/dev/uberlogger-stm32 -build stm32g030c6/Debug`
  Expected tail: `Build Finished. 0 errors, ...`. New `.c` files under `Core/Src/` are auto-globbed by the managed build — no project-file edit needed.
- **ESP:** `source /home/paulus-potter/esp-idf/export.sh && cd /home/paulus-potter/dev/uberlogger-esp32 && idf.py build`
  Expected tail: `Project build complete.`

**Branches:** STM work on `feature/phase2a-protocol-timestamp` (current). ESP work on its current branch (`feature/phase2a-protocol-timestamp`). Commit each task in the repo it touches.

---

## File Structure

**uberlogger-stm32 (STM firmware):**
- `Core/Inc/fault_line.h` *(create)* — fault-line API + PA10 pin macros. One responsibility: own the fault GPIO.
- `Core/Src/fault_line.c` *(create)* — drive PA10 (init-output / assert / clear).
- `Core/Src/app.c` *(modify)* — init the line at startup; clear at session start; assert on `frame_overrun()`.
- `Core/Src/spi_ctrl.c` *(modify)* — assert on SPI TX-timeout/abort.

**uberlogger-esp32 (ESP firmware):**
- `main/config.h` *(modify)* — `GPIO_STM32_FAULT` pin define.
- `main/spi_control.c` / `main/spi_control.h` *(modify)* — fault edge-ISR, pending flag, enable/disable.
- `main/logger.c` / `main/logger.h` *(modify)* — fault reaction (query+disambiguate+resync/finalize), resync counters, retrying overrun query, enable/disable the fault ISR at logging start/stop.
- `main/rest_server.c` *(modify)* — `RESYNC_COUNT` in `/ajax/getStatus`.
- `main/firmwareSTM32.c` *(modify)* — release the fault ISR + free the UART driver around flashing (pin-direction handoff).

**tooling:**
- `uberlogger-stm32/tools/bench/ul_soak.py` *(modify, Task 7)* — surface `RESYNC_COUNT`.

---

## Task 1: STM fault-line module (drive PA10)

**Files:**
- Create: `/home/paulus-potter/dev/uberlogger-stm32/Core/Inc/fault_line.h`
- Create: `/home/paulus-potter/dev/uberlogger-stm32/Core/Src/fault_line.c`

- [ ] **Step 1: Create the header**

`Core/Inc/fault_line.h`:
```c
/*
 * Uberlogger Firmware — out-of-band STM->ESP fault line.
 * Copyright (c) 2025 Tecnion Technologies. MIT License.
 */
#ifndef _FAULT_LINE_H
#define _FAULT_LINE_H

#include "main.h"

/* STM_USART_RX net = PA10 <-> ESP IO4. Repurposed STM->ESP fault line during
 * logging: driven push-pull, idle LOW, HIGH = fault (ring overrun or SPI tear).
 * Reverts to the ROM bootloader's USART1_RX automatically on the pre-flash STM
 * reset, so no explicit revert is needed here. PA10 has no other app function. */
#define FAULT_LINE_Pin        GPIO_PIN_10
#define FAULT_LINE_GPIO_Port  GPIOA

/* Configure PA10 as push-pull output, idle LOW. Call once at app init (GPIOA
 * clock is already enabled by MX_GPIO_Init). */
void fault_line_init_output(void);

/* Drive the line HIGH (latched until fault_line_clear). */
void fault_line_assert(void);

/* Drive the line LOW. Called at session (re)start. */
void fault_line_clear(void);

#endif /* _FAULT_LINE_H */
```

- [ ] **Step 2: Create the implementation**

`Core/Src/fault_line.c`:
```c
/*
 * Uberlogger Firmware — out-of-band STM->ESP fault line.
 * Copyright (c) 2025 Tecnion Technologies. MIT License.
 */
#include "fault_line.h"

void fault_line_init_output(void)
{
    GPIO_InitTypeDef g = {0};
    /* GPIOA clock is enabled in MX_GPIO_Init() before app_init(). */
    HAL_GPIO_WritePin(FAULT_LINE_GPIO_Port, FAULT_LINE_Pin, GPIO_PIN_RESET);
    g.Pin   = FAULT_LINE_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAULT_LINE_GPIO_Port, &g);
}

void fault_line_assert(void)
{
    HAL_GPIO_WritePin(FAULT_LINE_GPIO_Port, FAULT_LINE_Pin, GPIO_PIN_SET);
}

void fault_line_clear(void)
{
    HAL_GPIO_WritePin(FAULT_LINE_GPIO_Port, FAULT_LINE_Pin, GPIO_PIN_RESET);
}
```

- [ ] **Step 3: Build (the gate)**

Run:
```bash
/opt/st/stm32cubeide_2.1.1/headless-build.sh -data "$HOME/.stm32cubeide_ul_ws" -import /home/paulus-potter/dev/uberlogger-stm32 -build stm32g030c6/Debug 2>&1 | tail -5
```
Expected: `Build Finished. 0 errors,` (the new module compiles even though nothing calls it yet).

- [ ] **Step 4: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-stm32
git add Core/Inc/fault_line.h Core/Src/fault_line.c
git commit -m "feat(stm): add fault_line module (drive PA10 out-of-band fault line)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Wire the STM fault line into app + spi_ctrl

**Files:**
- Modify: `/home/paulus-potter/dev/uberlogger-stm32/Core/Src/app.c` (include; `app_init` ~line 88; session-start ~line 143; overrun gate ~line 172)
- Modify: `/home/paulus-potter/dev/uberlogger-stm32/Core/Src/spi_ctrl.c` (include; TX-timeout handler ~line 196)

- [ ] **Step 1: Include the header in app.c**

In `Core/Src/app.c`, add to the include block (after `#include "spi_ctrl.h"` at line 15):
```c
#include "fault_line.h"
```

- [ ] **Step 2: Initialize the line at app init**

In `Core/Src/app.c`, in `app_init(void)` (line 88), add the init call. The function currently only documents that state relies on static init; add:
```c
void app_init(void)
{
	/* ...existing comment... */
	fault_line_init_output();   /* PA10 push-pull output, idle LOW */
}
```
(Keep any existing body; just add the `fault_line_init_output();` line.)

- [ ] **Step 3: Clear the line at session start**

In `Core/Src/app.c`, at the `MAIN_WAIT_FOR_TRIGGER`→`MAIN_LOGGING` transition, next to the existing `frame_reset();` (line 143), add `fault_line_clear();`:
```c
		  			  tim3_counter = 0;
					  frame_reset();
					  fault_line_clear();   /* drop any latched fault from prior session */
					  time_result_write_ptr = 0;
```

- [ ] **Step 4: Assert the line on ring overrun**

In `Core/Src/app.c`, in `case MAIN_LOGGING:`, immediately BEFORE the overrun-gate `if` at line 172, add an explicit overrun→assert (so we only assert on the overrun cause, not on normal stop or external-trigger stop):
```c
		  	  if (frame_overrun())
		  	  {
		  		  fault_line_assert();   /* tell the ESP out-of-band before we stop */
		  	  }
		  	  if ((!logging_en || frame_overrun()) || (ext_trigger_input_value == 0 && config_trigger_mode() == TRIGGER_MODE_EXTERNAL))
			  {
```

- [ ] **Step 5: Assert the line on SPI TX-timeout in spi_ctrl.c**

In `Core/Src/spi_ctrl.c`, add the include near the top (after the existing includes):
```c
#include "fault_line.h"
```
Then in `spi_ctrl_loop()`, in the `case SPI_CTRL_SENDING:` TX-timeout branch, after `HAL_SPI_DMAStop(&hspi1);` and the `STM_DATA_RDY` reset (line ~198), add the assert:
```c
				HAL_SPI_DMAStop(&hspi1);
				// Not necessary for receiving, but no harm in making data_rdy low
				HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, RESET);
				fault_line_assert();   /* TX aborted mid-transfer -> ESP frame is torn */
```

- [ ] **Step 6: Clear the line on each successful TX-complete**

This is what makes repeated tears each produce a fresh rising edge for the ESP's edge ISR
(without it, the line would latch after the first tear and every later resync would be
missed). In `Core/Src/spi_ctrl.c`, in `HAL_SPI_TxCpltCallback` (line ~77), alongside the
existing `STM_DATA_RDY` reset at line 81, add the clear:
```c
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// Clear timeout interrupt
	CLEAR_BIT(TIM14->DIER, TIM_DIER_UIE);
	HAL_GPIO_WritePin(STM_DATA_RDY_GPIO_Port, STM_DATA_RDY_Pin, RESET);
	fault_line_clear();   /* good frame sent -> drop the fault line; next tear = new edge */
	CLEAR_BIT(spi_ctrl_state, SPI_CTRL_SENDING);
	...
}
```
A ring overrun still latches HIGH correctly: on overrun the STM stops sending, so no further
TX-complete fires to clear it, and the ESP still sees that edge.

- [ ] **Step 7: Build (the gate)**

Run:
```bash
/opt/st/stm32cubeide_2.1.1/headless-build.sh -data "$HOME/.stm32cubeide_ul_ws" -import /home/paulus-potter/dev/uberlogger-stm32 -build stm32g030c6/Debug 2>&1 | tail -5
```
Expected: `Build Finished. 0 errors,`.

- [ ] **Step 8: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-stm32
git add Core/Src/app.c Core/Src/spi_ctrl.c
git commit -m "feat(stm): assert fault line on ring-overrun and SPI TX-timeout

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: ESP fault GPIO ISR + pending flag

**Files:**
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/config.h` (~line 67-74)
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/spi_control.c` (ISR + enable/disable + flag accessor)
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/spi_control.h` (prototypes)

- [ ] **Step 1: Define the fault pin in config.h**

In `main/config.h`, after the UART defines (line 74 `#define GPIO_STM32_UART_RX  4`), add:
```c
// STM fault line (= GPIO_STM32_UART_RX / IO4, net STM_USART_RX). During logging
// the STM drives this HIGH on ring-overrun or SPI tear; the ESP reads it as an
// edge-ISR input. Time-shared with the bootloader UART RX during firmware update.
#define GPIO_STM32_FAULT 4
```

- [ ] **Step 2: Add the ISR, enable/disable, and flag accessor in spi_control.c**

In `main/spi_control.c`, after `gpio_handshake_isr_handler` (ends ~line 90), add:
```c
/* Out-of-band STM fault line (IO4). Set by an edge ISR; consumed by the logger
 * task. The same task notification wakes the logger as the data-ready handshake,
 * so the task distinguishes the two by checking spi_ctrl_fault_pending() first. */
static volatile uint8_t s_fault_pending = 0;

static void IRAM_ATTR gpio_fault_isr_handler(void* arg)
{
    BaseType_t xYieldRequired = pdFALSE;
    s_fault_pending = 1;
    vTaskNotifyGiveFromISR(xHandle_stm32, &xYieldRequired);
    portYIELD_FROM_ISR(xYieldRequired);
}

esp_err_t spi_ctrl_fault_int(uint8_t enable)
{
    if (enable == 1)
    {
        gpio_config_t io_conf = {
            .intr_type    = GPIO_INTR_POSEDGE,
            .mode         = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << GPIO_STM32_FAULT),
            .pull_down_en = 1,   // idle LOW when STM not driving / in reset
            .pull_up_en   = 0,
        };
        gpio_config(&io_conf);
        s_fault_pending = 0;
        if (gpio_isr_handler_add(GPIO_STM32_FAULT, gpio_fault_isr_handler, NULL) != ESP_OK)
        {
            ESP_LOGE(TAG_SPI_CTRL, "Unable to add fault ISR handler");
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    else
    {
        gpio_isr_handler_remove(GPIO_STM32_FAULT);
        s_fault_pending = 0;
        return ESP_OK;
    }
}

uint8_t spi_ctrl_fault_pending(void)
{
    uint8_t v = s_fault_pending;
    s_fault_pending = 0;
    return v;
}
```
(`GPIO_STM32_FAULT` resolves via `config.h`, already included by `spi_control.c`. The global GPIO ISR service is installed in `spi_ctrl_init()` at line 169, so `gpio_isr_handler_add` works.)

- [ ] **Step 3: Declare the new functions in spi_control.h**

In `main/spi_control.h`, after the existing `spi_ctrl_datardy_int` declaration, add:
```c
/* Enable (1) / disable (0) the IO4 STM-fault edge ISR. */
esp_err_t spi_ctrl_fault_int(uint8_t enable);

/* Read-and-clear the fault-pending flag (set by the IO4 edge ISR). */
uint8_t spi_ctrl_fault_pending(void);
```

- [ ] **Step 4: Build (the gate)**

Run:
```bash
source /home/paulus-potter/esp-idf/export.sh && cd /home/paulus-potter/dev/uberlogger-esp32 && idf.py build 2>&1 | tail -5
```
Expected: `Project build complete.` (functions compile though unused so far).

- [ ] **Step 5: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-esp32
git add main/config.h main/spi_control.c main/spi_control.h
git commit -m "feat(esp): add IO4 STM-fault edge ISR + pending flag

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: ESP logger fault reaction (query, disambiguate, resync/finalize)

**Files:**
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/logger.c` (retrying overrun query; resync state; fault handling in `Logtask_logging` `LOGTASK_LOGGING_BUSY` ~line 2368; enable/disable fault ISR at logging start/stop)
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/logger.h` (`Logger_getResyncCount` prototype)

- [ ] **Step 1: Add includes + resync state in logger.c**

In `main/logger.c`, ensure `#include "esp_timer.h"` is present (add near the other ESP includes if missing). Near `static uint8_t _stmOverrun = 0;` (line 111), add:
```c
static uint32_t _resyncCount = 0;          /* cumulative SPI-tear resyncs this session */
static uint32_t _resyncWindowStart = 0;    /* ms, start of the current 1s tear-storm window */
static uint32_t _resyncWindowCount = 0;    /* resyncs within the current window */
```

- [ ] **Step 2: Make the overrun query return a status (for retrying)**

In `main/logger.c`, change `Logger_queryOverrun` (line ~966) from `static void` to `static int` returning 0 on a clean read (echo matched, `_stmOverrun` updated) and -1 on failure. Edit the signature and the two early returns and the end:
```c
static int Logger_queryOverrun(void)
{
    spi_cmd_t cmd;
    spi_buffer = spi_ctrl_getRxData();
    cmd.command = STM32_CMD_GET_OVERRUN;
    cmd.data0   = 0;
    if (spi_ctrl_cmd(STM32_CMD_GET_OVERRUN, &cmd, sizeof(spi_cmd_t)) != ESP_OK)
    {
        ESP_LOGE(TAG_LOG, "Overrun query: no response from STM32");
        return -1;
    }
    if (spi_buffer[0] != STM32_CMD_GET_OVERRUN)
    {
        ESP_LOGE(TAG_LOG, "Overrun query: bad echo %u", spi_buffer[0]);
        spi_ctrl_print_rx_buffer(spi_buffer);
        return -1;
    }
    _stmOverrun = spi_buffer[1] ? 1 : 0;
    #ifdef DEBUG_LOGGING
    ESP_LOGI(TAG_LOG, "STM32 overrun flag = %u", _stmOverrun);
    #endif
    return 0;
}
```
If any existing caller used it as `void`, leaving the return value unused is fine in C — no caller change required.

- [ ] **Step 3: Add the resync-count accessor**

In `main/logger.c`, after `Logger_getOverrun` (line ~997), add:
```c
uint32_t Logger_getResyncCount(void)
{
    return _resyncCount;
}
```
In `main/logger.h`, after the `Logger_getOverrun` declaration, add:
```c
uint32_t Logger_getResyncCount(void);
```

- [ ] **Step 4: Handle the fault at the top of LOGTASK_LOGGING_BUSY**

In `main/logger.c`, in `Logtask_logging()`, `case LOGTASK_LOGGING_BUSY:` (line ~2368), insert the fault handler BEFORE the `if (_dataReceived)` block (line ~2369):
```c
            case LOGTASK_LOGGING_BUSY:
                if (spi_ctrl_fault_pending())
                {
                    // STM signalled ring-overrun or SPI tear out-of-band. Discard
                    // any partial frame and disambiguate via the sticky overrun flag.
                    _dataReceived = 0;
                    int ov = -1;
                    for (int i = 0; i < 3 && ov < 0; i++)
                    {
                        if (Logger_queryOverrun() == 0) ov = Logger_getOverrun();
                    }
                    if (ov == 1)
                    {
                        // Ring overrun: STM stopped and returned to IDLE. Finalize cleanly.
                        SET_ERROR(_errorCode, ERR_LOGGER_DATA_OVERRUN);
                        LogTask_stop();
                        finalwrite = 1;
                    }
                    else
                    {
                        // SPI tear (ov==0) or query unreadable (ov<0): discard + resync.
                        _resyncCount++;
                        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
                        if (now_ms - _resyncWindowStart > 1000)
                        {
                            _resyncWindowStart = now_ms;
                            _resyncWindowCount = 0;
                        }
                        if (++_resyncWindowCount > 10)
                        {
                            // Tear storm: link is unusable -> fail loudly.
                            ESP_LOGE(TAG_LOG, "Fault: tear storm (>10/s), stopping");
                            SET_ERROR(_errorCode, ERR_LOGGER_STM32_FAULTY_DATA);
                            LogTask_stop();
                            finalwrite = 1;
                        }
                    }
                }
                if (_dataReceived)
                {
```
(The rest of the existing `if (_dataReceived)` body is unchanged.)

- [ ] **Step 5: Enable/disable the fault ISR at logging start/stop**

In `main/logger.c`, find where the data-ready interrupt is enabled at logging start — `spi_ctrl_datardy_int(1)` — and add `spi_ctrl_fault_int(1);` immediately after it. Find where it is disabled (`spi_ctrl_datardy_int(0)`, at stop/finalize) and add `spi_ctrl_fault_int(0);` immediately after it. (Search: `grep -n "spi_ctrl_datardy_int" main/logger.c`; mirror each call site so the fault ISR's lifetime exactly matches the data-ready ISR's.)

Reset the resync counters when logging (re)starts: at the same start site, add:
```c
                _resyncCount = 0;
                _resyncWindowStart = 0;
                _resyncWindowCount = 0;
```

- [ ] **Step 6: Build (the gate)**

Run:
```bash
source /home/paulus-potter/esp-idf/export.sh && cd /home/paulus-potter/dev/uberlogger-esp32 && idf.py build 2>&1 | tail -5
```
Expected: `Project build complete.`

- [ ] **Step 7: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-esp32
git add main/logger.c main/logger.h
git commit -m "feat(esp): react to STM fault line — query, disambiguate, resync/finalize

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Expose RESYNC_COUNT in /ajax/getStatus

**Files:**
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/rest_server.c` (`logger_getStatus_handler` ~line 435)

- [ ] **Step 1: Add RESYNC_COUNT to the status JSON**

In `main/rest_server.c`, in `logger_getStatus_handler`, immediately after the `OVERRUN` line (line 435 `cJSON_AddNumberToObject(root, "OVERRUN", Logger_getOverrun());`), add:
```c
    cJSON_AddNumberToObject(root, "RESYNC_COUNT", Logger_getResyncCount());
```
(`logger.h` is already included by `rest_server.c`; the prototype was added in Task 4.)

- [ ] **Step 2: Build (the gate)**

Run:
```bash
source /home/paulus-potter/esp-idf/export.sh && cd /home/paulus-potter/dev/uberlogger-esp32 && idf.py build 2>&1 | tail -5
```
Expected: `Project build complete.`

- [ ] **Step 3: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-esp32
git add main/rest_server.c
git commit -m "feat(esp): expose RESYNC_COUNT in /ajax/getStatus

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Firmware-update pin handoff (IO4 ↔ UART)

The flash path (`firmwareSTM32.c`) reclaims IO4 as the bootloader UART TX (`uart_set_pin`). The fault ISR must be released before that and the UART driver freed + IO4 returned afterward, so logging can re-init IO4 as the fault input.

**Files:**
- Modify: `/home/paulus-potter/dev/uberlogger-esp32/main/firmwareSTM32.c` (flash enter ~line 386; flash exit/end of routine)

- [ ] **Step 1: Release the fault ISR before installing the UART driver**

In `main/firmwareSTM32.c`, ensure `#include "spi_control.h"` is present (add if missing). Immediately BEFORE `uart_driver_install(UART_PORT, ...)` (line ~386), add:
```c
    // IO4 is shared with the STM fault line during logging. Release its edge ISR
    // before the bootloader UART claims the pin as TX.
    spi_ctrl_fault_int(0);
```

- [ ] **Step 2: Free the UART driver + reset IO4 when flashing finishes**

In `main/firmwareSTM32.c`, at the END of the flash routine (after the bootloader sequence completes / before the function returns — locate the matching cleanup point after `bootload_stm()` and the reset of `GPIO_STM32_BOOT0`/`NRESET` at lines ~473-497), add:
```c
    // Return IO4 from the bootloader UART back to a plain GPIO so the next
    // logging session can re-arm the fault edge ISR (spi_ctrl_fault_int(1)).
    uart_driver_delete(UART_PORT);
    gpio_reset_pin(GPIO_STM32_FAULT);
```
(Place this on every return path that installed the driver; if the routine has a single tail, one placement suffices. Verify with `grep -n "return" main/firmwareSTM32.c` around the flash function.)

- [ ] **Step 3: Build (the gate)**

Run:
```bash
source /home/paulus-potter/esp-idf/export.sh && cd /home/paulus-potter/dev/uberlogger-esp32 && idf.py build 2>&1 | tail -5
```
Expected: `Project build complete.`

- [ ] **Step 4: Commit**

```bash
cd /home/paulus-potter/dev/uberlogger-esp32
git add main/firmwareSTM32.c
git commit -m "feat(esp): hand IO4 between fault line and bootloader UART around flashing

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Integrated hardware gate

**STOP — confirm prerequisites with the user before flashing either chip** (standing rule). Confirm: STM powered + SPI-wired to ESP; ESP reachable (download mode if needed). Flash **STM** (ST-Link SWD via STM32CubeProgrammer) and **ESP** (esptool/`idf.py flash`).

**Files:**
- Modify: `/home/paulus-potter/dev/uberlogger-stm32/tools/bench/ul_soak.py` (surface RESYNC_COUNT)

- [ ] **Step 1: Surface RESYNC_COUNT in the soak**

In `tools/bench/ul_soak.py`, in the status/summary reporting, read and print `RESYNC_COUNT` alongside `OVERRUN` (the field now exists in `/ajax/getStatus`). Add it to the API-load summary print and the final PASS/FAIL line so tears are visible. Commit:
```bash
cd /home/paulus-potter/dev/uberlogger-stm32
git add tools/bench/ul_soak.py
git commit -m "test(bench): surface RESYNC_COUNT in overrun soak

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

- [ ] **Step 2: Flash both chips** (after user confirmation). STM via ST-Link; ESP via `idf.py -p <port> flash` or esptool. Confirm `/ajax/getStatus` shows the new firmware, `ERRORCODE 0`, `OVERRUN 0`, `RESYNC_COUNT 0`.

- [ ] **Step 3: 1000 Hz soak (the headline gate)**

Run:
```bash
cd /home/paulus-potter/dev/uberlogger-stm32/tools/bench
python3 ul_soak.py --rate 14 --dur 600 --outdir /tmp/soak/r14_faultline
```
Expected: session **completes without `ERR 8` abort**; `OVERRUN=0`; `RESYNC_COUNT` reflects any tears (non-zero is acceptable — they were recovered); file continuity intact (no dropped frames). Contrast with the pre-fix run, which hard-aborted with `ERR_LOGGER_STM32_FAULTY_DATA`.

- [ ] **Step 4: Forced ring-overrun check**

Induce a ring overrun (e.g. starve the ESP / run at 1000 Hz under heavy load until the STM ring fills). Expected: `OVERRUN=1`, **clean finalize**, `ERRORCODE` shows `ERR_LOGGER_DATA_OVERRUN` (0x02), and **no** `ERR_LOGGER_STM32_FAULTY_DATA` (0x08). Confirm the device returns to a healthy idle state afterward.

- [ ] **Step 5: 500 Hz regression**

Run:
```bash
python3 ul_soak.py --rate 13 --dur 600 --outdir /tmp/soak/r13_faultline
```
Expected: **PASS** — `OVERRUN=0`, no dropped frames, monotonic, `RESYNC_COUNT` low/zero (matches the prior 500 Hz PASS).

- [ ] **Step 6: Firmware-update regression (the riskiest interaction)**

Trigger a STM firmware update through the ESP (the `/fwupdate` path or whatever the app exposes). Expected: the STM flashes successfully over the UART (proving the IO4↔UART direction handoff works), then a normal logging session starts cleanly afterward with the fault ISR re-armed (`RESYNC_COUNT` increments correctly under a subsequent 1000 Hz soak). If flashing fails, the Task 6 handoff is wrong — fix before proceeding.

- [ ] **Step 7: Record results**

Update the memory file `uberlogger-stm32-phase1-refactor.md` with the fault-line outcome (1000 Hz now recoverable, 500 Hz still PASS, firmware-update intact, any RESYNC_COUNT figures), and note whether 1000 Hz is now a releasable continuous rate or remains overrun-prone-but-graceful.

---

## Notes for the implementer

- **Standing rule:** confirm prerequisites with the user before flashing either chip (Task 7).
- **No wire-format change** — do not bump `UL_PROTOCOL_VERSION`. The fault line is purely out-of-band.
- **Graceful degradation** is intentional: old STM + new ESP (no fault edges → today's behavior) and new STM + old ESP (ESP ignores IO4) both remain safe. No version gate.
- If the STM `app_init()` body differs from what's quoted, just add the `fault_line_init_output();` call without disturbing existing logic.
- The ESP fault ISR and the data-ready handshake share one task notification; `spi_ctrl_fault_pending()` (checked first each loop) is what disambiguates. Do not add a second notification index.
