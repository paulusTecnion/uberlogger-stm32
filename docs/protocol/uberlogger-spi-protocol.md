# Uberlogger STM32 ↔ ESP32 SPI Protocol

**Status:** frozen (Phase 1). This document is the *prose companion* to the
machine-readable single source of truth, [`Core/Inc/ul_protocol.h`](../../Core/Inc/ul_protocol.h).

> `ul_protocol.h` is the authority for every name, value, struct field, and
> byte offset described here. It is vendored **byte-identically** into both
> repositories:
> - `uberlogger-stm32` : `Core/Inc/ul_protocol.h`
> - `uberlogger-esp32` : `main/ul_protocol.h`
>
> The two copies must be `diff`-clean. `UL_PROTOCOL_VERSION` (currently `2`) is
> the sync tripwire: each repo carries a build-visible
> `_Static_assert(UL_PROTOCOL_VERSION == 2, "ul_protocol.h copies are out of sync")`
> (STM32 in `Core/Src/config.c`, ESP32 in `main/logger.c`). If the prose below
> ever disagrees with the header, the header wins — fix the prose.

The STM32 is the data acquisition front-end (ADC + GPIO + RTC timestamps). The
ESP32 is the host: it issues commands and consumes the sampled data stream,
writing it to the SD card and/or the web UI.

---

## 1. Physical link

| Parameter      | Value                                             |
|----------------|---------------------------------------------------|
| Role           | STM32 = **SPI slave**, ESP32 = **SPI master**     |
| SPI mode       | **Mode 0** (CPOL = 0 / `SPI_POLARITY_LOW`, CPHA = 0 / `SPI_PHASE_1EDGE`) |
| Bit order      | **MSB first** (`SPI_FIRSTBIT_MSB`)                |
| Word size      | **8-bit** (`SPI_DATASIZE_8BIT`)                   |
| Clock          | ~**10 MHz**                                        |
| Chip select    | Hardware NSS (`SPI_NSS_HARD_INPUT`, STM32 PA15 = SPI1_NSS) |
| Flow signal    | `STM_DATA_RDY` (STM32 PB6, output) — see handshake |

The STM32 SPI peripheral is `hspi1`, configured in `Core/Src/main.c`
(`MX_SPI1_Init`). All transfers on the STM32 side are DMA-driven
(`HAL_SPI_Transmit_DMA` / `HAL_SPI_Receive_DMA`).

### The `DATA_RDY` handshake

`STM_DATA_RDY` is an STM32 output the ESP32 watches as an interrupt line. It is
the slave's way of telling the master "a transfer I owe you is staged in DMA,
clock it out now."

- **Driven HIGH** by the STM32 in `spi_ctrl_send()` the instant a TX DMA is
  armed (e.g. a command response, or a streaming data buffer).
- **Driven LOW** when the transaction completes, or on a timeout/abort (see §5).

There are two traffic patterns over the same link:

1. **Command path** (ESP32 → STM32, request/response). The ESP32 master clocks
   an 8-byte command (`spi_cmd_t`) to the slave, then reads back an 8-byte
   response framed as the same struct (`command` echoed, second byte = response
   code). The STM32 raises `DATA_RDY` when its response DMA is staged.
2. **Streaming path** (STM32 → ESP32, bulk sample data). While logging, the
   STM32 fills a `spi_msg_1_t` / `spi_msg_2_t` buffer, arms TX DMA, raises
   `DATA_RDY`, and the ESP32 clocks out the full 2048-byte transaction.

---

## 2. Command set (`stm32cmd_t`)

ESP32 → STM32. One byte, the `command` field of `spi_cmd_t`.

| Name                                 | Value | Meaning |
|--------------------------------------|-------|---------|
| `STM32_CMD_NOP`                      | 0x00  | No-op / ping |
| `STM32_CMD_SETTINGS_MODE`            | 0x01  | Enter settings/config mode |
| `STM32_CMD_SETTINGS_SYNC`            | 0x02  | Sync settings |
| `STM32_CMD_MEASURE_MODE`             | 0x03  | Enter measure/logging mode |
| `STM32_CMD_SET_RESOLUTION`           | 0x04  | Set ADC resolution (12/16 bit) |
| `STM32_CMD_SET_SAMPLE_RATE`          | 0x05  | Set sample rate |
| `STM32_CMD_SET_ADC_CHANNELS_ENABLED` | 0x06  | Set enabled-channel bitmask |
| `STM32_CMD_SET_DATETIME`             | 0x07  | Set RTC date/time |
| `STM32_CMD_SINGLE_SHOT_MEASUREMENT`  | 0x08  | One-shot measurement |
| `STM32_CMD_SEND_LAST_ADC_BYTES`      | 0x09  | Flush/return the last ADC buffer |
| `STM32_CMD_SET_LOGMODE`              | 0x0A  | Set log mode (raw/csv) |
| `STM32_CMD_SET_RANGE`                | 0x0B  | Set input range (10V/60V) |
| `STM32_CMD_SET_TRIGGER_MODE`         | 0x0C  | Set trigger mode |
| `CMD_UNKNOWN`                        | 0x0D  | Sentinel (not a wire command) |
| `STM32_CMD_SET_LP_CONFIG`            | 0x0E  | Configure low-power trigger mode (v2) |
| `STM32_CMD_SET_ARMED_WINDOW`         | 0x0F  | Set daily armed window (v2, reserved) |
| `STM32_CMD_GET_DATETIME`             | 0x10  | Read STM32 RTC date/time (v2, reserved) |

### Command / response struct (`spi_cmd_t`, 8 bytes)

| Offset | Field            | Notes |
|--------|------------------|-------|
| 0      | `command`        | a `stm32cmd_t` value (request), or echoed back (response) |
| 1      | `data` / `data0` | primary argument (request) **or** response code (response) — same byte, two names (see §6) |
| 2      | `data1`          | argument byte |
| 3      | `data2`          | argument byte |
| 4      | `data3`          | argument byte |
| 5      | `data4`          | argument byte |
| 6      | `data5`          | argument byte |
| 7      | `data6`          | argument byte |

`sizeof(spi_cmd_t) == 8` (pinned by `_Static_assert` in the header).

### Response codes

In a response frame, byte 1 carries the result code. Two **independent** enums
exist in `ul_protocol.h` that happen to share the values 1/2 on the wire — keep
them distinct when reading the header:

- `stm32resp_t` → `{ STM32_RESP_OK = 1, STM32_RESP_NOK }` (+ `RESP_OK`/`RESP_NOK`
  aliases). What the **ESP32** checks (`rx[1] == STM32_RESP_OK`).
- `spi_cmd_resp_t` → `{ CMD_RESP_NOP = 0, CMD_RESP_OK = 1, CMD_RESP_NOK = 2 }`.
  What the **STM32** writes (`resp.data = CMD_RESP_OK`). Note this enum is 0-based
  (`CMD_RESP_NOP = 0`), so OK/NOK land on 1/2 — matching the other enum by value,
  not by being the same type.

| Wire value | ESP32 (`stm32resp_t`)   | STM32 (`spi_cmd_resp_t`) |
|------------|-------------------------|--------------------------|
| 0          | —                       | `CMD_RESP_NOP`           |
| 1          | `STM32_RESP_OK`         | `CMD_RESP_OK`            |
| 2          | `STM32_RESP_NOK`        | `CMD_RESP_NOK`           |

The ESP32 validates a response as `rx[0] == <command echoed>` and
`rx[1] == STM32_RESP_OK`. The STM32 builds it as `resp.command = <cmd>` and
`resp.data = CMD_RESP_OK` (value 1, identical on the wire).

---

## 3. Streaming data layout

While logging, the STM32 ships fixed **2048-byte** transactions. Two struct
shapes alternate (the ESP32 disambiguates by start/stop framing bytes — the ADC
value `0xFFFF` is impossible, so it is reused as the `START_STOP_BYTE_VALUE`
marker).

Shared layout constants (`ul_protocol.h`):

```
DATA_LINES_PER_SPI_TRANSACTION = 70
ADC_VALUES_PER_SPI_TRANSACTION = 70 * 8  = 560
ADC_BYTES_PER_SPI_TRANSACTION  = 560 * 2 = 1120
GPIO_BYTES_PER_SPI_TRANSACTION = 70
TIME_BYTES_PER_SPI_TRANSACTION = 70 * 12 = 840
START_STOP_NUM_BYTES           = 2
```

### `spi_msg_1_t` (start-byte framed) — `sizeof == 2048`

| Offset | Field      | Size (bytes) | Notes |
|--------|------------|--------------|-------|
| 0      | `startByte`| 2            | start framing |
| 2      | `dataLen`  | 2            | `uint16_t` valid line count |
| 4      | `padding0` | 12           | alignment |
| 16     | `timeData` | 840          | `s_date_time_t[70]` |
| 856    | `gpioData` | 70           | one GPIO byte per line |
| 926    | `padding1` | 2            | alignment |
| 928    | `adcData`  | 1120         | union: `uint8_t[1120]` / `uint16_t[560]` |

### `spi_msg_2_t` (stop-byte framed, mirror) — `sizeof == 2048`

| Offset | Field      | Size (bytes) | Notes |
|--------|------------|--------------|-------|
| 0      | `adcData`  | 1120         | union: `uint8_t[1120]` / `uint16_t[560]` |
| 1120   | `padding1` | 2            | alignment |
| 1122   | `gpioData` | 70           | one GPIO byte per line |
| 1192   | `timeData` | 840          | `s_date_time_t[70]` |
| 2032   | `padding0` | 12           | alignment |
| 2044   | `dataLen`  | 2            | `uint16_t` valid line count |
| 2046   | `stopByte` | 2            | stop framing |

All offsets above are pinned by `_Static_assert`s in `ul_protocol.h`; if the
header fails to compile, the wire layout has been changed.

### `s_date_time_t` (12 bytes per line)

| Offset | Field        | Size |
|--------|--------------|------|
| 0      | `year`       | 1    |
| 1      | `month`      | 1    |
| 2      | `date`       | 1    |
| 3      | `hours`      | 1    |
| 4      | `minutes`    | 1    |
| 5      | `seconds`    | 1    |
| 6      | `padding1`   | 1    |
| 7      | `padding2`   | 1    |
| 8      | `subseconds` | 4 (`uint32_t`) |

`sizeof(s_date_time_t) == 12` (pinned).

---

## 4. Why two mirrored message shapes

`spi_msg_1_t` puts framing/metadata at the front; `spi_msg_2_t` mirrors it with
framing at the tail. They are the same 2048-byte envelope read from either end,
which lets the acquisition DMA double-buffer without copying and lets the ESP32
validate a transaction by its leading **or** trailing framing bytes. Both are
exactly 2048 bytes so four of them pack into a single SD-card flush buffer.

---

## 5. Watchdog / timeout behavior

SPI transfers on the STM32 are DMA-driven and guarded by two hardware timers
(see `Core/Src/spi_ctrl.c`):

- **TIM14 — TX-timeout watchdog.** Armed in `spi_ctrl_send()` when a TX DMA
  starts (and `DATA_RDY` is raised). If the master does not clock the transfer
  out before TIM14 elapses, `spi_ctrl_loop()` sees `SPI_CTRL_TX_TIMEOUT`, aborts
  the DMA, and drives `DATA_RDY` LOW.
- **TIM16 — RX-timeout watchdog.** Armed in `spi_ctrl_receive()` when an RX DMA
  starts. On elapse, `spi_ctrl_loop()` sees `SPI_CTRL_RX_TIMEOUT` and aborts.

`SPI_TIMEOUT` (1000 ms, `Core/Inc/spi_ctrl.h`) is the blocking-call ceiling.
On any timeout the link is returned to idle and `DATA_RDY` is cleared, so a
stalled master cannot wedge the slave. On the ESP32 side, the logger applies its
own per-transaction timeout proportional to `DATA_LINES_PER_SPI_TRANSACTION`.

---

## 6. Protocol v2 command byte layouts

Added in `UL_PROTOCOL_VERSION 2` (2026-06-11) for low-power triggered mode. All
three commands use the shared 8-byte `spi_cmd_t` struct (§2).

### `STM32_CMD_SET_LP_CONFIG` (0x0E)

Configures the low-power trigger source, threshold, edge, and capture duration.
This is the only v2 command that is fully implemented in Plan 1.

| Byte | Field              | Meaning |
|------|--------------------|---------|
| 0    | `command`          | `0x0E` |
| 1    | `data` / `data0`   | source: `0` = analog, `1` = digital |
| 2    | `data1`            | channel (1-based; 1–8 analog, 1–6 digital) |
| 3    | `data2`            | threshold low byte (LE `uint16_t`, raw ADC counts at current resolution) |
| 4    | `data3`            | threshold high byte |
| 5    | `data4`            | edge: `0` = rising-above, `1` = falling-below (raw-count domain; see §7) |
| 6    | `data5`            | duration low byte (LE `uint16_t`, seconds, ≥ 1) |
| 7    | `data6`            | duration high byte |

For a **digital source**, threshold and edge encode the GPIO pin selection:
`data1` carries the channel number and the trigger edge polarity is encoded in
`data4` (rising/falling). `data2`/`data3` threshold is ignored.

For **digital triggers** the GPIO pin for trigger monitoring is supplied by
`STM32_CMD_SET_TRIGGER_MODE`'s `gpio` field, selecting which DIO pin the EXTI is
armed on.

### `STM32_CMD_SET_ARMED_WINDOW` (0x0F) — reserved

**Not yet implemented** (Plan 2). Configures an optional daily armed window;
outside the window both chips drop to deepest sleep.

| Byte | Field    | Meaning |
|------|----------|---------|
| 0    | `command`| `0x0F` |
| 1    | `data`   | enable: `0` = off, `1` = on |
| 2    | `data1`  | window start low byte (LE `uint16_t`, minutes since midnight) |
| 3    | `data2`  | window start high byte |
| 4    | `data3`  | window end low byte (LE `uint16_t`, minutes since midnight) |
| 5    | `data4`  | window end high byte |
| 6–7  | reserved | |

### `STM32_CMD_GET_DATETIME` (0x10) — reserved

**Not yet implemented** (Plan 2). No request payload; the STM32 returns its
RTC date/time in the response frame. Exact response layout is TBD at
implementation — it will carry date/time fields sufficient for the ESP32 to
re-sync its wall clock after deep-sleep wake, reversing the current flow
(today time only flows ESP32 → STM32 via `STM32_CMD_SET_DATETIME`).

---

## 7. Low-power trigger mode

**Mode value 3** in `STM32_CMD_SET_TRIGGER_MODE`. Trigger parameters are then
supplied via `STM32_CMD_SET_LP_CONFIG` (§6). The low-power mode is bench-verified
in Plan 1 (`feature/low-power-mode`).

### State flow

```
IDLE
  │  MEASURE_MODE command (trigger-mode = LP)
  ▼
LP_PRECHECK
  │  Waits: re-arm holdoff elapsed AND signal on the "safe" side of the
  │  threshold (crossing semantics — the trigger fires on a directional
  │  crossing, not a level). Both conditions must be true before arming.
  ▼
LP_ARMED  ◄──────────────────────────────────────────────────┐
  │  Main loop: WFI (Sleep mode). ADC + TIM3 + DMA run.      │
  │  Analog: AWD1 hardware watchdog fires on threshold cross. │
  │  Digital: EXTI fires on selected pin edge.                │
  │  SETTINGS_MODE command → exits to CONFIG (settings edit). │
  ▼                                                           │
CAPTURING                                                     │
  │  Existing acquisition pipeline active (TIM3 → DMA →       │
  │  framing → SPI). Duration timer (RTC-based). DATA_READY   │
  │  rises on first full frame — also the ESP32 wake signal.  │
  │  EXT_PIN_VALUE (PA9) driven HIGH during capture.          │
  │  On duration expiry: final frame sent, holdoff started.   │
  └─► re-arm holdoff ──────────────────────────────────────────┘
```

`LP_PRECHECK` and `LP_ARMED` are new states in `Core/Src/app.c`. AWD arm/disarm
lives in `Core/Src/acquisition.c`. Settings are parsed in `Core/Src/config.c`.

### EXT_PIN_VALUE signaling (PA9)

`EXT_PIN_VALUE` (PA9) is driven **HIGH** for the entire capture duration and
**LOW** at all other times (idle, armed, holdoff). This is identical to the
`EXTERNAL` trigger mode's signaling from the ESP32's perspective: the existing
ESP32 firmware (≤1.3.3) requires no change to detect end-of-capture.

### Analog threshold domain and the inverting front end

Thresholds and edges sent in `STM32_CMD_SET_LP_CONFIG` are in the **raw ADC
count domain**, not in volts.

The analog front end is **inverting**: a rising input voltage produces a falling
raw ADC count. On the 10 V range the relationship is:

```
volts = 15.1699 × (1 − 2 × raw / 4095)
```

This corresponds to ESP32-side constant `V_OFFSET_10V = 151699029`. An analogous
formula applies on the 60 V range using `V_OFFSET_60V`.

Consequence for the ESP32: when the user requests a **rising-voltage** trigger,
the ESP32 must send **edge = 1 (falling-below)** in the raw domain, and vice
versa. The conversion formula to go from user volts to raw counts (10 V range):

```
raw = round(4095 × (1 − V / 15.1699) / 2)
```

**Threshold register scaling (AWD):** the AWD threshold registers are 12-bit.
In 12-bit mode the threshold is the raw count directly. In 16-bit
(oversampled) mode the STM32 right-shifts the 16-bit threshold value by 4 before
writing the AWD registers (`threshold >> 4`), so the AWD compares against the
upper 12 bits of the post-oversampling result. This has been **bench-proven** in
both 12-bit and 16-bit modes (Plan 1 `lp_bench.py`).

### Command handling while armed (`LP_ARMED` / `LP_PRECHECK`)

| Command                      | Response while armed |
|------------------------------|----------------------|
| `STM32_CMD_SETTINGS_MODE`    | OK; transitions to CONFIG (exits LP, allows settings edit) |
| `STM32_CMD_NOP`              | silently ignored (no response) |
| `STM32_CMD_SEND_LAST_ADC_BYTES` | served (flushes the final partial frame; see below) |
| all others                   | NOK |

**End-of-capture tail mechanics:** at capture end the STM32 has a partially
filled frame in its DMA buffer. This frame is available via
`STM32_CMD_SEND_LAST_ADC_BYTES`. Stock ESP32 firmware (≤1.3.3) never issues this
command during a logging session, so the final partial frame is **dropped** by
stock firmware. An LP-aware ESP32 (Plan 2) must request `SEND_LAST_ADC_BYTES`
at the end of each capture to collect this tail data.

**Note on SPI receive posted while armed:** if a pending SPI receive DMA was
armed at the point of trigger, it is cancelled via `spi_ctrl_cancel_receive()`
before the capture pipeline starts. This was verified necessary on the bench
(Plan 1) and is implemented.

### ESP32 settings re-sync after STM32 reset

The STM32 boots at 10 Hz defaults regardless of the previous session's settings.
The ESP32 must re-send all settings (resolution, rate, channels, range, trigger
mode, LP config) after any STM32 reset before arming. This is a Plan-2/3 design
requirement.

---

## 8. The `spi_cmd_t` / `stm32cmd_t` name reconciliation

Historically the two repos disagreed on **type names** (never on bytes):

| Concept           | STM32 (old)     | ESP32 (old) | Canonical (`ul_protocol.h`) |
|-------------------|-----------------|-------------|-----------------------------|
| command enum      | `spi_cmd_esp_t` | `stm32cmd_t`| `stm32cmd_t` (+ `spi_cmd_esp_t` alias) |
| 8-byte cmd struct | `spi_cmd_t`     | `spi_cmd_t` | `spi_cmd_t` |
| struct byte 1     | `.data`         | `.data0`    | anonymous union: both `data` **and** `data0` |

Resolution (type-name / field-name only — **zero** byte, value, or logic
change):

- The command enum is canonically `stm32cmd_t`; a `typedef stm32cmd_t spi_cmd_esp_t;`
  alias keeps the STM32's old name resolving.
- The 8-byte struct keeps the agreed name `spi_cmd_t`.
- Its second byte is exposed under **both** `data` (STM32 call sites) and
  `data0` (ESP32 call sites) via an anonymous union — same offset, same byte —
  so neither repo's call sites needed editing.
- All response constants from both repos are retained at their original values.

See the reconciliation comment at the top of `ul_protocol.h` for the
authoritative statement.
