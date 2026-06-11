# Hardware bench-verification harness

Automated behavior-preservation check for the STM32 firmware refactor, driving a
**real Uberlogger** over the ESP32's HTTP API. Used as the per-task bench gate
(see `docs/superpowers/plans/2026-06-05-uberlogger-stm32-phase1-refactor.md`).

## Why metrics, not byte-diff
The ADC reads live analog pins, so values legitimately differ run-to-run. These
scripts assert on **structure + rate + sanity** instead:
- **CSV** runs → column layout, row count vs `rate x duration`, median timestamp
  delta → inferred Hz, timestamp monotonicity, ADC ranges, stuck-channel check.
- **RAW** runs → byte-structure of the frame stream: `FA FB`/`FB FA` marker
  spacing (4096 for full 70-line transactions), per-frame `dataLen`, marker
  balance. This is the strongest oracle for the `framing` module — the frame
  bytes must stay structurally identical even though values vary.

Compare a build's metrics against the pristine baseline; same structure + same
inferred rate = behavior preserved.

## Prerequisites
- The Uberlogger reachable at `http://192.168.4.1` (connect to its `Uberlogger-XXXX`
  SoftAP), with an SD card inserted.
- `python3` (the ESP-IDF env python works).

## Usage
```bash
# one config:  --res {12,16} --rate <enum idx> --mode {0=RAW,1=CSV} --dur <s>
python3 tools/bench/ul_verify.py --res 16 --rate 12 --mode 1 --dur 8 --label 250-16-csv

# full matrix, writes /tmp/ul_metrics_<tag>.json + saves files to /tmp/ul_baseline/
python3 tools/bench/ul_matrix.py <tag>
```
`LOG_SAMPLE_RATE` enum indices: 4=every-10s, 5=1Hz, 9=25Hz, 11=100Hz, 12=250Hz.

## Per-task gate
1. Build + flash the firmware build under test (headless builder + STM32CubeProgrammer CLI).
2. `python3 tools/bench/ul_matrix.py <task>` and compare its summary to the
   pristine `baseline` (RAW `start_spacing`/`dataLens`/markers must match exactly;
   CSV `inferred_hz` must match; row counts vary only by start/stop jitter).
