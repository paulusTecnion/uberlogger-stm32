#!/usr/bin/env python3
"""LP-trigger bench verification script.

Drives ONE complete LP trigger cycle on a real Uberlogger via its HTTP API,
using a Korad KA3305P bench supply on /dev/ttyACM0 to set the analog voltage.

Prerequisites (caller must satisfy before running):
  - Device idle at http://192.168.4.1 with SD card inserted.
  - STM32 already flashed with LP_BENCH_FORCE enabled (correct override values).
  - Korad CH1 wired to AIN1 on the Uberlogger.
  - Caller MUST have already pushed setConfig to the device with the correct
    ADC_RESOLUTION, LOG_SAMPLE_RATE, LOG_MODE=1, EXT_TRIGGER_MODE=1, AVERAGE_SAMPLES=0
    BEFORE invoking this script.  This script does NOT call setConfig — that
    responsibility belongs to the test-matrix driver so the config matches the
    override values in firmware.

Usage:
    lp_bench.py --korad /dev/ttyACM0 --safe 2.0 --trip 6.0 --rate-hz 25 \\
                --duration 10 [--expect-min-rows N] [--label LABEL] \\
                [--hold-after 40]

Assertions performed on the captured CSV:
  - Exactly ONE new file created during the session.
  - Row count >= expect-min-rows (default: floor(duration * rate_hz * 0.85) - lines_per_tx).
  - Inferred sample rate within 5 % of --rate-hz.
  - AIN1 (column index 1, i.e. the 2nd column after timestamp) within ±0.25 V of
    --trip voltage for rows 6-10 (settled after ~5-row Korad output ramp).
  - Timestamps monotonic.
"""

import argparse
import datetime
import json
import math
import serial
import statistics
import sys
import time
import urllib.error
import urllib.request

# ---------------------------------------------------------------------------
# Device HTTP helpers
# ---------------------------------------------------------------------------

BASE = "http://192.168.4.1"
RATE_HZ = {5: 1.0, 6: 2.0, 7: 5.0, 8: 10.0, 9: 25.0, 10: 50.0, 11: 100.0, 12: 250.0}


def _req(path, method="GET", body=None, timeout=15):
    url = BASE + path
    data = body.encode() if body else None
    req = urllib.request.Request(url, data=data, method=method)
    if body:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read().decode("utf-8", "replace")


def get_status():
    try:
        return json.loads(_req("/ajax/getStatus"))
    except Exception:
        return {}


def get_state():
    return get_status().get("LOGGER_STATE")


def get_errorcode():
    return get_status().get("ERRORCODE", 0)


def wait_idle(timeout=20):
    """Wait until state is 1 or 9 (idle / single-shot idle)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = get_state()
        if s in (1, 9):
            return s
        time.sleep(0.5)
    return get_state()


def ensure_idle(label=""):
    """If session is active, stop it first."""
    s = get_state()
    if s == 2:
        print(f"  [{label}] state=2, stopping session first …")
        _req("/ajax/loggerStop", "POST")
        r = wait_idle(20)
        print(f"  [{label}] after stop: state={r}")
    elif s not in (1, 9):
        print(f"  [{label}] unexpected state={s}, waiting …")
        wait_idle(20)


def list_files():
    j = json.loads(_req("/ajax/getFileList/", timeout=20))
    out = {}
    for v in j.get("root", {}).values():
        if v.get("TYPE") == "FILE":
            out[v["NAME"]] = int(v["SIZE"])
    return out


# ---------------------------------------------------------------------------
# Korad KA3305P helper
# ---------------------------------------------------------------------------

class Korad:
    """Minimal driver for Korad KA3305P over USB-serial (9600 baud)."""

    def __init__(self, port):
        self._s = serial.Serial(port, baudrate=9600, timeout=1)
        time.sleep(0.3)
        # drain
        self._s.read(self._s.in_waiting or 1)

    def _send(self, cmd):
        self._s.write((cmd + "\n").encode())
        time.sleep(0.25)

    def _query(self, cmd, nbytes=16):
        self._s.read(self._s.in_waiting or 0)  # drain
        self._s.write((cmd + "\n").encode())
        time.sleep(0.25)
        return self._s.read(nbytes).decode("ascii", "replace").strip()

    def idn(self):
        return self._query("*IDN?", 64)

    def vset(self, volts):
        """Set CH1 voltage (0–8 V allowed for this bench)."""
        if volts < 0 or volts > 8:
            raise ValueError(f"Voltage {volts} out of 0–8 V bench range")
        self._send(f"VSET1:{volts:.2f}")

    def vread(self):
        """Readback set-point from supply."""
        raw = self._query("VSET1?")
        try:
            return float(raw)
        except ValueError:
            return None

    def close(self):
        self._s.close()


# ---------------------------------------------------------------------------
# CSV analysis
# ---------------------------------------------------------------------------

def analyze_csv(text, rate_hz, duration, expect_min_rows, trip_v, label, ain1_check_slice=None):
    """Parse CSV and return metrics dict with PASS/FAIL fields."""
    lines = [ln for ln in text.splitlines() if ln.strip()]
    metrics = {
        "label": label,
        "rows": 0,
        "inferred_hz": None,
        "timestamps_monotonic": None,
        "ain1_first10_ok": None,
        "row_count_ok": None,
        "hz_ok": None,
        "notes": [],
    }

    if not lines:
        metrics["notes"].append("empty file")
        return metrics

    header = lines[0].split(",")
    data_lines = lines[1:]
    metrics["rows"] = len(data_lines)
    metrics["header"] = lines[0]

    ts = []
    ain1_vals = []
    bad = 0
    for ln in data_lines:
        fields = ln.split(",")
        if len(fields) != len(header):
            bad += 1
            continue
        try:
            t = datetime.datetime.strptime(fields[0], "%Y-%m-%d %H:%M:%S.%f")
            ts.append(t.timestamp())
        except Exception:
            pass
        try:
            ain1_vals.append(float(fields[1]))  # AIN1 is column index 1
        except Exception:
            pass

    metrics["malformed_rows"] = bad

    # Timestamp metrics
    if len(ts) >= 3:
        dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        med = statistics.median(dts)
        metrics["median_dt_ms"] = round(med * 1000, 3)
        metrics["inferred_hz"] = round(1.0 / med, 2) if med > 0 else None
        metrics["timestamps_monotonic"] = all(d >= -1e-6 for d in dts)
        metrics["span_s"] = round(ts[-1] - ts[0], 2)
    else:
        metrics["notes"].append(f"too few timestamps ({len(ts)}) to infer rate")

    # Row count assertion
    metrics["row_count_ok"] = len(data_lines) >= expect_min_rows

    # Hz assertion (5 % tolerance)
    inf_hz = metrics.get("inferred_hz")
    if inf_hz is not None:
        metrics["hz_ok"] = abs(inf_hz - rate_hz) <= 0.05 * rate_hz
    else:
        metrics["hz_ok"] = False

    # AIN1 assertion: within ±0.15 V of trip voltage.
    # For rising-edge captures (safe < trip): check rows 1–10 (they should
    # already be at trip voltage when capture starts).
    # For falling-edge captures (safe > trip): the voltage is still mid-ramp
    # in the first ~5 rows; check rows 6–10 instead (settled region).
    # The slice is passed in by the caller; default to first 10.
    chk_slice = ain1_check_slice if ain1_check_slice is not None else slice(0, 10)
    check_rows = ain1_vals[chk_slice]
    # Tolerance: ±0.25 V — accommodates Korad KA3305P output accuracy (~2% at 6 V)
    # plus ADC noise. ±0.15 V was too tight for the 6 V setpoint.
    AIN1_TOL_V = 0.25
    if len(check_rows) >= 1:
        max_dev = max(abs(v - trip_v) for v in check_rows)
        metrics["ain1_max_dev_first10"] = round(max_dev, 4)
        metrics["ain1_first10_ok"] = max_dev <= AIN1_TOL_V
        metrics["ain1_first10_vals"] = [round(v, 4) for v in ain1_vals[:10]]
        metrics["ain1_checked_rows"] = f"[{chk_slice.start}:{chk_slice.stop}]"
    else:
        metrics["notes"].append("no AIN1 values parsed for check slice")
        metrics["ain1_first10_ok"] = False

    return metrics


# ---------------------------------------------------------------------------
# Main bench routine
# ---------------------------------------------------------------------------

def run_lp_bench(args):
    label = args.label
    rate_hz = args.rate_hz
    duration = args.duration
    safe_v = args.safe
    trip_v = args.trip
    hold_after = args.hold_after  # seconds to hold at trip voltage after capture (for no-retrigger test)

    # The STM32 fires when the voltage crosses the threshold (~4 V). The Korad
    # output ramps rather than stepping, so the first ~5 rows capture the signal
    # mid-ramp regardless of direction. Assert on rows 6–10 (indices 5–9) which
    # are settled near the final trip voltage in both rising and falling cases.
    ain1_check_slice = slice(5, 10)

    # lines_per_transaction: 25 at 25 Hz, 70 at >=100 Hz (DATA_LINES_PER_SPI_TRANSACTION)
    if rate_hz >= 100:
        lines_per_tx = 70
    else:
        lines_per_tx = 25

    # Default expect-min-rows: floor(duration * rate) * 0.85 minus one frame
    if args.expect_min_rows is not None:
        expect_min_rows = args.expect_min_rows
    else:
        expect_min_rows = max(0, int(math.floor(duration * rate_hz * 0.85)) - lines_per_tx)

    print(f"\n{'='*60}")
    print(f"LP-BENCH: {label}")
    print(f"  safe={safe_v} V  trip={trip_v} V  rate={rate_hz} Hz  duration={duration} s")
    print(f"  expect_min_rows={expect_min_rows}  hold_after={hold_after} s")
    print(f"{'='*60}")

    # --- Open Korad ---
    print(f"\n[1] Opening Korad on {args.korad} …")
    korad = Korad(args.korad)
    idn = korad.idn()
    print(f"    IDN: {idn!r}")
    if not idn:
        print("FAIL: Korad not responding")
        sys.exit(1)

    # --- Set safe voltage ---
    print(f"\n[2] Setting Korad CH1 to SAFE {safe_v} V …")
    korad.vset(safe_v)
    time.sleep(0.5)
    vr = korad.vread()
    print(f"    Readback: {vr} V")

    # --- Ensure device is idle ---
    print(f"\n[3] Ensuring device idle …")
    ensure_idle(label)
    s = wait_idle(20)
    print(f"    State: {s}")
    if s not in (1, 9):
        print(f"FAIL: device not idle after stop (state={s})")
        korad.close()
        sys.exit(1)

    # --- Record file list before session ---
    print(f"\n[4] Recording file list before session …")
    files_before = set(list_files().keys())
    print(f"    {len(files_before)} existing files")

    # --- loggerStart ---
    print(f"\n[5] Starting logger session …")
    r = _req("/ajax/loggerStart", "POST")
    print(f"    Response: {r.strip()[:80]}")
    # Wait for state 2
    t_wait = time.time()
    while time.time() - t_wait < 10:
        s = get_state()
        if s == 2:
            break
        time.sleep(0.3)
    s = get_state()
    print(f"    State after start: {s}")
    if s != 2:
        print(f"FAIL: never reached LOGGING state (state={s})")
        korad.close()
        sys.exit(1)

    # --- Quiet check: 15 s at safe voltage, state must stay 2 with errorcode 0 ---
    print(f"\n[6] Quiet check: 15 s at {safe_v} V (state must stay 2, errorcode 0) …")
    quiet_ok = True
    for i in range(30):  # 30 × 0.5 s = 15 s
        time.sleep(0.5)
        st = get_status()
        cs = st.get("LOGGER_STATE")
        ec = st.get("ERRORCODE", 0)
        if cs != 2:
            print(f"  !! state={cs} at quiet-check second {i*0.5:.1f} — expected 2")
            quiet_ok = False
            break
        if ec != 0:
            print(f"  !! errorcode={ec} at quiet-check second {i*0.5:.1f}")
            quiet_ok = False
            break
    if not quiet_ok:
        st = get_status()
        print(f"FAIL: quiet check failed. getStatus={json.dumps(st)}")
        korad.close()
        sys.exit(1)
    print(f"    Quiet check passed (state=2, errorcode=0 throughout).")

    # --- Set trip voltage, wait duration+8 s ---
    print(f"\n[7] Setting trip voltage {trip_v} V (trigger should fire) …")
    korad.vset(trip_v)
    time.sleep(0.5)
    vr = korad.vread()
    print(f"    Readback: {vr} V")

    wait_total = duration + 8
    print(f"    Waiting {wait_total} s (duration={duration}+8 margin) …")
    t0 = time.time()
    ec_ok = True
    while time.time() - t0 < wait_total:
        time.sleep(1.0)
        st = get_status()
        cs = st.get("LOGGER_STATE")
        ec = st.get("ERRORCODE", 0)
        elapsed = round(time.time() - t0, 1)
        if ec != 0:
            print(f"  !! errorcode={ec} at t={elapsed}s")
            ec_ok = False
        # state may be 2 throughout (session stays active after capture, STM32 re-arms)
        if cs not in (2,):
            print(f"  note: state={cs} at t={elapsed}s (session may have ended)")
    if not ec_ok:
        st = get_status()
        print(f"  WARNING: nonzero errorcode observed. getStatus={json.dumps(st)}")

    # --- Optional: hold at trip voltage (no-retrigger test) ---
    if hold_after > 0:
        print(f"\n[7b] Holding at {trip_v} V for {hold_after} s (no-retrigger check) …")
        time.sleep(hold_after)
        print(f"     Done holding.")

    # --- Set safe voltage, wait 3 s ---
    print(f"\n[8] Returning to safe voltage {safe_v} V …")
    korad.vset(safe_v)
    time.sleep(3.0)
    vr = korad.vread()
    print(f"    Readback: {vr} V")

    # --- loggerStop ---
    print(f"\n[9] Stopping logger session …")
    _req("/ajax/loggerStop", "POST")
    s = wait_idle(20)
    time.sleep(2)  # flush
    print(f"    State after stop: {s}")

    # --- List files, find new one ---
    print(f"\n[10] Checking for new files …")
    files_after = list_files()
    new_files = [n for n in files_after if n not in files_before]
    print(f"     New files: {new_files}")

    if len(new_files) == 0:
        print(f"FAIL: no new file created")
        korad.close()
        sys.exit(1)
    if len(new_files) > 1:
        print(f"FAIL: expected exactly 1 new file, got {len(new_files)}: {new_files}")
        korad.close()
        sys.exit(1)

    fname = new_files[0]
    fsize = files_after[fname]
    print(f"     File: {fname}  ({fsize} bytes)")

    # --- Download CSV ---
    print(f"\n[11] Downloading {fname} …")
    csv_text = _req("/ajax/getFileList/" + fname, timeout=60)
    print(f"     Downloaded {len(csv_text)} chars.")

    # --- Analyze CSV ---
    print(f"\n[12] Analyzing CSV …")
    metrics = analyze_csv(csv_text, rate_hz, duration, expect_min_rows, trip_v, label,
                          ain1_check_slice=ain1_check_slice)
    metrics["file"] = fname
    metrics["file_bytes"] = fsize
    metrics["expect_min_rows"] = expect_min_rows

    # --- Overall PASS/FAIL ---
    passes = [
        ("row_count_ok",          metrics.get("row_count_ok")),
        ("hz_ok",                 metrics.get("hz_ok")),
        ("timestamps_monotonic",  metrics.get("timestamps_monotonic")),
        ("ain1_first10_ok",       metrics.get("ain1_first10_ok")),
        ("single_new_file",       True),   # already checked above
    ]
    overall = all(v for _, v in passes)
    metrics["pass"] = overall

    print(f"\n{'='*60}")
    print(f"RESULT: {'PASS' if overall else 'FAIL'}  — {label}")
    print(f"  rows={metrics['rows']}  (min={expect_min_rows}  ok={metrics.get('row_count_ok')})")
    print(f"  inferred_hz={metrics.get('inferred_hz')}  (target={rate_hz}  ok={metrics.get('hz_ok')})")
    print(f"  timestamps_monotonic={metrics.get('timestamps_monotonic')}")
    print(f"  ain1_first10_ok={metrics.get('ain1_first10_ok')}  (trip={trip_v} V  "
          f"vals={metrics.get('ain1_first10_vals', [])[:3]} …)")
    if metrics.get("notes"):
        print(f"  notes: {metrics['notes']}")
    for name, val in passes:
        print(f"  {'OK' if val else 'FAIL'}: {name}")
    print(f"{'='*60}\n")

    print(json.dumps(metrics, indent=2))

    korad.close()
    return overall


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Korad-driven LP trigger bench verification for one trigger cycle.",
        epilog=(
            "IMPORTANT: caller must push setConfig (ADC_RESOLUTION, LOG_SAMPLE_RATE, "
            "LOG_MODE=1, EXT_TRIGGER_MODE=1, AVERAGE_SAMPLES=0) to the device BEFORE "
            "invoking this script so the ESP32 config matches the STM32 LP_BENCH_FORCE "
            "override values."
        ),
    )
    ap.add_argument("--korad", default="/dev/ttyACM0",
                    help="Serial port of Korad KA3305P (default: /dev/ttyACM0)")
    ap.add_argument("--safe", type=float, required=True,
                    help="Voltage (V) to hold while armed / idle (e.g. 2.0)")
    ap.add_argument("--trip", type=float, required=True,
                    help="Voltage (V) to set to trigger a capture (e.g. 6.0)")
    ap.add_argument("--rate-hz", type=float, required=True,
                    help="Expected sample rate in Hz (must match setConfig LOG_SAMPLE_RATE)")
    ap.add_argument("--duration", type=int, required=True,
                    help="LP capture duration in seconds (must match _lp_duration_s override)")
    ap.add_argument("--expect-min-rows", type=int, default=None,
                    help="Minimum rows in captured CSV. Default: floor(duration*rate*0.85)-lines_per_tx")
    ap.add_argument("--label", default="lp_bench",
                    help="Test label for logging (default: lp_bench)")
    ap.add_argument("--hold-after", type=int, default=0,
                    help="Seconds to hold at trip voltage AFTER capture (0=disabled). "
                         "Used for no-retrigger case E: supply stays high so signal never "
                         "returns to safe side, verifying the session produces only one capture.")

    args = ap.parse_args()
    ok = run_lp_bench(args)
    sys.exit(0 if ok else 1)
