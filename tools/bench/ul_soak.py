#!/usr/bin/env python3
"""Overrun soak: drive RAW 12-bit logging at a target rate while hammering the
HTTP API.  PASS under stop-on-fault firmware iff EITHER:
  (a) the soak ran the full duration with no fault (OVERRUN==0, ERRORCODE==0,
      device still logging at end), OR
  (b) a known fault stopped the device promptly mid-soak (ERRORCODE is exactly
      0x02=ERR_LOGGER_DATA_OVERRUN or 0x08=ERR_LOGGER_STM32_FAULTY_DATA) AND
      the stop was detected within PROMPT_STOP_MAX_S seconds of elapsing.
In both cases the captured data must still pass the v2 file-continuity check.

The high-rate RAW path (fs_code >= 13: 500/1000 Hz) is exactly where the STM->ESP
SPI ring is most likely to overrun, especially under concurrent web/API load. This
soak reproduces that stress and gates on OVERRUN + ERRORCODE from /ajax/getStatus.
With stop-on-fault firmware, any SPI fault (ring overrun OR tear) causes the device
to STOP logging and finalize, setting a non-zero ERRORCODE.

Usage:
    python3 tools/bench/ul_soak.py --rate 14 --dur 600
        --rate : fs_code index (13 = 500 Hz, 14 = 1000 Hz)
        --dur  : soak duration in seconds

Exit 0 on PASS, non-zero on FAIL. Exercised at the hardware gate (Task 7).
"""
import sys, os, json, time, argparse, threading
import urllib.request, urllib.error

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/tmp")

# Reuse the device base URL, request helper, state/settle helpers, file listing and
# the v2 RAW analyzer from ul_verify (the device is on its SoftAP).
try:
    from ul_verify import (BASE, _req, state, wait_settle, wait_state,
                           list_files, detect_format, analyze_raw_v2, PERIOD_US)
except Exception as e:  # pragma: no cover - import-time guard for standalone runs
    print(f"FATAL: cannot import ul_verify helpers: {e!r}")
    sys.exit(2)

# Endpoints worker threads poll throughout the soak. getFileList is intentionally
# NOT polled while logging: prior runs show it returns HTTP 500 with a file open
# mid-logging (benign/expected) -- we only list when idle.
POLL_ENDPOINTS_WHILE_LOGGING = [
    "/ajax/getStatus",
    "/ajax/getValues",
    "/ajax/getRawAdc",
    "/ajax/getConfig",
]

# Known ERRORCODE bits (stop-on-fault firmware).
ERR_LOGGER_DATA_OVERRUN     = 0x02
ERR_LOGGER_STM32_FAULTY_DATA = 0x08
KNOWN_FAULT_CODES = frozenset({ERR_LOGGER_DATA_OVERRUN, ERR_LOGGER_STM32_FAULTY_DATA})

# A fault-stop is "prompt" if detected within this many seconds of occurring.
# A hang or device-unreachable period longer than this is a FAIL.
PROMPT_STOP_MAX_S = 10.0

# LOGGER_STATE value that means "actively logging" (matches firmware enum).
LOGGER_STATE_LOGGING = 2


def _download_bytes(path, timeout=60):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as resp:
        return resp.read()


def _safe_get(path):
    """One transient-tolerant GET. Returns (ok, status_text). A single failed
    request must never kill the soak."""
    try:
        _req(path, timeout=8)
        return True, "ok"
    except urllib.error.HTTPError as e:
        return False, f"HTTP {e.code}"
    except Exception as e:
        return False, f"{type(e).__name__}"


class Hammer:
    """Concurrent API load: each worker round-robins the poll endpoints until told
    to stop, accumulating per-endpoint ok/err counters."""
    def __init__(self, endpoints):
        self.endpoints = endpoints
        self._stop = threading.Event()
        self.lock = threading.Lock()
        self.stats = {p: {"ok": 0, "err": 0, "last_err": None} for p in endpoints}

    def _worker(self, wid):
        i = wid
        while not self._stop.is_set():
            path = self.endpoints[i % len(self.endpoints)]
            ok, detail = _safe_get(path)
            with self.lock:
                s = self.stats[path]
                if ok:
                    s["ok"] += 1
                else:
                    s["err"] += 1
                    s["last_err"] = detail
            i += 1
            time.sleep(0.05)  # ~20 req/worker/s; tight enough to stress, not flood

    def start(self, n_workers=4):
        self._stop.clear()
        self.threads = [threading.Thread(target=self._worker, args=(w,), daemon=True)
                        for w in range(n_workers)]
        for t in self.threads:
            t.start()

    def stop(self):
        self._stop.set()
        for t in self.threads:
            t.join(timeout=5)

    def summary(self):
        with self.lock:
            return {p: dict(v) for p, v in self.stats.items()}


def get_status_fields():
    """Read /ajax/getStatus and return (overrun, errorcode, logger_state) as ints.
    Returns (None, None, None) on failure."""
    try:
        j = json.loads(_req("/ajax/getStatus"))
    except Exception as e:
        print(f"  !! getStatus read failed: {e!r}")
        return None, None, None

    def _field(key):
        try:
            return int(j[key])
        except (KeyError, TypeError, ValueError):
            return None

    overrun      = _field("OVERRUN")
    errorcode    = _field("ERRORCODE")
    logger_state = _field("LOGGER_STATE")
    return overrun, errorcode, logger_state


def run_soak(rate_idx, dur, n_workers, outdir):
    rate_hz = round(1_000_000.0 / PERIOD_US[rate_idx], 1) if rate_idx in PERIOD_US else "?"
    print(f"=== OVERRUN SOAK: RAW 12-bit  rate_idx={rate_idx} ({rate_hz} Hz)  "
          f"dur={dur}s  workers={n_workers} ===")

    s = wait_settle(); print(f"settle: {s}")
    before = set(list_files())

    # RAW (LOG_MODE=0) + 12-bit + target rate, matching the setConfig keys ul_verify uses.
    cfg = {"ADC_RESOLUTION": 12, "LOG_SAMPLE_RATE": rate_idx,
           "LOG_MODE": 0, "EXT_TRIGGER_MODE": 0, "AVERAGE_SAMPLES": 0}
    _req("/ajax/setConfig", "POST", json.dumps(cfg))
    s = wait_settle(); print(f"settle after setConfig: {s}")

    r = _req("/ajax/loggerStart", "POST")
    if not wait_state(2, 8):
        print(f"  !! never reached LOGGING (state={state()}); start resp={r.strip()[:60]}")

    hammer = Hammer(POLL_ENDPOINTS_WHILE_LOGGING)
    hammer.start(n_workers)
    print(f"  hammering {n_workers} workers across {POLL_ENDPOINTS_WHILE_LOGGING}")

    # -- Soak loop: run for `dur` seconds, but watch for a stop-on-fault early exit.
    t0 = time.time()
    fault_stop_elapsed = None   # seconds into soak when fault-stop was detected
    fault_stop_errorcode = None  # ERRORCODE observed at fault-stop
    try:
        while time.time() - t0 < dur:
            time.sleep(1.0)
            elapsed = time.time() - t0
            # Poll LOGGER_STATE to catch a stop-on-fault mid-soak.
            try:
                _, ec, ls = get_status_fields()
                if ls is not None and ls != LOGGER_STATE_LOGGING:
                    fault_stop_elapsed = elapsed
                    fault_stop_errorcode = ec
                    print(f"  !! device left LOGGING state at t+{elapsed:.1f}s  "
                          f"LOGGER_STATE={ls}  ERRORCODE={hex(ec) if ec is not None else None}")
                    break
            except Exception as poll_err:
                # Transient poll failure; keep the soak running.
                print(f"  (poll error at t+{elapsed:.1f}s: {poll_err!r})")
    finally:
        hammer.stop()

    # Issue stop only if the device is (still) logging; if it self-stopped, skip.
    if fault_stop_elapsed is None:
        _req("/ajax/loggerStop", "POST")
    wait_settle()
    time.sleep(2)  # let the session finalize

    stats = hammer.summary()
    print("  API load summary:")
    for p, v in stats.items():
        print(f"    {p:22} ok={v['ok']:6} err={v['err']:4} last_err={v['last_err']}")

    overrun, errorcode, logger_state = get_status_fields()
    print(f"  OVERRUN={overrun}  ERRORCODE={hex(errorcode) if errorcode is not None else None}"
          f"  LOGGER_STATE={logger_state}")

    # Optional file-continuity check: now idle, listing is safe.
    file_ok = None
    file_metrics = None
    after = list_files()
    new = [n for n in after if n not in before]
    if new:
        fname = sorted(new)[-1]
        print(f"  new file: {fname} ({after[fname]} bytes)")
        try:
            raw = _download_bytes("/ajax/getFileList/" + fname)
            os.makedirs(outdir, exist_ok=True)
            local = os.path.join(outdir, f"soak_r{rate_idx}__{fname}")
            with open(local, "wb") as fh:
                fh.write(raw)
            if detect_format(raw) == "v2":
                file_metrics = analyze_raw_v2(raw)
                file_ok = bool(file_metrics.get("ok")
                               and file_metrics.get("timestamps_monotonic")
                               and not file_metrics.get("anomalies"))
                print(f"  file continuity: ok={file_ok} "
                      f"frames={file_metrics.get('frames')} "
                      f"lines={file_metrics.get('total_lines')} "
                      f"monotonic={file_metrics.get('timestamps_monotonic')} "
                      f"anomalies={file_metrics.get('anomalies')}")
            else:
                print("  file is not v2 RAW; skipping continuity check")
        except Exception as e:
            print(f"  !! could not pull/analyze captured file: {e!r}")
    else:
        print("  !! no new file created during soak")

    # --- PASS/FAIL ---
    # Path A: full-duration clean run — no fault at all.
    full_run_clean = (fault_stop_elapsed is None
                      and overrun == 0
                      and (errorcode or 0) == 0)

    # Path B: fault-stop with a known, single-bit ERRORCODE that arrived promptly.
    fault_stop_ok = False
    fault_stop_reason = None
    if fault_stop_elapsed is not None:
        ec = fault_stop_errorcode if fault_stop_errorcode is not None else errorcode
        if ec is None:
            fault_stop_reason = "ERRORCODE unreadable after fault-stop"
        elif ec not in KNOWN_FAULT_CODES:
            fault_stop_reason = (f"ERRORCODE={hex(ec)} has unexpected bits "
                                 f"(expected one of "
                                 f"{[hex(c) for c in sorted(KNOWN_FAULT_CODES)]})")
        elif fault_stop_elapsed > PROMPT_STOP_MAX_S:
            fault_stop_reason = (f"fault-stop detected late: t+{fault_stop_elapsed:.1f}s "
                                 f"> PROMPT_STOP_MAX_S={PROMPT_STOP_MAX_S}s (possible hang)")
        else:
            fault_stop_ok = True

    # Unexpected ERRORCODE bits even on a non-stopped soak.
    unexpected_errorcode = (fault_stop_elapsed is None
                            and errorcode is not None
                            and errorcode not in (0,) | KNOWN_FAULT_CODES)

    passed = (full_run_clean or fault_stop_ok) and (file_ok is not False) and not unexpected_errorcode

    print("\n" + "=" * 56)
    if passed:
        if full_run_clean:
            print(f"PASS  rate_idx={rate_idx} ({rate_hz} Hz)  "
                  f"OVERRUN={overrun}  ERRORCODE={hex(errorcode) if errorcode is not None else None}  "
                  f"file_continuity={file_ok}  (full-duration clean run)")
        else:
            ec_used = fault_stop_errorcode if fault_stop_errorcode is not None else errorcode
            print(f"PASS  rate_idx={rate_idx} ({rate_hz} Hz)  "
                  f"ERRORCODE={hex(ec_used) if ec_used is not None else None}  "
                  f"fault_stop_at=t+{fault_stop_elapsed:.1f}s  "
                  f"file_continuity={file_ok}  (prompt fault-stop, known code)")
    else:
        reason = []
        if not full_run_clean and not fault_stop_ok:
            if fault_stop_elapsed is None and not full_run_clean:
                if overrun != 0:
                    reason.append(f"OVERRUN={overrun} (expected 0)")
                if (errorcode or 0) != 0:
                    reason.append(f"ERRORCODE={hex(errorcode)} unexpected on clean run")
            if fault_stop_reason:
                reason.append(fault_stop_reason)
        if unexpected_errorcode:
            reason.append(f"ERRORCODE={hex(errorcode)} has unexpected bits")
        if file_ok is False:
            reason.append("file continuity broken")
        if not reason:
            reason.append("unknown failure")
        print(f"FAIL  rate_idx={rate_idx} ({rate_hz} Hz)  -> {'; '.join(reason)}")
    print("=" * 56)
    return passed


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="RAW 12-bit overrun soak under API load")
    ap.add_argument("--rate", type=int, default=14,
                    help="fs_code index (13=500Hz, 14=1000Hz)")
    ap.add_argument("--dur", type=int, default=600, help="soak duration, seconds")
    ap.add_argument("--workers", type=int, default=4, help="concurrent API workers")
    ap.add_argument("--outdir", default="/tmp/ul_soak", help="where to save captures")
    a = ap.parse_args()
    ok = run_soak(a.rate, a.dur, a.workers, a.outdir)
    sys.exit(0 if ok else 1)
