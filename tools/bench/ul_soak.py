#!/usr/bin/env python3
"""Overrun soak: drive RAW 12-bit logging at a target rate while hammering the
HTTP API, and PASS iff the device reports zero STM ring overrun.

The high-rate RAW path (fs_code >= 13: 500/1000 Hz) is exactly where the STM->ESP
SPI ring is most likely to overrun, especially under concurrent web/API load. This
soak reproduces that stress and gates on the OVERRUN field (added in Task 6a) that
/ajax/getStatus now reports (0 = no overrun, 1 = STM ring overran during the session).

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


def get_overrun():
    """Read /ajax/getStatus and return the OVERRUN field (int) or None."""
    try:
        j = json.loads(_req("/ajax/getStatus"))
    except Exception as e:
        print(f"  !! getStatus read failed: {e!r}")
        return None
    if "OVERRUN" not in j:
        print("  !! getStatus has no OVERRUN field (firmware lacks Task 6a?)")
        return None
    try:
        return int(j["OVERRUN"])
    except Exception:
        return None


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

    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            time.sleep(1.0)
    finally:
        hammer.stop()

    _req("/ajax/loggerStop", "POST")
    wait_settle()
    time.sleep(2)  # let the session finalize (STM overrun query happens here)

    stats = hammer.summary()
    print("  API load summary:")
    for p, v in stats.items():
        print(f"    {p:22} ok={v['ok']:6} err={v['err']:4} last_err={v['last_err']}")

    overrun = get_overrun()
    print(f"  OVERRUN = {overrun}")

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

    # PASS iff OVERRUN == 0 AND (if we could analyze the file) continuity intact.
    overrun_ok = (overrun == 0)
    passed = overrun_ok and (file_ok is not False)

    print("\n" + "=" * 56)
    if passed:
        print(f"PASS  rate_idx={rate_idx} ({rate_hz} Hz)  OVERRUN={overrun}  "
              f"file_continuity={file_ok}")
    else:
        reason = []
        if not overrun_ok:
            reason.append(f"OVERRUN={overrun} (expected 0)")
        if file_ok is False:
            reason.append("file continuity broken")
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
