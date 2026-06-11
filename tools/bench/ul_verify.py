#!/usr/bin/env python3
"""Drive the Uberlogger over its HTTP API and verify ONE logging config.

Behavior-preserving-refactor oracle: we cannot byte-compare live analog data
run-to-run, so we assert on STRUCTURE + RATE + SANITY:
  - correct column layout
  - row count vs expected (rate x duration)  -> sample rate honored
  - median timestamp delta -> inferred Hz matches setting
  - timestamps monotonic
  - ADC values in range and not stuck (channel variance > 0)
Prints a metrics dict (JSON) the caller can compare across firmware builds.
"""
import sys, json, time, argparse, urllib.request, urllib.error, statistics, datetime

BASE = "http://192.168.4.1"
RATE_HZ = {5:1.0, 6:2.0, 7:5.0, 8:10.0, 9:25.0, 10:50.0, 11:100.0, 12:250.0}

def _req(path, method="GET", body=None, timeout=12):
    url = BASE + path
    data = body.encode() if body else None
    r = urllib.request.Request(url, data=data, method=method)
    if body: r.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(r, timeout=timeout) as resp:
        return resp.read().decode("utf-8", "replace")

def state():
    try:
        return json.loads(_req("/ajax/getStatus")).get("LOGGER_STATE")
    except Exception:
        return None

def wait_settle(timeout=15):
    """Wait until logger is idle (1) or single-shot (9)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = state()
        if s in (1, 9): return s
        time.sleep(0.5)
    return state()

def wait_state(target, timeout=10):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if state() == target: return True
        time.sleep(0.3)
    return False

def list_files():
    j = json.loads(_req("/ajax/getFileList/"))
    out = {}
    for v in j.get("root", {}).values():
        if v.get("TYPE") == "FILE":
            out[v["NAME"]] = int(v["SIZE"])
    return out

def analyze_csv(text, rate_hz, dur):
    lines = [l for l in text.splitlines() if l.strip()]
    res = {"rows": 0, "ok": False, "notes": []}
    if not lines:
        res["notes"].append("empty file"); return res
    header = lines[0].split(",")
    res["columns"] = len(header)
    res["header"] = lines[0]
    data = lines[1:]
    res["rows"] = len(data)
    # parse timestamps + adc
    ts = []
    adc_cols = list(range(1, 9))
    chan_vals = {c: [] for c in adc_cols}
    bad = 0
    for ln in data:
        f = ln.split(",")
        if len(f) != len(header): bad += 1; continue
        try:
            t = datetime.datetime.strptime(f[0], "%Y-%m-%d %H:%M:%S.%f")
            ts.append(t.timestamp())
        except Exception:
            try:  # older firmware "200-01-01" style -> skip ts, keep adc
                pass
            except Exception: pass
        for c in adc_cols:
            try: chan_vals[c].append(float(f[c]))
            except Exception: pass
    res["malformed_rows"] = bad
    if len(ts) >= 3:
        dts = [ts[i+1]-ts[i] for i in range(len(ts)-1)]
        med = statistics.median(dts)
        res["median_dt_ms"] = round(med*1000, 3)
        res["inferred_hz"] = round(1.0/med, 2) if med > 0 else None
        res["timestamps_monotonic"] = all(d >= -1e-6 for d in dts)
        res["span_s"] = round(ts[-1]-ts[0], 2)
    # per-channel range + stuck detection
    ranges = {}
    stuck = []
    for c in adc_cols:
        vs = chan_vals[c]
        if vs:
            mn, mx = min(vs), max(vs)
            ranges[f"AIN{c}"] = [round(mn,5), round(mx,5)]
            if mx == mn: stuck.append(f"AIN{c}")
    res["adc_ranges"] = ranges
    res["stuck_channels"] = stuck
    # expectations
    if rate_hz:  # fixed-rate (>=1 Hz) configs
        exp_rows = rate_hz * dur
        res["expected_rows_approx"] = exp_rows
        res["row_rate_ok"] = (0.7*exp_rows) <= len(data) <= (1.3*exp_rows)
        res["hz_ok"] = (res.get("inferred_hz") is not None and
                        abs(res["inferred_hz"] - rate_hz) <= max(0.1*rate_hz, 0.5))
        res["ok"] = bool(res.get("row_rate_ok") and res.get("hz_ok")
                         and res.get("timestamps_monotonic") and not stuck and bad == 0)
    else:  # sub-1Hz averaging path (STM 25Hz base, ESP decimates) -> no fixed Hz
        res["expected_rows_approx"] = None
        res["row_rate_ok"] = None
        res["hz_ok"] = None
        res["ok"] = bool(len(data) > 0 and res.get("timestamps_monotonic", True)
                         and not stuck and bad == 0)
    return res

def run(res_bits, rate_idx, mode, dur, avg, trig, label, outdir):
    rate_hz = RATE_HZ.get(rate_idx)
    print(f"\n=== {label}: res={res_bits} rate_idx={rate_idx}({rate_hz}Hz) mode={mode} dur={dur}s avg={avg} ===")
    s = wait_settle(); print(f"settle: {s}")
    before = set(list_files())
    cfg = {"ADC_RESOLUTION": res_bits, "LOG_SAMPLE_RATE": rate_idx,
           "LOG_MODE": mode, "EXT_TRIGGER_MODE": trig, "AVERAGE_SAMPLES": avg}
    _req("/ajax/setConfig", "POST", json.dumps(cfg))
    s = wait_settle(); print(f"settle after setConfig: {s}")
    r = _req("/ajax/loggerStart", "POST")
    if not wait_state(2, 8):
        print(f"  !! never reached LOGGING (state={state()}); start resp={r.strip()[:60]}")
    t0 = time.time(); time.sleep(dur)
    _req("/ajax/loggerStop", "POST")
    wait_settle()
    time.sleep(2)  # flush
    after = list_files()
    new = [n for n in after if n not in before]
    if not new:
        print("  !! no new file created"); return {"label": label, "ok": False, "error": "no file"}
    fname = sorted(new)[-1]
    print(f"  new file: {fname} ({after[fname]} bytes)")
    metrics = {"label": label, "file": fname, "bytes": after[fname],
               "res": res_bits, "rate_idx": rate_idx, "mode": mode}
    # download raw bytes and save locally (both modes) for later structural compare
    raw = _req("/ajax/getFileList/" + fname, timeout=40).encode("utf-8", "replace") \
          if mode == 1 else _download_bytes("/ajax/getFileList/" + fname)
    import os
    local = os.path.join(outdir, f"{label}__{fname}")
    with open(local, "wb") as fh: fh.write(raw)
    metrics["local"] = local
    if mode == 1:  # CSV: structure + rate + sanity
        metrics.update(analyze_csv(raw.decode("utf-8", "replace"), rate_hz, dur))
    else:  # RAW: byte-structure check
        metrics.update(analyze_raw(raw, dur))
    return metrics

def _download_bytes(path, timeout=40):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as resp:
        return resp.read()

def analyze_raw(data, dur):
    """Structural check of the RAW .dat: header + concatenated frame structs.
    Reports marker offsets/spacing and the per-frame dataLen so two builds can be
    compared structurally (values differ run-to-run; FRAMING must not)."""
    import re
    res = {"bytes": len(data)}
    starts = [m.start() for m in re.finditer(b"\xFA\xFB", data)]
    stops  = [m.start() for m in re.finditer(b"\xFB\xFA", data)]
    res["start_markers"] = len(starts)
    res["stop_markers"] = len(stops)
    res["first_start_off"] = starts[0] if starts else None
    deltas = [starts[i+1]-starts[i] for i in range(len(starts)-1)]
    res["start_spacing"] = sorted(set(deltas))  # expect [4096] for full-rate frames
    # dataLen of msg_1 lives 2 bytes after the FA FB start (offset +2, uint16 LE)
    dlens = []
    for s in starts:
        if s+4 <= len(data):
            dlens.append(int.from_bytes(data[s+2:s+4], "little"))
    res["msg1_dataLens"] = sorted(set(dlens))
    res["full_frame_structural"] = (res["start_spacing"] == [4096])
    res["ok"] = bool(starts and stops and len(data) > 0)
    return res

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--res", type=int, default=16)
    ap.add_argument("--rate", type=int, default=11)
    ap.add_argument("--mode", type=int, default=1)
    ap.add_argument("--dur", type=int, default=6)
    ap.add_argument("--avg", type=int, default=0)
    ap.add_argument("--trig", type=int, default=0)
    ap.add_argument("--label", default="run")
    ap.add_argument("--outdir", default="/tmp")
    a = ap.parse_args()
    m = run(a.res, a.rate, a.mode, a.dur, a.avg, a.trig, a.label, a.outdir)
    print(json.dumps(m, indent=2))
