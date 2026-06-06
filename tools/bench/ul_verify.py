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
import sys, json, time, argparse, urllib.request, urllib.error, statistics, datetime, struct

BASE = "http://192.168.4.1"
RATE_HZ = {5:1.0, 6:2.0, 7:5.0, 8:10.0, 9:25.0, 10:50.0, 11:100.0, 12:250.0,
           13:500.0, 14:1000.0}
# v2 RAW: fs_code -> per-line period in microseconds (0/unknown -> no fixed period)
PERIOD_US = {5:1000000, 6:500000, 7:200000, 8:100000, 9:40000, 10:20000,
             11:10000, 12:4000, 13:2000, 14:1000}
# v2 ul_frame_hdr_t markers / version
V2_START = b"\xFA\xFB"
V2_PROTOCOL_VERSION = 2
UL_FLAG_RES16 = 0x01
UL_FLAG_OVERRUN = 0x02
MAX_CAPACITY = 70  # capacity byte is untrusted; bound it when computing offsets

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
    else:  # RAW: route by container file-format version (v1 spi_msg vs v2 frames)
        fmt = detect_format(raw)
        metrics["raw_format"] = fmt
        if fmt == "v2":
            metrics.update(analyze_raw_v2(raw, dur))
        else:
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

def detect_format(data):
    """Auto-detect RAW .dat format from the container's file-format-version byte.

    .dat container: [uint32 header_length LE][settings[0] = file format version, ...]
    settings[0] lives at byte offset 4 (right after the 4-byte header_length).
      2 = old v1 frames (alternating spi_msg_1 / spi_msg_2)
      3 = new v2 frames (ul_frame_hdr_t + adc block + gpio block)
    """
    fmt_ver = data[4] if len(data) > 4 else 0
    return "v2" if fmt_ver >= 3 else "v1"   # 2 = old spi_msg_1/2 frames, 3 = v2 frames

def analyze_raw_v2(data, dur=None):
    """Structural + temporal check of a v2 RAW .dat.

    Container: [uint32 header_length LE][header_length-4 header bytes] then a stream
    of v2 frames, then a trailing uint64 LE total-row count at the END of the file.

    v2 frame = [14-byte ul_frame_hdr_t][adc: capacity*8 uint16 LE][gpio: capacity uint8]
      hdr (packed, LE): start[2]=FA FB, protocol_version(u8==2), flags(u8),
        base_epoch(u32 LE), base_subsec(u16 LE, Q16), fs_code(u8),
        line_count(u8, valid lines <= capacity), capacity(u8, stride), pad(u8)
    Per-line time: t_us(i) = base_epoch*1e6 + (base_subsec*1e6 >> 16) + i*PERIOD_US[fs_code]

    Reconstructs every valid line's timestamp, asserts strict monotonicity across the
    whole capture, that the inferred per-line period == PERIOD_US[fs_code], and that
    frame stride == 14 + capacity*17. Returns a metrics dict comparable across builds.
    """
    res = {"bytes": len(data), "format": "v2", "anomalies": []}
    if len(data) < 4:
        res["anomalies"].append("file too short for header_length"); res["ok"] = False
        return res
    header_length = int.from_bytes(data[0:4], "little")
    res["header_length"] = header_length
    # trailing uint64 LE total-row count lives at the END of the file
    if len(data) < header_length + 8:
        res["anomalies"].append("file too short for frames + trailing rowcount")
        res["ok"] = False
        return res
    total_rows_trailer = int.from_bytes(data[-8:], "little")
    res["total_rows_trailer"] = total_rows_trailer
    frame_end = len(data) - 8  # frames live between header and the 8-byte trailer

    pos = header_length
    times_us = []           # reconstructed per-line timestamps across the whole capture
    within_frame_diffs = [] # per-line spacing strictly inside a frame (must be exact)
    boundary_diffs = []     # spacing across frame boundaries (RTC-quantized)
    adc_all_min = None
    adc_all_max = None
    rep_adc = None          # representative first valid sample (8 channels)
    line_counts = []
    capacities = set()
    fs_codes = set()
    flags_seen = set()
    overrun_frames = 0
    total_lines = 0
    frame_strides = set()
    frame_no = 0

    while pos + 14 <= frame_end:
        hdr = data[pos:pos + 14]
        # start[2], protocol_version u8, flags u8, base_epoch u32, base_subsec u16,
        # fs_code u8, line_count u8, capacity u8, pad u8
        (s0, s1, proto, flags, base_epoch, base_subsec,
         fs_code, line_count, capacity, _pad) = struct.unpack("<BBBBIHBBBB", hdr)
        if not (s0 == 0xFA and s1 == 0xFB):
            res["anomalies"].append(f"frame {frame_no}@{pos}: bad start {s0:02X} {s1:02X}")
            break
        if proto != V2_PROTOCOL_VERSION:
            res["anomalies"].append(f"frame {frame_no}@{pos}: protocol_version {proto} != 2")
            break
        # capacity byte is untrusted -> bound it before computing offsets
        if not (0 < capacity <= MAX_CAPACITY):
            res["anomalies"].append(f"frame {frame_no}@{pos}: capacity {capacity} out of 1..{MAX_CAPACITY}")
            break
        stride = 14 + capacity * 17  # 14 hdr + adc(capacity*8*2) + gpio(capacity*1)
        frame_strides.add(stride)
        if pos + stride > frame_end:
            res["anomalies"].append(f"frame {frame_no}@{pos}: stride {stride} overruns frame region")
            break
        if line_count > capacity:
            res["anomalies"].append(f"frame {frame_no}@{pos}: line_count {line_count} > capacity {capacity}")
            line_count = capacity  # bound so we never read past the adc/gpio blocks
        capacities.add(capacity)
        fs_codes.add(fs_code)
        flags_seen.add(flags)
        if flags & UL_FLAG_OVERRUN:
            overrun_frames += 1
        line_counts.append(line_count)

        adc_off = pos + 14
        per = PERIOD_US.get(fs_code, 0)
        base_us = base_epoch * 1_000_000 + (base_subsec * 1_000_000 >> 16)
        frame_first_t = base_us
        for i in range(line_count):
            # adc[i*8 + ch], 8 channels of uint16 LE
            line_off = adc_off + i * 16
            adc = struct.unpack("<8H", data[line_off:line_off + 16])
            mn, mx = min(adc), max(adc)
            adc_all_min = mn if adc_all_min is None else min(adc_all_min, mn)
            adc_all_max = mx if adc_all_max is None else max(adc_all_max, mx)
            if rep_adc is None:
                rep_adc = list(adc)
            t = base_us + i * per
            if times_us:
                d = t - times_us[-1]
                if i == 0:
                    boundary_diffs.append(d)   # gap from previous frame's last line
                else:
                    within_frame_diffs.append(d)
            times_us.append(t)
        total_lines += line_count
        frame_no += 1
        pos += stride

    res["frames"] = frame_no
    res["total_lines"] = total_lines
    res["line_counts"] = sorted(set(line_counts))
    res["capacities"] = sorted(capacities)
    res["fs_codes"] = sorted(fs_codes)
    res["frame_stride"] = sorted(frame_strides)
    res["flags_seen"] = sorted(flags_seen)
    res["overrun_frames"] = overrun_frames
    res["adc_min"] = adc_all_min
    res["adc_max"] = adc_all_max
    res["adc_representative"] = rep_adc

    # rate / period checks
    fs = sorted(fs_codes)[0] if len(fs_codes) == 1 else None
    res["fs_code"] = fs
    expected_period = PERIOD_US.get(fs, 0) if fs is not None else 0
    res["expected_period_us"] = expected_period
    res["inferred_hz"] = round(1_000_000.0 / expected_period, 3) if expected_period else None

    # One STM RTC tick (1/2048 s ~= 488 us): a fresh per-frame base_subsec is
    # quantized to this resolution, so the gap across a frame boundary may differ
    # from the nominal period by up to ~one tick. WITHIN a frame, lines are derived
    # arithmetically from a single base -> spacing must be EXACTLY the table period.
    RTC_TICK_US = 1_000_000.0 / 2048.0
    if len(times_us) >= 2:
        all_diffs = within_frame_diffs + boundary_diffs
        res["timestamps_monotonic"] = all(d > 0 for d in all_diffs)
        res["timestamp_span_us"] = times_us[-1] - times_us[0]
        inferred = statistics.median(all_diffs) if all_diffs else None
        res["inferred_period_us"] = inferred
        within_exact = (expected_period > 0 and
                        all(d == expected_period for d in within_frame_diffs))
        # boundary gaps: monotonic and within one RTC tick of the nominal period
        boundary_ok = all(0 < d and abs(d - expected_period) <= RTC_TICK_US
                          for d in boundary_diffs)
        res["within_frame_period_exact"] = within_exact
        res["boundary_gaps_ok"] = boundary_ok
        res["period_ok"] = bool(within_exact and boundary_ok)
    else:
        res["timestamps_monotonic"] = True
        res["timestamp_span_us"] = 0
        res["inferred_period_us"] = None
        res["within_frame_period_exact"] = (expected_period == 0)
        res["boundary_gaps_ok"] = True
        res["period_ok"] = (expected_period == 0)

    # ADC sanity: not all-stuck across the whole capture
    res["adc_stuck"] = (adc_all_min is not None and adc_all_min == adc_all_max)
    # row-count trailer should agree with the lines we actually decoded
    res["rowcount_matches"] = (total_rows_trailer == total_lines)
    if not res["rowcount_matches"]:
        res["anomalies"].append(
            f"trailer rows {total_rows_trailer} != decoded lines {total_lines}")

    res["ok"] = bool(frame_no > 0
                     and res["timestamps_monotonic"]
                     and res["period_ok"]
                     and len(frame_strides) == 1
                     and not res["adc_stuck"]
                     and not res["anomalies"])
    return res

def compare_csv_equiv(v1_metrics, v2_metrics):
    """Assert a v2 capture is equivalent to a v1 capture of an overlapping config.

    Reuses the file's existing tolerance conventions (the same 0.1*Hz / 0.5 Hz band
    analyze_csv uses for inferred_hz, and the per-channel adc_ranges convention). The
    reconstructed v2 timestamp spacing must land within the RTC resolution (~0.49 ms,
    one tick of the 1/2048 s STM RTC) of v1's measured spacing.

    Returns (pass: bool, detail: str).
    """
    RTC_RES_MS = 1000.0 / 2048.0  # ~0.488 ms, one RTC subsecond tick
    fails = []

    v1_hz = v1_metrics.get("inferred_hz")
    v2_hz = v2_metrics.get("inferred_hz")
    if v1_hz is None or v2_hz is None:
        fails.append(f"missing inferred_hz (v1={v1_hz} v2={v2_hz})")
    else:
        # same band analyze_csv uses for hz_ok
        if abs(v2_hz - v1_hz) > max(0.1 * v1_hz, 0.5):
            fails.append(f"inferred_hz mismatch v1={v1_hz} v2={v2_hz}")

    # identical column layout (header string from analyze_csv, or column count)
    v1_hdr = v1_metrics.get("header")
    v2_hdr = v2_metrics.get("header")
    if v1_hdr is not None and v2_hdr is not None:
        if v1_hdr != v2_hdr:
            fails.append(f"column layout differs:\n  v1={v1_hdr}\n  v2={v2_hdr}")
    else:
        v1_cols = v1_metrics.get("columns")
        v2_cols = v2_metrics.get("columns")
        if v1_cols != v2_cols:
            fails.append(f"column count differs v1={v1_cols} v2={v2_cols}")

    # ADC ranges within tolerance: analog values differ run-to-run, so just require
    # both within the same physical range (per-channel mins/maxes overlap-comparable).
    v1_r = v1_metrics.get("adc_ranges")
    v2_r = v2_metrics.get("adc_ranges")
    if v1_r and v2_r:
        for ch, (lo1, hi1) in v1_r.items():
            if ch not in v2_r:
                fails.append(f"{ch} present in v1 but missing in v2"); continue
            lo2, hi2 = v2_r[ch]
            span1 = hi1 - lo1
            tol = max(0.1 * abs(span1), 0.5)  # 10% of v1 span, floor 0.5 units
            if abs(lo2 - lo1) > tol or abs(hi2 - hi1) > tol:
                fails.append(f"{ch} range v1={[lo1,hi1]} v2={[lo2,hi2]} beyond tol {tol:.3f}")

    # timestamp spacing within one RTC tick (~0.49 ms)
    v1_dt = v1_metrics.get("median_dt_ms")
    v2_dt = v2_metrics.get("median_dt_ms")
    if v2_dt is None and v2_metrics.get("inferred_period_us"):
        v2_dt = v2_metrics["inferred_period_us"] / 1000.0
    if v1_dt is None or v2_dt is None:
        fails.append(f"missing dt (v1={v1_dt} v2={v2_dt})")
    elif abs(v2_dt - v1_dt) > RTC_RES_MS:
        fails.append(f"timestamp spacing v1={v1_dt}ms v2={v2_dt}ms beyond RTC res {RTC_RES_MS:.3f}ms")

    if fails:
        return False, "; ".join(fails)
    return True, (f"equivalent: hz~{v2_hz}, dt~{v2_dt}ms (<= {RTC_RES_MS:.3f}ms RTC), "
                  f"columns + adc ranges match")

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
