#!/usr/bin/env python3
"""Offline regression for the Phase 2A v2 RAW tooling (no hardware).

Builds a synthetic v2 .dat (valid container header with settings[0]=3 + two
hand-crafted contiguous v2 frames + trailing uint64 row count) and exercises:
  - ul_verify.detect_format   -> "v2" for settings[0]=3, "v1" for =2
  - ul_verify.analyze_raw_v2  -> markers/version, bounded capacity, exact within-frame
                                 period, monotonic timestamps, stride 14+cap*17
  - ul_verify.compare_csv_equiv -> pass on matching metrics, fail on Hz mismatch
  - convert_raw.py            -> column-identical CSV, reconstructed monotonic times

Run:  python3 tools/bench/test_v2_fixture.py     (exit 0 on PASS)
"""
import struct, sys, os, subprocess, tempfile, datetime, json

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ul_verify import detect_format, analyze_raw_v2, compare_csv_equiv  # noqa: E402

# convert_raw.py lives in the sibling ESP32 repo; allow override via env for CI.
CONVERT_RAW = os.environ.get(
    "CONVERT_RAW",
    os.path.normpath(os.path.join(
        HERE, "..", "..", "..", "uberlogger-esp32", "front", "www", "convert_raw.py")))

NUM_ADC = 8
NUM_DIO = 6


def build_header(fmt_ver, adc_range=0, adc_type=0, adc_enabled=0xFF,
                 adc_res=12, log_rate=12, gpio_enabled=0x00, dec=0, sep=0):
    settings = bytes([fmt_ver, adc_range, adc_type, adc_enabled, adc_res,
                      log_rate, gpio_enabled, dec, sep])
    offsets = struct.pack('<8i', *([0] * 8))
    labels = b""
    for n in range(NUM_ADC):
        lbl = f"AIN{n+1}".encode()
        labels += bytes([len(lbl)]) + lbl
    for n in range(NUM_DIO):
        lbl = f"DIO{n+1}".encode()
        labels += bytes([len(lbl)]) + lbl
    body = settings + offsets + labels
    header_length = 4 + len(body)            # includes the 4-byte length field
    return struct.pack('<I', header_length) + body, header_length


def build_v2_frame(base_epoch, base_subsec, fs_code, line_count, capacity,
                   adc_seed=1000, flags=0):
    hdr = struct.pack('<BBBBIHBBBB',
                      0xFA, 0xFB, 2, flags, base_epoch, base_subsec,
                      fs_code, line_count, capacity, 0)
    adc_vals = []
    for i in range(capacity):
        for ch in range(8):
            adc_vals.append(((adc_seed + i * 8 + ch) & 0xFFFF) if i < line_count else 0)
    adc_block = struct.pack(f'<{capacity*8}H', *adc_vals)
    gpio_block = bytes([(i & 0xFF) for i in range(capacity)])
    return hdr + adc_block + gpio_block


def main():
    tmp = tempfile.mkdtemp(prefix="ul_v2fix_")
    fs_code = 12            # 250 Hz, period 4000 us (ms-resolution CSV is exact here)
    capacity = 70
    period = 4000
    f1_lines = 70
    f2_lines = 30
    base1_epoch = 1_700_000_000
    base1_subsec = 0
    # Contiguous frames, as a continuous capture produces: frame2 base = frame1 base
    # + f1_lines*period. base_subsec is stored Q16 (1/2048 s RTC), so the boundary
    # lands within one RTC tick of the nominal period.
    f1_base_us = base1_epoch * 1_000_000 + (base1_subsec * 1_000_000 >> 16)
    f2_base_us = f1_base_us + f1_lines * period
    base2_epoch = f2_base_us // 1_000_000
    f2_subsec_us = f2_base_us - base2_epoch * 1_000_000
    base2_subsec = (f2_subsec_us << 16) // 1_000_000

    header, hlen = build_header(3, log_rate=fs_code)
    frame1 = build_v2_frame(base1_epoch, base1_subsec, fs_code, f1_lines, capacity, adc_seed=1000)
    frame2 = build_v2_frame(base2_epoch, base2_subsec, fs_code, f2_lines, capacity, adc_seed=5000)
    total_rows = f1_lines + f2_lines
    data = header + frame1 + frame2 + struct.pack('<Q', total_rows)

    path = os.path.join(tmp, "synth_v2.dat")
    with open(path, "wb") as f:
        f.write(data)
    print(f"wrote {path}: {len(data)} bytes, header_length={hlen}, total_rows={total_rows}")
    print(f"frame stride = 14 + {capacity}*17 = {14 + capacity*17}")

    print("\n--- detect_format ---")
    print("v2 file (settings[0]=3) ->", detect_format(data))
    v1_header, _ = build_header(2)
    print("v1 file (settings[0]=2) ->", detect_format(v1_header + b"\x00" * 100))
    assert detect_format(data) == "v2"
    assert detect_format(v1_header + b"\x00" * 100) == "v1"

    print("\n--- analyze_raw_v2 ---")
    m = analyze_raw_v2(data)
    print(json.dumps(m, indent=2, default=str))
    assert m["ok"], "analyze_raw_v2 failed ok"
    assert m["frames"] == 2, m["frames"]
    assert m["total_lines"] == total_rows, m["total_lines"]
    assert m["timestamps_monotonic"], "not monotonic"
    assert m["period_ok"], "period mismatch"
    assert m["within_frame_period_exact"], "within-frame spacing not exact"
    assert m["frame_stride"] == [14 + capacity * 17], m["frame_stride"]
    assert m["inferred_hz"] == 250.0, m["inferred_hz"]
    assert m["rowcount_matches"], "rowcount mismatch"
    assert not m["adc_stuck"], "adc stuck"
    print("\nanalyze_raw_v2 assertions PASSED")

    print("\n--- convert_raw.py on synthetic v2 ---")
    r = subprocess.run([sys.executable, CONVERT_RAW, path],
                       capture_output=True, text=True)
    print(r.stdout)
    if r.returncode != 0:
        print("STDERR:", r.stderr)
        raise SystemExit("convert_raw.py failed")
    csv_path = path.rsplit('.dat', 1)[0] + '.csv'
    with open(csv_path) as f:
        csv_text = f.read()
    csv_lines = [l for l in csv_text.replace("\r", "\n").split("\n") if l.strip()]
    header_line, data_lines = csv_lines[0], csv_lines[1:]
    print(f"CSV header: {header_line}")
    print(f"CSV data rows: {len(data_lines)} (expected {total_rows})")
    for l in data_lines[:3]:
        print("  ", l)

    assert len(data_lines) == total_rows, f"row count {len(data_lines)} != {total_rows}"
    ncols = len(header_line.split(","))     # time + 8 AIN + 0 GPIO = 9
    assert ncols == 9, f"header cols {ncols} != 9"
    for l in data_lines:
        assert len(l.split(",")) == ncols, f"row col mismatch: {l}"

    ts = [datetime.datetime.strptime(l.split(",")[0], "%Y-%m-%d %H:%M:%S.%f")
          for l in data_lines]
    diffs = [(ts[i + 1] - ts[i]).total_seconds() for i in range(len(ts) - 1)]
    assert all(d > 0 for d in diffs), "CSV timestamps not monotonic"
    RTC_TICK = 1.0 / 2048.0
    per_s = period / 1_000_000.0
    within = [d for d in diffs if abs(d - per_s) < 1e-6]
    boundary = [d for d in diffs if abs(d - per_s) >= 1e-6]
    assert len(boundary) <= 1, f"unexpected off-period gaps: {boundary}"
    for d in boundary:
        assert d > 0 and abs(d - per_s) <= max(RTC_TICK, 0.0011), f"bad boundary {d}s"
    print(f"\nconvert_raw v2 CSV assertions PASSED ({ncols} cols, {total_rows} rows, "
          f"strictly monotonic; {len(within)} within-frame @{per_s*1000:.0f}ms, "
          f"{len(boundary)} boundary gap)")

    print("\n--- compare_csv_equiv ---")
    v1_fake = {"inferred_hz": 250.0, "header": header_line, "columns": 9,
               "median_dt_ms": 4.0,
               "adc_ranges": {f"AIN{i}": [-5.0, 5.0] for i in range(1, 9)}}
    v2_fake = {"inferred_hz": 250.0, "header": header_line, "columns": 9,
               "inferred_period_us": 4000.0,
               "adc_ranges": {f"AIN{i}": [-5.0, 5.0] for i in range(1, 9)}}
    ok, detail = compare_csv_equiv(v1_fake, v2_fake)
    print("equiv pass:", ok, "-", detail)
    assert ok, "compare_csv_equiv should pass for matching metrics"
    v2_bad = dict(v2_fake); v2_bad["inferred_hz"] = 100.0
    ok2, d2 = compare_csv_equiv(v1_fake, v2_bad)
    print("equiv fail (hz):", ok2, "-", d2)
    assert not ok2

    print("\n=== ALL FIXTURE CHECKS PASSED ===")


if __name__ == "__main__":
    main()
