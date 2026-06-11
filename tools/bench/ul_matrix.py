#!/usr/bin/env python3
"""Run the Bench Matrix and dump combined metrics for one firmware build."""
import sys, json, os
sys.path.insert(0, "/tmp")
from ul_verify import run

OUTDIR = "/tmp/ul_baseline"
os.makedirs(OUTDIR, exist_ok=True)
tag = sys.argv[1] if len(sys.argv) > 1 else "baseline"

# (label, res_bits, rate_idx, mode, dur)
CONFIGS = [
    ("250Hz-16-CSV", 16, 12, 1, 8),
    ("250Hz-12-CSV", 12, 12, 1, 8),
    ("100Hz-16-CSV", 16, 11, 1, 6),
    ("25Hz-16-CSV",  16,  9, 1, 8),
    ("1Hz-16-CSV",   16,  5, 1, 10),
    ("avg10s-16-CSV",16,  4, 1, 22),   # ESP32 IIR-averaging path
    ("250Hz-16-RAW", 16, 12, 0, 6),    # byte-structure oracle
    ("100Hz-16-RAW", 16, 11, 0, 4),    # byte-structure oracle
]

results = {}
for label, res, rate, mode, dur in CONFIGS:
    try:
        m = run(res, rate, mode, dur, 0, 0, label, OUTDIR)
    except Exception as e:
        m = {"label": label, "ok": False, "error": repr(e)}
    results[label] = m

outpath = f"/tmp/ul_metrics_{tag}.json"
with open(outpath, "w") as f:
    json.dump(results, f, indent=2)

print("\n\n================ SUMMARY (%s) ================" % tag)
print(f"{'config':16} {'ok':3} {'rows':6} {'inf_Hz':7} {'dt_ms':7} {'stuck':6} {'extra'}")
for label, m in results.items():
    extra = ""
    if m.get("mode") == 0:
        extra = f"spacing={m.get('start_spacing')} dLens={m.get('msg1_dataLens')} markers={m.get('start_markers')}/{m.get('stop_markers')}"
    print(f"{label:16} {str(m.get('ok')):3} {str(m.get('rows','-')):6} "
          f"{str(m.get('inferred_hz','-')):7} {str(m.get('median_dt_ms','-')):7} "
          f"{str(len(m.get('stuck_channels',[])) if 'stuck_channels' in m else '-'):6} {extra}")
print("metrics written to", outpath)
