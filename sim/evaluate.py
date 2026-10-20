#!/usr/bin/env python3
"""Score ulog-why against SITL ground-truth labels. Usage: evaluate.py <dir> [--bin path] [--out results.json]"""
import argparse, json, os, subprocess, collections
EXPECT = {"gps_loss": "gps", "baro_offset": "baro", "mag_offset": "mag", "thrust_loss": "propulsion",
          "battery_fail": "battery", "datalink_loss": "datalink", "none": "none"}
ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--bin", default="target/release/ulog-why")
ap.add_argument("--out"); ap.add_argument("-v", action="store_true"); a = ap.parse_args()
rows = []
for line in open(os.path.join(a.dir, "labels.jsonl")):
    lab = json.loads(line)
    if "ulg" not in lab or lab.get("error"): continue
    p = os.path.join(a.dir, lab["ulg"])
    if not os.path.exists(p): continue
    r = json.loads(subprocess.run([a.bin, p, "--json"], capture_output=True, text=True, check=True).stdout)
    v = r["verdict"]; root = v["root"]
    pred = v["root_category"]; exp = EXPECT[lab["fault"]]
    tinj = lab["inject_boot_ms"] / 1000
    dt = None if root is None else round(root["t"] - tinj, 2)
    cat_ok = pred == exp
    time_ok = exp == "none" or (dt is not None and -1.0 <= dt <= 15.0)
    rows.append(dict(trial=lab["trial"], fault=lab["fault"], mag=lab.get("mag"), expected=exp, predicted=pred,
                     dt_s=dt, cat_ok=cat_ok, ok=cat_ok and time_ok, anchor=(v["anchor"] or {}).get("what"),
                     root=(root or {}).get("what")))
    if a.v: print(f'{lab["trial"]:10} {lab["fault"]:14} -> {pred:11} dt={dt} {"OK" if rows[-1]["ok"] else "MISS"}  {(root or {}).get("what")}')
by = collections.defaultdict(lambda: [0, 0])
for r in rows: by[r["fault"]][0] += r["ok"]; by[r["fault"]][1] += 1
n = len(rows); k = sum(r["ok"] for r in rows)
print(f"overall {k}/{n} = {k / max(n, 1):.3f}")
for f_, (ok, tot) in sorted(by.items()): print(f"  {f_:14} {ok}/{tot}")
conf = collections.Counter((r["expected"], r["predicted"]) for r in rows)
print("confusion (expected -> predicted):", dict(conf))
dts = sorted(r["dt_s"] for r in rows if r["ok"] and r["dt_s"] is not None)
if dts: print(f"detection delay after injection (correct, non-control): median {dts[len(dts)//2]} s, max {dts[-1]} s")
if a.out: json.dump(dict(n=n, correct=k, per_fault=dict(by), rows=rows), open(a.out, "w"), indent=1)
