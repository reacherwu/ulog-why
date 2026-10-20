#!/usr/bin/env python3
"""Cross-check the Rust parser against pyulog: per-subscription sample counts must match exactly.
Usage: crosscheck_pyulog.py <bin> <log.ulg>..."""
import subprocess, sys
from pyulog import ULog
bin_, files = sys.argv[1], sys.argv[2:]
bad = 0
for f in files:
    u = ULog(f)
    py = {(d.name, d.multi_id): len(d.data["timestamp"]) for d in u.data_list}
    out = subprocess.run([bin_, "topics", f], capture_output=True, text=True)
    if out.returncode:
        print(f"{f}: rust parser error: {out.stderr.strip()}"); bad += 1; continue
    rs = {}
    for line in out.stdout.splitlines():
        n, m, c = line.split(); rs[(n, int(m))] = int(c)
    py = {k: v for k, v in py.items() if v}
    diff = {k: (py.get(k), rs.get(k)) for k in set(py) | set(rs) if py.get(k) != rs.get(k)}
    print(f"{f.split('/')[-1]}: {len(py)} subscriptions, {sum(py.values())} samples, mismatches {len(diff)} {list(diff.items())[:3]}")
    bad += bool(diff)
sys.exit(1 if bad else 0)
