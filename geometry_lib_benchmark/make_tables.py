#!/usr/bin/env python3
"""Turn results.csv into markdown tables (quick and dirty)."""
import csv
import sys
from collections import defaultdict
from statistics import geometric_mean

rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else "results.csv")))
LIBS = ["clipper1", "clipper2", "boost"]
SIZES = ["1k", "10k", "100k", "1M", "real"]

data = {}
for r in rows:
    data[(r["case"], r["size"], r["op"], r["lib"])] = r


def cell(r):
    if r is None:
        return "–"
    s = r["status"]
    if s == "ok":
        ms = float(r["median_ms"])
        return f"{ms:.3g}" if ms < 1000 else f"{ms:,.0f}"
    if s == "unsupported":
        return "n/a"
    if s.startswith("skipped"):
        return "skip"
    return "EXC"


def ops_of(prefix):
    seen = []
    for r in rows:
        if r["op"].startswith(prefix) and r["op"] not in seen:
            seen.append(r["op"])
    return seen


cases = []
for r in rows:
    if (r["case"], r["size"]) not in cases:
        cases.append((r["case"], r["size"]))

# 1) Per-operation timing tables
for op in ops_of(""):
    print(f"\n#### `{op}` — median time in ms (ratio vs Clipper 6.4.2: lower is faster)\n")
    print("| case | size | in vertices | Clipper 6.4.2 | Clipper2 | boost::geometry | Clipper2 / C1 | boost / C1 |")
    print("|---|---|---:|---:|---:|---:|---:|---:|")
    for c, sz in cases:
        rs = {l: data.get((c, sz, op, l)) for l in LIBS}
        if all(v is None for v in rs.values()):
            continue
        nv = next(v["input_vertices"] for v in rs.values() if v)

        def ratio(l):
            a, b = rs.get(l), rs.get("clipper1")
            if a and b and a["status"] == "ok" and b["status"] == "ok" and float(b["median_ms"]) > 0:
                return f"{float(a['median_ms']) / float(b['median_ms']):.2f}"
            return "–"

        print(f"| {c} | {sz} | {nv} | {cell(rs['clipper1'])} | {cell(rs['clipper2'])} | {cell(rs['boost'])} | {ratio('clipper2')} | {ratio('boost')} |")

# 2) Geometric-mean speed ratio summary per operation family and size
print("\n#### Geometric mean of time ratio vs Clipper 6.4.2 (all cases where both succeeded)\n")
print("| operation family | size | Clipper2 / C1 | boost / C1 | #cases |")
print("|---|---|---:|---:|---:|")
families = {"intersection": "intersection", "union": "union", "offset_miter": "offset miter", "offset_round": "offset round", "offset_square": "offset square"}
for fam, label in families.items():
    for sz in SIZES:
        r2, rb = [], []
        n = 0
        for (c, s, op, l), r in data.items():
            if s != sz or not op.startswith(fam) or l != "clipper1" or r["status"] != "ok":
                continue
            base = float(r["median_ms"])
            if base <= 0:
                continue
            n += 1
            x = data.get((c, s, op, "clipper2"))
            if x and x["status"] == "ok":
                r2.append(float(x["median_ms"]) / base)
            x = data.get((c, s, op, "boost"))
            if x and x["status"] == "ok":
                rb.append(float(x["median_ms"]) / base)
        if n:
            g2 = f"{geometric_mean(r2):.2f}" if r2 else "–"
            gb = f"{geometric_mean(rb):.2f}" if rb else "–"
            print(f"| {label} | {sz} | {g2} | {gb} | {n} |")

# 3) Correctness findings
print("\n#### Correctness findings (|area diff vs Clipper2| > 0.1 %, ring count differs, invalid boost output, exceptions)\n")
print("| case | size | op | lib | status | area rel. diff | ring diff | boost validity |")
print("|---|---|---|---|---|---:|---:|---|")
for r in rows:
    if r["lib"] == "clipper2":
        continue
    bad = False
    if r["status"].startswith("exception"):
        bad = True
    elif r["status"] == "ok":
        if abs(float(r["area_rel_diff_vs_clipper2"])) > 1e-3 or r["ring_diff_vs_clipper2"] != "0":
            bad = True
        if r["boost_validity"].startswith("INVALID"):
            bad = True
    if bad:
        print(f"| {r['case']} | {r['size']} | {r['op']} | {r['lib']} | {r['status'][:60]} | {r['area_rel_diff_vs_clipper2']} | {r['ring_diff_vs_clipper2']} | {r['boost_validity'][:80]} |")
