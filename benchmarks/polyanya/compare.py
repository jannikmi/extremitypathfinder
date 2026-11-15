"""Minimal comparison with Polyanya (issue #170): path lengths, preprocessing and query times.

Uses the world maps and query pairs of the pyvisgraph benchmark (``benchmarks.workloads``).
Polyanya runs via the Rust crate (``benchmarks/polyanya``, needs ``cargo``).
The maps are unbounded, Polyanya needs a bounded free space:
the bounding box of all obstacles and query points, extended by a margin, is the boundary.
A shortest path stays in the convex hull of the obstacles and its end points, so this does not change the paths.

    uv run python -m benchmarks.polyanya.compare --sizes 250 1k 2k --queries 200
"""

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

from benchmarks import adapters, workloads

HARNESS_DIR = Path(__file__).parent
BINARY = HARNESS_DIR / "target" / "release" / "polyanya-bench"
# Polyanya computes in f32: the relative tolerance of the path lengths
REL_TOL = 1e-4


def agree(a: float | None, b: float | None) -> bool:
    """both paths missing, or both lengths equal within the tolerance"""
    if a is None or b is None:
        return a is b
    return abs(a - b) <= REL_TOL * max(a, 1.0)


def build_harness():
    subprocess.run(
        ["cargo", "build", "--release", "--quiet"], cwd=HARNESS_DIR, check=True
    )


def bounding_box(holes, queries, margin: float = 1.0):
    xs = [x for h in holes for x, _ in h] + [p[0] for q in queries for p in q]
    ys = [y for h in holes for _, y in h] + [p[1] for q in queries for p in q]
    x0, x1, y0, y1 = (
        min(xs) - margin,
        max(xs) + margin,
        min(ys) - margin,
        max(ys) + margin,
    )
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def run_polyanya(holes, queries) -> dict:
    payload = {
        "boundary": bounding_box(holes, queries),
        "holes": holes,
        "queries": [[*s, *g] for s, g in queries],
    }
    result = subprocess.run(
        [str(BINARY)],
        input=json.dumps(payload),
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads(result.stdout)


def run_extremitypathfinder(holes, queries) -> dict:
    adapter = adapters.ExtremityPathfinder()
    start = time.perf_counter()
    env = adapter.build(holes)
    build_s = time.perf_counter() - start
    query_s, lengths = [], []
    for s, g in queries:
        start = time.perf_counter()
        path = adapter.query(env, s, g)
        query_s.append(time.perf_counter() - start)
        lengths.append(adapters.path_length(path) if path else None)
    return {
        "build_s": build_s,
        "query_s": query_s,
        "lengths": lengths,
        "numba": adapter.numba_active(),
    }


def compare(size: str, nr_queries: int) -> dict:
    data = workloads.load_world(workloads.parse_size(size))
    holes = data["holes"]
    queries = workloads.generate_queries([data], nr_queries)
    epf = run_extremitypathfinder(holes, queries)
    pa = run_polyanya(holes, queries)
    mismatches = [
        i
        for i, (a, b) in enumerate(zip(epf["lengths"], pa["lengths"], strict=True))
        if not agree(a, b)
    ]
    max_rel = max(
        (
            abs(a - b) / max(a, 1.0)
            for a, b in zip(epf["lengths"], pa["lengths"])
            if a is not None and b is not None
        ),
        default=0.0,
    )
    return {
        "map": data["name"],
        "vertices": data["vertices"],
        "epf_build_s": epf["build_s"],
        "pa_mesh_s": pa["triangulate_s"] + pa["merge_s"] + pa["bake_s"],
        "pa_triangulate_s": pa["triangulate_s"],
        "pa_polygons": pa["polygons"],
        "epf_query_ms": 1e3 * statistics.median(epf["query_s"]),
        "pa_query_ms": 1e3 * statistics.median(pa["query_s"]),
        "max_rel_diff": max_rel,
        "mismatches": mismatches,
        "numba": epf["numba"],
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--sizes", nargs="+", default=["250", "1k", "2k"])
    parser.add_argument("--queries", type=int, default=workloads.DEFAULT_NR_QUERIES)
    args = parser.parse_args(argv)
    build_harness()
    header = (
        f"{'map':<11}{'vertices':>9}{'epf build s':>13}{'pa mesh s':>11}"
        f"{'epf query ms':>14}{'pa query ms':>13}{'max rel diff':>14}{'mismatches':>12}"
    )
    print(header)
    failed = False
    for size in args.sizes:
        r = compare(size, args.queries)
        failed |= bool(r["mismatches"])
        print(
            f"{r['map']:<11}{r['vertices']:>9}{r['epf_build_s']:>13.3f}{r['pa_mesh_s']:>11.4f}"
            f"{r['epf_query_ms']:>14.3f}{r['pa_query_ms']:>13.4f}{r['max_rel_diff']:>14.1e}"
            f"{len(r['mismatches']):>12}"
        )
        if r["mismatches"]:
            print(f"  mismatching queries: {r['mismatches']}", file=sys.stderr)
    return int(failed)


if __name__ == "__main__":
    sys.exit(main())
