"""measure how the preparation time of an environment grows and where it goes

For each environment size it reports

* the time of the two steps of the preparation: points and wedges (``compile_polygon_datastructs()``)
  and the visibility graph (``compute_graph()``)
* the number of candidates and edges ``find_visible()`` receives per origin
* the growth of the graph time relative to the previous size, as the exponent ``p`` of ``n^p``

environments:

* ``grid``: the square boundary with randomly shifted square holes of ``large_env_query_profile.py``
  (sizes are holes per side)
* ``world``: the cleaned world maps of the benchmarks (sizes as in ``python -m benchmarks.run --sizes``,
  needs the ``benchmark`` dependency group and downloads the Natural Earth data once)
* ``worst``: a worst case of AREA when every edge is checked against every candidate in its angle range (#216):
  small squares on a convex arc (most see each other, sizes are the number of squares) below as many long slabs,
  whose edges span the directions of most squares

FINDINGS (November 2026, Apple M-series, Python 3.14, graph time in seconds):

* the tree over the candidates (#216, ``li_chao_tree.blocked_candidates()``) bounds the worst case.
  ``worst``, n = 804 / 1604 / 3204 vertices, before -> after:
  with Numba 3.7 -> 1.2, 23.9 -> 4.7, 169.8 -> 18.5; without Numba 9.5 -> 2.7, 69.1 -> 15.5.
  Before, the graph time grew with n^2.7 to n^2.8, now with n^2.0 (the number of pairs of extremities).
  The visibility of one query point in front of the slabs: 164 -> 1.6 ms (n = 1604), 646 -> 3.3 ms (n = 3204).
* the typical maps became faster too (close edges first, edges behind them are dropped, no sweep in Python):
  ``world 1k / 2k / 5k``: with Numba 1.08 -> 0.85, 3.54 -> 2.81, 26.5 -> 18.5; without Numba 1.49 -> 1.33, 5.19 -> 4.69.
  ``grid 16 / 24``: with Numba 2.43 -> 1.89, 12.1 -> 9.0; without Numba (16) 3.05 -> 2.33.

usage:
    python prototypes/complexity_profile.py grid 8 16 24
    python prototypes/complexity_profile.py worst 50 100 200
    uv run --all-extras --group benchmark python prototypes/complexity_profile.py world 500 1k 2k
    NUMBA_DISABLE_JIT=1 python prototypes/complexity_profile.py grid 8 16   # without Numba
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from extremitypathfinder import PolygonEnvironment, utils

sys.path.insert(0, str(Path(__file__).resolve().parent))


def grid_environment(size: str):
    from large_env_query_profile import build_environment

    boundary, holes, _ = build_environment(int(size), np.random.default_rng(0))
    return boundary, holes


def world_environment(size: str):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from benchmarks import workloads

    data = workloads.load_world(workloads.parse_size(size))
    return None, data["holes"]


def worst_environment(size: str):
    nr = int(size)
    holes = []
    for i in range(nr):
        x = 4.0 * (i - nr // 2)
        y = round(x * x / (4.0 * nr), 3)
        holes.append([(x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1)])
    width = 400.0 * nr
    bottom = 4.0 * nr + 4.0
    for i in range(nr):
        y = bottom + 2.0 * i
        holes.append([(-width, y), (width, y), (width, y + 1), (-width, y + 1)])
    top = bottom + 2.0 * nr + 1
    boundary = [
        (-2 * width, -5.0),
        (2 * width, -5.0),
        (2 * width, top),
        (-2 * width, top),
    ]
    return boundary, holes


def timed_preparation(boundary, holes) -> tuple[PolygonEnvironment, dict]:
    times = {}

    def timed(name, function):
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            result = function(*args, **kwargs)
            times[name] = time.perf_counter() - start
            return result

        return wrapper

    original = utils.compile_polygon_datastructs, utils.compute_graph
    utils.compile_polygon_datastructs = timed("wedges", original[0])
    utils.compute_graph = timed("graph", original[1])
    try:
        start = time.perf_counter()
        env = PolygonEnvironment(boundary, holes, validate=False)
        times["total"] = time.perf_counter() - start
    finally:
        utils.compile_polygon_datastructs, utils.compute_graph = original
    return env, times


def counted_preparation(boundary, holes) -> dict:
    counts = {"origins": 0, "candidates": 0, "edges": 0}
    original_find = utils.find_visible

    def find_visible(origin, candidates, edges_to_check, *args, **kwargs):
        counts["origins"] += 1
        counts["candidates"] += len(candidates)
        counts["edges"] += len(edges_to_check)
        return original_find(origin, candidates, edges_to_check, *args, **kwargs)

    utils.find_visible = find_visible
    try:
        PolygonEnvironment(boundary, holes, validate=False)
    finally:
        utils.find_visible = original_find
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("environment", choices=["grid", "world", "worst"])
    parser.add_argument("sizes", nargs="+")
    args = parser.parse_args()
    load = {
        "grid": grid_environment,
        "world": world_environment,
        "worst": worst_environment,
    }[args.environment]
    # warm up (Numba compilation)
    PolygonEnvironment(*worst_environment("2"))

    print(
        f"{'size':>6} {'n':>7} {'m':>7} {'total s':>9} {'wedges s':>9} {'graph s':>9}"
        f" {'growth':>7} {'candidates/origin':>18} {'edges/origin':>13}"
    )
    previous = None
    for size in args.sizes:
        boundary, holes = load(size)
        env, times = timed_preparation(boundary, holes)
        counts = counted_preparation(boundary, holes)
        n, m = len(env.coords), len(env.extremity_indices)
        origins = max(counts["origins"], 1)
        growth = ""
        if previous is not None and n != previous[0]:
            exponent = np.log(times["graph"] / previous[1]) / np.log(n / previous[0])
            growth = f"n^{exponent:.1f}"
        previous = n, times["graph"]
        print(
            f"{size:>6} {n:>7} {m:>7} {times['total']:>9.2f} {times['wedges']:>9.2f} {times['graph']:>9.2f}"
            f" {growth:>7} {counts['candidates'] / origins:>18.1f} {counts['edges'] / origins:>13.1f}"
        )


if __name__ == "__main__":
    main()
