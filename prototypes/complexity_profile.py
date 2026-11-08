"""measure how the preparation time of an environment grows and where it goes

For each environment size it reports

* the time of the two steps of the preparation: points and wedges (``compile_polygon_datastructs()``)
  and the visibility graph (``compute_graph()``)
* the number of edges ``find_visible()`` receives per origin
* the number of checks per candidate: how often AREA compares a candidate with an edge whose angle range contains it.
  It stays constant in the typical case and grows with the number of vertices in the worst case
  (cf. the cost of AREA in the engineering design)

The counts come from a second preparation with an instrumented ``check_candidates_one_edge()``,
so that the counting does not distort the timings.

environments:

* ``grid``: the square boundary with randomly shifted square holes of ``large_env_query_profile.py``
  (sizes are holes per side)
* ``world``: the cleaned world maps of the benchmarks (sizes as in ``python -m benchmarks.run --sizes``,
  needs the ``benchmark`` dependency group and downloads the Natural Earth data once)

usage:
    python prototypes/complexity_profile.py grid 8 16 24
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
        env = PolygonEnvironment(boundary, holes)
        times["total"] = time.perf_counter() - start
    finally:
        utils.compile_polygon_datastructs, utils.compute_graph = original
    return env, times


def counted_preparation(boundary, holes) -> dict:
    counts = {"origins": 0, "candidates": 0, "edges": 0, "checks": 0}
    original_find, original_one_edge = (
        utils.find_visible,
        utils.check_candidates_one_edge,
    )

    def find_visible(origin, candidates, edges_to_check, *args, **kwargs):
        counts["origins"] += 1
        counts["candidates"] += len(candidates)
        counts["edges"] += len(edges_to_check)
        return original_find(origin, candidates, edges_to_check, *args, **kwargs)

    def check_candidates_one_edge(edge_min_rep, edge_max_rep, *args):
        candidate_ptr, candidate_indices, representations = args[3], args[5], args[8]
        for candidate_idx in candidate_indices[candidate_ptr:]:
            representation = representations[candidate_idx]
            if representation > edge_max_rep:
                break
            if representation >= edge_min_rep:
                counts["checks"] += 1
        return original_one_edge(edge_min_rep, edge_max_rep, *args)

    utils.find_visible = find_visible
    utils.check_candidates_one_edge = check_candidates_one_edge
    try:
        PolygonEnvironment(boundary, holes)
    finally:
        utils.find_visible, utils.check_candidates_one_edge = (
            original_find,
            original_one_edge,
        )
    return counts


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("environment", choices=["grid", "world"])
    parser.add_argument("sizes", nargs="+")
    args = parser.parse_args()
    load = grid_environment if args.environment == "grid" else world_environment

    print(
        f"{'size':>6} {'n':>7} {'m':>7} {'total s':>9} {'wedges s':>9} {'graph s':>9}"
        f" {'edges/origin':>13} {'checks/candidate':>17}"
    )
    for size in args.sizes:
        boundary, holes = load(size)
        env, times = timed_preparation(boundary, holes)
        counts = counted_preparation(boundary, holes)
        n, m = len(env.coords), len(env.extremity_indices)
        edges = counts["edges"] / max(counts["origins"], 1)
        checks = counts["checks"] / max(counts["candidates"], 1)
        print(
            f"{size:>6} {n:>7} {m:>7} {times['total']:>9.2f} {times['wedges']:>9.2f} {times['graph']:>9.2f}"
            f" {edges:>13.1f} {checks:>17.2f}"
        )


if __name__ == "__main__":
    main()
