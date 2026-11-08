"""overhead of an angular sweep (visibility polygon) compared with AREA (#216)

An angular sweep keeps the edges crossed by the rotating ray ("active edges") in a sorted structure,
ordered by their distance along the ray. Per origin and edge it inserts and removes the edge once and reads
the closest active edge. This script measures

* the time of one sweep step (insert, read the closest, remove) for a growing number of active edges, with
  ``sortedcontainers.SortedList`` (a balanced structure, the only stable candidate on PyPI) and with
  a plain ``list`` and the standard library ``bisect`` (no dependency, insertion moves the list in memory).
  The edges compare by their distance along the current ray, which keeps a valid order for edges that do not cross.
* the time AREA currently takes per origin and edge when the visibility graph is computed
  (``compute_graph()`` divided by the origins times the edges), on the grid environment of
  ``complexity_profile.py``. A sweep needs about two sweep steps per origin and edge.

The findings and the decision are recorded at ``find_visible()`` in ``extremitypathfinder/utils.py``.

usage:
    uv run --with sortedcontainers python prototypes/sweep_tree_overhead.py
    uv run --with sortedcontainers python prototypes/sweep_tree_overhead.py --holes-per-side 16
    NUMBA_DISABLE_JIT=1 uv run --with sortedcontainers python prototypes/sweep_tree_overhead.py  # without Numba
"""

import argparse
import bisect
import math
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

# the current direction of the sweep ray, shared by all edges like the state of a sweep
RAY = (1.0, 0.0)


class Edge:
    __slots__ = ("x1", "x2", "y1", "y2")

    def __init__(self, x1, y1, x2, y2):
        self.x1, self.y1, self.x2, self.y2 = x1, y1, x2, y2

    def distance(self) -> float:
        """the distance from the origin along the current ray to the edge (ray and segment intersection)"""
        dx, dy = RAY
        ex, ey = self.x2 - self.x1, self.y2 - self.y1
        denominator = dx * ey - dy * ex
        return (self.x1 * ey - self.y1 * ex) / denominator if denominator else math.inf

    def __lt__(self, other) -> bool:
        return self.distance() < other.distance()


def shells(radii) -> list[Edge]:
    """edges that do not cross, all crossed by the ray, at the given distances"""
    return [Edge(r, -1.0, r, 1.0) for r in radii]


def sweep_step_time(make, insert, remove, nr_active: int, repetitions: int) -> float:
    """the time of one sweep step with ``nr_active`` active edges in µs"""
    radii = random.sample(range(1, 10 * nr_active), nr_active)
    active = make()
    for edge in shells(radii):
        insert(active, edge)
    # distinct from the active edges (no ties)
    edges = shells(r + 0.5 for r in random.choices(radii, k=repetitions))
    start = time.perf_counter()
    for edge in edges:
        insert(active, edge)
        active[0]  # the closest edge
        remove(active, edge)
    return (time.perf_counter() - start) / repetitions * 1e6


def sorted_list_step(nr_active: int, repetitions: int) -> float:
    from sortedcontainers import SortedList

    return sweep_step_time(
        SortedList, SortedList.add, SortedList.remove, nr_active, repetitions
    )


def bisect_list_step(nr_active: int, repetitions: int) -> float:
    def remove(active, edge):
        del active[bisect.bisect_left(active, edge)]

    return sweep_step_time(list, bisect.insort, remove, nr_active, repetitions)


def area_time_per_origin_and_edge(holes_per_side: int) -> tuple[int, int, float]:
    """the vertices, extremities and the time of AREA per origin and edge in µs"""
    from complexity_profile import grid_environment, timed_preparation

    env, times = timed_preparation(*grid_environment(str(holes_per_side)))
    n, m = len(env.coords), len(env.extremity_indices)
    # every extremity is an origin, every polygon has as many edges as vertices
    return n, m, times["graph"] / (m * n) * 1e6


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--holes-per-side", type=int, default=12)
    parser.add_argument("--repetitions", type=int, default=20_000)
    args = parser.parse_args()
    random.seed(1)

    print(f"{'active edges':>12} {'SortedList µs':>14} {'list+bisect µs':>15}")
    for nr_active in (10, 100, 1_000, 10_000, 100_000):
        sorted_list = sorted_list_step(nr_active, args.repetitions)
        bisect_list = bisect_list_step(nr_active, args.repetitions)
        print(f"{nr_active:>12} {sorted_list:>14.2f} {bisect_list:>15.2f}")

    a, b = shells((1.0, 2.0))
    start = time.perf_counter()
    for _ in range(200_000):
        a < b  # noqa: B015
    comparison = (time.perf_counter() - start) / 200_000 * 1e6
    print(f"one comparison of two edges: {comparison:.2f} µs")

    # warm up (Numba compilation)
    area_time_per_origin_and_edge(2)
    n, m, area = area_time_per_origin_and_edge(args.holes_per_side)
    print(
        f"AREA (grid, n={n}, m={m}): {area:.2f} µs per origin and edge,"
        f" a sweep needs about two sweep steps per origin and edge"
    )


if __name__ == "__main__":
    main()
