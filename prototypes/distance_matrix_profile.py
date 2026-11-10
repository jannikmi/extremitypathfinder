"""compare distance_matrix() with a loop over find_shortest_path() (#175)

environment: the square boundary with a grid of randomly shifted square holes of ``large_env_query_profile.py``
(4 vertices per hole). The points are random points of the free space.
Checks that both give identical lengths and prints the times of both.

usage:
    python -m prototypes.distance_matrix_profile --holes-per-side 5 9 --points 20 100
    python -m prototypes.distance_matrix_profile --no-numba
"""

import argparse
import importlib
import sys
import time

import numpy as np

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--holes-per-side", type=int, nargs="+", default=[5, 9])
parser.add_argument("--points", type=int, nargs="+", default=[20, 100])
parser.add_argument("--seed", type=int, default=0)
parser.add_argument(
    "--no-numba", action="store_true", help="run as if Numba was not installed"
)
args = parser.parse_args()
if args.no_numba:
    sys.modules["numba"] = None

# imported after blocking Numba (--no-numba)
PolygonEnvironment = importlib.import_module("extremitypathfinder").PolygonEnvironment
build_environment = importlib.import_module(
    "prototypes.large_env_query_profile"
).build_environment


def random_points(environment, n: int, size: float, rng):
    points = []
    while len(points) < n:
        point = (float(rng.uniform(0.0, size)), float(rng.uniform(0.0, size)))
        if environment.within_map(np.array(point)):
            points.append(point)
    return points


def loop(environment, points) -> np.ndarray:
    matrix = np.full((len(points), len(points)), np.inf)
    for i, a in enumerate(points):
        for j, b in enumerate(points):
            length = environment.find_shortest_path(a, b)[1]
            if length is not None:
                matrix[i, j] = length
    return matrix


def main():
    print("| vertices | extremities | points | loop | distance_matrix() | speed-up |")
    print("|---:|---:|---:|---:|---:|---:|")
    for holes_per_side in args.holes_per_side:
        rng = np.random.default_rng(args.seed)
        boundary, holes, size = build_environment(holes_per_side, rng)
        environment = PolygonEnvironment(boundary, holes)
        # warm-up (Numba compilation)
        environment.distance_matrix(random_points(environment, 3, size, rng))
        loop(environment, random_points(environment, 3, size, rng))
        for n in args.points:
            points = random_points(environment, n, size, rng)
            t0 = time.perf_counter()
            expected = loop(environment, points)
            t_loop = time.perf_counter() - t0
            t0 = time.perf_counter()
            matrix = environment.distance_matrix(points)
            t_matrix = time.perf_counter() - t0
            assert np.array_equal(matrix, expected), "the lengths differ from the loop"
            print(
                f"| {environment.nr_vertices} | {len(environment.extremity_indices)} | {n} "
                f"| {t_loop * 1e3:,.0f} ms | {t_matrix * 1e3:,.0f} ms | {t_loop / t_matrix:.1f}x |"
            )


if __name__ == "__main__":
    main()
