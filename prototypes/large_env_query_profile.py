"""profile path queries on a large polygon environment

measures how much of the query time is spent copying the visibility graph
(https://github.com/jannikmi/extremitypathfinder/issues/136)
and optionally lists the most expensive functions (cProfile)

environment: square boundary with a regular grid of randomly shifted square holes.
the number of extremities grows with the square of ``--holes-per-side`` (4 per hole),
the preparation time grows even faster: 25 holes per side take about a minute

usage:
    python prototypes/large_env_query_profile.py --holes-per-side 10 --profile
"""

import argparse
import cProfile
import pstats
import time

import numpy as np

from extremitypathfinder import PolygonEnvironment

HOLE_SPACING = 4.0
HOLE_SIZE = 1.0


def build_environment(holes_per_side: int, rng: np.random.Generator):
    holes = []
    for i in range(holes_per_side):
        for j in range(holes_per_side):
            x = HOLE_SPACING * i + 2.0 + rng.uniform(0.0, 1.0)
            y = HOLE_SPACING * j + 2.0 + rng.uniform(0.0, 1.0)
            # clockwise numbering
            holes.append(
                [
                    (x, y),
                    (x, y + HOLE_SIZE),
                    (x + HOLE_SIZE, y + HOLE_SIZE),
                    (x + HOLE_SIZE, y),
                ]
            )
    size = HOLE_SPACING * holes_per_side + 2.0
    boundary = [(0.0, 0.0), (size, 0.0), (size, size), (0.0, size)]
    return boundary, holes, size


def random_queries(nr_queries: int, size: float, rng: np.random.Generator):
    # from the left to the right border: paths cross the whole environment
    return [
        (
            (rng.uniform(0.0, 1.0), rng.uniform(0.0, size)),
            (rng.uniform(size - 1.0, size), rng.uniform(0.0, size)),
        )
        for _ in range(nr_queries)
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--holes-per-side", type=int, default=10)
    parser.add_argument("--queries", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--profile",
        action="store_true",
        help="list the most expensive functions of the queries",
    )
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    boundary, holes, size = build_environment(args.holes_per_side, rng)

    t_start = time.perf_counter()
    env = PolygonEnvironment(boundary, holes, validate=True)
    t_prep = time.perf_counter() - t_start
    graph = env.graph
    print(
        f"{len(holes)} holes, {len(env.extremity_indices)} extremities, "
        f"graph: {graph.number_of_nodes()} nodes, {graph.number_of_edges()} edges"
    )
    print(f"preparation: {t_prep:.2f} s")

    queries = random_queries(args.queries, size, rng)
    t_start = time.perf_counter()
    for start, goal in queries:
        env.find_shortest_path(start, goal)
    t_query = (time.perf_counter() - t_start) / args.queries

    t_start = time.perf_counter()
    for _ in range(args.queries):
        graph.copy()
    t_copy = (time.perf_counter() - t_start) / args.queries

    print(f"query (avg. over {args.queries}): {t_query * 1e3:.1f} ms")
    print(
        f"graph.copy(): {t_copy * 1e3:.1f} ms ({100 * t_copy / t_query:.0f}% of a query)"
    )

    if args.profile:
        # separate run: the profiler overhead would distort the timings above
        profiler = cProfile.Profile()
        profiler.enable()
        for start, goal in queries:
            env.find_shortest_path(start, goal)
        profiler.disable()
        print()
        pstats.Stats(profiler).sort_stats("cumulative").print_stats(15)


if __name__ == "__main__":
    main()
