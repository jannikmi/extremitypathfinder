"""Speed of extremitypathfinder on the small example environments.

Thin wrapper around the benchmark harness (``benchmarks.measure``):
each example is built and queried in a subprocess after a warm-up (Numba compiled, if installed),
with seeded random queries in the free space. The comparison with pyvisgraph
on the world map size sweep is ``python -m benchmarks.run`` (``make benchmark``).

usage: uv run --all-extras --group benchmark python scripts/speed_benchmarks.py [nr_queries]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from benchmarks import measure, run, workloads

PACKAGE = "extremitypathfinder"


def main(nr_queries: int = 1000):
    for data in workloads.small_example_maps():
        queries = workloads.generate_queries([data], nr_queries, region=None)
        result = measure.measure(PACKAGE, data["holes"], queries, budget=None)
        if result["status"] != "done":
            print(f"{data['name']}: {result['status']} {result.get('error', '')}")
            continue
        stats = run.query_stats(result["query_times_ns"])
        print(
            f"{data['name']} ({data['vertices']} vertices, Numba: {result['numba']}): "
            f"build {run.format_time(result['build_time'])}, "
            f"query p50 {run.format_time(stats['p50'])}, "
            f"mean {run.format_time(stats['mean'])} "
            f"({stats['queries_per_s']:.3g} queries/s, {stats['n']} queries)"
        )


if __name__ == "__main__":
    main(*(int(a) for a in sys.argv[1:2]))
