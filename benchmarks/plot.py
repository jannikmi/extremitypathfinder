"""SVG figures of the benchmark results (docs/_static/benchmark_*.svg).

Only ``ok`` data points are plotted: invalid, crashed and over budget runs are left out
(no markers, no extrapolation). Figures of a ``--quick`` run carry a placeholder watermark.

Usage::

    uv run --group benchmark python -m benchmarks.plot benchmarks/results/benchmark.json
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, LogLocator, NullFormatter

from benchmarks import measure, workloads
from benchmarks.adapters import PACKAGES

matplotlib.use("Agg")

# fixed colors per package (categorical slots 1 and 2), markers as secondary encoding
STYLE = {
    "extremitypathfinder": {"color": "#2a78d6", "marker": "o"},
    "pyvisgraph": {"color": "#eb6834", "marker": "s"},
}
TEXT_SECONDARY = "#52514e"
GRID = "#e4e3df"
LAND = "#d6d4cc"
FIGSIZE = (6.4, 4.0)
# deterministic SVG output
plt.rcParams.update(
    {
        "svg.hashsalt": "extremitypathfinder-benchmark",
        "svg.fonttype": "path",
        "font.size": 10,
        "axes.edgecolor": TEXT_SECONDARY,
        "axes.labelcolor": "#0b0b0b",
        "xtick.color": TEXT_SECONDARY,
        "ytick.color": TEXT_SECONDARY,
        "axes.spines.top": False,
        "axes.spines.right": False,
    }
)
METADATA = {"Date": None, "Creator": None}


def _ok_points(results: dict, package: str, value):
    xs, ys = [], []
    for entry in results["world"]:
        result = entry["results"].get(package)
        if result is None or result["status"] != measure.OK:
            continue
        xs.append(entry["map"]["vertices"])
        ys.append(value(result))
    return xs, ys


def _new_axes(results: dict, ylabel: str):
    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("vertices of the map")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="major", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    return fig, ax


def _finish(fig, ax, results: dict, path: Path, legend: bool = True):
    meta = results["meta"]
    system = meta["system"]
    caption = (
        f"{system['cpu_model']}, Python {system['python']}, "
        f"extremitypathfinder {meta['versions'].get('extremitypathfinder')} ({_numba(results)}), "
        f"pyvisgraph {meta['versions'].get('pyvisgraph')}"
    )
    fig.text(0.01, 0.01, caption, fontsize=7, color=TEXT_SECONDARY, ha="left")
    if meta["quick"]:
        fig.text(
            0.5,
            0.5,
            "PLACEHOLDER\n--quick run, not representative",
            fontsize=20,
            color="#e34948",
            alpha=0.35,
            ha="center",
            va="center",
            rotation=20,
        )
    if legend and ax.get_legend_handles_labels()[0]:
        ax.legend(frameon=False, loc="best")
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, format="svg", metadata=METADATA)
    plt.close(fig)
    # without trailing whitespace (pre-commit hook), like scripts/docs_figures.py
    svg = path.read_text(encoding="utf-8")
    path.write_text(
        "\n".join(line.rstrip() for line in svg.splitlines()) + "\n", encoding="utf-8"
    )


def _line(ax, xs, ys, package):
    style = STYLE[package]
    ax.plot(
        xs,
        ys,
        color=style["color"],
        marker=style["marker"],
        markersize=6,
        linewidth=2,
        label=package,
    )
    if xs:
        # direct label at the last point
        ax.annotate(
            package,
            (xs[-1], ys[-1]),
            xytext=(6, 0),
            textcoords="offset points",
            va="center",
            fontsize=8,
            color=TEXT_SECONDARY,
        )


def build_time(results: dict, path: Path):
    fig, ax = _new_axes(results, "build time (s)")
    for package in PACKAGES:
        _line(ax, *_ok_points(results, package, lambda r: r["build_time"]), package)
    budget = results["meta"]["budget"]
    if budget:
        ax.axhline(budget, color=TEXT_SECONDARY, linestyle="--", linewidth=1)
        ax.annotate(
            f"budget {budget:g} s",
            (0.01, budget),
            xycoords=("axes fraction", "data"),
            xytext=(0, 3),
            textcoords="offset points",
            fontsize=8,
            color=TEXT_SECONDARY,
        )
    ax.set_title("Preprocessing (build) time")
    _finish(fig, ax, results, path)


def query_time(results: dict, path: Path):
    fig, ax = _new_axes(results, "query time (s)")
    for package in PACKAGES:
        xs, stats = _ok_points(results, package, lambda r: r["query_stats"])
        _line(ax, xs, [s["p50"] for s in stats], package)
        if xs:
            ax.fill_between(
                xs,
                [s["p5"] for s in stats],
                [s["p95"] for s in stats],
                color=STYLE[package]["color"],
                alpha=0.15,
                linewidth=0,
            )
    ax.set_title("Query latency: median, 5th–95th percentile band")
    _finish(fig, ax, results, path)


def memory(results: dict, path: Path):
    fig, ax = _new_axes(results, "peak memory above baseline (MiB)")
    for package in PACKAGES:
        xs, ys = _ok_points(results, package, lambda r: r["memory_peak"] / 2**20)
        # ru_maxrss has a coarse resolution: tiny maps may show 0
        points = [(x, y) for x, y in zip(xs, ys) if y > 0]
        _line(ax, [p[0] for p in points], [p[1] for p in points], package)
    ax.set_title("Peak memory of build and queries")
    _finish(fig, ax, results, path)


def ratio(results: dict, path: Path):
    fig, ax = _new_axes(results, "pyvisgraph / extremitypathfinder")
    xs, build, query = [], [], []
    for entry in results["world"]:
        epf = entry["results"].get("extremitypathfinder", {})
        pvg = entry["results"].get("pyvisgraph", {})
        if epf.get("status") != measure.OK or pvg.get("status") != measure.OK:
            continue
        xs.append(entry["map"]["vertices"])
        build.append(pvg["build_time"] / epf["build_time"])
        query.append(pvg["query_stats"]["p50"] / epf["query_stats"]["p50"])
    ax.plot(xs, build, color="#1baf7a", marker="o", linewidth=2, label="build time")
    ax.plot(xs, query, color="#6250d6", marker="D", linewidth=2, label="query (p50)")
    ax.axhline(1, color=TEXT_SECONDARY, linewidth=1)
    # factors as plain numbers ("6", not "6×10⁰"), only on the major ticks
    ax.yaxis.set_major_locator(LogLocator(subs=(1.0, 2.0, 5.0)))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.set_title("Speed ratio (sizes where both are valid)")
    _finish(fig, ax, results, path)


def _numba(results: dict) -> str:
    """the execution backend of extremitypathfinder in the results"""
    modes = {
        r.get("numba")
        for e in results["world"] + results["examples"]
        for package, r in e["results"].items()
        if package == "extremitypathfinder" and "numba" in r
    }
    if modes == {True}:
        return "Numba"
    if modes == {False}:
        return "without Numba"
    return "Numba unknown" if not modes else "with and without Numba"


def world_example(results: dict, path: Path):
    """the smallest world map with the paths of both packages for one query.

    Shows an invalid path when one occurred, otherwise the first query.
    """
    if not results["world"]:
        # replace a stale figure of earlier results
        fig, ax = plt.subplots(figsize=(8.0, 4.4))
        ax.axis("off")
        ax.text(0.5, 0.5, "no world map results", ha="center", va="center")
        _finish(fig, ax, results, path, legend=False)
        return
    entry, query_idx = results["world"][0], 0
    for candidate in results["world"]:
        failures = [
            f
            for r in candidate["results"].values()
            for f in r.get("validation", {}).get("failures", [])
        ]
        if failures:
            entry, query_idx = candidate, failures[0]["query"]
            break
    spec = workloads.MapSpec(**entry["map"]["spec"])
    data = workloads.load_world(spec)
    start, goal = results["world_queries"][query_idx]

    fig, ax = plt.subplots(figsize=(8.0, 4.4))
    for hole in data["holes"]:
        xs, ys = zip(*hole)
        ax.fill(xs, ys, color=LAND, linewidth=0)
    for package in PACKAGES:
        result = entry["results"].get(package, {})
        failure = next(
            (
                f
                for f in result.get("validation", {}).get("failures", [])
                if f["query"] == query_idx
            ),
            None,
        )
        path_coords = result.get("example_paths", {}).get(str(query_idx))
        if not path_coords:
            continue
        xs, ys = zip(*path_coords)
        label = package + (" (invalid)" if failure else "")
        ax.plot(
            xs,
            ys,
            color=STYLE[package]["color"],
            linewidth=2,
            linestyle="--" if failure else "-",
            label=label,
        )
    ax.plot(*zip(start, goal), "o", color="#0b0b0b", markersize=5, linestyle="none")
    ax.set_aspect("equal")
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xlabel("longitude (°)")
    ax.set_ylabel("latitude (°)")
    ax.set_title(
        f"{entry['map']['name']} ({entry['map']['vertices']:,} vertices), query {query_idx}"
    )
    _finish(fig, ax, results, path)


FIGURES = {
    "benchmark_build_time.svg": build_time,
    "benchmark_query_time.svg": query_time,
    "benchmark_memory.svg": memory,
    "benchmark_ratio.svg": ratio,
    "benchmark_world_example.svg": world_example,
}


def write_figures(results: dict, directory: Path):
    directory.mkdir(parents=True, exist_ok=True)
    for name, function in FIGURES.items():
        function(results, directory / name)
    print(f"figures written to {directory}", file=sys.stderr)


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("raw", type=Path, help="raw results JSON of benchmarks.run")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "_static",
    )
    args = parser.parse_args(argv)
    with open(args.raw) as f:
        results = json.load(f)
    write_figures(results, args.output_dir)


if __name__ == "__main__":
    main()
