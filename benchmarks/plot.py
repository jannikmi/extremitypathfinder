"""SVG figures of the benchmark results (docs/_static/benchmark_*.svg).

Query times are plotted for ``ok`` runs only. Build time, memory and the build ratio also include
invalid runs: their build is complete, only their paths are wrong.
Crashed and over budget runs are left out (no markers, no extrapolation).
Figures of a ``--quick`` run carry a placeholder watermark.

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
from benchmarks.adapters import ADAPTERS, COMPARED_PACKAGES, NO_NUMBA, PACKAGES

matplotlib.use("Agg")

# fixed colors per package (categorical slots 1 to 3), markers as secondary encoding
STYLE = {
    "extremitypathfinder": {"color": "#2a78d6", "marker": "o"},
    "pyvisgraph": {"color": "#eb6834", "marker": "s"},
    NO_NUMBA: {"color": "#1baf7a", "marker": "^"},
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


BUILT = (measure.OK, measure.INVALID)


def _ok_points(results: dict, package: str, value, statuses=(measure.OK,)):
    xs, ys = [], []
    for entry in results["world"]:
        result = entry["results"].get(package)
        if result is None or result["status"] not in statuses:
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


def _finish(fig, ax, results: dict, path: Path, legend: bool = True, **legend_kwargs):
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
        ax.legend(frameon=False, **({"loc": "best"} | legend_kwargs))
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(path, format="svg", metadata=METADATA)
    plt.close(fig)
    # without trailing whitespace (pre-commit hook), like scripts/docs_figures.py
    svg = path.read_text(encoding="utf-8")
    path.write_text(
        "\n".join(line.rstrip() for line in svg.splitlines()) + "\n", encoding="utf-8"
    )


def _line(ax, xs, ys, package, label=None, linestyle="-"):
    style = STYLE[package]
    name = ADAPTERS[package].label
    ax.plot(
        xs,
        ys,
        color=style["color"],
        marker=style["marker"],
        markersize=6,
        linewidth=2,
        linestyle=linestyle,
        markerfacecolor=style["color"] if linestyle == "-" else "none",
        label=label or name,
    )
    if xs and label is None:
        # direct label at the last point
        ax.annotate(
            # short: a long label takes the width from the axes
            name.removeprefix("extremitypathfinder "),
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
        _line(
            ax, *_ok_points(results, package, lambda r: r["build_time"], BUILT), package
        )
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
    _finish(fig, ax, results, path, loc="lower right", fontsize=8)


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
    _finish(fig, ax, results, path, loc="lower right", fontsize=8)


def memory(results: dict, path: Path):
    fig, ax = _new_axes(results, "peak memory (MiB)")
    for package in COMPARED_PACKAGES:
        xs, ys = _ok_points(results, package, lambda r: r["memory_peak"] / 2**20, BUILT)
        # ru_maxrss has a coarse resolution: tiny maps may show 0
        points = [(x, y) for x, y in zip(xs, ys) if y > 0]
        _line(ax, [p[0] for p in points], [p[1] for p in points], package)
        # the total: including the baseline of the process (imports, warm-up, input)
        xs, ys = _ok_points(
            results,
            package,
            lambda r: (
                None
                if r.get("memory_baseline") is None
                else (r["memory_baseline"] + r["memory_peak"]) / 2**20
            ),
            BUILT,
        )
        points = [(x, y) for x, y in zip(xs, ys) if y is not None]
        _line(
            ax,
            [p[0] for p in points],
            [p[1] for p in points],
            package,
            label=f"{package}, total incl. imports",
            linestyle="--",
        )
    ax.set_title("Peak memory: above the baseline (solid), total (dashed)")
    # below the axes: no free corner for four entries
    _finish(
        fig,
        ax,
        results,
        path,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.17),
        ncol=2,
        fontsize=8,
    )


def memory_retained(results: dict, path: Path):
    """what is left of the peak after the build: the temporary data is the difference"""
    fig, ax = _new_axes(results, "memory above the baseline (MiB)")
    for package in COMPARED_PACKAGES:
        name = ADAPTERS[package].label
        for key, what, linestyle in (
            ("memory_peak", "peak", "-"),
            ("memory_retained", "retained after the build", "--"),
        ):
            xs, ys = _ok_points(
                results, package, lambda r, key=key: (r.get(key) or 0) / 2**20, BUILT
            )
            points = [(x, y) for x, y in zip(xs, ys) if y > 0]
            _line(
                ax,
                [p[0] for p in points],
                [p[1] for p in points],
                package,
                label=f"{name}, {what}",
                linestyle=linestyle,
            )
    ax.set_title("Memory above the baseline: peak (solid), retained (dashed)")
    _finish(
        fig,
        ax,
        results,
        path,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.17),
        ncol=2,
        fontsize=8,
    )


def ratio(results: dict, path: Path):
    _ratio_figure(
        results,
        path,
        "pyvisgraph",
        "extremitypathfinder",
        "pyvisgraph / extremitypathfinder",
        "Speed ratio (query: sizes where both are valid)",
    )


def numba_ratio(results: dict, path: Path):
    _ratio_figure(
        results,
        path,
        NO_NUMBA,
        "extremitypathfinder",
        "without Numba / with Numba",
        "Speed-up of extremitypathfinder by Numba",
    )


def _ratio_figure(results, path, slower, faster, ylabel, title):
    fig, ax = _new_axes(results, ylabel)
    # build: both built (also with invalid paths), query: both valid
    xs_build, build, xs_query, query = [], [], [], []
    for entry in results["world"]:
        epf = entry["results"].get(faster, {})
        pvg = entry["results"].get(slower, {})
        vertices = entry["map"]["vertices"]
        if epf.get("status") in BUILT and pvg.get("status") in BUILT:
            xs_build.append(vertices)
            build.append(pvg["build_time"] / epf["build_time"])
        if epf.get("status") == measure.OK and pvg.get("status") == measure.OK:
            xs_query.append(vertices)
            query.append(pvg["query_stats"]["p50"] / epf["query_stats"]["p50"])
    ax.plot(
        xs_build, build, color="#1baf7a", marker="o", linewidth=2, label="build time"
    )
    ax.plot(
        xs_query, query, color="#6250d6", marker="D", linewidth=2, label="query (p50)"
    )
    ax.axhline(1, color=TEXT_SECONDARY, linewidth=1)
    # factors as plain numbers ("6", not "6×10⁰"), only on the major ticks
    ax.yaxis.set_major_locator(LogLocator(subs=(1.0, 2.0, 5.0)))
    ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.set_title(title)
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
    for package in COMPARED_PACKAGES:
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


NUMBA_COLOR = "#6250d6"
BASE_COLOR = "#b5b3ab"
TEMPORARY_COLOR = "#e4e3df"


def numba_share(results: dict) -> int | None:
    """the memory of Numba in extremitypathfinder's baseline (imports and compiled code)

    The difference of the baselines (without input data) with and without Numba.
    None if they are not measured, or if the share is larger than a baseline of the run
    (measured under other conditions, e.g. on another machine).
    """
    baselines = {}
    for setup in results.get("baselines", {}).get("setups", []):
        if setup["package"] == "extremitypathfinder" and setup.get("status") == "done":
            baselines[setup["numba"]] = setup["memory_baseline"]
    if True not in baselines or False not in baselines:
        return None
    share = baselines[True] - baselines[False]
    for entry in _built(results):
        if entry["results"]["extremitypathfinder"]["memory_baseline"] < share:
            return None
    return share


def _built(results: dict) -> list[dict]:
    """the world maps on which extremitypathfinder completed the build"""
    return [
        entry
        for entry in results["world"]
        if entry["results"].get("extremitypathfinder", {}).get("status") in BUILT
    ]


def _data_segments(result: dict, package: str) -> list[tuple[float, str, str]]:
    """the peak above the baseline as bar segments: retained after the build, and the rest"""
    peak = result["memory_peak"]
    # in allocated bytes, the peak in pages of the process: possibly above the peak or negative on tiny maps
    retained = max(0, min(result.get("memory_retained") or 0, peak))
    return [
        (
            retained / 2**20,
            STYLE[package]["color"],
            f"retained data: {package}",
        ),
        (
            (peak - retained) / 2**20,
            TEMPORARY_COLOR,
            "temporary data, allocator overhead",
        ),
    ]


def memory_composition(results: dict, path: Path):
    """stacked bars per map size: baseline without Numba, Numba, retained and temporary data
    (extremitypathfinder) | baseline, retained and temporary data (pyvisgraph)"""
    share = numba_share(results)
    built = _built(results)
    if share is None:
        fig, ax = plt.subplots(figsize=FIGSIZE)
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "The baselines with and without Numba are missing or do not match the run\n"
            "(e.g. measured on another machine). On the machine of the run:\n"
            "python -m benchmarks.run --baselines benchmarks/results/benchmark.json",
            ha="center",
            va="center",
            color=TEXT_SECONDARY,
        )
        _finish(fig, ax, results, path, legend=False)
        return
    fig, ax = _new_axes(results, "peak memory (MiB)")
    ax.set_xscale("linear")
    ax.set_yscale("linear")
    ax.grid(True, axis="y", color=GRID, linewidth=0.8)
    ax.grid(False, axis="x")
    mib = 2**20
    width = 0.38
    labels, centers = [], []
    seen = set()

    def bar(x, bottom, height, color, label):
        # one legend entry per segment kind
        ax.bar(
            x,
            height,
            width,
            bottom=bottom,
            color=color,
            label=None if label in seen else label,
        )
        seen.add(label)

    for i, entry in enumerate(built):
        epf = entry["results"]["extremitypathfinder"]
        labels.append(f"{entry['map']['vertices']:,}")
        centers.append(i)
        base = (epf["memory_baseline"] - share) / mib
        segments = [
            (base, BASE_COLOR, "interpreter and imports without Numba"),
            (share / mib, NUMBA_COLOR, "Numba (imports and compiled functions)"),
            *_data_segments(epf, "extremitypathfinder"),
        ]
        bottom = 0.0
        for height, color, label in segments:
            bar(i - width / 2, bottom, height, color, label)
            bottom += height
        ax.annotate(
            f"{bottom:.0f}",
            (i - width / 2, bottom),
            xytext=(0, 2),
            textcoords="offset points",
            ha="center",
            fontsize=7,
            color=TEXT_SECONDARY,
        )
        pvg = entry["results"].get("pyvisgraph", {})
        if pvg.get("status") in BUILT:
            bottom = 0.0
            for height, color, label in (
                (
                    pvg["memory_baseline"] / mib,
                    BASE_COLOR,
                    "interpreter and imports without Numba",
                ),
                *_data_segments(pvg, "pyvisgraph"),
            ):
                bar(i + width / 2, bottom, height, color, label)
                bottom += height
            ax.annotate(
                f"{bottom:.0f}",
                (i + width / 2, bottom),
                xytext=(0, 2),
                textcoords="offset points",
                ha="center",
                fontsize=7,
                color=TEXT_SECONDARY,
            )
    ax.set_xticks(centers, labels)
    ax.set_title("Peak memory: extremitypathfinder (left), pyvisgraph (right)")
    _finish(
        fig,
        ax,
        results,
        path,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.17),
        ncol=2,
        fontsize=8,
    )


FIGURES = {
    "benchmark_build_time.svg": build_time,
    "benchmark_query_time.svg": query_time,
    "benchmark_memory.svg": memory,
    "benchmark_memory_retained.svg": memory_retained,
    "benchmark_memory_composition.svg": memory_composition,
    "benchmark_ratio.svg": ratio,
    "benchmark_numba_ratio.svg": numba_ratio,
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
