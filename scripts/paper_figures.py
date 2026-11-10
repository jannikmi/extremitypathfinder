"""Generates the figures of the JOSS paper (paper/*.png).

usage: uv run --all-extras --group benchmark python scripts/paper_figures.py [output_dir]  (default: paper)

The figures reuse the drawings of the documentation: the graph reduction of ``scripts/docs_figures.py``
and the benchmark figures of ``benchmarks/plot.py``, from the committed results
(``benchmarks/results/benchmark.json``). They are PNG, because the PDF of the paper is built with LaTeX.
The exports are deterministic: regenerating them without changes yields identical files.
"""

import io
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[1]
# the benchmarks package (the directory of this script is on the path already)
sys.path.insert(0, str(REPO_ROOT))

from docs_figures import draw_graph_reduction

from benchmarks import measure, plot
from benchmarks.adapters import ADAPTERS, PACKAGES

OUTPUT_DIR = REPO_ROOT / "paper"
RESULTS = REPO_ROOT / "benchmarks" / "results" / "benchmark.json"
DPI = 250


def save(fig, path: Path):
    """saves the figure as PNG with a palette of 256 colours (flat colours: no visible loss, a third of the size)"""
    buffer = io.BytesIO()
    fig.savefig(buffer, dpi=DPI, format="png", bbox_inches="tight", pad_inches=0.03)
    plt.close(fig)
    image = (
        Image.open(buffer).convert("RGB").quantize(colors=256, dither=Image.Dither.NONE)
    )
    image.save(path, format="png", optimize=True)
    print(f"written: {path} ({path.stat().st_size / 1000:.0f} kB)")


def graph_reduction(output_dir: Path):
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 6.0))
    # the figure is smaller than in the documentation
    draw_graph_reduction(axes, marker_size=2.8, path_width=2.4)
    for ax in axes:
        ax.title.set_fontsize(9)
    save(fig, output_dir / "graph_reduction.png")


def _log_axes(ax, ylabel: str, title: str):
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("vertices of the map")
    ax.set_ylabel(ylabel)
    ax.set_title(title, fontsize=9)
    ax.grid(True, which="major", color=plot.GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def benchmark_sweep(results: dict, output_dir: Path):
    """build time, query time and graph size over the size of the world map, side by side"""
    fig, axes = plt.subplots(1, 3, figsize=(7.2, 2.75))
    plot.draw_build_time(axes[0], results, direct_labels=False)
    _log_axes(axes[0], "time (s)", "build (preprocessing)")
    plot.draw_query_time(axes[1], results, direct_labels=False)
    _log_axes(axes[1], "time (s)", "query: median, 5th–95th pct.")
    plot.draw_graph_size(axes[2], results, direct_labels=False)
    _log_axes(axes[2], "edges", "visibility graph size")
    for ax in axes:
        ax.tick_params(labelsize=8)
        ax.xaxis.label.set_fontsize(8)
        ax.yaxis.label.set_fontsize(8)
        if ax.get_legend():
            ax.get_legend().remove()
    # one legend for all panels
    handles = [
        Line2D(
            [],
            [],
            color=plot.STYLE[package]["color"],
            marker=plot.STYLE[package]["marker"],
            linewidth=2,
            markersize=5,
            label=ADAPTERS[package].label,
        )
        for package in PACKAGES
    ]
    handles.append(
        Line2D(
            [],
            [],
            linestyle="none",
            marker="o",
            markersize=5,
            markeredgewidth=1.5,
            color=plot.TEXT_SECONDARY,
            markerfacecolor="white",
            label="invalid paths",
        )
    )
    fig.legend(
        handles=handles,
        loc="upper center",
        ncol=len(handles),
        frameon=False,
        fontsize=8,
        bbox_to_anchor=(0.5, 1.07),
    )
    fig.tight_layout(w_pad=1.2)
    save(fig, output_dir / "benchmark_sweep.png")


def world_example(results: dict, output_dir: Path):
    entry, query_idx = plot.world_example_query(results)
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    plot.draw_world_example(ax, results, entry, query_idx)
    nr_invalid = entry["results"]["pyvisgraph"]["validation"]["failed"]
    nr_queries = entry["results"]["pyvisgraph"]["validation"]["checked"]
    ax.set_title(
        f"world map with {entry['map']['vertices']:,} vertices: one of {nr_invalid} invalid"
        f" pyvisgraph paths of {nr_queries} queries",
        fontsize=9,
    )
    ax.tick_params(labelsize=8)
    ax.legend(frameon=False, fontsize=8, loc="lower left")
    save(fig, output_dir / "benchmark_world_example.png")


def main(output_dir: Path = OUTPUT_DIR):
    output_dir.mkdir(parents=True, exist_ok=True)
    with open(RESULTS) as f:
        results = json.load(f)
    if results["meta"]["quick"]:
        raise SystemExit(f"{RESULTS} holds a --quick run: run the full benchmark first")
    graph_reduction(output_dir)
    benchmark_sweep(results, output_dir)
    world_example(results, output_dir)
    # the invalid runs of the sweep, for the caption
    invalid = [
        (entry["map"]["vertices"], package)
        for entry in results["world"]
        for package, result in entry["results"].items()
        if result["status"] == measure.INVALID
    ]
    print(f"invalid runs (vertices, package): {invalid}")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else OUTPUT_DIR)
