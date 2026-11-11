"""Demo image of a shortest path on a simplified world map (land masses as obstacles, the sea is free space).

Standalone: generates ``docs/_static/demo_world.svg`` with the actual result of the package,
drawn with ``extremitypathfinder.plotting.draw()``. The map is the Natural Earth land data of the benchmarks
(``benchmarks/world_map.py``: downloaded once, checksum verified), simplified to a few thousand vertices
so that the visibility graph stays sparse and readable.
Coordinates are longitude/latitude degrees used as planar coordinates (paths do not cross the antimeridian).
The export is deterministic and has the dark mode colors of ``scripts/docs_figures.py``.

The start and goal lie in the sea of the default map only (other vertex counts may need other points).

usage: uv run --group benchmark python scripts/world_demo.py [-n VERTICES] [-o OUTPUT]
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from docs_figures import OUTPUT_DIR, dark_mode_css, export_svg

from benchmarks import world_map
from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlotStyle, draw

# (longitude, latitude), both in the sea: from the North Atlantic around Africa to the Tasman Sea
START = (-30.0, 40.0)
GOAL = (150.0, -40.0)
DEFAULT_NR_VERTICES = 2000
# thin lines and small markers for the large extent
STYLE = PlotStyle(
    graph_alpha=0.06,
    marker_size=1.5,
    line_width=0.4,
    path_width=2.2,
    outside_color=None,
)
# resolution of the embedded visibility graph image
RASTER_DPI = 100
# Antarctica is cropped
LAT_RANGE = (-65.0, 85.0)


def demo(output: Path, nr_vertices: int = DEFAULT_NR_VERTICES):
    world = world_map.build_map(target_vertices=nr_vertices)
    holes = [np.array(hole) for hole in world["holes"]]
    environment = PolygonEnvironment(None, holes, validate=True)
    for name, point in (("start", START), ("goal", GOAL)):
        if not environment.within_map(np.array(point)):
            raise SystemExit(
                f"the {name} {point} is on land of this map: choose another number of vertices"
            )
    path, length = environment.find_shortest_path(START, GOAL)
    print(
        f"world demo: path with {len(path)} points, length {length:.2f} degrees,"
        f" {environment.nr_vertices} vertices, {len(environment.extremity_indices)} extremities,"
        f" {environment.graph.number_of_edges()} graph edges"
    )
    fig, ax = plt.subplots(figsize=(10.0, 5.0))
    draw(environment, path, ax=ax, style=STYLE)
    # thousands of edges as vector paths would make the file megabytes large: embed the graph as image
    for artist in ax.get_children():
        if artist.get_gid() == "visibility-graph":
            artist.set_rasterized(True)
    ax.set_axis_off()
    ax.set_xlim(-180.0, 180.0)
    ax.set_ylim(*LAT_RANGE)
    with plt.rc_context({"savefig.dpi": RASTER_DPI}):
        export_svg(fig, output, dark_mode_css())


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("-n", "--vertices", type=int, default=DEFAULT_NR_VERTICES)
    parser.add_argument(
        "-o", "--output", type=Path, default=OUTPUT_DIR / "demo_world.svg"
    )
    args = parser.parse_args(argv)
    demo(args.output, args.vertices)


if __name__ == "__main__":
    main()
