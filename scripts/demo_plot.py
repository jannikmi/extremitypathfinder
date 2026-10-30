"""Generates the demo visualisation shown in the README and the documentation.

usage: python scripts/demo_plot.py [output.svg]

The image is drawn with ``extremitypathfinder.plotting.draw()``,
the path is the actual result of the package.
The export is deterministic: regenerating it without changes yields an identical file.
Colors are swapped for dark backgrounds with an embedded CSS media query (``prefers-color-scheme``).
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlotStyle, draw

OUTPUT = Path(__file__).resolve().parents[1] / "docs" / "_static" / "demo.svg"

BOUNDARY = [(0.0, 0.0), (50.0, 0.0), (50.0, 20.0), (0.0, 20.0)]
HOLES = [
    # decoration
    [(3.0, 14.0), (9.0, 17.5), (12.0, 15.5), (7.0, 13.0)],
    [(19.0, 15.5), (20.0, 18.5), (24.0, 18.0), (23.0, 15.0)],
    [(29.0, 1.5), (30.5, 3.5), (34.0, 3.0), (33.5, 1.0)],
    [(42.0, 1.5), (43.5, 4.0), (46.5, 2.5)],
    # near the start
    [(5.0, 4.0), (8.0, 10.0), (11.0, 3.5)],
    # an L-shaped wall rising from the boundary
    [(15.0, 0.0), (15.0, 12.0), (17.0, 12.0), (17.0, 3.0), (21.0, 3.0), (21.0, 0.0)],
    # two squares touching at a corner, right where the path would like to pass
    [(24.0, 5.0), (24.0, 9.0), (28.0, 9.0), (28.0, 5.0)],
    [(28.0, 9.0), (28.0, 13.0), (32.0, 13.0), (32.0, 9.0)],
    # a wall hanging from the boundary
    [(37.0, 6.0), (37.0, 20.0), (38.2, 20.0), (38.2, 6.0)],
    # in front of the goal
    [(41.0, 9.0), (43.0, 12.5), (45.5, 9.5)],
]
START = (2.0, 2.5)
GOAL = (47.0, 15.5)

LIGHT = PlotStyle(graph_alpha=0.15, marker_size=3.5)
# the same roles, stepped for dark backgrounds
DARK = {
    "boundary": "#6b6a65",
    "obstacles": "#4a4a46",
    "visibility-graph": "#3987e5",
    "extremities": "#3987e5",
    "path": "#d95926",
}


def clockwise(polygon):
    """the polygon with clockwise vertex order (holes)"""
    coords = np.array(polygon)
    x, y = coords.T
    area = np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))
    return coords if area < 0 else coords[::-1]


def dark_mode_css() -> str:
    path_color = DARK["path"]
    rules = [
        f"#boundary path {{ stroke: {DARK['boundary']} !important; }}",
        f"#obstacles path {{ fill: {DARK['obstacles']} !important; stroke: {DARK['obstacles']} !important; }}",
        f"#visibility-graph path {{ stroke: {DARK['visibility-graph']} !important; }}",
        (
            f"#extremities path, #extremities use {{ fill: {DARK['extremities']} !important;"
            f" stroke: {DARK['extremities']} !important; }}"
        ),
        f"#path path, #start path, #start use {{ stroke: {path_color} !important; }}",
        f"#goal path, #goal use {{ fill: {path_color} !important; stroke: {path_color} !important; }}",
    ]
    body = "\n    ".join(rules)
    return f"<style>\n  @media (prefers-color-scheme: dark) {{\n    {body}\n  }}\n</style>\n"


def main(output: Path = OUTPUT):
    environment = PolygonEnvironment(
        BOUNDARY, [clockwise(hole) for hole in HOLES], validate=True
    )
    path, length = environment.find_shortest_path(START, GOAL)
    print(f"path with {len(path)} points, length {length:.2f}")

    # deterministic SVG ids and no creation date: regenerating gives identical files
    with plt.rc_context({"svg.hashsalt": "extremitypathfinder"}):
        fig, ax = plt.subplots(figsize=(10.0, 4.0))
        draw(environment, path, ax=ax, style=LIGHT)
        ax.set_axis_off()
        margin = 0.4
        ax.set_xlim(-margin, 50.0 + margin)
        ax.set_ylim(-margin, 20.0 + margin)
        fig.savefig(
            output,
            format="svg",
            transparent=True,
            bbox_inches="tight",
            pad_inches=0.02,
            metadata={"Date": None},
        )
        plt.close(fig)

    # embed the dark mode colors after the opening <svg> tag
    svg = output.read_text(encoding="utf-8")
    end_of_tag = svg.index(">", svg.index("<svg")) + 1
    svg = svg[:end_of_tag] + "\n" + dark_mode_css() + svg[end_of_tag:]
    # without trailing whitespace (pre-commit hook)
    svg = "\n".join(line.rstrip() for line in svg.splitlines()) + "\n"
    output.write_text(svg, encoding="utf-8")
    print(f"written: {output} ({output.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else OUTPUT)
