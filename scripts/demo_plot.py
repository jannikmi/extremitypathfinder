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


def star(center, outer_radius, inner_radius, nr_points=5):
    """a star polygon: the tips are extremities, the inner corners are not"""
    angles = np.pi / 2 + np.arange(2 * nr_points) * np.pi / nr_points
    radii = np.where(np.arange(2 * nr_points) % 2 == 0, outer_radius, inner_radius)
    points = (
        np.array(center)
        + np.stack([np.cos(angles), np.sin(angles)], axis=1) * radii[:, None]
    )
    return [tuple(p) for p in np.round(points, 2)]


def rounded_rectangle(width, height, radius, nr_arc_points=4):
    """a rectangle with rounded corners: the corners of the boundary are no extremities"""
    corners = [
        (width - radius, radius),
        (width - radius, height - radius),
        (radius, height - radius),
        (radius, radius),
    ]
    points = []
    for k, (x, y) in enumerate(corners):
        # counter clockwise, each corner a quarter circle
        angles = -np.pi / 2 + k * np.pi / 2 + np.linspace(0.0, np.pi / 2, nr_arc_points)
        points += [(x + radius * np.cos(a), y + radius * np.sin(a)) for a in angles]
    return [tuple(p) for p in np.round(points, 2)]


def bowl(x_min, x_max, y_min, y_max, rim, nr_arc_points=7):
    """a block with a rounded bay: the corners along the bay are no extremities"""
    center_x = (x_min + x_max) / 2.0
    radius_x = (x_max - x_min) / 2.0 - rim
    radius_y = y_max - y_min - rim
    angles = np.linspace(0.0, -np.pi, nr_arc_points)
    bay = [
        (center_x + radius_x * np.cos(a), y_max + radius_y * np.sin(a)) for a in angles
    ]
    points = [(x_min, y_min), (x_max, y_min), (x_max, y_max)] + bay + [(x_min, y_max)]
    return [tuple(p) for p in np.round(points, 2)]


BOUNDARY = rounded_rectangle(50.0, 20.0, 2.0)
# concave obstacles: many vertices are no extremities (no graph nodes)
HOLES = [
    # decoration
    star((7.5, 15.5), 3.2, 1.4),
    # a plus sign
    [
        (20.6, 14.4),
        (22.4, 14.4),
        (22.4, 15.9),
        (23.9, 15.9),
        (23.9, 17.7),
        (22.4, 17.7),
        (22.4, 19.2),
        (20.6, 19.2),
        (20.6, 17.7),
        (19.1, 17.7),
        (19.1, 15.9),
        (20.6, 15.9),
    ],
    # a comb
    [
        (28.5, 1.0),
        (35.0, 1.0),
        (35.0, 3.8),
        (34.0, 3.8),
        (34.0, 2.2),
        (33.0, 2.2),
        (33.0, 3.8),
        (32.0, 3.8),
        (32.0, 2.2),
        (31.0, 2.2),
        (31.0, 3.8),
        (30.0, 3.8),
        (30.0, 2.2),
        (29.5, 2.2),
        (29.5, 3.8),
        (28.5, 3.8),
    ],
    # a bowl
    bowl(41.0, 46.5, 1.0, 4.6, 0.8),
    # near the start: a notched triangle
    [(5.0, 4.0), (8.0, 10.0), (11.0, 3.5), (8.2, 5.5)],
    # an L-shaped wall rising from the boundary
    [(15.0, 0.0), (15.0, 12.0), (17.0, 12.0), (17.0, 3.0), (21.0, 3.0), (21.0, 0.0)],
    # two squares touching at a corner, right where the path would like to pass
    [(24.0, 5.0), (24.0, 9.0), (28.0, 9.0), (28.0, 5.0)],
    [(28.0, 9.0), (28.0, 13.0), (32.0, 13.0), (32.0, 9.0)],
    # a wall hanging from the boundary, with a hook
    [(37.0, 6.0), (37.0, 20.0), (38.2, 20.0), (38.2, 7.2), (40.0, 7.2), (40.0, 6.0)],
    # in front of the goal: a notched triangle
    [(41.0, 9.0), (43.0, 12.5), (45.5, 9.5), (43.2, 10.4)],
]
START = (2.0, 2.5)
GOAL = (47.0, 15.5)

LIGHT = PlotStyle(graph_alpha=0.15, marker_size=3.5, vertex_color="#8a8984")
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
    print(
        f"{environment.nr_vertices} vertices, {len(environment.extremity_indices)} extremities,"
        f" {environment.graph.number_of_edges()} graph edges"
    )

    # deterministic SVG ids and no creation date: regenerating gives identical files
    with plt.rc_context({"svg.hashsalt": "extremitypathfinder"}):
        fig, ax = plt.subplots(figsize=(10.0, 4.0))
        draw(environment, path, ax=ax, vertices=True, style=LIGHT)
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
