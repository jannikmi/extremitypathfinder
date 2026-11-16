"""Generates the figures of the documentation and the demo image of the README.

usage: python scripts/docs_figures.py [output_dir]  (default: docs/_static)

All figures are drawn with ``extremitypathfinder.plotting`` (``draw()`` and ``draw_search()``),
the paths are the actual results of the package.
The exports are deterministic: regenerating them without changes yields identical files.
The colors of the demo image are swapped for dark backgrounds
with an embedded CSS media query (``prefers-color-scheme``).
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.ticker import MaxNLocator

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlotStyle, draw, draw_search

OUTPUT_DIR = Path(__file__).resolve().parents[1] / "docs" / "_static"


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

# a transparent banner: the outside of the boundary is not filled
LIGHT = PlotStyle(
    graph_alpha=0.15, marker_size=3.5, vertex_color="#8a8984", outside_color=None
)
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


def export_svg(fig, output: Path, css: str = ""):
    """saves the figure deterministically (SVG ids, no creation date) and closes it"""
    with plt.rc_context({"svg.hashsalt": "extremitypathfinder"}):
        fig.savefig(
            output,
            format="svg",
            transparent=True,
            bbox_inches="tight",
            pad_inches=0.02,
            metadata={"Date": None},
        )
    plt.close(fig)
    svg = output.read_text(encoding="utf-8")
    if css:
        # after the opening <svg> tag
        end_of_tag = svg.index(">", svg.index("<svg")) + 1
        svg = svg[:end_of_tag] + "\n" + css + svg[end_of_tag:]
    # without trailing whitespace (pre-commit hook)
    svg = "\n".join(line.rstrip() for line in svg.splitlines()) + "\n"
    output.write_text(svg, encoding="utf-8")


def demo(output_dir: Path):
    """the demo image of the README and the landing page"""
    environment = PolygonEnvironment(
        BOUNDARY, [clockwise(hole) for hole in HOLES], validate=True
    )
    path, length = environment.find_shortest_path(START, GOAL)
    print(
        f"demo: path with {len(path)} points, length {length:.2f}, {environment.nr_vertices} vertices,"
        f" {len(environment.extremity_indices)} extremities,"
        f" {environment.graph.number_of_edges()} graph edges"
    )
    fig, ax = plt.subplots(figsize=(10.0, 4.0))
    draw(environment, path, ax=ax, vertices=True, style=LIGHT)
    ax.set_axis_off()
    margin = 0.4
    ax.set_xlim(-margin, 50.0 + margin)
    ax.set_ylim(-margin, 20.0 + margin)
    export_svg(fig, output_dir / "demo.svg", dark_mode_css())


# the example of the usage documentation
USAGE_BOUNDARY = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
USAGE_HOLES = [[(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0)]]
USAGE_QUERY = ((4.5, 1.0), (4.0, 8.5))
# the grid of the usage documentation: size, blocked cells and a query
USAGE_GRID = (
    10,
    6,
    [
        # a wall at the border of the grid
        (3, 0),
        (3, 1),
        (3, 2),
        (3, 3),
        # an L-shaped obstacle
        (6, 3),
        (6, 4),
        (7, 3),
        # a cell touching the L-shape at a corner (diagonally adjacent)
        (5, 2),
    ],
)
# the straight line would pass through the corner where the cells touch
USAGE_GRID_QUERY = ((7.5, 1.5), (4.5, 4.5))
# touching obstacles (cf. tests/test_contacts.py): holes, start and goal
TOUCHING = {
    "touching_vertices": (
        [
            [(3.0, 5.0), (3.0, 7.0), (5.0, 7.0), (5.0, 5.0)],
            [(5.0, 2.0), (5.0, 5.0), (8.0, 5.0), (8.0, 2.0)],
        ],
        ((4.0, 4.0), (6.0, 6.0)),
    ),
    "touching_vertex_edge": (
        [
            [(3.0, 5.0), (3.0, 7.0), (7.0, 7.0), (7.0, 5.0)],
            [(5.0, 5.0), (6.0, 3.0), (4.0, 3.0)],
        ],
        ((1.0, 4.0), (9.0, 4.0)),
    ),
    "touching_edges": (
        [
            [(3.0, 4.0), (3.0, 6.0), (5.0, 6.0), (5.0, 4.0)],
            [(5.0, 3.0), (5.0, 5.0), (8.0, 5.0), (8.0, 3.0)],
        ],
        ((5.0, 1.0), (5.0, 8.0)),
    ),
}


# the agent clearance example of the usage documentation:
# the passage between the obstacles is narrower than twice the clearance
CLEARANCE_BOUNDARY = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0), (0.0, 8.0)]
CLEARANCE_HOLES = [
    [(3.0, 1.5), (6.0, 1.5), (6.0, 4.0), (3.0, 4.0)],
    [(6.0, 4.8), (9.0, 6.5), (5.0, 6.5)],
]
CLEARANCE = 0.5
CLEARANCE_QUERY = ((1.0, 3.0), (11.0, 4.0))


# inch per coordinate unit: the same scale for all figures
SCALE = 0.42


def new_axes():
    """axes with recessive spines and ticks: the coordinates relate the figures to the examples"""
    fig, ax = plt.subplots()
    for spine in ax.spines.values():
        spine.set_color(PlotStyle.obstacle_color)
    ax.tick_params(labelsize=8, colors=PlotStyle.vertex_color)
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(MaxNLocator(integer=True, steps=[1, 2, 5, 10]))
    return fig, ax


def export_to_scale(fig, ax, output: Path, margin: float = 0.5):
    """exports the figure with the same size per coordinate unit as all other figures"""
    ax.autoscale_view()
    (x_min, x_max), (y_min, y_max) = ax.get_xlim(), ax.get_ylim()
    x_min, x_max, y_min, y_max = (
        x_min - margin,
        x_max + margin,
        y_min - margin,
        y_max + margin,
    )
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    # additional space for the tick labels
    fig.set_size_inches((x_max - x_min) * SCALE + 0.5, (y_max - y_min) * SCALE + 0.4)
    export_svg(fig, output)


def usage_figures(output_dir: Path):
    """the steps of the algorithm on the example of the usage documentation"""
    environment = PolygonEnvironment(USAGE_BOUNDARY, USAGE_HOLES, validate=True)
    result = environment.search(*USAGE_QUERY)
    figures = {
        "environment": lambda ax: draw(environment, ax=ax, graph=False, vertices=True),
        "environment_graph": lambda ax: draw(environment, ax=ax, vertices=True),
        "search_graph": lambda ax: draw_search(
            environment, result, ax=ax, path=False, vertices=True
        ),
        "search": lambda ax: draw_search(environment, result, ax=ax, vertices=True),
    }
    for name, draw_figure in figures.items():
        fig, ax = new_axes()
        draw_figure(ax)
        export_to_scale(fig, ax, output_dir / f"{name}.svg")

    size_x, size_y, obstacles = USAGE_GRID
    grid = PolygonEnvironment.from_grid(size_x, size_y, obstacles, validate=True)
    path, _ = grid.find_shortest_path(*USAGE_GRID_QUERY)
    fig, ax = new_axes()
    draw(grid, path, ax=ax, graph=False, vertices=True)
    # the borders of all grid cells, also of the blocked ones: cell (x, y) spans [x, x+1] x [y, y+1]
    cell_borders = [((x, 0), (x, size_y)) for x in range(size_x + 1)]
    cell_borders += [((0, y), (size_x, y)) for y in range(size_y + 1)]
    ax.add_collection(
        LineCollection(
            cell_borders,
            colors=PlotStyle.boundary_color,
            alpha=0.3,
            linewidths=0.6,
            # above the obstacles, below the boundary and the path
            zorder=1.2,
            gid="cells",
        )
    )
    export_to_scale(fig, ax, output_dir / "grid.svg")


def touching_figures(output_dir: Path):
    """paths around touching obstacles (unbounded environments)"""
    for name, (holes, query) in TOUCHING.items():
        environment = PolygonEnvironment(None, holes, validate=True)
        result = environment.search(*query)
        fig, ax = new_axes()
        # the vertices show the contact points
        draw_search(environment, result, ax=ax, vertices=True)
        export_to_scale(fig, ax, output_dir / f"{name}.svg")


def clearance_environment(join_style: str = "mitre") -> PolygonEnvironment:
    """the clearance example: the boundary shrunk and the obstacles inflated with Shapely"""
    from shapely import Polygon, union_all

    boundary = Polygon(CLEARANCE_BOUNDARY).buffer(-CLEARANCE, join_style=join_style)
    obstacles = union_all(
        [
            Polygon(hole).buffer(CLEARANCE, join_style=join_style)
            for hole in CLEARANCE_HOLES
        ]
    )
    return PolygonEnvironment.from_shapely(boundary, obstacles, validate=True)


def clearance_figure(output_dir: Path):
    """the path with clearance (configuration space) against the path without clearance"""
    environment = clearance_environment()
    path, _ = environment.find_shortest_path(*CLEARANCE_QUERY)
    original = PolygonEnvironment(CLEARANCE_BOUNDARY, CLEARANCE_HOLES, validate=True)
    path_without, _ = original.find_shortest_path(*CLEARANCE_QUERY)
    fig, ax = new_axes()
    draw(environment, path, ax=ax, graph=False)
    rings = [CLEARANCE_BOUNDARY, *CLEARANCE_HOLES]
    ax.add_collection(
        LineCollection(
            [[*ring, ring[0]] for ring in rings],
            colors=PlotStyle.vertex_color,
            linewidths=PlotStyle.line_width,
            # above the inflated obstacles, below the path
            zorder=3.5,
            gid="original-polygons",
        )
    )
    x, y = np.array(path_without).T
    ax.plot(
        x,
        y,
        color=PlotStyle.path_color,
        linewidth=PlotStyle.line_width * 1.5,
        linestyle="--",
        zorder=3.8,
        gid="path-without-clearance",
    )
    export_to_scale(fig, ax, output_dir / "clearance.svg")


def complete_visibility_graph(environment: PolygonEnvironment) -> list:
    """the edges of the classical visibility graph of all polygon vertices, computed with Shapely

    For comparison only: two vertices are connected when the segment between them lies within the closed
    free space and passes no point where obstacles touch (touching obstacles block paths in this package).
    """
    from shapely import LineString, Point, Polygon, prepared, union_all

    boundary = Polygon(environment.boundary_polygon)
    holes = [Polygon(hole) for hole in environment.holes]
    free = prepared.prep(boundary.difference(union_all(holes)))
    rings = [boundary.exterior] + [hole.exterior for hole in holes]
    # distinct points: touching obstacles share vertices
    vertices = list(dict.fromkeys(tuple(v) for v in environment.coords))
    # vertices lying on another polygon: there, obstacles touch each other or the boundary
    contacts = [
        Point(v)
        for i, ring in enumerate(rings)
        for v in ring.coords[:-1]
        if any(other.intersects(Point(v)) for j, other in enumerate(rings) if j != i)
    ]
    edges = []
    for i, start in enumerate(vertices):
        for goal in vertices[i + 1 :]:
            segment = LineString([start, goal])
            if not free.covers(segment):
                continue
            through_contact = any(
                c.coords[0] not in (start, goal) and segment.intersects(c)
                for c in contacts
            )
            if not through_contact:
                edges.append((start, goal))
    return edges


def draw_graph_reduction(axes, marker_size: float = 3.5, path_width: float = 3.0):
    """the demo environment: the complete visibility graph of all vertices against the reduced graph
    of the extremities with the shortest path (two axes)"""
    environment = PolygonEnvironment(
        BOUNDARY, [clockwise(hole) for hole in HOLES], validate=True
    )
    path, _ = environment.find_shortest_path(START, GOAL)
    complete = complete_visibility_graph(environment)
    nr_points = len({tuple(v) for v in environment.coords})
    style = PlotStyle(
        graph_alpha=0.3,
        line_width=0.7,
        path_width=path_width,
        marker_size=marker_size,
        vertex_color="#8a8984",
        outside_color=None,
    )
    # the complete graph drawn like the reduced graph, without marking the extremities
    # the shortest path is the same on both graphs
    draw(
        environment,
        path,
        ax=axes[0],
        graph=False,
        extremities=False,
        vertices=True,
        style=style,
    )
    axes[0].add_collection(
        LineCollection(
            complete,
            colors=style.graph_color,
            alpha=style.graph_alpha,
            linewidths=style.line_width,
            zorder=2,
            gid="complete-visibility-graph",
        )
    )
    draw(environment, path, ax=axes[1], vertices=True, style=style)
    titles = (
        f"complete visibility graph: {nr_points} vertices, {len(complete):,} edges",
        (
            f"reduced visibility graph: {len(environment.extremity_indices)} extremities,"
            f" {environment.graph.number_of_edges()} edges"
        ),
    )
    for ax, title in zip(axes, titles):
        ax.set_axis_off()
        margin = 0.4
        ax.set_xlim(-margin, 50.0 + margin)
        ax.set_ylim(-margin, 20.0 + margin)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=10, color="#0b0b0b", loc="left")
    print(f"graph reduction: {titles[0]}; {titles[1]}")


def graph_reduction_figure(output_dir: Path):
    """the reduction of the visibility graph (About page and JOSS paper)"""
    fig, axes = plt.subplots(2, 1, figsize=(10.0, 8.6))
    draw_graph_reduction(axes)
    export_svg(fig, output_dir / "graph_reduction.svg")


def main(output_dir: Path = OUTPUT_DIR):
    output_dir.mkdir(parents=True, exist_ok=True)
    demo(output_dir)
    usage_figures(output_dir)
    touching_figures(output_dir)
    try:
        import shapely  # noqa: F401
    except ImportError:
        print("skipped: clearance.svg, graph_reduction.svg (require Shapely)")
    else:
        clearance_figure(output_dir)
        graph_reduction_figure(output_dir)
    for file in sorted(output_dir.glob("*.svg")):
        print(f"written: {file} ({file.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else OUTPUT_DIR)
