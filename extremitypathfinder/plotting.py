"""Plotting environments and shortest path searches with Matplotlib

:func:`draw` and :func:`draw_search` draw onto Matplotlib axes,
:func:`plot_environment` and :func:`plot_search` export these drawings as files.
"""

import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.collections import LineCollection, PatchCollection
from matplotlib.patches import Polygon

from extremitypathfinder.extremitypathfinder import PolygonEnvironment, SearchResult
from extremitypathfinder.types import InputCoord

PLOTTING_DIR = "all_plots"
PLOT_FILE_ENDING = ".svg"
# in inch
_EXPORT_SIZE = (10.0, 8.0)


@dataclass(frozen=True)
class PlotStyle:
    """The colors (matplotlib color specifications) and sizes used for drawing

    The defaults are readable on light and dark backgrounds.
    """

    boundary_color: str = "#8a8984"
    obstacle_color: str = "#c9c8c2"
    vertex_color: str = "#52514e"
    extremity_color: str = "#2a78d6"
    graph_color: str = "#2a78d6"
    graph_alpha: float = 0.3
    query_edge_color: str = "#189c6d"
    query_edge_alpha: float = 0.8
    path_color: str = "#eb6834"
    # in points
    line_width: float = 1.0
    path_width: float = 3.0
    marker_size: float = 5.0


DEFAULT_STYLE = PlotStyle()


def _draw_endpoints(ax: Axes, start, goal, style: PlotStyle):
    """start: ring, goal: disk"""
    area = (2.5 * style.marker_size) ** 2
    ax.scatter(
        [start[0]],
        [start[1]],
        s=area,
        facecolors="none",
        edgecolors=style.path_color,
        linewidths=style.path_width,
        zorder=5,
        gid="start",
    )
    ax.scatter(
        [goal[0]], [goal[1]], s=area, color=style.path_color, zorder=5, gid="goal"
    )


def draw(
    environment: PolygonEnvironment,
    path: Sequence[InputCoord] = (),
    *,
    ax: Axes | None = None,
    graph: bool = True,
    extremities: bool = True,
    vertices: bool = False,
    style: PlotStyle = DEFAULT_STYLE,
) -> Axes:
    """Draws an environment and optionally a path onto matplotlib axes.

    The drawing composes with other matplotlib code: the figure can be adjusted and saved in any format.

    Every element is drawn with a group id (``gid``), which is kept in SVG exports
    (e.g. for styling them with CSS): ``boundary``, ``obstacles``, ``visibility-graph``,
    ``vertices``, ``extremities``, ``path``, ``start`` and ``goal``.

    :param environment: the environment to draw
    :param path: the path to draw, e.g. as returned by :meth:`PolygonEnvironment.find_shortest_path`
    :param ax: the axes to draw on, a new figure is created when not given
    :param graph: whether to draw the visibility graph
    :param extremities: whether to mark the extremities
    :param vertices: whether to mark all polygon vertices
    :param style: the colors and sizes
    :return: the axes drawn on
    """
    if ax is None:
        _, ax = plt.subplots()
    width = style.line_width
    if environment.boundary_polygon is not None:
        boundary = Polygon(
            environment.boundary_polygon,
            fill=False,
            edgecolor=style.boundary_color,
            linewidth=width,
            joinstyle="round",
            # above the obstacles touching it
            zorder=1.5,
            gid="boundary",
        )
        ax.add_patch(boundary)
    # the same color for the edges: touching obstacles appear as one
    obstacles = PatchCollection(
        [Polygon(hole) for hole in environment.holes],
        facecolor=style.obstacle_color,
        edgecolor=style.obstacle_color,
        linewidth=width,
        joinstyle="round",
        zorder=1,
        gid="obstacles",
    )
    ax.add_collection(obstacles)

    coords = environment.coords
    if graph:
        edges = LineCollection(
            [(coords[i], coords[j]) for i, j in environment.graph.edges],
            colors=style.graph_color,
            alpha=style.graph_alpha,
            linewidths=width,
            capstyle="round",
            zorder=2,
            gid="visibility-graph",
        )
        ax.add_collection(edges)

    marker_area = style.marker_size**2
    if vertices:
        x, y = coords.T
        ax.scatter(
            x, y, s=marker_area, color=style.vertex_color, zorder=3, gid="vertices"
        )
    if extremities and len(environment.extremity_indices) > 0:
        x, y = coords[environment.extremity_indices].T
        ax.scatter(
            x,
            y,
            s=marker_area,
            color=style.extremity_color,
            zorder=3,
            gid="extremities",
        )

    if len(path) > 0:
        x, y = np.array(path, dtype=float).T
        ax.plot(
            x,
            y,
            color=style.path_color,
            linewidth=style.path_width,
            solid_capstyle="round",
            solid_joinstyle="round",
            zorder=4,
            gid="path",
        )
        _draw_endpoints(ax, (x[0], y[0]), (x[-1], y[-1]), style)

    ax.set_aspect("equal")
    ax.autoscale_view()
    return ax


def draw_search(
    environment: PolygonEnvironment,
    result: SearchResult,
    *,
    ax: Axes | None = None,
    graph: bool = True,
    query_edges: bool = True,
    path: bool = True,
    extremities: bool = True,
    vertices: bool = False,
    style: PlotStyle = DEFAULT_STYLE,
) -> Axes:
    """Draws the result of a search with :meth:`PolygonEnvironment.search` onto matplotlib axes.

    Like :func:`draw` with the found path. In addition, the temporary edges connecting start and goal
    to the visibility graph are drawn dashed (group id ``query-edges``).
    Start and goal are also marked when there is no path (or it is not drawn).
    Searches terminating without a graph search (e.g. when the goal is directly visible) have no temporary edges.

    :param environment: the environment the search was performed in
    :param result: the result of the search
    :param query_edges: whether to draw the temporary edges of start and goal
    :param path: whether to draw the found path (e.g. ``False`` to show the search graph only)
    :return: the axes drawn on

    The other parameters are those of :func:`draw`.
    """
    ax = draw(
        environment,
        result.path if path else (),
        ax=ax,
        graph=graph,
        extremities=extremities,
        vertices=vertices,
        style=style,
    )
    coords = result.coords
    if query_edges and result.query_edges:
        edges = LineCollection(
            [(coords[i], coords[j]) for i, j, _ in result.query_edges],
            colors=style.query_edge_color,
            alpha=style.query_edge_alpha,
            linewidths=style.line_width,
            linestyles="dashed",
            capstyle="round",
            zorder=2.5,
            gid="query-edges",
        )
        ax.add_collection(edges)
    if not path or len(result.path) == 0:
        # start and goal are the last coordinates
        _draw_endpoints(ax, coords[-2], coords[-1], style)
    ax.autoscale_view()
    return ax


def _export(ax: Axes, plotting_dir, file_name: str | None, default_name: str) -> Path:
    if file_name is None:
        file_name = f"{default_name}_{time.time_ns()}{PLOT_FILE_ENDING}"
    directory = Path(plotting_dir)
    directory.mkdir(parents=True, exist_ok=True)
    file_path = directory / file_name
    ax.figure.savefig(file_path, bbox_inches="tight")
    plt.close(ax.figure)
    return file_path.resolve()


def plot_environment(
    environment: PolygonEnvironment,
    plotting_dir=PLOTTING_DIR,
    *,
    file_name: str | None = None,
    style: PlotStyle = DEFAULT_STYLE,
) -> Path:
    """Exports a plot of the environment with its visibility graph and extremities (cf. :func:`draw`).

    :param environment: the environment to plot
    :param plotting_dir: the folder to store the plot in (created if missing)
    :param file_name: the name of the file, its extension selects the format.
        Default: ``environment_<timestamp>.svg``
    :param style: the colors and sizes
    :return: the path of the file
    """
    _, ax = plt.subplots(figsize=_EXPORT_SIZE)
    draw(environment, ax=ax, style=style)
    return _export(ax, plotting_dir, file_name, "environment")


def plot_search(
    environment: PolygonEnvironment,
    result: SearchResult,
    plotting_dir=PLOTTING_DIR,
    *,
    file_name: str | None = None,
    style: PlotStyle = DEFAULT_STYLE,
) -> Path:
    """Exports a plot of the result of a search with :meth:`PolygonEnvironment.search` (cf. :func:`draw_search`).

    :param environment: the environment the search was performed in
    :param result: the result of the search
    :param plotting_dir: the folder to store the plot in (created if missing)
    :param file_name: the name of the file, its extension selects the format.
        Default: ``search_<timestamp>.svg``
    :param style: the colors and sizes
    :return: the path of the file
    """
    _, ax = plt.subplots(figsize=_EXPORT_SIZE)
    draw_search(environment, result, ax=ax, style=style)
    return _export(ax, plotting_dir, file_name, "search")
