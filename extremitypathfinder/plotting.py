import time
from os import makedirs
from os.path import abspath, join

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

from extremitypathfinder.extremitypathfinder import PolygonEnvironment, SearchResult

EXPORT_RESOLUTION = 200  # dpi
EXPORT_SIZE_X = 19.0  # inch
EXPORT_SIZE_Y = 11.0  # inch

POLYGON_SETTINGS = {
    "edgecolor": "black",
    "fill": False,
    "linewidth": 1.0,
}

SHOW_PLOTS = False
PLOTTING_DIR = "all_plots"
PLOT_FILE_ENDING = ".svg"


def get_plot_name(file_name="plot", plotting_dir=PLOTTING_DIR):
    return abspath(
        join(plotting_dir, file_name + "_" + str(time.time())[:-7] + PLOT_FILE_ENDING)
    )


def export_plot(fig, file_name, plotting_dir=PLOTTING_DIR):
    makedirs(plotting_dir, exist_ok=True)
    fig.set_size_inches(EXPORT_SIZE_X, EXPORT_SIZE_Y, forward=True)
    plt.savefig(get_plot_name(file_name, plotting_dir), dpi=EXPORT_RESOLUTION)
    plt.close()


def mark_points(vertex_iter, **kwargs):
    try:
        coordinates = [v.tolist() for v in vertex_iter]
    except AttributeError:
        coordinates = list(vertex_iter)
    coords_zipped = list(zip(*coordinates))
    if coords_zipped:  # there might be no vertices at all
        plt.scatter(*coords_zipped, **kwargs)


def draw_edge(v1, v2, c, alpha, **kwargs):
    x1, y1 = v1
    x2, y2 = v2
    plt.plot([x1, x2], [y1, y2], color=c, alpha=alpha, **kwargs)


def draw_polygon(ax, coords, **kwargs):
    kwargs.update(POLYGON_SETTINGS)
    polygon = Polygon(coords, **kwargs)
    ax.add_patch(polygon)


def draw_boundaries(map, ax):
    if map.boundary_polygon is not None:
        draw_polygon(ax, map.boundary_polygon)
    for h in map.holes:
        draw_polygon(ax, h, facecolor="grey", fill=True)

    mark_points(map.all_vertices, c="black", s=15)
    mark_points(map.all_extremities, c="red", s=50)


def draw_internal_graph(map: PolygonEnvironment, ax):
    graph = map.graph
    coords = map.coords
    for n in graph.nodes:
        start = coords[n]
        all_goals = [coords[i] for i in graph.neighbors(n)]
        for goal in all_goals:
            draw_edge(start, goal, c="red", alpha=0.2, linewidth=2)


def set_limits(map, ax, query_coordinates=None):
    coordinates = map.boundary_polygon
    if coordinates is None:
        coordinates = map.coords
    if query_coordinates is not None:
        coordinates = np.concatenate((coordinates, query_coordinates))
    ax.set_xlim(
        (
            min(coordinates[:, 0]) - 1,
            max(coordinates[:, 0]) + 1,
        )
    )
    ax.set_ylim(
        (
            min(coordinates[:, 1]) - 1,
            max(coordinates[:, 1]) + 1,
        )
    )


def draw_path(vertex_path):
    # start, path and goal in green
    if not vertex_path:
        return
    mark_points(vertex_path, c="g", alpha=0.9, s=50)
    mark_points([vertex_path[0], vertex_path[-1]], c="g", s=100)
    v1 = vertex_path[0]
    for v2 in vertex_path[1:]:
        draw_edge(v1, v2, c="g", alpha=1.0)
        v1 = v2


def draw_loaded_map(map, plotting_dir=PLOTTING_DIR):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    export_plot(fig, "map_plot", plotting_dir)
    if SHOW_PLOTS:
        plt.show()


def draw_prepared_map(map, plotting_dir=PLOTTING_DIR):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)
    export_plot(fig, "prepared_map_plot", plotting_dir)
    if SHOW_PLOTS:
        plt.show()


def draw_with_path(map, result: SearchResult, plotting_dir=PLOTTING_DIR):
    fig, ax = plt.subplots()

    graph = result.graph
    coords = result.coords
    vertex_path = result.path
    all_nodes = graph.nodes
    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax, coords[-2:])

    if len(vertex_path) > 0:
        # additionally draw:
        # new edges yellow
        start, goal = vertex_path[0], vertex_path[-1]
        goal_idx = result.idx_goal
        start_idx = result.idx_start

        if start_idx is not None:
            for n_idx in graph.neighbors(start_idx):
                n = coords[n_idx]
                draw_edge(start, n, c="y", alpha=0.7)

        if goal_idx is not None:
            # edges only run towards goal
            for n_idx in all_nodes:
                if goal_idx in graph.neighbors(n_idx):
                    n = coords[n_idx]
                    draw_edge(n, goal, c="y", alpha=0.7)

    # start, path and goal in green
    draw_path(vertex_path)

    export_plot(fig, "graph_path_plot", plotting_dir)
    if SHOW_PLOTS:
        plt.show()


def draw_only_path(
    map, vertex_path, start_coordinates, goal_coordinates, plotting_dir=PLOTTING_DIR
):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax, np.array((start_coordinates, goal_coordinates)))
    draw_path(vertex_path)
    mark_points([start_coordinates, goal_coordinates], c="g", s=100)

    export_plot(fig, "path_plot", plotting_dir)
    if SHOW_PLOTS:
        plt.show()


def draw_graph(map, result: SearchResult, plotting_dir=PLOTTING_DIR):
    fig, ax = plt.subplots()

    graph = result.graph
    nodes = graph.nodes
    coords = result.coords
    all_nodes = [coords[i] for i in nodes]
    mark_points(all_nodes, c="black", s=30)

    for i in nodes:
        x, y = coords[i]
        neighbour_idxs = graph.neighbors(i)
        for n2_idx in neighbour_idxs:
            x2, y2 = coords[n2_idx]
            dx, dy = x2 - x, y2 - y
            plt.arrow(
                x,
                y,
                dx,
                dy,
                head_width=0.15,
                head_length=0.5,
                head_starts_at_zero=False,
                shape="full",
                length_includes_head=True,
            )

    set_limits(map, ax, coords[-2:])

    export_plot(fig, "graph_plot", plotting_dir)
    if SHOW_PLOTS:
        plt.show()


def plot_environment(environment: PolygonEnvironment, plotting_dir=PLOTTING_DIR):
    """Plots the input polygons and the visibility graph of an environment.

    :param environment: the environment to plot
    :param plotting_dir: the folder to store the plots in
    """
    draw_loaded_map(environment, plotting_dir)
    draw_prepared_map(environment, plotting_dir)


def plot_search(
    environment: PolygonEnvironment, result: SearchResult, plotting_dir=PLOTTING_DIR
):
    """Plots the result of a search with :meth:`PolygonEnvironment.search`.

    Plots the found path and, if the search built one, the search graph.

    :param environment: the environment the search was performed in
    :param result: the result of the search
    :param plotting_dir: the folder to store the plots in
    """
    start_coordinates, goal_coordinates = result.coords[-2:]
    draw_only_path(
        environment, result.path, start_coordinates, goal_coordinates, plotting_dir
    )
    # in some cases (e.g. direct path possible) no graph is being created!
    if result.graph is not None:
        draw_graph(environment, result, plotting_dir)
        draw_with_path(environment, result, plotting_dir)
