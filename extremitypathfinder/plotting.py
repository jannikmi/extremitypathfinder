import time
from os import makedirs
from os.path import abspath, exists, join

import matplotlib.pyplot as plt
import networkx as nx
from matplotlib.patches import Polygon

from extremitypathfinder.extremitypathfinder import PolygonEnvironment

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


def get_plot_name(file_name="plot"):
    return abspath(join(PLOTTING_DIR, file_name + "_" + str(time.time())[:-7] + ".png"))


def export_plot(fig, file_name):
    fig.set_size_inches(EXPORT_SIZE_X, EXPORT_SIZE_Y, forward=True)
    plt.savefig(get_plot_name(file_name), dpi=EXPORT_RESOLUTION)
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
    # TODO outside dark grey
    # TODO fill holes light grey
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


def set_limits(map, ax):
    ax.set_xlim(
        (
            min(map.boundary_polygon[:, 0]) - 1,
            max(map.boundary_polygon[:, 0]) + 1,
        )
    )
    ax.set_ylim(
        (
            min(map.boundary_polygon[:, 1]) - 1,
            max(map.boundary_polygon[:, 1]) + 1,
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


def draw_loaded_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    export_plot(fig, "map_plot")
    if SHOW_PLOTS:
        plt.show()


def draw_prepared_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)
    export_plot(fig, "prepared_map_plot")
    if SHOW_PLOTS:
        plt.show()


def draw_with_path(map, graph: nx.DiGraph, vertex_path):
    fig, ax = plt.subplots()

    coords = map._coords_tmp
    all_nodes = graph.nodes
    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)

    if len(vertex_path) > 0:
        # additionally draw:
        # new edges yellow
        start, goal = vertex_path[0], vertex_path[-1]
        goal_idx = map._idx_goal_tmp
        start_idx = map._idx_start_tmp

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

    export_plot(fig, "graph_path_plot")
    if SHOW_PLOTS:
        plt.show()


def draw_only_path(map, vertex_path, start_coordinates, goal_coordinates):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    draw_path(vertex_path)
    mark_points([start_coordinates, goal_coordinates], c="g", s=100)

    export_plot(fig, "path_plot")
    if SHOW_PLOTS:
        plt.show()


def draw_graph(map, graph: nx.DiGraph):
    fig, ax = plt.subplots()

    nodes = graph.nodes
    coords = map._coords_tmp
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

    set_limits(map, ax)

    export_plot(fig, "graph_plot")
    if SHOW_PLOTS:
        plt.show()


class PlottingEnvironment(PolygonEnvironment):
    """Extends PolygonEnvironment. In addition to the base functionality it
    plots graphs of the polygons, the visibility graph and the computed path.
    Stores all graphs in the folder defined by plotting_dir parameter."""

    def __init__(self, plotting_dir=PLOTTING_DIR):
        super().__init__()
        global PLOTTING_DIR
        PLOTTING_DIR = plotting_dir
        if not exists(plotting_dir):
            makedirs(plotting_dir)

    def store(self, *args, **kwargs):
        """In addition to storing, also plots a graph of the input polygons."""
        super().store(*args, **kwargs)
        draw_loaded_map(self)

    def prepare(self):
        """Also draws a prepared map with the computed visibility graph."""
        super().prepare()
        draw_prepared_map(self)

    def find_shortest_path(self, start_coordinates, goal_coordinates, *args, **kwargs):
        """Also draws the computed shortest path."""
        # important to not delete the temp graph! for plotting
        vertex_path, distance = super().find_shortest_path(
            start_coordinates, goal_coordinates, *args, free_space_after=False, **kwargs
        )

        draw_only_path(self, vertex_path, start_coordinates, goal_coordinates)
        if self.temp_graph:  # in some cases (e.g. direct path possible) no graph is being created!
            draw_graph(self, self.temp_graph)
            draw_with_path(self, self.temp_graph, vertex_path)
            del self.temp_graph  # free the memory

        # extract the coordinates from the path
        return vertex_path, distance
