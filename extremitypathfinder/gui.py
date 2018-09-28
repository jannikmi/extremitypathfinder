import time

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# from helpers import *

EXPORT_RESOLUTION = 200  # dpi
EXPORT_SIZE_X = 19.0  # inch
EXPORT_SIZE_Y = 11.0  # inch

POLYGON_SETTINGS = {
    'edgecolor': 'black',
    'fill': False,
    'linewidth': 1.0,
}


def get_plot_name(file_name='plot'):
    return './plots/' + file_name + '_' + str(time.time())[:-7] + '.png'


def export_plot(fig, file_name):
    fig.set_size_inches(EXPORT_SIZE_X, EXPORT_SIZE_Y, forward=True)
    plt.savefig(get_plot_name(file_name), dpi=EXPORT_RESOLUTION)


def mark_points(vertex_list, **kwargs):
    x = []
    y = []
    for v in vertex_list:
        x.append(v.coordinates[0])
        y.append(v.coordinates[1])

    plt.scatter(x, y, **kwargs)


def draw_edge(v1, v2, c, alpha):
    x1, y1 = v1.coordinates
    x2, y2 = v2.coordinates
    plt.plot([x1, x2], [y1, y2], color=c, alpha=alpha)


def draw_polygon(ax, coords, **kwargs):
    kwargs.update(POLYGON_SETTINGS)
    polygon = Polygon(coords, **kwargs)
    ax.add_patch(polygon)


def draw_boundaries(map, ax):
    # TODO outside light grey
    # TODO fill holes light grey
    draw_polygon(ax, map.boundary_polygon.coordinates)
    for h in map.holes:
        draw_polygon(ax, h.coordinates, facecolor='grey', fill=True)

    mark_points(map.all_vertices, c='black', s=15)
    mark_points(map.all_extremities, c='red', s=50)


def draw_internal_graph(map, ax):
    for start, all_goals in map.graph.get_neighbours():
        for goal in all_goals:
            draw_edge(start, goal, c='red', alpha=0.2)


def set_limits(map, ax):
    ax.set_xlim((min(map.boundary_polygon.coordinates[:, 0]) - 1, max(map.boundary_polygon.coordinates[:, 0]) + 1))
    ax.set_ylim((min(map.boundary_polygon.coordinates[:, 1]) - 1, max(map.boundary_polygon.coordinates[:, 1]) + 1))


def draw_path(vertex_path):
    # start, path and goal in green
    if vertex_path:
        mark_points(vertex_path, c='g', alpha=0.9, s=50)
        mark_points([vertex_path[0], vertex_path[1]], c='g', s=100)
        v1 = vertex_path[0]
        for v2 in vertex_path[1:]:
            draw_edge(v1, v2, c='g', alpha=1.0)
            v1 = v2

# TODO unify functions and give int to decide what to draw
def draw_loaded_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    export_plot(fig, 'map_plot')
    plt.show()


def draw_prepared_map(map):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)
    export_plot(fig, 'prepared_map_plot')
    plt.show()


def draw_with_path(map, temp_graph, start, goal, vertex_path):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    draw_internal_graph(map, ax)
    set_limits(map, ax)

    # additionally draw:
    # new edges yellow
    if start in temp_graph.get_all_nodes():
        for n2, d in temp_graph._existing_edges_from(start):
            draw_edge(start, n2, c='y', alpha=0.7)

    all_nodes = temp_graph.get_all_nodes()
    if goal in all_nodes:
        # edges only run towards goal
        for n1 in all_nodes:
            if goal in temp_graph.get_neighbours_of(n1):
                draw_edge(n1, goal, c='y', alpha=0.7)

    # start, path and goal in green
    if vertex_path:
        mark_points(vertex_path, c='g', alpha=0.9, s=50)
        mark_points([start, goal], c='g', s=100)
        v1 = vertex_path[0]
        for v2 in vertex_path[1:]:
            draw_edge(v1, v2, c='g', alpha=1.0)
            v1 = v2

        export_plot(fig, 'graph_path_plot')
    plt.show()


def draw_only_path(map, vertex_path):
    fig, ax = plt.subplots()

    draw_boundaries(map, ax)
    set_limits(map, ax)
    draw_path(vertex_path)

    export_plot(fig, 'path_plot')
    plt.show()


def draw_graph(graph):
    fig, ax = plt.subplots()

    all_nodes = graph.get_all_nodes()
    mark_points(all_nodes, c='black', s=30)

    for n in all_nodes:
        x, y = n.coordinates
        neigbours = graph.get_neighbours_of(n)
        for n2 in neigbours:
            x2, y2 = n2.coordinates
            dx, dy = x2 - x, y2 - y
            plt.arrow(x, y, dx, dy, head_width=0.15, head_length=0.5, head_starts_at_zero=False, shape='full',
                      length_includes_head=True)

    ax.set_xlim((min(n.coordinates[0] for n in all_nodes) - 1, max(n.coordinates[0] for n in all_nodes) + 1))
    ax.set_ylim((min(n.coordinates[1] for n in all_nodes) - 1, max(n.coordinates[1] for n in all_nodes) + 1))

    export_plot(fig, 'graph_plot')
    plt.show()


if __name__ == '__main__':
    pass
