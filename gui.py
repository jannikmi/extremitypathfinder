import numpy as np
from matplotlib.lines import Line2D
from matplotlib.artist import Artist
from matplotlib.mlab import dist_point_to_segment


# from helpers import *


class PolygonInteractor(object):
    """
    A polygon editor.

    Key-bindings

      't' toggle vertex markers on and off.  When vertex markers are on,
          you can move them, delete them

      'd' delete the vertex under point

      'i' insert a vertex at point.  You must be within epsilon of the
          line connecting two existing vertices

    """

    showverts = True
    epsilon = 5  # max pixel distance to count as a vertex hit

    def __init__(self, ax, poly):
        if poly.figure is None:
            raise RuntimeError('You must first add the polygon to a figure '
                               'or canvas before defining the interactor')
        self.ax = ax
        canvas = poly.figure.canvas
        self.poly = poly

        x, y = zip(*self.poly.xy)
        self.line = Line2D(x, y,
                           marker='o', markerfacecolor='r',
                           animated=True)
        self.ax.add_line(self.line)

        self.cid = self.poly.add_callback(self.poly_changed)
        self._ind = None  # the active vert

        canvas.mpl_connect('draw_event', self.draw_callback)
        canvas.mpl_connect('button_press_event', self.button_press_callback)
        canvas.mpl_connect('key_press_event', self.key_press_callback)
        canvas.mpl_connect('button_release_event', self.button_release_callback)
        canvas.mpl_connect('motion_notify_event', self.motion_notify_callback)
        self.canvas = canvas

    def draw_callback(self, event):
        self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        self.ax.draw_artist(self.poly)
        self.ax.draw_artist(self.line)
        # do not need to blit here, this will fire before the screen is
        # updated

    def poly_changed(self, poly):
        'this method is called whenever the polygon object is called'
        # only copy the artist props to the line (except visibility)
        vis = self.line.get_visible()
        Artist.update_from(self.line, poly)
        self.line.set_visible(vis)  # don't use the poly visibility state

    def get_ind_under_point(self, event):
        'get the index of the vertex under point if within epsilon tolerance'

        # display coords
        xy = np.asarray(self.poly.xy)
        xyt = self.poly.get_transform().transform(xy)
        xt, yt = xyt[:, 0], xyt[:, 1]
        d = np.hypot(xt - event.x, yt - event.y)
        indseq, = np.nonzero(d == d.min())
        ind = indseq[0]

        if d[ind] >= self.epsilon:
            ind = None

        return ind

    def button_press_callback(self, event):
        'whenever a mouse button is pressed'
        if not self.showverts:
            return
        if event.inaxes is None:
            return
        if event.button != 1:
            return
        self._ind = self.get_ind_under_point(event)

    def button_release_callback(self, event):
        'whenever a mouse button is released'
        if not self.showverts:
            return
        if event.button != 1:
            return
        self._ind = None

    def key_press_callback(self, event):
        'whenever a key is pressed'
        if not event.inaxes:
            return
        if event.key == 't':
            self.showverts = not self.showverts
            self.line.set_visible(self.showverts)
            if not self.showverts:
                self._ind = None
        elif event.key == 'd':
            ind = self.get_ind_under_point(event)
            if ind is not None:
                self.poly.xy = np.delete(self.poly.xy,
                                         ind, axis=0)
                self.line.set_data(zip(*self.poly.xy))
        elif event.key == 'i':
            xys = self.poly.get_transform().transform(self.poly.xy)
            p = event.x, event.y  # display coords
            for i in range(len(xys) - 1):
                s0 = xys[i]
                s1 = xys[i + 1]
                d = dist_point_to_segment(p, s0, s1)
                if d <= self.epsilon:
                    self.poly.xy = np.insert(
                        self.poly.xy, i + 1,
                        [event.xdata, event.ydata],
                        axis=0)
                    self.line.set_data(zip(*self.poly.xy))
                    break
        if self.line.stale:
            self.canvas.draw_idle()

    def motion_notify_callback(self, event):
        'on mouse movement'
        if not self.showverts:
            return
        if self._ind is None:
            return
        if event.inaxes is None:
            return
        if event.button != 1:
            return
        x, y = event.xdata, event.ydata

        self.poly.xy[self._ind] = x, y
        if self._ind == 0:
            self.poly.xy[-1] = x, y
        elif self._ind == len(self.poly.xy) - 1:
            self.poly.xy[0] = x, y
        self.line.set_data(zip(*self.poly.xy))

        self.canvas.restore_region(self.background)
        self.ax.draw_artist(self.poly)
        self.ax.draw_artist(self.line)
        self.canvas.blit(self.ax.bbox)


from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt


def mark_points(vertex_list, c, s):
    x = []
    y = []
    for v in vertex_list:
        x.append(v.coordinates[0])
        y.append(v.coordinates[1])

    plt.scatter(x, y, s=s, c=c, )


def draw_edge(v1, v2, c, alpha):
    x1, y1 = v1.coordinates
    x2, y2 = v2.coordinates
    plt.plot([x1, x2], [y1, y2], color=c, alpha=alpha)


def map_drawing_helper(map, ax):
    polygon_settings = {
        'edgecolor': 'black',
        'fill': False,
        'linewidth': 1.0,
    }

    def draw_polygon(coords):
        polygon = Polygon(coords, **polygon_settings)
        ax.add_patch(polygon)

    # outside light grey
    draw_polygon(map.boundary_polygon.coordinates)
    for h in map.holes:
        # TODO fill light grey
        draw_polygon(h.coordinates)

    mark_points(map.all_vertices, c='black', s=15)
    mark_points(map.all_extremities, c='red', s=50)

    for start, all_goals in map.graph.get_neighbours():
        for goal in all_goals:
            draw_edge(start, goal, c='red', alpha=0.3)

    ax.set_xlim((min(map.boundary_polygon.coordinates[:, 0]) - 1, max(map.boundary_polygon.coordinates[:, 0]) + 1))
    ax.set_ylim((min(map.boundary_polygon.coordinates[:, 1]) - 1, max(map.boundary_polygon.coordinates[:, 1]) + 1))


def draw_map(map):
    fig, ax = plt.subplots()

    map_drawing_helper(map, ax)
    # TODO export png from the different steps
    # TODO export png depending on the current status of the map!
    plt.show()


def draw_with_path(map, temp_graph, start, goal, vertex_path):
    fig, ax = plt.subplots()

    map_drawing_helper(map, ax)

    # additionally draw:
    # new edges yellow
    if start in temp_graph.get_all_nodes():
        for n2, d in temp_graph._existing_edges_from(start):
            draw_edge(start, n2, c='y', alpha=0.7)

    all_nodes = temp_graph.get_all_nodes()
    if goal in all_nodes:
        # edges only run towards goal TODO
        for n1 in all_nodes:
            if goal in temp_graph.get_neighbours_of(n1):
                draw_edge(n1, goal, c='y', alpha=0.7)

    # start, path and goal in green
    mark_points(vertex_path, c='g', s=50)
    mark_points([start, goal], c='g', s=80)
    if vertex_path:
        v1 = vertex_path[0]
        for v2 in vertex_path[1:]:
            draw_edge(v1,v2,c='g',alpha=1)
            v1=v2

    plt.show()


def draw_graph(graph):
    fig, ax = plt.subplots()

    all_nodes = graph.get_all_nodes()
    mark_points(all_nodes, c='black', s=30)

    for n in all_nodes:
        x,y = n.coordinates
        neigbours = graph.get_neighbours_of(n)
        for n2 in neigbours:
            x2,y2 = n2.coordinates
            dx,dy = x2-x,y2-y
            plt.arrow(x,y,dx,dy,head_width=0.2,head_length=0.7,head_starts_at_zero=False,shape='full',length_includes_head=True)

    ax.set_xlim((min(n.coordinates[0] for n in all_nodes) - 1, max(n.coordinates[0] for n in all_nodes) + 1))
    ax.set_ylim((min(n.coordinates[1] for n in all_nodes) - 1, max(n.coordinates[1] for n in all_nodes) + 1))

    plt.show()


if __name__ == '__main__':
    pass
    # # counter clockwise edge numbering!
    # # polygon1 = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    # # polygon1 = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # polygon1 = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # # clockwise numbering!
    # # holes1 = []
    # # holes1 = [[(3.0, 7.0), (5.0, 9.0), (5.0, 7.0), ], ]
    # holes1 = [[(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0), ], ]
    #
    # map = Map()
    # map.store(polygon1, holes1)
    # # print(map.all_extremities)
    # map.prepare()
    # draw_map(map)
    # import matplotlib.pyplot as plt
    # from matplotlib.patches import Polygon
    #
    # theta = np.arange(0, 2 * np.pi, 0.1)
    # r = 1.5
    #
    # xs = r * np.cos(theta)
    # ys = r * np.sin(theta)
    #
    # poly = Polygon(np.column_stack([xs, ys]), animated=True)
    #
    # fig, ax = plt.subplots()
    # ax.add_patch(poly)
    # p = PolygonInteractor(ax, poly)
    #
    # ax.set_title('Click and drag a point to move it')
    # ax.set_xlim((-2, 2))
    # ax.set_ylim((-2, 2))
    # plt.show()
