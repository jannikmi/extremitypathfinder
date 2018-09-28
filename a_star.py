# MODIFIED
# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

import math


class SimpleGraph:
    def __init__(self):
        self.edges = {}

    def neighbours(self, id):
        return self.edges[id]


example_graph = SimpleGraph()
example_graph.edges = {
    'A': ['B'],
    'B': ['A', 'C', 'D'],
    'C': ['A'],
    'D': ['E', 'A'],
    'E': ['B']
}

import collections


class Queue:
    def __init__(self):
        self.elements = collections.deque()

    def empty(self):
        return len(self.elements) == 0

    def put(self, x):
        self.elements.append(x)

    def get(self):
        return self.elements.popleft()


# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)


def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = ">"
        if x2 == x1 - 1: r = "<"
        if y2 == y1 + 1: r = "v"
        if y2 == y1 - 1: r = "^"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r


def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width), end="")
        print()


# data from main article
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in
                  [21, 22, 51, 52, 81, 82, 93, 94, 111, 112, 123, 124, 133, 134, 141, 142, 153, 154, 163, 164, 171, 172,
                   173, 174, 175, 183, 184, 193, 194, 201, 202, 203, 204, 205, 213, 214, 223, 224, 243, 244, 253, 254,
                   273, 274, 283, 284, 303, 304, 313, 314, 333, 334, 343, 344, 373, 374, 403, 404, 433, 434]]


class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def neighbors(self, id):
        (x, y) = id
        results = [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
        if (x + y) % 2 == 0: results.reverse()  # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        super().__init__(width, height)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)


diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}

import heapq


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]


def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbours(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


# thanks to @m1sp <Jaiden Mispy> for this simpler version of
# reconstruct_path that doesn't have duplicate entries

# def reconstruct_path(came_from, start, goal):
#     current = goal
#     path = []
#     while current != start:
#         path.append(current)
#         current = came_from[current]
#     path.append(start)  # optional
#     path.reverse()  # optional
#     return path


def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    # geometrical property of this problem: it is always shortest to directly reach a node
    #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
    # when goal is directly reachable

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbours(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far


from helpers import HeuristicGraph, Vertex


# TODO numba precompilation
def modified_a_star(heuristic_graph: HeuristicGraph, start: Vertex, goal: Vertex):

    def reconstruct_path():
        # backtrack where path came from
        current = goal
        path = []
        while current is not None:
            path.append(current)
            current = came_from[current]
        # path.reverse()  # to get the actual path
        return path

    heuristic_graph.set_goal_node(goal)  # computes the heuristic for all vertices once # TODO lazy

    # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
    #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
    # -> when goal is directly reachable, there can be no other shorter path to it
    if goal in heuristic_graph.get_neighbours(start):
        return [start, goal], heuristic_graph.get_heuristic(start)

    # idea: make sure to not 'walk back' delete edges with increasing heuristic?!
    #   -> not worth it. requires basically a a* search...

    priority_queue = PriorityQueue()
    priority_queue.put(start, 0.0)
    came_from = {start: None, }
    cost_so_far = {start: 0.0, }

    while not priority_queue.empty():
        # always 'expand' the node with the lowest current cost estimate (= cost_so_far + heuristic)
        current = priority_queue.get()

        # look at the distances to all neighbours
        for next_node, distance in heuristic_graph._existing_edges_from(current):

            # since the current node is the one with the lowest cost estimate
            #   and the goal is directly reachable from the current node (-> heuristic == distance),
            # the cost estimate is actually the true cost.
            # because of the geometric property mentioned above there can be no other shortest path to the goal
            # the algorithm can be terminated (regular a* would now still continue to fully expand the current node)
            # optimisation: let _exiting_edges_from() return the goal node first if it is among the neighbours
            if next_node == goal:
                total_path_length = cost_so_far[current] + distance
                came_from[next_node] = current
                return reconstruct_path(), total_path_length

            new_cost = cost_so_far[current] + distance
            if new_cost < cost_so_far.get(next_node, default=math.inf):
                # found shortest path to this node so far
                cost_so_far[next_node] = new_cost
                came_from[next_node] = current
                priority = new_cost + heuristic_graph.get_heuristic(next_node)
                priority_queue.put(next_node, priority)

    # should never occur
    raise ValueError('goal not reachable')
