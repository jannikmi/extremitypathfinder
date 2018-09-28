from typing import List

import numpy as np


class AngleRepresentation:
    """
    a class automatically computing a representation for the angle from the origin to a given vector
    value in [0.0 : 4.0[
    every quadrant contains angle measures from 0.0 to 1.0
    there are 4 quadrants (counter clockwise numbering)
    0 / 360 degree -> 0.0
    90 degree -> 1.0
    180 degree -> 2.0
    270 degree -> 3.0
    ...
    Useful for comparing angles without actually computing expensive trigonometrical functions
    This representation does not grow directly proportional to its represented angle,
    but it its bijective and monotonous:
    rep(p1) > rep(p2) <=> angle(p1) > angle(p2)
    rep(p1) = rep(p2) <=> angle(p1) = angle(p2)
    angle(p): counter clockwise angle between the two line segments (0,0)'--(1,0)' and (0,0)'--p
    with (0,0)' being the vector representing the origin

    """
    quadrant = None
    angle_measure = None
    value = None

    def __init__(self, np_vector):
        # 2D vector: (dx, dy) = np_vector
        norm = np.linalg.norm(np_vector)
        if norm == 0.0:
            # make sure norm is not 0!
            raise ValueError('received null vector:', np_vector, norm)

        dx_positive = np_vector[0] >= 0
        dy_positive = np_vector[1] >= 0

        if dx_positive and dy_positive:
            self.quadrant = 0.0
            self.angle_measure = np_vector[1] / norm

        elif not dx_positive and dy_positive:
            self.quadrant = 1.0
            self.angle_measure = -np_vector[0] / norm

        elif not dx_positive and not dy_positive:
            self.quadrant = 2.0
            self.angle_measure = -np_vector[1] / norm

        else:
            self.quadrant = 3.0
            self.angle_measure = np_vector[0] / norm

        self.value = self.quadrant + self.angle_measure


class Vertex:
    # defining static attributes on class
    # TODO optimize memory usage
    # TODO link edges and neighbours already during creation?
    # __slots__ = ['coordinate', ]

    coordinates = None
    is_extremity = False

    # a container for temporally storing shifted coordinates
    coordinates_translated = None
    angle_representation = None
    distance_to_origin = None

    def __init__(self, coordinates):
        self.coordinates = np.array(coordinates)

    def translate(self, new_origin):
        # TODO lazy evaluation! how to know if still up to date?!
        # store the coordinate value of the point relative to the new origin vector
        self.coordinates_translated = self.coordinates - new_origin.coordinates
        self.distance_to_origin = np.linalg.norm(self.coordinates_translated)
        if self.distance_to_origin == 0:
            # the coordinages of the origin and this vertex are equal
            # an angle is not defined in this case!
            self.angle_representation = None
        else:
            self.angle_representation = AngleRepresentation(self.coordinates_translated)


class PolygonVertex(Vertex):
    # TODO how to link to polygon? needed?
    # TODO link real polygon or remove ID
    polygon_ID = None
    edge1 = None
    edge2 = None
    neighbours = [None, None]

    def __init__(self, polygon_ID, coordinates):
        super(PolygonVertex, self).__init__(coordinates)
        self.polygon_ID = polygon_ID

    # TODO setter
    def declare_extremity(self):
        self.is_extremity = True

    def set_edge1(self, e1):
        self.edge1 = e1
        # ordering is important! the numbering convention has to stay intact!
        self.neighbours[0] = e1.vertex1

    def set_edge2(self, e2):
        self.edge2 = e2
        # ordering is important! the numbering convention has to stay intact!
        self.neighbours[1] = e2.vertex1


class Edge:
    vertex1 = None
    vertex2 = None

    def __init__(self, vertex1, vertex2):
        self.vertex1 = vertex1
        self.vertex2 = vertex2


class Polygon:
    is_hole: bool = False
    length: int = None
    vertices: List[PolygonVertex] = []
    edges: List[Edge] = []
    extremities: List[PolygonVertex] = None
    coordinates = None  # np 2D array

    def find_extremities(self):
        """
        identify all vertices with an inside angle of > 180 degree ('extremities')
        expected edge numbering:
            outer boundary polygon: counter clockwise
            holes: clockwise
        :return:
        """
        self.extremities = []
        # extremity_indices = []
        # extremity_index = -1
        vertex1 = self.vertices[-2]
        p1 = vertex1.coordinates
        vertex2 = self.vertices[-1]
        p2 = vertex2.coordinates

        for vertex3 in self.vertices:

            p3 = vertex3.coordinates
            # since consequent vertices are not permitted to be equal,
            #   the angle representation of the difference is well defined
            if (AngleRepresentation(p3 - p2).value - AngleRepresentation(p1 - p2).value) % 4 < 2.0:
                # basic idea:
                #   - translate the coordinate system to have p2 as origin
                #   - compute the angle representations of both vectors representing the edges
                #   - "rotate" the coordinate system (equal to deducting) so that the p1p2 representation is 0
                #   - check in which quadrant the p2p3 representation lies
                # %4 because the quadrant has to be in [0,1,2,3] (representation in [0:4[)
                # if the representation lies within quadrant 0 or 1 (<2.0), the inside angle
                #   (for boundary polygon inside, for holes outside) between p1p2p3 is > 180 degree
                # then p2 = extremity
                vertex2.declare_extremity()
                self.extremities.append(vertex2)

            # move to the next point
            # vertex1=vertex2
            vertex2 = vertex3
            p1 = p2
            p2 = p3

    def __init__(self, coordinate_list, is_hole):

        # store the coordinates in the format suiting the inside_polygon() function
        self.coordinates = np.array(coordinate_list)

        self.is_hole = is_hole
        self.length = len(coordinate_list)
        if self.length < 3:
            raise ValueError('This is not a valid polygon:', coordinate_list, '# edges:', self.length)

        self.vertices = [PolygonVertex(id(self), coordinate) for coordinate in coordinate_list]

        vertex1 = self.vertices[-1]
        for vertex2 in self.vertices:
            edge = Edge(vertex1, vertex2)
            # ordering is important! the numbering convention has to stay intact!
            vertex1.set_edge2(edge)
            vertex2.set_edge1(edge)
            self.edges.append(edge)
            vertex1 = vertex2

        self.find_extremities()

    def translate(self, new_origin: Vertex):
        for vertex in self.vertices:
            vertex.translate(new_origin)


class DirectedHeuristicGraph:
    # TODO better performance when working with just id instead of full vertex class?!
    # TODO but keep coords etc. for drawing graph later on...

    distances: dict = {}
    neighbours: dict = {}
    all_nodes: set = set()

    # the heuristic must NEVER OVERESTIMATE the actual cost (here: actual shortest distance)
    # <=> must always be lowest for node with the POSSIBLY lowest cost
    # <=> heuristic is LOWER BOUND for the cost
    # the heuristic here: distance to the goal node (is always the shortest possible distance!)
    heuristic: dict = {}
    goal_node: Vertex = None

    def __init__(self, nodes):
        self.all_nodes.update(nodes)

    # TODO getter setter. make non accesible!

    def get_all_nodes(self):
        return self.all_nodes

    def get_neighbours(self):
        return self.neighbours.items()

    def get_neighbours_of(self, node):
        return self.neighbours.get(node, set())

    def get_distance(self, node1, node2):
        # directed edges: just one direction is being stored
        try:
            return self.distances[(node1, node2)]
        except KeyError:
            return None

    def get_heuristic(self, node):
        return self.heuristic[node]

    def _existing_edges_from(self, node1):
        # optimisation: when goal node is reachable return it first (-> a star search terminates)
        if self.goal_node in self.neighbours[node1]:
            yield self.goal_node, self.get_distance(node1, self.goal_node)  # not return!
        for node2 in self.neighbours[node1]:
            yield node2, self.get_distance(node1, node2)

    #
    # def get_edges_from(self,node1):
    #     return {(node2, self.get_distance(node1, node2)) for node2 in self.neighbours[node1]}

    def add_directed_edge(self, node1, node2, distance):
        self.neighbours.setdefault(node1, set()).add(node2)
        self.distances[(node1, node2)] = distance
        self.all_nodes.add(node1)
        self.all_nodes.add(node2)

    def add_undirected_edge(self, node1, node2, distance):
        self.add_directed_edge(node1, node2, distance)
        self.add_directed_edge(node2, node1, distance)

    def add_multiple_undirected_edges(self, node1, node_distance_iter):
        for node2, distance in node_distance_iter:
            self.add_undirected_edge(node1, node2, distance)

    def add_multiple_directed_edges(self, node1, node_distance_iter):
        for node2, distance in node_distance_iter:
            self.add_directed_edge(node1, node2, distance)

    def set_goal_node(self, goal_node):
        assert goal_node in self.all_nodes  # has no outgoing edges -> no neighbours
        self.goal_node = goal_node
        # compute heuristic for all
        for node in self.neighbours.keys():
            self.heuristic[node] = np.linalg.norm(node.coordinates - goal_node.coordinates)
