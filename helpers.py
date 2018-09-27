import numpy as np

'''
requirements for the data:
 - no self intersections
 - no vertices with identical coordinates, even with holes and boundary polygon 
    (this would make visibility detection impossible)
 - expected edge numbering:
    outer boundary polygon: counter clockwise
    holes: clockwise
    
'''

"""
:param self: a list of numpy 2D coordinates (arrays, vectors) representing the edges of a polygon.

"""


def inside_polygon(x, y, coords):
    # TODO should also return true for point on the polygon edges?!
    contained = False
    # the edge from the last to the first point is checked first
    i = -1
    y1 = coords[1][-1]
    y_gt_y1 = y > y1
    for y2 in coords[1]:
        y_gt_y2 = y > y2
        if y_gt_y1:
            if not y_gt_y2:
                x1 = coords[0][i]
                x2 = coords[0][i + 1]
                # only crossings "right" of the point should be counted
                x1GEx = x <= x1
                x2GEx = x <= x2
                # compare the slope of the line [p1-p2] and [p-p2]
                # depending on the position of p2 this determines whether the polygon edge is right or left of the point
                # to avoid expensive division the divisors (of the slope dy/dx) are brought to the other side
                # ( dy/dx > a  ==  dy > a * dx )
                if (x1GEx and x2GEx) or ((x1GEx or x2GEx) and (y2 - y) * (x2 - x1) <= (y2 - y1) * (x2 - x)):
                    contained = not contained

        else:
            if y_gt_y2:
                x1 = coords[0][i]
                x2 = coords[0][i + 1]
                # only crossings "right" of the point should be counted
                x1GEx = x <= x1
                x2GEx = x <= x2
                if (x1GEx and x2GEx) or ((x1GEx or x2GEx) and (y2 - y) * (x2 - x1) >= (y2 - y1) * (x2 - x)):
                    contained = not contained

        y1 = y2
        y_gt_y1 = y_gt_y2
        i += 1

    # TODO if not contained, pruefen ob direkt auf linie!
    return contained


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
    ID = None
    coordinates = None
    is_extremity = False

    # a container for temporally storing shifted coordinates
    coordinates_translated = None
    angle_representation = None
    distance_to_origin = None

    def __init__(self, ID, coordinates):
        self.ID = ID
        self.coordinates = np.array(coordinates)

    def translate(self, new_origin):
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
    polygon_ID = None
    edge1 = None
    edge2 = None

    def __init__(self, polygon_ID, ID, coordinates):
        super(PolygonVertex, self).__init__(ID, coordinates)
        self.polygon_ID = polygon_ID

    def declare_extremity(self):
        self.is_extremity = True

    def neighbours(self):
        # ordering is important! the numbering convention has to stay intact!
        return self.edge1.vertex1, self.edge2.vertex2


class Edge:
    polygon_ID = None
    ID = None
    vertex1 = None
    vertex2 = None

    def __init__(self, polygon_ID, ID, vertex1, vertex2):
        self.polygon_ID = polygon_ID
        self.ID = ID
        self.vertex1 = vertex1
        self.vertex2 = vertex2


class Polygon:
    ID = None
    is_hole = False
    length = None
    vertices = []
    edges = []
    extremities = None
    coordinates = None

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
                # extremity_indices.append(extremity_index % self.length)

            # move to the next point
            # vertex1=vertex2
            vertex2 = vertex3
            p1 = p2
            p2 = p3
            # extremity_index += 1

        # return extremity_indices

    def __init__(self, coordinate_list, ID, is_hole):

        # store the coordinates in the format suiting the inside_polygon() function
        zipped_coords = list(zip(*coordinate_list))
        self.coordinates = np.array(zipped_coords[0]), np.array(zipped_coords[1])

        self.ID = ID
        self.is_hole = is_hole
        self.length = len(coordinate_list)
        if self.length < 3:
            raise ValueError('This is not a valid polygon:', coordinate_list, '# edges:', self.length)

        self.vertices = [PolygonVertex(self.ID, vertex_id, coordinate) for (vertex_id, coordinate) in
                         enumerate(coordinate_list)]

        vertex1 = self.vertices[-1]
        for edge_id, vertex2 in enumerate(self.vertices):
            edge = Edge(self.ID, edge_id, vertex1, vertex2)
            vertex1.edge2 = edge
            vertex2.edge1 = edge
            self.edges.append(edge)
            vertex1 = vertex2

        self.find_extremities()

    def translate(self, new_origin):
        for vertex in self.vertices:
            vertex.translate(new_origin)


# preparation of the map
# TODO fct. for ensuring clockwise and counter clockwise numbering
# take 2 edges (3 vertices) compute angle between them and


# TODO class for keeping preloaded map
class Map:
    boundary_polygon = None
    holes = None
    all_vertices = None
    all_edges = None
    all_extremities = None

    boundary_extremities = None
    hole_extremities = None
    prepared = False

    def store(self, boundary_coordinates, hole_coordinates, validation=False):
        self.prepared = False
        # 'loading the map

        if validation:
            # TODO validating polygons
            # TODO rectification
            #  - no self intersections
            #  - no neighbouring IDentical edges
            #  - expected edge numbering:
            #     outer boundary polygon: counter clockwise
            #     holes: clockwise
            pass

        self.boundary_polygon = Polygon(boundary_coordinates, 0, is_hole=False)
        self.all_edges = self.boundary_polygon.edges
        self.all_vertices = self.boundary_polygon.vertices
        self.all_extremities = self.boundary_polygon.extremities
        polygon_ID = 1
        self.holes = []
        for coordinates in hole_coordinates:
            hole_polygon = Polygon(coordinates, polygon_ID, is_hole=True)
            self.holes.append(hole_polygon)
            self.all_extremities += hole_polygon.extremities
            self.all_edges.append(hole_polygon.edges)
            self.all_vertices.append(hole_polygon.vertices)
            polygon_ID += 1

    def translate(self, new_origin):
        self.boundary_polygon.translate(new_origin)
        for hole in self.holes:
            hole.translate(new_origin)

    def find_visible(self, query_vertex, candidate_vertices):
        """
        :param query_vertex: the vertex for which the visibility to the vertices should be checked
        :param candidate_vertices: the set of all vertices which should be checked for visibility
        :return: a set of tuples of all vertices visible from the query vertex and the corresponding distance
        """

        visible_vertices = set()
        if len(candidate_vertices) == 0:
            return visible_vertices

        # TODO also allow non extremity polygon vertices
        # TODO also allow same coordinate vertices

        if type(query_vertex) == PolygonVertex:
            # when the query vertex belongs to a polygon
            # direct neighbours of the query vertex are visible
            # neighbouring vertices are reachable with the distance equal to the edge length
            n1, n2 = query_vertex.neighbours()
            if n1 in candidate_vertices:
                visible_vertices.add(n1)
                candidate_vertices.remove(n1)
            if n2 in candidate_vertices:
                visible_vertices.add(n2)
                candidate_vertices.remove(n2)
            if len(candidate_vertices) == 0:
                return visible_vertices

        # compute the angle representations and distances for all vertices respective to the query point
        self.translate(new_origin=query_vertex)

        def find_within_range(repr1, repr2, vertex_set, angle_range_less_180):

            # filter out all vertices whose representation lies within the range between
            #   the two given angle representations
            # vertices with the same representation should NOT be returned! (they can be visible!)
            # which range ('clockwise' or 'counter-clockwise') should be checked is determined by:
            #   - query angle (range) is < 180deg or not (>= 180deg)

            repr_diff = abs(repr1 - repr2)
            if repr_diff == 0.0:
                return set()

            min_repr_val = min(repr1, repr2)
            max_repr_val = max(repr1, repr2)  # = min_angle + angle_diff

            def lies_within(vertex):
                # vertices with the same representation should NOT be returned!
                return min_repr_val < vertex.angle_representation.value < max_repr_val

            # when the range contains the 0.0 value (transition from 3.99... -> 0.0)
            # it is easier to check if a representation does NOT lie within this range
            # -> filter_fct = not_within
            def not_within(vertex):
                # vertices with the same representation should NOT be returned!
                return not (min_repr_val <= vertex.angle_representation.value <= max_repr_val)

            if repr_diff < 2.0:
                # angle < 180 deg
                if angle_range_less_180:
                    filter_fct = lies_within
                else:
                    # the actual range to search is from min_val to max_val, but clockwise!
                    filter_fct = not_within

            elif repr_diff == 2.0:
                # angle == 180deg
                # for some query points it is unknown if they lie on an edge
                # an angle of 180deg might appear even if it is expected to be <180deg
                # if angle_range_less_180:
                #     raise ValueError(repr1, repr2, repr_diff)

                # which range to filter is determined by the order of the points
                # since the polygons follow a numbering convention,
                # the 'left' side of p1-p2 always lies inside the map
                # -> filter out everything on the right side (='behind')
                if repr1 < repr2:
                    filter_fct = lies_within
                else:
                    filter_fct = not_within

            else:
                # angle > 180deg
                if angle_range_less_180:
                    filter_fct = not_within
                else:
                    filter_fct = lies_within

            return set(filter(filter_fct, vertex_set))

        if type(query_vertex) == PolygonVertex:
            # eliminate all vertices 'behind' the query point from the candidate set
            # when the query vertex is an extremity the 'outer' angle is < 180 degree
            # then the difference between the angle representation of the two edges has to be < 2.0
            # all vertices within the angle of the two neighbouring edges are not visible (no candidates!)
            # vertices with the same angle representation might be visible!
            repr1 = n1.angle_representation.value
            repr2 = n2.angle_representation.value
            candidate_vertices.difference_update(
                find_within_range(repr1, repr2, candidate_vertices, angle_range_less_180=query_vertex.is_extremity))
            if len(candidate_vertices) == 0:
                return visible_vertices

        # all edges have to be checked
        edges_to_check = set(self.all_edges)
        if type(query_vertex) == PolygonVertex:
            # except the neighbouring edges (handled already)
            edges_to_check.remove(query_vertex.edge1)
            edges_to_check.remove(query_vertex.edge2)

        priority_edges = set()

        # goal: eliminating all vertices lying 'behind' any edge
        while len(candidate_vertices) > 0 and len(edges_to_check) > 0:
            # check prioritized items first
            try:
                edge = priority_edges.pop()
                edges_to_check.remove(edge)
            except KeyError:
                edge = edges_to_check.pop()

            vertices_to_check = candidate_vertices.copy()
            # the vertices belonging to the edge itself (its vertices) must not be checked.
            # use discard() instead of remove() to not raise an error (they might not be candidates)
            vertices_to_check.discard(edge.vertex1)
            vertices_to_check.discard(edge.vertex2)
            if len(vertices_to_check) == 0:
                continue

            if edge.vertex1.distance_to_origin == 0.0:
                # vertex1 has the same coordinates as the query vertex
                # (but does not belong to the same polygon, not identical!)
                # its angle representation is not defined (no line segment from vertex1 to query vertex!)
                # but everything between its neighbouring edges is not visible
                v1, v2 = edge.vertex1.neighbours
                range_less_180 = edge.vertex1.is_extremity
                # TODO do not check neighbour edge
            elif edge.vertex2.distance_to_origin == 0.0:
                v1, v2 = edge.vertex2.neighbours
                range_less_180 = edge.vertex2.is_extremity
                # TODO do not check neighbour edge
            else:
                v1, v2 = edge.vertex1, edge.vertex2
                range_less_180 = True

            # for all candidate edges check if there are any candidate vertices (besides the ones belonging to the edge)
            #   within this angle range
            repr1 = v1.angle_representation.value
            repr2 = v2.angle_representation.value
            # the "view range" of an edge from a query point (spanned by the two vertices of the edge)
            #   is normally < 180deg,
            # but in the case that the query point directly lies on the edge the angle is 180deg
            vertices_to_check = find_within_range(repr1, repr2, vertices_to_check, angle_range_less_180=range_less_180)
            if len(vertices_to_check) == 0:
                continue

            # if a vertex is farther away from the query point than both vertices of the edge,
            #    it surely lies behind the edge
            max_distance = max(edge.vertex1.distance_to_origin, edge.vertex2.distance_to_origin)
            vertices_behind = set(filter(lambda extr: extr.distance_to_origin > max_distance, vertices_to_check))
            # they do not have to be checked
            vertices_to_check.difference_update(vertices_behind)
            if len(vertices_to_check) == 0:
                # also done later, only needed if skipping this edge
                candidate_vertices.difference_update(vertices_behind)
                continue

            # if the edge is closer than both vertices it surely lies in front (
            min_distance = min(edge.vertex1.distance_to_origin, edge.vertex2.distance_to_origin)
            vertices_in_front = set(
                filter(lambda extr: extr.distance_to_origin < min_distance, vertices_to_check))
            # they do not have to be checked (safes computation)
            vertices_to_check.difference_update(vertices_in_front)

            # in any other case it has to be tested if the line segment from query point (=origin) to the vertex v
            #    has an intersection with the current edge p1---p2
            # vertices directly on the edge are allowed (not eliminated)!
            p1 = edge.vertex1.coordinates_translated
            p2 = edge.vertex2.coordinates_translated

            def lies_behind(p1, p2, v):
                # solve the set of equations
                # (p2-p1) lambda + (p1) = (v) mu
                #  in matrix form A x = b:
                # [(p1-p2) (v)] (lambda, mu)' = (p1)
                # because the vertex lies within the angle range between the two edge vertices
                #    (together with the other conditions on the polygons)
                #   this set of linear equations is always solvable (the matrix is regular)
                A = np.array([p1 - p2, v])
                b = np.array(p1)
                x = np.linalg.solve(A, b)
                # vertices on the edge are possibly visible! ( < not <=)
                return x[1] < 1.0

            for vertex in vertices_to_check:
                if lies_behind(p1, p2, vertex.coordinates_translated):
                    vertices_behind.add(vertex)
                else:
                    vertices_in_front.add(vertex)

            # vertices behind any edge are not visible
            candidate_vertices.difference_update(vertices_behind)
            if len(candidate_vertices) == 0:
                return visible_vertices

            # check the neighbouring edges of all vertices which lie in front of the edge next first
            # (prioritize them)
            # they lie in front and hence will eliminate other vertices faster
            # the fewer vertex candidates remain, the faster the procedure
            # TODO improve: increase priority every time and draw highest priority items
            #   but this involves sorting (expensive for large polygons!)
            #   idea: work with a list of sets, add new set for higher priority
            for e in vertices_in_front:
                # only add the neighbour edges to the priority set if they still have to be checked!
                priority_edges.update(edges_to_check.intersection({e.edge1, e.edge2}))

        # all remaining vertices are visible
        visible_vertices.update(candidate_vertices)
        # TODO optimization: when two vertices are visible and have the same angle representation,
        #   only the closest should be considered
        # would reduce the number of edges in the graph (-> speed up finding the shortest path )
        # but requires some more effort to ensure that really no overlapping edges are present in the graph
        # Because the candidate set is not always complete, previously checked visible vertices might not be present.
        # deleting overlapping edges could only be done at the end of graph construction (when all edges are known)

        # return a set of tuples: (vertex, distance)
        return {(e, e.distance_to_origin) for e in visible_vertices}

    def prepare(self):
        # compute the all directly reachable extremities based on visibility
        # compute the distances between all directly reachable extremities
        # TODO ? compute shortest paths between all directly reachable extremities
        # store as graph
        # TODO matrix? sparse matrix data type? look up what a star implementation needs

        # precompute
        # construct graph of visible (=directly reachable) extremities
        # fixed ordering of the extremities (IDs) through the list self.all_extremities
        graph = {e: set() for e in self.all_extremities}
        extremities_to_check = set(self.all_extremities)
        for extremity1 in self.all_extremities:
            # extremities can only be visible to each other (bi-directional relation -> undirected graph)
            # so in the future do not check extremities which have been checked already
            # this would only give the same result when algorithms are correct
            # TODO test if relation is really bidirectional (find_visible(x,y) = find_visible(y,x))
            # also the extremity itself must not be checked when looking for visible neighbours
            extremities_to_check.remove(extremity1)
            additionally_visible = self.find_visible(extremity1, extremities_to_check)
            graph[extremity1] |= additionally_visible
            for extremity2, distance in additionally_visible:
                graph[extremity2] |= {(extremity1, distance)}

        self.prepared = True
        # TODO safe graph
        print(graph)

    def find_shortest_path(self, start_coordinates, goal_coordinates):
        # path planning query:
        # make sure start and goal are within the boundary polygon and outside of all holes

        def within_map(query_coords):
            # within the boundary polygon and outside of all holes
            x, y = query_coords
            if not inside_polygon(x, y, self.boundary_polygon.coordinates):
                return False
            for hole in self.holes:
                if inside_polygon(x, y, hole.coordinates):
                    return False
            return True

        if not (within_map(start_coordinates) and within_map(goal_coordinates)):
            raise ValueError('start or goal do not lie within the map')

        if start_coordinates == goal_coordinates:
            # start and goal are identical and can be reached instantly
            return ([start_coordinates, goal_coordinates], 0.0)

        # TODO might be directly visible!

        # check if start and goal nodes have identical coordinates with one of the vertices
        # TODO optimisations for visibility test in this case
        # TODO watch out. might not be extremity. not 180 angle!
        new_start_vertex = True
        new_goal_vertex = True
        new_start_node = True
        new_goal_node = True
        for v in self.all_vertices:
            if v.coordinates == start_coordinates:
                start_vertex = v
                new_start_vertex = False
                if v.is_extremity:
                    # do not add new node in graph
                    new_start_node = False

            if v.coordinates == goal_coordinates:
                goal_vertex = v
                new_goal_vertex = False
                if v.is_extremity:
                    # do not add new node in graph
                    new_goal_node = False
                break

        # compute the all directly reachable extremities from start and goal based on visibility
        # TODO
        if new_start_vertex:
            start_vertex = Vertex(-1, start_coordinates)
        if new_goal_vertex:
            goal_vertex = Vertex(-1, goal_coordinates)

        if new_start_node:
            # TODO copy graph?!
            start_visibles = self.find_visible(start_vertex, set(self.all_extremities))
            # TODO add to graph

        if new_goal_node:
            goal_visibles = self.find_visible(goal_vertex, set(self.all_extremities))
            # TODO add to graph

        # temporary graph

        # a star algorithm
        # TODO more clever approach


if __name__ == "__main__":
    # counter clockwise edge numbering!
    # polygon1 = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    # polygon1 = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    polygon1 = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # clockwise numbering!
    # holes1 = []
    holes1 = [[(3.0, 7.0), (5.0, 9.0), (5.0, 7.0), ], ]

    map = Map()
    map.store(polygon1, holes1)
    print(map.all_extremities)
    map.prepare()

# TODO polygon consisting of 2 vertices?!
