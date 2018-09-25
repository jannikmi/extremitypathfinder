import numpy as np

'''
requirements for the data:
 - no self intersections
 - no neighbouring IDentical edges
 - expected edge numbering:
    outer boundary polygon: counter clockwise
    holes: clockwise
    
'''

"""
:param self: a list of numpy 2D coordinates (arrays, vectors) representing the edges of a polygon.

"""


def inside_polygon(x, y, coords):
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
    but it its bijective and monotonous

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
    # TODO maybe link real polygon not just ID
    polygon_ID = None
    ID = None
    coordinates = None
    # a container for temporally storing shifted coordinates
    coordinates_translated = None
    angle_representation = None
    distance_to_origin = None
    is_extremity = False
    edge1 = None
    edge2 = None

    def __init__(self, polygon_ID, ID, coordinates):
        self.polygon_ID = polygon_ID
        self.ID = ID
        self.coordinates = np.array(coordinates)

    def declare_extremity(self):
        self.is_extremity = True

    def translate(self, new_origin):
        # store the coordinate value of the point relative to the new origin vector
        self.coordinates_translated = self.coordinates - new_origin
        self.angle_representation = AngleRepresentation(self.coordinates_translated)
        self.distance_to_origin = np.linalg.norm(self.coordinates_translated)

    def neighbours(self):
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
        vertix1 = self.vertices[-2]
        p1 = vertix1.coordinates
        vertix2 = self.vertices[-1]
        p2 = vertix2.coordinates

        for vertix3 in self.vertices:
            p3 = vertix3.coordinates
            if AngleRepresentation(p3 - p2).value - AngleRepresentation(p1 - p2).value % 4 < 2.0:
                # basic idea:
                #   - translate the coordinate system to have p2 as origin
                #   - compute the angle representations of both vectors representing the edges
                #   - "rotate" the coordinate system (equal to deducting) so that the p1p2 representation is 0
                #   - check in which quadrant the p2p3 representation lies
                # %4 because the quadrant has to be in [0,1,2,3] (representation in [0:4[)
                # if the representation lies within quadrant 0 or 1 (<2.0), the inside angle
                #   (for boundary polygon inside, for holes outside) between p1p2p3 is > 180 degree
                # then p2 = extremity
                vertix2.declare_extremity()
                self.extremities.append(vertix2)
                # extremity_indices.append(extremity_index % self.length)

            # move to the next point
            # vertix1=vertix2
            vertix2=vertix3
            p1 = p2
            p2 = p3
            # extremity_index += 1

        # return extremity_indices

    def __init__(self, coordinate_list, ID, is_hole):
        self.ID = ID
        self.is_hole = is_hole
        self.length = len(coordinate_list)
        if self.length < 3:
            raise ValueError('This is not a valid polygon:', coordinate_list, '# edges:', self.length)

        for vertex_id, coordinate in enumerate(coordinate_list):
            self.vertices.append(Vertex(self.ID, vertex_id, coordinate))

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
    all_extremities = None

    boundary_extremities = None
    hole_extremities = None

    def store(self, boundary_coordinates, holes, validation=False):
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
        for hole in holes:
            hole_polygon = Polygon(hole, polygon_ID, is_hole=True)
            self.holes.append(hole_polygon)
            self.all_extremities += hole_polygon.extremities
            self.all_edges.append(hole.edges)
            self.all_vertices.append(hole.vertices)
            polygon_ID += 1

        # compute the all directly reachable extremities based on visibility
        # compute the distances between all directly reachable extremities
        # TODO ? compute shortest paths between all directly reachable extremities
        # store as graph
        # TODO matrix? sparse matrix data type? look up what a star implementation needs

    def prepare(self):
        # find and store the extremities of the boundary polygon and all the holes
        # self.boundary_extremities = find_extremities(self.boundary_polygon)
        #
        # self.hole_extremities = []
        # for hole in self.holes:
        #     self.hole_extremities.append(find_extremities(hole))

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

    def translate(self, new_origin):
        self.boundary_polygon.translate(new_origin)
        for hole in self.holes:
            hole.translate(new_origin)

    def find_visible(self, query_vertex, candidate_extremities):
        """
        :param query_vertex: the vertex for which the visibility to the extremities should be checked
        :param candidate_extremities: the set of all extremities which should be checked for visibility
        :return: a set of tuples of all extremities visible from the query vertex and the corresponding distance
        """

        visible_extremities = set()
        if len(candidate_extremities) == 0:
            return visible_extremities

        if query_vertex.is_extremity:
            # when the query point is an extremity itself...
            # the vertex itself should not be considered
            # this is already being done before calling .find_visible()
            # candidate_extremities.remove(query_vertex)

            # direct neighbours of the query extremity are visible
            # neighbouring extremities are reachable with the distance equal to the edge length
            n1, n2 = query_vertex.neighbours()
            if n1.is_extremity:
                visible_extremities.add(n1)
                candidate_extremities.discard(n1)  # maybe was not even a candidate, do not use remove()!
            if n2.is_extremity:
                visible_extremities.add(n2)
                candidate_extremities.discard(n2)
            if len(candidate_extremities) == 0:
                return visible_extremities

        # compute the angle representations and distances for all vertices respective to the query point
        self.translate(new_origin=query_vertex)

        def find_within_range(repr1, repr2, extremity_set):
            # eliminates all extremities whose representation lies within the given two angle representations
            # extremities with the same representation are allowed

            repr_diff = abs(repr1 - repr2)
            min_repr_val = min(repr1, repr2)
            max_repr_val = max(repr1, repr2)  # = min_angle + angle_diff

            def lies_within(extremity):
                return min_repr_val <= extremity.angle_representation.value <= max_repr_val

            def not_within(extremity):
                return not (min_repr_val < extremity.angle_representation.value < max_repr_val)

            # for both use cases:
            #   - "view range" of an edge from a query point (spanned by the two vertices of the edge)
            #   - "outer angle" of an extremity (spanned by the 2 neighbouring edges)
            # the angle range to eliminate is always <= 180 (= 2.0 in angle representation)
            if repr_diff > 2.0:
                # the actual range to search is from min_val to max_val, but clockwise!
                # it hence contains the 0.0 value (transition from 3.99... -> 0.0)
                # it is easier to check if a representation does NOT lie within this range
                filter_fct = not_within
            else:
                filter_fct = lies_within

            return set(filter(filter_fct, extremity_set))

        if query_vertex.is_extremity:
            # eliminate all extremities 'behind' the query point from the candidate set
            # since the query point is an extremity the 'outer' angle is < 180 degree
            # hence the difference between the angle representation of the two edges has to be < 2.0
            # all extremities within the angle of the two neighbouring edges are not visible (no candidates!)
            # extremities with the same angle might be visible!
            repr1 = n1.angle_representation.value
            repr2 = n2.angle_representation.value
            candidate_extremities.difference_update(find_within_range(repr1, repr2, candidate_extremities))
            if len(candidate_extremities) == 0:
                return visible_extremities

        # all edges except the neighbouring edges have to be checked
        edges_to_check = set(self.all_edges)

        if query_vertex.is_extremity:
            edges_to_check.remove(query_vertex.edge1)
            edges_to_check.remove(query_vertex.edge2)

        priority_edges = set()

        # goal: eliminating all extremities lying 'behind' any edge
        # at least one extremity has to be visible in every polygon
        # but since the initial candidate set must not be complete, check until no candidates or edges are left to check
        while len(candidate_extremities) > 0 and len(edges_to_check) > 0:
            # check prioritized items first
            try:
                edge = priority_edges.pop()
                edges_to_check.remove(edge)
            except KeyError:
                edge = edges_to_check.pop()

            extremities_to_check = candidate_extremities.copy()
            # the extremities belonging to the edge itself (its vertices) must not be checked.
            # use discard() instead of remove() to not raise an error (they might not be candidates)
            extremities_to_check.discard(edge.vertex1)
            extremities_to_check.discard(edge.vertex2)
            if len(extremities_to_check) == 0:
                continue

            # for all candidate edges check if there are extremities (besides the ones belonging to the edge)
            #   lying within this angle
            repr1 = edge.vertex1.angle_representation.value
            repr2 = edge.vertex2.angle_representation.value
            extremities_to_check = find_within_range(repr1, repr2, extremities_to_check)
            if len(extremities_to_check) == 0:
                continue

            # if a vertex is farther away from the query point than both vertices of the edge,
            #    it surely lies behind the edge
            max_distance = max(edge.vertex1.distance_to_origin, edge.vertex2.distance_to_origin)
            extremities_behind = set(filter(lambda extr: extr.distance_to_origin > max_distance, extremities_to_check))
            # they do not have to be checked
            extremities_to_check.difference_update(extremities_behind)
            if len(extremities_to_check) == 0:
                # also done later, only needed if skipping this edge
                candidate_extremities.difference_update(extremities_behind)
                continue

            # if the edge is closer than both vertices it surely lies in front (
            min_distance = min(edge.vertex1.distance_to_origin, edge.vertex2.distance_to_origin)
            extremities_in_front = set(
                filter(lambda extr: extr.distance_to_origin < min_distance, extremities_to_check))
            # they do not have to be checked (safes computation)
            extremities_to_check.difference_update(extremities_in_front)

            # in any other case it has to be tested if the line segment from query point (=origin) to the extremity e
            #    has an intersection with the current edge p1---p2
            # extremities directly on the edge are allowed (not eliminated)!
            p1 = edge.vertex1.coordinates_translated
            p2 = edge.vertex2.coordinates_translated

            def lies_behind(p1, p2, e):
                # solve the set of equations
                # (p2-p1) lambda + (p1) = (e) mu
                #  in matrix form A x = b:
                # [(p1-p2) (q)] (lambda, mu)' = (p1)
                # because the extremity lies within the angle range between the two edge vertices
                #    (together with the other conditions on the polygons)
                #   this set of linear equations is always solvable (the matrix is regular)
                A = np.array([p1 - p2, e])
                b = np.array(p1)
                x = np.linalg.solve(A, b)
                # extremities on edge are possibly visible! ( < not <=)
                return x[1] < 1.0

            for extremity in extremities_to_check:
                if lies_behind(p1, p2, extremity.coordinates_translated):
                    extremities_behind.add(extremity)
                else:
                    extremities_in_front.add(extremity)

            # extremities behind any edge are not visible
            candidate_extremities.difference_update(extremities_behind)
            if len(candidate_extremities) == 0:
                return visible_extremities

            # check the neighbouring edges of all extremities which lie in front of the edge next first
            # (prioritize them)
            # they lie in front and hence will eliminate other extremities faster
            # the fewer extremity candidates remain, the faster the procedure
            # TODO improve: increase priority every time and draw highest priority items
            #   but this involves sorting (expensive for large polygons!)
            #   idea: work with a list of sets, add new set for higher priority
            for e in extremities_in_front:
                # only add the neighbour edges to the priority set if they still have to be checked!
                priority_edges.update(edges_to_check.intersection({e.edge1, e.edge2}))

        # all remaining extremities are visible
        visible_extremities.update(candidate_extremities)
        # TODO optimization: when two extremities are visible and have the same angle representation,
        #   only the closest should be considered
        # would reduce the number of edges in the graph (-> speed up finding the shortest path )
        # but requires some more effort to ensure that really no overlapping edges are present in the graph
        # Because the candidate set is not always complete, previously checked visible extremities might not be present.
        # deleting overlapping edges could only be done at the end of graph construction (when all edges are known)

        # return a set of tuples: (extremity, distance)
        return {(e, e.distance_to_origin) for e in visible_extremities}

# path planning query:
# make sure start and goal are within the boundary polygon and outside of all holes

# compute the all directly reachable extremities from start and goal based on visibility

# a star algorithm
# TODO more clever approach

if __name__ == "__main__":
    polygon1 = [(0.0,0.0),(10.0,0.0),(10.0,10.0),(0.0,10.0)]
    holes1 = []

    map = Map()
    map.store(polygon1,holes1)
    print(map.all_extremities)

