from copy import copy

from a_star import modified_a_star
from gui import draw_loaded_map, draw_prepared_map, draw_graph, draw_only_path, draw_with_path
from helper_fcts import *
from helper_classes import *


# TODO verbosity option

class Map:
    # class for keeping preloaded map for consecutive path queries
    boundary_polygon: Polygon = None
    holes: List[Polygon] = None
    all_edges: List[Edge] = None
    all_vertices: List[Vertex] = None
    all_extremities: List[Vertex] = None

    # boundary_extremities = None
    # hole_extremities = None
    prepared: bool = False
    graph: DirectedHeuristicGraph = {}

    def store(self, boundary_coordinates, list_of_hole_coordinates, validation=False, export_plots=False):
        self.prepared = False
        # 'loading the map
        boundary_coordinates = np.array(boundary_coordinates)
        list_of_hole_coordinates = [np.array(hole_coords) for hole_coords in list_of_hole_coordinates]
        if validation:
            validate(boundary_coordinates, list_of_hole_coordinates)

        self.boundary_polygon = Polygon(boundary_coordinates, is_hole=False)
        # IMPORTANT: make a copy of the list instead of linking to the same list (python!)
        self.all_edges = self.boundary_polygon.edges.copy()
        self.all_vertices = self.boundary_polygon.vertices.copy()
        self.all_extremities = self.boundary_polygon.extremities.copy()
        self.holes = []
        for coordinates in list_of_hole_coordinates:
            hole_polygon = Polygon(coordinates, is_hole=True)
            self.holes.append(hole_polygon)
            self.all_extremities += hole_polygon.extremities
            self.all_edges += hole_polygon.edges
            self.all_vertices += hole_polygon.vertices

        if export_plots:
            draw_loaded_map(self)

    def translate(self, new_origin):
        self.boundary_polygon.translate(new_origin)
        for hole in self.holes:
            hole.translate(new_origin)

    def find_visible(self, query_vertex, candidate_set):
        """
        :param query_vertex: the vertex for which the visibility to the vertices should be checked.
            also non extremity polygon vertices and vertices with the same coordinates are allowed.
        :param candidates: the set of all vertices which should be checked for visibility
        :return: a set of tuples of all vertices visible from the query vertex and the corresponding distance
        """

        visible_vertices = set()
        if len(candidate_set) == 0:
            return visible_vertices

        candidates = candidate_set.copy()
        if type(query_vertex) == PolygonVertex:
            # when the query vertex belongs to a polygon
            # direct neighbours of the query vertex are visible
            # neighbouring vertices are reachable with the distance equal to the edge length
            n1, n2 = query_vertex.neighbours
            if n1 in candidates:
                visible_vertices.add(n1)
                candidates.remove(n1)
            if n2 in candidates:
                visible_vertices.add(n2)
                candidates.remove(n2)
            if len(candidates) == 0:
                return {(e, np.linalg.norm(query_vertex.coordinates - e.coordinates)) for e in visible_vertices}

        # compute the angle representations and distances for all vertices respective to the query point
        self.translate(new_origin=query_vertex)

        def find_within_range(repr1, repr2, vertex_set, angle_range_less_180):
            # filter out all vertices whose representation lies within the range between
            #   the two given angle representations
            # vertices with the same representation should also be returned!
            # they can be visible, but will be ruled out if they lie behind any edges!
            # which range ('clockwise' or 'counter-clockwise') should be checked is determined by:
            #   - query angle (range) is < 180deg or not (>= 180deg)

            repr_diff = abs(repr1 - repr2)
            if repr_diff == 0.0:
                return set()

            min_repr_val = min(repr1, repr2)
            max_repr_val = max(repr1, repr2)  # = min_angle + angle_diff

            def lies_within(vertex):
                # vertices with the same representation should NOT be returned!
                return min_repr_val <= vertex.angle_representation.value <= max_repr_val

            # when the range contains the 0.0 value (transition from 3.99... -> 0.0)
            # it is easier to check if a representation does NOT lie within this range
            # -> filter_fct = not_within
            def not_within(vertex):
                # vertices with the same representation should NOT be returned!
                return not (min_repr_val < vertex.angle_representation.value < max_repr_val)

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
            candidates.difference_update(
                find_within_range(repr1, repr2, candidates, angle_range_less_180=query_vertex.is_extremity))
            if len(candidates) == 0:
                return {(e, e.distance_to_origin) for e in visible_vertices}

        # all edges have to be checked
        edges_to_check = set(self.all_edges)
        if type(query_vertex) == PolygonVertex:
            # except the neighbouring edges (handled already)
            edges_to_check.remove(query_vertex.edge1)
            edges_to_check.remove(query_vertex.edge2)

        priority_edges = set()

        # goal: eliminating all vertices lying 'behind' any edge
        while len(candidates) > 0 and len(edges_to_check) > 0:
            # check prioritized items first
            try:
                edge = priority_edges.pop()
                edges_to_check.remove(edge)
            except KeyError:
                edge = edges_to_check.pop()

            vertices_to_check = candidates.copy()
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
                e1 = edge.vertex1.edge1
                # do not check the other neighbouring edge of vertex1 in the future
                edges_to_check.discard(e1)
                priority_edges.discard(e1)
            elif edge.vertex2.distance_to_origin == 0.0:
                v1, v2 = edge.vertex2.neighbours
                range_less_180 = edge.vertex2.is_extremity
                e1 = edge.vertex2.edge2
                edges_to_check.discard(e1)
                priority_edges.discard(e1)
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
                candidates.difference_update(vertices_behind)
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
                # TODO allowed!?: detect when nodes are identical (x[0] == 0.0 or 1.0 and x[1] == 0.0 or 1.0
                return x[1] < 1.0

            for vertex in vertices_to_check:
                if lies_behind(p1, p2, vertex.coordinates_translated):
                    vertices_behind.add(vertex)
                else:
                    vertices_in_front.add(vertex)

            # vertices behind any edge are not visible
            candidates.difference_update(vertices_behind)
            # if there are no more candidates left. immediately quit checking edges
            if len(candidates) == 0:
                return {(e, e.distance_to_origin) for e in visible_vertices}

            # check the neighbouring edges of all vertices which lie in front of the edge next first
            # (prioritize them)
            # they lie in front and hence will eliminate other vertices faster
            # the fewer vertex candidates remain, the faster the procedure
            # TODO improvement: increase priority every time and draw highest priority items
            #   but this involves sorting (expensive for large polygons!)
            #   idea: work with a list of sets, add new set for higher priority
            for e in vertices_in_front:
                # only add the neighbour edges to the priority set if they still have to be checked!
                if type(e) == PolygonVertex:
                    # only vertices belonging to polygons have neighbours
                    priority_edges.update(edges_to_check.intersection({e.edge1, e.edge2}))

        # all edges have been checked
        # all remaining vertices were not concealed behind any edge and hence are visible
        visible_vertices.update(candidates)

        # return a set of tuples: (vertex, distance)
        return {(e, e.distance_to_origin) for e in visible_vertices}

    def prepare(self, export_plots=False):
        # compute the all directly reachable extremities based on visibility
        # compute the distances between all directly reachable extremities
        # store as graph

        # precompute
        # construct graph of visible (=directly reachable) extremities
        # fixed ordering of the extremities (IDs) through the list self.all_extremities
        self.graph = DirectedHeuristicGraph(self.all_extremities)
        # {e: set() for e in self.all_extremities}
        extremities_to_check = set(self.all_extremities)
        for extremity1 in self.all_extremities:
            # extremities can only be visible to each other (bi-directional relation -> undirected graph)
            # so in the future do not check extremities which have been checked already
            # this would only give the same result when algorithms are correct
            # also the extremity itself must not be checked when looking for visible neighbours
            extremities_to_check.remove(extremity1)
            self.graph.add_multiple_undirected_edges(extremity1, self.find_visible(extremity1, extremities_to_check))

        self.prepared = True
        if export_plots:
            draw_prepared_map(self)
        # TODO pre compute shortest paths between all directly reachable extremities. advantages?!
        # does it really safe computations during query time?!

    def find_shortest_path(self, start_coordinates, goal_coordinates, export_plots=False):
        # path planning query:
        # make sure the map has been loaded and prepared
        if self.boundary_polygon is None:
            raise ValueError('No Polygons have been loaded into the map yet.')
        if not self.prepared:
            self.prepare()

        # make sure start and goal are within the boundary polygon and outside of all holes
        def within_map(query_coords):
            # within the boundary polygon and outside of all holes
            x, y = query_coords
            if not inside_polygon(x, y, self.boundary_polygon.coordinates, border_value=True):
                return False
            for hole in self.holes:
                if inside_polygon(x, y, hole.coordinates, border_value=False):
                    return False
            return True

        if not (within_map(start_coordinates) and within_map(goal_coordinates)):
            raise ValueError('start or goal do not lie within the map')

        if start_coordinates == goal_coordinates:
            # start and goal are identical and can be reached instantly
            return [start_coordinates, goal_coordinates], 0.0

        # check if start and goal nodes have identical coordinates with one of the vertices
        # optimisations for visibility test can be made in this case:
        #   everything behind the neighbouring 2 edges is not visible
        # running graph shortest path search with identical but separate nodes would lead to
        # longer running time and resulting paths with 0 segments
        # query point might not be an extremity (regular polygon vertex)
        start_vertex_existed = False
        goal_vertex_existed = False
        start_is_extremity = False
        goal_is_extremity = False
        for v in self.all_vertices:
            if np.all(v.coordinates == start_coordinates):
                start_vertex_existed = True
                start_vertex = v
                if v.is_extremity:
                    # for this node all the
                    # do not add new node in graph
                    start_is_extremity = True
                break

        for v in self.all_vertices:
            if np.all(v.coordinates == goal_coordinates):
                goal_vertex_existed = True
                goal_vertex = v
                if v.is_extremity:
                    # do not add new node in graph
                    goal_is_extremity = True
                break

        if not start_vertex_existed:
            start_vertex = Vertex(start_coordinates)
        if not goal_vertex_existed:
            goal_vertex = Vertex(goal_coordinates)

        # create temporary graph (real copy to not edit the original prepared graph)
        # a shallow copy: constructs a new compound object and then (to the extent possible)
        #   inserts references into it to the objects found in the original.
        temporary_graph = copy(self.graph)

        # when start and goal are both extremities
        #  they are both present in the graph already and their connections are already known
        # computing the all directly reachable extremities from start and goal based on visibility is not required
        candidates = set()
        if not start_is_extremity:
            # when the start is not an extremity the connections to all other extremities have to be checked
            candidates.update(self.all_extremities)

        if not goal_is_extremity:
            # when the goal is not an extremity the connections to the goal are unknown and have to be checked
            # when it is an extremity it is contained in the all_extremities and gets checked automatically!
            # IMPORTANT: check if the goal node is visible from the start node!
            # query point might not be a vertex, but lie directly on an edge! (angle = 180deg)
            # has to be considered in .find_visible()
            candidates.add(goal_vertex)

        if not goal_vertex_existed:
            # IMPORTANT: manually translate the goal vertex, because it is not part of any polygon
            #   and hence does not get translated automatically
            goal_vertex.translate(start_vertex)

        visibles_n_distances = self.find_visible(start_vertex, candidates)

        # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
        #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
        # -> when goal is directly reachable, there can be no other shorter path to it
        for v, d in visibles_n_distances:
            if v == goal_vertex:
                vertex_path = [start_vertex, goal_vertex]
                print(vertex_path)
                draw_with_path(self, temporary_graph, goal_vertex, start_vertex, vertex_path)
                return [start_coordinates, goal_coordinates], d

            # modified a star algorithm returns the shortest path from goal to start
            # add to graph, but only in the direction: start <-extremity (being called the other way round!)
            temporary_graph.add_directed_edge(v, start_vertex, d)

        if not goal_is_extremity:
            # query point might not be a vertex, but lie directly on an edge! (angle = 180deg)
            # has to be considered in .find_visible()
            # start node does not have to be considered, because of the earlier check for the start node
            visibles_n_distances = self.find_visible(goal_vertex, set(self.all_extremities))
            # modified a star algorithm returns the shortest path from goal to start
            # add to graph, but only in the direction: extremity <- goal (being called the other way round)
            temporary_graph.add_multiple_directed_edges(goal_vertex, visibles_n_distances)

        # TODO find other more clever approach than modified a star?!
        # function returns the shortest path from goal to start (computational reasons), so just swap the parameters
        vertex_path, distance = modified_a_star(temporary_graph, start=goal_vertex, goal=start_vertex)
        # extract the coordinates from the path
        # print('goal reached. terminating now.')
        # print(vertex_path)
        if export_plots:
            draw_graph(temporary_graph)
            draw_with_path(self, temporary_graph, goal_vertex, start_vertex, vertex_path)
            draw_only_path(self, vertex_path)
        return [tuple(v.coordinates) for v in vertex_path], distance


if __name__ == "__main__":
    # counter clockwise edge numbering!
    # polygon1 = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    # polygon1 = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    polygon1 = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # clockwise numbering!
    # holes1 = []
    # holes1 = [[(3.0, 7.0), (5.0, 9.0), (5.0, 7.0), ], ]
    holes1 = [[(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0), ], ]

    map = Map()
    map.store(polygon1, holes1)
    # print(map.all_extremities)
    map.prepare()
    # draw_map(map)

    start_coords = (9.5, 1.0)
    goal_coords = (9.5, 9.0)

    print(map.find_shortest_path(start_coords, goal_coords))

    # TODO command line support?! create files with polygons and holes?
