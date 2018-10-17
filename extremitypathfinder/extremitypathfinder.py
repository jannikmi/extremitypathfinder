import pickle
from copy import deepcopy
from typing import List

import numpy as np

from .graph_search import modified_a_star
from .helper_classes import DirectedHeuristicGraph, Edge, Polygon, PolygonVertex, Vertex
from .helper_fcts import check_data_requirements, convert_gridworld, find_within_range, inside_polygon, lies_behind

# TODO possible to allow polygon consisting of 2 vertices only(=barrier)? lots of functions need at least 3 vertices atm


# Reference:
#   [1] Vinther, Anders Strand-Holm, Magnus Strand-Holm Vinther, and Peyman Afshani.
#   "Pathfinding in Two-dimensional Worlds"
#   http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf

DEFAULT_PICKLE_NAME = 'environment.pickle'


# is not a helper function to make it an importable part of the package
def load_pickle(path=DEFAULT_PICKLE_NAME):
    print('loading map from:', path)
    with open(path, 'rb') as f:
        return pickle.load(f)


class PolygonEnvironment:
    # class for keeping preloaded map for consecutive path queries
    boundary_polygon: Polygon = None
    holes: List[Polygon] = None

    # TODO find way to not store separate list of all (already stored in the polygons)
    all_edges: List[Edge] = None
    all_vertices: List[Vertex] = None
    all_extremities: List[Vertex] = None

    # boundary_extremities = None
    # hole_extremities = None
    prepared: bool = False
    graph: DirectedHeuristicGraph = None
    temp_graph: DirectedHeuristicGraph = None  # for storing and plotting the graph during a query

    def store(self, boundary_coordinates, list_of_hole_coordinates, validate=False):
        self.prepared = False
        # 'loading the map
        boundary_coordinates = np.array(boundary_coordinates)
        list_of_hole_coordinates = [np.array(hole_coords) for hole_coords in list_of_hole_coordinates]
        if validate:
            check_data_requirements(boundary_coordinates, list_of_hole_coordinates)

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

    def store_grid_world(self, size_x: int, size_y: int, obstacle_iter: iter, simplify: bool = True, validate=False):
        """
        prerequisites: grid world must not have single non-obstacle cells which are surrounded by obstacles
        ("white cell in black surrounding" = useless for path planning)
        :param size_x: the horizontal grid world size
        :param size_y: the vertical grid world size
        :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
        :param validate:
        :param simplify: whether the polygons should be simplified or not. reduces edge amount, allow diagonal edges
        """
        boundary_coordinates, list_of_hole_coordinates = convert_gridworld(size_x, size_y, obstacle_iter, simplify)
        self.store(boundary_coordinates, list_of_hole_coordinates, validate)

    def export_pickle(self, path=DEFAULT_PICKLE_NAME):
        print('storing map class in:', path)
        with open(path, 'wb') as f:
            pickle.dump(self, f)
        print('done.\n')

    def translate(self, new_origin):
        # shifts the coordinate system
        # computing the angle representations, shifted coordinates and distances for all vertices
        #   respective to the query point (lazy!)
        self.boundary_polygon.translate(new_origin)
        for hole in self.holes:
            hole.translate(new_origin)

    def find_visible(self, vertex_candidates, edges_to_check):
        """
        # IMPORTANT: self.translate(new_origin=query_vertex) always has to be called before!
            (for computing the angle representations wrt. the query vertex)
        query_vertex: a vertex for which the visibility to the vertices should be checked.
            also non extremity vertices, polygon vertices and vertices with the same coordinates are allowed.
            query point also might lie directly on an edge! (angle = 180deg)

        :param vertex_candidates: the set of all vertices which should be checked for visibility.
            IMPORTANT: is being manipulated, so has to be a copy!
            IMPORTANT: must not contain the query vertex!

        :param edges_to_check: the set of edges which determine visibility
        :return: a set of tuples of all vertices visible from the query vertex and the corresponding distance
        """

        visible_vertices = set()
        if len(vertex_candidates) == 0:
            return visible_vertices

        priority_edges = set()
        # goal: eliminating all vertices lying 'behind' any edge
        # TODO improvement in combination with priority: process edges roughly in sequence, but still allow jumps
        # would follow closer edges more often which have a bigger chance to eliminate candidates -> speed up
        while len(vertex_candidates) > 0 and len(edges_to_check) > 0:
            # check prioritized items first
            try:
                edge = priority_edges.pop()
                edges_to_check.remove(edge)
            except KeyError:
                edge = edges_to_check.pop()

            lies_on_edge = False
            v1, v2 = edge.vertex1, edge.vertex2
            if v1.get_distance_to_origin() == 0.0:
                # vertex1 has the same coordinates as the query vertex -> on the edge
                lies_on_edge = True
                # (but does not belong to the same polygon, not identical!)
                # mark this vertex as not visible (would otherwise add 0 distance edge in the graph)
                vertex_candidates.discard(v1)
                # its angle representation is not defined (no line segment from vertex1 to query vertex!)
                range_less_180 = v1.is_extremity
                # do not check the other neighbouring edge of vertex1 in the future
                e1 = v1.edge1
                edges_to_check.discard(e1)
                priority_edges.discard(e1)
                # everything between its two neighbouring edges is not visible for sure
                v1, v2 = v1.get_neighbours()

            elif v2.get_distance_to_origin() == 0.0:
                lies_on_edge = True
                vertex_candidates.discard(v2)
                range_less_180 = v2.is_extremity
                e1 = v2.edge2
                edges_to_check.discard(e1)
                priority_edges.discard(e1)
                v1, v2 = v2.get_neighbours()

            repr1 = v1.get_angle_representation()
            repr2 = v2.get_angle_representation()

            repr_diff = abs(repr1 - repr2)
            if repr_diff == 2.0:
                # angle == 180deg -> on the edge
                lies_on_edge = True
                range_less_180 = False  # does actually not matter here

            if lies_on_edge:
                # when the query vertex lies on an edge (or vertex) no behind/in front checks must be performed!
                # the neighbouring edges are visible for sure
                try:
                    vertex_candidates.remove(v1)
                    visible_vertices.add(v1)
                except KeyError:
                    pass
                try:
                    vertex_candidates.remove(v2)
                    visible_vertices.add(v2)
                except KeyError:
                    pass

                # all the candidates between the two vertices v1 v2 are not visible for sure
                # candidates with the same representation should not be deleted, because they can be visible!
                vertex_candidates.difference_update(
                    find_within_range(repr1, repr2, repr_diff, vertex_candidates, angle_range_less_180=range_less_180,
                                      equal_repr_allowed=False))
                continue

            # case: a 'regular' edge
            # eliminate all candidates which are blocked by the edge
            # that means inside the angle range spanned by the edge and actually behind it
            vertices_to_check = vertex_candidates.copy()
            # the vertices belonging to the edge itself (its vertices) must not be checked.
            # use discard() instead of remove() to not raise an error (they might not be candidates)
            vertices_to_check.discard(v1)
            vertices_to_check.discard(v2)
            if len(vertices_to_check) == 0:
                continue

            # assert repr1 is not None
            # assert repr2 is not None

            # for all candidate edges check if there are any candidate vertices (besides the ones belonging to the edge)
            #   within this angle range
            # the "view range" of an edge from a query point (spanned by the two vertices of the edge)
            #   is always < 180deg when the edge is not running through the query point (=180 deg)
            #  candidates with the same representation as v1 or v2 should be considered.
            #   they can be visible, but should be ruled out if they lie behind any edge!
            vertices_to_check = find_within_range(repr1, repr2, repr_diff, vertices_to_check, angle_range_less_180=True,
                                                  equal_repr_allowed=True)
            if len(vertices_to_check) == 0:
                continue

            # if a candidate is farther away from the query point than both vertices of the edge,
            #    it surely lies behind the edge
            max_distance = max(v1.get_distance_to_origin(), v2.get_distance_to_origin())
            vertices_behind = set(filter(lambda extr: extr.get_distance_to_origin() > max_distance, vertices_to_check))
            # they do not have to be checked, no intersection computation necessary
            # TODO improvement: increase the neighbouring edges' priorities when there were extremities behind
            vertices_to_check.difference_update(vertices_behind)
            if len(vertices_to_check) == 0:
                # also done later, only needed if skipping this edge
                vertex_candidates.difference_update(vertices_behind)
                continue

            # if the candidate is closer than both edge vertices it surely lies in front (
            min_distance = min(v1.get_distance_to_origin(), v2.get_distance_to_origin())
            vertices_in_front = set(
                filter(lambda extr: extr.get_distance_to_origin() < min_distance, vertices_to_check))
            # they do not have to be checked (safes computation)
            vertices_to_check.difference_update(vertices_in_front)

            # for all remaining vertices v it has to be tested if the line segment from query point (=origin) to v
            #    has an intersection with the current edge p1---p2
            # vertices directly on the edge are allowed (not eliminated)!
            p1 = v1.get_coordinates_translated()
            p2 = v2.get_coordinates_translated()
            for vertex in vertices_to_check:
                if lies_behind(p1, p2, vertex.get_coordinates_translated()):
                    vertices_behind.add(vertex)
                else:
                    vertices_in_front.add(vertex)

            # vertices behind any edge are not visible
            vertex_candidates.difference_update(vertices_behind)
            # if there are no more candidates left. immediately quit checking edges
            if len(vertex_candidates) == 0:
                break

            # check the neighbouring edges of all vertices which lie in front of the edge next first
            # (prioritize them)
            # they lie in front and hence will eliminate other vertices faster
            # the fewer vertex candidates remain, the faster the procedure
            # TODO improvement: increase priority every time and draw highest priority items
            #   but this involves sorting (expensive for large polygons!)
            #   idea: work with a list of sets, add new set for higher priority, no real sorting, but still managing!
            # TODO test speed impact
            for e in vertices_in_front:
                # only add the neighbour edges to the priority set if they still have to be checked!
                if type(e) == PolygonVertex:
                    # only vertices belonging to polygons have neighbours
                    priority_edges.update(edges_to_check.intersection({e.edge1, e.edge2}))

        # all edges have been checked
        # all remaining vertices were not concealed behind any edge and hence are visible
        visible_vertices.update(vertex_candidates)

        # return a set of tuples: (vertex, distance)
        return {(e, e.get_distance_to_origin()) for e in visible_vertices}

    def prepare(self):
        """
        precomputes all directly reachable extremities based on visibility
        and the distances between them
        internally stores a visibility graph optimized (reduced) for path planning
        :return:

        NOTE: pre computing the shortest paths between all directly reachable extremities
            and storing them in the graph would not be an advantage, because then the graph is fully connected
            a star would visit every node in the graph at least once (-> disadvantage!).
            TODO maybe advantage with optimized a star
        """

        if self.prepared:
            raise ValueError('this environment is already prepared. load new polygons first.')

        # preprocessing the map
        # construct graph of visible (=directly reachable) extremities
        # and optimize graph further at construction time
        self.graph = DirectedHeuristicGraph()
        extremities_to_check = set(self.all_extremities)
        # have to run for all (also last one!), because existing edges might get deleted every loop
        while len(extremities_to_check) > 0:
            query_extremity: PolygonVertex = extremities_to_check.pop()
            # extremities are always visible to each other (bi-directional relation -> undirected graph)
            #  -> do not check extremities which have been checked already
            #  (would only give the same result when algorithms are correct)
            # the extremity itself must not be checked when looking for visible neighbours

            self.translate(new_origin=query_extremity)

            visible_vertices = set()
            candidate_extremities = extremities_to_check.copy()
            # remove the extremities with the same coordinates as the query extremity
            candidate_extremities.difference_update(
                {c for c in candidate_extremities if c.get_angle_representation() is None})

            # these vertices all belong to a polygon
            # direct neighbours of the query vertex are visible
            # neighbouring vertices are reachable with the distance equal to the edge length
            n1, n2 = query_extremity.get_neighbours()
            try:
                candidate_extremities.remove(n1)
                visible_vertices.add((n1, n1.get_distance_to_origin()))
            except KeyError:
                pass
            try:
                candidate_extremities.remove(n2)
                visible_vertices.add((n2, n2.get_distance_to_origin()))
            except KeyError:
                pass

            # even though candidate_extremities might be empty now
            # must not skip to next loop here, because existing graph edges might get deleted here!

            # eliminate all vertices 'behind' the query point from the candidate set
            # since the query vertex is an extremity the 'outer' angle is < 180 degree
            # then the difference between the angle representation of the two edges has to be < 2.0
            # all vertices between the angle of the two neighbouring edges ('outer side')
            #   are not visible (no candidates!)
            # vertices with the same angle representation might be visible! do not delete them!
            repr1 = n1.get_angle_representation()
            repr2 = n2.get_angle_representation()
            repr_diff = abs(repr1 - repr2)
            candidate_extremities.difference_update(
                find_within_range(repr1, repr2, repr_diff, candidate_extremities, angle_range_less_180=True,
                                  equal_repr_allowed=False))

            # as shown in [1, Ch. II 4.4.2 "Property One"] Starting from any point lying "in front of" an extremity e,
            # such that both adjacent edges are visible, one will never visit e, because everything is
            # reachable on a shorter path without e (except e itself). An extremity e1 lying in the area "in front of"
            #   extremity e hence is never the next vertex in a shortest path coming from e.
            #   And also in reverse: when coming from e1 everything else than e itself can be reached faster
            #   without visiting e2. -> e1 and e do not have to be connected in the graph.
            # IMPORTANT: this condition only holds for building the basic visibility graph!
            #   when a query point happens to be an extremity, edges to the (visible) extremities in front
            #   MUST be added to the graph!
            # find extremities which fulfill this condition for the given query extremity
            repr1 = (repr1 + 2.0) % 4.0  # rotate 180 deg
            repr2 = (repr2 + 2.0) % 4.0
            # IMPORTANT: the true angle diff does not change, but the repr diff does! compute again
            repr_diff = abs(repr1 - repr2)

            # IMPORTANT: check all extremities here, not just current candidates
            # do not check extremities with equal coordinates (also query extremity itself!)
            #   and with the same angle representation (those edges must not get deleted from graph!)
            temp_candidates = set(filter(lambda e: e.get_angle_representation() is not None, self.all_extremities))
            lie_in_front = find_within_range(repr1, repr2, repr_diff, temp_candidates, angle_range_less_180=True,
                                             equal_repr_allowed=False)

            # already existing edges in the graph to the extremities in front have to be removed
            self.graph.remove_multiple_undirected_edges(query_extremity, lie_in_front)
            # do not consider when looking for visible extremities (NOTE: they might actually be visible!)
            candidate_extremities.difference_update(lie_in_front)

            # all edges except the neighbouring edges (handled above!) have to be checked
            edges_to_check = set(self.all_edges)
            edges_to_check.remove(query_extremity.edge1)
            edges_to_check.remove(query_extremity.edge2)

            visible_vertices.update(self.find_visible(candidate_extremities, edges_to_check))

            self.graph.add_multiple_undirected_edges(query_extremity, visible_vertices)

        # join all nodes with the same coordinates
        self.graph.make_clean()
        self.prepared = True

    def find_shortest_path(self, start_coordinates, goal_coordinates, free_space_after=True):
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

        # could check if start and goal nodes have identical coordinates with one of the vertices
        # optimisations for visibility test can be made in this case:
        # for extremities the visibility has already been (except for in front) computed
        # BUT: too many cases possible: e.g. multiple vertices identical to query point...
        # -> always create new query vertices

        # include start and goal vertices in the graph
        start_vertex = Vertex(start_coordinates)
        goal_vertex = Vertex(goal_coordinates)

        # check the goal node first (earlier termination possible)
        self.translate(new_origin=goal_vertex)  # do before checking angle representations!
        # IMPORTANT: manually translate the start vertex, because it is not part of any polygon
        #   and hence does not get translated automatically
        start_vertex.mark_outdated()

        # the visibility of only the graphs nodes have to be checked (not all extremities!)
        # points with the same angle representation should not be considered visible
        # (they also cause errors in the algorithms, because their angle repr is not defined!)
        candidates = set(filter(lambda n: n.get_angle_representation() is not None, self.graph.get_all_nodes()))
        # IMPORTANT: check if the start node is visible from the goal node!
        candidates.add(start_vertex)

        visibles_n_distances_goal = self.find_visible(candidates, edges_to_check=set(self.all_edges))
        if len(visibles_n_distances_goal) == 0:
            # The goal node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        # create temporary graph
        # DirectedHeuristicGraph implements __deepcopy__() to not change the original precomputed self.graph
        # but to still not create real copies of vertex instances!
        self.temp_graph = deepcopy(self.graph)

        # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
        #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
        # -> when goal is directly reachable, there can be no other shorter path to it. Terminate
        for v, d in visibles_n_distances_goal:
            if v == start_vertex:
                return [start_coordinates, goal_coordinates], d

            # add unidirectional edges to the temporary graph
            # add edges in the direction: extremity (v) -> goal
            # TODO: improvement: add edges last, after filtering them. instead of deleting edges
            self.temp_graph.add_directed_edge(v, goal_vertex, d)

        self.translate(new_origin=start_vertex)  # do before checking angle representations!
        # the visibility of only the graphs nodes have to be checked
        # the goal node does not have to be considered, because of the earlier check
        candidates = set(filter(lambda n: n.get_angle_representation() is not None, self.graph.get_all_nodes()))
        visibles_n_distances_start = self.find_visible(candidates, edges_to_check=set(self.all_edges))
        if len(visibles_n_distances_start) == 0:
            # The start node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        # add edges in the direction: start -> extremity
        # TODO: improvement: add edges last, after filtering them. instead of deleting edges
        self.temp_graph.add_multiple_directed_edges(start_vertex, visibles_n_distances_start)

        # also here unnecessary edges in the graph can be deleted when start or goal lie in front of visible extremities
        # IMPORTANT: when a query point happens to coincide with an extremity, edges to the (visible) extremities
        #  in front MUST be added to the graph! Handled by always introducing new (non extremity, non polygon) vertices.

        # for every extremity that is visible from either goal or stat
        # NOTE: edges are undirected! self.temp_graph.get_neighbours_of(start_vertex) == set()
        # neighbours_start = self.temp_graph.get_neighbours_of(start_vertex)
        neighbours_start = {n for n, d in visibles_n_distances_start}
        # the goal vertex might be marked visible, it is not an extremity -> skip
        neighbours_start.discard(goal_vertex)
        neighbours_goal = self.temp_graph.get_neighbours_of(goal_vertex)
        for vertex in neighbours_start | neighbours_goal:
            # assert type(vertex) == PolygonVertex and vertex.is_extremity

            # check only if point is visible
            temp_candidates = set()
            if vertex in neighbours_start:
                temp_candidates.add(start_vertex)

            if vertex in neighbours_goal:
                temp_candidates.add(goal_vertex)

            if len(temp_candidates) > 0:
                self.translate(new_origin=vertex)
                # IMPORTANT: manually translate the goal and start vertices
                start_vertex.mark_outdated()
                goal_vertex.mark_outdated()

                n1, n2 = vertex.get_neighbours()
                repr1 = (n1.get_angle_representation() + 2.0) % 4.0  # rotated 180 deg
                repr2 = (n2.get_angle_representation() + 2.0) % 4.0
                repr_diff = abs(repr1 - repr2)

                # IMPORTANT: special case:
                # here the nodes must stay connected if they have the same angle representation!
                lie_in_front = find_within_range(repr1, repr2, repr_diff, temp_candidates, angle_range_less_180=True,
                                                 equal_repr_allowed=False)
                self.temp_graph.remove_multiple_undirected_edges(vertex, lie_in_front)

        # NOTE: exploiting property 2 from [1] here would be more expensive than beneficial
        vertex_path, distance = modified_a_star(self.temp_graph, start_vertex, goal_vertex)

        if free_space_after:
            del self.temp_graph  # free the memory

        # extract the coordinates from the path
        return [tuple(v.coordinates) for v in vertex_path], distance


if __name__ == "__main__":
    pass

    # TODO command line support. read polygons and holes from .json files?
