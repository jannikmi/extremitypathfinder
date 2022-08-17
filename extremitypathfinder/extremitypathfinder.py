import itertools
import pickle
from copy import deepcopy
from typing import Iterable, List, Optional, Set, Tuple

import numpy as np

from extremitypathfinder.global_settings import (
    DEFAULT_PICKLE_NAME,
    INPUT_COORD_LIST_TYPE,
    INPUT_COORD_TYPE,
    LENGTH_TYPE,
    OBSTACLE_ITER_TYPE,
    PATH_TYPE,
)
from extremitypathfinder.helper_classes import (
    DirectedHeuristicGraph,
    Edge,
    Polygon,
    PolygonVertex,
    Vertex,
    angle_rep_inverse,
)
from extremitypathfinder.helper_fcts import (
    check_data_requirements,
    convert_gridworld,
    find_visible,
    find_visible2,
    find_within_range,
    find_within_range2,
    get_angle_repr,
    inside_polygon,
)

# TODO possible to allow polygon consisting of 2 vertices only(=barrier)? lots of functions need at least 3 vertices atm


# is not a helper function to make it an importable part of the package
def load_pickle(path=DEFAULT_PICKLE_NAME):
    print("loading map from:", path)
    with open(path, "rb") as f:
        return pickle.load(f)


# TODO document parameters
class PolygonEnvironment:
    """Class allowing to use polygons to represent "2D environments" and use them for path finding.

    Keeps a "loaded" and prepared environment for consecutive path queries.
    Internally uses a visibility graph optimised for shortest path finding.
    General approach and some optimisations theoretically described in:
    [1] Vinther, Anders Strand-Holm, Magnus Strand-Holm Vinther, and Peyman Afshani.
    "`Pathfinding in Two-dimensional Worlds
    <https://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__"
    """

    boundary_polygon: Polygon = None
    holes: List[Polygon] = None
    prepared: bool = False
    graph: DirectedHeuristicGraph = None
    temp_graph: DirectedHeuristicGraph = None  # for storing and plotting the graph during a query
    _all_extremities: Optional[List[PolygonVertex]] = None
    _all_vertices: Optional[List[PolygonVertex]] = None

    @property
    def polygons(self) -> Iterable[Polygon]:
        yield self.boundary_polygon
        yield from self.holes

    @property
    def all_vertices(self) -> List[PolygonVertex]:
        if self._all_vertices is None:
            self._all_vertices = list(itertools.chain(*iter(p.vertices for p in self.polygons)))
        return self._all_vertices

    @property
    def nr_vertices(self) -> int:
        return len(self.all_vertices)

    @property
    def extremity_indices(self) -> Set[int]:
        # TODO refactor
        for p in self.polygons:
            p._find_extremities()
        # Attention: only consider extremities that are actually within the map
        return {idx for idx, v in enumerate(self.all_vertices) if v.is_extremity and self.within_map(v.coordinates)}

    @property
    def extremity_mask(self) -> np.ndarray:
        mask = np.full(self.nr_vertices, False, dtype=bool)
        for i in self.extremity_indices:
            mask[i] = True
        return mask

    @property
    def all_extremities(self) -> List[PolygonVertex]:
        return [self.all_vertices[i] for i in self.extremity_indices]

        # TODO
        # if self._all_extremities is None:
        # return self._all_extremities

    @property
    def all_edges(self) -> Set[Edge]:
        return set(itertools.chain(*iter(p.edges for p in self.polygons)))

    def store(
        self,
        boundary_coordinates: INPUT_COORD_LIST_TYPE,
        list_of_hole_coordinates: INPUT_COORD_LIST_TYPE,
        validate: bool = False,
    ):
        """saves the passed input polygons in the environment

        .. note:: the passed polygons must meet these requirements:

            * given as numpy or python array of coordinate tuples: ``[(x1,y1), (x2,y2,)...]``
            * no repetition of the first point at the end
            * at least 3 vertices (no single points or lines allowed)
            * no consequent vertices with identical coordinates in the polygons (same coordinates allowed)
            * no self intersections
            * edge numbering has to follow these conventions: boundary polygon counter clockwise, holes clockwise

        :param boundary_coordinates: array of coordinates with counter clockwise edge numbering
        :param list_of_hole_coordinates: array of coordinates with clockwise edge numbering
        :param validate: whether the requirements of the data should be tested

        :raises AssertionError: when validate=True and the input is invalid.
        """
        self.prepared = False
        # loading the map
        boundary_coordinates = np.array(boundary_coordinates)
        list_of_hole_coordinates = [np.array(hole_coords) for hole_coords in list_of_hole_coordinates]
        if validate:
            check_data_requirements(boundary_coordinates, list_of_hole_coordinates)

        self.boundary_polygon = Polygon(boundary_coordinates, is_hole=False)
        # IMPORTANT: make a copy of the list instead of linking to the same list (python!)
        self.holes = [Polygon(coordinates, is_hole=True) for coordinates in list_of_hole_coordinates]

    def store_grid_world(
        self,
        size_x: int,
        size_y: int,
        obstacle_iter: OBSTACLE_ITER_TYPE,
        simplify: bool = True,
        validate: bool = False,
    ):
        """Convert a grid-like into a polygon environment and save it

        Prerequisites: grid world must not have single non-obstacle cells which are surrounded by obstacles
        ("white cells in black surrounding" = useless for path planning)

        :param size_x: the horizontal grid world size
        :param size_y: the vertical grid world size
        :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
        :param validate: whether the input should be validated
        :param simplify: whether the polygons should be simplified or not. reduces edge amount, allow diagonal edges
        """
        boundary_coordinates, list_of_hole_coordinates = convert_gridworld(size_x, size_y, obstacle_iter, simplify)
        self.store(boundary_coordinates, list_of_hole_coordinates, validate)

    def export_pickle(self, path: str = DEFAULT_PICKLE_NAME):
        print("storing map class in:", path)
        with open(path, "wb") as f:
            pickle.dump(self, f)
        print("done.\n")

    def translate(self, new_origin: Vertex):
        """shifts the coordinate system to a new origin

        computing the angle representations, shifted coordinates and distances for all vertices
        respective to the query point (lazy!)

        :param new_origin: the origin of the coordinate system to be shifted to
        """
        for p in self.polygons:
            p.translate(new_origin)

    def prepare(self):  # TODO include in storing functions? breaking change!
        """Computes a visibility graph optimized (=reduced) for path planning and stores it

        Computes all directly reachable extremities based on visibility and their distance to each other

        .. note::
            Multiple polygon vertices might have identical coords.
            They must be treated as distinct vertices here, since their attached edges determine visibility.
            In the created graph however, these nodes must be merged at the end to avoid ambiguities!

        .. note::
            Pre computing the shortest paths between all directly reachable extremities
            and storing them in the graph would not be an advantage, because then the graph is fully connected.
            A star would visit every node in the graph at least once (-> disadvantage!).
        """
        if self.prepared:
            raise ValueError("this environment is already prepared. load new polygons first.")

        # preprocessing the map
        # construct graph of visible (=directly reachable) extremities
        # and optimize graph further at construction time
        # NOTE: initialise the graph with all extremities.
        #   even if a node has no edges (visibility to other extremities), it should still be included!
        extremities = self.all_extremities
        nr_extremities = len(extremities)
        if nr_extremities == 0:
            # TODO
            self.graph = DirectedHeuristicGraph(extremities)
            return

        vertices = self.all_vertices
        nr_vertices = len(vertices)
        extremity_indices = self.extremity_indices
        extremity_mask = self.extremity_mask

        # TODO more performant way of computing
        all_edges = list(self.all_edges)
        graph = DirectedHeuristicGraph(extremities)
        coords = np.stack([v.coordinates for v in vertices])
        vertex_edge_idxs = np.stack([(all_edges.index(v.edge1), all_edges.index(v.edge2)) for v in vertices])
        # TODO sparse matrix. problematic: default value is 0.0
        angle_representations = np.full((nr_vertices, nr_vertices), np.nan)
        edge_vertex_idxs = np.stack(
            [(vertices.index(edge.vertex1), vertices.index(edge.vertex2)) for edge in all_edges]
        )

        if len(extremity_indices) != len(extremities):
            raise ValueError

        # TODO reuse
        def get_repr(idx_origin, i):
            coords_origin = coords[idx_origin]
            return get_angle_repr(coords_origin, i, angle_representations, coords)

        def get_neighbours(i: int) -> Tuple[int, int]:
            edge_idx1, edge_idx2 = vertex_edge_idxs[i]
            neigh_idx1 = edge_vertex_idxs[edge_idx1, 0]
            neigh_idx2 = edge_vertex_idxs[edge_idx2, 1]
            return neigh_idx1, neigh_idx2

        def get_coordinates_translated(orig_idx: int, i: int) -> np.ndarray:
            coords_origin = coords[orig_idx]
            coords_v = coords[i]
            return coords_v - coords_origin

        def get_distance_to_origin(orig_idx: int, i: int) -> float:
            coords = get_coordinates_translated(orig_idx, i)
            return np.linalg.norm(coords, ord=2)

        extremity_indices = list(extremity_indices)
        # TODO use orig_ptr
        for _orig_ptr, origin_idx in enumerate(extremity_indices):
            # vertices all belong to a polygon
            idx_n1, idx_n2 = get_neighbours(origin_idx)
            # ATTENTION: polygons may intersect -> neighbouring extremities must NOT be visible from each other!
            # eliminate all vertices 'behind' the query point from the candidate set
            # since the query vertex is an extremity the 'outer' angle is < 180 degree
            # then the difference between the angle representation of the two edges has to be < 2.0
            # all vertices between the angle of the two neighbouring edges ('outer side')
            #   are not visible (no candidates!)
            # ATTENTION: vertices with the same angle representation might be visible and must NOT be deleted!
            n1_repr = get_repr(origin_idx, idx_n1)
            n2_repr = get_repr(origin_idx, idx_n2)

            # TODO lazy init? same as angle repr
            vert_idx2dist = {i: get_distance_to_origin(origin_idx, i) for i in range(nr_vertices)}

            idx2repr = {i: get_repr(origin_idx, i) for i in extremity_indices}
            # only consider extremities with coords different from the query extremity
            # (angle representation not None)
            # the origin extremity itself must also not be checked when looking for visible neighbours
            idx2repr = {i: r for i, r in idx2repr.items() if r is not None}
            idxs_behind = find_within_range2(
                n1_repr,
                n2_repr,
                idx2repr,
                angle_range_less_180=True,
                equal_repr_allowed=False,
            )

            # TODO
            # extremities are always visible to each other
            # (bi-directional relation -> undirected edges in the graph)
            #  -> do not check extremities which have been checked already
            #  (must give the same result when algorithms are correct)
            # idx2repr_tmp = {i: r for i, r in idx2repr.items() if i > origin_idx}
            # idxs_behind = find_within_range2(
            #     n1_repr,
            #     n2_repr,
            #     idx2repr_tmp,
            #     angle_range_less_180=True,
            #     equal_repr_allowed=False,
            # )
            #
            # if idxs_behind_ != idxs_behind:
            #     x = 1
            #
            # idxs_behind__ = {i for i in idxs_behind_ if i > origin_idx}
            # if idxs_behind__ != idxs_behind:
            #     raise ValueError

            # as shown in [1, Ch. II 4.4.2 "Property One"]: Starting from any point lying "in front of" an extremity e,
            # such that both adjacent edges are visible, one will never visit e, because everything is
            # reachable on a shorter path without e (except e itself).
            # An extremity e1 lying in the area "in front of" extremity e hence is never the next vertex
            # in a shortest path coming from e.
            # And also in reverse: when coming from e1 everything else than e itself can be reached faster
            # without visiting e.
            # -> e1 and e do not have to be connected in the graph.
            # IMPORTANT: this condition only holds for building the basic visibility graph without start and goal node!
            # When a query point (start/goal) happens to be an extremity, edges to the (visible) extremities in front
            # MUST be added to the graph!
            # Find extremities which fulfill this condition for the given query extremity
            n1_repr_inv = angle_rep_inverse(n1_repr)
            n2_repr_inv = angle_rep_inverse(n2_repr)

            # IMPORTANT: check all extremities here, not just current candidates
            # do not check extremities with equal coords (also query extremity itself!)
            #   and with the same angle representation (those edges must not get deleted from graph!)
            lie_in_front_idx = find_within_range2(
                n1_repr_inv,
                n2_repr_inv,
                idx2repr,
                angle_range_less_180=True,
                equal_repr_allowed=False,
            )

            # do not consider points lying in front when looking for visible extremities,
            # even if they are actually be visible
            # do not consider points found to lie behind
            idx2repr = {i: r for i, r in idx2repr.items() if i not in lie_in_front_idx and i not in idxs_behind}

            # all edges have to be checked, except the 2 neighbouring edges (handled above!)
            nr_edges = len(all_edges)
            edge_idxs2check = set(range(nr_edges))
            edge1_idx, edge2_idx = vertex_edge_idxs[origin_idx]
            edge_idxs2check.remove(edge1_idx)
            edge_idxs2check.remove(edge2_idx)
            coords_origin = coords[origin_idx]
            visible_idxs = find_visible2(
                extremity_mask,
                angle_representations,
                coords,
                vertex_edge_idxs,
                edge_vertex_idxs,
                edge_idxs2check,
                coords_origin,
                idx2repr,
                vert_idx2dist,
            )

            # TODO graph: also use indices instead of vertices
            origin_extremity = vertices[origin_idx]
            visible_vertex2dist_map = {vertices[i]: get_distance_to_origin(origin_idx, i) for i in visible_idxs}
            graph.add_multiple_undirected_edges(origin_extremity, visible_vertex2dist_map)
            # optimisation: "thin out" the graph
            # remove already existing edges in the graph to the extremities in front
            lie_in_front = {vertices[i] for i in lie_in_front_idx}
            graph.remove_multiple_undirected_edges(origin_extremity, lie_in_front)

        graph.make_clean()  # join all nodes with the same coords

        self.graph = graph
        self.prepared = True

    def within_map(self, coords: INPUT_COORD_TYPE):
        """checks if the given coordinates lie within the boundary polygon and outside of all holes

        :param coords: numerical tuple representing coordinates
        :return: whether the given coordinate is a valid query point
        """

        x, y = coords
        if not inside_polygon(x, y, self.boundary_polygon.coordinates, border_value=True):
            return False
        for hole in self.holes:
            if inside_polygon(x, y, hole.coordinates, border_value=False):
                return False
        return True

    def find_shortest_path(
        self,
        start_coordinates: INPUT_COORD_TYPE,
        goal_coordinates: INPUT_COORD_TYPE,
        free_space_after: bool = True,
        verify: bool = True,
    ) -> Tuple[PATH_TYPE, LENGTH_TYPE]:
        """computes the shortest path and its length between start and goal node

        :param start_coordinates: a (x,y) coordinate tuple representing the start node
        :param goal_coordinates:  a (x,y) coordinate tuple representing the goal node
        :param free_space_after: whether the created temporary search graph self.temp_graph
            should be deleted after the query
        :param verify: whether it should be checked if start and goal points really lie inside the environment.
         if points close to or on polygon edges should be accepted as valid input, set this to ``False``.
        :return: a tuple of shortest path and its length. ([], None) if there is no possible path.
        """
        # path planning query:
        # make sure the map has been loaded and prepared
        if self.boundary_polygon is None:
            raise ValueError("No Polygons have been loaded into the map yet.")
        if not self.prepared:
            self.prepare()

        if verify and not self.within_map(start_coordinates):
            raise ValueError("start point does not lie within the map")
        if verify and not self.within_map(goal_coordinates):
            raise ValueError("goal point does not lie within the map")
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

        # the visibility of only the graphs nodes has to be checked (not all extremities!)
        # points with the same angle representation should not be considered visible
        # (they also cause errors in the algorithms, because their angle repr is not defined!)
        candidates = set(
            filter(
                lambda n: n.get_angle_representation() is not None,
                self.graph.get_all_nodes(),
            )
        )
        # IMPORTANT: check if the start node is visible from the goal node!
        # NOTE: all edges are being checked, it is computationally faster to compute all visibilities in one go
        candidates.add(start_vertex)

        # TODO use new variant
        visibles_n_distances_goal = find_visible(candidates, edges_to_check=self.all_edges)
        if len(visibles_n_distances_goal) == 0:
            # The goal node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        # create temporary graph TODO make more performant, avoid real copy
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
            self.temp_graph.add_directed_edge(v, goal_vertex, d)

        self.translate(new_origin=start_vertex)  # do before checking angle representations!
        # the visibility of only the graphs nodes have to be checked
        # the goal node does not have to be considered, because of the earlier check
        candidates = set(
            filter(
                lambda n: n.get_angle_representation() is not None,
                self.graph.get_all_nodes(),
            )
        )

        # TODO use new variant
        visibles_n_distances_start = find_visible(candidates, edges_to_check=self.all_edges)
        if len(visibles_n_distances_start) == 0:
            # The start node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        # add edges in the direction: start -> extremity
        self.temp_graph.add_multiple_directed_edges(start_vertex, visibles_n_distances_start)

        # also here unnecessary edges in the graph can be deleted when start or goal lie in front of visible extremities
        # IMPORTANT: when a query point happens to coincide with an extremity, edges to the (visible) extremities
        #  in front MUST be added to the graph! Handled by always introducing new (non extremity, non polygon) vertices.

        # for every extremity that is visible from either goal or start
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
                repr1 = angle_rep_inverse(n1.get_angle_representation())  # rotated 180 deg
                repr2 = angle_rep_inverse(n2.get_angle_representation())

                # IMPORTANT: special case:
                # here the nodes must stay connected if they have the same angle representation!
                lie_in_front = find_within_range(
                    repr1,
                    repr2,
                    temp_candidates,
                    angle_range_less_180=True,
                    equal_repr_allowed=False,
                )
                self.temp_graph.remove_multiple_undirected_edges(vertex, lie_in_front)

        # NOTE: exploiting property 2 from [1] here would be more expensive than beneficial
        vertex_path, distance = self.temp_graph.modified_a_star(start_vertex, goal_vertex)

        if free_space_after:
            del self.temp_graph  # free the memory

        # extract the coordinates from the path
        return [tuple(v.coordinates) for v in vertex_path], distance


if __name__ == "__main__":
    # TODO command line support. read polygons and holes from .json files?
    pass
