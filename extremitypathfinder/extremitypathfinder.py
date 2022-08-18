import pickle
from copy import deepcopy
from typing import List, Optional, Tuple

import numpy as np

from extremitypathfinder.global_settings import (
    DEFAULT_PICKLE_NAME,
    INPUT_COORD_LIST_TYPE,
    LENGTH_TYPE,
    OBSTACLE_ITER_TYPE,
    PATH_TYPE,
    InputCoords,
)
from extremitypathfinder.helper_classes import (
    DirectedHeuristicGraph,
    Polygon,
    PolygonVertex,
    angle_rep_inverse,
    compute_extremity_idxs,
)
from extremitypathfinder.helper_fcts import (
    check_data_requirements,
    convert_gridworld,
    find_visible2,
    find_within_range,
    get_angle_repr,
    get_repr_n_dists,
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
    def nr_edges(self) -> int:
        return self.nr_vertices

    @property
    def all_extremities(self) -> List[Tuple]:
        coords = self.coords
        return [tuple(coords[i]) for i in self.extremity_indices]

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
        boundary_coordinates = np.array(boundary_coordinates, dtype=float)
        list_of_hole_coordinates = [np.array(hole_coords, dtype=float) for hole_coords in list_of_hole_coordinates]
        if validate:
            check_data_requirements(boundary_coordinates, list_of_hole_coordinates)

        self.boundary_polygon = Polygon(boundary_coordinates, is_hole=False)

        # IMPORTANT: make a copy of the list instead of linking to the same list (python!)
        self.holes = [Polygon(coordinates, is_hole=True) for coordinates in list_of_hole_coordinates]

        list_of_polygons = [boundary_coordinates] + list_of_hole_coordinates
        nr_total_pts = sum(map(len, list_of_polygons))
        vertex_edge_idxs = np.empty((nr_total_pts, 2), dtype=int)
        # TODO required? inverse of the other
        edge_vertex_idxs = np.empty((nr_total_pts, 2), dtype=int)
        edge_idx = 0
        offset = 0
        extremity_idxs = set()
        for poly in list_of_polygons:

            poly_extr_idxs = compute_extremity_idxs(poly)
            poly_extr_idxs = {i + offset for i in poly_extr_idxs}
            extremity_idxs |= poly_extr_idxs

            nr_coords = len(poly)
            v1 = -1 % nr_coords
            # TODO col 1 is just np.arange?!
            for v2 in range(nr_coords):
                v1_idx = v1 + offset
                v2_idx = v2 + offset
                edge_vertex_idxs[edge_idx, 0] = v1_idx
                edge_vertex_idxs[edge_idx, 1] = v2_idx
                vertex_edge_idxs[v1_idx, 1] = edge_idx
                vertex_edge_idxs[v2_idx, 0] = edge_idx
                # move to the next vertex/edge
                v1 = v2
                edge_idx += 1

            offset = edge_idx

        # assert edge_idx == nr_total_pts

        coords = np.concatenate(list_of_polygons, axis=0)
        # Attention: only consider extremities that are actually within the map
        extremity_idxs = [i for i in extremity_idxs if self.within_map(coords[i])]

        mask = np.full(nr_total_pts, False, dtype=bool)
        for i in extremity_idxs:
            mask[i] = True

        self.nr_vertices = nr_total_pts
        self.edge_vertex_idxs = edge_vertex_idxs
        self.vertex_edge_idxs = vertex_edge_idxs
        self.coords = coords
        self.extremity_indices = extremity_idxs
        self.extremity_mask = mask

        self.reprs_n_distances = {i: get_repr_n_dists(i, coords) for i in extremity_idxs}

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

    def prepare(self):
        """Computes a visibility graph optimized (=reduced) for path planning and stores it

        Computes all directly reachable extremities based on visibility and their distance to each other
        pre-procesing of the map. pre-computation for faster shortest path queries
        optimizes graph further at construction time

        NOTE: initialise the graph with all extremities.
        even if a node has no edges (visibility to other extremities, dangling node),
        it must still be included!

        .. note::
            Multiple polygon vertices might have identical coords_rel.
            They must be treated as distinct vertices here, since their attached edges determine visibility.
            In the created graph however, these nodes must be merged at the end to avoid ambiguities!

        .. note::
            Pre computing the shortest paths between all directly reachable extremities
            and storing them in the graph would not be an advantage, because then the graph is fully connected.
            A* would visit every node in the graph at least once (-> disadvantage!).
        """
        if self.prepared:
            raise ValueError("this environment is already prepared. load new polygons first.")

        nr_extremities = len(self.extremity_indices)
        if nr_extremities == 0:
            self.graph = DirectedHeuristicGraph()
            return

        # TODO pre computation and storage of all
        # TODO more performant way of computing
        nr_edges = self.nr_edges
        extremity_indices = self.extremity_indices
        extremity_mask = self.extremity_mask
        coords = self.coords
        vertex_edge_idxs = self.vertex_edge_idxs
        edge_vertex_idxs = self.edge_vertex_idxs

        # TODO sparse matrix. problematic: default value is 0.0
        # angle_representations = np.full((nr_vertices, nr_vertices), np.nan)

        # TODO reuse
        def get_repr(idx_origin, i):
            coords_origin = coords[idx_origin]
            return get_angle_repr(coords_origin, coords[i])

        def get_neighbours(i: int) -> Tuple[int, int]:
            edge_idx1, edge_idx2 = vertex_edge_idxs[i]
            neigh_idx1 = edge_vertex_idxs[edge_idx1, 0]
            neigh_idx2 = edge_vertex_idxs[edge_idx2, 1]
            return neigh_idx1, neigh_idx2

        def get_relative_coords(orig_idx: int, i: int) -> np.ndarray:
            coords_origin = coords[orig_idx]
            coords_v = coords[i]
            return coords_v - coords_origin

        def get_distance_to_origin(orig_idx: int, i: int) -> float:
            coords_rel = get_relative_coords(orig_idx, i)
            return np.linalg.norm(coords_rel, ord=2)

        extremity_coord_map = {i: coords[i] for i in extremity_indices}
        graph = DirectedHeuristicGraph(extremity_coord_map)
        for extr_ptr, origin_idx in enumerate(extremity_indices):
            # extremities are always visible to each other
            # (bi-directional relation -> undirected edges in the graph)
            #  -> do not check extremities which have been checked already
            #  (must give the same result when algorithms are correct)
            # the origin extremity itself must also not be checked when looking for visible neighbours
            candidate_idxs = extremity_indices[extr_ptr + 1 :]

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

            vert_idx2repr, vert_idx2dist = self.reprs_n_distances[origin_idx]

            # only consider extremities with coords_rel different from the query extremity
            # (angle representation not None)

            candidate_idxs = {i for i in candidate_idxs if vert_idx2repr[i] is not np.nan}
            idxs_behind = find_within_range(
                n1_repr,
                n2_repr,
                vert_idx2repr,
                candidate_idxs,
                angle_range_less_180=True,
                equal_repr_allowed=False,
            )
            # do not consider points found to lie behind
            candidate_idxs.difference_update(idxs_behind)

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
            # do not check extremities with equal coords_rel (also query extremity itself!)
            #   and with the same angle representation (those edges must not get deleted from graph!)
            idxs_in_front = find_within_range(
                n1_repr_inv,
                n2_repr_inv,
                vert_idx2repr,
                candidate_idxs,
                angle_range_less_180=True,
                equal_repr_allowed=False,
            )
            # do not consider points lying in front when looking for visible extremities,
            # even if they are actually be visible
            candidate_idxs.difference_update(idxs_in_front)

            # all edges have to be checked, except the 2 neighbouring edges (handled above!)
            edge_idxs2check = set(range(nr_edges))
            edge1_idx, edge2_idx = vertex_edge_idxs[origin_idx]
            edge_idxs2check.remove(edge1_idx)
            edge_idxs2check.remove(edge2_idx)
            coords_origin = coords[origin_idx]
            visible_idxs = find_visible2(
                extremity_mask,
                coords,
                vertex_edge_idxs,
                edge_vertex_idxs,
                edge_idxs2check,
                coords_origin,
                vert_idx2repr,
                vert_idx2dist,
                candidate_idxs,
                origin_idx,
            )

            visible_vertex2dist_map = {i: get_distance_to_origin(origin_idx, i) for i in visible_idxs}
            graph.add_multiple_undirected_edges(origin_idx, visible_vertex2dist_map)
            # optimisation: "thin out" the graph
            # remove already existing edges in the graph to the extremities in front
            graph.remove_multiple_undirected_edges(origin_idx, idxs_in_front)

        graph.join_identical()  # join all nodes with the same coords_rel

        self.graph = graph
        self.prepared = True

    def within_map(self, coords: InputCoords):
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
        start_coordinates: InputCoords,
        goal_coordinates: InputCoords,
        free_space_after: bool = True,
        verify: bool = True,
    ) -> Tuple[PATH_TYPE, LENGTH_TYPE]:
        """computes the shortest path and its length between start and goal node

        :param start_coordinates: a (x,y) coordinate tuple representing the start node
        :param goal_coordinates:  a (x,y) coordinate tuple representing the goal node
        :param free_space_after: whether the created temporary search graph graph
            should be deleted after the query
        :param verify: whether it should be checked if start and goal points really lie inside the environment.
         if points close to or on polygon edges should be accepted as valid input, set this to ``False``.
        :return: a tuple of shortest path and its length. ([], None) if there is no possible path.
        """

        # TODO reuse
        def get_relative_coords(coords_origin: np.ndarray, i: int) -> np.ndarray:
            coords_v = coords[i]
            return coords_v - coords_origin

        def get_distance_to_origin(coords_origin: np.ndarray, i: int) -> float:
            coords_rel = get_relative_coords(coords_origin, i)
            return np.linalg.norm(coords_rel, ord=2)

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

        coords_start = np.array(start_coordinates)
        coords_goal = np.array(goal_coordinates)
        if np.array_equal(coords_start, coords_goal):
            # start and goal are identical and can be reached instantly
            return [start_coordinates, goal_coordinates], 0.0

        nr_edges = self.nr_edges
        vertex_edge_idxs = self.vertex_edge_idxs
        edge_vertex_idxs = self.edge_vertex_idxs
        # temporarily extend data structures
        extremity_mask = np.append(self.extremity_mask, (False, False))
        coords = np.append(self.coords, (coords_start, coords_goal), axis=0)
        idx_start = self.nr_vertices
        idx_goal = self.nr_vertices + 1

        # start and goal nodes could be identical with one ore more of the vertices
        # BUT: this is an edge case -> compute visibility as usual and later try to merge with the graph

        # create temporary graph
        # TODO make more performant, avoid real copy
        # DirectedHeuristicGraph implements __deepcopy__() to not change the original precomputed self.graph
        # but to still not create real copies of vertex instances!
        graph = deepcopy(self.graph)
        # graph = self.graph

        # check the goal node first (earlier termination possible)
        idx_origin = idx_goal
        # the visibility of only the graphs nodes has to be checked (not all extremities!)
        # points with the same angle representation should not be considered visible
        # (they also cause errors in the algorithms, because their angle repr is not defined!)
        # IMPORTANT: also check if the start node is visible from the goal node!
        # NOTE: all edges are being checked, it is computationally faster to compute all visibilities in one go
        self.graph.get_all_nodes().add(idx_start)
        coords_origin = coords[idx_origin]
        edge_idxs2check = set(range(nr_edges))

        vert_idx2repr, vert_idx2dist = get_repr_n_dists(idx_origin, coords)
        candidate_idxs = {i for i in (self.graph.get_all_nodes()) if not vert_idx2dist[i] == 0.0}
        visible_idxs = find_visible2(
            extremity_mask,
            coords,
            vertex_edge_idxs,
            edge_vertex_idxs,
            edge_idxs2check,
            coords_origin,
            vert_idx2repr,
            vert_idx2dist,
            candidate_idxs,
            idx_origin,
        )
        visibles_n_distances_map = {i: vert_idx2dist[i] for i in visible_idxs}

        if len(visibles_n_distances_map) == 0:
            # The goal node does not have any neighbours. Hence there is not possible path to the goal.
            return [], None

        for i, d in visibles_n_distances_map.items():
            if i == idx_start:
                # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
                #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
                # -> when goal is directly reachable, there can be no other shorter path to it. Terminate
                return [start_coordinates, goal_coordinates], d

            # add unidirectional edges to the temporary graph
            # add edges in the direction: extremity (v) -> goal
            graph.add_directed_edge(i, idx_goal, d)

        idx_origin = idx_start
        coords_origin = coords[idx_origin]
        # the visibility of only the graphs nodes have to be checked
        # the goal node does not have to be considered, because of the earlier check
        edge_idxs2check = set(range(nr_edges))  # new copy
        vert_idx2repr, vert_idx2dist = get_repr_n_dists(idx_origin, coords)
        candidate_idxs = {i for i in self.graph.get_all_nodes() if not vert_idx2dist[i] == 0.0}
        visible_idxs = find_visible2(
            extremity_mask,
            coords,
            vertex_edge_idxs,
            edge_vertex_idxs,
            edge_idxs2check,
            coords_origin,
            vert_idx2repr,
            vert_idx2dist,
            candidate_idxs,
            idx_origin,
        )

        if len(visible_idxs) == 0:
            # The start node does not have any neighbours. Hence there is no possible path to the goal.
            return [], None

        # add edges in the direction: start -> extremity
        visibles_n_distances_map = {i: vert_idx2dist[i] for i in visible_idxs}
        graph.add_multiple_directed_edges(idx_start, visibles_n_distances_map)

        # Note: also here unnecessary edges in the graph could be deleted when start or goal lie
        # optimising the graph here however is more expensive than beneficial,
        # as it is only being used for a single query

        # ATTENTION: update to new coordinates
        graph.coord_map = {i: coords[i] for i in graph.all_nodes}
        graph.join_identical()

        vertex_id_path, distance = graph.modified_a_star(idx_start, idx_goal, coords_goal)

        # clean up
        # TODO re-use the same graph
        graph.remove_node(idx_start)
        graph.remove_node(idx_goal)
        if free_space_after:
            del graph  # free the memory

        else:
            self.temp_graph = graph

        # extract the coordinates from the path
        vertex_path = [tuple(coords[i]) for i in vertex_id_path]
        return vertex_path, distance


if __name__ == "__main__":
    # TODO command line support. read polygons and holes from .json files?
    pass
