import pickle
from collections import ChainMap
from collections.abc import Iterable
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np

from extremitypathfinder import configs, utils
from extremitypathfinder import types as t
from extremitypathfinder.configs import DEFAULT_PICKLE_NAME
from extremitypathfinder.types import (
    InputCoord,
    InputCoordList,
    Length,
    ObstacleIterator,
    OptionalInputCoordList,
    Path,
)
from extremitypathfinder.wedges import Point, Points


@dataclass
class SearchResult:
    """The result of a shortest path search with :meth:`PolygonEnvironment.search`.

    * ``path``: the shortest path from start to goal, ``[]`` if there is no possible path
    * ``length``: the length of the path, ``None`` if there is no possible path
    * ``coords``: the coordinates of all polygon vertices followed by the start and the goal coordinates
    * ``idx_start``, ``idx_goal``: the nodes of start and goal in the search graph (indices in ``coords``).
      Start and goal are merged with graph nodes with identical coordinates.
      ``None`` when the search terminated without a graph search,
      e.g. when the goal is directly visible from the start.
    * ``query_edges``: the temporary edges ``(node, neighbour, length)``
      connecting start and goal to the visibility graph (``None`` without a graph search)
    * ``graph``: the search graph (the visibility graph extended by the query edges).
      Only built on request (a copy of the visibility graph). ``None`` without a graph search.
    """

    path: Path
    length: Length
    coords: np.ndarray
    idx_start: int | None = None
    idx_goal: int | None = None
    query_edges: list[tuple[int, int, float]] | None = None
    # the precomputed visibility graph of the environment (not changed by the search)
    visibility_graph: t.Graph | None = field(default=None, repr=False, compare=False)

    @cached_property
    def graph(self) -> t.Graph | None:
        if self.query_edges is None:
            return None
        graph = self.visibility_graph.copy()
        graph.add_weighted_edges_from(self.query_edges)
        return graph


class PolygonEnvironment:
    """Class allowing to use polygons to represent "2D environments" and use them for path finding.

    Keeps a "loaded" environment with a precomputed visibility graph for consecutive path queries.
    Internally uses a visibility graph optimised for shortest path finding.
    General approach and some optimisations theoretically described in:
    [1] Vinther, Anders Strand-Holm, Magnus Strand-Holm Vinther, and Peyman Afshani.
    "`Pathfinding in Two-dimensional Worlds
    <https://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__"

    The environment is initialised in one step: the polygons are passed to the constructor
    (or a grid with blocked cells to :meth:`from_grid`) and the visibility graph is computed right away.
    Afterwards these attributes are available:

    * ``boundary_polygon``: the boundary coordinates (``None`` for unbounded environments)
    * ``holes``: the list of hole coordinates
    * ``coords``: the coordinates of all polygon vertices (boundary first, then all holes)
    * ``extremity_indices``: the indices of all extremities in ``coords`` (at most one per vertex position)
    * ``graph``: the precomputed visibility graph (:class:`networkx.Graph`) of the extremities
    * ``points``: the vertex positions within the map and the free space around them
      (cf. :mod:`extremitypathfinder.wedges`)

    Obstacles are the holes and everything outside the boundary polygon.
    Obstacles that touch form a wall: a path never passes between two obstacles that touch
    (shared vertices, a vertex on an edge, overlapping edges), and never through a point where
    an obstacle touches itself. Paths may run along obstacle edges and through the vertices of a single obstacle.
    A contact means exactly equal coordinates or a vertex exactly on an edge, near-contacts (tiny gaps) do not block.
    Overlapping polygons act as their union.
    """

    nr_vertices: int
    holes: list[np.ndarray]
    extremity_indices: np.ndarray
    reprs_n_distances: dict[int, tuple[np.ndarray, np.ndarray]]
    graph: t.Graph
    boundary_polygon: np.ndarray | None
    coords: np.ndarray
    edge_vertex_idxs: np.ndarray
    vertex_edge_idxs: np.ndarray
    points: Points

    def __init__(
        self,
        boundary_coordinates: OptionalInputCoordList,
        list_of_hole_coordinates: InputCoordList,
        validate: bool = False,
    ):
        """stores the passed input polygons in the environment and computes the visibility graph

        .. note:: the passed polygons must meet these requirements:

            * given as numpy or python array of coordinate tuples: ``[(x1,y1), (x2,y2,)...]``
            * no repetition of the first point at the end
            * at least 3 vertices (no single points or lines allowed)
            * no consequent vertices with identical coordinates in the polygons (same coordinates allowed)
            * no self intersections. A polygon may touch itself (identical vertices, a vertex on an edge),
              but must not cross itself there
            * no collinear overlapping edges within a polygon (zero-width spikes or slits)
            * edge numbering has to follow these conventions: boundary polygon counter clockwise, holes clockwise

            Polygons may touch or intersect each other. Touching obstacles block paths.

        :param boundary_coordinates: array of coordinates with counter clockwise edge numbering.
            When ``None``, the environment is unbounded and at least one hole must
            be supplied.
        :param list_of_hole_coordinates: array of coordinates with clockwise edge numbering
        :param validate: whether the requirements of the data should be tested

        :raises TypeError: when validate=True and the input has an invalid shape.
        :raises ValueError: when validate=True and the input violates the requirements above,
            and when ``boundary_coordinates`` is ``None`` and no holes are given.
        """
        # loading the map
        list_of_hole_coordinates = [
            np.array(hole_coords, dtype=configs.DTYPE_FLOAT)
            for hole_coords in list_of_hole_coordinates
        ]
        if boundary_coordinates is not None:
            boundary_coordinates = np.array(
                boundary_coordinates, dtype=configs.DTYPE_FLOAT
            )
        elif not list_of_hole_coordinates:
            raise ValueError(
                "At least one hole is required when boundary_coordinates is None."
            )
        if validate:
            if boundary_coordinates is None:
                utils.check_hole_data_requirements(list_of_hole_coordinates)
            else:
                utils.check_data_requirements(
                    boundary_coordinates, list_of_hole_coordinates
                )

        # Note: independent copy!
        self.holes = list_of_hole_coordinates
        self.boundary_polygon = boundary_coordinates

        (
            self.coords,
            self.edge_vertex_idxs,
            self.vertex_edge_idxs,
            self.points,
        ) = utils.compile_polygon_datastructs(
            boundary_coordinates, list_of_hole_coordinates
        )
        self.extremity_indices = self.points.extremity_idxs

        self.nr_vertices = len(self.coords)
        # the angle representations and distances of all vertices seen from every extremity
        self.reprs_n_distances = utils.cmp_reps_n_distance_dict(
            self.coords, self.extremity_indices
        )

        # start and goal points will be stored after all polygon coordinates
        self.idx_start = self.nr_vertices
        self.idx_goal = self.nr_vertices + 1

        self._compute_graph()

    @classmethod
    def from_grid(
        cls,
        size_x: int,
        size_y: int,
        obstacle_iter: ObstacleIterator,
        validate: bool = False,
    ) -> "PolygonEnvironment":
        """Convert a grid with blocked cells (obstacles) into a polygon environment

        Prerequisites: the grid must not have single non-obstacle cells which are surrounded by obstacles
        ("white cells in black surrounding" = useless for path planning)

        Diagonally adjacent blocked cells touch at a corner and block paths (no corner cutting).

        :param size_x: the horizontal grid size
        :param size_y: the vertical grid size
        :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
        :param validate: whether the input should be validated
        """
        boundary_coordinates, list_of_hole_coordinates = utils.convert_grid(
            size_x, size_y, obstacle_iter
        )
        return cls(boundary_coordinates, list_of_hole_coordinates, validate)

    @property
    def nr_edges(self) -> int:
        return self.nr_vertices

    @property
    def all_extremities(self) -> list[tuple]:
        coords = self.coords
        return [tuple(coords[i]) for i in self.extremity_indices]

    @property
    def all_vertices(self) -> list[tuple]:
        coords = self.coords
        return [tuple(coords[i]) for i in range(self.nr_vertices)]

    def export_pickle(self, path: str = DEFAULT_PICKLE_NAME):
        print("storing map class in:", path)
        with open(path, "wb") as f:
            pickle.dump(self, f)
        print("done.\n")

    def _compute_graph(self):
        """Computes a visibility graph optimized (=reduced) for path planning and stores it

        Computes all directly reachable extremities based on visibility and their distance to each other
        pre-procesing of the map. pre-computation for faster shortest path queries
        optimizes graph further at construction time

        NOTE: initialise the graph with all extremities.
        even if a node has no edges (visibility to other extremities, dangling node),
        it must still be included!

        .. note::
            The extremities are the vertex positions with a free wedge larger than 180 degree,
            at most one per position (cf. :mod:`extremitypathfinder.wedges`).

        .. note::
            Pre computing the shortest paths between all directly reachable extremities
            and storing them in the graph would not be an advantage, because then the graph is fully connected.
            A* would visit every node in the graph at least once (-> disadvantage!).
        """
        self.graph = utils.compute_graph(
            self.extremity_indices,
            self.reprs_n_distances,
            self.coords,
            self.edge_vertex_idxs,
            self.points,
        )

    def within_map(self, coords: np.ndarray) -> bool:
        """Check whether coordinates lie outside holes and within any boundary.

        :param coords: numerical tuple representing coordinates
        :return: whether the given coordinate is a valid query point
        """
        return utils.is_within_map(coords, self.boundary_polygon, self.holes)

    def get_visible_idxs(
        self,
        origin: int,
        candidates: Iterable[int],
        coords: np.ndarray,
        vert_idx2repr: np.ndarray,
        vert_idx2dist: np.ndarray,
        query_points: dict[int, Point | None] | None = None,
    ) -> set[int]:
        """finds the candidates visible from a query point (start or goal)

        A query point may leave through any free wedge of its point (cf. :mod:`extremitypathfinder.wedges`).

        :param query_points: the points at the query points (``None``: no polygon passes the query point).
            Looked up when not given.
        """
        if query_points is not None and origin in query_points:
            point = query_points[origin]
        else:
            point = self.points.at(coords[origin], origin)
        wedges = [] if point is None else point.wedges
        # Note: candidates with identical coordinates are not visible (they get merged)
        candidates = {i for i in candidates if vert_idx2dist[i] != 0.0}
        return utils.find_visible_from(
            origin,
            point,
            wedges,
            candidates,
            coords,
            vert_idx2repr,
            vert_idx2dist,
            self.edge_vertex_idxs,
            self.points,
            query_points,
        )

    def find_shortest_path(
        self,
        start_coordinates: InputCoord,
        goal_coordinates: InputCoord,
        verify: bool = True,
    ) -> tuple[Path, Length]:
        """computes the shortest path and its length between start and goal node

        :param start_coordinates: a (x,y) coordinate tuple representing the start node
        :param goal_coordinates:  a (x,y) coordinate tuple representing the goal node
        :param verify: whether it should be checked if start and goal points really lie inside the environment.
         if points close to or on polygon edges should be accepted as valid input, set this to ``False``.
         Start and goal may lie on a contact point of touching obstacles, the path may leave through
         any free space next to it. On an edge shared by two touching obstacles (inside a wall) there is no path.
        :return: a tuple of shortest path and its length. ([], None) if there is no possible path.
        """
        result = self.search(start_coordinates, goal_coordinates, verify)
        return result.path, result.length

    def search(
        self,
        start_coordinates: InputCoord,
        goal_coordinates: InputCoord,
        verify: bool = True,
    ) -> SearchResult:
        """computes the shortest path between start and goal node, like :meth:`find_shortest_path`

        In addition to the path and its length, the returned :class:`SearchResult`
        contains the data of the search, e.g. for plotting it.
        The precomputed visibility graph is not changed by the search.

        :param start_coordinates: a (x,y) coordinate tuple representing the start node
        :param goal_coordinates:  a (x,y) coordinate tuple representing the goal node
        :param verify: whether it should be checked if start and goal points really lie inside the environment.
         if points close to or on polygon edges should be accepted as valid input, set this to ``False``.
        :return: the result of the search
        """
        coords_start = np.array(start_coordinates, dtype=float)
        coords_goal = np.array(goal_coordinates, dtype=float)
        if verify and not self.within_map(coords_start):
            raise ValueError("start point does not lie within the map")
        if verify and not self.within_map(coords_goal):
            raise ValueError("goal point does not lie within the map")

        start = self.idx_start
        goal = self.idx_goal
        # temporarily extend data structure
        coords = np.append(self.coords, (coords_start, coords_goal), axis=0)

        def no_graph_result(path: Path, length: Length) -> SearchResult:
            return SearchResult(path, length, coords)

        if np.array_equal(coords_start, coords_goal):
            # start and goal are identical and can be reached instantly
            return no_graph_result([start_coordinates, goal_coordinates], 0.0)

        # the points at start and goal, and the graph nodes at their positions (merged with them)
        query_points = {
            start: self.points.at(coords_start, start),
            goal: self.points.at(coords_goal, goal),
        }
        start_node, goal_node = (_graph_node(query_points[i]) for i in (start, goal))

        # the angle representations and distances seen from start and goal
        # Note: the environment does not get modified by the query
        query_reprs_n_distances = {}
        reprs_n_distances = ChainMap(query_reprs_n_distances, self.reprs_n_distances)

        # check the goal node first (earlier termination possible)
        origin = goal
        # the visibility of only the graph nodes has to be checked (not all extremities!)
        # IMPORTANT: also check if the start node is visible from the goal node!
        candidate_idxs: set[int] = set(self.graph.nodes)
        candidate_idxs.add(start)
        # Note: the graph node at the start position must not hide the start
        #   (only the closest of candidates in the same direction is visible, cf. find_visible()):
        #   the start may leave its point through any free wedge, the graph node only through its own
        candidates_goal = candidate_idxs - {start_node}
        vert_idx2repr, vert_idx2dist = query_reprs_n_distances[origin] = (
            utils.cmp_reps_n_distances(origin, coords)
        )
        visibles_goal = self.get_visible_idxs(
            origin,
            candidates_goal,
            coords,
            vert_idx2repr,
            vert_idx2dist,
            query_points,
        )
        if len(visibles_goal) == 0:
            # The goal node does not have any neighbours. Hence there is not possible path to the goal.
            return no_graph_result([], None)

        # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
        #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
        # -> when goal is directly reachable, there can be no other shorter path to it. Terminate
        if start in visibles_goal:
            d = vert_idx2dist[start]
            return no_graph_result([start_coordinates, goal_coordinates], d)

        # merge start and goal with the graph nodes at their positions
        start_mapped = start if start_node is None else start_node
        goal_mapped = goal if goal_node is None else goal_node

        # the temporary edges connecting start and goal to the graph
        # Note: the precomputed self.graph is neither changed nor copied (copying is expensive for large graphs)
        #   https://github.com/jannikmi/extremitypathfinder/issues/136
        # Note: also here unnecessary edges in the graph could be deleted
        # optimising the graph here however is more expensive than beneficial,
        # as the graph is only being used for a single query
        query_edges = [(i, goal_mapped, vert_idx2dist[i]) for i in visibles_goal]

        origin = start
        # the visibility of only the graphs nodes have to be checked
        # the goal node does not have to be considered, because of the earlier check
        vert_idx2repr, vert_idx2dist = query_reprs_n_distances[origin] = (
            utils.cmp_reps_n_distances(origin, coords)
        )
        visibles_start = self.get_visible_idxs(
            origin,
            candidate_idxs,
            coords,
            vert_idx2repr,
            vert_idx2dist,
            query_points,
        )

        if len(visibles_start) == 0:
            # The start node does not have any neighbours. Hence there is no possible path to the goal.
            return no_graph_result([], None)

        query_edges += [(start_mapped, i, vert_idx2dist[i]) for i in visibles_start]
        query_adjacency: dict[int, dict[int, float]] = {}
        for i, j, d in query_edges:
            query_adjacency.setdefault(i, {})[j] = d
            query_adjacency.setdefault(j, {})[i] = d

        def graph_result(path: Path, length: Length) -> SearchResult:
            return SearchResult(
                path,
                length,
                coords,
                start_mapped,
                goal_mapped,
                query_edges,
                self.graph,
            )

        def l2_distance(n1, n2):
            return utils.get_distance(n1, n2, reprs_n_distances)

        id_path = utils.astar_path(
            self.graph, query_adjacency, start_mapped, goal_mapped, l2_distance
        )
        if id_path is None:
            return graph_result([], None)

        # compute distance
        distance = 0.0
        v1 = id_path[0]
        for v2 in id_path[1:]:
            distance += l2_distance(v1, v2)
            v1 = v2

        # extract the coordinates from the path
        path = [tuple(coords[i]) for i in id_path]
        return graph_result(path, distance)


def _graph_node(point: Point | None) -> int | None:
    """the graph node of a point (an extremity), ``None`` if the point is no extremity"""
    if point is None or not point.is_extremity:
        return None
    return point.idx
