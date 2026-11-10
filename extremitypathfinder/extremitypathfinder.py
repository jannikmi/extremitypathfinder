import pickle
from collections.abc import Iterable
from typing import Self

import numpy as np

from extremitypathfinder import configs, queries, utils
from extremitypathfinder import types as t
from extremitypathfinder.configs import DEFAULT_PICKLE_NAME
from extremitypathfinder.queries import SearchResult
from extremitypathfinder.types import (
    InputCoord,
    InputCoordList,
    Length,
    ObstacleIterator,
    OptionalInputCoordList,
    Path,
)
from extremitypathfinder.wedges import Point, Points

__all__ = ("PolygonEnvironment", "SearchResult")

_PICKLE_FORMAT_KEY = "_pickle_format_version"


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

    * ``boundary_polygon``: the boundary coordinates (``None`` for unbounded environments),
      counter clockwise numbering (reversed if given clockwise)
    * ``holes``: the list of hole coordinates, clockwise numbering (reversed if given counter clockwise)
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
    graph: t.Graph
    boundary_polygon: np.ndarray | None
    coords: np.ndarray
    edge_vertex_idxs: np.ndarray
    vertex_edge_idxs: np.ndarray
    points: Points

    def __init__(
        self,
        boundary_coordinates: OptionalInputCoordList,
        list_of_hole_coordinates: Iterable[InputCoordList],
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
            * the vertex numbering may be clockwise or counter clockwise. Internally the boundary polygon
              is numbered counter clockwise and the holes clockwise: polygons with the opposite numbering
              are stored reversed (``boundary_polygon``, ``holes``)

            Polygons may touch or intersect each other. Touching obstacles block paths.

        :param boundary_coordinates: array of coordinates (clockwise or counter clockwise numbering).
            When ``None``, the environment is unbounded and at least one hole must
            be supplied.
        :param list_of_hole_coordinates: arrays of coordinates (clockwise or counter clockwise numbering)
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
        # accept both vertex numberings: reverse polygons with the opposite numbering
        input_boundary, input_holes = boundary_coordinates, list_of_hole_coordinates
        boundary_coordinates, list_of_hole_coordinates = utils.orient_polygons(
            boundary_coordinates, list_of_hole_coordinates
        )
        if validate:
            # the error messages give the vertex indices of the input
            holes_reversed = [
                hole is not input_hole
                for hole, input_hole in zip(list_of_hole_coordinates, input_holes)
            ]
            if boundary_coordinates is None:
                utils.check_hole_data_requirements(
                    list_of_hole_coordinates, holes_reversed
                )
            else:
                utils.check_data_requirements(
                    boundary_coordinates,
                    list_of_hole_coordinates,
                    boundary_coordinates is not input_boundary,
                    holes_reversed,
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

    @classmethod
    def from_shapely(
        cls,
        boundary: t.GeometryInput,
        obstacles: t.GeometryInput = None,
        validate: bool = False,
    ) -> Self:
        """Create an environment from `Shapely <https://shapely.readthedocs.io/>`__ or GeoJSON-like geometries

        Requires Shapely (``pip install "extremitypathfinder[shapely]"``).
        The geometries are converted into the coordinate input of the constructor:
        the result is the same environment as from the equivalent coordinate lists.

        * the exterior ring of the boundary polygon becomes the boundary, its interior rings become holes
        * the exterior rings of the obstacle polygons become holes. Their interior rings (e.g. islands in a lake)
          are ignored with a warning: the free space inside an obstacle cannot be reached from the outside
          and is treated as part of the obstacle
        * the repeated closing vertex of the rings and repeated consecutive vertices are removed,
          a third (z) coordinate is ignored

        Geometries may be Shapely geometries, GeoJSON-like mappings (geometries, features, feature collections)
        or objects with a ``__geo_interface__`` (e.g. GeoPandas geometries).

        :param boundary: the boundary as a single polygon. When ``None``, the environment is unbounded
            and at least one obstacle must be supplied.
        :param obstacles: the obstacles: a polygon, a multi-polygon, a geometry collection of polygons
            or an iterable of these. Empty geometries are skipped.
        :param validate: whether the converted input should be validated,
            with the same validation as the coordinate input of the constructor
        :raises ImportError: when Shapely is not installed
        :raises TypeError: when a geometry is not a polygon (or a collection of polygons)
        :raises ValueError: when the boundary is not a single polygon,
            when a ring has fewer than 3 distinct vertices, and the errors of the constructor
        """
        boundary_coordinates, list_of_hole_coordinates = utils.convert_shapely(
            boundary, obstacles
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

    def __getstate__(self):
        """adds the pickle format version to the state. Subclasses overriding this must call ``super()``"""
        state = super().__getstate__()
        # (dict, slots) for subclasses with ``__slots__``
        state, slots = state if isinstance(state, tuple) else (state, None)
        state = {**state, _PICKLE_FORMAT_KEY: configs.PICKLE_FORMAT_VERSION}
        return state if slots is None else (state, slots)

    def __setstate__(self, state):
        state, slots = state if isinstance(state, tuple) else (state, None)
        state = dict(state)
        pickle_format = state.pop(_PICKLE_FORMAT_KEY, None)
        if pickle_format != configs.PICKLE_FORMAT_VERSION:
            found = (
                "no pickle format version (pickled before 3.0)"
                if pickle_format is None
                else f"pickle format {pickle_format}"
            )
            raise ValueError(
                "The environment was pickled with an incompatible version of extremitypathfinder "
                f"({found}, expected {configs.PICKLE_FORMAT_VERSION}). "
                "Recreate the environment from its polygons and export it again."
            )
        self.__dict__.update(state)
        for name, value in (slots or {}).items():
            setattr(self, name, value)

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
        return queries.find_visible_from_query(
            origin,
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
        *,
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
        result = self.search(start_coordinates, goal_coordinates, verify=verify)
        return result.path, result.length

    def search(
        self,
        start_coordinates: InputCoord,
        goal_coordinates: InputCoord,
        *,
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
        if verify and not self.within_map(np.array(start_coordinates, dtype=float)):
            raise ValueError("start point does not lie within the map")
        if verify and not self.within_map(np.array(goal_coordinates, dtype=float)):
            raise ValueError("goal point does not lie within the map")
        return queries.search(
            start_coordinates,
            goal_coordinates,
            self.graph,
            self.coords,
            self.edge_vertex_idxs,
            self.points,
        )

    def distance_matrix(
        self,
        points_a: Iterable[InputCoord],
        points_b: Iterable[InputCoord] | None = None,
        *,
        verify: bool = True,
    ) -> np.ndarray:
        """computes the shortest path lengths between many points (a distance matrix)

        Gives the same lengths as :meth:`find_shortest_path` for every pair, but faster:
        the visibility of every point is computed once (instead of twice per pair)
        and the visibility graph is searched with A* for every pair, like :meth:`find_shortest_path`.
        The environment and its visibility graph are not changed.

        :param points_a: the start points, (x,y) coordinate tuples
        :param points_b: the goal points, (x,y) coordinate tuples. ``None``: the points of ``points_a``
        :param verify: whether it should be checked if the points really lie inside the environment
            (cf. :meth:`find_shortest_path`)
        :return: the lengths of the shortest paths, a float array of shape ``(len(points_a), len(points_b))``:
            entry ``[i, j]`` is the length from ``points_a[i]`` to ``points_b[j]``, ``inf`` if there is no path
        :raises ValueError: when ``verify=True`` and a point does not lie within the map
        """
        coords_a = queries.as_coord_array(points_a)
        coords_b = coords_a if points_b is None else queries.as_coord_array(points_b)
        if verify:
            checked = [("points_a", coords_a)]
            if points_b is not None:
                checked.append(("points_b", coords_b))
            for name, query_coords in checked:
                for i, point in enumerate(query_coords):
                    if not self.within_map(point):
                        raise ValueError(
                            f"{name}[{i}] {tuple(point.tolist())} does not lie within the map"
                        )
        return queries.distance_matrix(
            coords_a,
            coords_b,
            self.graph,
            self.coords,
            self.edge_vertex_idxs,
            self.points,
        )
