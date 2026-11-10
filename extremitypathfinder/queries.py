"""shortest path queries in a prepared environment

The algorithms behind the query methods of ``PolygonEnvironment``: the visibility of query points,
the search in the visibility graph extended by the temporary edges of start and goal (``search()``)
and the shortest path lengths between many points (``distance_matrix()``).

Queries work on a copy of the coordinates extended by the query points and on query local data.
They never change the environment or its visibility graph (cf. the invariants of the contributing guide).
"""

import itertools
from collections.abc import Iterable
from dataclasses import dataclass, field
from functools import cached_property

import numpy as np

from extremitypathfinder import configs, utils
from extremitypathfinder import types as t
from extremitypathfinder.types import InputCoord, Length, Path
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
      Only built on first access (a copy of the visibility graph of the environment at that time).
      ``None`` without a graph search.
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


def graph_node(point: Point | None) -> int | None:
    """the graph node of a point (an extremity), ``None`` if the point is no extremity"""
    if point is None or not point.is_extremity:
        return None
    return point.idx


def _coordinate(point: t.FloatArray) -> t.Coordinate:
    """the point as a tuple of Python floats"""
    return float(point[0]), float(point[1])


def as_coord_array(points: Iterable[InputCoord]) -> np.ndarray:
    """the query points as a float array of shape (n, 2)"""
    coords = np.array(list(points), dtype=configs.DTYPE_FLOAT)
    if coords.size == 0:
        return coords.reshape(0, 2)
    if coords.ndim != 2 or coords.shape[1] != 2:
        raise ValueError(
            f"the points must be (x,y) coordinate pairs, got an array of shape {coords.shape}"
        )
    if not np.all(np.isfinite(coords)):
        raise ValueError("the coordinates of the points must be finite")
    return coords


def find_visible_from_query(
    origin: int,
    candidates: Iterable[int],
    coords: np.ndarray,
    vert_idx2repr: np.ndarray,
    vert_idx2dist: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
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
        point = points.at(coords[origin], origin)
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
        edge_vertex_idxs,
        points,
        query_points,
    )


def search_graph(
    graph: t.Graph,
    start: int,
    goal: int,
    query_edges: list[tuple[int, int, float]],
    goal_distances: np.ndarray,
) -> tuple[list[int] | None, Length]:
    """the shortest path between two graph nodes (or query points) with A*, and its length

    Searches the visibility graph extended by the temporary edges of start and goal,
    without changing the graph (cf. ``utils.astar_path()``).

    :param query_edges: the temporary edges ``(node, neighbour, length)`` of start and goal
    :param goal_distances: the distances of all coordinates to the goal: the A* heuristic
    :return: the node indices of the path and its length, ``(None, None)`` if there is no path
    """
    query_adjacency: dict[int, dict[int, float]] = {}
    for i, j, d in query_edges:
        query_adjacency.setdefault(i, {})[j] = d
        query_adjacency.setdefault(j, {})[i] = d

    def distance_to_goal(node: int, _goal: int) -> float:
        return goal_distances[node]

    id_path = utils.astar_path(graph, query_adjacency, start, goal, distance_to_goal)
    if id_path is None:
        return None, None

    # the length of the path: the sum of the weights of its edges (Euclidean distances)
    distance = 0.0
    for v1, v2 in itertools.pairwise(id_path):
        distance += utils.edge_weight(graph, query_adjacency, v1, v2)
    return id_path, distance


def search(
    coords_start: t.FloatArray,
    coords_goal: t.FloatArray,
    graph: t.Graph,
    vertex_coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
) -> SearchResult:
    """the shortest path between start and goal (cf. :meth:`PolygonEnvironment.search`)

    :param coords_start: the (x,y) coordinates of the start, two finite values
    :param coords_goal: the (x,y) coordinates of the goal, two finite values
    :param vertex_coords: the coordinates of all polygon vertices.
        Start and goal get the indices after them.
    :return: the result, with the path as tuples of Python floats and the length as a Python float
    """
    start_coordinates = _coordinate(coords_start)
    goal_coordinates = _coordinate(coords_goal)
    start = len(vertex_coords)
    goal = start + 1
    # temporarily extend data structure
    coords = np.append(vertex_coords, (coords_start, coords_goal), axis=0)

    def no_graph_result(path: Path, length: Length) -> SearchResult:
        return SearchResult(path, length, coords)

    if np.array_equal(coords_start, coords_goal):
        # start and goal are identical and can be reached instantly
        return no_graph_result([start_coordinates, goal_coordinates], 0.0)

    # the points at start and goal, and the graph nodes at their positions (merged with them)
    query_points = {
        start: points.at(coords_start, start),
        goal: points.at(coords_goal, goal),
    }
    start_node, goal_node = (graph_node(query_points[i]) for i in (start, goal))

    # check the goal node first (earlier termination possible)
    origin = goal
    # the visibility of only the graph nodes has to be checked (not all extremities!)
    # IMPORTANT: also check if the start node is visible from the goal node!
    candidate_idxs: set[int] = set(graph.nodes)
    candidate_idxs.add(start)
    # Note: the graph node at the start position must not hide the start
    #   (only the closest of candidates in the same direction is visible, cf. find_visible()):
    #   the start may leave its point through any free wedge, the graph node only through its own
    candidates_goal = candidate_idxs - {start_node}
    vert_idx2repr, vert_idx2dist = utils.cmp_reps_n_distances(origin, coords)
    # the distances to the goal: the A* heuristic
    # (also for a graph node merged with the goal: identical coordinates, identical distances)
    goal_distances = vert_idx2dist
    visibles_goal = find_visible_from_query(
        origin,
        candidates_goal,
        coords,
        vert_idx2repr,
        vert_idx2dist,
        edge_vertex_idxs,
        points,
        query_points,
    )
    if len(visibles_goal) == 0:
        # The goal node does not have any neighbours. Hence there is not possible path to the goal.
        return no_graph_result([], None)

    # IMPORTANT geometrical property of this problem: it is always shortest to directly reach a node
    #   instead of visiting other nodes first (there is never an advantage through reduced edge weight)
    # -> when goal is directly reachable, there can be no other shorter path to it. Terminate
    if start in visibles_goal:
        d = float(vert_idx2dist[start])
        return no_graph_result([start_coordinates, goal_coordinates], d)

    # merge start and goal with the graph nodes at their positions
    start_mapped = start if start_node is None else start_node
    goal_mapped = goal if goal_node is None else goal_node

    # the temporary edges connecting start and goal to the graph
    # Note: the precomputed graph is neither changed nor copied (copying is expensive for large graphs)
    #   https://github.com/jannikmi/extremitypathfinder/issues/136
    # Note: also here unnecessary edges in the graph could be deleted
    # optimising the graph here however is more expensive than beneficial,
    # as the graph is only being used for a single query
    query_edges = [(i, goal_mapped, vert_idx2dist[i]) for i in visibles_goal]

    origin = start
    # the visibility of only the graphs nodes have to be checked
    # the goal node does not have to be considered, because of the earlier check
    vert_idx2repr, vert_idx2dist = utils.cmp_reps_n_distances(origin, coords)
    visibles_start = find_visible_from_query(
        origin,
        candidate_idxs,
        coords,
        vert_idx2repr,
        vert_idx2dist,
        edge_vertex_idxs,
        points,
        query_points,
    )

    if len(visibles_start) == 0:
        # The start node does not have any neighbours. Hence there is no possible path to the goal.
        return no_graph_result([], None)

    query_edges += [(start_mapped, i, vert_idx2dist[i]) for i in visibles_start]

    def graph_result(path: Path, length: Length) -> SearchResult:
        return SearchResult(
            path,
            length,
            coords,
            start_mapped,
            goal_mapped,
            query_edges,
            graph,
        )

    id_path, distance = search_graph(
        graph, start_mapped, goal_mapped, query_edges, goal_distances
    )
    if id_path is None:
        return graph_result([], None)

    # extract the coordinates from the path
    path = [_coordinate(coords[i]) for i in id_path]
    return graph_result(path, float(distance))


def distance_matrix(
    coords_a: np.ndarray,
    coords_b: np.ndarray,
    graph: t.Graph,
    vertex_coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
) -> np.ndarray:
    """the shortest path lengths between all points of ``coords_a`` and ``coords_b``

    Identical to :func:`search` for every pair (cf. :meth:`PolygonEnvironment.distance_matrix`),
    but the visibility of every point is computed only once.

    :param coords_a: the start points, shape (n, 2)
    :param coords_b: the goal points, shape (m, 2)
    :param vertex_coords: the coordinates of all polygon vertices
    :return: the lengths, shape (n, m), ``inf`` without a path
    """
    nr_vertices = len(vertex_coords)
    # each position is computed once: query points with identical coordinates are merged
    unique_idx: dict[tuple[float, float], int] = {}
    unique_coords: list[np.ndarray] = []

    def unique(query_coords: np.ndarray) -> list[int]:
        idxs = []
        for point in query_coords:
            key = (float(point[0]), float(point[1]))
            if key not in unique_idx:
                unique_idx[key] = nr_vertices + len(unique_coords)
                unique_coords.append(point)
            idxs.append(unique_idx[key])
        return idxs

    idxs_a = unique(coords_a)
    idxs_b = unique(coords_b)
    matrix = np.full((len(idxs_a), len(idxs_b)), np.inf, dtype=configs.DTYPE_FLOAT)
    if len(idxs_a) == 0 or len(idxs_b) == 0:
        return matrix

    sources = list(dict.fromkeys(idxs_a))
    targets = list(dict.fromkeys(idxs_b))
    # temporarily extend data structure (cf. search())
    coords = np.append(vertex_coords, np.array(unique_coords), axis=0)
    query_points = {i: points.at(coords[i], i) for i in unique_idx.values()}
    graph_nodes = np.fromiter(graph.nodes, dtype=int)
    graph_node_at = {i: graph_node(point) for i, point in query_points.items()}

    # the visibility of every query point, from the point (cf. search(): the visibility is checked from the goal)
    # query point -> visible graph node -> distance
    visible_nodes: dict[int, dict[int, float]] = {}
    # target -> source -> distance, for the sources directly visible from the target
    visible_sources: dict[int, dict[int, float]] = {}
    is_target = set(targets)
    for origin in unique_idx.values():
        vert_idx2repr, vert_idx2dist = utils.cmp_reps_n_distances(origin, coords)
        query_candidates = sources if origin in is_target else []
        visible = _find_visible_from_query_point(
            origin,
            graph_nodes,
            query_candidates,
            coords,
            vert_idx2repr,
            vert_idx2dist,
            edge_vertex_idxs,
            points,
            query_points,
        )
        visible_nodes[origin] = {
            i: vert_idx2dist[i] for i in visible if i < nr_vertices
        }
        visible_sources[origin] = {
            i: vert_idx2dist[i] for i in visible if i >= nr_vertices
        }

    distances: dict[tuple[int, int], float] = {}
    for target in targets:
        # the distances seen from the goal: the A* heuristic (cf. search())
        goal_distances = utils.cmp_reps_n_distances(target, coords)[1]
        goal_node = graph_node_at[target]
        goal_mapped = target if goal_node is None else goal_node
        goal_edges = [(i, goal_mapped, d) for i, d in visible_nodes[target].items()]
        for source in sources:
            if source == target:
                distances[source, target] = 0.0
            elif source in visible_sources[target]:
                # the shortest path is the direct line of sight (cf. search())
                distances[source, target] = visible_sources[target][source]
            elif not goal_edges or not visible_nodes[source]:
                # the goal or the start does not see any graph node: no path (cf. search())
                distances[source, target] = np.inf
            else:
                start_node = graph_node_at[source]
                start_mapped = source if start_node is None else start_node
                query_edges = goal_edges + [
                    (start_mapped, i, d) for i, d in visible_nodes[source].items()
                ]
                _, length = search_graph(
                    graph, start_mapped, goal_mapped, query_edges, goal_distances
                )
                distances[source, target] = np.inf if length is None else length

    for i, source in enumerate(idxs_a):
        for j, target in enumerate(idxs_b):
            matrix[i, j] = distances[source, target]
    return matrix


def _find_visible_from_query_point(
    origin: int,
    graph_nodes: np.ndarray,
    query_candidates: list[int],
    coords: np.ndarray,
    vert_idx2repr: np.ndarray,
    vert_idx2dist: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
    query_points: dict[int, Point | None],
) -> set[int]:
    """the graph nodes and the other query points visible from a query point

    Of candidates with identical angle representation only the closest one is visible
    (cf. ``utils.find_visible()``).
    The visibility of each query candidate is the same as in :func:`search` with the origin as goal
    and the candidate as start: it is hidden by a closer graph node in the same direction
    (the path then passes through that node), but not by the graph node at its own position
    or by other query points. A query point must not hide graph nodes or other query points either:
    query points sharing a representation with another candidate are checked in separate rounds.
    """
    # representation -> distance of the closest graph node in this direction
    closest_node: dict[float, float] = {}
    for rep, dist in zip(
        vert_idx2repr[graph_nodes].tolist(),
        vert_idx2dist[graph_nodes].tolist(),
    ):
        if dist < closest_node.get(rep, np.inf):
            closest_node[rep] = dist
    rounds: list[dict[float, int]] = [{}]
    for i in query_candidates:
        dist = vert_idx2dist[i]
        if dist == 0.0:
            continue
        rep = float(vert_idx2repr[i])
        first = 0
        if rep in closest_node:
            # Note: the graph node at the position of the candidate has the same distance
            if closest_node[rep] < dist:
                # hidden by a closer graph node
                continue
            first = 1
        for k in itertools.count(first):
            if k == len(rounds):
                rounds.append({})
            if rep not in rounds[k]:
                rounds[k][rep] = i
                break
    visible: set[int] = set()
    for k, query_round in enumerate(rounds):
        candidates = set(query_round.values())
        if k == 0:
            candidates.update(graph_nodes.tolist())
        if not candidates:
            continue
        visible |= find_visible_from_query(
            origin,
            candidates,
            coords,
            vert_idx2repr,
            vert_idx2dist,
            edge_vertex_idxs,
            points,
            query_points,
        )
    return visible
