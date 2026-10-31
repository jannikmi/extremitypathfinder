import json
import pickle
from collections.abc import Callable, Iterable
from heapq import heappop, heappush
from itertools import combinations, count

import networkx as nx
import numpy as np
import numpy.linalg

from extremitypathfinder import configs
from extremitypathfinder import types as t
from extremitypathfinder.configs import (
    BOUNDARY_JSON_KEY,
    DEFAULT_PICKLE_NAME,
    HOLES_JSON_KEY,
)
from extremitypathfinder.utils_numba import (
    _has_clockwise_numbering,
    _inside_polygon,
    _lies_behind,
    _no_identical_consequent_vertices,
    edge_vertex_mappings,
    is_within_map,
    reps_n_distances,
)
from extremitypathfinder.wedges import Point, Points, Wedge, check_polygon_contacts


def cmp_reps_n_distances(
    orig_idx: int, coords: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """the angle representations and distances of all coordinates seen from the origin"""
    return reps_n_distances(coords - coords[orig_idx])


def cmp_reps_n_distance_dict(
    coords: np.ndarray, extremity_indices: np.ndarray
) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    # Note: distance and angle representation relation are symmetric,
    # but exploiting this has been found to be slower than using the numpy functionality with slight overhead
    reps_n_distance_dict = {
        i: cmp_reps_n_distances(i, coords) for i in extremity_indices
    }
    return reps_n_distance_dict


def _get_intersection_status(p1, p2, q1, q2):
    # return:
    #   0: no intersection
    #   1: intersection in ]p1;p2[
    # Note: intersections directly in p1, p2, q1 or q2 are not detected,
    #   crossings at vertices are detected by check_polygon_contacts()
    # solve the set of equations
    # (p2-p1) lambda + (p1) = (q2-q1) mu + (q1)
    #  in matrix form A x = b:
    # [(p2-p1) (q1-q2)] (lambda, mu)' = (q1-p1)
    A = np.array([p2 - p1, q1 - q2]).T
    b = np.array(q1 - p1)
    try:
        x = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        # line segments are parallel (matrix is singular, set of equations is not solvable)
        return 0

    # not crossing the line segment is considered to be ok
    # so x == 0.0 or x == 1.0 is not considered an intersection
    # assert np.allclose((p2 - p1) * x[0] + p1, (q2 - q1) * x[1] + q1)
    # assert np.allclose(np.dot(A, x), b)
    if x[0] <= 0.0 or x[1] <= 0.0 or x[0] >= 1.0 or x[1] >= 1.0:
        return 0
    # if np.all(0.0 <= x) and np.all(x <= 1.0):
    #     return 2
    else:
        return 1


def _no_self_intersection(coords):
    polygon_length = len(coords)
    for index_p1, index_q1 in combinations(range(polygon_length), 2):
        # always: index_p1 < index_q1
        if index_p1 == index_q1 - 1 or (
            index_p1 == 0 and index_q1 == polygon_length - 1
        ):
            # neighbouring edges never have an intersection
            # Note: solving for their common vertex might be imprecise
            continue
        p1, p2 = coords[index_p1], coords[(index_p1 + 1) % polygon_length]
        q1, q2 = coords[index_q1], coords[(index_q1 + 1) % polygon_length]
        intersect_status = _get_intersection_status(p1, p2, q1, q2)
        if intersect_status == 1:
            return False

    return True


def _check_polygon(polygon):
    """ensures that all the following conditions on the polygons are fulfilled:
    - must at least contain 3 vertices
    - no consequent vertices with identical coordinates in the polygons! In general might have the same coordinates
    - a polygon must not have self intersections (intersections with other polygons are allowed).
      A polygon may touch itself (identical vertices, a vertex on an edge), but must not cross itself there
    - no collinear overlapping edges (zero-width spikes or slits)
    """
    if polygon.size == 0:
        raise TypeError("Given polygons must at least contain 3 vertices.")
    if polygon.ndim != 2:
        raise TypeError("Each point of a polygon must consist of two values (x,y).")
    if polygon.shape[0] < 3:
        raise TypeError("Given polygons must at least contain 3 vertices.")
    if polygon.shape[1] != 2:
        raise TypeError("Each point of a polygon must consist of two values (x,y).")
    if not np.all(np.isfinite(polygon)):
        raise ValueError("Polygon coordinates must be finite.")
    if not _no_identical_consequent_vertices(polygon):
        raise ValueError("Consequent vertices of a polynomial must not be identical.")
    if not _no_self_intersection(polygon):
        raise ValueError("The given polygon has self intersections")
    # Translating near the origin avoids catastrophic cancellation for small
    # polygons represented by large-magnitude coordinates.
    translated = polygon - polygon[0]
    x_coords = translated[:, 0]
    y_coords = translated[:, 1]
    area_twice = np.dot(x_coords, np.roll(y_coords, -1)) - np.dot(
        y_coords, np.roll(x_coords, -1)
    )
    if area_twice == 0.0:
        raise ValueError("Polygon area must be non-zero.")
    check_polygon_contacts(polygon)


def orient_polygon(coords: np.ndarray, clockwise: bool) -> np.ndarray:
    """the polygon with the wanted vertex numbering, reversed if it has the opposite numbering

    Input which is no polygon (shape, fewer than 3 vertices) is returned unchanged,
    it is rejected by the validation (cf. ``_check_polygon()``).

    :param coords: the (x,y) coordinates of the polygon
    :param clockwise: whether the vertex numbering should be clockwise
    :return: the polygon (an independent copy if reversed)
    """
    if coords.ndim != 2 or coords.shape[0] < 3 or coords.shape[1] != 2:
        return coords
    if _has_clockwise_numbering(coords) == clockwise:
        return coords
    return coords[::-1].copy()


def orient_polygons(
    boundary_coords: np.ndarray | None, list_hole_coords: list[np.ndarray]
) -> tuple[np.ndarray | None, list[np.ndarray]]:
    """the polygons with the vertex numbering convention (for easier computations):

    * outer boundary polygon: counter clockwise
    * holes: clockwise

    The free space then lies on the left side of every edge.
    """
    if boundary_coords is not None:
        boundary_coords = orient_polygon(boundary_coords, clockwise=False)
    list_hole_coords = [
        orient_polygon(hole_coords, clockwise=True) for hole_coords in list_hole_coords
    ]
    return boundary_coords, list_hole_coords


def check_data_requirements(
    boundary_coords: np.ndarray, list_hole_coords: list[np.ndarray]
):
    """ensures that the basic polygon requirements (s. above) are fulfilled by all polygons

    Note: the vertex numbering is not checked, polygons are oriented beforehand (cf. ``orient_polygons()``)

    :param boundary_coords:
    :param list_hole_coords:
    :return:
    """
    _check_polygon(boundary_coords)
    check_hole_data_requirements(list_hole_coords)


def check_hole_data_requirements(list_hole_coords: list[np.ndarray]):
    """Validate the shape of hole polygons."""
    for hole_coords in list_hole_coords:
        _check_polygon(hole_coords)


def find_visible(
    origin: int,
    candidates: set[int],
    edges_to_check: set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    candidate_edges: dict[int, set[int]] | None = None,
) -> set[int]:
    """finds the candidates whose lines of sight from the origin are not blocked by any of the edges

    "Angle Range Elimination Algorithm" (AREA):
    the edges are sorted after the minimum angle representation of their vertices,
    the candidates after their angle representation.
    Only the candidates within the angle range of an edge have to be checked against it:
    such a candidate is blocked when it lies behind the edge.
    Edges whose angle range contains the representation 0.0 ("crossing" the direction (1, 0))
    are checked separately in a rotated coordinate system.

    A line of sight touching an edge only in one of its vertices (or running along it) is not blocked by the edge.
    Whether lines of sight may pass through the vertices is decided by their free wedges (cf. ``wedges.py``).
    Of candidates with identical angle representation only the closest one is returned.

    :param origin: the index of the coordinates the visibility is checked from
    :param candidates: the indices of the coordinates to check.
        IMPORTANT: must not contain coordinates identical to the origin.
    :param edges_to_check: the edges blocking the lines of sight.
        IMPORTANT: no edge may touch the origin (cf. ``find_visible_from()``).
    :param representations: the angle representations of all coordinates seen from the origin
    :param distances: the distances of all coordinates to the origin
    :param candidate_edges: the edges touching a candidate (e.g. a query point on an edge),
        they do not block it
    :return: the visible candidates
    """
    if len(candidates) == 0:
        return set()
    # only keep the closest of the candidates with identical angle representation
    candidates_sorted = _eliminate_eq_candidates(candidates, distances, representations)
    edges = np.fromiter(edges_to_check, dtype=int, count=len(edges_to_check))
    edges_min_rep, edges_max_rep, edges_max_dist, edges_is_crossing = (
        _edge_angle_ranges(edges, edge_vertex_idxs, representations, distances)
    )

    def check(edges_sorted, edges_min_rep, edges_max_rep, representations):
        _check_candidates(
            candidates_sorted,
            edges_sorted,
            edges_max_rep,
            edges_min_rep,
            origin,
            distances,
            representations,
            coords,
            edge_vertex_idxs,
            edges_max_dist,
            candidate_edges or {},
        )

    regular_edges = edges[~edges_is_crossing[edges]]
    check(
        sorted(regular_edges, key=lambda e: edges_min_rep[e]),
        edges_min_rep,
        edges_max_rep,
        representations,
    )

    crossing_edges = edges[edges_is_crossing[edges]]
    if len(crossing_edges) == 0 or len(candidates_sorted) == 0:
        return set(candidates_sorted)

    candidates_sorted, edges_max_rep, edges_min_rep, representations = rotate_crossing(
        candidates_sorted,
        edges_is_crossing,
        edges_max_rep,
        edges_min_rep,
        representations,
    )
    check(
        sorted(crossing_edges, key=lambda e: edges_min_rep[e]),
        edges_min_rep,
        edges_max_rep,
        representations,
    )
    return set(candidates_sorted)


def _eliminate_eq_candidates(
    candidates: Iterable[int], distances: np.ndarray, representations: np.ndarray
) -> list[int]:
    """the candidates sorted after their angle representation, only the closest of identical representations"""
    candidates_sorted = sorted(
        candidates, key=lambda i: (representations[i], distances[i])
    )
    closest = candidates_sorted[:1]
    for candidate in candidates_sorted[1:]:
        if representations[candidate] != representations[closest[-1]]:
            closest.append(candidate)
    return closest


def _edge_angle_ranges(
    edges: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """the angle range of the edges seen from the origin

    a single edge blocks less than 180 degree:
    a difference of the representations > 2.0 means the range contains 0.0 ("crossing" edge)

    :return: per edge (indexed by the edge index): the minimum and maximum representation of its vertices,
        the maximum distance of its vertices and whether it is crossing
    """
    nr_edges_total = len(edge_vertex_idxs)
    edges_min_rep = np.zeros(nr_edges_total, dtype=float)
    edges_max_rep = np.zeros(nr_edges_total, dtype=float)
    edges_max_dist = np.zeros(nr_edges_total, dtype=float)
    edges_is_crossing = np.zeros(nr_edges_total, dtype=bool)
    i1 = edge_vertex_idxs[edges, 0]
    i2 = edge_vertex_idxs[edges, 1]
    edges_min_rep[edges] = np.minimum(representations[i1], representations[i2])
    edges_max_rep[edges] = np.maximum(representations[i1], representations[i2])
    edges_max_dist[edges] = np.maximum(distances[i1], distances[i2])
    edges_is_crossing[edges] = edges_max_rep[edges] - edges_min_rep[edges] > 2.0
    return edges_min_rep, edges_max_rep, edges_max_dist, edges_is_crossing


def rotate_crossing(
    candidate_idxs, edges_is_crossing, edges_max_rep, edges_min_rep, representations
):
    if not np.all(edges_min_rep >= 0.0):
        raise ValueError

    # special case: edges cross the origin
    # trick: rotate coordinate system to avoid dealing with origin crossings
    # -> implementation for regular edges can be reused!
    # bring the minimum maximal angle representation ("supremum") of all crossing edges to 0
    supremum_cross_edge_rep = np.min(edges_max_rep[edges_is_crossing])
    infimum_to_0 = 4.0 - supremum_cross_edge_rep
    # Note: adding an angle < 180deg to the minimum edge_idx representations (quadrant 1 & 2) cannot lead to "overflow"
    edges_min_rep[edges_is_crossing] = edges_min_rep[edges_is_crossing] + infimum_to_0
    edges_max_rep[edges_is_crossing] = (
        edges_max_rep[edges_is_crossing] + infimum_to_0
    ) % 4.0
    # Note: all maximum representations were moved to 1. or 2. quadrant
    # -> became the smaller that the previously min rep! -> swap
    tmp = edges_min_rep
    edges_min_rep = edges_max_rep
    edges_max_rep = tmp
    # apply same transformation also to the representations of the candidates (and of all other vertices)
    # IMPORTANT: work on independent copy
    representations = (representations + infimum_to_0) % 4.0
    # Note: new sorting is required
    candidate_idxs = sorted(candidate_idxs, key=lambda i: representations[i])
    return candidate_idxs, edges_max_rep, edges_min_rep, representations


def check_candidates_one_edge(
    edge_min_rep: float,
    edge_max_rep: float,
    edge_max_dist: float,
    p1: int,
    p2: int,
    candidate_ptr: int,
    origin: int,
    candidate_indices: list[int],
    coords: np.ndarray,
    distances: np.ndarray,
    representations: np.ndarray,
    edge_idx: int,
    candidate_edges: dict[int, set[int]],
):
    """removes the candidates blocked by a single edge

    :param candidate_edges: the edges touching a candidate, they do not block it
    :param candidate_ptr: the first candidate which might lie within the angle range of the edge
    :return: the first candidate which might lie within the angle range of the following edges
    """
    # the candidates with a representation smaller than the minimum representation of the edge
    # are not blocked by this edge and the following edges (larger or equal minimum representation)
    for candidate_idx in candidate_indices[candidate_ptr:]:
        if representations[candidate_idx] >= edge_min_rep:
            break
        candidate_ptr += 1

    candidate_ptr_curr = candidate_ptr
    # Note: the candidate list shrinks during the iteration
    while candidate_ptr_curr < len(candidate_indices):
        candidate_idx = candidate_indices[candidate_ptr_curr]
        candidate_rep = representations[candidate_idx]
        if candidate_rep > edge_max_rep:
            # the remaining candidates lie outside of the angle range of the edge
            break
        if candidate_rep == edge_min_rep or candidate_rep == edge_max_rep:
            # the line of sight touches the edge only in one of its vertices (or runs along the edge)
            visibility_is_blocked = False
        elif edge_idx in candidate_edges.get(candidate_idx, ()):
            # the candidate lies on the edge
            visibility_is_blocked = False
        else:
            # optimisation: a candidate farther away than both vertices of the edge surely lies behind it
            # ATTENTION: a candidate closer than both vertices still needs to be checked!
            visibility_is_blocked = distances[
                candidate_idx
            ] > edge_max_dist or _lies_behind(p1, p2, candidate_idx, origin, coords)

        if visibility_is_blocked:
            # Note: keep the pointer at the same position (the list shrank)
            candidate_indices.pop(candidate_ptr_curr)
        else:
            candidate_ptr_curr += 1

    return candidate_ptr


def _check_candidates(
    candidate_idxs: list[int],
    edge_idxs_sorted: list[int],
    edges_max_rep: np.ndarray,
    edges_min_rep: np.ndarray,
    origin: int,
    distances: np.ndarray,
    representations: np.ndarray,
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    edges_max_dist: np.ndarray,
    candidate_edges: dict[int, set[int]],
):
    candidate_ptr = 0
    for edge_idx in edge_idxs_sorted:
        if len(candidate_idxs) == 0:
            # Note: length is decreasing
            break

        edge_min_rep = edges_min_rep[edge_idx]
        if candidate_ptr == len(candidate_idxs) - 1:
            # pointing to the last candidate (w/ highest representation)
            candidate_idx = candidate_idxs[candidate_ptr]
            candidate_rep = representations[candidate_idx]
            if edge_min_rep > candidate_rep:
                # optimisation: the edge has a higher minimum representation that the last candidate
                # all following edges have higher minimum representation and hence can't block the visibility
                # -> visibility computation is finished
                break

        edge_max_rep = edges_max_rep[edge_idx]
        edge_max_dist = edges_max_dist[edge_idx]
        i1, i2 = edge_vertex_idxs[edge_idx]

        candidate_ptr = check_candidates_one_edge(
            edge_min_rep,
            edge_max_rep,
            edge_max_dist,
            i1,
            i2,
            candidate_ptr,
            origin,
            candidate_idxs,
            coords,
            distances,
            representations,
            edge_idx,
            candidate_edges,
        )


def find_visible_from(
    origin: int,
    point: Point | None,
    wedges: list[Wedge],
    candidates: set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
    query_points: dict[int, Point | None] | None = None,
) -> set[int]:
    """finds the candidates visible from the origin, following the rules of the free wedges (cf. ``wedges.py``)

    1. the lines of sight leave the origin within the given free wedges of its point (rule 2)
    2. the polygon edges block the lines of sight (AREA, ``find_visible()``),
       except the edges at the origin (covered by 1.)
    3. the lines of sight pass through points and arrive at contacts within a free wedge (rules 3 and 4)

    :param point: the point at the origin, ``None`` when no polygon passes the origin (all directions are free)
    :param wedges: the free wedges of the point the lines of sight may leave through
    :param candidates: the indices of the coordinates to check.
        Candidates identical to the origin are not visible.
    :param query_points: the points of the query points among the candidates (cf. ``Points.filter_visible()``)
    :return: the visible candidates
    """
    edges = set(range(len(edge_vertex_idxs)))
    if point is not None:
        edges.difference_update(point.incident_edges)
        origin_position = coords[origin]
        candidates = {
            i
            for i in candidates
            if point.in_wedges(representations[i], coords[i] - origin_position, wedges)
        }

    # the edges at a query candidate do not block it (its arrival is decided by rule 4), like at the origin.
    # Note: a query point inside an edge might not lie exactly on it in the floating point computations
    candidate_edges = {
        i: query_point.incident_edges
        for i, query_point in (query_points or {}).items()
        if query_point is not None
    }
    visible = find_visible(
        origin,
        candidates,
        edges,
        coords,
        representations,
        distances,
        edge_vertex_idxs,
        candidate_edges,
    )
    return points.filter_visible(
        origin, visible, coords, representations, distances, query_points
    )


def _find_visible_and_in_front(
    origin: int,
    candidates: set[int],
    candidates_in_front: set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
) -> tuple[set[int], set[int]]:
    """finds the extremities visible from an extremity and the extremities lying "in front" of it

    :param candidates: the extremities to check for visibility
    :param candidates_in_front: the extremities to check whether they lie "in front"
    :return: the extremities lying in front, the visible extremities (not lying in front)
    """
    point = points.by_idx[origin]
    # as shown in [1, Ch. II 4.4.2 "Property One"]: Starting from any point lying "in front of" an extremity e,
    # such that both bounding rays of its wedge are visible, one will never visit e, because everything is
    # reachable on a shorter path without e (except e itself).
    # An extremity e1 lying in the area "in front of" extremity e hence is never the next vertex
    # in the shortest path coming from e.
    # And also in reverse: when coming from e1 everything else than e itself can be reached faster
    # without visiting e.
    # -> e1 and e do not have to be connected in the graph.
    # The area "in front of" e: opposite of the angle range outside of the wedge of e (cf. ``Point.in_front()``),
    # extremities exactly opposite of a bounding ray do not lie in front (those edges must not get deleted)
    # IMPORTANT: this condition only holds for building the basic visibility graph without start and goal node!
    # When a query point (start/goal) happens to be an extremity, edges to the (visible) extremities in front
    # MUST be added to the graph!
    # IMPORTANT: check all extremities here, not just current candidates
    origin_position = coords[origin]
    candidates_in_front = {
        i
        for i in candidates_in_front
        if point.in_front(representations[i], coords[i] - origin_position)
    }
    # do not consider points lying in front when looking for visible extremities,
    # even if they are actually visible.
    candidates = candidates.difference(candidates_in_front)
    visible_idxs = find_visible_from(
        origin,
        point,
        [point.extremity_wedge],
        candidates,
        coords,
        representations,
        distances,
        edge_vertex_idxs,
        points,
    )
    return candidates_in_front, visible_idxs


def get_distance(n1, n2, reprs_n_distances):
    if n2 > n1:
        # Note: start and goal nodex get added last -> highest idx
        # for the lower idxs the distances to goal and start have not been computed
        # -> use the higher indices to access the distances
        tmp = n1
        n1 = n2
        n2 = tmp
    _, dists = reprs_n_distances[n1]
    distance = dists[n2]
    return distance


# adapted from networkx.astar_path() (networkx 3.7), 3-clause BSD license:
#   Copyright (c) 2004-2026, NetworkX Developers
#   Aric Hagberg, Dan Schult, Pieter Swart
#   All rights reserved.
# the conditions and the disclaimer of the license: LICENSES/networkx.txt
def astar_path(
    graph: t.Graph,
    query_edges: dict[int, dict[int, float]],
    source: int,
    target: int,
    heuristic: Callable[[int, int], float],
) -> list[int] | None:
    """the shortest path from source to target with A*, ``None`` if there is no path

    Searches the graph extended by the query edges without modifying (or copying) the graph.
    Same algorithm as ``networkx.astar_path()``.

    :param graph: the weighted graph (edge attribute ``weight``)
    :param query_edges: additional (temporary) edges: node -> neighbour -> weight, in both directions
    :param heuristic: an admissible estimate of the distance between two nodes
    """
    adjacency = graph._adj  # for speed-up (cf. networkx.astar_path())
    no_neighbours: dict = {}

    def neighbours(node: int) -> Iterable[tuple[int, float]]:
        for neighbour, attributes in adjacency.get(node, no_neighbours).items():
            yield neighbour, attributes["weight"]
        yield from query_edges.get(node, no_neighbours).items()

    # the counter breaks ties in the priority (the nodes are never compared)
    c = count()
    queue = [(0.0, next(c), source, 0.0, None)]
    # enqueued node -> (cost to reach, heuristic), explored node -> parent closest to the source
    enqueued: dict[int, tuple[float, float]] = {}
    explored: dict[int, int | None] = {}
    while queue:
        _, _, node, dist, parent = heappop(queue)
        if node == target:
            path = [node]
            node = parent
            while node is not None:
                path.append(node)
                node = explored[node]
            path.reverse()
            return path

        if node in explored:
            # do not override the parent of the source
            if explored[node] is None:
                continue
            # skip paths that were enqueued before a better one was found
            if enqueued[node][0] < dist:
                continue

        explored[node] = parent
        for neighbour, weight in neighbours(node):
            cost = dist + weight
            if neighbour in enqueued:
                enqueued_cost, h = enqueued[neighbour]
                if enqueued_cost <= cost:
                    continue
            else:
                h = heuristic(neighbour, target)
            enqueued[neighbour] = cost, h
            heappush(queue, (cost + h, next(c), neighbour, cost, node))
    return None


def compute_graph(
    extremity_indices: np.ndarray,
    reprs_n_distances: dict[int, np.ndarray],
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    points: Points,
) -> t.Graph:
    """computes the visibility graph of the extremities

    Note: there is at most one extremity per point, no graph nodes have to be merged
    """
    graph = t.Graph()
    # IMPORTANT: add all extremities (even if they turn out to be dangling in the end),
    # adding start and goal nodes at query time might connect them!
    graph.add_nodes_from(extremity_indices)

    # Note: also the last extremity has to be checked (without candidates left):
    # its graph neighbours lying "in front" of it must be disconnected
    for extr_ptr, origin_idx in enumerate(extremity_indices):
        vert_idx2repr, vert_idx2dist = reprs_n_distances[origin_idx]
        # optimisation: extremities are always visible to each other
        # (bidirectional relation -> undirected edges in the graph)
        #  -> do not check extremities which have been checked already
        #  (must give the same result when algorithms are correct)
        # the origin extremity itself must also not be checked when looking for visible neighbours
        candidate_idxs = set(extremity_indices[extr_ptr + 1 :])
        # Note: also the nodes previously connected to the current origin must be considered for removal
        candidates_in_front = candidate_idxs | set(graph.neighbors(origin_idx))
        idxs_in_front, visible_idxs = _find_visible_and_in_front(
            origin_idx,
            candidate_idxs,
            candidates_in_front,
            coords,
            vert_idx2repr,
            vert_idx2dist,
            edge_vertex_idxs,
            points,
        )
        # "thin out" the graph:
        # remove already existing edges in the graph to the extremities in front
        for i in idxs_in_front:
            try:
                graph.remove_edge(origin_idx, i)
            except nx.exception.NetworkXError:
                pass

        for i in visible_idxs:
            graph.add_edge(origin_idx, i, weight=vert_idx2dist[i])

    return graph


def _try_extraction(json_data, key):
    try:
        extracted_data = json_data[key]
    except KeyError as e:
        raise ValueError(f"The expected key {key} was not found in the JSON file:\n{e}")
    return extracted_data


def _convert2polygon(json_list):
    return [tuple(coord_pair_list) for coord_pair_list in json_list]


def read_json(path2json_file):
    """
    Parse data from a JSON file and save as lists of tuples for both boundary and holes.
    NOTE: The format of the JSON file is explained in the command line script (argparse definition)

    :param path2json_file: The path to the input json file
    :return: The parsed lists of boundaries and holes
    """
    # parse data from the input file
    with open(path2json_file) as json_file:
        json_data = json_file.read()
    json_loaded = json.loads(json_data)
    boundary_data = _try_extraction(json_loaded, BOUNDARY_JSON_KEY)
    holes_data = _try_extraction(json_loaded, HOLES_JSON_KEY)
    boundary_coordinates = _convert2polygon(boundary_data)
    list_of_holes = [_convert2polygon(hole_data) for hole_data in holes_data]
    return boundary_coordinates, list_of_holes


def convert_grid(size_x: int, size_y: int, obstacle_iter: iter) -> (list, list):
    """
    prerequisites: the grid must not have non-obstacle cells which are surrounded by obstacles
    ("single white cell in black surrounding" = useless for path planning)
    :param size_x: the horizontal grid size
    :param size_y: the vertical grid size
    :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
    :return: a boundary polygon (counterclockwise numbering) and a list of hole polygons (clockwise numbering)
    NOTE: convert the grid into polygons in a way that coordinates coincide with grid!
        -> no conversion of obtained graphs needed!
        the origin of the polygon coordinate system is (-0.5,-0.5) in the grid cell system (= corners of the grid)
    """

    assert size_x > 0 and size_y > 0

    if len(obstacle_iter) == 0:
        # there are no obstacles. return just the simple boundary rectangle
        boundary = np.array(
            [(0, 0), (size_x, 0), (size_x, size_y), (0, size_y)], dtype=float
        )
        return boundary, []

    # convert (x,y) into np.arrays
    # obstacle_iter = [np.array(o) for o in obstacle_iter]
    obstacle_iter = np.array(obstacle_iter)

    def within_grid(pos):
        return 0 <= pos[0] < size_x and 0 <= pos[1] < size_y

    def is_equal(pos1, pos2):
        return np.all(pos1 == pos2)

    def pos_in_iter(pos, iter):
        for i in iter:
            if is_equal(pos, i):
                return True
        return False

    def is_obstacle(pos):
        return pos_in_iter(pos, obstacle_iter)

    def is_blocked(pos):
        return not within_grid(pos) or is_obstacle(pos)

    def is_unblocked(pos):
        return within_grid(pos) and not is_obstacle(pos)

    def find_start(start_pos, boundary_detect_fct, **kwargs):
        # returns the lowest and leftmost unblocked grid cell from the start position
        # for which the detection function evaluates to True
        start_x, start_y = start_pos
        for y in range(start_y, size_y):
            for x in range(start_x, size_x):
                pos = np.array([x, y])
                if boundary_detect_fct(pos, **kwargs):
                    return pos

    # north, east, south, west
    directions = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]], dtype=int)
    # the correct offset to determine where nodes should be added.
    offsets = np.array([[0, 1], [1, 1], [1, 0], [0, 0]], dtype=int)

    def construct_polygon(start_pos, boundary_detect_fct, cntr_clockwise_wanted: bool):
        current_pos = start_pos.copy()
        # (at least) the west and south are blocked
        #   -> there has to be a polygon node at the current position (bottom left corner of the cell)
        edge_list = [start_pos]
        forward_index = 0  # start with moving north
        forward_vect = directions[forward_index]
        left_index = (forward_index - 1) % 4
        # left_vect = directions[(forward_index - 1) % 4]
        just_turned = True

        # follow the border between obstacles and free cells ("wall") until one
        # reaches the start position again
        while True:
            # left has to be checked first
            # do not check if just turned left or right (-> the left is blocked for sure)
            # left_pos = current_pos + left_vect
            if not (
                just_turned or boundary_detect_fct(current_pos + directions[left_index])
            ):
                # print('< turn left')
                forward_index = left_index
                left_index = (forward_index - 1) % 4
                forward_vect = directions[forward_index]
                just_turned = True

                # add a new node at the correct position
                # decrease the index first!
                edge_list.append(current_pos + offsets[forward_index])

                # move forward (previously left, there is no obstacle)
                current_pos += forward_vect
            else:
                forward_pos = current_pos + forward_vect
                if boundary_detect_fct(forward_pos):
                    node_pos = current_pos + offsets[forward_index]
                    # there is a node in the bottom left corner of the start position (offset= (0,0) )
                    if is_equal(node_pos, start_pos):
                        # check and terminate if this node does already exist
                        break

                    # add a new node at the correct position
                    edge_list.append(node_pos)
                    # print('> turn right')
                    left_index = forward_index
                    forward_index = (forward_index + 1) % 4
                    forward_vect = directions[forward_index]
                    just_turned = True
                    # print(direction_index,forward_vect,just_turned,edge_list,)
                else:
                    # print('^ move forward')
                    current_pos += forward_vect
                    just_turned = False

        if cntr_clockwise_wanted:
            # make edge numbering counterclockwise!
            edge_list.reverse()
        return np.array(edge_list, dtype=float)

    # build the boundary polygon
    # start at the lowest and leftmost unblocked grid cell
    start_pos = find_start(start_pos=(0, 0), boundary_detect_fct=is_unblocked)
    # print(start_pos+directions[3])
    # raise ValueError
    boundary_edges = construct_polygon(
        start_pos, boundary_detect_fct=is_blocked, cntr_clockwise_wanted=True
    )

    # detect which of the obstacles have to be converted into holes
    # just the obstacles inside the boundary polygon are part of holes
    # shift coordinates by +(0.5,0.5) for correct detection
    # the border value does not matter here

    def get_unchecked_obstacles(
        obstacles: Iterable, poly: np.ndarray, required_val: bool = True
    ) -> list:
        unchecked_obstacles = []
        for o in obstacles:
            p = o + 0.5
            if _inside_polygon(p, poly, border_value=True) == required_val:
                unchecked_obstacles.append(o)

        return unchecked_obstacles

    unchecked_obstacles = get_unchecked_obstacles(obstacle_iter, boundary_edges)

    hole_list = []
    while len(unchecked_obstacles) > 0:
        start_pos = find_start(
            start_pos=(0, 0), boundary_detect_fct=pos_in_iter, iter=unchecked_obstacles
        )
        hole = construct_polygon(
            start_pos, boundary_detect_fct=is_unblocked, cntr_clockwise_wanted=False
        )

        # detect which of the obstacles still do not belong to any hole:
        # delete the obstacles which are included in the just constructed hole
        unchecked_obstacles = get_unchecked_obstacles(
            unchecked_obstacles, hole, required_val=False
        )

        hole_list.append(hole)

    return boundary_edges, hole_list


def _cmp_edge_and_vertex_idxs(list_of_polygons):
    # compute edge and vertex indices from polygon data structure
    edge_and_vertex_indices = [
        edge_vertex_mappings(len(coords_poly)) for coords_poly in list_of_polygons
    ]
    offset = 0
    for edge_vertex_idxs, vertex_edge_idxs in edge_and_vertex_indices:
        edge_vertex_idxs += offset
        vertex_edge_idxs += offset
        offset += len(edge_vertex_idxs)
    edge_vertex_idxs, vertex_edge_idxs = zip(*edge_and_vertex_indices)
    edge_vertex_idxs = np.concatenate(edge_vertex_idxs, axis=0)
    vertex_edge_idxs = np.concatenate(vertex_edge_idxs, axis=0)
    return edge_vertex_idxs, vertex_edge_idxs


def compile_polygon_datastructs(
    boundary_coordinates: np.ndarray | None,
    list_of_hole_coordinates: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, Points]:
    """compiles the data structures of the polygons

    :return: the coordinates of all vertices (boundary first, then all holes),
        the edge -> vertex and vertex -> edge index mappings
        and the points of the environment (the vertex positions within the map, cf. ``wedges.py``)
    """
    list_of_polygons = list_of_hole_coordinates
    if boundary_coordinates is not None:
        list_of_polygons = [boundary_coordinates] + list_of_polygons
    coords = np.concatenate(list_of_polygons, axis=0, dtype=configs.DTYPE_FLOAT)
    edge_vertex_idxs, vertex_edge_idxs = _cmp_edge_and_vertex_idxs(list_of_polygons)
    vertex_polygon_idxs = np.repeat(
        np.arange(len(list_of_polygons)), [len(p) for p in list_of_polygons]
    )
    points = Points(
        coords,
        edge_vertex_idxs,
        vertex_edge_idxs,
        vertex_polygon_idxs,
        None if boundary_coordinates is None else 0,
        lambda p: is_within_map(p, boundary_coordinates, list_of_hole_coordinates),
    )
    return coords, edge_vertex_idxs, vertex_edge_idxs, points


def load_pickle(path=DEFAULT_PICKLE_NAME):
    """loads an environment exported with :meth:`PolygonEnvironment.export_pickle`

    :raises ValueError: when the environment was pickled with an incompatible version of this package (e.g. 2.x).
        Recreate the environment from its polygons instead.
    :raises AttributeError: when the pickled class no longer exists (e.g. ``PlottingEnvironment`` of 2.x)
    """
    print("loading map from:", path)
    with open(path, "rb") as f:
        return pickle.load(f)
