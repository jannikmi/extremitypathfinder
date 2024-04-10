import itertools
import json
import pickle
from itertools import combinations
from typing import Dict, Iterable, List, Set, Tuple

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
    _angle_rep_inverse,
    _compute_repr_n_dist,
    _fill_edge_vertex_idxs,
    _fill_extremity_mask,
    _has_clockwise_numbering,
    _inside_polygon,
    _lies_behind,
    _no_identical_consequent_vertices,
)


def cmp_reps_n_distances(orig_idx: int, coords: np.ndarray) -> np.ndarray:
    coords_orig = coords[orig_idx]
    coords_translated = coords - coords_orig
    repr_n_dists = np.apply_along_axis(
        _compute_repr_n_dist, axis=1, arr=coords_translated
    )
    return repr_n_dists.T


def cmp_reps_n_distance_dict(
    coords: np.ndarray, extremity_indices: np.ndarray
) -> Dict[int, np.ndarray]:
    # Note: distance and angle representation relation are symmetric,
    # but exploiting this has been found to be slower than using the numpy functionality with slight overhead
    reps_n_distance_dict = {
        i: cmp_reps_n_distances(i, coords) for i in extremity_indices
    }
    return reps_n_distance_dict


def is_within_map(
    p: np.ndarray, boundary: np.ndarray, holes: Iterable[np.ndarray]
) -> bool:
    if not _inside_polygon(p, boundary, border_value=True):
        return False
    for hole in holes:
        if _inside_polygon(p, hole, border_value=False):
            return False
    return True


def _get_intersection_status(p1, p2, q1, q2):
    # return:
    #   0: no intersection
    #   1: intersection in ]p1;p2[
    # TODO support 2 remaining possibilities
    #   2: intersection directly in p1 or p2
    #   3: intersection directly in q1 or q2
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
    # again_check = []
    for index_p1, index_q1 in combinations(range(polygon_length), 2):
        # always: index_p1 < index_q1
        if index_p1 == index_q1 - 1 or index_p1 == index_q1 + 1:
            # neighbouring edges never have an intersection
            continue
        p1, p2 = coords[index_p1], coords[(index_p1 + 1) % polygon_length]
        q1, q2 = coords[index_q1], coords[(index_q1 + 1) % polygon_length]
        intersect_status = _get_intersection_status(p1, p2, q1, q2)
        if intersect_status == 1:
            return False
        # if intersect_status == 2:
        # TODO 4 different options. check which side the other edge lies on.
        # if edge changes sides this is a an intersection
        # again_check.append((p1, p2, q1, q2))
        # print(p1, p2, q1, q2)

    # TODO check for intersections across 2 edges! use computed intersection

    return True


def _check_polygon(polygon):
    """ensures that all the following conditions on the polygons are fulfilled:
    - must at least contain 3 vertices
    - no consequent vertices with identical coordinates in the polygons! In general might have the same coordinates
    - a polygon must not have self intersections (intersections with other polygons are allowed)
    """
    if not polygon.shape[0] >= 3:
        raise TypeError("Given polygons must at least contain 3 vertices.")
    if not polygon.shape[1] == 2:
        raise TypeError("Each point of a polygon must consist of two values (x,y).")
    if not _no_identical_consequent_vertices(polygon):
        raise ValueError("Consequent vertices of a polynomial must not be identical.")
    if not _no_self_intersection(polygon):
        raise ValueError("The given polygon has self intersections")


def check_data_requirements(
    boundary_coords: np.ndarray, list_hole_coords: List[np.ndarray]
):
    """ensures that all the following conditions on the polygons are fulfilled:
        - basic polygon requirements (s. above)
        - edge numbering has to follow this convention (for easier computations):
            * outer boundary polygon: counter clockwise
            * holes: clockwise

    :param boundary_coords:
    :param list_hole_coords:
    :return:
    """
    _check_polygon(boundary_coords)
    if _has_clockwise_numbering(boundary_coords):
        raise ValueError(
            "Vertex numbering of the boundary polygon must be counter clockwise."
        )
    for hole_coords in list_hole_coords:
        _check_polygon(hole_coords)
        if not _has_clockwise_numbering(hole_coords):
            raise ValueError("Vertex numbering of hole polygon must be clockwise.")


def _find_within_range(
    repr1: float,
    repr2: float,
    candidate_idxs: Set[int],
    angle_range_less_180: bool,
    equal_repr_allowed: bool,
    representations: np.ndarray,
) -> Set[int]:
    """
    filters out all vertices whose representation lies within the range between
      the two given angle representations
    which range ('clockwise' or 'counter-clockwise') should be checked is determined by:
      - query angle (range) is < 180deg or not (>= 180deg)
    :param repr1:
    :param repr2:
    :param candidate_idxs:
    :param angle_range_less_180: whether the angle between repr1 and repr2 is < 180 deg
    :param equal_repr_allowed: whether vertices with the same representation should also be returned
    :param representations:
    :return:
    """
    if len(candidate_idxs) == 0:
        return set()

    repr_diff = abs(repr1 - repr2)
    if repr_diff == 0.0:
        return set()

    min_repr = min(repr1, repr2)
    max_repr = max(repr1, repr2)  # = min_angle + angle_diff

    def repr_within(r):
        # Note: vertices with the same representation will NOT be returned!
        return min_repr < r < max_repr

    # depending on the angle the included range is clockwise or anti-clockwise
    # (from min_repr to max_val or the other way around)
    # when the range contains the 0.0 value (transition from 3.99... -> 0.0)
    # it is easier to check if a representation does NOT lie within this range
    # -> invert filter condition
    # special case: angle == 180deg
    on_line_inv = repr_diff == 2.0 and repr1 >= repr2
    # which range to filter is determined by the order of the points
    # since the polygons follow a numbering convention,
    # the 'left' side of p1-p2 always lies inside the map
    # -> filter out everything on the right side (='outside')
    # ^: XOR
    inversion_condition = on_line_inv or ((repr_diff < 2.0) ^ angle_range_less_180)

    def within_filter_func(r: float) -> bool:
        repr_eq = r == min_repr or r == max_repr
        if repr_eq and equal_repr_allowed:
            return True
        if repr_eq and not equal_repr_allowed:
            return False

        res = repr_within(r)
        if inversion_condition:
            res = not res
        return res

    idxs_within = {i for i in candidate_idxs if within_filter_func(representations[i])}
    return idxs_within


def get_neighbour_idxs(
    i: int, vertex_edge_idxs: np.ndarray, edge_vertex_idxs: np.ndarray
) -> Tuple[int, int]:
    edge_idx1, edge_idx2 = vertex_edge_idxs[i]
    neigh_idx1 = edge_vertex_idxs[edge_idx1, 0]
    neigh_idx2 = edge_vertex_idxs[edge_idx2, 1]
    return neigh_idx1, neigh_idx2


def find_visible(
    origin: int,
    candidates: Set[int],
    edges_to_check: Set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    extremity_mask: np.ndarray,
) -> Set[int]:
    """
    TODO
    for all origin extremities
    precompute all required ranges etc for all edges
    sort all edges after their minimum representation
    also sort the candidate extremities after their angle representation
    for every edge_idx
        if the minimum representation of the edge_idx is smaller than the repr of the candidate:
        check the next candidate, move start candidate pointer along
        if the maximum representation of the edge_idx is bigger than  the repr of the candidate:
        check the next edge_idx, start at the start candidate index again
        check if the edge_idx is blocking the visibility of the node: if yes delete from candidates
        check the next node

        optimisation: if flag, invert edge_idx representations and also eliminate
        all candidates within range (without lies behind check!)


    :param origin: the vertex for which the visibility to the other candidates should be checked.
    :param candidates: the set of all vertex ids which should be checked for visibility.
        IMPORTANT: is being manipulated, so has to be a copy!
        IMPORTANT: must not contain any vertices with equal coordinates (e.g. the origin vertex itself)!
    :param edges_to_check: the set of edges which determine visibility
    :return: a set of all vertices visible from the origin
    """
    nr_candidates_total = len(candidates)
    if nr_candidates_total == 0:
        return candidates
    # TODO immutable
    # eliminate candidates with equal representations: only keep the closest (min dist)
    candidates_sorted = _eliminate_eq_candidates(candidates, distances, representations)

    (
        crossing_edges,
        edges_is_crossing,
        edges_max_dist,
        edges_max_rep,
        edges_min_rep,
        non_crossing_edges,
        edge_vertex_idxs,
    ) = _compile_visibility_datastructs(
        distances,
        edge_vertex_idxs,
        edges_to_check,
        extremity_mask,
        representations,
        vertex_edge_idxs,
    )

    # check non-crossing edges
    # TODO skipped edges. argsort
    # sort after the minimum representation
    edge_idxs_sorted = sorted(non_crossing_edges, key=lambda e: edges_min_rep[e])
    # edge_ptr_iter = iter(ptr for ptr in edge_ptrs if not edges_is_crossing[ptr])
    _check_candidates(
        candidates_sorted,
        edge_idxs_sorted,
        edges_max_rep,
        edges_min_rep,
        origin,
        distances,
        representations,
        coords,
        edge_vertex_idxs,
        edges_max_dist,
    )

    # when there are no origin-crossing edges, we are done
    if len(crossing_edges) == 0:
        return set(candidates_sorted)

    # check origin-crossing edges
    candidates_sorted, edges_max_rep, edges_min_rep, representations = rotate_crossing(
        candidates_sorted,
        edges_is_crossing,
        edges_max_rep,
        edges_min_rep,
        representations,
    )

    # start with checking the first candidate again
    edge_idxs_sorted = sorted(crossing_edges, key=lambda e: edges_min_rep[e])
    _check_candidates(
        candidates_sorted,
        edge_idxs_sorted,
        edges_max_rep,
        edges_min_rep,
        origin,
        distances,
        representations,
        coords,
        edge_vertex_idxs,
        edges_max_dist,
    )
    # TODO avoid conversion?
    candidates_ = set(candidates_sorted)
    return candidates_


def _eliminate_eq_candidates(
    candidates: List[int], distances: np.ndarray, representations: np.ndarray
) -> List[int]:
    # sort after angle representation, then after distance ascending
    candidates_sorted = sorted(
        candidates, key=lambda i: (representations[i], distances[i])
    )
    rep_prev = representations[candidates_sorted[0]]
    i = 1
    while i < len(candidates_sorted):
        candidate = candidates_sorted[i]
        rep = representations[candidate]
        if np.isnan(rep) or rep == rep_prev:
            # the candidate is equal to the origin OR
            # two candidates have equal angle representation
            # only keep the closest (=1st)
            candidates_sorted.pop(i)
        else:
            rep_prev = rep
            i += 1

    return candidates_sorted


def _compile_visibility_datastructs(
    distances,
    edge_vertex_idxs,
    edges_to_check,
    extremity_mask,
    representations,
    vertex_edge_idxs,
):
    edges = list(edges_to_check)
    nr_edges_total = len(edge_vertex_idxs)
    edges_min_rep = np.zeros(nr_edges_total, dtype=float)
    edges_max_rep = np.zeros(nr_edges_total, dtype=float)
    edges_max_dist = np.zeros(nr_edges_total, dtype=float)
    edges_is_crossing = np.zeros(nr_edges_total, dtype=bool)
    edges_to_skip = set()
    crossing_edges = set()
    non_crossing_edges = set()
    copied_indices = False
    for e in edges:
        if e in edges_to_skip:
            # Note: edges will be skipped, because they do not appear the sets of crossing or non-crossing edges
            continue

        # Attention: introduces new indexing! beware of confusion
        i1, i2 = edge_vertex_idxs[e]
        r1, r2 = representations[i1], representations[i2]

        identical_node = None
        if np.isnan(r1):
            identical_node = i1
        elif np.isnan(r2):
            identical_node = i2

        if identical_node is not None:
            # one of the edge vertices is identical to origin
            # no points lie truly "behind" this edge as there is no "direction of sight" defined
            # <-> angle representation/range undefined for just this single edge
            # however if one considers the point neighbouring in the other direction (<-> two edges)
            # these two neighbouring edges define an invisible angle range
            # -> simply move the pointer
            i1, i2 = get_neighbour_idxs(
                identical_node, vertex_edge_idxs, edge_vertex_idxs
            )
            r1, r2 = representations[i1], representations[i2]
            min_rep = min(r1, r2)
            max_rep = max(r1, r2)

            # Note: the second edge should not be considered twice
            e1, e2 = vertex_edge_idxs[identical_node]
            if e1 != e:
                edges_to_skip.add(e1)
            if e2 != e:
                edges_to_skip.add(e2)
            # TODO remove
            if e1 != e and e2 != e:
                raise ValueError()

            if not copied_indices:
                edge_vertex_idxs = edge_vertex_idxs.copy()
                copied_indices = True

            # point to the neighbouring vertices (used for looking up the coordinates)
            edge_vertex_idxs[e] = (i1, i2)

            # the "outside the polygon" angle range should be eliminated
            # this angle range is greater than 180 degree if the node identical to the origin is NOT an extremity
            deg_gr_180_exp = not extremity_mask[identical_node]
            deg_gr_180_actual = max_rep - min_rep > 2.0
            # an edge "crosses the origin" (rep: 4.0 -> 0.0)
            # when its vertex representation difference is unlike expected
            is_crossing = deg_gr_180_actual != deg_gr_180_exp

            # set distance to 0 in order to mark all candidates within range as "lying behind"
            max_dist = 0.0

            # mark the identical vertex as not visible (would otherwise add 0 distance edge in the graph)
            # # TODO needs to be added and will be combined later?! check in separate function?!
            # if i1 != origin:
            #     candidates.discard(i1)
        else:
            min_rep = min(r1, r2)
            max_rep = max(r1, r2)
            rep_diff = max_rep - min_rep
            # special case: angle == 180deg <-> lies on the line
            on_the_edge = rep_diff == 2.0
            if on_the_edge:
                # "edge case": origin lies on the edge
                # the edge blocks the visibility to the "outside the polygon"
                # depending on the vertex numbering, the outside angle range crosses the origin or not
                # (from min_repr to max_val or the other way around)
                # when the range contains the 0.0 value (transition from 3.99... -> 0.0)
                # it is easier to check if a representation does NOT lie within this range
                # -> invert filter condition
                is_crossing = r1 > r2
                # TODO edge case one of the reps is 0?!
                # set distance to 0 in order to mark all candidates within range as "lying behind"
                max_dist = 0.0
            else:
                # regular edge
                # a single edge_idx can block at most 180 degree
                is_crossing = rep_diff > 2.0
                max_dist = max(distances[i1], distances[i2])

        if is_crossing:
            crossing_edges.add(e)
        else:
            non_crossing_edges.add(e)

        edges_min_rep[e] = min_rep
        edges_max_rep[e] = max_rep
        edges_max_dist[e] = max_dist
        edges_is_crossing[e] = is_crossing

    return (
        crossing_edges,
        edges_is_crossing,
        edges_max_dist,
        edges_max_rep,
        edges_min_rep,
        non_crossing_edges,
        edge_vertex_idxs,
    )


def rotate_crossing(
    candidate_idxs, edges_is_crossing, edges_max_rep, edges_min_rep, representations
):
    if not np.all(edges_min_rep >= 0.0):
        raise ValueError

    # TODO refactor
    if not np.any(edges_is_crossing):
        return set(candidate_idxs)

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
    # apply same transformation also to candidate representations
    # TODO avoid large copy. use dict
    representations = representations.copy()  # IMPORTANT: work on independent copy
    representations[candidate_idxs] = (
        representations[candidate_idxs] + infimum_to_0
    ) % 4.0
    # Note: new sorting is required
    candidate_idxs = sorted(candidate_idxs, key=lambda i: representations[i])
    # TODO move to tests
    # non_nan_reps = representations[np.logical_not(np.isnan(representations))]
    # assert np.all(non_nan_reps >= 0.0)
    # assert np.all(non_nan_reps <= 4.0)
    # if not np.all(edges_min_rep >= 0.0):
    #     raise ValueError
    # assert np.all(edges_min_rep <= 4.0)
    # assert np.min(edges_min_rep[edges_is_crossing]) == 0.0
    # assert np.all(edges_max_rep >= 0.0)
    # assert np.all(edges_max_rep <= 4.0)
    # for r_min, r_max in zip(edges_min_rep[edges_is_crossing], edges_max_rep[edges_is_crossing]):
    #     if r_min > r_max:
    #         raise ValueError
    return candidate_idxs, edges_max_rep, edges_min_rep, representations


def check_candidates_one_edge(
    edge_min_rep: float,
    edge_max_rep: float,
    edge_max_dist: float,
    p1: int,
    p2: int,
    candidate_ptr: int,
    origin: int,
    candidate_indices: List[int],
    coords: np.ndarray,
    distances: np.ndarray,
    representations: np.ndarray,
):
    # start over at the same candidate than the previous edge
    # TODO Note: check within range: edge case, same representation than edge vertex.
    #  do not include (edge does not block view)
    for candidate_idx in candidate_indices[candidate_ptr:]:
        candidate_rep = representations[candidate_idx]
        # a candidate does not have to be considered,
        # when its representation is smaller or equal than the minimum representation of the edge
        if candidate_rep >= edge_min_rep:
            # candidate has to be considered
            break
        # this also is the case for all consequent edges (have a larger or equal minimum representation!)
        candidate_ptr += 1

    # start at the start candidate index again
    candidate_ptr_curr = candidate_ptr
    while 1:
        # Note: candidate list shrinks during the iteration -> avoid index error
        try:
            candidate_idx = candidate_indices[candidate_ptr_curr]
        except IndexError:
            break

        if candidate_idx == p1 or candidate_idx == p2:
            # an edge cannot block its own vertices
            # move pointer to the next candidate
            candidate_ptr_curr += 1
            continue

        candidate_rep = representations[candidate_idx]

        if edge_max_rep < candidate_rep:
            # the maximum representation of the edge is smaller than the repr of the candidate,
            # -> check the next edge
            break

        # candidate representation lies between edge_min_rep and edge_max_rep
        # assert edge_min_rep <= candidate_rep <= edge_max_rep

        # check if the edge is blocking the visibility of the node: if yes delete from candidates
        dist_to_candidate = distances[candidate_idx]
        equal_reps = edge_min_rep == candidate_rep or edge_max_rep == candidate_rep
        if equal_reps:
            # edge of the candidate itself should not block visibility
            # distance must be truly larger to block visibility.
            visibility_is_blocked = dist_to_candidate > edge_max_dist
        else:
            # optimisation: if a candidate is farther away from the query point than both vertices of the edge,
            #   it surely lies behind the edge
            # ATTENTION: even if a candidate is closer to the query point than both vertices of the edge,
            #   it still needs to be checked!
            further_away = dist_to_candidate > edge_max_dist
            visibility_is_blocked = further_away or _lies_behind(
                p1, p2, candidate_idx, origin, coords
            )

        if visibility_is_blocked:
            candidate_indices.pop(candidate_ptr_curr)
            # Note: keep ptr at the same position (list shrank)
        else:
            # move pointer to the next candidate
            candidate_ptr_curr += 1

    return candidate_ptr


def _check_candidates(
    candidate_idxs: List[int],
    edge_idxs_sorted: List[int],
    edges_max_rep: np.ndarray,
    edges_min_rep: np.ndarray,
    origin: int,
    distances: np.ndarray,
    representations: np.ndarray,
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    edges_max_dist: np.ndarray,
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
        )


def _find_visible_and_in_front(
    origin: int,
    nr_edges: int,
    coords: np.ndarray,
    candidates: Set[int],
    candidates_in_front: Set[int],
    extremity_mask: np.ndarray,
    distances: np.ndarray,
    representations: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    edge_vertex_idxs: np.ndarray,
):
    # vertices all belong to a polygon
    n1, n2 = get_neighbour_idxs(origin, vertex_edge_idxs, edge_vertex_idxs)
    n1_repr = representations[n1]
    n2_repr = representations[n2]
    # as shown in [1, Ch. II 4.4.2 "Property One"]: Starting from any point lying "in front of" an extremity e,
    # such that both adjacent edges are visible, one will never visit e, because everything is
    # reachable on a shorter path without e (except e itself).
    # An extremity e1 lying in the area "in front of" extremity e hence is never the next vertex
    # in the shortest path coming from e.
    # And also in reverse: when coming from e1 everything else than e itself can be reached faster
    # without visiting e.
    # -> e1 and e do not have to be connected in the graph.
    # IMPORTANT: this condition only holds for building the basic visibility graph without start and goal node!
    # When a query point (start/goal) happens to be an extremity, edges to the (visible) extremities in front
    # MUST be added to the graph!
    # Find extremities which fulfill this condition for the given query extremity
    # IMPORTANT: check all extremities here, not just current candidates
    # do not check extremities with equal coords_rel (also query extremity itself!)
    #   and with the same angle representation (those edges must not get deleted from graph!)
    candidates_in_front = _find_within_range(
        repr1=_angle_rep_inverse(n1_repr),
        repr2=_angle_rep_inverse(n2_repr),
        candidate_idxs=candidates_in_front,
        angle_range_less_180=True,
        equal_repr_allowed=False,
        representations=representations,
    )
    # do not consider points lying in front when looking for visible extremities,
    # even if they are actually visible.
    candidates.difference_update(candidates_in_front)
    # ATTENTION: polygons may intersect -> neighbouring extremities must NOT be visible from each other!
    # eliminate all vertices 'behind' the query point from the candidate set
    # since the query vertex is an extremity the 'outer' angle is < 180 degree
    # then the difference between the angle representation of the two edges has to be < 2.0
    # all vertices between the angle of the two neighbouring edges ('outer side')
    #   are not visible (no candidates!)
    # ATTENTION: vertices with the same angle representation might be visible and must NOT be deleted!
    idxs_behind = _find_within_range(
        n1_repr,
        n2_repr,
        candidates,
        angle_range_less_180=True,
        equal_repr_allowed=False,
        representations=representations,
    )
    # do not consider points found to lie behind
    candidates.difference_update(idxs_behind)

    # all edges have to be checked, except the 2 neighbouring edges (handled above!)
    # TODO edge set to ignore instead
    edge_idxs2check = set(range(nr_edges))
    edge_idxs2check.difference_update(vertex_edge_idxs[origin])
    visible_idxs = find_visible(
        origin,
        candidates,
        edge_idxs2check,
        coords,
        representations,
        distances,
        edge_vertex_idxs,
        vertex_edge_idxs,
        extremity_mask,
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


def _find_identical(
    candidates: Iterable[int], reprs_n_distances: Dict[int, np.ndarray]
) -> Dict[int, int]:
    # for shortest path computations all graph nodes should be unique
    # join all nodes with the same coordinates
    merging_mapping = {}
    # symmetric relation -> only consider one direction
    for n1, n2 in itertools.combinations(candidates, 2):
        dist = get_distance(n1, n2, reprs_n_distances)
        if dist == 0.0:  # same coordinates
            merging_mapping[n2] = n1

    return merging_mapping


def find_identical_single(
    i: int, candidates: Iterable[int], reprs_n_distances: Dict[int, np.ndarray]
) -> int:
    # for shortest path computations all graph nodes should be unique
    # join all nodes with the same coordinates
    # symmetric relation -> only consider one direction
    for n in candidates:
        if i == n:
            continue
        dist = get_distance(i, n, reprs_n_distances)
        if dist == 0.0:  # same coordinates
            return n
    return i


def compute_graph(
    nr_edges: int,
    extremity_indices: Iterable[int],
    reprs_n_distances: Dict[int, np.ndarray],
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    extremity_mask: np.ndarray,
    vertex_edge_idxs: np.ndarray,
) -> t.Graph:
    graph = t.Graph()
    # IMPORTANT: add all extremities (even if they turn out to be dangling in the end),
    # adding start and goal nodes at query time might connect them!
    graph.add_nodes_from(extremity_indices)

    # optimisation: no not check the last extremity as no other candidates will remain (cf. below)
    for extr_ptr, origin_idx in enumerate(extremity_indices[:-1]):
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
            nr_edges,
            coords,
            candidate_idxs,
            candidates_in_front,
            extremity_mask,
            vert_idx2dist,
            vert_idx2repr,
            vertex_edge_idxs,
            edge_vertex_idxs,
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

    merge_mapping = _find_identical(graph.nodes, reprs_n_distances)
    if len(merge_mapping) > 0:
        nx.relabel_nodes(graph, merge_mapping, copy=False)

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


def convert_gridworld(
    size_x: int, size_y: int, obstacle_iter: iter, simplify: bool = True
) -> (list, list):
    """
    prerequisites: grid world must not have non-obstacle cells which are surrounded by obstacles
    ("single white cell in black surrounding" = useless for path planning)
    :param size_x: the horizontal grid world size
    :param size_y: the vertical grid world size
    :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
    :param simplify: whether the polygons should be simplified or not. reduces edge amount, allow diagonal edges
    :return: a boundary polygon (counterclockwise numbering) and a list of hole polygons (clockwise numbering)
    NOTE: convert grid world into polygons in a way that coordinates coincide with grid!
        -> no conversion of obtained graphs needed!
        the origin of the polygon coordinate system is (-0.5,-0.5) in the grid cell system (= corners of the grid world)
    """

    assert size_x > 0 and size_y > 0

    if len(obstacle_iter) == 0:
        # there are no obstacles. return just the simple boundary rectangle
        return [
            np.array(x, y)
            for x, y in [(0, 0), (size_x, 0), (size_x, size_y), (0, size_y)]
        ], []

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

    if simplify:
        # TODO
        raise NotImplementedError()

    # detect which of the obstacles have to be converted into holes
    # just the obstacles inside the boundary polygon are part of holes
    # shift coordinates by +(0.5,0.5) for correct detection
    # the border value does not matter here

    def get_unchecked_obstacles(
        obstacles: Iterable, poly: np.ndarray, required_val: bool = True
    ) -> List:
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

        if simplify:
            # TODO
            pass

        hole_list.append(hole)

    return boundary_edges, hole_list


def _cmp_extremity_mask(coordinates: np.ndarray) -> np.ndarray:
    """identify all protruding points = vertices with an inside angle of > 180 degree ('extremities')
    expected edge numbering:
        outer boundary polygon: counterclockwise
        holes: clockwise

    basic idea:
      - translate the coordinate system to have p2 as origin
      - compute the angle representations of both vectors representing the edges
      - "rotate" the coordinate system (equal to deducting) so that the p1p2 representation is 0
      - check in which quadrant the p2p3 representation lies
    %4 because the quadrant has to be in [0,1,2,3] (representation in [0:4[)
    if the representation lies within quadrant 0 or 1 (<2.0), the inside angle
      (for boundary polygon inside, for holes outside) between p1p2p3 is > 180 degree
    then p2 = extremity
    :param coordinates:
    :return:
    """
    extremity_mask = np.full(len(coordinates), False, dtype=bool)
    _fill_extremity_mask(coordinates, extremity_mask)
    return extremity_mask


def _cmp_extremities(
    list_of_polygons, coords, boundary_coordinates, list_of_hole_coordinates
):
    extremity_masks = [
        _cmp_extremity_mask(coords_poly) for coords_poly in list_of_polygons
    ]
    extremity_mask = np.concatenate(extremity_masks, axis=0, dtype=bool)
    # Attention: since polygons are allowed to overlap, only consider extremities that are actually within the map
    for extremity_idx in np.where(extremity_mask)[0]:
        extrimity_coords = coords[extremity_idx]
        if not is_within_map(
            extrimity_coords, boundary_coordinates, list_of_hole_coordinates
        ):
            extremity_mask[extremity_idx] = False
    extremity_indices = np.where(extremity_mask)[0]
    return extremity_indices, extremity_mask


def _cmp_edge_vertex_idxs(coordinates: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    nr_coords = len(coordinates)
    edge_vertex_idxs = np.empty((nr_coords, 2), dtype=int)
    vertex_edge_idxs = np.empty((nr_coords, 2), dtype=int)
    _fill_edge_vertex_idxs(edge_vertex_idxs, vertex_edge_idxs)
    return edge_vertex_idxs, vertex_edge_idxs


def _cmp_edge_and_vertex_idxs(list_of_polygons):
    # compute edge and vertex indices from polygon data structure
    edge_and_vertex_indices = [
        _cmp_edge_vertex_idxs(coords_poly) for coords_poly in list_of_polygons
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
    boundary_coordinates: np.ndarray, list_of_hole_coordinates: List[np.ndarray]
):
    list_of_polygons = [boundary_coordinates] + list_of_hole_coordinates
    coords = np.concatenate(list_of_polygons, axis=0, dtype=configs.DTYPE_FLOAT)
    edge_vertex_idxs, vertex_edge_idxs = _cmp_edge_and_vertex_idxs(list_of_polygons)
    extremity_indices, extremity_mask = _cmp_extremities(
        list_of_polygons, coords, boundary_coordinates, list_of_hole_coordinates
    )
    return coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs


def load_pickle(path=DEFAULT_PICKLE_NAME):
    print("loading map from:", path)
    with open(path, "rb") as f:
        return pickle.load(f)
