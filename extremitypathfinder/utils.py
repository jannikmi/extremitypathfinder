import itertools
import json
import math
import pickle
from itertools import combinations
from typing import Dict, Iterable, List, Optional, Set, Tuple

import networkx as nx
import numpy as np
import numpy.linalg

from extremitypathfinder.configs import BOUNDARY_JSON_KEY, DEFAULT_PICKLE_NAME, HOLES_JSON_KEY


def compute_repr_n_dist(np_vector: np.ndarray) -> Tuple[float, float]:
    """computing representation for the angle from the origin to a given vector

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
    but it its bijective and monotonous:
    rep(p1) > rep(p2) <=> angle(p1) > angle(p2)
    rep(p1) = rep(p2) <=> angle(p1) = angle(p2)
    angle(p): counter clockwise angle between the two line segments (0,0)'--(1,0)' and (0,0)'--p
    with (0,0)' being the vector representing the origin

    :param np_vector:
    :return:
    """
    dx, dy = np_vector
    distance = math.sqrt(dx**2 + dy**2)  # l-2 norm
    if distance == 0.0:
        angle_measure = np.nan
    else:
        # 2D vector: (dx, dy) = np_vector
        dx_positive = dx >= 0
        dy_positive = dy >= 0

        if dx_positive and dy_positive:
            quadrant = 0.0
            angle_measure = dy

        elif not dx_positive and dy_positive:
            quadrant = 1.0
            angle_measure = -dx

        elif not dx_positive and not dy_positive:
            quadrant = 2.0
            angle_measure = -dy

        else:
            quadrant = 3.0
            angle_measure = dx

        # normalise angle measure to [0; 1]
        angle_measure /= distance
        angle_measure += quadrant

    return angle_measure, distance


def cmp_reps_n_distances(orig_idx: int, coords: np.ndarray) -> np.ndarray:
    coords_orig = coords[orig_idx]
    coords_translated = coords - coords_orig
    repr_n_dists = np.apply_along_axis(compute_repr_n_dist, axis=1, arr=coords_translated)
    return repr_n_dists.T


def inside_polygon(p: np.ndarray, coords: np.ndarray, border_value: bool) -> bool:
    # should return the border value for point equal to any polygon vertex
    # TODO overflow possible with large values when comparing slopes, change procedure
    # and if the point p lies on any polygon edge
    p1 = coords[-1, :]
    for p2 in coords[:]:
        if np.array_equal(p2, p):
            return border_value
        rep_p1_p, _ = compute_repr_n_dist(p1 - p)
        rep_p2_p, _ = compute_repr_n_dist(p2 - p)
        if abs(rep_p1_p - rep_p2_p) == 2.0:
            return border_value
        p1 = p2

    # regular point in polygon algorithm
    # TODO use optimised implementation
    x, y = p

    contained = False
    # the edge from the last to the first point is checked first
    i = -1
    y1 = coords[-1, 1]
    y_gt_y1 = y > y1
    for y2 in coords[:, 1]:
        y_gt_y2 = y > y2
        if y_gt_y1:
            if not y_gt_y2:
                x1 = coords[i, 0]
                x2 = coords[i + 1, 0]
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
                x1 = coords[i, 0]
                x2 = coords[i + 1, 0]
                # only crossings "right" of the point should be counted
                x1GEx = x <= x1
                x2GEx = x <= x2
                if (x1GEx and x2GEx) or ((x1GEx or x2GEx) and (y2 - y) * (x2 - x1) >= (y2 - y1) * (x2 - x)):
                    contained = not contained

        y1 = y2
        y_gt_y1 = y_gt_y2
        i += 1

    return contained


def is_within_map(p: np.ndarray, boundary: np.ndarray, holes: Iterable[np.ndarray]) -> bool:
    if not inside_polygon(p, boundary, border_value=True):
        return False
    for hole in holes:
        if inside_polygon(p, hole, border_value=False):
            return False
    return True


def no_identical_consequent_vertices(coords):
    p1 = coords[-1]
    for p2 in coords:
        # TODO adjust allowed difference: rtol, atol
        if np.array_equal(p1, p2):
            return False
        p1 = p2

    return True


def get_intersection_status(p1, p2, q1, q2):
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
        # not crossing the line segment is considered to be ok
        # so x == 0.0 or x == 1.0 is not considered an intersection
        # assert np.allclose((p2 - p1) * x[0] + p1, (q2 - q1) * x[1] + q1)
        # assert np.allclose(np.dot(A, x), b)
        if x[0] <= 0.0 or x[1] <= 0.0 or x[0] >= 1.0 or x[1] >= 1.0:
            return 0
        # if np.all(0.0 <= x) and np.all(x <= 1.0):
        #     return 2
    except np.linalg.LinAlgError:
        # line segments are parallel (matrix is singular, set of equations is not solvable)
        return 0

    return 1


# special case of has_intersection()
def lies_behind_inner(p1: np.ndarray, p2: np.ndarray, v: np.ndarray) -> bool:
    # solve the set of equations
    # (p2-p1) lambda + (p1) = (v) mu
    #  in matrix form A x = b:
    # [(p1-p2) (v)] (lambda, mu)' = (p1)
    # because the vertex lies within the angle range between the two edge vertices
    #    (together with the other conditions on the polygons)
    #   this set of linear equations is always solvable (the matrix is regular)
    A = np.array([p1 - p2, v]).T
    b = np.array(p1)
    x = np.linalg.solve(A, b)

    # Debug:
    # try:
    #     x = np.linalg.solve(A, b)
    # except np.linalg.LinAlgError:
    #     raise ValueError
    # assert np.allclose((p2 - p1) * x[0] + p1, v * x[1])
    # assert np.allclose(np.dot(A, x), b)

    # vertices on the edge are possibly visible! ( < not <=)
    return x[1] < 1.0


def lies_behind(idx_p1: int, idx_p2: int, idx_v: int, idx_orig: int, coords: np.ndarray) -> bool:
    coords_origin = coords[idx_orig]
    coords_p1_rel = coords[idx_p1] - coords_origin
    coords_p2_rel = coords[idx_p2] - coords_origin
    coords_v_rel = coords[idx_v] - coords_origin
    return lies_behind_inner(coords_p1_rel, coords_p2_rel, coords_v_rel)


def no_self_intersection(coords):
    polygon_length = len(coords)
    # again_check = []
    for index_p1, index_q1 in combinations(range(polygon_length), 2):
        # always: index_p1 < index_q1
        if index_p1 == index_q1 - 1 or index_p1 == index_q1 + 1:
            # neighbouring edges never have an intersection
            continue
        p1, p2 = coords[index_p1], coords[(index_p1 + 1) % polygon_length]
        q1, q2 = coords[index_q1], coords[(index_q1 + 1) % polygon_length]
        intersect_status = get_intersection_status(p1, p2, q1, q2)
        if intersect_status == 1:
            return False
        # if intersect_status == 2:
        # TODO 4 different options. check which side the other edge lies on.
        # if edge changes sides this is a an intersection
        # again_check.append((p1, p2, q1, q2))
        # print(p1, p2, q1, q2)

    # TODO check for intersections across 2 edges! use computed intersection

    return True


def has_clockwise_numbering(coords: np.ndarray) -> bool:
    """tests if a polygon has clockwise vertex numbering
    approach: Sum over the edges, (x2 − x1)(y2 + y1). If the result is positive the curve is clockwise.
    from:
    https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    :param coords: the list of (x,y) coordinates representing the polygon to be tested
    :return: true if the polygon has been given in clockwise numbering
    """
    total_sum = 0.0
    p1 = coords[-1]
    for p2 in coords:
        x1, y1 = p1
        x2, y2 = p2
        total_sum += (x2 - x1) * (y2 + y1)
        p1 = p2
    return total_sum > 0


def check_polygon(polygon):
    """ensures that all the following conditions on the polygons are fulfilled:
    - must at least contain 3 vertices
    - no consequent vertices with identical coordinates in the polygons! In general might have the same coordinates
    - a polygon must not have self intersections (intersections with other polygons are allowed)
    """
    if not polygon.shape[0] >= 3:
        raise TypeError("Given polygons must at least contain 3 vertices.")
    if not polygon.shape[1] == 2:
        raise TypeError("Each point of a polygon must consist of two values (x,y).")
    if not no_identical_consequent_vertices(polygon):
        raise ValueError("Consequent vertices of a polynomial must not be identical.")
    if not no_self_intersection(polygon):
        raise ValueError("A polygon must not intersect it")


def check_data_requirements(boundary_coords: np.ndarray, list_hole_coords: List[np.ndarray]):
    """ensures that all the following conditions on the polygons are fulfilled:
        - basic polygon requirements (s. above)
        - edge numbering has to follow this convention (for easier computations):
            * outer boundary polygon: counter clockwise
            * holes: clockwise

    TODO test
    todo - polygons must not intersect each other
    TODO data rectification

    :param boundary_coords:
    :param list_hole_coords:
    :return:
    """
    check_polygon(boundary_coords)
    if has_clockwise_numbering(boundary_coords):
        raise ValueError("Vertex numbering of the boundary polygon must be counter clockwise.")
    for hole_coords in list_hole_coords:
        check_polygon(hole_coords)
        if not has_clockwise_numbering(hole_coords):
            raise ValueError("Vertex numbering of hole polygon must be clockwise.")


def find_within_range(
    vert_idx2repr: np.ndarray,
    repr1: float,
    repr2: float,
    candidate_idxs: Set[int],
    angle_range_less_180: bool,
    equal_repr_allowed: bool,
) -> Set[int]:
    """
    filters out all vertices whose representation lies within the range between
      the two given angle representations
    which range ('clockwise' or 'counter-clockwise') should be checked is determined by:
      - query angle (range) is < 180deg or not (>= 180deg)
    :param repr1:
    :param repr2:
    :param representations:
    :param angle_range_less_180: whether the angle between repr1 and repr2 is < 180 deg
    :param equal_repr_allowed: whether vertices with the same representation should also be returned
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
        # Note: vertices with the same representation will not NOT be returned!
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

    idxs_within = {i for i in candidate_idxs if within_filter_func(vert_idx2repr[i])}
    return idxs_within


def get_neighbour_idxs(i: int, vertex_edge_idxs: np.ndarray, edge_vertex_idxs: np.ndarray) -> Tuple[int, int]:
    edge_idx1, edge_idx2 = vertex_edge_idxs[i]
    neigh_idx1 = edge_vertex_idxs[edge_idx1, 0]
    neigh_idx2 = edge_vertex_idxs[edge_idx2, 1]
    return neigh_idx1, neigh_idx2


def skip_edge(
    node: int,
    edge2discard: int,
    candidates: Set[int],
    edge_vertex_idxs: np.ndarray,
    extremity_mask: np.ndarray,
    vertex_edge_idxs: np.ndarray,
):
    # (note: not identical, does not belong to the same polygon!)
    # mark this vertex as not visible (would otherwise add 0 distance edge in the graph)
    candidates.discard(node)
    # no points lie truly "behind" this edge as there is no "direction of sight" defined
    # <-> angle representation/range undefined for just this single edge
    # however if one considers the point neighbouring in the other direction (<-> two edges)
    # these two neighbouring edges define an invisible angle range
    # -> simply move the pointer
    v1, v2 = get_neighbour_idxs(node, vertex_edge_idxs, edge_vertex_idxs)
    range_less_180 = extremity_mask[node]
    # do not check the other neighbouring edge of vertex1 in the future (has been considered already)
    edge_idx = vertex_edge_idxs[node][edge2discard]
    return edge_idx, range_less_180, v1, v2


def find_candidates_behind(
    origin: int, v1: int, v2: int, candidates: Set[int], distances: np.ndarray, coords: np.ndarray
) -> Set[int]:
    dist_v1 = distances[v1]
    dist_v2 = distances[v2]
    max_distance = max(dist_v1, dist_v2)
    idxs_behind = set()
    # for all remaining vertices v it has to be tested if the line segment from query point (=origin) to v
    #    has an intersection with the current edge p1---p2
    for idx in candidates:
        # if a candidate is farther away from the query point than both vertices of the edge,
        #   it surely lies behind the edge
        # ATTENTION: even if a candidate is closer to the query point than both vertices of the edge,
        #   it still needs to be checked!
        dist2orig = distances[idx]
        further_away = dist2orig > max_distance
        if further_away or lies_behind(v1, v2, idx, origin, coords):
            idxs_behind.add(idx)
        # vertex lies in front of this edge
    return idxs_behind


def clean_visibles(visible_idxs: Set[int], cand_idx2repr: np.ndarray, vert_idx2dist: np.ndarray) -> Set[int]:
    # in case some vertices have the same representation, only return (link) the closest vertex
    if len(visible_idxs) <= 1:
        return visible_idxs

    cleaned = set()
    visible_idxs_sorted = sorted(visible_idxs, key=lambda i: cand_idx2repr[i])
    min_dist = np.inf
    first_idx = visible_idxs_sorted[0]
    rep_prev = cand_idx2repr[first_idx]
    selected_idx = 0
    for i in visible_idxs_sorted:
        rep = cand_idx2repr[i]
        if rep != rep_prev:
            cleaned.add(selected_idx)
            min_dist = np.inf
            rep_prev = rep

        dist = vert_idx2dist[i]
        if dist < min_dist:
            selected_idx = i
            min_dist = dist

    cleaned.add(selected_idx)
    return cleaned


def find_visible(
    origin: int,
    candidates: Set[int],
    edges_to_check: Set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    extremity_mask: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    vertex_edge_idxs: np.ndarray,
) -> Set[int]:
    """
    query_vertex: a vertex for which the visibility to the vertices should be checked.
        also non extremity vertices, polygon vertices and vertices with the same coordinates are allowed.
        query point also might lie directly on an edge! (angle = 180deg)
    :param candidates: the set of all vertex ids which should be checked for visibility.
        IMPORTANT: is being manipulated, so has to be a copy!
        IMPORTANT: must not contain the query vertex!
    :param edges_to_check: the set of edges which determine visibility
    :return: a set of tuples of all vertices visible from the query vertex and the corresponding distance
    """
    if len(candidates) == 0:
        return candidates

    visibles = set()
    # goal: eliminating all vertices lying 'behind' any edge
    while len(candidates) > 0:
        try:
            edge = edges_to_check.pop()
        except KeyError:
            break  # no more edges left to check

        v1, v2 = edge_vertex_idxs[edge]
        lies_on_edge = False
        range_less_180 = False

        if distances[v1] == 0.0:
            # vertex1 of the edge has the same coordinates as the query vertex
            # -> the origin lies on the edge
            lies_on_edge = True
            edge, range_less_180, v1, v2 = skip_edge(
                node=v1,
                edge2discard=0,
                candidates=candidates,
                edge_vertex_idxs=edge_vertex_idxs,
                extremity_mask=extremity_mask,
                vertex_edge_idxs=vertex_edge_idxs,
            )
            edges_to_check.discard(edge)

        elif distances[v2] == 0.0:
            # same for vertex2 of the edge
            # NOTE: it is unsupported that v1 as well as v2 have the same coordinates as the query vertex
            # (edge with length 0)
            lies_on_edge = True
            edge, range_less_180, v1, v2 = skip_edge(
                node=v2,
                edge2discard=1,
                candidates=candidates,
                edge_vertex_idxs=edge_vertex_idxs,
                extremity_mask=extremity_mask,
                vertex_edge_idxs=vertex_edge_idxs,
            )
            edges_to_check.discard(edge)

        repr1 = representations[v1]
        repr2 = representations[v2]
        repr_diff = abs(repr1 - repr2)
        if repr_diff == 2.0:
            # angle == 180deg -> on the edge
            lies_on_edge = True

        if lies_on_edge:
            # when the query vertex lies on an edge (or vertex) no behind/in front checks must be performed!
            # the neighbouring edges are visible for sure
            # attention: only add to visible set if vertex was a candidate!
            try:
                candidates.remove(v1)
                visibles.add(v1)
            except KeyError:
                pass
            try:
                candidates.remove(v2)
                visibles.add(v2)
            except KeyError:
                pass

            # all the candidates between the two vertices v1 v2 are not visible for sure
            # candidates with the same representation must not be deleted, because they can be visible!
            invisible_candidate_idxs = find_within_range(
                representations, repr1, repr2, candidates, angle_range_less_180=range_less_180, equal_repr_allowed=False
            )
            candidates.difference_update(invisible_candidate_idxs)
            continue

        # case: a 'regular' edge
        # eliminate all candidates which are blocked by the edge
        # that means inside the angle range spanned by the edge and actually behind it
        cand_idxs_tmp = candidates.copy()
        # the vertices belonging to the edge itself (its vertices) must not be checked.
        # use discard() instead of remove() to not raise an error (they might not be candidates)
        cand_idxs_tmp.discard(v1)
        cand_idxs_tmp.discard(v2)

        # for all candidate edges check if there are any candidate vertices (besides the ones belonging to the edge)
        #   within this angle range
        # the "view range" of an edge from a query point (spanned by the two vertices of the edge)
        #   is always < 180deg when the edge is not running through the query point (=180 deg)
        #  candidates with the same representation as v1 or v2 should be considered.
        #   they can be visible, but should be ruled out if they lie behind any edge!
        cand_idxs_tmp = find_within_range(
            representations, repr1, repr2, cand_idxs_tmp, angle_range_less_180=True, equal_repr_allowed=True
        )

        idxs_behind = find_candidates_behind(origin, v1, v2, cand_idxs_tmp, distances, coords)
        # vertices behind any edge are not visible
        candidates.difference_update(idxs_behind)

    # all edges have been checked
    # all remaining vertices were not concealed behind any edge and hence are visible
    visibles.update(candidates)
    return clean_visibles(visibles, representations, distances)


def find_visible_and_in_front(
    origin: int,
    nr_edges: int,
    coords: np.ndarray,
    candidates: Set[int],
    extremities: Iterable[int],
    extremity_mask: np.ndarray,
    distances: np.ndarray,
    representations: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    edge_vertex_idxs: np.ndarray,
):
    # vertices all belong to a polygon
    n1, n2 = get_neighbour_idxs(origin, vertex_edge_idxs, edge_vertex_idxs)
    # ATTENTION: polygons may intersect -> neighbouring extremities must NOT be visible from each other!
    # eliminate all vertices 'behind' the query point from the candidate set
    # since the query vertex is an extremity the 'outer' angle is < 180 degree
    # then the difference between the angle representation of the two edges has to be < 2.0
    # all vertices between the angle of the two neighbouring edges ('outer side')
    #   are not visible (no candidates!)
    # ATTENTION: vertices with the same angle representation might be visible and must NOT be deleted!
    n1_repr = representations[n1]
    n2_repr = representations[n2]
    idxs_behind = find_within_range(
        representations, n1_repr, n2_repr, candidates, angle_range_less_180=True, equal_repr_allowed=False
    )
    # do not consider points found to lie behind
    candidates.difference_update(idxs_behind)
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
    # IMPORTANT: check all extremities here, not just current candidates
    # do not check extremities with equal coords_rel (also query extremity itself!)
    #   and with the same angle representation (those edges must not get deleted from graph!)
    idxs_in_front = find_within_range(
        representations,
        repr1=angle_rep_inverse(n1_repr),
        repr2=angle_rep_inverse(n2_repr),
        candidate_idxs=set(extremities),
        angle_range_less_180=True,
        equal_repr_allowed=False,
    )
    # do not consider points lying in front when looking for visible extremities,
    # even if they are actually be visible.
    candidates.difference_update(idxs_in_front)
    # all edges have to be checked, except the 2 neighbouring edges (handled above!)
    edge_idxs2check = set(range(nr_edges))
    edge_idxs2check.difference_update(vertex_edge_idxs[origin])
    visible_idxs = find_visible(
        origin,
        candidates,
        edge_idxs2check,
        coords,
        representations,
        distances,
        extremity_mask,
        edge_vertex_idxs,
        vertex_edge_idxs,
    )
    return idxs_in_front, visible_idxs


def compute_graph_edges(
    nr_edges: int,
    extremity_indices: Iterable[int],
    reprs_n_distances: Dict[int, np.ndarray],
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    extremity_mask: np.ndarray,
    vertex_edge_idxs: np.ndarray,
) -> Dict[Tuple[int, int], float]:
    # IMPORTANT: add all extremities (even if they turn out to be dangling in the end)

    connections = {}
    for extr_ptr, origin_idx in enumerate(extremity_indices):
        vert_idx2repr, vert_idx2dist = reprs_n_distances[origin_idx]
        # optimisation: extremities are always visible to each other
        # (bi-directional relation -> undirected edges in the graph)
        #  -> do not check extremities which have been checked already
        #  (must give the same result when algorithms are correct)
        # the origin extremity itself must also not be checked when looking for visible neighbours
        candidate_idxs = set(extremity_indices[extr_ptr + 1 :])
        idxs_in_front, visible_idxs = find_visible_and_in_front(
            origin_idx,
            nr_edges,
            coords,
            candidate_idxs,
            extremity_indices,
            extremity_mask,
            vert_idx2dist,
            vert_idx2repr,
            vertex_edge_idxs,
            edge_vertex_idxs,
        )
        # "thin out" the graph:
        # remove already existing edges in the graph to the extremities in front

        for i in idxs_in_front:
            connections.pop((origin_idx, i), None)
            connections.pop((i, origin_idx), None)

        for i in visible_idxs:
            d = vert_idx2dist[i]
            connections[(origin_idx, i)] = d
            connections[(i, origin_idx)] = d

    return connections


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


def find_identical(candidates: Iterable[int], reprs_n_distances: Dict[int, np.ndarray]) -> Dict[int, int]:
    # for shortest path computations all graph nodes should be unique
    # join all nodes with the same coordinates
    merging_mapping = {}
    # symmetric relation -> only consider one direction
    for n1, n2 in itertools.combinations(candidates, 2):
        dist = get_distance(n1, n2, reprs_n_distances)
        if dist == 0.0:  # same coordinates
            merging_mapping[n2] = n1

    return merging_mapping


def find_identical_single(i: int, candidates: Iterable[int], reprs_n_distances: Dict[int, np.ndarray]) -> int:
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
) -> nx.DiGraph:
    edges = compute_graph_edges(
        nr_edges,
        extremity_indices,
        reprs_n_distances,
        coords,
        edge_vertex_idxs,
        extremity_mask,
        vertex_edge_idxs,
    )

    graph = nx.DiGraph()
    # IMPORTANT: add all extremities (even if they turn out to be dangling in the end),
    # adding start and goal nodes at query time might connect them!
    graph.add_nodes_from(extremity_indices)
    for (start, goal), dist in edges.items():
        graph.add_edge(start, goal, weight=dist)

    merge_mapping = find_identical(graph.nodes, reprs_n_distances)
    if len(merge_mapping) > 0:
        nx.relabel_nodes(graph, merge_mapping, copy=False)

    return graph


def try_extraction(json_data, key):
    try:
        extracted_data = json_data[key]
    except KeyError as e:
        raise ValueError(f"The expected key {key} was not found in the JSON file:\n{e}")
    return extracted_data


def convert2polygon(json_list):
    return [tuple(coord_pair_list) for coord_pair_list in json_list]


def read_json(path2json_file):
    """
    Parse data from a JSON file and save as lists of tuples for both boundary and holes.
    NOTE: The format of the JSON file is explained in the command line script (argparse definition)

    :param path2json_file: The path to the input json file
    :return: The parsed lists of boundaries and holes
    """
    # parse data from the input file
    with open(path2json_file, "r") as json_file:
        json_data = json_file.read()
    json_loaded = json.loads(json_data)
    boundary_data = try_extraction(json_loaded, BOUNDARY_JSON_KEY)
    holes_data = try_extraction(json_loaded, HOLES_JSON_KEY)
    boundary_coordinates = convert2polygon(boundary_data)
    list_of_holes = [convert2polygon(hole_data) for hole_data in holes_data]
    return boundary_coordinates, list_of_holes


def convert_gridworld(size_x: int, size_y: int, obstacle_iter: iter, simplify: bool = True) -> (list, list):
    """
    prerequisites: grid world must not have non-obstacle cells which are surrounded by obstacles
    ("single white cell in black surrounding" = useless for path planning)
    :param size_x: the horizontal grid world size
    :param size_y: the vertical grid world size
    :param obstacle_iter: an iterable of coordinate pairs (x,y) representing blocked grid cells (obstacles)
    :param simplify: whether the polygons should be simplified or not. reduces edge amount, allow diagonal edges
    :return: an boundary polygon (counter clockwise numbering) and a list of hole polygons (clockwise numbering)
    NOTE: convert grid world into polygons in a way that coordinates coincide with grid!
        -> no conversion of obtained graphs needed!
        the origin of the polygon coordinate system is (-0.5,-0.5) in the grid cell system (= corners of the grid world)
    """

    assert size_x > 0 and size_y > 0

    if len(obstacle_iter) == 0:
        # there are no obstacles. return just the simple boundary rectangle
        return [np.array(x, y) for x, y in [(0, 0), (size_x, 0), (size_x, size_y), (0, size_y)]], []

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
            if not (just_turned or boundary_detect_fct(current_pos + directions[left_index])):
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
                    # there is a node at the bottom left corner of the start position (offset= (0,0) )
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
            # make edge numbering counter clockwise!
            edge_list.reverse()
        return np.array(edge_list, dtype=float)

    # build the boundary polygon
    # start at the lowest and leftmost unblocked grid cell
    start_pos = find_start(start_pos=(0, 0), boundary_detect_fct=is_unblocked)
    # print(start_pos+directions[3])
    # raise ValueError
    boundary_edges = construct_polygon(start_pos, boundary_detect_fct=is_blocked, cntr_clockwise_wanted=True)

    if simplify:
        # TODO
        raise NotImplementedError()

    # detect which of the obstacles have to be converted into holes
    # just the obstacles inside the boundary polygon are part of holes
    # shift coordinates by +(0.5,0.5) for correct detection
    # the border value does not matter here

    def get_unchecked_obstacles(obstacles: Iterable, poly: np.ndarray, required_val: bool = True) -> List:
        unchecked_obstacles = []
        for o in obstacles:
            p = o + 0.5
            if inside_polygon(p, poly, border_value=True) == required_val:
                unchecked_obstacles.append(o)

        return unchecked_obstacles

    unchecked_obstacles = get_unchecked_obstacles(obstacle_iter, boundary_edges)

    hole_list = []
    while len(unchecked_obstacles) > 0:
        start_pos = find_start(start_pos=(0, 0), boundary_detect_fct=pos_in_iter, iter=unchecked_obstacles)
        hole = construct_polygon(start_pos, boundary_detect_fct=is_unblocked, cntr_clockwise_wanted=False)

        # detect which of the obstacles still do not belong to any hole:
        # delete the obstacles which are included in the just constructed hole
        unchecked_obstacles = get_unchecked_obstacles(unchecked_obstacles, hole, required_val=False)

        if simplify:
            # TODO
            pass

        hole_list.append(hole)

    return boundary_edges, hole_list


def angle_rep_inverse(repr: Optional[float]) -> Optional[float]:
    if repr is None:
        repr_inv = None
    else:
        repr_inv = (repr + 2.0) % 4.0
    return repr_inv


def compute_extremity_idxs(coordinates: np.ndarray) -> List[int]:
    """identify all protruding points = vertices with an inside angle of > 180 degree ('extremities')
    expected edge numbering:
        outer boundary polygon: counter clockwise
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
    nr_coordinates = len(coordinates)
    extr_idxs = []
    p1 = coordinates[-2]
    p2 = coordinates[-1]
    for i, p3 in enumerate(coordinates):
        # since consequent vertices are not permitted to be equal,
        #   the angle representation of the difference is well defined
        diff_p3_p2 = p3 - p2
        diff_p1_p2 = p1 - p2
        repr_p3_p2, _ = compute_repr_n_dist(diff_p3_p2)
        repr_p1_p2, _ = compute_repr_n_dist(diff_p1_p2)
        rep_diff = repr_p3_p2 - repr_p1_p2
        if rep_diff % 4.0 < 2.0:  # inside angle > 180 degree
            # p2 is an extremity
            idx_p2 = (i - 1) % nr_coordinates
            extr_idxs.append(idx_p2)

        # move to the next point
        p1 = p2
        p2 = p3
    return extr_idxs


def load_pickle(path=DEFAULT_PICKLE_NAME):
    print("loading map from:", path)
    with open(path, "rb") as f:
        return pickle.load(f)
