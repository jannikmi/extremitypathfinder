import math
from collections.abc import Sequence

import numpy as np

try:
    from numba import b1, f8, i8, njit, typeof, void
except ImportError:
    # replace Numba functionality with "transparent" implementations
    from extremitypathfinder.numba_replacements import (
        b1,
        f8,
        i8,
        njit,
        typeof,
        void,
    )

FloatTuple = typeof((1.0, 1.0))


@njit(FloatTuple(f8[:]), cache=True)
def _compute_repr_n_dist(np_vector: np.ndarray) -> tuple[float, float]:
    """computing representation for the angle from the origin to a given vector

    value in [0.0 : 4.0[
    every quadrant contains angle measures from 0.0 to 1.0:
    the "diamond angle" |dy| / (|dx| + |dy|) measured from the start of the quadrant (e.g. dy / (dx + dy))
    there are 4 quadrants (counterclockwise numbering)
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
    angle(p): counterclockwise angle between the two line segments (0,0)'--(1,0)' and (0,0)'--p
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

        # normalise angle measure to [0; 1]: the diamond angle.
        # Its derivative with respect to the angle is bounded away from 0,
        # so it resolves directions close to the axes as finely as all others.
        # The sine (measure / distance) is flat at the axes and maps directions up to about 1e-8 rad apart
        # to the same value (wrong visibility decisions, cf. tests/test_core_regressions.py)
        angle_measure /= abs(dx) + abs(dy)
        angle_measure += quadrant

    return angle_measure, distance


def reps_n_distances(vectors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """the angle representations and lengths of vectors, vectorised ``_compute_repr_n_dist()``

    Note: the same floating point operations, hence identical results (NaN for zero vectors)
    """
    dx = vectors[:, 0]
    dy = vectors[:, 1]
    distances = np.sqrt(dx**2 + dy**2)
    dx_positive = dx >= 0
    dy_positive = dy >= 0
    quadrants = [
        dx_positive & dy_positive,
        ~dx_positive & dy_positive,
        ~dx_positive & ~dy_positive,
    ]
    quadrant = np.select(quadrants, [0.0, 1.0, 2.0], 3.0)
    angle_measure = np.select(quadrants, [dy, -dx, -dy], dx)
    with np.errstate(invalid="ignore", divide="ignore"):
        representations = angle_measure / (np.abs(dx) + np.abs(dy)) + quadrant
    representations[distances == 0.0] = np.nan
    return representations, distances


@njit(b1(f8[:], f8[:], f8[:]), cache=True)
def _lies_on_edge(p: np.ndarray, p1: np.ndarray, p2: np.ndarray) -> bool:
    """whether the point lies exactly on the edge p1-p2 (its vertices included)

    exact cross product test (exact for coordinates whose products have no rounding errors, e.g. integers)
    or the angle representations of the edge vertices seen from the point differ by exactly 180 degree
    """
    v1 = p1 - p
    v2 = p2 - p
    collinear = v1[0] * v2[1] - v1[1] * v2[0] == 0.0
    # collinear: on the edge when between its vertices
    if collinear and v1[0] * v2[0] <= 0.0 and v1[1] * v2[1] <= 0.0:
        return True
    rep1, _ = _compute_repr_n_dist(v1)
    rep2, _ = _compute_repr_n_dist(v2)
    return abs(rep1 - rep2) == 2.0


@njit(b1(f8[:], f8[:, :], b1), cache=True)
def _inside_polygon(p: np.ndarray, coords: np.ndarray, border_value: bool) -> bool:
    # should return the border value for point equal to any polygon vertex
    # and if the point p lies on any polygon edge
    p1 = coords[-1, :]
    for p2 in coords[:]:
        if _lies_on_edge(p, p1, p2):
            return border_value
        p1 = p2

    # regular point in polygon algorithm: ray casting
    x, y = p
    x_coords = coords[:, 0]
    y_coords = coords[:, 1]
    nr_coords = len(x_coords)
    inside = False

    # the edge from the last to the first point is checked first
    y1 = y_coords[-1]
    y_gt_y1 = y > y1
    for i in range(nr_coords):
        y2 = y_coords[i]
        y_gt_y2 = y > y2
        if y_gt_y1 ^ y_gt_y2:  # XOR
            # [p1-p2] crosses horizontal line in p
            x1 = x_coords[i - 1]
            x2 = x_coords[i]
            # only count crossings "right" of the point ( >= x)
            x_le_x1 = x <= x1
            x_le_x2 = x <= x2
            if x_le_x1 or x_le_x2:
                if x_le_x1 and x_le_x2:
                    # p1 and p2 are both to the right -> valid crossing
                    inside = not inside
                else:
                    # compare the slope of the line [p1-p2] and [p-p2]
                    # depending on the position of p2 this determines whether
                    # the polygon edge is right or left of the point
                    # to avoid expensive division the divisors (of the slope dy/dx) are brought to the other side
                    # ( dy/dx > a  ==  dy > a * dx )
                    # Note: the products overflow only for coordinate differences beyond ~1e154 (float64)
                    # only one of the points is to the right
                    slope1 = (y2 - y) * (x2 - x1)
                    slope2 = (y2 - y1) * (x2 - x)
                    # NOTE: accept slope equality to also detect if p lies directly on an edge
                    if y_gt_y1:
                        if slope1 <= slope2:
                            inside = not inside
                    elif slope1 >= slope2:  # NOT y_gt_y1
                        inside = not inside

        # next point
        y1 = y2
        y_gt_y1 = y_gt_y2

    return inside


@njit(b1(f8[:, :]), cache=True)
def _no_identical_consequent_vertices(coords: np.ndarray) -> bool:
    p1 = coords[-1]
    for p2 in coords:
        # Note: an exact comparison suffices,
        # the angle representation of an edge is only undefined for identical vertices
        if np.array_equal(p1, p2):
            return False
        p1 = p2

    return True


@njit(b1(f8[:, :]), cache=True)
def _has_clockwise_numbering(coords: np.ndarray) -> bool:
    """tests if a polygon has clockwise vertex numbering
    approach: Sum over the edges, (x2 − x1)(y2 + y1). If the result is positive the curve is clockwise.
    from:
    https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
    :param coords: the list of (x,y) coordinates representing the polygon to be tested
    :return: true if the polygon has been given in clockwise numbering
    """
    # Translating near the origin avoids catastrophic cancellation for small
    # polygons represented by large-magnitude coordinates (cf. _check_polygon()).
    y0 = coords[0, 1]
    total_sum = 0.0
    p1 = coords[-1]
    for p2 in coords:
        x1, y1 = p1
        x2, y2 = p2
        total_sum += (x2 - x1) * ((y2 - y0) + (y1 - y0))
        p1 = p2
    return total_sum > 0


def edge_vertex_mappings(nr_vertices: int) -> tuple[np.ndarray, np.ndarray]:
    """the edge <-> vertex index mappings of a single polygon

    edge i connects the vertices i-1 and i: edge_vertex_idxs[i] = (i-1, i)
    vertex i connects the edges i and i+1: vertex_edge_idxs[i] = (i, i+1)

    :return: edge_vertex_idxs, vertex_edge_idxs
    """
    idxs = np.arange(nr_vertices)
    edge_vertex_idxs = np.stack([(idxs - 1) % nr_vertices, idxs], axis=1)
    vertex_edge_idxs = np.stack([idxs, (idxs + 1) % nr_vertices], axis=1)
    return edge_vertex_idxs, vertex_edge_idxs


@njit(b1(f8[:], f8[:], f8[:]), cache=True)
def _lies_behind_inner(p1: np.ndarray, p2: np.ndarray, v: np.ndarray) -> bool:
    # special case of get_intersection_status()
    # solve the set of equations
    # (p2-p1) lambda + (p1) = (v) mu
    #  in matrix form A x = b:
    # [(p1-p2) (v)] (lambda, mu)' = (p1)
    # because the vertex lies within the angle range between the two edge vertices
    #    (together with the other conditions on the polygons)
    #   this set of linear equations is always solvable (the matrix is regular)
    # only mu is needed: Cramer's rule instead of a general solver (#185).
    # No LAPACK call: identical in both modes and Numba does not need SciPy.
    # With integer coordinates (e.g. grids) the determinant and the numerator are exact,
    # so a vertex exactly on the edge (mu == 1) is decided exactly.
    a = p1[0] - p2[0]
    b = p1[1] - p2[1]
    det = a * v[1] - v[0] * b
    if det == 0.0:
        # the direction is parallel to the edge (in floating point): the line of sight does not cross the edge,
        # at most it runs along it, which does not block (cf. the Numerical Policy).
        # E.g. the vertex and the edge lie on one ray from the origin (collinear vectors of different length
        # might have representations differing in the last bit), or the determinant of nearly parallel
        # directions rounds to 0 (#3).
        # Without this check, Numba raises a ZeroDivisionError and plain Python returns ±inf < 1.
        return False
    mu = (a * p1[1] - b * p1[0]) / det
    # vertices on the edge are possibly visible! ( < not <=)
    return mu < 1.0


def is_within_map(
    p: np.ndarray,
    boundary: np.ndarray | None,
    holes: Sequence[np.ndarray],
    hole_bounds: tuple[np.ndarray, np.ndarray] | None = None,
) -> bool:
    """whether the point lies within the boundary polygon (or on its border) and not inside any hole

    :param hole_bounds: the bounding boxes of the holes (``polygon_bounds(holes)``).
        Only the holes whose closed bounding box contains the point are tested:
        a point outside of it lies neither inside the hole nor on its border. Same result, faster.
    """
    if boundary is not None and not _inside_polygon(p, boundary, border_value=True):
        return False
    if hole_bounds is not None:
        minima, maxima = hole_bounds
        in_box = np.flatnonzero(np.all((minima <= p) & (p <= maxima), axis=1))
        holes = [holes[i] for i in in_box]
    return not any(_inside_polygon(p, hole, border_value=False) for hole in holes)


def get_neighbour_idxs(
    i: int, vertex_edge_idxs: np.ndarray, edge_vertex_idxs: np.ndarray
) -> tuple[int, int]:
    """the indices of the previous and the next vertex of a polygon vertex"""
    edge_idx1, edge_idx2 = vertex_edge_idxs[i]
    return int(edge_vertex_idxs[edge_idx1, 0]), int(edge_vertex_idxs[edge_idx2, 1])


@njit(b1(i8, i8, i8, i8, f8[:, :]), cache=True)
def _lies_behind(
    idx_p1: int, idx_p2: int, idx_v: int, idx_orig: int, coords: np.ndarray
) -> bool:
    coords_origin = coords[idx_orig]
    coords_p1_rel = coords[idx_p1] - coords_origin
    coords_p2_rel = coords[idx_p2] - coords_origin
    coords_v_rel = coords[idx_v] - coords_origin
    return _lies_behind_inner(coords_p1_rel, coords_p2_rel, coords_v_rel)


# nodes of the edge tree up to this height above the leaves store no edge:
# their (at most ``DIRECT_CHECK_SIZE``) candidates are checked right away (cf. ``_blocked_candidates()``)
DIRECT_CHECK_LEVEL = 4
DIRECT_CHECK_SIZE = 2**DIRECT_CHECK_LEVEL


@njit(f8(i8, i8, f8[:, :], f8[:, :]), cache=True)
def _edge_distance(
    edge: int, candidate: int, edge_terms: np.ndarray, candidate_terms: np.ndarray
) -> float:
    """the distance of an edge along the ray from the origin through a candidate, in multiples of its distance

    The same operations as ``_lies_behind_inner()`` with the terms of the edge computed in advance.
    """
    a = edge_terms[edge, 0]
    b = edge_terms[edge, 1]
    det = a * candidate_terms[candidate, 1] - candidate_terms[candidate, 0] * b
    if det == 0.0:
        return np.inf
    return edge_terms[edge, 2] / det


@njit(b1(i8, i8, f8[:, :], f8[:, :], i8[:], i8[:]), cache=True)
def _behind_edge(
    edge: int,
    candidate: int,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> bool:
    """whether a candidate within the angle range of an edge lies behind it (cf. ``_lies_behind()``)

    Not behind the edges it touches (its "own" edges, cf. ``_blocked_candidates()``).
    """
    # optimisation: a candidate farther away than both vertices of the edge surely lies behind it
    # ATTENTION: a candidate closer than both vertices still needs to be checked!
    # vertices on the edge are possibly visible! ( < not <=)
    behind = candidate_terms[candidate, 2] > edge_terms[edge, 3]
    if not behind:
        behind = _edge_distance(edge, candidate, edge_terms, candidate_terms) < 1.0
    if not behind:
        return False
    for i in range(own_edges_start[candidate], own_edges_start[candidate + 1]):
        if own_edges[i] == edge:
            return False
    return True


@njit(void(i8[:], b1[:], i8, i8, i8, f8[:, :], f8[:, :], i8[:], i8[:]), cache=True)
def _insert_edge(
    tree: np.ndarray,
    blocked: np.ndarray,
    node: int,
    level: int,
    edge: int,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> None:
    """adds an edge spanning all directions of a node to the tree (cf. ``_blocked_candidates()``)

    :param level: the height of the node above the leaves
    """
    size = len(tree)
    while level > DIRECT_CHECK_LEVEL:
        stored = tree[node]
        if stored < 0:
            tree[node] = edge
            return
        # optimisation: an edge farther away than both vertices of another edge lies behind it in all directions
        # of the node. It blocks no candidate the other edge does not block (cf. ``_behind_edge()``)
        if edge_terms[stored, 3] < edge_terms[edge, 4]:
            return
        if edge_terms[edge, 3] < edge_terms[stored, 4]:
            tree[node] = edge
            return
        # the candidates of the node: [first, first + 2^level), the left child holds those up to the middle
        first = (node << level) - size
        middle = first + (1 << level) // 2 - 1
        closer_in_middle = _edge_distance(
            edge, middle, edge_terms, candidate_terms
        ) < _edge_distance(stored, middle, edge_terms, candidate_terms)
        closer_at_first = _edge_distance(
            edge, first, edge_terms, candidate_terms
        ) < _edge_distance(stored, first, edge_terms, candidate_terms)
        if closer_in_middle:
            tree[node] = edge
            edge = stored
        # the edge not stored can only be closer on one side of the middle: pass it on to that child
        level -= 1
        node *= 2
        if closer_at_first == closer_in_middle:
            node += 1
    # the few candidates of a node at the bottom are checked right away
    first = (node << level) - size
    for candidate in range(first, first + (1 << level)):
        if not blocked[candidate]:
            blocked[candidate] = _behind_edge(
                edge,
                candidate,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )


@njit(b1[:](i8[:], i8[:], i8[:], f8[:, :], f8[:, :], i8[:], i8[:]), cache=True)
def _blocked_candidates(
    range_edges: np.ndarray,
    range_first: np.ndarray,
    range_last: np.ndarray,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> np.ndarray:
    """which candidates lie behind an edge, in ``O((r log k + k) log k)`` for r ranges and k candidates

    The candidates are sorted by direction. Each range is an edge with the first and the last candidate
    (inclusive positions) whose direction lies strictly within the angle range of the edge.

    A segment tree over the candidates stores at most one edge per node: of all edges spanning the directions
    of a node, the closest one in the middle direction ("Li Chao tree"). Seen from the origin, two edges swap
    their order at most once (two lines cross at most once), so the edge losing in the middle can only be closer
    on one side and is passed on to that child. The nodes at the bottom (up to ``DIRECT_CHECK_SIZE`` candidates)
    store no edge: an edge arriving there is checked against their candidates right away, like every edge
    with a small range. The closest edge in the direction of a candidate has then either been checked
    against it or is stored on the way from its leaf to the root: only these edges remain to be checked.

    Edges are only compared in the directions of candidates. No crossings are computed: overlapping polygons
    (crossing edges) need no special treatment.

    :param edge_terms: per edge (row: the edge index), with its vertices relative to the origin:
        ``a = x1 - x2``, ``b = y1 - y2``, ``a * y1 - b * x1`` (cf. ``_lies_behind_inner()``),
        the maximum distance of its vertices, a lower bound of its distance
    :param candidate_terms: per candidate (in sorted order), relative to the origin: x, y, its distance
    :param own_edges_start: the edges touching candidate i, which do not block it (e.g. a query point on an edge):
        ``own_edges[own_edges_start[i] : own_edges_start[i + 1]]``.
        IMPORTANT: the candidate must lie on these edges. An edge in front of it would be taken for the closest one
        in its direction, and the edges behind that edge would not be checked.
    """
    nr_candidates = len(candidate_terms)
    blocked = np.zeros(nr_candidates, dtype=np.bool_)
    # a complete binary tree: node i has the children 2i and 2i+1, the leaves would start at ``size``
    size = 1
    while size < nr_candidates:
        size *= 2
    tree = np.full(size, -1, dtype=np.int64)

    for i in range(len(range_edges)):
        edge = int(range_edges[i])
        first = int(range_first[i])
        last = int(range_last[i])
        if last - first < DIRECT_CHECK_SIZE:
            # the most frequent case: a few candidates are checked right away
            for candidate in range(first, last + 1):
                if not blocked[candidate]:
                    blocked[candidate] = _behind_edge(
                        edge,
                        candidate,
                        edge_terms,
                        candidate_terms,
                        own_edges_start,
                        own_edges,
                    )
            continue
        # the nodes which together cover exactly the range: at most two per level
        first += size
        last += size
        level = 0
        while first <= last:
            if first & 1:
                _insert_edge(
                    tree,
                    blocked,
                    first,
                    level,
                    edge,
                    edge_terms,
                    candidate_terms,
                    own_edges_start,
                    own_edges,
                )
                first += 1
            if not last & 1:
                _insert_edge(
                    tree,
                    blocked,
                    last,
                    level,
                    edge,
                    edge_terms,
                    candidate_terms,
                    own_edges_start,
                    own_edges,
                )
                last -= 1
            first >>= 1
            last >>= 1
            level += 1

    # most nodes are empty: per node the closest node above it which stores an edge (0: none), level by level
    above = np.zeros(2 * size, dtype=np.int64)
    first = 1
    while first < size:
        stored_above = np.where(
            tree[first : 2 * first] >= 0,
            np.arange(first, 2 * first),
            above[first : 2 * first],
        )
        above[2 * first : 4 * first] = np.repeat(stored_above, 2)
        first *= 2

    for candidate in range(nr_candidates):
        node = above[candidate + size]
        while node > 0 and not blocked[candidate]:
            blocked[candidate] = _behind_edge(
                tree[node],
                candidate,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )
            node = above[node]
    return blocked
