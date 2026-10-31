import math
from collections.abc import Iterable

import numpy as np

from extremitypathfinder import configs

try:
    from numba import b1, f8, i8, njit, typeof
except ImportError:
    # replace Numba functionality with "transparent" implementations
    from extremitypathfinder.numba_replacements import b1, f8, i8, njit, typeof

FloatTuple = typeof((1.0, 1.0))


@njit(FloatTuple(f8[:]), cache=True)
def _compute_repr_n_dist(np_vector: np.ndarray) -> tuple[float, float]:
    """computing representation for the angle from the origin to a given vector

    value in [0.0 : 4.0[
    every quadrant contains angle measures from 0.0 to 1.0
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

        # normalise angle measure to [0; 1]
        angle_measure /= distance
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
        representations = angle_measure / distances + quadrant
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
    A = np.empty((2, 2), dtype=configs.DTYPE_FLOAT)
    A[:, 0] = p1 - p2
    A[:, 1] = v
    # Note: parallel lines are not possible here (singular matrix)
    x = np.linalg.solve(A, p1)
    # vertices on the edge are possibly visible! ( < not <=)
    return x[1] < 1.0


def is_within_map(
    p: np.ndarray,
    boundary: np.ndarray | None,
    holes: Iterable[np.ndarray],
) -> bool:
    """whether the point lies within the boundary polygon (or on its border) and not inside any hole"""
    if boundary is not None and not _inside_polygon(p, boundary, border_value=True):
        return False
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
