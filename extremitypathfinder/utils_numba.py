import math
from typing import Optional, Tuple

import numpy as np

from extremitypathfinder import configs

try:
    from numba import b1, f8, i8, njit, typeof, void
except ImportError:
    using_numba = False
    # replace Numba functionality with "transparent" implementations
    from extremitypathfinder.numba_replacements import b1, f8, i8, njit, typeof, void

FloatTuple = typeof((1.0, 1.0))


@njit(FloatTuple(f8[:]), cache=True)
def _compute_repr_n_dist(np_vector: np.ndarray) -> Tuple[float, float]:
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


@njit(cache=True)
def _angle_rep_inverse(repr: Optional[float]) -> Optional[float]:
    if repr is None:
        repr_inv = None
    else:
        repr_inv = (repr + 2.0) % 4.0
    return repr_inv


@njit(b1(f8[:], f8[:, :], b1), cache=True)
def _inside_polygon(p: np.ndarray, coords: np.ndarray, border_value: bool) -> bool:
    # should return the border value for point equal to any polygon vertex
    # TODO overflow possible with large values when comparing slopes, change procedure
    # and if the point p lies on any polygon edge
    p1 = coords[-1, :]
    for p2 in coords[:]:
        if np.array_equal(p2, p):
            return border_value
        rep_p1_p, _ = _compute_repr_n_dist(p1 - p)
        rep_p2_p, _ = _compute_repr_n_dist(p2 - p)
        if abs(rep_p1_p - rep_p2_p) == 2.0:
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
        # TODO adjust allowed difference: rtol, atol
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
    total_sum = 0.0
    p1 = coords[-1]
    for p2 in coords:
        x1, y1 = p1
        x2, y2 = p2
        total_sum += (x2 - x1) * (y2 + y1)
        p1 = p2
    return total_sum > 0


@njit(void(f8[:, :], b1[:]), cache=True)
def _fill_extremity_mask(coordinates, extremity_mask):
    nr_coordinates = len(extremity_mask)
    p1 = coordinates[-2]
    p2 = coordinates[-1]
    for i, p3 in enumerate(coordinates):
        # since consequent vertices are not permitted to be equal,
        #   the angle representation of the difference is well-defined
        diff_p3_p2 = p3 - p2
        diff_p1_p2 = p1 - p2
        repr_p3_p2, _ = _compute_repr_n_dist(diff_p3_p2)
        repr_p1_p2, _ = _compute_repr_n_dist(diff_p1_p2)
        rep_diff = repr_p3_p2 - repr_p1_p2
        if rep_diff % 4.0 < 2.0:  # inside angle > 180 degree
            # p2 is an extremity
            idx_p2 = (i - 1) % nr_coordinates
            extremity_mask[idx_p2] = True

        # move to the next point
        p1 = p2
        p2 = p3


# TODO
@njit(void(i8[:, :], i8[:, :]), cache=True)
def _fill_edge_vertex_idxs(edge_vertex_idxs, vertex_edge_idxs):
    nr_coords = len(edge_vertex_idxs)
    v1 = -1 % nr_coords
    # TODO col 1 is just np.arange?!
    for edge_idx, v2 in enumerate(range(nr_coords)):
        v1_idx = v1
        v2_idx = v2
        edge_vertex_idxs[edge_idx, 0] = v1_idx
        edge_vertex_idxs[edge_idx, 1] = v2_idx
        vertex_edge_idxs[v1_idx, 1] = edge_idx
        vertex_edge_idxs[v2_idx, 0] = edge_idx
        # move to the next vertex/edge
        v1 = v2


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
    # A = np.array([p1 - p2, v]).T
    x = np.linalg.solve(A, p1)
    # Note: parallel lines are not possible here (singular matrix)
    # try:
    #     x = np.linalg.solve(A, p1)
    # except np.linalg.LinAlgError:
    #     raise Exception("parallel lines")

    # Debug:
    # assert np.allclose((p2 - p1) * x[0] + p1, v * x[1])
    # assert np.allclose(np.dot(A, x), b)

    # vertices on the edge are possibly visible! ( < not <=)
    return x[1] < 1.0


@njit(b1(i8, i8, i8, i8, f8[:, :]), cache=True)
def _lies_behind(
    idx_p1: int, idx_p2: int, idx_v: int, idx_orig: int, coords: np.ndarray
) -> bool:
    coords_origin = coords[idx_orig]
    coords_p1_rel = coords[idx_p1] - coords_origin
    coords_p2_rel = coords[idx_p2] - coords_origin
    coords_v_rel = coords[idx_v] - coords_origin
    return _lies_behind_inner(coords_p1_rel, coords_p2_rel, coords_v_rel)
