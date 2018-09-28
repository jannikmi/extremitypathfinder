from itertools import combinations

import numpy as np

from extremitypathfinder.helper_classes import AngleRepresentation


# TODO numba precompilation of some parts possible?! do profiling first!
# TODO lazy_python: origin shift translate, angle representatione evaluation, graph a star heuristic


def inside_polygon(x, y, coords, border_value):
    # should return the border value for point equal to any polygon vertex
    for c in coords[:]:
        if np.all(c == [x, y]):
            return border_value

    # and if the point p lies on any polygon edge
    p = np.array([x, y])
    p1 = coords[-1, :]
    for p2 in coords[:]:
        if abs((AngleRepresentation(p1 - p).value - AngleRepresentation(p2 - p).value)) == 2.0:
            return border_value
        p1 = p2

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


def no_identical_consequent_vertices(coords):
    p1 = coords[-1]
    for p2 in coords:
        assert not np.all(p1 == p2)
        p1 = p2

    return True


def no_self_intersection(coords):
    def has_intersection(p1, p2, q1, q2):
        # solve the set of equations
        # (p2-p1) lambda + (p1) = (q2-q1) mu + (q1)
        #  in matrix form A x = b:
        # [(p2-p1) (q1-q2)] (lambda, mu)' = (q1-p1)
        A = np.array([p2 - p1, q1 - q2])
        b = np.array(q1 - p1)
        try:
            x = np.linalg.solve(A, b)
            # not crossing the line segment is considered to be ok
            # so x == 0.0 or x == 1.0 is not considered an intersection
            return 0.0 < x[0] < 1.0
        except np.linalg.LinAlgError:
            # line segments might be parallel (set of equations not solvable)
            return False

    polygon_length = len(coords)
    for index_p1, index_q1 in combinations(range(polygon_length), 2):
        # TODO optimisation: neighbouring edges never have an intersection
        p1, p2 = coords[index_p1], coords[(index_p1 + 1) % polygon_length]
        q1, q2 = coords[index_q1], coords[(index_q1 + 1) % polygon_length]
        if has_intersection(p1, p2, q1, q2):
            return False

    # TODO check for intersections across 2 edges!
    return True


def has_clockwise_numbering(coords):
    # approach: when numbering is clockwise:
    # at least the majority of averaged consequent point triplets lie...
    #  ... inside the polygon when the second point is an extremity
    #  ... else outside

    threshold = round(len(coords) / 2)
    positive_tests = 0

    p1 = coords[-2]
    p2 = coords[-1]
    for p3 in coords:
        x, y = (p1 + p2 + p3) / 3.0
        result = inside_polygon(x, y, np.array([p1, p2, p3]), border_value=True)
        if (AngleRepresentation(p3 - p2).value - AngleRepresentation(p1 - p2).value) % 4.0 <= 2.0:
            # p2 would be considered to be an extremity if the numbering was correct
            if result:
                positive_tests += 1
                if positive_tests >= threshold:
                    return True
        elif not result:
            positive_tests += 1
            if positive_tests >= threshold:
                return True

        p1 = p2
        p2 = p3

    return False


def validate(boundary_coords, list_hole_coords):
    # TODO test
    # TODO possible to allow polygon consisting of 2 vertices only(=barrier)? lots of algorithms need at least 3 vertices
    """
    ensure that all the following conditions on the polygons are fulfilled:
        - must at least contain 3 vertices
        - no consequent vertices with identical coordinates in the polygons! In general might have the same coordinates
        - a polygon must not have self intersections (intersections with other polygons are allowed)
        - edge numbering has to follow this convention (for easier computations):
            * outer boundary polygon: counter clockwise
            * holes: clockwise
    :param boundary_coords:
    :param list_hole_coords:
    :return:
    """

    assert boundary_coords.shape[0] >= 3
    assert boundary_coords.shape[1] == 2
    assert no_identical_consequent_vertices(boundary_coords)
    assert no_self_intersection(boundary_coords)
    assert not has_clockwise_numbering(boundary_coords)

    for hole_coords in list_hole_coords:
        assert hole_coords.shape[0] >= 3
        assert hole_coords.shape[1] == 2
        assert no_identical_consequent_vertices(hole_coords)
        assert no_self_intersection(hole_coords)
        assert has_clockwise_numbering(hole_coords)

    # TODO rectification
