"""
TODO test find_visible(), ...
TODO test if relation is really bidirectional (y in find_visible(x,y) <=> x in find_visible(y,x))
TODO test input data validation
"""
from os.path import abspath, join, pardir
from typing import Dict, Set

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.utils import (
    clean_visibles,
    compute_extremity_idxs,
    compute_repr_n_dist,
    has_clockwise_numbering,
    inside_polygon,
    read_json,
)
from tests.helpers import proto_test_case


def test_inside_polygon():
    for border_value in [True, False]:

        def test_fct(input):
            polygon_test_case = np.array([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)])
            p = np.array(input, dtype=float)
            return inside_polygon(p, polygon_test_case, border_value)

        p_test_cases = [
            # (x,y),
            # inside
            (0.0, 0.0),
            # # outside
            (-2.0, 2.0),
            (0, 2.0),
            (2.0, 2.0),
            (-2.0, 0),
            (2.0, 0),
            (-2.0, -2.0),
            (0, -2.0),
            (2.0, -2.0),
            # on the line test cases
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
            (0.0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
        ]
        expected_results = [
            True,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            # on the line test cases
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
        ]

        proto_test_case(list(zip(p_test_cases, expected_results)), test_fct)


def test_read_json():
    path2json_file = abspath(join(__file__, pardir, pardir, "example.json"))
    boundary_coordinates, list_of_holes = read_json(path2json_file)
    assert len(boundary_coordinates) == 5
    assert len(boundary_coordinates[0]) == 2
    assert len(list_of_holes) == 2
    first_hole = list_of_holes[0]
    assert len(first_hole) == 4
    assert len(first_hole[0]) == 2
    environment = PolygonEnvironment()
    environment.store(boundary_coordinates, list_of_holes, validate=True)


@pytest.mark.parametrize(
    "visible_idxs, cand_idx2repr, vert_idx2dist, expected",
    [
        (set(), {}, {}, set()),
        ({0}, {0: 0.0}, {0: 0.0}, {0}),
        # different repr -> keep both
        ({0, 1}, {0: 0.0, 1: 1.0}, {0: 0.0, 1: 0.0}, {0, 1}),
        ({0, 1}, {0: 0.5, 1: 1.0}, {0: 0.0, 1: 1.0}, {0, 1}),
        ({0, 1}, {0: 0.5, 1: 1.0}, {0: 1.0, 1: 1.0}, {0, 1}),
        # same repr -> keep one the one with the lower dist
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 0.0, 1: 1.0}, {0}),
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 0.0, 1: 1.1}, {0}),
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 1.0, 1: 0.0}, {1}),
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 1.1, 1: 0.0}, {1}),
    ],
)
def test_clean_visible_idxs(
    visible_idxs: Set[int], cand_idx2repr: Dict[int, float], vert_idx2dist: Dict[int, float], expected: Set[int]
):
    res = clean_visibles(visible_idxs, cand_idx2repr, vert_idx2dist)
    assert res == expected


@pytest.mark.parametrize(
    "coords, expected",
    [
        (
            [
                (0, 0),
                (10, 0),
                (9, 5),
                (10, 10),
                (0, 10),
            ],
            {2},
        ),
        (
            [
                (0, 0),
                (10, 0),
                (10, 10),
                (0, 10),
            ],
            set(),
        ),
        (
            [
                (0, 0),
                (-2, -2),
                (-3, -2.5),
                (-3, -4),
                (2, -3),
                (1, 2.5),
                (0, -1),
            ],
            {6, 1},
        ),
    ],
)
def test_compute_extremity_idxs(coords, expected):
    coords = np.array(coords)
    res = compute_extremity_idxs(coords)
    assert set(res) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        # clockwise numbering!
        ([(3.0, 7.0), (5.0, 9.0), (5.0, 7.0)], True),
        ([(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0)], True),
        ([(0.0, 0.0), (0.0, 1.0), (1.0, 0.0)], True),
        # # counter clockwise edge numbering!
        ([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], False),
        ([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], False),
        ([(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)], False),
        ([(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)], False),
    ],
)
def test_clockwise_numering(input, expected):
    def clockwise_test_fct(input):
        return has_clockwise_numbering(np.array(input))

    assert clockwise_test_fct(input) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        ([0.0, -5.0], (3.0, 5.0)),
        ([-5.0, 0.0], (2.0, 5.0)),
        ([-1.0, 0.0], (2.0, 1.0)),
        ([1.0, 0.0], (0.0, 1.0)),
        ([0.0, 1.0], (1.0, 1.0)),
        ([-6.0, -5.0], (2.64018439966448, 7.810249675906654)),
        ([-5.0, -6.0], (2.768221279597376, 7.810249675906654)),
    ],
)
def test_compute_repr_n_dist(input, expected):
    def test_fct(input):
        return compute_repr_n_dist(np.array(input))

    assert test_fct(input) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        ([1.0, 0.0], 0.0),
        ([0.0, 1.0], 1.0),
        ([-1.0, 0.0], 2.0),
        ([0.0, -1.0], 3.0),
        ([2.0, 0.0], 0.0),
        ([0.0, 2.0], 1.0),
        ([-2.0, 0.0], 2.0),
        ([0.0, -2.0], 3.0),
    ],
)
def test_angle_representation(input, expected):
    def func(input):
        repr, dist = compute_repr_n_dist(np.array(input))
        return repr

    assert func(input) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        ([1.0, 0.0], 0.0),
        ([0.0, 1.0], 1.0),
        ([-1.0, 0.0], 2.0),
        ([0.0, -1.0], 3.0),
        ([2.0, 0.0], 0.0),
        ([0.0, 2.0], 1.0),
        ([-2.0, 0.0], 2.0),
        ([0.0, -2.0], 3.0),
        ([1.0, 1.0], 0.0),
        ([-1.0, 1.0], 1.0),
        ([-1.0, -1.0], 2.0),
        ([1.0, -1.0], 3.0),
        ([1.0, 0.00001], 0.0),
        ([0.00001, 1.0], 0.0),
        ([-1.0, 0.00001], 1.0),
        ([0.00001, -1.0], 3.0),
        ([1.0, -0.00001], 3.0),
        ([-0.00001, 1.0], 1.0),
        ([-1.0, -0.00001], 2.0),
        ([-0.00001, -1.0], 2.0),
    ],
)
def test_angle_repr_quadrant(input, expected):
    def func(input):
        repr, dist = compute_repr_n_dist(np.array(input))
        return repr

    res = func(input)
    assert res >= expected
    assert res < expected + 1
