from os.path import abspath, join, pardir

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, configs
from extremitypathfinder.utils import (
    _has_clockwise_numbering,
    _inside_polygon,
    read_json,
)
from extremitypathfinder.utils_numba import _compute_repr_n_dist
from tests.helpers import proto_test_case
from tests.test_find_visible import _clean_visibles


def test_inside_polygon():
    polygon_test_case = np.array(
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)], dtype=configs.DTYPE_FLOAT
    )

    for border_value in [True, False]:

        def test_fct(input, border_value=border_value):
            p = np.array(input, dtype=configs.DTYPE_FLOAT)
            return _inside_polygon(p, polygon_test_case, border_value)

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
    PolygonEnvironment(boundary_coordinates, list_of_holes, validate=True)


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
    visible_idxs: set[int],
    cand_idx2repr: dict[int, float],
    vert_idx2dist: dict[int, float],
    expected: set[int],
):
    res = _clean_visibles(visible_idxs, cand_idx2repr, vert_idx2dist)
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
    # the extremities of a single polygon: a boundary (counter clockwise) or a hole (clockwise)
    coords = np.array(coords, dtype=configs.DTYPE_FLOAT)
    if _has_clockwise_numbering(coords):
        environment = PolygonEnvironment(None, [coords], validate=True)
    else:
        environment = PolygonEnvironment(coords, [], validate=True)
    assert set(environment.extremity_indices.tolist()) == expected


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
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        return _has_clockwise_numbering(inp)

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
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        return _compute_repr_n_dist(inp)

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
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        repr, _dist = _compute_repr_n_dist(inp)
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
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        repr, _dist = _compute_repr_n_dist(inp)
        return repr

    res = func(input)
    assert res >= expected
    assert res < expected + 1
