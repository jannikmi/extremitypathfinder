import unittest
from os.path import abspath, join, pardir
from typing import Dict, Set

import numpy as np
import pytest
from helpers import proto_test_case

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.helper_classes import AngleRepresentation
from extremitypathfinder.helper_fcts import clean_visibles, has_clockwise_numbering, inside_polygon, read_json


# TODO test find_visible(), ...
# TODO test invalid data detection
class HelperFctsTest(unittest.TestCase):
    def value_test_fct(input):
        np_2D_coord_vector = np.array(input)
        return AngleRepresentation(np_2D_coord_vector).value

    data = [
        ([1.0, 0.0], 0.0),
        ([0.0, 1.0], 1.0),
        ([-1.0, 0.0], 2.0),
        ([0.0, -1.0], 3.0),
        ([2.0, 0.0], 0.0),
        ([0.0, 2.0], 1.0),
        ([-2.0, 0.0], 2.0),
        ([0.0, -2.0], 3.0),
    ]

    proto_test_case(data, value_test_fct)

    def test_inside_polygon(self):
        # TODO more detailed test. edges with slopes... also in timezonefinder

        for border_value in [True, False]:

            def test_fct(input):
                polygon_test_case = np.array([(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)])
                x, y = input

                return inside_polygon(x, y, polygon_test_case, border_value)

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

    def test_clockwise_numering(self):
        def clockwise_test_fct(input):
            return has_clockwise_numbering(np.array(input))

        data = [
            # clockwise numbering!
            ([(3.0, 7.0), (5.0, 9.0), (5.0, 7.0)], True),
            ([(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0)], True),
            ([(0.0, 0.0), (0.0, 1.0), (1.0, 0.0)], True),
            # # counter clockwise edge numbering!
            ([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], False),
            ([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], False),
            ([(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)], False),
            ([(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)], False),
        ]
        proto_test_case(data, clockwise_test_fct)

    def test_read_json(self):
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


# TODO test if relation is really bidirectional (y in find_visible(x,y) <=> x in find_visible(y,x))
