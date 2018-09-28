import random
import unittest
from math import degrees, radians, sqrt
import pytest

import numpy as np
from six.moves import range

from helpers import *


class HelperTest(unittest.TestCase):

    def proto_test_case(self, data, fct):
        for input, expected_output in data:
            # print(input, expected_output, fct(input))
            actual_output = fct(input)
            if actual_output != expected_output:
                print('input: {} expected: {} got: {}'.format(input, expected_output, actual_output))
            assert actual_output == expected_output

    def test_angle_repr(self):

        with pytest.raises(ValueError):
            AngleRepresentation(np.array([0.0, 0.0]))

        def quadrant_test_fct(input):
            np_2D_coord_vector = np.array(input)
            return AngleRepresentation(np_2D_coord_vector).quadrant

        data = [
            ([1.0, 0.0], 0.0),
            ([0.0, 1.0], 0.0),
            ([-1.0, 0.0], 1.0),
            ([0.0, -1.0], 3.0),

            ([2.0, 0.0], 0.0),
            ([0.0, 2.0], 0.0),
            ([-2.0, 0.0], 1.0),
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
        ]

        self.proto_test_case(data, quadrant_test_fct)

        # TODO test:
        # randomized
        # every quadrant contains angle measures from 0.0 to 1.0
        # angle %360!
        #     rep(p1) > rep(p2) <=> angle(p1) > angle(p2)
        #     rep(p1) = rep(p2) <=> angle(p1) = angle(p2)
        # repr value in [0.0 : 4.0[

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

        self.proto_test_case(data, value_test_fct)

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
                (-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0),
                (0.0, 1),
                (0, -1),
                (1, 0),
                (-1, 0),
            ]
            expected_results = [
                True, False, False, False, False, False, False, False, False,
                # on the line test cases
                border_value, border_value, border_value, border_value,border_value, border_value, border_value, border_value,
            ]

            self.proto_test_case(list(zip(p_test_cases, expected_results)), test_fct)


# TODO test if relation is really bidirectional (find_visible(x,y) = find_visible(y,x))


if __name__ == '__main__':
    # suite = unittest.TestLoader().loadTestsFromTestCase(HelperTest)
    # unittest.TextTestRunner(verbosity=2).run(suite)
    unittest.main()
