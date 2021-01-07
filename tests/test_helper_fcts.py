import unittest

import numpy as np

from extremitypathfinder.helper_classes import AngleRepresentation
from extremitypathfinder.helper_fcts import has_clockwise_numbering, inside_polygon
from helpers import proto_test_case


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
                (-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0),
                (0.0, 1),
                (0, -1),
                (1, 0),
                (-1, 0),
            ]
            expected_results = [
                True, False, False, False, False, False, False, False, False,
                # on the line test cases
                border_value, border_value, border_value, border_value, border_value, border_value, border_value,
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


# TODO test if relation is really bidirectional (y in find_visible(x,y) <=> x in find_visible(y,x))


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(HelperFctsTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    # unittest.main()
