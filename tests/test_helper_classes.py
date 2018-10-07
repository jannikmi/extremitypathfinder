import unittest

import numpy as np
import pytest

from extremitypathfinder.helper_classes import AngleRepresentation
from test_helpers import proto_test_case


class HelperClassesTest(unittest.TestCase):

    def test_angle_repr(self):
        with pytest.raises(ValueError):
            AngleRepresentation(np.array([0.0, 0.0]))

        #
        # def quadrant_test_fct(input):
        #     np_2D_coord_vector = np.array(input)
        #     return AngleRepresentation(np_2D_coord_vector).quadrant
        #
        # data = [
        #     ([1.0, 0.0], 0.0),
        #     ([0.0, 1.0], 0.0),
        #     ([-1.0, 0.0], 1.0),
        #     ([0.0, -1.0], 3.0),
        #
        #     ([2.0, 0.0], 0.0),
        #     ([0.0, 2.0], 0.0),
        #     ([-2.0, 0.0], 1.0),
        #     ([0.0, -2.0], 3.0),
        #
        #     ([1.0, 1.0], 0.0),
        #     ([-1.0, 1.0], 1.0),
        #     ([-1.0, -1.0], 2.0),
        #     ([1.0, -1.0], 3.0),
        #
        #     ([1.0, 0.00001], 0.0),
        #     ([0.00001, 1.0], 0.0),
        #     ([-1.0, 0.00001], 1.0),
        #     ([0.00001, -1.0], 3.0),
        #
        #     ([1.0, -0.00001], 3.0),
        #     ([-0.00001, 1.0], 1.0),
        #     ([-1.0, -0.00001], 2.0),
        #     ([-0.00001, -1.0], 2.0),
        # ]
        #
        # proto_test_case(data, quadrant_test_fct)

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

        proto_test_case(data, value_test_fct)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(HelperClassesTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    # unittest.main()
