import unittest

import numpy as np
import pytest
from helpers import proto_test_case

from extremitypathfinder import helper_classes
from extremitypathfinder.helper_classes import AngleRepresentation, Polygon, Vertex

helper_classes.origin = Vertex((-5.0, -5.0))


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
        # repr value in [0.0 : 4.0]

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

    def test_vertex_translation(self):
        data = [
            ([1.0, 1.0], [6.0, 6.0]),
            ([0.0, 0.0], [5.0, 5.0]),
            ([-12.0, -3.0], [-7.0, 2.0]),
            ([-4.0, 5.0], [1.0, 10.0]),
            ([3.0, -2.0], [8.0, 3.0]),
        ]

        def translation_test_fct(input):
            return Vertex(input).get_coordinates_translated().tolist()

        proto_test_case(data, translation_test_fct)

    def test_vertex_angle_repr(self):
        data = [
            ([0.0, -5.0], 0.0),
            ([-5.0, 0.0], 1.0),
            ([-6.0, -5.0], 2.0),
            ([-5.0, -6.0], 3.0),
        ]

        def angle_repr_test_fct(input):
            return Vertex(input).get_angle_representation()

        proto_test_case(data, angle_repr_test_fct)

    def test_vertex_distance_to_origin(self):
        data = [
            ([0.0, 0.0], np.sqrt(50)),
            ([-5.0, 0.0], 5.0),
            ([0.0, -5], 5.0),
            ([-3.0, -2.0], np.sqrt(13)),
            ([2.0, 5.0], np.sqrt(149)),
        ]

        def dist_to_origin_test_fct(input):
            return Vertex(input).get_distance_to_origin()

        proto_test_case(data, dist_to_origin_test_fct)

    def test_polygon_bad_input(self):
        with pytest.raises(ValueError) as err:
            Polygon([(0, 0), (1, 1)], is_hole=False)
        assert "not a valid polygon" in str(err)

    def test_polygon_extremities(self):
        data = [
            (
                [
                    (0, 0),
                    (10, 0),
                    (9, 5),
                    (10, 10),
                    (0, 10),
                ],
                [(9, 5)],
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
                [(0, -1), (-2, -2)],
            ),
        ]

        def find_extremities_test_fct(input):
            extremities = Polygon(input, is_hole=False).extremities
            return [tuple(e.coordinates) for e in extremities]

        proto_test_case(data, find_extremities_test_fct)


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(HelperClassesTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    # unittest.main()
