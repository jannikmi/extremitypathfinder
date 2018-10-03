import unittest

import pytest

from extremitypathfinder.extremitypathfinder import PolygonEnvironment, load_pickle
from .test_helpers import proto_test_case


class MainTest(unittest.TestCase):

    def test_fct(self):
        environment = PolygonEnvironment()

        size_x, size_y = 19, 10
        obstacle_iter = [
            # (x,y),

            # obstacles changing boundary
            (0, 1),
            (1, 1),
            (2, 1),
            (3, 1),

            (17, 9),
            (17, 8),
            (17, 7),

            (17, 5),
            (17, 4),
            (17, 3),
            (17, 2),
            (17, 1),
            (17, 0),

            # hole 1
            (5, 5),
            (5, 6),
            (6, 6),
            (6, 7),
            (7, 7),

            # hole 2
            # (6, 4),

        ]
        #
        # size_x, size_y = 5,4
        # obstacle_iter = [
        #     # (x,y),
        #
        #     # obstacles changing boundary
        #     (3, 0),
        #     (3, 1),
        #
        #     # hole 1
        #     (1,2),
        #
        # ]

        environment.store_grid_world(size_x, size_y, obstacle_iter, simplify=False, validate=False, export_plots=False)
        environment.prepare(export_plots=False)

        start_coordinates = (17, 9.0)
        goal_coordinates = (17, 0.5)
        # path, length = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=False)
        # path, length = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=True)
        # print(path)

        # TODO

        # test if path distance is correct

        # should stay the same if extremities lie on direct path

        # test if points outside the map are being rejected
        for start_coordinates, goal_coordinates in [
            # outside of map region
            ((-1, 5.0), (17, 0.5)),
            ((17, 0.5), (-1, 5.0)),
            ((20, 5.0), (17, 0.5)),
            ((17, 0.5), (20, 5.0)),
            ((1, -5.0), (17, 0.5)),
            ((17, 0.5), (1, -5.0)),
            ((1, 11.0), (17, 0.5)),
            ((17, 0.5), (1, 11.0)),

            # outside boundary polygon
            ((17.5, 5.0), (17, 0.5)),
            ((17, 0.5), (17.5, 5.0)),
            ((1, 1.5), (17, 0.5)),
            ((17, 0.5), (1, 1.5)),

            # inside hole
            ((6.5, 6.5), (17, 0.5)),
            ((17, 0.5), (6.5, 6.5)),
        ]:
            with pytest.raises(ValueError):
                print(start_coordinates, goal_coordinates)
                environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=False)

        # points on the polygon edges (vertices) should be accepted!


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(MainTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    # unittest.main()
