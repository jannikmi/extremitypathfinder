import unittest

from math import sqrt
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
            (7, 5),
        ]

        environment.store_grid_world(size_x, size_y, obstacle_iter, simplify=False, validate=False, export_plots=False)
        assert len(environment.all_extremities) == 17  # should detect all extremities
        # environment.prepare(export_plots=False)
        environment.prepare(export_plots=True)

        assert len(environment.graph.all_nodes) == 16  # identical nodes should get joined in the graph!

        # TODO


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
                environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=False)

        for ((start_coordinates, goal_coordinates), expected_output) in [
            # ((start,goal),(path,distance))
            # identical nodes
            (((15, 5), (15, 5)), ([(15, 5), (15, 5)], 0.0)),

            # directly reachable
            (((15, 5), (15, 6)), ([(15, 5), (15, 6)], 1.0)),
            (((15, 6), (15, 5)), ([(15, 6), (15, 5)], 1.0)),
            (((15, 5), (16, 6)), ([(15, 5), (16, 6)], sqrt(2))),
            (((16, 6), (15, 5)), ([(16, 6), (15, 5)], sqrt(2))),
            # on edge
            (((15, 0), (15, 6)), ([(15, 0), (15, 6)], 6.0)),
            (((15, 6), (15, 0)), ([(15, 6), (15, 0)], 6.0)),
            (((17, 5), (16, 5)), ([(17, 5), (16, 5)], 1.0)),
            (((16, 5), (17, 5)), ([(16, 5), (17, 5)], 1.0)),
            # on edge of hole
            (((7, 8), (7, 9)), ([(7, 8), (7, 9)], 1.0)),
            (((7, 9), (7, 8)), ([(7, 9), (7, 8)], 1.0)),

            # directly reachable through a single vertex (does not change distance!)

        ]:
            # test if path and distance are correct
            # print(input, expected_output, fct(input))
            # actual_output = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=True)
            actual_output = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=False)

            if actual_output != expected_output:
                print('input: {} expected: {} got: {}'.format((start_coordinates, goal_coordinates), expected_output,
                                                              actual_output))
            assert actual_output == expected_output


        # even after many queries the internal graph should have the same structure as before
        # when the deep copy mechanism works correctly
        assert len(environment.graph.all_nodes) == 16  # identical nodes should get joined in the graph!


        # points on the polygon edges (vertices) should be accepted!
        # FIXME and have direct connection to all visible extremities!

        # **NOTE**: If two Polygons have vertices with identical coordinates (this is allowed), paths through these vertices are theoretically possible!
        # When the paths should be blocked, use a single polygon with multiple identical vertices instead (also allowed).


        # when two nodes have the same angle representation there should only be an edge to the closer node!


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(MainTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    # unittest.main()
