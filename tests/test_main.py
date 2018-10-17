import unittest
from math import sqrt

import pytest

from extremitypathfinder.extremitypathfinder import PolygonEnvironment


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

        environment.store_grid_world(size_x, size_y, obstacle_iter, simplify=False, validate=False)
        assert len(environment.all_extremities) == 17  # should detect all extremities
        environment.prepare()

        assert len(environment.graph.all_nodes) == 16  # identical nodes should get joined in the graph!

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
                environment.find_shortest_path(start_coordinates, goal_coordinates)

        for ((start_coordinates, goal_coordinates), expected_output) in [
            # ((start,goal),(path,distance))
            # identical nodes
            (((15, 5), (15, 5)), ([(15, 5), (15, 5)], 0.0)),

            # directly reachable
            (((15, 5), (15, 6)), ([(15, 5), (15, 6)], 1.0)),
            (((15, 6), (15, 5)), ([(15, 6), (15, 5)], 1.0)),
            (((15, 5), (16, 6)), ([(15, 5), (16, 6)], sqrt(2))),
            (((16, 6), (15, 5)), ([(16, 6), (15, 5)], sqrt(2))),

            # points on the polygon edges (vertices) should be accepted!
            # on edge
            (((15, 0), (15, 6)), ([(15, 0), (15, 6)], 6.0)),
            (((15, 6), (15, 0)), ([(15, 6), (15, 0)], 6.0)),
            (((17, 5), (16, 5)), ([(17, 5), (16, 5)], 1.0)),
            (((16, 5), (17, 5)), ([(16, 5), (17, 5)], 1.0)),
            # on edge of hole
            (((7, 8), (7, 9)), ([(7, 8), (7, 9)], 1.0)),
            (((7, 9), (7, 8)), ([(7, 9), (7, 8)], 1.0)),

            # on vertex
            (((4, 2), (4, 3)), ([(4, 2), (4, 3)], 1.0)),
            (((4, 3), (4, 2)), ([(4, 3), (4, 2)], 1.0)),
            # on vertex of hole
            (((6, 8), (6, 9)), ([(6, 8), (6, 9)], 1.0)),
            (((6, 9), (6, 8)), ([(6, 9), (6, 8)], 1.0)),

            # on two vertices
            # coinciding with edge (direct neighbour)
            (((4, 2), (4, 1)), ([(4, 2), (4, 1)], 1.0)),
            (((5, 5), (5, 7)), ([(5, 5), (5, 7)], 2.0)),
            # should have direct connection to all visible extremities! connected in graph
            (((6, 8), (5, 7)), ([(6, 8), (5, 7)], sqrt(2))),
            (((4, 1), (5, 7)), ([(4, 1), (5, 7)], sqrt(1 ** 2 + 6 ** 2))),
            # should have direct connection to all visible extremities! even if not connected in graph!
            (((4, 2), (5, 7)), ([(4, 2), (5, 7)], sqrt(1 ** 2 + 5 ** 2))),

            # mix of edges and vertices, directly visible
            (((2, 2), (5, 7)), ([(2, 2), (5, 7)], sqrt(3 ** 2 + 5 ** 2))),

            # also regular points should have direct connection to all visible extremities!
            (((10, 3), (17, 6)), ([(10, 3), (17, 6)], sqrt(7 ** 2 + 3 ** 2))),
            (((10, 3), (8, 8)), ([(10, 3), (8, 8)], sqrt(2 ** 2 + 5 ** 2))),
            # even if the query point lies in front of an extremity! (test if new query vertices are being created!)
            (((10, 3), (8, 5)), ([(10, 3), (8, 5)], sqrt(2 ** 2 + 2 ** 2))),

            # using a* graph search:
            # directly reachable through a single vertex (does not change distance!)
            (((5, 1), (3, 3)), ([(5, 1), (4, 2), (3, 3)], sqrt(2 ** 2 + 2 ** 2))),

            # If two Polygons have vertices with identical coordinates (this is allowed),
            #   paths through these vertices are theoretically possible!
            (((6.5, 5.5), (7.5, 6.5)), ([(6.5, 5.5), (7, 6), (7.5, 6.5)], sqrt(1 ** 2 + 1 ** 2))),

            # distance should stay the same even if multiple extremities lie on direct path
            # test if path is skipping passed extremities
            (((8, 4), (8, 8)), ([(8, 4), (8, 5), (8, 6), (8, 7), (8, 8)], 4)),
            (((8, 4), (8, 9)), ([(8, 4), (8, 5), (8, 6), (8, 7), (8, 8), (8, 9)], 5)),

            # regular examples
            (((0.5, 6), (18.5, 0.5)),
             ([(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (17, 6), (18, 6), (18.5, 0.5)], 23.18783787537749)),

            (((0.5, 6), (9, 6)),
             ([(0.5, 6.0), (5, 5), (6, 5), (7, 6), (8, 6), (9, 6)], 9.023985791019538)),

            (((0.5, 6), (18.5, 9)),
             ([(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (18, 7), (18.5, 9.0)], 19.869364068640845)),
            # 19.86936406864084

            (((6.9, 4), (7, 9)),
             ([(6.9, 4.0), (7, 6), (8, 7), (8, 8), (7, 9)], 5.830925564196269)),

            (((6.5, 4), (7, 9)),
             ([(6.5, 4.0), (7, 6), (8, 7), (8, 8), (7, 9)], 5.889979937555021)),
        ]:

            # test if path and distance are correct
            # print(input, expected_output, fct(input))
            actual_output = environment.find_shortest_path(start_coordinates, goal_coordinates)

            if actual_output != expected_output:
                print('input: {} expected: {} got: {}'.format((start_coordinates, goal_coordinates), expected_output,
                                                              actual_output))
            assert actual_output == expected_output

        # when the deep copy mechanism works correctly
        # even after many queries the internal graph should have the same structure as before
        # otherwise the temporarily added vertices during a query stay stored
        assert len(environment.graph.all_nodes) == 16

        # TODO test: When the paths should be blocked, use a single polygon with multiple identical
        #   vertices instead (also allowed).

        # TODO test graph construction
        # when two nodes have the same angle representation there should only be an edge to the closer node!
        # test if property 1 is being properly exploited
        # (extremities lying in front of each other need not be connected)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(MainTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    unittest.main()
