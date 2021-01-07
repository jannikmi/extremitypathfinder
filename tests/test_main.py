import unittest
from math import sqrt

import pytest

from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment

# TODO
# PLOT_TEST_RESULTS = True
PLOT_TEST_RESULTS = False
TEST_PLOT_OUTPUT_FOLDER = 'plots'

if PLOT_TEST_RESULTS:
    print('plotting test environment enabled.')
    ENVIRONMENT_CLASS = PlottingEnvironment
    CONSTRUCTION_KWARGS = {"plotting_dir": TEST_PLOT_OUTPUT_FOLDER}
else:
    ENVIRONMENT_CLASS = PolygonEnvironment
    CONSTRUCTION_KWARGS = {}

# size_x, size_y, obstacle_iter
GRID_ENV_PARAMS = (19, 10, [
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
])

INVALID_DESTINATION_DATA = [
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
]

TEST_DATA_GRID_ENV = [
    # ((start,goal),(path,distance))
    # shortest paths should be distinct (reverse will automatically be tested)

    # identical nodes
    (((15, 5), (15, 5)), ([(15, 5), (15, 5)], 0.0)),

    # directly reachable
    (((15, 5), (15, 6)), ([(15, 5), (15, 6)], 1.0)),
    (((15, 5), (16, 6)), ([(15, 5), (16, 6)], sqrt(2))),

    # points on the polygon edges (vertices) should be accepted!
    # on edge
    (((15, 0), (15, 6)), ([(15, 0), (15, 6)], 6.0)),
    (((17, 5), (16, 5)), ([(17, 5), (16, 5)], 1.0)),
    # on edge of hole
    (((7, 8), (7, 9)), ([(7, 8), (7, 9)], 1.0)),

    # on vertex
    (((4, 2), (4, 3)), ([(4, 2), (4, 3)], 1.0)),
    # on vertex of hole
    (((6, 8), (6, 9)), ([(6, 8), (6, 9)], 1.0)),

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

    (((0.5, 6), (9, 5.5)),
     ([(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (9.0, 5.5)], 8.727806217396338)),

    (((0.5, 6), (18.5, 9)),
     ([(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (18, 7), (18.5, 9.0)], 19.869364068640845)),

    (((6.9, 4), (7, 9)),
     ([(6.9, 4.0), (7, 6), (8, 7), (8, 8), (7, 9)], 5.830925564196269)),

    (((6.5, 4), (7, 9)),
     ([(6.5, 4.0), (7, 6), (8, 7), (8, 8), (7, 9)], 5.889979937555021)),
]

POLY_ENV_PARAMS = (
    # boundary_coordinates
    [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)],
    # list_of_holes
    [[(3.0, 7.0), (5.0, 9.0), (4.6, 7.0), (5.0, 4.0), ], ],
)

TEST_DATA_POLY_ENV = [
    # ((start,goal),(path,distance))
    # identical nodes
    (((1, 1), (1, 1)), ([(1, 1), (1, 1)], 0.0)),

    # directly reachable
    (((1, 1), (1, 2)), ([(1, 1), (1, 2)], 1.0)),
    (((1, 1), (2, 1)), ([(1, 1), (2, 1)], 1.0)),

    # points on the polygon edges (vertices) should be accepted!
    # on edge (boundary polygon)
    (((1, 0), (1, 1)), ([(1, 0), (1, 1)], 1.0)),
    (((9.5, 2.5), (8.5, 2.5)), ([(9.5, 2.5), (8.5, 2.5)], 1.0)),
    (((0, 2), (0, 1)), ([(0, 2), (0, 1)], 1.0)),  # both
    (((1, 0), (5, 0)), ([(1, 0), (5, 0)], 4.0)),  # both
    # on edge of hole
    (((4, 8), (3, 8)), ([(4, 8), (3, 8)], 1.0)),
    (((4, 8), (4.1, 8.1)), ([(4, 8), (4.1, 8.1)], sqrt(2 * (0.1 ** 2)))),  # both

    # on vertex
    (((9, 5), (8, 5)), ([(9, 5), (8, 5)], 1.0)),
    # on vertex of hole
    (((3, 7), (2, 7)), ([(3, 7), (2, 7)], 1.0)),

    # on two vertices
    # coinciding with edge (direct neighbour)
    (((3, 7), (5, 9)), ([(3, 7), (5, 9)], sqrt(8))),
    (((4.6, 7), (5, 9)), ([(4.6, 7), (5, 9)], sqrt((0.4 ** 2) + (2 ** 2)))),
    # should have direct connection to all visible extremities! connected in graph
    (((5, 4), (5, 9)), ([(5, 4), (5, 9)], 5)),
    # should have a direct connection to all visible extremities! even if not connected in graph!
    (((9, 5), (5, 9)), ([(9, 5), (5, 9)], sqrt(2 * (4 ** 2)))),

    # using a* graph search:
    # directly reachable through a single vertex (does not change distance!)
    (((9, 4), (9, 6)), ([(9, 4), (9, 5), (9, 6)], 2)),
]

OVERLAP_POLY_ENV_PARAMS = (
    # boundary_coordinates
    [(9.5, 10.5), (25.5, 10.5), (25.5, 0.5), (49.5, 0.5), (49.5, 49.5), (0.5, 49.5), (0.5, 16.5), (9.5, 16.5),
     (9.5, 45.5), (15.5, 45.5), (15.5, 30.5), (35.5, 30.5), (35.5, 14.5), (0.5, 14.5), (0.5, 0.5), (9.5, 0.5)],
    # list_of_holes
    [[(40.5, 4.5), (29.5, 4.5), (29.5, 15.0), (40.5, 15.0), ],
     [(45.40990195143968, 14.5), (44.59009804856032, 14.5), (43.39009804883972, 20.5), (46.60990195116028, 20.5), ],
     [(40.5, 34.5), (24.5, 34.5), (24.5, 40.5), (40.5, 40.5), ],
     [(31.5, 5.390098048839718), (31.5, 10.909901951439679), (42.5, 13.109901951160282), (42.5, 7.590098048560321), ],
     ],
)

TEST_DATA_OVERLAP_POLY_ENV = [
    # ((start,goal),(path,distance))
    (((1, 1), (5, 20)), ([(1, 1), (9.5, 10.5), (25.5, 10.5), (29.5, 4.5), (40.5, 4.5), (42.5, 7.590098048560321),
                          (42.5, 13.109901951160282), (35.5, 30.5), (24.5, 34.5), (15.5, 45.5), (9.5, 45.5), (5, 20)],
                         132.71677685197986)),
]

SEPARATED_ENV = (
    [(5, 5), (-5, 5), (-5, -5), (5, -5)],
    [[(-5.1, 1), (-5.1, 2), (5.1, 2), (5.1, 1)]],  # intersecting polygons -> no path possible
    # [[(-5, 1), (-5, 2), (5, 2), (5, 1)]], # hole lies on the edges -> path possible
)

TEST_DATA_SEPARATE_ENV = [
    # ((start,goal),(path,distance))
    (((0, 0), (0, 4)), ([], None)),  # unreachable
]


# ((start,goal),(path,distance))


def try_test_cases(environment, test_cases):
    def validate(start_coordinates, goal_coordinates, expected_output):
        output = environment.find_shortest_path(start_coordinates, goal_coordinates)
        path, length = output
        assert type(path) is list
        expected_path, expected_length = expected_output
        if expected_length is None:
            correct_result = length is None and path == expected_path
        else:
            correct_result = path == expected_path and length == pytest.approx(expected_length)
        if correct_result:
            status_str = 'OK'
        else:
            status_str = 'XX'
        print(f'{status_str} input: {(start_coordinates, goal_coordinates)} ')
        if PLOT_TEST_RESULTS:
            assert correct_result, \
                f'unexpected result (path, length): got {output} instead of {expected_output} '

    print('testing if path and distance are correct:')
    for ((start_coordinates, goal_coordinates), expected_output) in test_cases:
        validate(start_coordinates, goal_coordinates, expected_output)
        # automatically test reversed!
        path, length = expected_output
        expected_output_reversed = list(reversed(path)), length
        validate(goal_coordinates, start_coordinates, expected_output_reversed)


class MainTest(unittest.TestCase):

    def test_fct(self):
        grid_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)

        grid_env.store_grid_world(*GRID_ENV_PARAMS, simplify=False, validate=False)
        assert len(list(grid_env.all_extremities)) == 17, 'extremities do not get detected correctly!'
        grid_env.prepare()
        # raise ValueError
        assert len(grid_env.graph.all_nodes) == 16, 'identical nodes should get joined in the graph!'

        # test if points outside the map are being rejected
        for start_coordinates, goal_coordinates in INVALID_DESTINATION_DATA:
            with pytest.raises(ValueError):
                grid_env.find_shortest_path(start_coordinates, goal_coordinates)

        print('testing grid environment')
        try_test_cases(grid_env, TEST_DATA_GRID_ENV)

        # when the deep copy mechanism works correctly
        # even after many queries the internal graph should have the same structure as before
        # otherwise the temporarily added vertices during a query stay stored
        assert len(grid_env.graph.all_nodes) == 16, 'the graph should stay unchanged by shortest path queries!'

        nr_nodes_env1_old = len(grid_env.graph.all_nodes)

        poly_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
        poly_env.store(*POLY_ENV_PARAMS, validate=True)
        NR_EXTR_POLY_ENV = 4
        assert len(list(poly_env.all_extremities)) == NR_EXTR_POLY_ENV, \
            f'the environment should detect all {NR_EXTR_POLY_ENV} extremities!'
        poly_env.prepare()
        nr_nodes_env2 = len(poly_env.graph.all_nodes)
        assert nr_nodes_env2 == NR_EXTR_POLY_ENV, \
            f'the visibility graph should store all {NR_EXTR_POLY_ENV} extremities {list(poly_env.all_extremities)}!' \
            f'\n found: {poly_env.graph.all_nodes}'

        nr_nodes_env1_new = len(grid_env.graph.all_nodes)
        assert nr_nodes_env1_new == nr_nodes_env1_old, \
            'node amount of an grid_env should not change by creating another grid_env!'
        assert grid_env.graph is not poly_env.graph, \
            'different environments share the same graph object'
        assert grid_env.graph.all_nodes is not poly_env.graph.all_nodes, \
            'different environments share the same set of nodes'

        print('\ntesting polygon environment')
        try_test_cases(poly_env, TEST_DATA_POLY_ENV)

        # TODO test: When the paths should be blocked, use a single polygon with multiple identical
        #   vertices instead (also allowed?! change data requirements in doc!).

        # TODO test graph construction
        # when two nodes have the same angle representation there should only be an edge to the closer node!
        # test if property 1 is being properly exploited
        # (extremities lying in front of each other need not be connected)

    def test_overlapping_polygon(self):
        overlap_poly_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
        overlap_poly_env.store(*OVERLAP_POLY_ENV_PARAMS)
        overlap_poly_env.prepare()
        print('\ntesting polygon environment with overlapping polygons')
        try_test_cases(overlap_poly_env, TEST_DATA_OVERLAP_POLY_ENV)

    def test_separated_environment(self):
        env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
        env.store(*SEPARATED_ENV)
        env.prepare()
        print('\ntesting polygon environment with two separated areas')
        try_test_cases(env, TEST_DATA_SEPARATE_ENV)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(MainTest)
    unittest.TextTestRunner(verbosity=2).run(suite)
    unittest.main()
