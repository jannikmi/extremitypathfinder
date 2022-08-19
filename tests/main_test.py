import pytest

from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment
from tests.test_cases import (
    GRID_ENV_PARAMS,
    INVALID_DESTINATION_DATA,
    OVERLAP_POLY_ENV_PARAMS,
    POLY_ENV_PARAMS,
    SEPARATED_ENV,
    TEST_DATA_GRID_ENV,
    TEST_DATA_OVERLAP_POLY_ENV,
    TEST_DATA_POLY_ENV,
    TEST_DATA_SEPARATE_ENV,
)

# PLOT_TEST_RESULTS = True
PLOT_TEST_RESULTS = False
TEST_PLOT_OUTPUT_FOLDER = "plots"

if PLOT_TEST_RESULTS:
    print("plotting test environment enabled.")
    ENVIRONMENT_CLASS = PlottingEnvironment
    CONSTRUCTION_KWARGS = {"plotting_dir": TEST_PLOT_OUTPUT_FOLDER}
else:
    ENVIRONMENT_CLASS = PolygonEnvironment
    CONSTRUCTION_KWARGS = {}


# TODO pytest parameterize


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
            status_str = "OK"
        else:
            status_str = "XX"
        print(f"{status_str} input: {(start_coordinates, goal_coordinates)} ")
        assert correct_result, f"unexpected result (path, length): got {output} instead of {expected_output} "

    print("testing if path and distance are correct:")
    for ((start_coordinates, goal_coordinates), expected_output) in test_cases:
        validate(start_coordinates, goal_coordinates, expected_output)
        # automatically test reversed!
        path, length = expected_output
        expected_output_reversed = list(reversed(path)), length
        validate(goal_coordinates, start_coordinates, expected_output_reversed)


def test_grid_env():
    grid_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)

    grid_env.store_grid_world(*GRID_ENV_PARAMS, simplify=False, validate=False)
    nr_extremities = len(grid_env.all_extremities)
    assert nr_extremities == 17, "extremities do not get detected correctly!"
    grid_env.prepare()
    nr_graph_nodes = len(grid_env.graph.nodes)
    assert nr_graph_nodes == 16, "identical nodes should get joined in the graph!"

    # test if points outside the map are being rejected
    for start_coordinates, goal_coordinates in INVALID_DESTINATION_DATA:
        with pytest.raises(ValueError):
            grid_env.find_shortest_path(start_coordinates, goal_coordinates)

    print("testing grid environment")
    try_test_cases(grid_env, TEST_DATA_GRID_ENV)

    # when the deep copy mechanism works correctly
    # even after many queries the internal graph should have the same structure as before
    # otherwise the temporarily added vertices during a query stay stored
    nr_graph_nodes = len(grid_env.graph.nodes)
    # TODO
    # assert nr_graph_nodes == 16, "the graph should stay unchanged by shortest path queries!"

    nr_nodes_env1_old = len(grid_env.graph.nodes)


def test_poly_env():
    poly_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
    poly_env.store(*POLY_ENV_PARAMS, validate=True)
    nr_exp_extremities = 4
    assert (
        len(list(poly_env.all_extremities)) == nr_exp_extremities
    ), f"the environment should detect all {nr_exp_extremities} extremities!"
    poly_env.prepare()
    nr_nodes_env2 = len(poly_env.graph.nodes)
    # TODO
    # assert nr_nodes_env2 == nr_exp_extremities, (
    #     f"the visibility graph should store all {nr_exp_extremities} extremities {list(poly_env.all_extremities)}!"
    #     f"\n found: {poly_env.graph.nodes}"
    # )

    # nr_nodes_env1_new = len(grid_env.graph.nodes)
    # assert (
    #     nr_nodes_env1_new == nr_nodes_env1_old
    # ), "node amount of an grid_env should not change by creating another grid_env!"
    # assert grid_env.graph is not poly_env.graph, "different environments share the same graph object"
    # assert (
    #     grid_env.graph.nodes is not poly_env.graph.nodes
    # ), "different environments share the same set of nodes"

    print("\ntesting polygon environment")
    try_test_cases(poly_env, TEST_DATA_POLY_ENV)

    # TODO test: When the paths should be blocked, use a single polygon with multiple identical
    #   vertices instead (also allowed?! change data requirements in doc!).

    # TODO test graph construction
    # when two nodes have the same angle representation there should only be an edge to the closer node!
    # test if property 1 is being properly exploited
    # (extremities lying in front of each other need not be connected)


def test_overlapping_polygon():
    overlap_poly_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
    overlap_poly_env.store(*OVERLAP_POLY_ENV_PARAMS)
    overlap_poly_env.prepare()
    print("\ntesting polygon environment with overlapping polygons")
    try_test_cases(overlap_poly_env, TEST_DATA_OVERLAP_POLY_ENV)


def test_separated_environment():
    env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
    env.store(*SEPARATED_ENV)
    env.prepare()
    print("\ntesting polygon environment with two separated areas")
    try_test_cases(env, TEST_DATA_SEPARATE_ENV)
