import itertools

import pytest

from extremitypathfinder import utils
from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment
from tests.helpers import other_edge_intersects
from tests.test_cases import (
    GRID_ENV_PARAMS,
    INVALID_DESTINATION_DATA,
    OVERLAP_POLY_ENV_PARAMS,
    POLY_ENV_PARAMS,
    POLYGON_ENVS,
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
        output = environment.find_shortest_path(
            start_coordinates, goal_coordinates, verify=True
        )
        path, length = output
        assert isinstance(path, list), "path should be a list"
        expected_path, expected_length = expected_output
        if expected_length is None:
            correct_result = length is None and path == expected_path
        else:
            correct_result = path == expected_path and length == pytest.approx(
                expected_length
            )
        if correct_result:
            status_str = "OK"
        else:
            status_str = "XX"
        print(f"{status_str} input: {(start_coordinates, goal_coordinates)} ")
        assert correct_result, f"unexpected result (path, length): got {output} instead of {expected_output} "

    print("testing if path and distance are correct:")
    for (start_coordinates, goal_coordinates), expected_output in test_cases:
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
    # nr_graph_nodes = len(grid_env.graph.nodes)
    # TODO
    # assert nr_graph_nodes == 16, "the graph should stay unchanged by shortest path queries!"
    # nr_nodes_env1_old = len(grid_env.graph.nodes)


def test_poly_env():
    poly_env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
    poly_env.store(*POLY_ENV_PARAMS, validate=True)
    nr_exp_extremities = 4
    assert (
        len(list(poly_env.all_extremities)) == nr_exp_extremities
    ), f"the environment should detect all {nr_exp_extremities} extremities!"
    # nr_nodes_env2 = len(poly_env.graph.nodes)
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
    print("\ntesting polygon environment with overlapping polygons")
    try_test_cases(overlap_poly_env, TEST_DATA_OVERLAP_POLY_ENV)


def test_separated_environment():
    env = ENVIRONMENT_CLASS(**CONSTRUCTION_KWARGS)
    env.store(*SEPARATED_ENV)
    print("\ntesting polygon environment with two separated areas")
    try_test_cases(env, TEST_DATA_SEPARATE_ENV)


@pytest.mark.parametrize(
    "env_data",
    POLYGON_ENVS,
)
def test_extremity_neighbour_connection(env_data):
    # if two extremities are direct neighbours in a polygon, they also must be connected in the prepared graph
    # exception: there is another polygon edge intersecting that
    print("\ntesting if all two direct extremity neighbour are connected")
    env = PolygonEnvironment()
    env.store(*env_data)
    coords = env.coords
    graph = env.graph
    extremities = env.extremity_indices
    edge_vertex_idxs = env.edge_vertex_idxs

    def connection_as_expected(i1: int, i2: int):
        if i2 not in extremities:
            return
        should_be_connected = not other_edge_intersects(
            i1, i2, edge_vertex_idxs, coords
        )
        graph_neighbors_e = set(graph.neighbors(i1))
        are_connected = i2 in graph_neighbors_e
        assert should_be_connected == are_connected

    for e in extremities:
        n1, n2 = utils.get_neighbour_idxs(e, env.vertex_edge_idxs, edge_vertex_idxs)
        connection_as_expected(e, n1)
        connection_as_expected(e, n2)


@pytest.mark.parametrize(
    "env_data",
    POLYGON_ENVS,
)
def test_all_coords_work_as_input(env_data):
    # if two extremities are direct neighbours in a polygon, they also must be connected in the prepared graph
    # exception: there is another polygon edge intersecting that
    print("\ntesting if all two direct extremity neighbour are connected")
    env = PolygonEnvironment()
    env.store(*env_data)
    coords = env.coords
    nr_vertices = env.nr_vertices

    for start, goal in itertools.product(range(nr_vertices), repeat=2):
        coords_start = coords[start]
        coords_goal = coords[goal]

        if not env.within_map(coords_start):
            continue
        if not env.within_map(coords_goal):
            continue

        env.find_shortest_path(coords_start, coords_goal, verify=True)
