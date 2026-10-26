import itertools

import matplotlib.pyplot as plt
import numpy as np
import pytest

from extremitypathfinder import utils
from extremitypathfinder.extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlottingEnvironment, set_limits
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


def try_test_cases(environment, test_cases):
    if PLOT_TEST_RESULTS:
        environment = PlottingEnvironment(
            environment, plotting_dir=TEST_PLOT_OUTPUT_FOLDER
        )

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
    grid_env = PolygonEnvironment.from_grid_world(
        *GRID_ENV_PARAMS, simplify=False, validate=False
    )
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

    # even after many queries the internal graph should have the same structure as before
    # otherwise the temporarily added vertices during a query stay stored
    nr_graph_nodes = len(grid_env.graph.nodes)
    assert (
        nr_graph_nodes == 16
    ), "the graph should stay unchanged by shortest path queries!"


def test_poly_env():
    poly_env = PolygonEnvironment(*POLY_ENV_PARAMS, validate=True)
    nr_exp_extremities = 4
    assert (
        len(list(poly_env.all_extremities)) == nr_exp_extremities
    ), f"the environment should detect all {nr_exp_extremities} extremities!"
    nr_nodes_env2 = len(poly_env.graph.nodes)
    assert nr_nodes_env2 == nr_exp_extremities, (
        f"the visibility graph should store all {nr_exp_extremities} extremities {list(poly_env.all_extremities)}!"
        f"\n found: {poly_env.graph.nodes}"
    )

    print("\ntesting polygon environment")
    try_test_cases(poly_env, TEST_DATA_POLY_ENV)


@pytest.mark.parametrize("env_params", POLYGON_ENVS)
def test_graph_links_only_closest_node_per_direction(env_params):
    # when two nodes have the same angle representation there should only be an edge to the closer node
    environment = PolygonEnvironment(*env_params)
    graph = environment.graph
    for node in graph.nodes:
        representations, _ = environment.reprs_n_distances[node]
        neighbour_reprs = [representations[n] for n in graph.neighbors(node)]
        assert len(neighbour_reprs) == len(set(neighbour_reprs))


def test_unbounded_poly_env():
    holes = [[(0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]]

    environment = PolygonEnvironment(None, holes, validate=True)

    assert environment.boundary_polygon is None
    assert environment.within_map(np.array((-100.0, 1.0)))
    assert environment.within_map(np.array((100.0, 1.0)))
    assert not environment.within_map(np.array((1.0, 1.0)))
    path, length = environment.find_shortest_path((-1.0, 0.5), (3.0, 0.5))
    assert path == [(-1.0, 0.5), (0.0, 0.0), (2.0, 0.0), (3.0, 0.5)]
    assert length == pytest.approx(2 * np.sqrt(1.25) + 2)

    figure, axes = plt.subplots()
    set_limits(environment, axes, np.array((path[0], path[-1])))
    assert axes.get_xlim() == pytest.approx((-2.0, 4.0))
    plt.close(figure)


def test_poly_env_without_boundary_or_holes():
    with pytest.raises(ValueError, match="At least one hole is required"):
        PolygonEnvironment(None, [], validate=True)


@pytest.mark.parametrize(
    ("holes", "message"),
    [
        ([[]], "at least contain 3 vertices"),
        (
            [[(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]],
            "must consist of two values",
        ),
    ],
)
def test_poly_env_without_boundary_validates_holes_before_inference(holes, message):
    with pytest.raises(TypeError, match=message):
        PolygonEnvironment(None, holes, validate=True)


def test_overlapping_polygon():
    overlap_poly_env = PolygonEnvironment(*OVERLAP_POLY_ENV_PARAMS)
    print("\ntesting polygon environment with overlapping polygons")
    try_test_cases(overlap_poly_env, TEST_DATA_OVERLAP_POLY_ENV)


def test_separated_environment():
    env = PolygonEnvironment(*SEPARATED_ENV)
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
    env = PolygonEnvironment(*env_data)
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
    env = PolygonEnvironment(*env_data)
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


def test_plotting_environment_wraps_environment(tmp_path):
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    plotting_env = PlottingEnvironment(environment, plotting_dir=str(tmp_path))
    assert plotting_env.environment is environment
    nr_plots = len(list(tmp_path.iterdir()))
    assert (
        nr_plots == 2
    ), "the input polygons and the visibility graph should be plotted"

    # a query without direct path, the search graph is plotted as well
    (start_coordinates, goal_coordinates), _ = next(
        case for case in TEST_DATA_POLY_ENV if len(case[1][0]) > 2
    )
    path, length = plotting_env.find_shortest_path(start_coordinates, goal_coordinates)
    assert (path, length) == environment.find_shortest_path(
        start_coordinates, goal_coordinates
    )
    assert len(list(tmp_path.iterdir())) > nr_plots, "the path should be plotted"
    assert environment.temp_graph is None
