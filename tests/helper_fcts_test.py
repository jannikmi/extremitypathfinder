from os.path import abspath, join, pardir

import networkx as nx
import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, configs
from extremitypathfinder.utils import (
    _has_clockwise_numbering,
    _inside_polygon,
    astar_path,
    polygon_bounds,
    read_json,
)
from extremitypathfinder.utils_numba import _compute_repr_n_dist, is_within_map
from tests.helpers import proto_test_case
from tests.test_find_visible import _clean_visibles


def test_inside_polygon():
    polygon_test_case = np.array(
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)], dtype=configs.DTYPE_FLOAT
    )

    for border_value in [True, False]:

        def test_fct(input, border_value=border_value):
            p = np.array(input, dtype=configs.DTYPE_FLOAT)
            return _inside_polygon(p, polygon_test_case, border_value)

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
            (-1.0, -1.0),
            (1.0, -1.0),
            (1.0, 1.0),
            (-1.0, 1.0),
            (0.0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
        ]
        expected_results = [
            True,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            False,
            # on the line test cases
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
            border_value,
        ]

        proto_test_case(list(zip(p_test_cases, expected_results)), test_fct)


@pytest.mark.parametrize("seed", range(5))
def test_is_within_map_with_hole_bounds(seed):
    """testing only the holes whose bounding box contains a point gives the same results"""
    rng = np.random.default_rng(seed)
    boundary = np.array(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)], dtype=configs.DTYPE_FLOAT
    )
    triangle = np.array([(0.0, 0.0), (2.0, 3.0), (3.0, 0.0)], dtype=configs.DTYPE_FLOAT)
    # integer offsets: holes touching each other in vertices and along edges
    holes = [
        triangle + rng.integers(0, 16, size=2).astype(configs.DTYPE_FLOAT)
        for _ in range(15)
    ]
    bounds = polygon_bounds(holes)
    points = [rng.uniform(-1.0, 21.0, size=2) for _ in range(300)]
    for hole in holes:
        # vertices, edge midpoints and the corners of the bounding box
        points.extend(hole)
        points.extend((hole + np.roll(hole, 1, axis=0)) / 2.0)
        lower, upper = hole.min(axis=0), hole.max(axis=0)
        points.extend([lower, upper, (lower[0], upper[1]), (upper[0], lower[1])])
    for point in points:
        point = np.asarray(point, dtype=configs.DTYPE_FLOAT)
        for map_boundary in (boundary, None):
            expected = is_within_map(point, map_boundary, holes)
            assert is_within_map(point, map_boundary, holes, bounds) == expected


def test_polygon_bounds_without_polygons():
    minima, maxima = polygon_bounds([])
    assert minima.shape == maxima.shape == (0, 2)
    assert is_within_map(np.array((1.0, 1.0)), None, [], (minima, maxima))


def test_read_json():
    path2json_file = abspath(join(__file__, pardir, pardir, "example.json"))
    boundary_coordinates, list_of_holes = read_json(path2json_file)
    assert len(boundary_coordinates) == 5
    assert len(boundary_coordinates[0]) == 2
    assert len(list_of_holes) == 2
    first_hole = list_of_holes[0]
    assert len(first_hole) == 4
    assert len(first_hole[0]) == 2
    PolygonEnvironment(boundary_coordinates, list_of_holes, validate=True)


@pytest.mark.parametrize(
    "visible_idxs, cand_idx2repr, vert_idx2dist, expected",
    [
        (set(), {}, {}, set()),
        ({0}, {0: 0.0}, {0: 0.0}, {0}),
        # different repr -> keep both
        ({0, 1}, {0: 0.0, 1: 1.0}, {0: 0.0, 1: 0.0}, {0, 1}),
        ({0, 1}, {0: 0.5, 1: 1.0}, {0: 0.0, 1: 1.0}, {0, 1}),
        ({0, 1}, {0: 0.5, 1: 1.0}, {0: 1.0, 1: 1.0}, {0, 1}),
        # same repr -> keep one the one with the lower dist
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 0.0, 1: 1.0}, {0}),
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 0.0, 1: 1.1}, {0}),
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 1.0, 1: 0.0}, {1}),
        ({0, 1}, {0: 0.0, 1: 0.0}, {0: 1.1, 1: 0.0}, {1}),
    ],
)
def test_clean_visible_idxs(
    visible_idxs: set[int],
    cand_idx2repr: dict[int, float],
    vert_idx2dist: dict[int, float],
    expected: set[int],
):
    res = _clean_visibles(visible_idxs, cand_idx2repr, vert_idx2dist)
    assert res == expected


@pytest.mark.parametrize(
    "coords, expected",
    [
        (
            [
                (0, 0),
                (10, 0),
                (9, 5),
                (10, 10),
                (0, 10),
            ],
            {2},
        ),
        (
            [
                (0, 0),
                (10, 0),
                (10, 10),
                (0, 10),
            ],
            set(),
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
            {6, 1},
        ),
    ],
)
def test_compute_extremity_idxs(coords, expected):
    # the extremities of a single polygon: a boundary (counter clockwise) or a hole (clockwise)
    coords = np.array(coords, dtype=configs.DTYPE_FLOAT)
    if _has_clockwise_numbering(coords):
        environment = PolygonEnvironment(None, [coords], validate=True)
    else:
        environment = PolygonEnvironment(coords, [], validate=True)
    assert set(environment.extremity_indices.tolist()) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        # clockwise numbering!
        ([(3.0, 7.0), (5.0, 9.0), (5.0, 7.0)], True),
        ([(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0)], True),
        ([(0.0, 0.0), (0.0, 1.0), (1.0, 0.0)], True),
        # # counter clockwise edge numbering!
        ([(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)], False),
        ([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)], False),
        ([(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (10.0, 10.0), (0.0, 10.0)], False),
        ([(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)], False),
    ],
)
def test_clockwise_numering(input, expected):
    def clockwise_test_fct(input):
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        return _has_clockwise_numbering(inp)

    assert clockwise_test_fct(input) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        ([0.0, -5.0], (3.0, 5.0)),
        ([-5.0, 0.0], (2.0, 5.0)),
        ([-1.0, 0.0], (2.0, 1.0)),
        ([1.0, 0.0], (0.0, 1.0)),
        ([0.0, 1.0], (1.0, 1.0)),
        # quadrant 2 + diamond angle 5 / (6 + 5) resp. 6 / (5 + 6)
        ([-6.0, -5.0], (2.0 + 5.0 / 11.0, 7.810249675906654)),
        ([-5.0, -6.0], (2.0 + 6.0 / 11.0, 7.810249675906654)),
    ],
)
def test_compute_repr_n_dist(input, expected):
    def test_fct(input):
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        return _compute_repr_n_dist(inp)

    assert test_fct(input) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        ([1.0, 0.0], 0.0),
        ([0.0, 1.0], 1.0),
        ([-1.0, 0.0], 2.0),
        ([0.0, -1.0], 3.0),
        ([2.0, 0.0], 0.0),
        ([0.0, 2.0], 1.0),
        ([-2.0, 0.0], 2.0),
        ([0.0, -2.0], 3.0),
    ],
)
def test_angle_representation(input, expected):
    def func(input):
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        repr, _dist = _compute_repr_n_dist(inp)
        return repr

    assert func(input) == expected


@pytest.mark.parametrize(
    "input, expected",
    [
        ([1.0, 0.0], 0.0),
        ([0.0, 1.0], 1.0),
        ([-1.0, 0.0], 2.0),
        ([0.0, -1.0], 3.0),
        ([2.0, 0.0], 0.0),
        ([0.0, 2.0], 1.0),
        ([-2.0, 0.0], 2.0),
        ([0.0, -2.0], 3.0),
        ([1.0, 1.0], 0.0),
        ([-1.0, 1.0], 1.0),
        ([-1.0, -1.0], 2.0),
        ([1.0, -1.0], 3.0),
        ([1.0, 0.00001], 0.0),
        ([0.00001, 1.0], 0.0),
        ([-1.0, 0.00001], 1.0),
        ([0.00001, -1.0], 3.0),
        ([1.0, -0.00001], 3.0),
        ([-0.00001, 1.0], 1.0),
        ([-1.0, -0.00001], 2.0),
        ([-0.00001, -1.0], 2.0),
    ],
)
def test_angle_repr_quadrant(input, expected):
    def func(input):
        inp = np.array(input, dtype=configs.DTYPE_FLOAT)
        repr, _dist = _compute_repr_n_dist(inp)
        return repr

    res = func(input)
    assert res >= expected
    assert res < expected + 1


def test_astar_path_uses_query_edges_without_changing_the_graph():
    graph = nx.Graph()
    graph.add_weighted_edges_from([(0, 1, 1.0), (1, 2, 1.0), (0, 2, 5.0)])
    edges_before = sorted(graph.edges(data="weight"))

    def no_heuristic(n1, n2):
        return 0.0

    assert astar_path(graph, {}, 0, 2, no_heuristic) == [0, 1, 2]
    # a shortcut through the query edges
    query_edges = {0: {3: 0.5}, 3: {0: 0.5, 2: 0.5}, 2: {3: 0.5}}
    assert astar_path(graph, query_edges, 0, 2, no_heuristic) == [0, 3, 2]
    # nodes only connected by query edges
    query_edges = {4: {0: 1.0}, 0: {4: 1.0}, 5: {2: 1.0}, 2: {5: 1.0}}
    assert astar_path(graph, query_edges, 4, 5, no_heuristic) == [4, 0, 1, 2, 5]
    # no path
    assert astar_path(graph, {}, 0, 6, no_heuristic) is None
    assert astar_path(graph, {6: {7: 1.0}, 7: {6: 1.0}}, 0, 7, no_heuristic) is None
    assert sorted(graph.edges(data="weight")) == edges_before
    assert 3 not in graph


@pytest.mark.parametrize("seed", range(20))
def test_astar_path_matches_networkx(seed):
    rng = np.random.default_rng(seed)
    nr_nodes = 30
    positions = rng.uniform(0.0, 10.0, (nr_nodes + 2, 2))

    def distance(n1, n2):
        return float(np.linalg.norm(positions[n1] - positions[n2]))

    graph = nx.Graph()
    graph.add_nodes_from(range(nr_nodes))
    for n1, n2 in rng.integers(0, nr_nodes, (60, 2)):
        if n1 != n2:
            graph.add_edge(int(n1), int(n2), weight=distance(n1, n2))
    # query nodes after the graph nodes, one query edge duplicates a graph edge
    source, target = nr_nodes, nr_nodes + 1
    query_edges = {}
    for query_node in (source, target):
        for n in rng.choice(nr_nodes, 4, replace=False):
            n = int(n)
            query_edges.setdefault(query_node, {})[n] = distance(query_node, n)
            query_edges.setdefault(n, {})[query_node] = distance(query_node, n)
    u, v = next(iter(graph.edges))
    query_edges.setdefault(u, {})[v] = graph.edges[u, v]["weight"]
    query_edges.setdefault(v, {})[u] = graph.edges[u, v]["weight"]

    search_graph = graph.copy()
    for n1, neighbours in query_edges.items():
        for n2, d in neighbours.items():
            search_graph.add_edge(n1, n2, weight=d)
    try:
        expected = nx.astar_path(search_graph, source, target, heuristic=distance)
    except nx.NetworkXNoPath:
        expected = None
    edges_before = sorted(graph.edges(data="weight"))
    assert astar_path(graph, query_edges, source, target, distance) == expected
    assert sorted(graph.edges(data="weight")) == edges_before
