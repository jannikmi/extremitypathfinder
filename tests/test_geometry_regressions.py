"""Regression coverage for documented polygon geometry edge cases."""

from math import sqrt

import networkx as nx
import pytest

from extremitypathfinder import PolygonEnvironment


BOUNDARY = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
RECTANGULAR_HOLE = [
    (3.0, 4.0),
    (3.0, 6.0),
    (7.0, 6.0),
    (7.0, 4.0),
]
DETOUR_DISTANCE = 4.0 + 2.0 * sqrt(5.0)


def _weighted_edges(graph):
    """Return a stable snapshot without depending on NetworkX iteration order."""
    return {
        (min(int(node1), int(node2)), max(int(node1), int(node2)), data["weight"])
        for node1, node2, data in graph.edges(data=True)
    }


def test_near_collinear_hole_vertex_does_not_change_shortest_distance():
    # The fifth hole vertex lies only 1e-12 above the straight lower edge. It is
    # deliberately not part of the reference route; the lower corners remain
    # mutually visible through free space.
    near_collinear_hole = RECTANGULAR_HOLE + [(5.0, 4.0 + 1e-12)]
    environment = PolygonEnvironment()
    environment.store(BOUNDARY, [near_collinear_hole], validate=True)

    path, distance = environment.find_shortest_path((1.0, 5.0), (9.0, 5.0))

    assert path == [(1.0, 5.0), (3.0, 4.0), (7.0, 4.0), (9.0, 5.0)]
    assert distance == pytest.approx(DETOUR_DISTANCE)


@pytest.mark.parametrize("scale", [1e-6, 1.0, 1e9])
def test_shortest_path_scales_with_coordinate_magnitude(scale):
    def scaled(polygon):
        return [(x * scale, y * scale) for x, y in polygon]

    environment = PolygonEnvironment()
    environment.store(
        scaled(BOUNDARY),
        [scaled(RECTANGULAR_HOLE)],
        validate=True,
    )

    _, distance = environment.find_shortest_path(
        (1.0 * scale, 5.0 * scale),
        (9.0 * scale, 5.0 * scale),
    )

    assert distance == pytest.approx(DETOUR_DISTANCE * scale, rel=1e-12)


def test_touching_holes_keep_shared_vertex_non_blocking():
    # The two clockwise holes share (5, 5). The package's documented semantics
    # permit a route through that duplicate vertex and along touching edges.
    holes = [
        [(3.0, 5.0), (3.0, 7.0), (5.0, 7.0), (5.0, 5.0)],
        [(5.0, 3.0), (5.0, 5.0), (7.0, 5.0), (7.0, 3.0)],
    ]
    environment = PolygonEnvironment()
    environment.store(BOUNDARY, holes, validate=True)

    path, distance = environment.find_shortest_path((1.0, 5.0), (9.0, 5.0))

    assert (5.0, 5.0) in path
    assert distance == pytest.approx(8.0)


@pytest.mark.parametrize(
    ("boundary", "holes", "error", "message"),
    [
        (
            [(0.0, 0.0), (1.0, 0.0)],
            [],
            TypeError,
            "at least contain 3 vertices",
        ),
        (
            [(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (0.0, 1.0)],
            [],
            ValueError,
            "must not be identical",
        ),
        (
            [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)],
            [],
            ValueError,
            "self intersections",
        ),
        (
            list(reversed(BOUNDARY)),
            [],
            ValueError,
            "boundary polygon must be counter clockwise",
        ),
        (
            [(0.0, 0.0), (1.0, 0.0), (float("nan"), 1.0)],
            [],
            ValueError,
            "coordinates must be finite",
        ),
        (
            [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)],
            [],
            ValueError,
            "area must be non-zero",
        ),
        (
            BOUNDARY,
            [list(reversed(RECTANGULAR_HOLE))],
            ValueError,
            "hole polygon must be clockwise",
        ),
    ],
)
def test_invalid_polygons_fail_predictably(boundary, holes, error, message):
    environment = PolygonEnvironment()

    with pytest.raises(error, match=message):
        environment.store(boundary, holes, validate=True)


def test_repeated_queries_do_not_mutate_precomputed_graph():
    environment = PolygonEnvironment()
    environment.store(BOUNDARY, [RECTANGULAR_HOLE], validate=True)
    nodes_before = set(environment.graph.nodes)
    edges_before = _weighted_edges(environment.graph)
    queries = [
        ((1.0, 5.0), (9.0, 5.0), DETOUR_DISTANCE),
        ((1.0, 2.0), (9.0, 2.0), 8.0),
        ((9.0, 5.0), (1.0, 5.0), DETOUR_DISTANCE),
    ]

    for _ in range(3):
        for start, goal, expected_distance in queries:
            _, distance = environment.find_shortest_path(start, goal)
            assert distance == pytest.approx(expected_distance)
            assert set(environment.graph.nodes) == nodes_before
            assert _weighted_edges(environment.graph) == edges_before
            assert environment.idx_start not in environment.graph
            assert environment.idx_goal not in environment.graph
            assert nx.is_weighted(environment.graph)
