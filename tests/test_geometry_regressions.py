"""Regression coverage for documented polygon geometry edge cases."""

from math import hypot, sqrt

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


def _orientation(point1, point2, point3):
    return (point2[0] - point1[0]) * (point3[1] - point1[1]) - (
        point2[1] - point1[1]
    ) * (point3[0] - point1[0])


def _properly_intersects(segment_start, segment_end, edge_start, edge_end):
    return (
        _orientation(segment_start, segment_end, edge_start)
        * _orientation(segment_start, segment_end, edge_end)
        < 0.0
        and _orientation(edge_start, edge_end, segment_start)
        * _orientation(edge_start, edge_end, segment_end)
        < 0.0
    )


def _strictly_inside_polygon(point, polygon):
    """Independent ray-casting check; polygon edges are considered outside."""
    x, y = point
    inside = False
    for point1, point2 in zip(polygon, polygon[1:] + polygon[:1]):
        if (
            _orientation(point1, point2, point) == 0.0
            and min(point1[0], point2[0]) <= x <= max(point1[0], point2[0])
            and min(point1[1], point2[1]) <= y <= max(point1[1], point2[1])
        ):
            return False
        if (point1[1] > y) != (point2[1] > y):
            crossing_x = point1[0] + (y - point1[1]) * (point2[0] - point1[0]) / (
                point2[1] - point1[1]
            )
            if x < crossing_x:
                inside = not inside
    return inside


def _assert_valid_path(path, reported_distance, holes):
    measured_distance = sum(
        hypot(point2[0] - point1[0], point2[1] - point1[1])
        for point1, point2 in zip(path, path[1:])
    )
    assert reported_distance == pytest.approx(measured_distance)

    for segment_start, segment_end in zip(path, path[1:]):
        midpoint = tuple((a + b) / 2.0 for a, b in zip(segment_start, segment_end))
        for hole in holes:
            assert not _strictly_inside_polygon(midpoint, hole)
            for edge_start, edge_end in zip(hole, hole[1:] + hole[:1]):
                assert not _properly_intersects(
                    segment_start,
                    segment_end,
                    edge_start,
                    edge_end,
                )


def test_near_collinear_hole_vertex_does_not_change_shortest_distance():
    # The fifth hole vertex lies only 1e-12 above the straight lower edge. It is
    # deliberately not part of the reference route; the lower corners remain
    # mutually visible through free space.
    near_collinear_hole = RECTANGULAR_HOLE + [(5.0, 4.0 + 1e-12)]
    environment = PolygonEnvironment()
    environment.store(BOUNDARY, [near_collinear_hole], validate=True)

    path, distance = environment.find_shortest_path((1.0, 5.0), (9.0, 5.0))

    assert distance == pytest.approx(DETOUR_DISTANCE)
    _assert_valid_path(path, distance, [near_collinear_hole])


@pytest.mark.parametrize(
    ("scale", "offset"),
    [(1e-6, 0.0), (1.0, 0.0), (1e9, 0.0), (1.0, 1e9)],
)
def test_shortest_path_scales_with_coordinate_magnitude(scale, offset):
    def scaled(polygon):
        return [(x * scale + offset, y * scale + offset) for x, y in polygon]

    environment = PolygonEnvironment()
    holes = [scaled(RECTANGULAR_HOLE)]
    environment.store(
        scaled(BOUNDARY),
        holes,
        validate=True,
    )

    path, distance = environment.find_shortest_path(
        (1.0 * scale + offset, 5.0 * scale + offset),
        (9.0 * scale + offset, 5.0 * scale + offset),
    )

    assert distance == pytest.approx(DETOUR_DISTANCE * scale, rel=1e-12)
    _assert_valid_path(path, distance, holes)


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
    _assert_valid_path(path, distance, holes)


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
            [0.0, 1.0, 2.0],
            [],
            TypeError,
            "must consist of two values",
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
            path, distance = environment.find_shortest_path(start, goal)
            assert distance == pytest.approx(expected_distance)
            _assert_valid_path(path, distance, [RECTANGULAR_HOLE])
            assert set(environment.graph.nodes) == nodes_before
            assert _weighted_edges(environment.graph) == edges_before
            assert environment.idx_start not in environment.graph
            assert environment.idx_goal not in environment.graph
            assert nx.is_weighted(environment.graph)
