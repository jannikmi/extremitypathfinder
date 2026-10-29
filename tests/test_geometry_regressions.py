"""Regression coverage for documented polygon geometry edge cases."""

import itertools
from math import hypot, sqrt

import networkx as nx
import pytest

from extremitypathfinder import PolygonEnvironment
from tests.test_cases import POLYGON_ENVS

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
        for point1, point2 in itertools.pairwise(path)
    )
    assert reported_distance == pytest.approx(measured_distance)

    for segment_start, segment_end in itertools.pairwise(path):
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
    environment = PolygonEnvironment(BOUNDARY, [near_collinear_hole], validate=True)

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

    holes = [scaled(RECTANGULAR_HOLE)]
    environment = PolygonEnvironment(
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


def test_touching_holes_block_at_shared_vertex():
    # The two clockwise holes share (5, 5). Touching obstacles form a wall (#24):
    # the route along the touching edges through the shared vertex is blocked,
    # the shortest path goes around the upper left hole.
    holes = [
        [(3.0, 5.0), (3.0, 7.0), (5.0, 7.0), (5.0, 5.0)],
        [(5.0, 3.0), (5.0, 5.0), (7.0, 5.0), (7.0, 3.0)],
    ]
    environment = PolygonEnvironment(BOUNDARY, holes, validate=True)

    path, distance = environment.find_shortest_path((1.0, 5.0), (9.0, 5.0))

    assert (5.0, 5.0) not in path
    assert distance == pytest.approx(2.0 + 2.0 * sqrt(2.0) + 2.0 * sqrt(5.0))
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
        # https://github.com/jannikmi/extremitypathfinder/issues/135:
        # the chain (2,2) -> (5,5) -> (9,9) crosses the edge (8,2) -> (2,8) at the vertex (5,5)
        (
            BOUNDARY,
            [[(2.0, 2.0), (5.0, 5.0), (9.0, 9.0), (8.0, 2.0), (2.0, 8.0)]],
            ValueError,
            "self intersections",
        ),
        # the two corners at the identical vertices (5, 5) interleave: a figure eight
        (
            BOUNDARY,
            [
                [
                    (3.0, 5.0),
                    (3.0, 7.0),
                    (5.0, 7.0),
                    (5.0, 5.0),
                    (5.0, 2.0),
                    (8.0, 2.0),
                    (8.0, 5.0),
                    (5.0, 5.0),
                ]
            ],
            ValueError,
            "self intersections",
        ),
        # the vertex (3,4) lies on the diagonal edge (0,0) -> (6,8), its neighbours on opposite sides
        (
            BOUNDARY,
            [
                [
                    (0.0, 0.0),
                    (6.0, 8.0),
                    (11.0, 0.0),
                    (4.0, 1.0),
                    (3.0, 4.0),
                    (2.0, 7.0),
                    (-5.0, 7.0),
                ]
            ],
            ValueError,
            "self intersections",
        ),
        # zero-width spike along a diagonal: the rays (-3,-3) and (-2,-2) seen from (7,7) are identical
        (
            BOUNDARY,
            [[(0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (7.0, 7.0), (5.0, 5.0), (4.0, 0.0)]],
            ValueError,
            "overlapping edges",
        ),
        # zero-width spike: the consecutive edges (6,3) -> (8,3) -> (5,3) overlap
        (
            BOUNDARY,
            [[(3.0, 3.0), (3.0, 6.0), (6.0, 6.0), (6.0, 3.0), (8.0, 3.0), (5.0, 3.0)]],
            ValueError,
            "overlapping edges",
        ),
        # zero-width slit: the edges (2.5,5) -> (2.5,3) and (2.5,3) -> (2.5,5) overlap
        (
            BOUNDARY,
            [
                [
                    (1.0, 1.0),
                    (1.0, 5.0),
                    (2.5, 5.0),
                    (2.5, 3.0),
                    (2.0, 2.0),
                    (3.0, 2.0),
                    (2.5, 3.0),
                    (2.5, 5.0),
                    (4.0, 5.0),
                    (4.0, 1.0),
                ]
            ],
            ValueError,
            "overlapping edges",
        ),
    ],
)
def test_invalid_polygons_fail_predictably(boundary, holes, error, message):
    with pytest.raises(error, match=message):
        PolygonEnvironment(boundary, holes, validate=True)


@pytest.mark.parametrize(
    ("boundary", "holes"),
    [
        # a hole touching itself at the vertex (5, 5)
        (
            BOUNDARY,
            [
                [
                    (3.0, 5.0),
                    (3.0, 7.0),
                    (5.0, 7.0),
                    (5.0, 5.0),
                    (8.0, 5.0),
                    (8.0, 2.0),
                    (5.0, 2.0),
                    (5.0, 5.0),
                ]
            ],
        ),
        # a boundary touching itself at the vertex (5, 5)
        (
            [
                (0.0, 0.0),
                (5.0, 0.0),
                (5.0, 5.0),
                (10.0, 5.0),
                (10.0, 10.0),
                (5.0, 10.0),
                (5.0, 5.0),
                (0.0, 5.0),
            ],
            [],
        ),
        # the vertex (7, 6) touches the edge (8, 6) -> (4, 6) of the same hole
        (
            BOUNDARY,
            [
                [
                    (2.0, 2.0),
                    (2.0, 8.0),
                    (8.0, 8.0),
                    (8.0, 6.0),
                    (4.0, 6.0),
                    (4.0, 4.0),
                    (6.0, 4.0),
                    (7.0, 6.0),
                    (8.0, 4.0),
                    (8.0, 2.0),
                ]
            ],
        ),
    ],
)
def test_polygons_touching_themselves_are_valid(boundary, holes):
    PolygonEnvironment(boundary, holes, validate=True)


def test_repeated_queries_do_not_mutate_precomputed_graph():
    environment = PolygonEnvironment(BOUNDARY, [RECTANGULAR_HOLE], validate=True)
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


@pytest.mark.parametrize("env_params", POLYGON_ENVS)
def test_paths_between_extremities_are_symmetric(env_params):
    """A goal identical to an extremity must be merged with that graph node."""
    environment = PolygonEnvironment(*env_params)

    extremities = environment.all_extremities
    for start, goal in itertools.combinations(extremities, 2):
        try:
            _, distance = environment.find_shortest_path(start, goal)
        except ValueError:
            # extremities of overlapping polygons may lie outside of the map
            continue
        _, reverse_distance = environment.find_shortest_path(goal, start)
        if distance is None:
            assert reverse_distance is None
        else:
            assert reverse_distance == pytest.approx(distance)


@pytest.mark.parametrize(
    "obstacles, expected_distance",
    [
        ([], hypot(4.0, 2.0)),
        ([(2, 1), (3, 1)], hypot(1.5, 1.5) + hypot(2.5, 0.5)),
    ],
)
def test_grid_with_default_arguments(obstacles, expected_distance):
    environment = PolygonEnvironment.from_grid(5, 3, obstacles)

    _, distance = environment.find_shortest_path((0.5, 0.5), (4.5, 2.5))
    assert distance == pytest.approx(expected_distance)


def test_path_does_not_pass_through_reflex_vertices():
    # https://github.com/jannikmi/extremitypathfinder/issues/84:
    # the path used to run along y = 172 through the reflex vertices (155.6, 172) and (252.4, 172)
    # of the T-shaped hole, i.e. through the hole
    boundary = [
        (161.0, 47.0),
        (247.0, 47.0),
        (247.0, 7.0),
        (361.0, 7.0),
        (361.0, 441.0),
        (7.0, 441.0),
        (7.0, 7.0),
        (161.0, 7.0),
    ]
    holes = [
        [
            (272.0, 172.0),
            (272.0, 36.0),
            (136.0, 36.0),
            (136.0, 172.0),
            (155.6, 172.0),
            (155.6, 332.4),
            (252.4, 332.4),
            (252.4, 172.0),
        ]
    ]
    environment = PolygonEnvironment(boundary, holes, validate=True)

    path, distance = environment.find_shortest_path(
        (50.004004, 44.004004), (318.004004, 44.004004)
    )

    assert path == [
        (50.004004, 44.004004),
        (155.6, 332.4),
        (252.4, 332.4),
        (318.004004, 44.004004),
    ]
    _assert_valid_path(path, distance, holes)
