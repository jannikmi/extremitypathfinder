"""Regressions found by reviewing the core algorithms (visibility, free wedges, graph, query)"""

import itertools
import pickle
from fractions import Fraction
from math import hypot

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, utils, wedges
from extremitypathfinder.utils_numba import (
    _compute_repr_n_dist,
    _lies_on_edge,
    reps_n_distances,
)
from tests.test_cases import GRID_ENV_PARAMS, POLYGON_ENVS

# the edge (0, 0) -> B is not axis parallel: points on it are not exactly on its line in floating point
B = (7.600508873702164, 8.401240233046146)
SLANTED_BOUNDARY = [(0.0, 0.0), B, (0.0, 10.0)]


def test_query_points_on_a_slanted_edge_see_directly():
    """a query point on an edge is not blocked by that edge (in both query directions)"""
    environment = PolygonEnvironment(SLANTED_BOUNDARY, [], validate=True)
    goal = (1.0, 5.0)
    end = np.array(B)
    starts = [
        tuple(t * end)
        for t in np.linspace(0.01, 0.99, 99)
        if _lies_on_edge(t * end, np.zeros(2), end)
    ]
    assert len(starts) > 10
    for start in starts:
        distance = hypot(goal[0] - start[0], goal[1] - start[1])
        assert environment.find_shortest_path(start, goal) == (
            [start, goal],
            pytest.approx(distance),
        )
        assert environment.find_shortest_path(goal, start) == (
            [goal, start],
            pytest.approx(distance),
        )


def test_passing_a_contact_is_detected_despite_rounding():
    """rule 3 also applies when the angle representations of collinear points differ in the last bit"""
    # exactly collinear with the origin: the candidate is 3 * contact (exact in floating point),
    # but |dx| + |dy| rounds differently for both vectors
    contact = (24.189145995204086, 1.9504636963259347)
    cx, cy = contact
    holes = [
        [(cx - 1.0, cy), (cx - 1.0, cy + 1.0), (cx, cy + 1.0), contact],
        [contact, (cx + 1.0, cy), (cx + 1.0, cy - 1.0), (cx, cy - 1.0)],
    ]
    environment = PolygonEnvironment(
        [(-10.0, -10.0), (250.0, -10.0), (250.0, 250.0), (-10.0, 250.0)],
        holes,
        validate=True,
    )
    origin, candidate = environment.idx_start, environment.idx_goal
    coords = np.append(environment.coords, [(0.0, 0.0), (3.0 * cx, 3.0 * cy)], axis=0)
    representations, distances = utils.cmp_reps_n_distances(origin, coords)
    contact_idx = int(np.flatnonzero(np.all(coords == contact, axis=1))[0])
    # precondition: the line of sight passes through the contact, but the representations differ
    assert coords[candidate][0] / 3.0 == cx and coords[candidate][1] / 3.0 == cy
    assert representations[contact_idx] != representations[candidate]

    visible = environment.points.filter_visible(
        origin, {candidate}, coords, representations, distances, {candidate: None}
    )

    # the line of sight passes between the two touching holes
    assert visible == set()


def test_extremity_opposite_of_a_bounding_ray_stays_connected():
    """an extremity exactly opposite of a bounding ray of another extremity does not lie "in front" of it"""
    # (10, 10): extremity with a bounding ray towards (8, 8)
    # (12, 12): extremity exactly opposite of this ray, seen from (10, 10)
    holes = [
        [(8.0, 8.0), (6.0, 10.0), (10.0, 10.0)],
        [(12.0, 12.0), (11.5, 12.5), (12.0, 13.0), (12.5, 12.5)],
    ]
    environment = PolygonEnvironment(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)], holes, validate=True
    )
    node = {tuple(environment.coords[i]): i for i in environment.extremity_indices}
    assert environment.graph.has_edge(node[(10.0, 10.0)], node[(12.0, 12.0)])


def test_queries_do_not_modify_the_environment():
    environment = PolygonEnvironment(*POLYGON_ENVS[2])

    def state():
        # Note: not the pickled graph, networkx caches views (e.g. ``graph.nodes``) on first access
        attributes = {k: v for k, v in environment.__dict__.items() if k != "graph"}
        graph = environment.graph
        return pickle.dumps(attributes), list(graph.nodes), list(graph.edges.data())

    state_before = state()
    environment.find_shortest_path((4.0, 4.5), (6.0, 4.5))
    environment.find_shortest_path((9.1, 4.0), (9.1, 6.0))
    assert state() == state_before


def test_last_extremity_is_reduced_as_well():
    """the graph neighbours "in front" of the last extremity get disconnected"""
    holes = [
        # (13, 14): the obstacle lies to the lower right
        [(13.0, 14.0), (15.0, 14.0), (15.0, 12.0), (13.0, 12.0)],
        # (10, 10), the last vertex: the obstacle lies to the lower left, (13, 14) lies in front of it
        [(10.0, 8.0), (8.0, 8.0), (8.0, 10.0), (10.0, 10.0)],
    ]
    environment = PolygonEnvironment(
        [(0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0)], holes, validate=True
    )
    node = {tuple(environment.coords[i]): i for i in environment.extremity_indices}
    assert node[(10.0, 10.0)] == environment.extremity_indices[-1]
    assert not environment.graph.has_edge(node[(13.0, 14.0)], node[(10.0, 10.0)])
    # the shortest paths stay the same
    assert environment.find_shortest_path((14.0, 15.0), (9.0, 11.0))[
        1
    ] == pytest.approx(hypot(5.0, 4.0))


@pytest.mark.parametrize(
    "environment",
    [PolygonEnvironment(*env) for env in POLYGON_ENVS]
    + [PolygonEnvironment.from_grid(*GRID_ENV_PARAMS)],
)
def test_graph_nodes_in_front_of_each_other_are_not_connected(environment):
    """the graph reduction ("Property One") applies to every extremity, the last one included"""
    coords = environment.coords
    for u, v in environment.graph.edges:
        for origin, other in ((u, v), (v, u)):
            point = environment.points.by_idx[origin]
            representations, _ = utils.cmp_reps_n_distances(origin, coords)
            vector = coords[other] - coords[origin]
            assert not point.in_front(representations[other], vector)


def test_vectorised_representations_are_identical():
    rng = np.random.default_rng(0)
    vectors = np.concatenate(
        [
            rng.normal(size=(1000, 2)) * 10.0 ** rng.integers(-6, 9, size=(1000, 1)),
            rng.integers(-5, 6, size=(200, 2)).astype(float),
            [[0.0, 0.0], [-0.0, 1.0], [1.0, -0.0], [-0.0, -0.0], [-3.0, 0.0]],
        ]
    )
    representations, distances = reps_n_distances(vectors)
    for vector, rep, distance in zip(vectors, representations, distances):
        expected_rep, expected_distance = _compute_repr_n_dist(vector)
        assert distance == expected_distance
        if np.isnan(expected_rep):
            assert np.isnan(rep)
        else:
            assert rep == expected_rep


def _star_grid(holes_per_side: int):
    """a square boundary with a grid of 8-vertex star holes (every second vertex is an extremity)"""
    holes = []
    for i in range(holes_per_side):
        for j in range(holes_per_side):
            cx, cy = 4.0 * i + 3.0, 4.0 * j + 3.0
            radii = (1.2, 0.5) * 4
            # clockwise
            angles = [-k * np.pi / 4 for k in range(8)]
            holes.append(
                [
                    (cx + r * np.cos(a), cy + r * np.sin(a))
                    for r, a in zip(radii, angles, strict=True)
                ]
            )
    size = 4.0 * holes_per_side + 2.0
    return [(0.0, 0.0), (size, 0.0), (size, size), (0.0, size)], holes


def test_environment_size_without_graph_is_linear_in_the_vertices():
    """the environment does not keep data of size O(extremities * vertices), only the graph may grow faster

    Previously the angle representations and distances of all vertices seen from every extremity were stored:
    peak memory and pickle size grew quadratically (e.g. 1 GiB for 10k vertices).
    """
    bytes_per_vertex = []
    for holes_per_side in (3, 6):
        environment = PolygonEnvironment(*_star_grid(holes_per_side), validate=True)
        state = {k: v for k, v in environment.__dict__.items() if k != "graph"}
        bytes_per_vertex.append(len(pickle.dumps(state)) / environment.nr_vertices)
    # 4 times the vertices and extremities: previously about 3 times the bytes per vertex
    assert bytes_per_vertex[1] < 1.3 * bytes_per_vertex[0]


def _star_grid_datastructs(holes_per_side: int):
    boundary, holes = _star_grid(holes_per_side)
    coords, edge_vertex_idxs, *_ = utils.compile_polygon_datastructs(
        np.array(boundary, dtype=float), [np.array(h, dtype=float) for h in holes]
    )
    return coords, edge_vertex_idxs


def test_edges_through_points_bounded_memory():
    """the bounding box test of all positions against all edges is processed in small chunks

    Previously up to 2**22 position-edge pairs were tested at once:
    several boolean arrays of 4 MiB each for about 2,000 vertices.
    """
    import tracemalloc

    # 2,052 vertices: 4.2 million position-edge pairs
    coords, edge_vertex_idxs = _star_grid_datastructs(16)
    tracemalloc.start()
    try:
        wedges._find_edges_through_points(coords, coords, edge_vertex_idxs)
        peak = tracemalloc.get_traced_memory()[1]
    finally:
        tracemalloc.stop()
    # previously 8.6 MiB, now 3.4 MiB
    assert peak < 5 * 2**20


def test_edges_through_points_independent_of_the_chunks(monkeypatch):
    """the edges through every position (and their order) do not depend on how the positions are chunked"""
    rng = np.random.default_rng(0)
    # axis parallel edges between integer points: exact on-edge tests, overlapping and crossing edges
    starts = rng.integers(0, 12, size=(40, 2))
    lengths = rng.integers(1, 8, size=40)
    horizontal = rng.random(40) < 0.5
    ends = starts + np.where(horizontal[:, None], [1, 0], [0, 1]) * lengths[:, None]
    coords = np.concatenate([starts, ends]).astype(float)
    edge_vertex_idxs = np.stack([np.arange(40), np.arange(40, 80)], axis=1)
    # all integer points of the area: vertices, points on one or several edges, points off any edge
    xs, ys = np.meshgrid(np.arange(20.0), np.arange(20.0))
    positions = np.stack([xs.ravel(), ys.ravel()], axis=1)

    expected = wedges._find_edges_through_points(positions, coords, edge_vertex_idxs)
    assert any(len(edges) >= 2 for edges in expected.values())
    nr_edges = len(edge_vertex_idxs)
    # one position per chunk, several positions per chunk with a partial last chunk, a single chunk
    for pairs in (1, 7 * nr_edges, 13 * nr_edges, len(positions) * nr_edges):
        monkeypatch.setattr(wedges, "MAX_BOX_TEST_PAIRS", pairs)
        assert (
            wedges._find_edges_through_points(positions, coords, edge_vertex_idxs)
            == expected
        )


# float32 triangles of a triangulated hexagon map (prototypes/real_world_geometry.py),
# shifted by a few float32 units in the last place: they overlap or leave gaps of about 1e-8
CORNER_TRIANGLES = [
    [
        (0.062222279608249664, 0.251468300819397),
        (0.06222224980592728, 0.3951645493507385),
        (0.12444452941417694, 0.28739243745803833),
    ],
    [
        (0.06222224980592728, 0.3951644003391266),
        (0.12444455921649933, 0.35924032330513),
        (0.12444451451301575, 0.2873923182487488),
    ],
    [
        (0.12444455921649933, 0.35924050211906433),
        (0.1866668462753296, 0.251468300819397),
        (0.12444454431533813, 0.28739237785339355),
    ],
]
UNIT_SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


def _orientation(a, b, c) -> int:
    """the exact sign of the cross product (b - a) x (c - a)"""
    a, b, c = ([Fraction(x) for x in p] for p in (a, b, c))
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    return (cross > 0) - (cross < 0)


def test_lines_of_sight_close_to_an_axis_do_not_cut_obstacle_corners():
    """directions about 1e-12 rad apart close to the vertical axis are told apart

    The angle representation used to be the sine of the angle within its quadrant, flat at the axes:
    seen from the lower vertex B of the right triangle, its upper vertex A and the corner V of the middle
    triangle got the same representation. The line of sight A-B "touched V only" and cut the middle triangle.
    """
    a, b = CORNER_TRIANGLES[2][0], CORNER_TRIANGLES[0][2]
    v, w = CORNER_TRIANGLES[1][1], CORNER_TRIANGLES[1][2]
    # exactly: A and B lie on opposite sides of the edge V-W of the middle triangle, the line A-B crosses it
    assert _orientation(v, w, a) != _orientation(v, w, b)
    assert _orientation(a, b, v) != _orientation(a, b, w)
    # V lies inside the right triangle: no gap between the middle and the right triangle
    right = CORNER_TRIANGLES[2]
    assert all(_orientation(right[k], right[(k + 1) % 3], v) == -1 for k in range(3))

    environment = PolygonEnvironment(UNIT_SQUARE, CORNER_TRIANGLES, validate=True)
    start, goal = (0.125, 0.5), (0.125, 0.1)
    corner = (0.1866668462753296, 0.251468300819397)
    path, length = environment.find_shortest_path(start, goal)
    # around the outer corner of the right triangle
    # (shorter than around the two left corners of the left triangles)
    assert [tuple(map(float, p)) for p in path] == [start, corner, goal]
    expected = hypot(corner[0] - start[0], corner[1] - start[1]) + hypot(
        goal[0] - corner[0], goal[1] - corner[1]
    )
    assert length == pytest.approx(expected)


# two float32 triangles separated by a crack of about 3e-13 (near-contacts do not block)
CRACK_TRIANGLES = [
    [
        (0.19627825915813446, 0.5795714259147644),
        (0.2500000596046448, 0.4999999403953552),
        (0.19627830386161804, 0.5000000596046448),
    ],
    [
        (0.19627822935581207, 0.5000000596046448),
        (0.24999991059303284, 0.4999999403953552),
        (0.2499999701976776, 0.2500000596046448),
    ],
]


@pytest.mark.parametrize("reverse", [False, True])
def test_vertices_close_to_an_axis_are_not_hidden(reverse):
    """of candidates with the same angle representation only the closest is visible:
    with the sine representation, directions about 3e-12 rad apart close to the horizontal axis
    collapsed and hid the far end of the crack. The path went around the upper triangle (48 % longer)."""
    upper, lower = CRACK_TRIANGLES
    # exactly: the lower edge of the upper triangle lies above the upper edge of the lower triangle
    # (both end vertices of each edge lie strictly on the free side of the other edge): a free crack
    assert _orientation(upper[1], upper[2], lower[0]) == 1
    assert _orientation(upper[1], upper[2], lower[1]) == 1
    assert _orientation(lower[0], lower[1], upper[1]) == 1
    assert _orientation(lower[0], lower[1], upper[2]) == 1

    environment = PolygonEnvironment(UNIT_SQUARE, CRACK_TRIANGLES, validate=True)
    start, goal = (0.3, 0.5), (0.15, 0.5)
    # through the crack: from its right end on the upper triangle to its left end on the lower one
    expected_path = [start, upper[1], lower[0], goal]
    if reverse:
        start, goal = goal, start
        expected_path = expected_path[::-1]
    path, length = environment.find_shortest_path(start, goal)
    assert [tuple(map(float, p)) for p in path] == expected_path
    expected = sum(
        hypot(q[0] - p[0], q[1] - p[1]) for p, q in itertools.pairwise(expected_path)
    )
    assert length == pytest.approx(expected)


def test_spikes_too_narrow_for_floating_point_are_rejected():
    """a spike (here a slit) about 1e-17 wide, produced by a Shapely union at the map border:
    its free side cannot be decided (without validation lines of sight crossed the polygon).
    The sine representation rejected it only because both rays got the same value ("overlapping edges")."""
    polygon = [
        (0.0, 0.6528825163841248),
        (1.3877787807814457e-17, 0.6812687516212463),
        (0.03801143914461136, 0.7032146453857422),
        (0.0, 0.6812687516212463),
        (0.0, 0.9083582758903503),
        (-0.1, 0.78),
    ]
    for hole in (polygon, polygon[::-1]):
        with pytest.raises(ValueError, match="almost the same direction"):
            PolygonEnvironment(
                [(-1, -1), (1, -1), (1, 2), (-1, 2)], [hole], validate=True
            )
