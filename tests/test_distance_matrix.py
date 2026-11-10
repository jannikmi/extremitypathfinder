"""Shortest path lengths between many points: ``PolygonEnvironment.distance_matrix()`` (#175)

The lengths must be identical to a loop over ``find_shortest_path()`` (also in the last bit),
for random points of the free space, polygon vertices, points on edges and contacts.
"""

import copy
from math import hypot, inf

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment
from tests import test_contacts as contacts
from tests.test_random_queries import ENV_IDS, ENVIRONMENTS, _random_query_points

CONTACT_ENVIRONMENTS = {
    "shared vertex": (contacts.BOUNDARY, contacts.SHARED_VERTEX_HOLES),
    "vertex on edge": (contacts.BOUNDARY, contacts.VERTEX_ON_EDGE_HOLES),
    "overlapping edges": (contacts.BOUNDARY, contacts.OVERLAPPING_EDGES_HOLES),
    "touching tips": (contacts.BOUNDARY, contacts.TOUCHING_TIPS_HOLES),
    "diagonal edge": (contacts.BOUNDARY, contacts.DIAGONAL_EDGE_HOLES),
    "hole touching itself": (contacts.BOUNDARY, [contacts.SELF_TOUCHING_HOLE]),
    "unbounded": (None, contacts.SHARED_VERTEX_HOLES),
}


def _loop(environment: PolygonEnvironment, points_a, points_b) -> np.ndarray:
    """the lengths of a loop over find_shortest_path(), ``inf`` without a path"""
    matrix = np.full((len(points_a), len(points_b)), inf)
    for i, a in enumerate(points_a):
        for j, b in enumerate(points_b):
            length = environment.find_shortest_path(a, b, verify=False)[1]
            if length is not None:
                matrix[i, j] = length
    return matrix


def _assert_identical_to_loop(environment, points_a, points_b=None):
    matrix = environment.distance_matrix(points_a, points_b, verify=False)
    expected = _loop(environment, points_a, points_a if points_b is None else points_b)
    assert matrix.shape == expected.shape
    # identical, not only close: the same visibility decisions and the same sums of edge lengths
    assert np.array_equal(matrix, expected)


def _vertices_and_edge_midpoints(env_params, step: int = 1):
    boundary, holes = env_params
    polygons = [] if boundary is None else [boundary]
    points = []
    for polygon in polygons + holes:
        polygon = [tuple(map(float, p)) for p in polygon]
        for p, q in zip(polygon, polygon[1:] + polygon[:1]):
            points += [p, ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)]
    return points[::step]


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_random_points_and_vertices_identical_to_the_loop(env_id):
    env_params = ENVIRONMENTS[env_id]
    environment = PolygonEnvironment(*env_params)
    points = [tuple(p) for p in _random_query_points(env_params, seed=3, n=8)]
    vertices = _vertices_and_edge_midpoints(env_params)
    points += vertices[:: max(1, len(vertices) // 10)]
    _assert_identical_to_loop(environment, points)


@pytest.mark.parametrize("env_id", list(CONTACT_ENVIRONMENTS))
def test_contacts_identical_to_the_loop(env_id):
    # vertices, contacts, points on edges and grid points: many collinear points (shared directions)
    env_params = CONTACT_ENVIRONMENTS[env_id]
    environment = PolygonEnvironment(*env_params)
    points = _vertices_and_edge_midpoints(env_params)
    points += [(x / 2, y / 2) for x in range(0, 21, 4) for y in range(0, 21, 4)]
    points = sorted(set(points))
    _assert_identical_to_loop(environment, points)
    # different start and goal points
    _assert_identical_to_loop(environment, points[::2], points[1::2])


def test_collinear_points_do_not_hide_each_other():
    # the points lie on one line: each one is directly visible from all others
    environment = PolygonEnvironment(contacts.BOUNDARY, [])
    points = [(1.0, 1.0), (2.0, 2.0), (3.0, 3.0), (2.0, 2.0)]
    matrix = environment.distance_matrix(points)
    expected = [[hypot(a[0] - b[0], a[1] - b[1]) for b in points] for a in points]
    assert np.array_equal(matrix, expected)
    _assert_identical_to_loop(environment, points)


def test_no_path_is_inf():
    # a wall separates the map (cf. test_contacts.py)
    boundary = [(5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0), (5.0, -5.0)]
    holes = [[(-5.0, 1.0), (-5.0, 2.0), (5.0, 2.0), (5.0, 1.0)]]
    environment = PolygonEnvironment(boundary, holes)
    matrix = environment.distance_matrix([(0.0, 0.0), (0.0, 4.0)], [(1.0, 0.0)])
    assert matrix.tolist() == [[1.0], [inf]]


def test_shapes():
    environment = PolygonEnvironment(contacts.BOUNDARY, contacts.SHARED_VERTEX_HOLES)
    points = [(1.0, 1.0), (9.0, 9.0), (1.0, 9.0)]
    assert environment.distance_matrix(points).shape == (3, 3)
    assert environment.distance_matrix(points, points[:2]).shape == (3, 2)
    assert environment.distance_matrix([], points).shape == (0, 3)
    assert environment.distance_matrix(points, []).shape == (3, 0)
    assert environment.distance_matrix(np.array(points)).shape == (3, 3)
    with pytest.raises(ValueError, match="coordinate pairs"):
        environment.distance_matrix([(1.0, 1.0, 1.0)])


def test_points_outside_the_map_are_rejected():
    environment = PolygonEnvironment(contacts.BOUNDARY, contacts.SHARED_VERTEX_HOLES)
    with pytest.raises(ValueError, match=r"points_b\[1\]"):
        environment.distance_matrix([(1.0, 1.0)], [(1.0, 2.0), (11.0, 1.0)])
    with pytest.raises(ValueError, match=r"points_a\[0\]"):
        environment.distance_matrix([(-1.0, 1.0)])


def test_the_environment_is_not_changed():
    env_params = ENVIRONMENTS["grid"]
    environment = PolygonEnvironment(*env_params)
    before = copy.deepcopy(environment.__dict__)
    points = [tuple(p) for p in _random_query_points(env_params, seed=5, n=6)]
    points += _vertices_and_edge_midpoints(env_params, step=7)
    environment.distance_matrix(points, verify=False)

    after = environment.__dict__
    assert after.keys() == before.keys()
    for name in ("coords", "extremity_indices", "edge_vertex_idxs", "vertex_edge_idxs"):
        assert np.array_equal(after[name], before[name])
    graph, graph_before = environment.graph, before["graph"]
    assert sorted(graph.nodes) == sorted(graph_before.nodes)
    assert sorted(graph.edges(data=True)) == sorted(graph_before.edges(data=True))
