"""Regressions found by reviewing the core algorithms (visibility, free wedges, graph, query)"""

from math import hypot

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, utils
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
    contact = (29.0, 29.0)
    holes = [
        [(26.0, 29.0), (26.0, 32.0), (29.0, 32.0), contact],
        [contact, (32.0, 29.0), (32.0, 26.0), (29.0, 26.0)],
    ]
    environment = PolygonEnvironment(
        [(-10.0, -10.0), (250.0, -10.0), (250.0, 250.0), (-10.0, 250.0)],
        holes,
        validate=True,
    )
    origin, candidate = environment.idx_start, environment.idx_goal
    coords = np.append(environment.coords, [(0.0, 0.0), (145.0, 145.0)], axis=0)
    representations, distances = utils.cmp_reps_n_distances(origin, coords)
    contact_idx = int(np.flatnonzero(np.all(coords == contact, axis=1))[0])
    # precondition: the line of sight passes through the contact, but the representations differ
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
    keys_before = set(environment.reprs_n_distances)
    environment.find_shortest_path((4.0, 4.5), (6.0, 4.5))
    environment.find_shortest_path((9.1, 4.0), (9.1, 6.0))
    assert set(environment.reprs_n_distances) == keys_before


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
            representations, _ = environment.reprs_n_distances[origin]
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
