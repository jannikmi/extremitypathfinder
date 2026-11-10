"""Visibility and shortest paths from random query points (#138)

The other tests query mostly from polygon vertices and hand-picked points.
Here start and goal are random points of the free space of the test environments, checked with
independent references:

* the visibility from a query point against the reference implementation of ``test_find_visible.py``
* the paths: they stay within the free space, reversing start and goal gives the same length,
  and the length equals the shortest path in an unpruned visibility graph
  (without the elimination of the extremities "in front", Property One in [1, Ch. II 4.4.2])
* transforms of the environment and the query points (#3): scaling by a power of two gives identical results,
  inexact transforms (shifting by decimals, scaling by 1/3, small rotations) the same reachability and lengths
  within a tolerance. The random points keep a distance to all edges: rounding legitimately turns contacts
  of points on edges into near-contacts (numerical policy).

The random points come from fixed seeds: the tests are deterministic.
"""

import itertools
from math import cos, hypot, sin

import networkx as nx
import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, utils, wedges
from tests.test_cases import GRID_ENV_PARAMS, POLYGON_ENVS
from tests.test_contacts import BOUNDARY, SELF_TOUCHING_HOLE
from tests.test_find_visible import find_visible_reference
from tests.test_geometry_regressions import (
    _orientation,
    _properly_intersects,
    _strictly_inside_polygon,
)

NR_QUERY_POINTS = 12
# the minimum distance of a random query point to all edges, relative to the size of the environment
EDGE_MARGIN = 1e-6
# the measured maximum relative length difference after the inexact transforms is 1.3e-10 (cf. #3)
TRANSFORM_REL_TOLERANCE = 1e-9

_grid_env = PolygonEnvironment.from_grid(*GRID_ENV_PARAMS)
ENVIRONMENTS = {
    "separated": POLYGON_ENVS[0],
    "overlapping polygons": POLYGON_ENVS[1],
    "polygon": POLYGON_ENVS[2],
    "grid": (
        _grid_env.boundary_polygon.tolist(),
        [h.tolist() for h in _grid_env.holes],
    ),
    # a single polygon with repeated vertices (#135)
    "hole touching itself": (BOUNDARY, [SELF_TOUCHING_HOLE]),
}
ENV_IDS = list(ENVIRONMENTS)


def _polygons(env_params):
    boundary, holes = env_params
    polygons = [] if boundary is None else [boundary]
    return [[tuple(map(float, p)) for p in polygon] for polygon in polygons + holes]


def _segment_point_distance(point, start, end) -> float:
    p, a, b = (np.array(x, dtype=float) for x in (point, start, end))
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * ab)))


def _random_query_points(env_params, seed: int, n: int = NR_QUERY_POINTS):
    """random points of the free space with a distance to all edges"""
    environment = PolygonEnvironment(*env_params)
    polygons = _polygons(env_params)
    all_coords = np.array([p for polygon in polygons for p in polygon])
    low, high = all_coords.min(axis=0), all_coords.max(axis=0)
    margin = EDGE_MARGIN * float(np.max(high - low))
    edges = [
        (p1, p2)
        for polygon in polygons
        for p1, p2 in zip(polygon, polygon[1:] + polygon[:1])
    ]
    rng = np.random.default_rng(seed)
    points = []
    while len(points) < n:
        point = tuple(float(c) for c in rng.uniform(low, high))
        if not environment.within_map(np.array(point)):
            continue
        if min(_segment_point_distance(point, *edge) for edge in edges) <= margin:
            continue
        points.append(point)
    return points


@pytest.fixture(scope="module", params=ENV_IDS)
def env_id(request):
    return request.param


@pytest.fixture(scope="module")
def environment(env_id):
    return PolygonEnvironment(*ENVIRONMENTS[env_id], validate=True)


@pytest.fixture(scope="module")
def query_points(env_id):
    return _random_query_points(ENVIRONMENTS[env_id], seed=ENV_IDS.index(env_id))


def test_find_visible_from_random_points(environment, query_points):
    # all vertices as candidates: also the environments without extremities see something
    candidates = set(range(environment.nr_vertices))
    edges = set(range(environment.nr_edges))
    extremities = set(environment.extremity_indices.tolist())
    nr_visible = 0
    for point in query_points:
        coords = np.append(environment.coords, [point], axis=0)
        origin = len(coords) - 1
        representations, distances = utils.cmp_reps_n_distances(origin, coords)
        args = (
            origin,
            candidates,
            edges,
            coords,
            representations,
            distances,
            environment.edge_vertex_idxs,
        )
        expected = find_visible_reference(*args)
        nr_visible += len(expected)
        assert utils.find_visible(*args) == expected, f"from {point}"
        # no polygon passes a random query point: the same visibility through the environment
        # (its candidates are the graph nodes, i.e. extremities)
        expected = find_visible_reference(origin, extremities, *args[2:])
        found = environment.get_visible_idxs(
            origin, extremities, coords, representations, distances
        )
        assert found == expected, f"from {point}"
    assert nr_visible > 0


def _on_border(point, polygon) -> bool:
    return any(
        _orientation(p1, p2, point) == 0.0
        and min(p1[0], p2[0]) <= point[0] <= max(p1[0], p2[0])
        and min(p1[1], p2[1]) <= point[1] <= max(p1[1], p2[1])
        for p1, p2 in zip(polygon, polygon[1:] + polygon[:1])
    )


def _assert_path_in_free_space(path, length, env_params):
    boundary, holes = env_params
    polygons = _polygons(env_params)
    measured = sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in itertools.pairwise(path))
    assert length == pytest.approx(measured, rel=1e-12)
    for a, b in itertools.pairwise(path):
        midpoint = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        if boundary is not None:
            boundary_ = polygons[0]
            assert _strictly_inside_polygon(midpoint, boundary_) or _on_border(
                midpoint, boundary_
            ), f"segment {a} - {b} leaves the boundary"
        for hole in polygons[len(polygons) - len(holes) :]:
            assert not _strictly_inside_polygon(midpoint, hole), (
                f"segment {a} - {b} runs through a hole"
            )
        for polygon in polygons:
            for edge in zip(polygon, polygon[1:] + polygon[:1]):
                assert not _properly_intersects(a, b, *edge), (
                    f"segment {a} - {b} crosses the edge {edge}"
                )


def _unpruned_graph(environment) -> nx.Graph:
    """the visibility graph of the extremities without the elimination of the extremities "in front"

    Not a supergraph of ``environment.graph``: of collinear extremities, the graph construction may also connect
    an extremity to a farther one (it checks only the later extremities), with the same path lengths.
    """
    coords = environment.coords
    extremities = environment.extremity_indices
    graph = nx.Graph()
    graph.add_nodes_from(extremities)
    for origin in extremities:
        representations, distances = utils.cmp_reps_n_distances(origin, coords)
        candidates = set(extremities.tolist()) - {int(origin)}
        _, visible = utils._find_visible_and_in_front(
            origin,
            candidates,
            set(),
            coords,
            representations,
            distances,
            environment.edge_vertex_idxs,
            environment.points,
        )
        for i in visible:
            graph.add_edge(int(origin), int(i), weight=float(distances[i]))
    return graph


def _reference_length(environment, unpruned_graph, start, goal) -> float | None:
    """the shortest path length in the unpruned visibility graph, extended by start and goal"""
    coords = np.append(environment.coords, (start, goal), axis=0)
    idx_start, idx_goal = environment.idx_start, environment.idx_goal
    # no polygon passes a random query point
    query_points = {idx_start: None, idx_goal: None}
    graph = unpruned_graph.copy()
    extremities = set(environment.extremity_indices.tolist())
    for origin, candidates in (
        (idx_start, extremities | {idx_goal}),
        (idx_goal, extremities),
    ):
        representations, distances = utils.cmp_reps_n_distances(origin, coords)
        visible = environment.get_visible_idxs(
            origin, candidates, coords, representations, distances, query_points
        )
        for i in visible:
            graph.add_edge(origin, int(i), weight=float(distances[i]))
    try:
        return nx.dijkstra_path_length(graph, idx_start, idx_goal)
    except (nx.NetworkXNoPath, nx.NodeNotFound):
        return None


def test_paths_between_random_points(env_id, environment, query_points):
    env_params = ENVIRONMENTS[env_id]
    unpruned_graph = _unpruned_graph(environment)
    for start, goal in itertools.combinations(query_points, 2):
        path, length = environment.find_shortest_path(start, goal)
        reverse_path, reverse_length = environment.find_shortest_path(goal, start)
        expected = _reference_length(environment, unpruned_graph, start, goal)
        if expected is None:
            assert (path, length) == ([], None)
            assert (reverse_path, reverse_length) == ([], None)
            continue
        assert length is not None and reverse_length is not None, f"{start} - {goal}"
        assert path[0] == start and path[-1] == goal
        assert reverse_path[0] == goal and reverse_path[-1] == start
        assert length == pytest.approx(expected, rel=1e-12)
        assert reverse_length == pytest.approx(length, rel=1e-12)
        _assert_path_in_free_space(path, length, env_params)
        _assert_path_in_free_space(reverse_path, reverse_length, env_params)


def test_paths_between_extremities_match_the_unpruned_graph(environment):
    """the elimination of extremities "in front" keeps the shortest paths"""
    unpruned_lengths = dict(
        nx.all_pairs_dijkstra_path_length(_unpruned_graph(environment))
    )
    extremities = environment.extremity_indices
    coords = environment.coords
    for i, j in itertools.combinations(extremities, 2):
        start, goal = tuple(coords[i]), tuple(coords[j])
        _, length = environment.find_shortest_path(start, goal, verify=False)
        expected = unpruned_lengths[i].get(j)
        if expected is None:
            assert length is None, f"{start} - {goal}"
        else:
            assert length == pytest.approx(expected, rel=1e-12), f"{start} - {goal}"


def _transformed(env_params, transform):
    boundary, holes = env_params
    boundary = None if boundary is None else [transform(p) for p in boundary]
    return boundary, [[transform(p) for p in hole] for hole in holes]


def _vertex_and_edge_points(environment):
    points = []
    for polygon in [environment.boundary_polygon, *environment.holes]:
        if polygon is None:
            continue
        for p1, p2 in zip(polygon, np.roll(polygon, -1, axis=0)):
            points.append(tuple(map(float, p1)))
            points.append(tuple(map(float, (p1 + p2) / 2.0)))
    return [p for p in points if environment.within_map(np.array(p))]


@pytest.mark.parametrize("exponent", [-3, 5])
def test_scaling_by_a_power_of_two_gives_identical_results(
    env_id, environment, query_points, exponent
):
    factor = 2.0**exponent

    def scale(p):
        return (p[0] * factor, p[1] * factor)

    scaled_env = PolygonEnvironment(*_transformed(ENVIRONMENTS[env_id], scale))
    # including vertices and points on edges: scaling is exact and keeps the contacts
    points = query_points + _vertex_and_edge_points(environment)
    rng = np.random.default_rng(exponent + 10)
    pairs = [
        (points[i], points[j])
        for i, j in rng.choice(len(points), size=(60, 2))
        if i != j
    ]
    pairs += list(itertools.combinations(query_points, 2))
    for start, goal in pairs:
        path, length = environment.find_shortest_path(start, goal, verify=False)
        scaled_path, scaled_length = scaled_env.find_shortest_path(
            scale(start), scale(goal), verify=False
        )
        assert scaled_path == [scale(p) for p in path], f"{start} - {goal}"
        if length is None:
            assert scaled_length is None
        else:
            assert scaled_length == length * factor, f"{start} - {goal}"


def _rotation(angle):
    c, s = cos(angle), sin(angle)
    return lambda p: (c * p[0] - s * p[1], s * p[0] + c * p[1])


INEXACT_TRANSFORMS = {
    "shift by 0.1": lambda p: (p[0] + 0.1, p[1] + 0.1),
    "shift by 1e6": lambda p: (p[0] + 1e6, p[1] + 1e6),
    "scale by 1/3": lambda p: (p[0] / 3.0, p[1] / 3.0),
    "rotate by 1e-3": _rotation(1e-3),
}


@pytest.mark.parametrize("transform_id", INEXACT_TRANSFORMS)
def test_inexact_transforms_keep_reachability_and_lengths(
    env_id, environment, query_points, transform_id
):
    transform = INEXACT_TRANSFORMS[transform_id]
    transformed_env = PolygonEnvironment(*_transformed(ENVIRONMENTS[env_id], transform))
    for start, goal in itertools.combinations(query_points, 2):
        _, length = environment.find_shortest_path(start, goal)
        _, transformed_length = transformed_env.find_shortest_path(
            transform(start), transform(goal)
        )
        if length is None:
            assert transformed_length is None, f"{start} - {goal}"
            continue
        assert transformed_length is not None, f"{start} - {goal}"
        if transform_id == "scale by 1/3":
            length /= 3.0
        assert transformed_length == pytest.approx(
            length, rel=TRANSFORM_REL_TOLERANCE
        ), f"{start} - {goal}"


def _search_without_skipping(environment, monkeypatch, start, goal):
    """the search connecting the query points also to the graph nodes they lie "in front" of"""
    with monkeypatch.context() as m:
        m.setattr(
            environment.points,
            "in_front_of",
            lambda candidates, *_: np.zeros(len(candidates), dtype=bool),
        )
        return environment.search(start, goal, verify=False)


def test_skipping_the_nodes_in_front_of_query_points_keeps_the_paths(
    environment, query_points, monkeypatch
):
    """#222: query points on edges, at vertices and at extremities included"""
    points = query_points + _vertex_and_edge_points(environment)
    rng = np.random.default_rng(222)
    pairs = [
        (points[i], points[j])
        for i, j in rng.choice(len(points), size=(150, 2))
        if i != j
    ]
    pairs += list(itertools.combinations(query_points, 2))
    nr_edges, nr_edges_unskipped = 0, 0
    for start, goal in pairs:
        result = environment.search(start, goal, verify=False)
        expected = _search_without_skipping(environment, monkeypatch, start, goal)
        if expected.length is None:
            assert result.length is None, f"{start} - {goal}"
            continue
        assert result.length == pytest.approx(expected.length, rel=1e-12), (
            f"{start} - {goal}"
        )
        nr_edges += len(result.query_edges or [])
        nr_edges_unskipped += len(expected.query_edges or [])
    # Note: not strictly fewer in every environment (e.g. the goal first shortcut).
    # test_core_regressions.py checks that a node is skipped.
    assert nr_edges <= nr_edges_unskipped


def test_in_front_of_matches_point_in_front(environment, query_points):
    """the vectorised test gives the same result as ``Point.in_front()``"""
    points = environment.points
    extremities = environment.extremity_indices
    for position in query_points + _vertex_and_edge_points(environment):
        coords = np.append(environment.coords, [position], axis=0)
        origin = len(coords) - 1
        representations, distances = utils.cmp_reps_n_distances(origin, coords)
        candidates = extremities[distances[extremities] != 0.0]
        found = points.in_front_of(candidates, coords[origin], coords, representations)
        expected = [
            points.by_idx[i].in_front(
                wedges._rep(coords[origin] - coords[i]), coords[origin] - coords[i]
            )
            for i in candidates
        ]
        assert found.tolist() == expected, f"from {position}"
