"""the floating point decisions are exact on integer coordinates (#3)

Compared with the exact decisions of ``tests/exact.py`` on random integer inputs, including the hard cases:
parallel vectors of different length, directions one unit apart at 2^20 and points exactly on edges.
``_lies_behind_inner()`` is covered in ``test_acceleration.py``.
The audit of all kinds of inputs (decimals, near-degenerate): ``prototypes/float_audit.py``.
"""

import itertools

import numpy as np
import pytest

from extremitypathfinder import utils
from extremitypathfinder.utils_numba import (
    _compute_repr_n_dist,
    _has_clockwise_numbering,
    _inside_polygon,
    _lies_on_edge,
    reps_n_distances,
)
from extremitypathfinder.wedges import Point, _rep
from tests import exact

NR_CASES = 2000
LARGE = 2**20


def _sign(x) -> int:
    return int(x > 0) - int(x < 0)


@pytest.fixture
def rng():
    return np.random.default_rng(3)


def _vector_pairs(rng):
    """pairs of non-zero integer vectors: random, parallel of different length, one unit apart at 2^20"""
    u = rng.integers(-LARGE, LARGE + 1, size=(NR_CASES, 2))
    random = rng.integers(-LARGE, LARGE + 1, size=(NR_CASES, 2))
    parallel = u * rng.integers(1, 100, size=(NR_CASES, 1))
    one_unit_apart = u + rng.integers(-1, 2, size=(NR_CASES, 2))
    small = rng.integers(-20, 21, size=(NR_CASES, 2, 2))
    pairs = [
        (u, random),
        (u, parallel),
        (u, one_unit_apart),
        (small[:, 0], small[:, 1]),
    ]
    vectors = np.concatenate([np.stack(p, axis=1) for p in pairs]).astype(float)
    return vectors[np.all(vectors.any(axis=2), axis=1)]


def test_angle_representations_order_directions_exactly(rng):
    pairs = _vector_pairs(rng)
    vectorised, _ = reps_n_distances(pairs.reshape(-1, 2))
    vectorised = vectorised.reshape(-1, 2)
    for (u, v), (rep_u, rep_v) in zip(pairs, vectorised):
        assert _compute_repr_n_dist(u)[0] == rep_u
        assert _compute_repr_n_dist(v)[0] == rep_v
        assert _sign(rep_u - rep_v) == _sign(exact.rep(u) - exact.rep(v)), (u, v)


def test_lies_on_edge_exactly(rng):
    p1 = rng.integers(-LARGE, LARGE + 1, size=(NR_CASES, 2))
    step = rng.integers(-50, 51, size=(NR_CASES, 2))
    p2 = p1 + 3 * step
    # on the edge (vertices included), on its line beyond the vertices, next to it, random
    on_line = p1 + rng.integers(-1, 5, size=(NR_CASES, 1)) * step
    next_to = on_line + rng.integers(-1, 2, size=(NR_CASES, 2))
    random = rng.integers(-LARGE, LARGE + 1, size=(NR_CASES, 2))
    nr_on_edge = 0
    for p in (on_line, next_to, random):
        for point, a, b in np.stack([p, p1, p2], axis=1).astype(float):
            expected = exact.on_edge(point, a, b)
            assert _lies_on_edge(point, a, b) == expected, (point, a, b)
            nr_on_edge += expected
    assert nr_on_edge > NR_CASES / 2


def _star_polygon(rng, scale):
    """a simple polygon with integer vertices: sorted by angle around their mean"""
    while True:
        points = np.unique(
            rng.integers(-scale, scale + 1, size=(rng.integers(3, 12), 2)), axis=0
        )
        center = points.mean(axis=0)
        angles = np.arctan2(*(points - center).T[::-1])
        polygon = points[np.lexsort((np.hypot(*(points - center).T), angles))]
        if len(polygon) >= 3 and exact.signed_area2(polygon) != 0:
            return polygon.astype(float)


@pytest.mark.parametrize("scale", [20, LARGE])
def test_inside_polygon_exactly(rng, scale):
    for _ in range(50):
        polygon = _star_polygon(rng, scale)
        # vertices, points on and next to the edges (integer), random points
        queries = list(polygon)
        for p1, p2 in zip(np.roll(polygon, 1, axis=0), polygon):
            queries.append(p1 + 2 * ((p2 - p1) // 4))
            queries.append(np.floor((p1 + p2) / 2) + rng.integers(-1, 2, size=2))
        queries += list(rng.integers(-scale, scale + 1, size=(50, 2)).astype(float))
        for p in queries:
            p = np.ascontiguousarray(p, dtype=float)
            for border_value in (True, False):
                expected = exact.inside_polygon(p, polygon, border_value)
                assert _inside_polygon(p, polygon, border_value) == expected, (
                    p,
                    polygon,
                )


@pytest.mark.parametrize("scale", [20, LARGE])
def test_clockwise_numbering_exactly(rng, scale):
    for _ in range(500):
        polygon = _star_polygon(rng, scale)
        if rng.integers(2):
            polygon = np.ascontiguousarray(polygon[::-1])
        assert _has_clockwise_numbering(polygon) == (exact.signed_area2(polygon) > 0)


def _crossing_edges(polygon) -> list[tuple[int, int]]:
    """all pairs of non-neighbouring edges crossing in their interiors (exact, all pairs)"""
    n = len(polygon)
    return [
        (i, j)
        for i, j in itertools.combinations(range(n), 2)
        if j - i not in (1, n - 1)
        and exact.crossing(
            polygon[i], polygon[(i + 1) % n], polygon[j], polygon[(j + 1) % n]
        )
    ]


@pytest.mark.parametrize("scale", [3, 20, LARGE])
@pytest.mark.parametrize("max_pairs", [1, 7, utils.MAX_CROSSING_TEST_PAIRS])
def test_find_crossing_exactly(rng, monkeypatch, scale, max_pairs):
    """the bounding box sweep and the crossing test of the validation (#193), in chunks of any size

    Small scales give many vertices on edges, collinear and parallel edges.
    """
    monkeypatch.setattr(utils, "MAX_CROSSING_TEST_PAIRS", max_pairs)
    for _ in range(300):
        nr_vertices = int(rng.integers(3, 16))
        polygon = rng.integers(-scale, scale + 1, size=(nr_vertices, 2)).astype(float)
        expected = _crossing_edges(polygon)
        found = utils._find_crossing(polygon)
        if not expected:
            assert found is None, polygon
            continue
        assert found is not None, polygon
        i, j, position = found
        assert (i, j) in expected, polygon
        # the crossing lies on both edges (up to rounding)
        for edge in (i, j):
            p1, p2 = polygon[edge], polygon[(edge + 1) % nr_vertices]
            d1, d2 = p2 - p1, position - p1
            assert abs(d1[0] * d2[1] - d1[1] * d2[0]) <= 1e-9 * np.dot(d1, d1)
            assert 0.0 < np.dot(d1, d2) < np.dot(d1, d1)


def test_in_wedges_exactly_near_the_bounding_rays(rng):
    """directions near a bounding ray are compared exactly (``RAY_WINDOW``)"""
    start, end = rng.integers(-LARGE, LARGE + 1, size=(2, NR_CASES, 2))
    factors = rng.integers(1, 100, size=NR_CASES)
    units = rng.integers(-1, 2, size=(NR_CASES, 2))
    for a, b, k, unit in zip(start, end, factors, units):
        a, b = a.astype(float), b.astype(float)
        rep_a, rep_b = _rep(a), _rep(b)
        if not (a.any() and b.any()) or rep_a == rep_b:
            continue
        point = Point(0, 2, set(), [(rep_a, rep_b)], None, [(a, rep_a), (b, rep_b)])
        for bound in (a, b):
            for direction in (k * bound, bound + unit, -bound):
                if not direction.any():
                    continue
                expected = exact.in_closure(direction, a, b)
                assert (
                    point.in_wedges(_rep(direction), direction, [(rep_a, rep_b)])
                    == expected
                )
                mask = point.in_wedges_mask(
                    np.array([_rep(direction)]), direction[None, :], [(rep_a, rep_b)]
                )
                assert mask.tolist() == [expected]


# exactly collinear float vectors (cross product 0.0) whose representations differ in the last bit
# (found by a search over random ray directions and integer factors, full precision)
LAST_BIT_RAYS = [
    (
        np.array([2.9437902314850017, 2.307702229625077]),
        np.array([100.08886787049006, 78.46187580725262]),
    ),
    (
        np.array([7.798711114410413, 6.44747655086141]),
        np.array([374.3381334916998, 309.47887444134767]),
    ),
    (
        np.array([0.25517446524156107, 8.582084415940123]),
        np.array([3.572442513381855, 120.14918182316173]),
    ),
]


@pytest.mark.parametrize(("ray", "same"), LAST_BIT_RAYS)
def test_in_wedges_collinear_last_bit(ray, same):
    """a direction exactly on a bounding ray lies in the closure of its wedge, even if its own representation
    differs from the ray's in the last bit (``RAY_WINDOW``, ``Point.rep()``)"""
    assert ray[0] * same[1] - ray[1] * same[0] == 0.0
    rep = _rep(ray)
    assert _rep(same) != rep
    point = Point(0, 2, set(), [], None, [(ray, rep)])
    same_rep = _rep(same)
    # the ray is the end of the first wedge and the start of the second
    assert point.in_wedges(same_rep, same, [(rep - 0.25, rep)])
    assert point.in_wedges(same_rep, same, [(rep, rep + 0.25)])
    for wedge in ((rep - 0.25, rep), (rep, rep + 0.25)):
        assert point.in_wedges_mask(np.array([same_rep]), same[None, :], [wedge]).all()


def test_in_front_mask_exactly_opposite_of_a_bounding_ray():
    """a direction exactly opposite of a bounding ray does not lie in front, even if its representation
    lies within the front range by a difference in the last bit: the vectorised test decides it like
    ``Point.in_front()`` (#262)"""
    within_range = []
    for ray, same in LAST_BIT_RAYS:
        rep = _rep(ray)
        opposite = -same
        opposite_rep = _rep(opposite)
        normal = np.array([-ray[1], ray[0]])
        # the ray as the end and as the start of an extremity wedge (larger than 180 degree)
        for wedge, side in (
            (((rep - 2.5) % 4.0, rep), 0.1),
            ((rep, (rep + 2.5) % 4.0), -0.1),
        ):
            point = Point(0, 1, set(), [wedge], wedge, [(ray, rep)])
            assert not point.in_front(opposite_rep, opposite)
            mask = point.in_front_mask(np.array([opposite_rep]), opposite[None, :])
            assert mask.tolist() == [False]
            start, end = wedge
            front_start = (end + 2.0) % 4.0
            size = ((start + 2.0) % 4.0 - front_start) % 4.0
            within_range.append(0.0 < (opposite_rep - front_start) % 4.0 < size)
            # a direction clearly within the front range lies in front
            inside = -(ray + side * normal)
            assert point.in_front(_rep(inside), inside)
            assert point.in_front_mask(np.array([_rep(inside)]), inside[None, :]).all()
    # the comparison of the representations alone would be wrong for some of these cases
    assert any(within_range)
