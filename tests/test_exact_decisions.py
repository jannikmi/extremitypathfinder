"""the floating point decisions are exact on integer coordinates (#3)

Compared with the exact decisions of ``tests/exact.py`` on random integer inputs, including the hard cases:
parallel vectors of different length, directions one unit apart at 2^20 and points exactly on edges.
``_lies_behind_inner()`` is covered in ``test_acceleration.py``.
The audit of all kinds of inputs (decimals, near-degenerate): ``prototypes/float_audit.py``.
"""

import numpy as np
import pytest

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


def test_in_wedges_exactly_near_the_bounding_rays(rng):
    """directions near a bounding ray are compared exactly (``RAY_TOLERANCE``)"""
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
