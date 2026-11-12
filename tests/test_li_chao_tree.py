"""Tests of the Li Chao tree of edges over the candidates (``li_chao_tree.py``), independent of ``utils.find_visible()``"""

import numpy as np
import pytest

from extremitypathfinder import li_chao_tree
from extremitypathfinder.utils_numba import _behind_edge


def _terms(rng, nr_edges, nr_candidates):
    """random edges in front of or behind candidates on the positive x side, sorted by direction"""
    angles = np.sort(rng.uniform(-1.0, 1.0, nr_candidates))
    radii = rng.uniform(1.0, 10.0, nr_candidates)
    candidates = np.column_stack([radii * np.cos(angles), radii * np.sin(angles)])
    p1 = np.column_stack([rng.uniform(1, 10, nr_edges), rng.uniform(-3, 3, nr_edges)])
    p2 = np.column_stack([rng.uniform(1, 10, nr_edges), rng.uniform(-3, 3, nr_edges)])
    a = p1[:, 0] - p2[:, 0]
    b = p1[:, 1] - p2[:, 1]
    numerator = a * p1[:, 1] - b * p1[:, 0]
    max_dist = np.maximum(np.hypot(*p1.T), np.hypot(*p2.T))
    min_dist = np.abs(numerator) / np.hypot(a, b)
    edge_terms = np.column_stack([a, b, numerator, max_dist, min_dist])
    candidate_terms = np.column_stack([candidates, np.hypot(*candidates.T)])
    return edge_terms, candidate_terms, p1, p2


def _ranges(edge_terms, candidate_terms, p1, p2):
    """all candidates strictly between the directions of the vertices of every edge, close edges first"""
    cand_angle = np.arctan2(candidate_terms[:, 1], candidate_terms[:, 0])
    edges, firsts, lasts = [], [], []
    for edge in np.argsort(edge_terms[:, 3], kind="stable"):
        lo, hi = sorted(
            [np.arctan2(p1[edge, 1], p1[edge, 0]), np.arctan2(p2[edge, 1], p2[edge, 0])]
        )
        inside = np.flatnonzero((cand_angle > lo) & (cand_angle < hi))
        if len(inside):
            edges.append(edge)
            firsts.append(inside[0])
            lasts.append(inside[-1])
    return [np.array(x, dtype=np.int64) for x in (edges, firsts, lasts)]


@pytest.mark.parametrize("nr_candidates", [1, 2, 17, 64, 100, 300])
@pytest.mark.parametrize("seed", range(5))
def test_blocked_candidates_equal_brute_force(seed, nr_candidates):
    rng = np.random.default_rng(seed)
    edge_terms, candidate_terms, p1, p2 = _terms(rng, 60, nr_candidates)
    edges, firsts, lasts = _ranges(edge_terms, candidate_terms, p1, p2)
    no_own_edges = (
        np.zeros(nr_candidates + 1, dtype=np.int64),
        np.zeros(0, dtype=np.int64),
    )

    blocked = li_chao_tree.blocked_candidates(
        edges, firsts, lasts, edge_terms, candidate_terms, *no_own_edges
    )

    expected = np.zeros(nr_candidates, dtype=bool)
    for edge, first, last in zip(edges, firsts, lasts):
        for candidate in range(first, last + 1):
            expected[candidate] |= _behind_edge(
                edge, candidate, edge_terms, candidate_terms, *no_own_edges
            )
    assert blocked.tolist() == expected.tolist()


@pytest.mark.parametrize(
    "nr_candidates, size", [(0, 1), (1, 1), (2, 2), (3, 4), (64, 64), (65, 128)]
)
def test_tree_size(nr_candidates, size):
    assert li_chao_tree.tree_size(nr_candidates) == size
