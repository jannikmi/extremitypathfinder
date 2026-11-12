"""Li Chao tree of edges over the candidates of a visibility check (the core of AREA, cf. ``utils.find_visible()``)

Problem: which of ``k`` candidates (points seen from an origin, sorted by direction) lie behind one of ``n`` edges?
Each edge only matters for the candidates within its angle range: a contiguous range of the sorted candidates.
Checking every edge against every candidate of its range takes ``O(n k)``, e.g. for long walls behind many obstacles.

The tree
--------
A segment tree over the sorted candidates: node ``i`` has the children ``2i`` and ``2i + 1``, the leaves start at
``size`` (a power of two, at least ``k``). Every node stands for a contiguous range of candidates, the root for
all of them. The range of an edge is covered exactly by at most two nodes per level ("canonical nodes").

A node stores at most one edge: of all edges inserted into it, the closest one in the middle direction
of the node. This is the Li Chao tree [1]_ (a segment tree storing one function per node), with the
distance of an edge along the ray through a candidate in place of a linear function.
Seen from the origin, two edges swap their order along the rotating ray at most once (two lines cross at most once).
The edge losing in the middle can therefore only be closer on one side of the middle: it is passed on to that
child, where the same happens again. Inserting an edge takes ``O(log k)`` per canonical node.

The closest edge in the direction of a candidate is then stored in one of the nodes between its leaf and the root.
Only these edges remain to be checked against the candidate (``O(log k)``).

Details that keep it fast and exact
-----------------------------------
- Edges are only compared in the directions of candidates, with the formula of ``utils_numba._lies_behind_inner()``.
  No crossings are computed: overlapping polygons (crossing edges) need no special treatment.
  A candidate is only removed by the check of a single edge (``utils_numba._behind_edge()``).
- The nodes up to ``DIRECT_CHECK_LEVEL`` above the leaves store no edge: an edge arriving there is checked against
  their at most ``DIRECT_CHECK_SIZE`` candidates right away, like every edge with a small range.
  Typical maps have few candidates per edge, and most edges never reach the tree.
- An edge entirely behind the stored edge (its line is farther away than both vertices of the stored one) is dropped,
  an edge entirely in front replaces it. Inserting the close edges first drops most hidden edges at once.
- Candidates touching edges (e.g. query points on an edge) are not blocked by these ("own") edges.

Worst case ``O((r log k + k) log k)`` for ``r`` ranges (at most two per edge), in practice much faster.

.. [1] Li Chao tree, e.g. https://cp-algorithms.com/geometry/convex_hull_trick.html#li-chao-tree

Layer: above ``utils_numba`` (the edge and candidate checks), used by ``utils.find_visible()``.
All functions are compiled by Numba, if installed (cf. ``numba_replacements.py``).
"""

import numpy as np

try:
    from numba import b1, f8, i8, njit, void
except ImportError:
    from extremitypathfinder.numba_replacements import b1, f8, i8, njit, void

from extremitypathfinder.utils_numba import (
    EDGE_MAX_DIST,
    EDGE_MIN_DIST,
    _behind_edge,
    _edge_distance,
)

# nodes of the tree up to this height above the leaves store no edge:
# their (at most ``DIRECT_CHECK_SIZE``) candidates are checked right away.
# Measured with ``prototypes/complexity_profile.py`` (grid and worst case maps): values from 2 to 7 make no measurable
# difference, the setup per origin dominates (#259).
DIRECT_CHECK_LEVEL = 4
DIRECT_CHECK_SIZE = 2**DIRECT_CHECK_LEVEL

NO_EDGE = -1


@njit(i8(i8), cache=True)
def tree_size(nr_candidates: int) -> int:
    """the number of leaves: the smallest power of two not below the number of candidates"""
    size = 1
    while size < nr_candidates:
        size *= 2
    return size


@njit(
    void(b1[:], i8, i8, i8, f8[:, :], f8[:, :], i8[:], i8[:]),
    cache=True,
)
def _check_candidates_directly(
    blocked: np.ndarray,
    first: int,
    last: int,
    edge: int,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> None:
    """marks the not yet blocked candidates at the positions ``first`` to ``last`` (inclusive) behind the edge"""
    for candidate in range(first, last + 1):
        if not blocked[candidate]:
            blocked[candidate] = _behind_edge(
                edge,
                candidate,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )


@njit(
    void(i8[:], b1[:], i8, i8, i8, f8[:, :], f8[:, :], i8[:], i8[:]),
    cache=True,
)
def _insert_edge(
    tree: np.ndarray,
    blocked: np.ndarray,
    node: int,
    level: int,
    edge: int,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> None:
    """adds an edge spanning all directions of a node to the tree (cf. the module docstring)

    :param level: the height of the node above the leaves
    """
    size = len(tree)
    while level > DIRECT_CHECK_LEVEL:
        stored = tree[node]
        if stored == NO_EDGE:
            tree[node] = edge
            return
        # an edge farther away than both vertices of another edge lies behind it in all directions of the node.
        # It blocks no candidate the other edge does not block (cf. ``_behind_edge()``)
        if edge_terms[stored, EDGE_MAX_DIST] < edge_terms[edge, EDGE_MIN_DIST]:
            return
        if edge_terms[edge, EDGE_MAX_DIST] < edge_terms[stored, EDGE_MIN_DIST]:
            tree[node] = edge
            return
        # the candidates of the node: [first, first + 2^level), the left child holds those up to the middle
        first = (node << level) - size
        middle = first + (1 << level) // 2 - 1
        closer_in_middle = _edge_distance(
            edge, middle, edge_terms, candidate_terms
        ) < _edge_distance(stored, middle, edge_terms, candidate_terms)
        closer_at_first = _edge_distance(
            edge, first, edge_terms, candidate_terms
        ) < _edge_distance(stored, first, edge_terms, candidate_terms)
        if closer_in_middle:
            tree[node] = edge
            edge = stored
        # the loser (``edge``) can only be closer on one side of the middle: pass it on to that child.
        # Closer at the first candidate and the middle alike (or at neither): the order does not change
        # up to the middle, if at all behind it.
        level -= 1
        node *= 2
        if closer_at_first == closer_in_middle:
            node += 1
    first = (node << level) - size
    _check_candidates_directly(
        blocked,
        first,
        first + (1 << level) - 1,
        edge,
        edge_terms,
        candidate_terms,
        own_edges_start,
        own_edges,
    )


@njit(
    void(i8[:], b1[:], i8, i8, i8, f8[:, :], f8[:, :], i8[:], i8[:]),
    cache=True,
)
def _insert_range(
    tree: np.ndarray,
    blocked: np.ndarray,
    first: int,
    last: int,
    edge: int,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> None:
    """adds an edge to the nodes which together cover exactly the candidates ``first`` to ``last`` (inclusive)"""
    size = len(tree)
    first += size
    last += size
    level = 0
    # bottom-up: at most two nodes per level
    while first <= last:
        if first & 1:
            _insert_edge(
                tree,
                blocked,
                first,
                level,
                edge,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )
            first += 1
        if not last & 1:
            _insert_edge(
                tree,
                blocked,
                last,
                level,
                edge,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )
            last -= 1
        first >>= 1
        last >>= 1
        level += 1


@njit(i8[:](i8[:]), cache=True)
def _nearest_stored_ancestors(tree: np.ndarray) -> np.ndarray:
    """per node (and leaf) the closest node above it which stores an edge (0: none)

    Most nodes are empty: this skips them when walking from a leaf to the root.
    """
    size = len(tree)
    above = np.zeros(2 * size, dtype=np.int64)
    first = 1
    while first < size:
        # level by level
        nodes = np.arange(first, 2 * first)
        stored_above = np.where(
            tree[first : 2 * first] != NO_EDGE, nodes, above[first : 2 * first]
        )
        above[2 * first : 4 * first] = np.repeat(stored_above, 2)
        first *= 2
    return above


@njit(b1[:](i8[:], i8[:], i8[:], f8[:, :], f8[:, :], i8[:], i8[:]), cache=True)
def blocked_candidates(
    range_edges: np.ndarray,
    range_first: np.ndarray,
    range_last: np.ndarray,
    edge_terms: np.ndarray,
    candidate_terms: np.ndarray,
    own_edges_start: np.ndarray,
    own_edges: np.ndarray,
) -> np.ndarray:
    """which candidates lie behind an edge (cf. the module docstring)

    :param range_edges: the edges of the ranges. Close edges first (cf. ``utils._candidate_ranges()``).
    :param range_first: the position of the first candidate of every range (candidates sorted by direction)
    :param range_last: the position of the last candidate (inclusive) of every range.
        The candidates of a range lie strictly within the angle range of its edge.
    :param edge_terms: per edge (row: the edge index), cf. ``utils_numba.compute_edge_terms()``
    :param candidate_terms: per candidate (in sorted order), cf. ``utils_numba.compute_candidate_terms()``
    :param own_edges_start: the edges touching candidate i, which do not block it (e.g. a query point on an edge):
        ``own_edges[own_edges_start[i] : own_edges_start[i + 1]]``.
        IMPORTANT: the candidate must lie on these edges. An edge in front of it would be taken for the closest one
        in its direction, and the edges behind that edge would not be checked.
    :return: per candidate whether it lies behind an edge
    """
    nr_candidates = len(candidate_terms)
    blocked = np.zeros(nr_candidates, dtype=np.bool_)
    tree = np.full(tree_size(nr_candidates), NO_EDGE, dtype=np.int64)

    for i in range(len(range_edges)):
        edge = int(range_edges[i])
        first = int(range_first[i])
        last = int(range_last[i])
        if last - first < DIRECT_CHECK_SIZE:
            # the most frequent case: a few candidates are checked right away
            _check_candidates_directly(
                blocked,
                first,
                last,
                edge,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )
        else:
            _insert_range(
                tree,
                blocked,
                first,
                last,
                edge,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )

    # the edges stored on the way from the leaf of a candidate to the root remain to be checked
    size = len(tree)
    above = _nearest_stored_ancestors(tree)
    for candidate in range(nr_candidates):
        node = above[candidate + size]
        while node > 0 and not blocked[candidate]:
            blocked[candidate] = _behind_edge(
                tree[node],
                candidate,
                edge_terms,
                candidate_terms,
                own_edges_start,
                own_edges,
            )
            node = above[node]
    return blocked
