import numpy as np
import pytest

from extremitypathfinder import utils

# [[ 0  1], [ 1  2], [ 2  3], [ 3  4], [ 4  5], [ 5  6], [ 6  7], [ 7  8], [ 8  9], [ 9 10], [10 11], [11 12], [12 13], [13 14], [14 15], [15  0], [16 17], [17 18], [18 19], [19 16], [20 21], [21 22], [22 23], [23 20], [24 25], [25 26], [26 27], [27 24], [28 29], [29 30], [30 31], [31 28], [32 33], [33 34], [34 35], [35 32]])
vertex_edge_idxs = np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5],


# test function for find_visible_ using @pytest.mark.parametrize
@pytest.mark.parametrize(
    "origin, candidates, edges_to_check, coords, representations, distances, edge_vertex_idxs, vertex_edge_idxs, extremity_mask, expected",
    [
        (
                20,
                {1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
                {1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
                np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0],
                          [5, 0], [6, 0], [7, 0], [8, 0], [9, 0]]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5],
                          [5, 6], [6, 7], [7, 8], [8, 9], [9, 10]]),
                np.array([[0], [0, 1], [1, 2], [2, 3], [3, 4],
                          [4, 5], [5, 6], [6, 7], [7, 8], [8, 9]]),
                np.array([True, False, False, False, False,
                          False, False, False, False, True]),
                {1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
        ),
        (
                20,
                {1, 2, 3, 4, 5, 6, 7, 8, 9, 10},
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
                np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0],
                          [5, 0], [6, 0], [7, 0], [8, 0], [9, 0]]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5],
                          [5, 6], [6, 7], [7, 8], [8, 9], [9, 10]]),
                np.array([[0], [0, 1], [1, 2], [2, 3], [3, 4],
                          [4, 5], [5, 6], [6, 7], [7, 8], [8, 9]]),
                np.array([True, False, False, False, False,
                          False, False, False, False, True]),
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
        ),
        (
                20,
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
                np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0],
                          [5, 0], [6, 0], [7, 0], [8, 0], [9, 0]]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5],
                          [5, 6], [6, 7], [7, 8], [8, 9], [9, 10]]),
                np.array([[0], [0, 1], [1, 2], [2, 3], [3, 4],
                          [4, 5], [5, 6], [6, 7], [7, 8], [8, 9]]),
                np.array([True, False, False, False, False,
                          False, False, False, False, True]),
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
        ),
        (
                20,
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),

                np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0],
                          [5, 0], [6, 0], [7, 0], [8, 0], [9, 0]]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9]),
                np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5],
                          [5, 6], [6, 7], [7, 8], [8, 9], [9, 10]]),
                np.array([[0], [0, 1], [1, 2], [2, 3], [3, 4],
                          [4, 5], [5, 6], [6, 7], [7, 8], [8, 9]]),
                np.array([True, False, False, False, False,
                          False, False, False, False, True]),
                set([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]),
        ),
    ]
)
def test_find_visible(origin,
                      candidates,
                      edges_to_check,
                      coords,
                      representations,
                      distances,
                      edge_vertex_idxs,
                      vertex_edge_idxs,
                      extremity_mask, expected):
    candidates_ = utils.find_visible_(
        origin,
        candidates,
        edges_to_check,
        coords,
        representations,
        distances,
        edge_vertex_idxs,
        vertex_edge_idxs,
        extremity_mask,
    )
    assert candidates_ == expected
