"""Ensure compatibility jobs exercise the requested execution backend."""

import os

import numpy as np
import pytest

from extremitypathfinder import utils_numba


def test_execution_backend():
    expected = os.environ.get("EXPECT_NUMBA")
    if expected is None:
        pytest.skip("backend assertion is configured by tox")
    compiled = hasattr(utils_numba._lies_behind_inner, "nopython_signatures")
    assert compiled == (expected == "1")
    if compiled:
        assert utils_numba._lies_behind_inner.nopython_signatures


def test_linear_solve_visibility():
    # The ray crosses the edge at half its length. This also exercises
    # Numba's SciPy-backed solve when the acceleration extra is installed.
    p1 = np.array([1.0, -1.0])
    p2 = np.array([1.0, 1.0])
    assert utils_numba._lies_behind_inner(p1, p2, np.array([2.0, 0.0]))
    assert not utils_numba._lies_behind_inner(p1, p2, np.array([0.5, 0.0]))
