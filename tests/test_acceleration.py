"""Ensure compatibility jobs exercise the requested execution backend."""

import importlib.util
import itertools
import os
import subprocess
import sys
from fractions import Fraction

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
    # The ray crosses the edge at half its length.
    p1 = np.array([1.0, -1.0])
    p2 = np.array([1.0, 1.0])
    assert utils_numba._lies_behind_inner(p1, p2, np.array([2.0, 0.0]))
    assert not utils_numba._lies_behind_inner(p1, p2, np.array([0.5, 0.0]))


def _lies_behind_exact(p1, p2, v) -> bool:
    p1, p2, v = ([Fraction(int(c)) for c in p] for p in (p1, p2, v))
    a = p1[0] - p2[0]
    b = p1[1] - p2[1]
    mu = (a * p1[1] - b * p1[0]) / (a * v[1] - v[0] * b)
    return mu < 1


# a vertex exactly on the edge (mu == 1) is not behind it.
# np.linalg.solve (LAPACK, used before #185) decided these cases wrongly: mu < 1
VERTICES_ON_EDGE = [
    ((-8, -7), (0, 3), (-4, -2)),
    ((-8, -8), (4, 7), (0, 2)),
    ((-8, -7), (2, -1), (-3, -4)),
    ((-8, -7), (7, 5), (2, 1)),
]


@pytest.mark.parametrize("p1, p2, v", VERTICES_ON_EDGE)
def test_lies_behind_vertex_on_edge(p1, p2, v):
    assert not utils_numba._lies_behind_inner(
        np.array(p1, dtype=float), np.array(p2, dtype=float), np.array(v, dtype=float)
    )


def test_lies_behind_exact_on_integer_coordinates():
    # integer coordinates (e.g. grids): the decision is exact (#185).
    # The vertex lies within the angle range of the edge (as when called), on, in front of or behind it.
    rng = np.random.default_rng(185)
    nr_checked = 0
    for p1, p2 in rng.integers(-12, 13, size=(400, 2, 2)):
        if p1[0] * p2[1] - p1[1] * p2[0] == 0:
            # the origin lies on the line through the edge
            continue
        for k, l in itertools.product(range(1, 4), range(1, 4)):
            # positive combinations of p1 and p2 lie within the angle range
            for v in (k * p1 + l * p2, (k * p1 + l * p2) // 3, p1 + (p2 - p1) * k // 4):
                if v[0] * (p1[1] - p2[1]) == v[1] * (p1[0] - p2[0]):
                    # parallel to the edge (not possible when called)
                    continue
                expected = _lies_behind_exact(p1, p2, v)
                result = utils_numba._lies_behind_inner(
                    p1.astype(float), p2.astype(float), v.astype(float)
                )
                assert result == expected, (p1, p2, v)
                nr_checked += 1
    assert nr_checked > 5000


@pytest.mark.skipif(
    importlib.util.find_spec("numba") is None, reason="Numba is not installed"
)
def test_numba_without_scipy():
    # the numba extra does not install SciPy (#185): compiling must not need it
    code = (
        "import sys\n"
        "sys.modules['scipy'] = None\n"
        "import numpy as np\n"
        "from extremitypathfinder import PolygonEnvironment, utils_numba\n"
        "assert utils_numba._lies_behind_inner.nopython_signatures\n"
        "env = PolygonEnvironment([(0, 0), (4, 0), (4, 4), (0, 4)], [[(1, 1), (3, 1), (2, 3)]])\n"
        "path, length = env.find_shortest_path((0.5, 2), (3.5, 2))\n"
        "assert len(path) == 3, path\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)
