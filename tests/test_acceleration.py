"""Ensure compatibility jobs exercise the requested execution backend."""

import importlib.util
import itertools
import os
import subprocess
import sys
import tempfile

import numpy as np
import pytest

from extremitypathfinder import utils_numba
from tests import exact


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


# the determinant rounds to 0, although the exact one is not 0: (p1 - p2) and v are parallel as decimals
# (19.8, -4.4) and (-17.1, 3.8), but not as floats
DETERMINANT_ROUNDS_TO_ZERO = ((18.8, 3.5), (-1.0, 7.9), (-17.1, 3.8000000000000003))


@pytest.mark.filterwarnings("error")
def test_lies_behind_determinant_rounds_to_zero():
    # a direction parallel to the edge (in floating point) is not behind it, in both modes (#3).
    # Previously Numba raised a ZeroDivisionError and plain Python decided by the sign of ±inf.
    p1, p2, v = (np.array(p, dtype=float) for p in DETERMINANT_ROUNDS_TO_ZERO)
    a = p1[0] - p2[0]
    b = p1[1] - p2[1]
    assert a * v[1] - v[0] * b == 0.0
    for edge in ((p1, p2), (p2, p1)):
        for vertex in (v, -v):
            assert not utils_numba._lies_behind_inner(*edge, vertex)


def test_lies_behind_exact_on_integer_coordinates():
    # integer coordinates (e.g. grids): the decision is exact (#185).
    # The vertex lies within or near the angle range of the edge (as when called),
    # on, in front of or behind the edge.
    rng = np.random.default_rng(185)
    nr_checked = 0
    for p1, p2 in rng.integers(-12, 13, size=(400, 2, 2)):
        if p1[0] * p2[1] - p1[1] * p2[0] == 0:
            # the origin lies on the line through the edge
            continue
        # positive combinations of p1 and p2 lie within the angle range,
        # the floor division rounds them to integers (possibly just outside of it).
        # Points p1 + (p2 - p1) k / 4 lie on the edge (when integer) or next to it.
        vertices = [
            k * p1 + l * p2 for k, l in itertools.product(range(1, 4), repeat=2)
        ]
        vertices += [
            (k * p1 + l * p2) // 3 for k, l in itertools.product(range(1, 4), repeat=2)
        ]
        vertices += [p1 + (p2 - p1) * k // 4 for k in range(1, 4)]
        for v in vertices:
            if v[0] * (p1[1] - p2[1]) == v[1] * (p1[0] - p2[0]):
                # parallel to the edge (not possible when called)
                continue
            expected = exact.lies_behind(p1, p2, v)
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
    with tempfile.TemporaryDirectory() as cache_dir:
        # an empty cache: Numba compiles while SciPy is blocked
        env = {**os.environ, "NUMBA_CACHE_DIR": cache_dir}
        subprocess.run([sys.executable, "-c", code], check=True, env=env)
