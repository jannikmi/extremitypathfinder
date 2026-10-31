"""Polygons are accepted with clockwise and counter clockwise vertex numbering (#161)."""

import itertools

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, utils
from extremitypathfinder.utils_numba import _has_clockwise_numbering
from tests.main_test import try_test_cases
from tests.test_cases import (
    GRID_ENV_PARAMS,
    OVERLAP_POLY_ENV_PARAMS,
    POLY_ENV_PARAMS,
    SEPARATED_ENV,
    TEST_DATA_GRID_ENV,
    TEST_DATA_OVERLAP_POLY_ENV,
    TEST_DATA_POLY_ENV,
    TEST_DATA_SEPARATE_ENV,
)

GRID_POLYGONS = utils.convert_grid(*GRID_ENV_PARAMS)
ENVS_N_TEST_DATA = [
    pytest.param(POLY_ENV_PARAMS, TEST_DATA_POLY_ENV, id="poly"),
    pytest.param(OVERLAP_POLY_ENV_PARAMS, TEST_DATA_OVERLAP_POLY_ENV, id="overlap"),
    pytest.param(SEPARATED_ENV, TEST_DATA_SEPARATE_ENV, id="separated"),
    pytest.param(GRID_POLYGONS, TEST_DATA_GRID_ENV, id="grid"),
]
UNBOUNDED_HOLES = [
    [(0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)],
    [(4.0, 3.0), (5.0, 3.0), (5.0, -1.0), (4.0, -1.0)],
]


def _reversed(polygon):
    return list(reversed([tuple(coord) for coord in polygon]))


def _assert_orientation(environment):
    if environment.boundary_polygon is not None:
        assert not _has_clockwise_numbering(environment.boundary_polygon)
    for hole in environment.holes:
        assert _has_clockwise_numbering(hole)


@pytest.mark.parametrize("validate", [False, True])
@pytest.mark.parametrize(
    ("reverse_boundary", "reverse_holes"),
    list(itertools.product([False, True], repeat=2)),
)
@pytest.mark.parametrize(("env_params", "test_data"), ENVS_N_TEST_DATA)
def test_either_vertex_numbering(
    env_params, test_data, reverse_boundary, reverse_holes, validate
):
    boundary, holes = env_params
    if reverse_boundary:
        boundary = _reversed(boundary)
    if reverse_holes:
        holes = [_reversed(hole) for hole in holes]
    environment = PolygonEnvironment(boundary, holes, validate=validate)
    _assert_orientation(environment)
    try_test_cases(environment, test_data)


@pytest.mark.parametrize("validate", [False, True])
def test_mixed_hole_numbering(validate):
    boundary, holes = GRID_POLYGONS
    assert len(holes) > 1
    # every other hole reversed
    holes = [_reversed(hole) if i % 2 else hole for i, hole in enumerate(holes)]
    environment = PolygonEnvironment(_reversed(boundary), holes, validate=validate)
    _assert_orientation(environment)
    try_test_cases(environment, TEST_DATA_GRID_ENV)


@pytest.mark.parametrize("validate", [False, True])
@pytest.mark.parametrize("reverse_holes", [False, True])
def test_unbounded_either_vertex_numbering(reverse_holes, validate):
    holes = UNBOUNDED_HOLES
    if reverse_holes:
        holes = [_reversed(hole) for hole in holes]
    environment = PolygonEnvironment(None, holes, validate=validate)
    _assert_orientation(environment)
    assert environment.boundary_polygon is None
    assert not environment.within_map(np.array((1.0, 1.0)))
    assert environment.within_map(np.array((-100.0, 1.0)))
    path, length = environment.find_shortest_path((-1.0, 0.5), (3.0, 0.5))
    assert path == [(-1.0, 0.5), (0.0, 0.0), (2.0, 0.0), (3.0, 0.5)]
    assert length == pytest.approx(2 * np.sqrt(1.25) + 2)


def test_correct_numbering_unchanged():
    boundary, holes = POLY_ENV_PARAMS
    environment = PolygonEnvironment(boundary, holes)
    np.testing.assert_array_equal(environment.boundary_polygon, boundary)
    np.testing.assert_array_equal(environment.holes[0], holes[0])


def test_reversed_polygons_stored_reversed():
    boundary, holes = POLY_ENV_PARAMS
    environment = PolygonEnvironment(
        _reversed(boundary), [_reversed(hole) for hole in holes]
    )
    np.testing.assert_array_equal(environment.boundary_polygon, boundary)
    np.testing.assert_array_equal(environment.holes[0], holes[0])


def test_input_not_modified():
    boundary = np.array(_reversed(POLY_ENV_PARAMS[0]))
    original = boundary.copy()
    PolygonEnvironment(boundary, [])
    np.testing.assert_array_equal(boundary, original)


@pytest.mark.parametrize(
    "coords",
    [
        np.empty((0, 2)),
        np.array([0.0, 1.0, 2.0]),
        np.array([(0.0, 0.0), (1.0, 0.0)]),
        np.array([(0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]),
    ],
)
@pytest.mark.parametrize("clockwise", [False, True])
def test_orient_polygon_ignores_invalid_shapes(coords, clockwise):
    # left for the validation to reject
    assert utils.orient_polygon(coords, clockwise) is coords


def test_orientation_large_coordinates():
    # a small polygon far from the origin: without translating the coordinates
    # the orientation test suffers from catastrophic cancellation
    offset = 1e9
    ulp = np.spacing(offset)
    triangle = offset + ulp * np.array([(6.0, -4.0), (-19.0, 31.0), (-15.0, 26.0)])
    assert _has_clockwise_numbering(triangle)
    assert not _has_clockwise_numbering(triangle[::-1].copy())
