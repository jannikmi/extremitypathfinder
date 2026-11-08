"""Shapely and GeoJSON-like geometries as input: ``PolygonEnvironment.from_shapely()`` (#166).

Skipped when Shapely is not installed (an optional extra, cf. docs/contributing/testing.md).
"""

import json
import sys

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, utils
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

shapely = pytest.importorskip("shapely")
from shapely.geometry import mapping

ENVS_N_TEST_DATA = [
    pytest.param(POLY_ENV_PARAMS, TEST_DATA_POLY_ENV, id="poly"),
    pytest.param(OVERLAP_POLY_ENV_PARAMS, TEST_DATA_OVERLAP_POLY_ENV, id="overlap"),
    pytest.param(SEPARATED_ENV, TEST_DATA_SEPARATE_ENV, id="separated"),
    pytest.param(utils.convert_grid(*GRID_ENV_PARAMS), TEST_DATA_GRID_ENV, id="grid"),
]
SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
OBSTACLE = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
OBSTACLE2 = [(6.0, 6.0), (8.0, 6.0), (8.0, 8.0), (6.0, 8.0)]


def _assert_same_environment(env, expected):
    np.testing.assert_array_equal(env.coords, expected.coords)
    np.testing.assert_array_equal(env.extremity_indices, expected.extremity_indices)
    assert sorted(env.graph.edges(data="weight")) == sorted(
        expected.graph.edges(data="weight")
    )


class _GeoInterface:
    def __init__(self, geometry):
        self.__geo_interface__ = mapping(geometry)


@pytest.mark.parametrize("env_params, test_data", ENVS_N_TEST_DATA)
@pytest.mark.parametrize(
    "as_boundary_polygon", [True, False], ids=["holes-in-boundary", "obstacles"]
)
def test_same_environment_as_coordinates(env_params, test_data, as_boundary_polygon):
    boundary, holes = env_params
    expected = PolygonEnvironment(boundary, holes)
    if as_boundary_polygon:
        env = PolygonEnvironment.from_shapely(shapely.Polygon(boundary, holes))
    else:
        env = PolygonEnvironment.from_shapely(
            shapely.Polygon(boundary), [shapely.Polygon(h) for h in holes]
        )
    _assert_same_environment(env, expected)
    try_test_cases(env, test_data)


def _geometry_inputs():
    obstacles = [shapely.Polygon(OBSTACLE), shapely.Polygon(OBSTACLE2)]
    multi = shapely.MultiPolygon(obstacles)
    collection = {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "geometry": mapping(o), "properties": {}}
            for o in obstacles
        ],
    }
    return [
        pytest.param(obstacles, id="list"),
        pytest.param(multi, id="multipolygon"),
        pytest.param(shapely.GeometryCollection(obstacles), id="collection"),
        pytest.param(
            [shapely.Polygon(OBSTACLE), [shapely.Polygon(OBSTACLE2)]], id="nested"
        ),
        pytest.param(mapping(multi), id="geojson-geometry"),
        pytest.param(json.loads(json.dumps(collection)), id="geojson-features"),
        pytest.param(_GeoInterface(multi), id="geo-interface"),
        pytest.param([_GeoInterface(o) for o in obstacles], id="geo-interface-list"),
        pytest.param(
            [shapely.Polygon(OBSTACLE), shapely.Polygon(), shapely.Polygon(OBSTACLE2)],
            id="with-empty",
        ),
    ]


@pytest.mark.parametrize("obstacles", _geometry_inputs())
@pytest.mark.parametrize("bounded", [True, False], ids=["bounded", "unbounded"])
def test_geometry_types(obstacles, bounded):
    boundary = SQUARE if bounded else None
    expected = PolygonEnvironment(boundary, [OBSTACLE, OBSTACLE2])
    shapely_boundary = shapely.Polygon(SQUARE) if bounded else None
    env = PolygonEnvironment.from_shapely(shapely_boundary, obstacles, validate=True)
    _assert_same_environment(env, expected)


@pytest.mark.parametrize(
    "boundary",
    [
        pytest.param(lambda p: mapping(p), id="geojson"),
        pytest.param(lambda p: _GeoInterface(p), id="geo-interface"),
        pytest.param(lambda p: shapely.MultiPolygon([p]), id="one-part-multipolygon"),
    ],
)
def test_boundary_types(boundary):
    polygon = shapely.Polygon(SQUARE, [OBSTACLE])
    env = PolygonEnvironment.from_shapely(
        boundary(polygon), [shapely.Polygon(OBSTACLE2)]
    )
    _assert_same_environment(env, PolygonEnvironment(SQUARE, [OBSTACLE, OBSTACLE2]))


def test_repeated_vertices_removed():
    """Shapely accepts repeated consecutive vertices, the coordinate input does not"""
    boundary = shapely.Polygon(SQUARE[:2] + [SQUARE[1]] + SQUARE[2:])
    obstacle = shapely.Polygon(OBSTACLE + [OBSTACLE[-1]])
    env = PolygonEnvironment.from_shapely(boundary, obstacle, validate=True)
    _assert_same_environment(env, PolygonEnvironment(SQUARE, [OBSTACLE]))


def test_z_coordinate_ignored():
    boundary = shapely.Polygon([(x, y, 5.0) for x, y in SQUARE])
    env = PolygonEnvironment.from_shapely(boundary, shapely.Polygon(OBSTACLE))
    _assert_same_environment(env, PolygonEnvironment(SQUARE, [OBSTACLE]))


def test_obstacle_interiors_ignored_with_warning():
    lake = [(1.0, 1.0), (9.0, 1.0), (9.0, 9.0), (1.0, 9.0)]
    island = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]
    with pytest.warns(UserWarning, match="ignored 1 interior ring"):
        env = PolygonEnvironment.from_shapely(
            shapely.Polygon(SQUARE), shapely.Polygon(lake, [island])
        )
    _assert_same_environment(env, PolygonEnvironment(SQUARE, [lake]))


def test_no_warning_without_obstacle_interiors(recwarn):
    PolygonEnvironment.from_shapely(shapely.Polygon(SQUARE, [OBSTACLE]))
    assert not [w for w in recwarn if issubclass(w.category, UserWarning)]


@pytest.mark.parametrize(
    "boundary, obstacles",
    [
        pytest.param(shapely.Point(0, 0), None, id="point-boundary"),
        pytest.param(shapely.Polygon(SQUARE), shapely.LineString(OBSTACLE), id="line"),
        pytest.param(None, [shapely.Polygon(OBSTACLE), 5], id="number"),
        pytest.param(None, "POLYGON ((0 0, 1 0, 1 1, 0 0))", id="string"),
    ],
)
def test_wrong_type(boundary, obstacles):
    with pytest.raises(TypeError):
        PolygonEnvironment.from_shapely(boundary, obstacles)


@pytest.mark.parametrize(
    "boundary",
    [
        pytest.param(
            shapely.MultiPolygon(
                [shapely.Polygon(OBSTACLE), shapely.Polygon(OBSTACLE2)]
            ),
            id="two-polygons",
        ),
        pytest.param(shapely.Polygon(), id="empty"),
    ],
)
def test_boundary_not_a_single_polygon(boundary):
    with pytest.raises(ValueError, match="single polygon"):
        PolygonEnvironment.from_shapely(boundary, shapely.Polygon(OBSTACLE))


def test_unbounded_without_obstacles():
    with pytest.raises(ValueError, match="At least one hole"):
        PolygonEnvironment.from_shapely(None, [])


INVALID_INPUTS = [
    pytest.param(
        [(0.0, 0.0), (4.0, 4.0), (4.0, 0.0), (0.0, 4.0)], ValueError, id="bowtie"
    ),
    pytest.param([(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)], ValueError, id="zero-area"),
    pytest.param(
        [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (1.0, 0.0), (0.0, 2.0)],
        None,
        id="self-touching-vertex-on-edge",
    ),
]


@pytest.mark.parametrize("hole, error", INVALID_INPUTS)
def test_same_validation_as_coordinates(hole, error):
    """``validate=True`` accepts and rejects exactly the inputs of the coordinate input"""
    boundary = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]

    def outcome(construct):
        try:
            construct()
        except (TypeError, ValueError) as e:
            return type(e)
        return None

    expected = outcome(lambda: PolygonEnvironment(boundary, [hole], validate=True))
    assert expected is error
    for construct in (
        lambda: PolygonEnvironment.from_shapely(
            shapely.Polygon(boundary), shapely.Polygon(hole), validate=True
        ),
        lambda: PolygonEnvironment.from_shapely(
            shapely.Polygon(boundary, [hole]), validate=True
        ),
    ):
        assert outcome(construct) is expected


def test_missing_shapely(monkeypatch):
    monkeypatch.setitem(sys.modules, "shapely", None)
    with pytest.raises(ImportError, match=r"extremitypathfinder\[shapely\]"):
        PolygonEnvironment.from_shapely(None, [OBSTACLE])


@pytest.mark.parametrize(
    "obstacles",
    [
        pytest.param(
            {"type": "Feature", "geometry": None, "properties": {}}, id="feature"
        ),
        pytest.param(
            {
                "type": "FeatureCollection",
                "features": [
                    {"type": "Feature", "geometry": mapping(shapely.Polygon(OBSTACLE))},
                    {"type": "Feature", "geometry": None, "properties": {}},
                ],
            },
            id="feature-collection",
        ),
    ],
)
def test_null_geometry_skipped(obstacles):
    """GeoJSON features may have a null geometry (e.g. missing values in GeoPandas)"""
    env = PolygonEnvironment.from_shapely(
        shapely.Polygon(SQUARE), [obstacles, shapely.Polygon(OBSTACLE)]
    )
    expected_holes = [OBSTACLE] * (2 if "features" in obstacles else 1)
    _assert_same_environment(env, PolygonEnvironment(SQUARE, expected_holes))


def test_not_geojson_mapping():
    with pytest.raises(TypeError, match="not a GeoJSON geometry"):
        PolygonEnvironment.from_shapely(shapely.Polygon(SQUARE), {"a": 1})


@pytest.mark.parametrize(
    "obstacle",
    [
        pytest.param([(1.0, 1.0)] * 4, id="single-point"),
        pytest.param([(1.0, 1.0), (2.0, 1.0), (2.0, 1.0), (1.0, 1.0)], id="two-points"),
    ],
)
@pytest.mark.parametrize("validate", [True, False])
def test_collapsed_ring(obstacle, validate):
    """rings with fewer than 3 distinct vertices after removing repeated vertices"""
    with pytest.raises(ValueError, match="fewer than 3 distinct vertices"):
        PolygonEnvironment.from_shapely(
            shapely.Polygon(SQUARE), shapely.Polygon(obstacle), validate=validate
        )


def test_repeated_vertex_before_closing():
    """a repeated first vertex before the closing vertex keeps the vertex order"""
    obstacle = shapely.Polygon(OBSTACLE + [OBSTACLE[0]])
    env = PolygonEnvironment.from_shapely(shapely.Polygon(SQUARE), obstacle)
    _assert_same_environment(env, PolygonEnvironment(SQUARE, [OBSTACLE]))
