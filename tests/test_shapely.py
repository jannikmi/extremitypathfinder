"""Shapely and GeoJSON-like geometries as input: ``PolygonEnvironment.from_shapely()`` (#166).

Skipped when Shapely is not installed (an optional extra, cf. docs/contributing/testing.md).
"""

import json
import sys

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment
from tests.main_test import try_test_cases
from tests.test_cases import ENVS_N_TEST_DATA
from tests.test_contacts import (
    BOUNDARY,
    OVERLAPPING_EDGES_HOLES,
    SELF_TOUCHING_HOLE,
    SHARED_VERTEX_HOLES,
    TOUCHING_TIPS_HOLES,
    VERTEX_ON_EDGE_HOLES,
)
from tests.test_geometry_regressions import INVALID_POLYGON_ENVS

shapely = pytest.importorskip("shapely")
from shapely.geometry import mapping

OBSTACLE, OBSTACLE2 = SHARED_VERTEX_HOLES


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
    boundary = BOUNDARY if bounded else None
    expected = PolygonEnvironment(boundary, [OBSTACLE, OBSTACLE2])
    shapely_boundary = shapely.Polygon(BOUNDARY) if bounded else None
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
    polygon = shapely.Polygon(BOUNDARY, [OBSTACLE])
    env = PolygonEnvironment.from_shapely(
        boundary(polygon), [shapely.Polygon(OBSTACLE2)]
    )
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, [OBSTACLE, OBSTACLE2]))


def test_repeated_vertices_removed():
    """Shapely accepts repeated consecutive vertices, the coordinate input does not"""
    boundary = shapely.Polygon(BOUNDARY[:2] + [BOUNDARY[1]] + BOUNDARY[2:])
    obstacle = shapely.Polygon(OBSTACLE + [OBSTACLE[-1]])
    env = PolygonEnvironment.from_shapely(boundary, obstacle, validate=True)
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, [OBSTACLE]))


def test_z_coordinate_ignored():
    boundary = shapely.Polygon([(x, y, 5.0) for x, y in BOUNDARY])
    env = PolygonEnvironment.from_shapely(boundary, shapely.Polygon(OBSTACLE))
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, [OBSTACLE]))


def test_obstacle_interiors_ignored_with_warning():
    lake = [(1.0, 1.0), (9.0, 1.0), (9.0, 9.0), (1.0, 9.0)]
    island = [(4.0, 4.0), (6.0, 4.0), (6.0, 6.0), (4.0, 6.0)]
    with pytest.warns(UserWarning, match="ignored 1 interior ring"):
        env = PolygonEnvironment.from_shapely(
            shapely.Polygon(BOUNDARY), shapely.Polygon(lake, [island])
        )
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, [lake]))


def test_no_warning_without_obstacle_interiors(recwarn):
    PolygonEnvironment.from_shapely(shapely.Polygon(BOUNDARY, [OBSTACLE]))
    assert not [w for w in recwarn if issubclass(w.category, UserWarning)]


@pytest.mark.parametrize(
    "boundary, obstacles",
    [
        pytest.param(shapely.Point(0, 0), None, id="point-boundary"),
        pytest.param(
            shapely.Polygon(BOUNDARY), shapely.LineString(OBSTACLE), id="line"
        ),
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


def _to_shapely(boundary, holes):
    return shapely.Polygon(boundary), [shapely.Polygon(hole) for hole in holes]


@pytest.mark.parametrize(
    "holes",
    [
        pytest.param(SHARED_VERTEX_HOLES, id="shared-vertex"),
        pytest.param([SELF_TOUCHING_HOLE], id="self-touching"),
        pytest.param(VERTEX_ON_EDGE_HOLES, id="vertex-on-edge"),
        pytest.param(OVERLAPPING_EDGES_HOLES, id="overlapping-edges"),
        pytest.param(TOUCHING_TIPS_HOLES, id="touching-tips"),
    ],
)
def test_contacts_accepted(holes):
    """touching polygons are valid input here (Shapely rejects a self-touching polygon)"""
    env = PolygonEnvironment.from_shapely(*_to_shapely(BOUNDARY, holes), validate=True)
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, holes, validate=True))


# Shapely warns about NaN coordinates
@pytest.mark.filterwarnings("ignore:invalid value encountered:RuntimeWarning")
@pytest.mark.parametrize(
    ("boundary", "holes", "error", "message"), INVALID_POLYGON_ENVS
)
def test_same_validation_as_coordinates(boundary, holes, error, message):
    """``validate=True`` rejects the invalid coordinate input with the same error"""
    try:
        shapely_boundary, obstacles = _to_shapely(boundary, holes)
    except (TypeError, ValueError):
        pytest.skip("not representable as a Shapely polygon")
    if "must not be identical" in message:
        # Shapely accepts repeated consecutive vertices: from_shapely() removes them
        PolygonEnvironment.from_shapely(shapely_boundary, obstacles, validate=True)
        return
    with pytest.raises(error, match=message):
        PolygonEnvironment.from_shapely(shapely_boundary, obstacles, validate=True)


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
        shapely.Polygon(BOUNDARY), [obstacles, shapely.Polygon(OBSTACLE)]
    )
    expected_holes = [OBSTACLE] * (2 if "features" in obstacles else 1)
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, expected_holes))


def test_not_geojson_mapping():
    with pytest.raises(TypeError, match="not a GeoJSON geometry"):
        PolygonEnvironment.from_shapely(shapely.Polygon(BOUNDARY), {"a": 1})


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
            shapely.Polygon(BOUNDARY), shapely.Polygon(obstacle), validate=validate
        )


def test_repeated_vertex_before_closing():
    """a repeated first vertex before the closing vertex keeps the vertex order"""
    obstacle = shapely.Polygon(OBSTACLE + [OBSTACLE[0]])
    env = PolygonEnvironment.from_shapely(shapely.Polygon(BOUNDARY), obstacle)
    _assert_same_environment(env, PolygonEnvironment(BOUNDARY, [OBSTACLE]))


# the agent clearance recipe of the usage documentation (#167)
CLEARANCE_BOUNDARY = [(0.0, 0.0), (12.0, 0.0), (12.0, 8.0), (0.0, 8.0)]
CLEARANCE_HOLES = [
    [(3.0, 1.5), (6.0, 1.5), (6.0, 4.0), (3.0, 4.0)],
    [(6.0, 4.8), (9.0, 6.5), (5.0, 6.5)],
]
CLEARANCE = 0.5
CLEARANCE_QUERY = ((1.0, 3.0), (11.0, 4.0))


def _clearance_environment(join_style):
    boundary = shapely.Polygon(CLEARANCE_BOUNDARY).buffer(
        -CLEARANCE, join_style=join_style
    )
    obstacles = shapely.union_all(
        [
            shapely.Polygon(hole).buffer(CLEARANCE, join_style=join_style)
            for hole in CLEARANCE_HOLES
        ]
    )
    return PolygonEnvironment.from_shapely(boundary, obstacles, validate=True)


def _distance_to_obstacles(path):
    """the smallest distance of the path to the original obstacles and the outside of the boundary"""
    line = shapely.LineString(path)
    obstacles = shapely.union_all([shapely.Polygon(h) for h in CLEARANCE_HOLES])
    outside = shapely.Polygon(CLEARANCE_BOUNDARY).exterior
    return min(line.distance(obstacles), line.distance(outside))


@pytest.mark.parametrize(
    "join_style, min_distance",
    [
        # mitred joins keep the clearance exactly
        ("mitre", CLEARANCE),
        # bevelled joins cut right-angled corners: clearance * sin(45 degree)
        ("bevel", CLEARANCE * np.sin(np.pi / 4)),
        # round joins approximate the arcs from inside: slightly closer than the clearance
        ("round", 0.499),
    ],
)
def test_clearance_recipe(join_style, min_distance):
    environment = _clearance_environment(join_style)
    path, length = environment.find_shortest_path(*CLEARANCE_QUERY)
    distance = _distance_to_obstacles(path)
    assert distance == pytest.approx(min_distance, abs=1e-3)
    assert distance <= CLEARANCE + 1e-12
    if join_style == "mitre":
        # the documented example: the passage closes, the path goes around the obstacle
        assert length == pytest.approx(11.908326913195985)
        assert distance == pytest.approx(CLEARANCE, abs=1e-12)
        assert len(environment.extremity_indices) == 6
    if join_style == "round":
        # many extremities: one per arc vertex
        assert len(environment.extremity_indices) > 100


def test_clearance_path_without_clearance_is_shorter():
    original = PolygonEnvironment(CLEARANCE_BOUNDARY, CLEARANCE_HOLES, validate=True)
    path, length = original.find_shortest_path(*CLEARANCE_QUERY)
    assert length == pytest.approx(10.23606797749979)
    # it passes between the obstacles, too close to them
    assert _distance_to_obstacles(path) == 0.0


@pytest.mark.parametrize(
    "start",
    [
        pytest.param((3.0, 1.2), id="inside-inflated-obstacle"),
        pytest.param((0.3, 3.0), id="outside-shrunk-boundary"),
    ],
)
def test_clearance_start_where_the_agent_does_not_fit(start):
    environment = _clearance_environment("mitre")
    assert not environment.within_map(np.array(start))
    with pytest.raises(ValueError, match="start point does not lie within the map"):
        environment.find_shortest_path(start, CLEARANCE_QUERY[1])
