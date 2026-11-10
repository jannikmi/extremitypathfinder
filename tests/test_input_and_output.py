"""The format of the input is always checked, the format of the results does not depend on the input."""

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, load_pickle

BOUNDARY = [(0, 0), (10, 0), (10, 10), (0, 10)]
HOLE = [(4, 4), (6, 4), (6, 6), (4, 6)]
NAN = float("nan")
INF = float("inf")


@pytest.fixture(scope="module")
def environment():
    return PolygonEnvironment(BOUNDARY, [HOLE])


def _assert_python_floats(path, length):
    assert type(length) is float
    assert len(path) >= 2
    for point in path:
        assert type(point) is tuple
        assert [type(value) for value in point] == [float, float]


@pytest.mark.parametrize(
    "start, goal, expected_path",
    [
        # around the hole (graph search)
        ((1, 5), (9, 5), None),
        # directly visible
        ((1, 1), (9, 1), [(1.0, 1.0), (9.0, 1.0)]),
        ([1, 1], [9, 1], [(1.0, 1.0), (9.0, 1.0)]),
        (np.array([1, 1]), np.array([9, 1]), [(1.0, 1.0), (9.0, 1.0)]),
        ((np.float32(1), np.int64(1)), (9, 1), [(1.0, 1.0), (9.0, 1.0)]),
        # identical
        ((1, 1), (1, 1), [(1.0, 1.0), (1.0, 1.0)]),
        (np.array([1, 1]), [1.0, 1.0], [(1.0, 1.0), (1.0, 1.0)]),
        # start and goal on graph nodes
        ((4, 4), (6, 6), None),
    ],
)
def test_paths_and_lengths_are_python_floats(environment, start, goal, expected_path):
    path, length = environment.find_shortest_path(start, goal)
    _assert_python_floats(path, length)
    if expected_path is not None:
        assert path == expected_path
    result = environment.search(start, goal)
    assert (result.path, result.length) == (path, length)
    _assert_python_floats(result.path, result.length)


def test_path_is_printed_without_numpy_types(environment):
    path, length = environment.find_shortest_path((1, 5), (9, 5))
    assert "np." not in repr(path) + repr(length)


@pytest.mark.parametrize(
    "boundary, holes, error, message",
    [
        ([], [], TypeError, "boundary polygon: .* at least contain 3 vertices"),
        ([(0, 0), (10, 0)], [], TypeError, "boundary polygon: .* 3 vertices"),
        (BOUNDARY, [[(4, 4), (6, 4)]], TypeError, "hole 0: .* 3 vertices"),
        (BOUNDARY, [HOLE, []], TypeError, "hole 1: .* 3 vertices"),
        (None, [[(4, 4), (6, 4)]], TypeError, "hole 0: .* 3 vertices"),
        (
            [(0, 0, 0), (10, 0, 0), (10, 10, 0)],
            [],
            TypeError,
            "boundary polygon: Each point .* two values",
        ),
        ([0, 10, 10, 0], [], TypeError, "boundary polygon: Each point .* two values"),
        # input numpy cannot convert: ragged, not numeric, too large for a float
        (
            [(0, 0), (1,), (2, 2)],
            [],
            TypeError,
            "boundary polygon: expected a list of .* coordinate pairs",
        ),
        (BOUNDARY, [[(4, 4), (6, "a"), (6, 6)]], TypeError, "hole 0: expected a list"),
        (None, [HOLE, [(4, 4), (10**400, 4), (6, 6)]], TypeError, "hole 1: expected a"),
        (
            [(0, 0), (10, 0), (NAN, 10), (0, 10)],
            [],
            ValueError,
            "boundary polygon: Polygon coordinates must be finite",
        ),
        (
            BOUNDARY,
            [HOLE, [(4, 4), (INF, 4), (6, 6)]],
            ValueError,
            "hole 1: Polygon coordinates must be finite",
        ),
    ],
)
@pytest.mark.parametrize("validate", [False, True])
def test_malformed_polygons_are_always_rejected(
    boundary, holes, error, message, validate
):
    with pytest.raises(error, match=message):
        PolygonEnvironment(boundary, holes, validate=validate)


@pytest.mark.parametrize(
    "point, error, message",
    [
        ((1, 1, 1), TypeError, "point must consist of two values"),
        ((1,), TypeError, "point must consist of two values"),
        (1, TypeError, "point must consist of two values"),
        ([(1, 1), (2, 2)], TypeError, "point must consist of two values"),
        ((1, (1, 2)), TypeError, "point: expected two values"),
        ((1, "a"), TypeError, "point: expected two values"),
        ((NAN, 1), ValueError, "point must be finite"),
        ((1, INF), ValueError, "point must be finite"),
        ((-INF, 1), ValueError, "point must be finite"),
    ],
)
@pytest.mark.parametrize("verify", [False, True])
@pytest.mark.parametrize("as_start", [False, True])
def test_malformed_query_points_are_always_rejected(
    environment, point, error, message, verify, as_start
):
    start, goal = (point, (9, 1)) if as_start else ((9, 1), point)
    name = "start" if as_start else "goal"
    with pytest.raises(error, match=f"{name} {message}"):
        environment.find_shortest_path(start, goal, verify=verify)
    with pytest.raises(error, match=f"{name} {message}"):
        environment.search(start, goal, verify=verify)


@pytest.mark.parametrize(
    "size_x, size_y, obstacles, error, message",
    [
        (0, 5, [], ValueError, "size_x must be positive, got 0"),
        (5, -1, [(1, 1)], ValueError, "size_y must be positive, got -1"),
        (5.5, 5, [], TypeError, "size_x must be an integer, got 5.5"),
        (5, "5", [], TypeError, "size_y must be an integer, got '5'"),
        (5, 5, [(2.5, 2)], TypeError, "blocked cells must be integers"),
        (5, 5, [("a", "b")], TypeError, "blocked cells must be integers"),
        (5, 5, [(1, 2, 3)], TypeError, "blocked cell must consist of two values"),
        (5, 5, [(1, 2), (3,)], TypeError, "blocked cell must consist of two values"),
        (5, 5, [(INF, 1)], TypeError, "blocked cells must be integers"),
        (5, 5, [(NAN, 1)], TypeError, "blocked cells must be integers"),
        (5, 5, [(True, False)], TypeError, "blocked cells must be integers"),
        (
            5,
            5,
            [(1e30, 1)],
            ValueError,
            r"cell \(1\d{30}, 1\) lies outside of the grid",
        ),
        (True, 5, [], TypeError, "size_x must be an integer, got True"),
        (np.int64(0), 5, [], ValueError, "size_x must be positive, got 0"),
        # 16 * 16 overflows as uint8
        (
            np.uint8(16),
            np.uint8(16),
            [(x, y) for x in range(16) for y in range(16)],
            ValueError,
            "All cells of the grid are blocked",
        ),
        (5, 5, [1, 2], TypeError, "blocked cell must consist of two values"),
        (5, 5, [(2, 2), (7, 7)], ValueError, r"cell \(7, 7\) lies outside of the grid"),
        (5, 5, [(5, 0)], ValueError, r"cell \(5, 0\) lies outside of the grid"),
        (5, 5, [(0, -1)], ValueError, r"cell \(0, -1\) lies outside of the grid"),
        (
            2,
            2,
            [(0, 0), (1, 0), (0, 1), (1, 1)],
            ValueError,
            "All cells of the grid are blocked",
        ),
        (1, 1, [(0, 0), (0, 0)], ValueError, "All cells of the grid are blocked"),
    ],
)
def test_malformed_grids_are_rejected(size_x, size_y, obstacles, error, message):
    with pytest.raises(error, match=message):
        PolygonEnvironment.from_grid(size_x, size_y, obstacles)


@pytest.mark.parametrize(
    "obstacles",
    [
        [(2, 2)],
        ((2, 2),),
        (cell for cell in [(2, 2)]),
        {(2, 2)},
        np.array([(2, 2)]),
        # integral floats and repeated cells are unambiguous
        [(2.0, 2.0)],
        [(2, 2), (2, 2)],
    ],
    ids=["list", "tuple", "generator", "set", "array", "floats", "repeated"],
)
def test_grid_accepts_any_iterable_of_cells(obstacles):
    environment = PolygonEnvironment.from_grid(5, 5, obstacles)
    assert environment.holes[0].tolist() == [[2, 2], [2, 3], [3, 3], [3, 2]]
    path, _ = environment.find_shortest_path((0, 0), (4, 4))
    assert path == [(0.0, 0.0), (2.0, 3.0), (4.0, 4.0)]


def test_grid_without_obstacles_accepts_a_generator():
    environment = PolygonEnvironment.from_grid(np.int64(3), 2, iter(()))
    assert environment.holes == []
    assert environment.boundary_polygon.tolist() == [[0, 0], [3, 0], [3, 2], [0, 2]]


def test_pickle_export_and_import_do_not_print(environment, tmp_path, capsys):
    path = str(tmp_path / "environment.pickle")
    assert environment.export_pickle(path=path) == path
    load_pickle(path=path)
    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""


def test_grid_with_narrow_integer_sizes():
    # 16 * 17 overflows as uint8: not mistaken for a grid without free cells
    obstacles = [(x, 0) for x in range(16)]
    environment = PolygonEnvironment.from_grid(np.uint8(16), np.uint8(17), obstacles)
    assert sorted(environment.boundary_polygon.tolist()) == [
        [0, 1],
        [0, 17],
        [16, 1],
        [16, 17],
    ]


@pytest.mark.parametrize("boundary", [BOUNDARY, None])
def test_no_path_is_an_empty_list_and_none(boundary):
    # the start is enclosed by a ring of four touching holes
    ring = [
        [(2, 2), (8, 2), (8, 3), (2, 3)],
        [(2, 7), (8, 7), (8, 8), (2, 8)],
        [(2, 3), (3, 3), (3, 7), (2, 7)],
        [(7, 3), (8, 3), (8, 7), (7, 7)],
    ]
    environment = PolygonEnvironment(boundary, ring)
    assert environment.find_shortest_path((5, 5), (9, 9)) == ([], None)
    path, length = environment.find_shortest_path((1, 1), (9, 9))
    _assert_python_floats(path, length)


@pytest.mark.parametrize("verify", [False, True])
@pytest.mark.parametrize("point", [(NAN, 2), (2, INF)])
def test_distance_matrix_rejects_non_finite_points(environment, point, verify):
    with pytest.raises(ValueError, match="points must be finite"):
        environment.distance_matrix([(1, 1), point], verify=verify)
    with pytest.raises(ValueError, match="points must be finite"):
        environment.distance_matrix([(1, 1)], [point], verify=verify)
