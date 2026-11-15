"""Saving and loading environments with the stable file format (``PolygonEnvironment.save()`` and ``load()``)."""

import pickle

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, configs
from tests.test_cases import ENVS_N_TEST_DATA, POLY_ENV_PARAMS

START, GOAL = (1.0, 1.0), (9.0, 9.0)


def graph_data(environment):
    """the nodes, and the neighbours with their weights in iteration order: equal for identical graphs"""
    graph = environment.graph
    return list(graph.nodes), {
        node: list(graph.adj[node].items()) for node in graph.nodes
    }


def assert_same_environment(loaded, original):
    assert type(loaded) is type(original)
    assert np.array_equal(loaded.coords, original.coords)
    assert np.array_equal(loaded.extremity_indices, original.extremity_indices)
    assert loaded.nr_vertices == original.nr_vertices
    assert (loaded.idx_start, loaded.idx_goal) == (
        original.idx_start,
        original.idx_goal,
    )
    if original.boundary_polygon is None:
        assert loaded.boundary_polygon is None
    else:
        assert np.array_equal(loaded.boundary_polygon, original.boundary_polygon)
    assert len(loaded.holes) == len(original.holes)
    for loaded_hole, hole in zip(loaded.holes, original.holes):
        assert np.array_equal(loaded_hole, hole)
    # bit-identical weights and the same order of the neighbours (the search breaks ties the same way)
    assert graph_data(loaded) == graph_data(original)


@pytest.mark.parametrize(("env_params", "test_data"), ENVS_N_TEST_DATA)
def test_round_trip(env_params, test_data, tmp_path):
    original = PolygonEnvironment(*env_params)
    path = tmp_path / "env.npz"
    assert original.save(path) == path
    loaded = PolygonEnvironment.load(path)

    assert_same_environment(loaded, original)
    for (start, goal), (expected_path, expected_length) in test_data:
        # queries (also of invalid points) behave identically
        expected = original.find_shortest_path(start, goal)
        assert loaded.find_shortest_path(start, goal) == expected


def test_unbounded_environment(tmp_path):
    holes = [
        [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)],
        [(6.0, 1.0), (8.0, 1.0), (7.0, 5.0)],
    ]
    original = PolygonEnvironment(None, holes)
    original.save(tmp_path / "unbounded.npz")
    loaded = PolygonEnvironment.load(tmp_path / "unbounded.npz")

    assert_same_environment(loaded, original)
    assert loaded.boundary_polygon is None
    start, goal = (3.0, 0.0), (7.0, 6.0)
    assert loaded.find_shortest_path(start, goal) == original.find_shortest_path(
        start, goal
    )


def test_environment_without_edges(tmp_path):
    # a graph without edges: a single extremity per obstacle and no visibility between them is not required,
    # a convex boundary alone has no extremities at all
    original = PolygonEnvironment([(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)], [])
    assert original.graph.number_of_edges() == 0
    original.save(tmp_path / "convex.npz")
    loaded = PolygonEnvironment.load(tmp_path / "convex.npz")

    assert_same_environment(loaded, original)
    assert loaded.find_shortest_path(
        (1.0, 1.0), (3.0, 3.0)
    ) == original.find_shortest_path((1.0, 1.0), (3.0, 3.0))


def test_random_queries_are_identical(tmp_path):
    original = PolygonEnvironment(*POLY_ENV_PARAMS)
    original.save(tmp_path / "env.npz")
    loaded = PolygonEnvironment.load(tmp_path / "env.npz")
    rng = np.random.default_rng(0)
    compared = 0
    while compared < 100:
        start, goal = rng.uniform(0.0, 10.0, size=(2, 2))
        if not (original.within_map(start) and original.within_map(goal)):
            continue
        assert loaded.find_shortest_path(start, goal) == original.find_shortest_path(
            start, goal
        )
        compared += 1
    points = [START, GOAL, (5.0, 1.0)]
    assert np.array_equal(
        loaded.distance_matrix(points, points), original.distance_matrix(points, points)
    )


def test_the_path_is_used_as_given(tmp_path):
    # ``numpy.savez_compressed`` would append ".npz" to a path without this extension
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    path = tmp_path / "environment.data"
    environment.save(str(path))
    assert path.exists()
    assert [p.name for p in tmp_path.iterdir()] == ["environment.data"]
    assert_same_environment(PolygonEnvironment.load(str(path)), environment)


class SubclassedEnvironment(PolygonEnvironment):
    pass


def test_load_returns_an_instance_of_the_subclass(tmp_path):
    environment = SubclassedEnvironment(*POLY_ENV_PARAMS)
    environment.save(tmp_path / "env.npz")
    assert isinstance(
        SubclassedEnvironment.load(tmp_path / "env.npz"), SubclassedEnvironment
    )


def test_loaded_environment_can_be_pickled_and_saved_again(tmp_path):
    original = PolygonEnvironment(*POLY_ENV_PARAMS)
    original.save(tmp_path / "a.npz")
    loaded = PolygonEnvironment.load(tmp_path / "a.npz")
    loaded.save(tmp_path / "b.npz")
    assert_same_environment(PolygonEnvironment.load(tmp_path / "b.npz"), original)
    assert_same_environment(pickle.loads(pickle.dumps(loaded)), original)


# --- files that are rejected -------------------------------------------------------------------------------------


@pytest.fixture
def arrays(tmp_path):
    """the arrays of a saved environment with an unbounded boundary polygon and holes: to be modified"""
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    environment.save(tmp_path / "valid.npz")
    assert environment.graph.number_of_edges() > 0
    with np.load(tmp_path / "valid.npz", allow_pickle=False) as file:
        return {key: file[key] for key in file.files}


def load_modified(tmp_path, arrays):
    path = tmp_path / "modified.npz"
    np.savez_compressed(path, **arrays)
    return PolygonEnvironment.load(path)


def test_valid_arrays_load(tmp_path, arrays):
    # the baseline of the tests below: only the modification makes them fail
    load_modified(tmp_path, arrays)


def test_file_does_not_exist(tmp_path):
    with pytest.raises(FileNotFoundError):
        PolygonEnvironment.load(tmp_path / "missing.npz")


def test_pickle_file_is_rejected(tmp_path):
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    environment.export_pickle(path=str(tmp_path / "env.pickle"))
    with pytest.raises(ValueError, match="not an environment file.*pickle"):
        PolygonEnvironment.load(tmp_path / "env.pickle")


UNPICKLED: list[bool] = []


def _unpickle():
    UNPICKLED.append(True)


class Payload:
    """code that runs when it is unpickled"""

    def __reduce__(self):
        return _unpickle, ()


def test_pickled_arrays_are_rejected(tmp_path, arrays):
    # the load never unpickles: an object array inside of the file is rejected
    UNPICKLED.clear()
    arrays["weights"] = np.array([Payload()], dtype=object)
    with pytest.raises(ValueError, match="not an environment file.*pickle"):
        load_modified(tmp_path, arrays)
    assert not UNPICKLED


def test_corrupted_file_is_rejected(tmp_path, arrays):
    path = tmp_path / "env.npz"
    # incompressible data: flipping a byte in the middle hits the stored array
    np.savez_compressed(path, **arrays)
    data = bytearray(path.read_bytes())
    data[len(data) // 2] ^= 0xFF
    path.write_bytes(bytes(data))
    with pytest.raises(ValueError, match="corrupted|truncated|not an environment file"):
        PolygonEnvironment.load(path)


def test_truncated_file_is_rejected(tmp_path, arrays):
    path = tmp_path / "env.npz"
    np.savez_compressed(path, **arrays)
    path.write_bytes(path.read_bytes()[:-30])
    with pytest.raises(ValueError):
        PolygonEnvironment.load(path)


@pytest.mark.parametrize(
    "key",
    [
        "format_version",
        "boundary",
        "hole_sizes",
        "hole_coords",
        "extremity_indices",
        "edges",
        "weights",
    ],
)
def test_missing_array_is_rejected(tmp_path, arrays, key):
    del arrays[key]
    with pytest.raises(ValueError, match=f"missing the arrays.*{key}"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize("version", [0, configs.FILE_FORMAT_VERSION + 1])
def test_other_format_version_is_rejected(tmp_path, arrays, version):
    arrays["format_version"] = np.array(version, dtype=np.int64)
    with pytest.raises(ValueError, match=f"format version {version}"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize("version", [np.array([1]), np.array(1.0)])
def test_format_version_must_be_a_single_integer(tmp_path, arrays, version):
    arrays["format_version"] = version
    with pytest.raises(ValueError, match="'format_version' must be a single integer"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize(
    ("key", "kind"),
    [
        ("boundary", "int"),
        ("hole_coords", "int"),
        ("weights", "int"),
        ("hole_sizes", "float"),
        ("extremity_indices", "float"),
        ("edges", "float"),
    ],
)
def test_wrong_dtype_is_rejected(tmp_path, arrays, key, kind):
    arrays[key] = arrays[key].astype(np.int64 if kind == "int" else np.float64)
    with pytest.raises(ValueError, match=f"'{key}' must be"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize(
    "key",
    ["boundary", "hole_coords", "hole_sizes", "extremity_indices", "edges", "weights"],
)
def test_wrong_shape_is_rejected(tmp_path, arrays, key):
    array = arrays[key]
    # one dimension more or less, and a wrong number of columns
    arrays[key] = array[0] if array.ndim == 2 and key != "edges" else array[..., None]
    with pytest.raises(ValueError, match=f"'{key}' must have"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize("key", ["boundary", "hole_coords", "edges"])
def test_wrong_number_of_columns_is_rejected(tmp_path, arrays, key):
    arrays[key] = np.hstack([arrays[key], arrays[key][:, :1]])
    with pytest.raises(
        ValueError, match=f"'{key}' must have 2 dimensions with 2 columns"
    ):
        load_modified(tmp_path, arrays)


def test_hole_sizes_must_match_the_coordinates(tmp_path, arrays):
    arrays["hole_sizes"] = arrays["hole_sizes"] + 1
    with pytest.raises(ValueError, match="'hole_sizes'"):
        load_modified(tmp_path, arrays)


def test_holes_need_three_vertices(tmp_path, arrays):
    arrays["hole_sizes"] = np.array([2, arrays["hole_sizes"][0] - 2], dtype=np.int64)
    with pytest.raises(ValueError, match="'hole_sizes'"):
        load_modified(tmp_path, arrays)


def test_weights_and_edges_must_have_the_same_length(tmp_path, arrays):
    arrays["weights"] = arrays["weights"][:-1]
    with pytest.raises(ValueError, match="same length"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize("value", [np.nan, np.inf])
def test_non_finite_coordinates_are_rejected(tmp_path, arrays, value):
    arrays["hole_coords"][0, 0] = value
    with pytest.raises(ValueError, match="hole 0: Polygon coordinates must be finite"):
        load_modified(tmp_path, arrays)


def test_boundary_with_fewer_than_three_vertices_is_rejected(tmp_path, arrays):
    arrays["boundary"] = arrays["boundary"][:2]
    with pytest.raises(ValueError, match="boundary polygon: .*3 vertices"):
        load_modified(tmp_path, arrays)


def test_boundary_must_be_counter_clockwise(tmp_path, arrays):
    arrays["boundary"] = arrays["boundary"][::-1].copy()
    with pytest.raises(
        ValueError, match="boundary polygon must be numbered counter clockwise"
    ):
        load_modified(tmp_path, arrays)


def test_holes_must_be_clockwise(tmp_path, arrays):
    arrays["hole_coords"] = arrays["hole_coords"][::-1].copy()
    with pytest.raises(ValueError, match="hole 0 must be numbered clockwise"):
        load_modified(tmp_path, arrays)


def test_extremities_must_match_the_polygons(tmp_path, arrays):
    # the file was written for other polygons
    arrays["hole_coords"] = arrays["hole_coords"] + np.array([0.0, 0.5])
    arrays["extremity_indices"] = arrays["extremity_indices"][:-1]
    with pytest.raises(ValueError, match="extremities of the file differ"):
        load_modified(tmp_path, arrays)


def test_edge_index_out_of_range_is_rejected(tmp_path, arrays):
    arrays["edges"][0, 1] = 10**6
    with pytest.raises(ValueError, match="only connect extremities"):
        load_modified(tmp_path, arrays)


def test_negative_edge_index_is_rejected(tmp_path, arrays):
    arrays["edges"][0, 1] = -1
    with pytest.raises(ValueError, match="only connect extremities"):
        load_modified(tmp_path, arrays)


def test_edge_to_a_vertex_that_is_no_extremity_is_rejected(tmp_path, arrays):
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    non_extremities = set(range(environment.nr_vertices)) - set(
        environment.extremity_indices.tolist()
    )
    assert non_extremities
    arrays["edges"][0, 1] = non_extremities.pop()
    with pytest.raises(ValueError, match="only connect extremities"):
        load_modified(tmp_path, arrays)


def test_self_loop_is_rejected(tmp_path, arrays):
    arrays["edges"][0, 1] = arrays["edges"][0, 0]
    with pytest.raises(ValueError, match="with itself"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize("reversed_duplicate", [False, True])
def test_duplicate_edge_is_rejected(tmp_path, arrays, reversed_duplicate):
    duplicate = arrays["edges"][:1, ::-1] if reversed_duplicate else arrays["edges"][:1]
    arrays["edges"] = np.vstack([arrays["edges"], duplicate])
    arrays["weights"] = np.append(arrays["weights"], arrays["weights"][0])
    with pytest.raises(ValueError, match="twice"):
        load_modified(tmp_path, arrays)


@pytest.mark.parametrize("value", [0.0, -1.0, np.nan, np.inf])
def test_invalid_weight_is_rejected(tmp_path, arrays, value):
    arrays["weights"][0] = value
    with pytest.raises(ValueError, match="finite and positive"):
        load_modified(tmp_path, arrays)


def test_file_of_format_version_1_still_loads(tmp_path):
    # a file written by format version 1, spelled out: it must keep loading and describe the same visibility graph
    # as the preparation computes. If this fails because the visibility graph changed (other edges are kept),
    # increase ``configs.FILE_FORMAT_VERSION`` (older files are then rejected, so this test expects the ``ValueError``).
    path = tmp_path / "version1.npz"
    np.savez_compressed(
        path,
        format_version=np.array(1, dtype=np.int64),
        boundary=np.array(
            [[0, 0], [10, 0], [9, 5], [10, 10], [0, 10]], dtype=np.float64
        ),
        hole_sizes=np.array([4], dtype=np.int64),
        hole_coords=np.array([[3, 7], [5, 9], [4.6, 7], [5, 4]], dtype=np.float64),
        extremity_indices=np.array([2, 5, 6, 8], dtype=np.int64),
        edges=np.array([[5, 8], [5, 6], [6, 8]], dtype=np.int64),
        weights=np.array([3.605551275463989, 2.8284271247461903, 5.0]),
    )
    loaded = PolygonEnvironment.load(path)
    assert_same_environment(loaded, PolygonEnvironment(*POLY_ENV_PARAMS))
