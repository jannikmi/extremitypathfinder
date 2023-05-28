import itertools
import pickle

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, utils
from tests import test_cases
from tests.test_cases import GRID_ENV_PARAMS, POLY_ENV_PARAMS, POLYGON_ENVS


def compile_boundary_data(env):
    boundary, holes = env
    boundary = np.array(boundary)
    holes = [np.array(hole) for hole in holes]
    # (coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs)
    return utils.compile_boundary_data_fr_polys(boundary, holes)


def _yield_input_args(boundary_data):
    coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs = boundary_data
    candidates = set(np.where(extremity_mask)[0])
    nr_edges = len(edge_vertex_idxs)
    edges_to_check = set(range(nr_edges))
    for origin in range(len(coords)):  # all possible vertices as origins
        reps_n_distances = utils.cmp_reps_n_distances(origin, coords)
        representations, distances = reps_n_distances

        # the origin itself is not a candidate (always visible)
        candidates_ = candidates - {origin}
        # do not check the 2 edges having the origin as vertex
        edges_to_check_ = edges_to_check - set(vertex_edge_idxs[origin])
        yield (
            origin,
            candidates_,
            edges_to_check_,
            coords,
            representations,
            distances,
            edge_vertex_idxs,
            vertex_edge_idxs,
            extremity_mask,
        )


def _yield_reference(boundary_data):
    # TODO move old implementation to tests test_cases.FIND_VISIBLE_TEST_CASES
    # TODO compile test cases with output from ref. impl.
    for (
        origin,
        candidates,
        edges_to_check,
        coords,
        representations,
        distances,
        edge_vertex_idxs,
        vertex_edge_idxs,
        extremity_mask,
    ) in _yield_input_args(boundary_data):
        expected_visible = utils.find_visible(
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
        yield (
            origin,
            candidates,
            edges_to_check,
            coords,
            representations,
            distances,
            edge_vertex_idxs,
            vertex_edge_idxs,
            extremity_mask,
            expected_visible,
        )


grid_env = PolygonEnvironment()
grid_env.store_grid_world(*GRID_ENV_PARAMS, simplify=False, validate=False)
grid_env_polygons = grid_env.boundary_polygon, grid_env.holes

all_envs = POLYGON_ENVS + [grid_env_polygons]
all_env_boundary_data = [compile_boundary_data(env) for env in all_envs]
test_cases_expected = [list(_yield_reference(env)) for env in all_env_boundary_data]
test_cases_expected = list(itertools.chain.from_iterable(test_cases_expected))


@pytest.mark.parametrize(
    "origin,candidates,edges_to_check,coords,representations,distances,edge_vertex_idxs,vertex_edge_idxs,extremity_mask,expected_visible",
    test_cases_expected,
)
def test_find_visible(
    origin,
    candidates,
    edges_to_check,
    coords,
    representations,
    distances,
    edge_vertex_idxs,
    vertex_edge_idxs,
    extremity_mask,
    expected_visible,
):
    found_visible = utils.find_visible_(
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
    if found_visible != expected_visible:
        print("origin", origin)
        print("expected_visible", expected_visible)
        print("found_visible", found_visible)
        outp = [
            origin,
            candidates,
            edges_to_check,
            coords,
            representations,
            distances,
            edge_vertex_idxs,
            vertex_edge_idxs,
            extremity_mask,
            expected_visible,
        ]
        # outp = [list(x) if isinstance(x, set) else x for x in outp]
        # outp = [x.tolist() if isinstance(x, np.ndarray) else x for x in outp]
        with open(f"outp.pickle", "wb") as f:
            pickle.dump(outp, f)
        x = 1
    assert found_visible == expected_visible, f"expected {expected_visible} but got {found_visible}"
