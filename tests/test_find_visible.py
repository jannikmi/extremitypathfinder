import itertools

import numpy as np
import pytest

from extremitypathfinder import utils
from tests import test_cases


def get_find_visible_args(boundary_data):
    coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs = boundary_data
    # check that it works for all kind of inputs
    candidates = set(np.where(extremity_mask)[0])
    nr_edges = len(edge_vertex_idxs)
    edges_to_check = set(range(nr_edges))
    return (candidates,
            edges_to_check,
            coords,
            edge_vertex_idxs,
            vertex_edge_idxs,
            extremity_mask)

def _yield_reference(boundary_data):
    (candidates,
     edges_to_check,
     coords,
     edge_vertex_idxs,
     vertex_edge_idxs,
     extremity_mask) = get_find_visible_args(boundary_data)
    for origin in range(len(coords)):  # all possible vertices as origins
        reps_n_distances = utils.cmp_reps_n_distances(origin, coords)
        representations, distances = reps_n_distances

        # TODO move old implementation to tests test_cases.FIND_VISIBLE_TEST_CASES
        # TODO compile test cases with output from ref. impl.
        expected_visible = utils.find_visible(origin,
                                              candidates,
                                              edges_to_check,
                                              coords,
                                              representations,
                                              distances,
                                              edge_vertex_idxs,
                                              vertex_edge_idxs,
                                              extremity_mask)
        yield boundary_data, origin, expected_visible


@pytest.mark.parametrize(
    "coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs", test_cases.ALL_ENV_BOUNDARY_DATA)
def test_find_visible_definition(coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs
                                 ):
    # check that it works for all kind of inputs
    found_visible = np.where(extremity_mask)[0]
    nr_edges = len(edge_vertex_idxs)
    edges_to_check = set(range(nr_edges))
    for origin in range(len(coords)):  # all possible vertices as origins
        reps_n_distances = utils.cmp_reps_n_distances(origin, coords)
        representations, distances = reps_n_distances

        found_visible = utils.find_visible_(
            origin,
            found_visible,
            edges_to_check,
            coords,
            representations,
            distances,
            edge_vertex_idxs,
            vertex_edge_idxs,
            extremity_mask,
        )


test_cases_expected = [list(_yield_reference(env)) for env in test_cases.ALL_ENV_BOUNDARY_DATA]
test_cases_expected = list(itertools.chain.from_iterable(test_cases_expected))

@pytest.mark.parametrize(
    "boundary_data, origin, expected_visible",
    test_cases_expected)
def test_find_visible(boundary_data, origin, expected_visible
                      ):
    (candidates,
     edges_to_check,
     coords,
     edge_vertex_idxs,
     vertex_edge_idxs,
     extremity_mask) = get_find_visible_args(boundary_data)
    reps_n_distances = utils.cmp_reps_n_distances(origin, coords)
    representations, distances = reps_n_distances
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
        print("boundary_data", boundary_data)
        print("origin", origin)
        print("expected_visible", expected_visible)
        print("found_visible", found_visible)
        x=1
    assert found_visible == expected_visible, f"expected {expected_visible} but got {found_visible}"
