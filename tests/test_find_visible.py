"""tests for visibility detection

testing against old reference implementation

current test cases: all vertices of the test polygons
"""

import itertools

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, configs, utils
from extremitypathfinder.utils import _find_within_range, _lies_behind
from tests.test_cases import GRID_ENV_PARAMS, POLYGON_ENVS


def find_candidates_behind(
    origin: int,
    v1: int,
    v2: int,
    candidates: set[int],
    distances: np.ndarray,
    coords: np.ndarray,
) -> set[int]:
    dist_v1 = distances[v1]
    dist_v2 = distances[v2]
    max_distance = max(dist_v1, dist_v2)
    idxs_behind = set()
    # for all remaining vertices v it has to be tested if the line segment from query point (=origin) to v
    #    has an intersection with the current edge p1---p2
    for idx in candidates:
        # if a candidate is farther away from the query point than both vertices of the edge,
        #   it surely lies behind the edge
        # ATTENTION: even if a candidate is closer to the query point than both vertices of the edge,
        #   it still needs to be checked!
        dist2orig = distances[idx]
        further_away = dist2orig > max_distance
        if further_away or _lies_behind(v1, v2, idx, origin, coords):
            idxs_behind.add(idx)
        # vertex lies in front of this edge
    return idxs_behind


def _clean_visibles(
    visible_idxs: set[int], cand_idx2repr: np.ndarray, vert_idx2dist: np.ndarray
) -> set[int]:
    # in case some vertices have the same representation, only return (link) the closest vertex
    if len(visible_idxs) <= 1:
        return visible_idxs

    cleaned = set()
    visible_idxs_sorted = sorted(visible_idxs, key=lambda i: cand_idx2repr[i])
    min_dist = np.inf
    first_idx = visible_idxs_sorted[0]
    rep_prev = cand_idx2repr[first_idx]
    selected_idx = 0
    for i in visible_idxs_sorted:
        rep = cand_idx2repr[i]
        if rep != rep_prev:
            cleaned.add(selected_idx)
            min_dist = np.inf
            rep_prev = rep

        dist = vert_idx2dist[i]
        if dist < min_dist:
            selected_idx = i
            min_dist = dist

    cleaned.add(selected_idx)
    return cleaned


def find_visible_reference(
    origin: int,
    candidates: set[int],
    edges_to_check: set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    edge_vertex_idxs: np.ndarray,
) -> set[int]:
    """
    :param origin: the vertex for which the visibility to the other candidates should be checked.
    :param candidates: the set of all vertex ids which should be checked for visibility.
        IMPORTANT: must not contain any vertices with equal coordinates (e.g. the origin vertex itself)!
    :param edges_to_check: the set of edges which determine visibility.
        IMPORTANT: no edge may touch the origin
    :return: a set of all vertices visible from the origin
    """
    if len(candidates) == 0:
        return candidates

    candidates = candidates.copy()
    # goal: eliminating all vertices lying behind any edge ("blocking the view")
    for edge in edges_to_check:
        v1, v2 = edge_vertex_idxs[edge]
        repr1 = representations[v1]
        repr2 = representations[v2]
        # the "view range" of an edge from the origin (spanned by the two vertices of the edge)
        #   is always < 180deg (no edge runs through the origin)
        assert abs(repr1 - repr2) != 2.0
        # eliminate all candidates which are blocked by the edge
        # that means inside the angle range spanned by the edge AND actually behind it.
        # Candidates with the same representation as v1 or v2 are not blocked by the edge:
        #   the line of sight touches the edge only in a vertex (or runs along the edge)
        idxs_behind = _find_within_range(
            repr1,
            repr2,
            candidates,
            angle_range_less_180=True,
            equal_repr_allowed=False,
            representations=representations,
        )
        idxs_behind = find_candidates_behind(
            origin, v1, v2, idxs_behind, distances, coords
        )
        # vertices behind any edge are not visible and should not be considered any further
        candidates.difference_update(idxs_behind)

    # all edges have been checked
    # all remaining vertices were not concealed behind any edge and hence are visible
    return _clean_visibles(candidates, representations, distances)


def compile_boundary_data(env):
    boundary, holes = env
    boundary = np.array(boundary, dtype=configs.DTYPE_FLOAT)
    holes = [np.array(hole, dtype=configs.DTYPE_FLOAT) for hole in holes]
    # (coords, edge_vertex_idxs, vertex_edge_idxs, points)
    return utils.compile_polygon_datastructs(boundary, holes)


def _yield_input_args(boundary_data):
    coords, edge_vertex_idxs, _vertex_edge_idxs, points = boundary_data
    candidates = set(points.extremity_idxs.tolist())
    nr_edges = len(edge_vertex_idxs)
    for origin in points.idxs:  # all points as origins
        representations, distances = utils.cmp_reps_n_distances(origin, coords)
        # the origin itself is not a candidate (always visible)
        candidates_ = candidates - {int(origin)}
        # do not check the edges touching the origin (cf. find_visible_from())
        edges_to_check = set(range(nr_edges)) - points.by_idx[origin].incident_edges
        yield (
            origin,
            candidates_,
            edges_to_check,
            coords,
            representations,
            distances,
            edge_vertex_idxs,
        )


def _yield_reference(boundary_data):
    for args in _yield_input_args(boundary_data):
        yield (*args, find_visible_reference(*args))


grid_env = PolygonEnvironment.from_grid(*GRID_ENV_PARAMS, validate=False)
grid_env_polygons = grid_env.boundary_polygon, grid_env.holes

all_envs = POLYGON_ENVS + [grid_env_polygons]
all_env_boundary_data = [compile_boundary_data(env) for env in all_envs]
test_cases_expected = [list(_yield_reference(env)) for env in all_env_boundary_data]
test_cases_expected = list(itertools.chain.from_iterable(test_cases_expected))


@pytest.mark.parametrize(
    "origin,candidates,edges_to_check,coords,representations,distances,edge_vertex_idxs,visibles_expected",
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
    visibles_expected,
):
    visibles_found = utils.find_visible(
        origin,
        candidates,
        edges_to_check,
        coords,
        representations,
        distances,
        edge_vertex_idxs,
    )
    assert visibles_found == visibles_expected, (
        f"expected {visibles_expected} but got {visibles_found}"
    )


def _visibility_relation(boundary_data) -> set[tuple[tuple[float, float], ...]]:
    """all pairs of visible extremities, as computed for the graph construction

    Note: without the (one-sided) elimination of extremities lying "in front"
    """
    coords, edge_vertex_idxs, _vertex_edge_idxs, points = boundary_data
    extremity_indices = points.extremity_idxs
    relation = set()
    for origin in extremity_indices:
        representations, distances = utils.cmp_reps_n_distances(origin, coords)
        candidates = set(extremity_indices.tolist()) - {int(origin)}
        _, visible_idxs = utils._find_visible_and_in_front(
            origin,
            candidates,
            set(),
            coords,
            representations,
            distances,
            edge_vertex_idxs,
            points,
        )
        for i in visible_idxs:
            relation.add((tuple(coords[origin]), tuple(coords[i])))
    return relation


@pytest.mark.parametrize("boundary_data", all_env_boundary_data)
def test_visibility_is_symmetric(boundary_data):
    # the graph construction only checks each pair of extremities in one direction
    relation = _visibility_relation(boundary_data)
    asymmetric = {(x, y) for x, y in relation if (y, x) not in relation}
    assert not asymmetric, f"visible only in one direction: {sorted(asymmetric)}"
