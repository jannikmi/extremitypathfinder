"""tests for visibility detection

testing against old reference implementation

current test cases: all vertices of the test polygons
TODO add more test cases, random but valid query points (within polygons)
    hypothesis.readthedocs.io/
"""

import itertools
from typing import Set, Tuple

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment, configs, utils
from extremitypathfinder.utils import (
    _find_within_range,
    _lies_behind,
    get_neighbour_idxs,
)
from tests.test_cases import GRID_ENV_PARAMS, POLYGON_ENVS


def find_candidates_behind(
    origin: int,
    v1: int,
    v2: int,
    candidates: Set[int],
    distances: np.ndarray,
    coords: np.ndarray,
) -> Set[int]:
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
    visible_idxs: Set[int], cand_idx2repr: np.ndarray, vert_idx2dist: np.ndarray
) -> Set[int]:
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
    candidates: Set[int],
    edges_to_check: Set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    extremity_mask: np.ndarray,
) -> Set[int]:
    """
    :param origin: the vertex for which the visibility to the other candidates should be checked.
    :param candidates: the set of all vertex ids which should be checked for visibility.
        IMPORTANT: is being manipulated, so has to be a copy!
        IMPORTANT: must not contain any vertices with equal coordinates (e.g. the origin vertex itself)!
    :param edges_to_check: the set of edges which determine visibility
    :return: a set of all vertices visible from the origin
    """
    if len(candidates) == 0:
        return candidates

    candidates = candidates.copy()

    # optimisation: check edges with the highest angle range first ("blocking the most view"),
    # as they have the highest chance of eliminating candidates early on
    # this is especially important for large maps with many candidates
    edge_angle_range = {}
    samples = {}
    edges_to_skip = set()

    def skip_edge(node: int, edge2discard: int) -> Tuple[int, bool, int, int]:
        # node identical to origin
        # mark this vertex as not visible (would otherwise add 0 distance edge in the graph)
        candidates.discard(node)
        # no points lie truly "behind" this edge as there is no "direction of sight" defined
        # <-> angle representation/range undefined for just this single edge
        # however if one considers the point neighbouring in the other direction (<-> two edges)
        # these two neighbouring edges define an invisible angle range
        # -> simply move the pointer
        v1, v2 = get_neighbour_idxs(node, vertex_edge_idxs, edge_vertex_idxs)
        range_less_180 = extremity_mask[node]
        # do not check the other neighbouring edge of vertex1 in the future (has been considered already)
        edge_idx = vertex_edge_idxs[node][edge2discard]
        return edge_idx, range_less_180, v1, v2

    for edge in edges_to_check:
        if edge in edges_to_skip:
            continue

        v1, v2 = edge_vertex_idxs[edge]

        # the "view range" of an edge from a query point (spanned by the two vertices of the edge)
        #   is always < 180deg
        # edge case: edge is running through the query point -> 180 deg
        range_less_180 = True
        lies_on_edge = False

        if distances[v1] == 0.0:
            angle_range = np.inf
            # vertex1 of the edge has the same coordinates as the query vertex
            # (note: not identical, does not belong to the same polygon!)
            # -> the origin lies on the edge
            lies_on_edge = True
            edge_to_skip, range_less_180, v1, v2 = skip_edge(
                node=v1,
                edge2discard=0,
            )
            edges_to_skip.add(edge_to_skip)
        elif distances[v2] == 0.0:
            angle_range = np.inf
            # same for vertex2 of the edge
            # NOTE: it is unsupported that v1 as well as v2 have the same coordinates as the query vertex
            # (edge with length 0)
            lies_on_edge = True
            edge_to_skip, range_less_180, v1, v2 = skip_edge(
                node=v2,
                edge2discard=1,
            )
            edges_to_skip.add(edge_to_skip)

        repr1 = representations[v1]
        repr2 = representations[v2]
        # case: a 'regular' edge
        angle_range = abs(repr1 - repr2)
        if angle_range == 2.0:  # 180deg -> on the edge
            lies_on_edge = True
        elif angle_range > 2.0:
            # angle range blocked by a single edge is always <180 degree
            angle_range = 4 - angle_range

        edge_angle_range[edge] = angle_range
        samples[edge] = (v1, v2, repr1, repr2, lies_on_edge, range_less_180)

    # edges with the highest angle range first
    edges_prioritised = sorted(
        edge_angle_range.keys(), reverse=True, key=lambda e: edge_angle_range[e]
    )

    visibles = set()
    # goal: eliminating all vertices lying behind any edge ("blocking the view")
    for edge in edges_prioritised:
        if len(candidates) == 0:
            break

        # for all candidate edges check if there are any candidate vertices (besides the ones belonging to the edge)
        #   within this angle range
        v1, v2, repr1, repr2, lies_on_edge, range_less_180 = samples[edge]
        if lies_on_edge:
            # the query vertex lies on an edge (or vertex)
            # the neighbouring edges are visible for sure
            # attention: only add to visible set if vertex was a candidate!
            try:
                candidates.remove(v1)
                visibles.add(v1)
            except KeyError:
                pass
            try:
                candidates.remove(v2)
                visibles.add(v2)
            except KeyError:
                pass
            idxs_behind = candidates
            # all the candidates between the two vertices v1 v2 are not visible for sure
            # candidates with the same representation must not be deleted, because they can be visible!
            equal_repr_allowed = False

        else:
            # case: a 'regular' edge
            # eliminate all candidates which are blocked by the edge
            # that means inside the angle range spanned by the edge AND actually behind it
            idxs_behind = candidates.copy()
            # Attention: the vertices belonging to the edge itself (its vertices) must not be checked,
            # use discard() instead of remove() to not raise an error (they might not be candidates)
            idxs_behind.discard(v1)
            idxs_behind.discard(v2)
            #  candidates with the same representation as v1 or v2 should be considered.
            #   they may be visible, but must be ruled out if they lie behind any edge!
            equal_repr_allowed = True

        idxs_behind = _find_within_range(
            repr1,
            repr2,
            idxs_behind,
            range_less_180,
            equal_repr_allowed,
            representations,
        )
        if not lies_on_edge:
            # Note: when the origin lies on the edge, all candidates within the angle range lie behind the edge
            # -> actual "behind/in front" checks can be skipped!
            idxs_behind = find_candidates_behind(
                origin, v1, v2, idxs_behind, distances, coords
            )

        # vertices behind any edge are not visible and should not be considered any further
        candidates.difference_update(idxs_behind)

    # all edges have been checked
    # all remaining vertices were not concealed behind any edge and hence are visible
    visibles.update(candidates)
    return _clean_visibles(visibles, representations, distances)


def compile_boundary_data(env):
    boundary, holes = env
    boundary = np.array(boundary, dtype=configs.DTYPE_FLOAT)
    holes = [np.array(hole, dtype=configs.DTYPE_FLOAT) for hole in holes]
    # (coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs)
    return utils.compile_polygon_datastructs(boundary, holes)


def _yield_input_args(boundary_data):
    coords, extremity_indices, extremity_mask, vertex_edge_idxs, edge_vertex_idxs = (
        boundary_data
    )
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
        visibles_expected = find_visible_reference(
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
            visibles_expected,
        )


grid_env = PolygonEnvironment()
grid_env.store_grid_world(*GRID_ENV_PARAMS, simplify=False, validate=False)
grid_env_polygons = grid_env.boundary_polygon, grid_env.holes

all_envs = POLYGON_ENVS + [grid_env_polygons]
all_env_boundary_data = [compile_boundary_data(env) for env in all_envs]
test_cases_expected = [list(_yield_reference(env)) for env in all_env_boundary_data]
test_cases_expected = list(itertools.chain.from_iterable(test_cases_expected))


@pytest.mark.parametrize(
    "origin,candidates,edges_to_check,coords,representations,distances,edge_vertex_idxs,vertex_edge_idxs,extremity_mask,visibles_expected",
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
        vertex_edge_idxs,
        extremity_mask,
    )
    assert (
        visibles_found == visibles_expected
    ), f"expected {visibles_expected} but got {visibles_found}"
