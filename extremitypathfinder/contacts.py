"""Contacts between obstacles

Obstacles are the holes and everything outside the boundary polygon.
Obstacles that touch form a wall: a path never passes between two obstacles that touch,
and never through a point where an obstacle touches itself.
https://github.com/jannikmi/extremitypathfinder/issues/24

A contact is a point where several polygon vertices have identical coordinates,
or where a vertex lies exactly on an edge (``abs(rep1 - rep2) == 2.0``, as in ``_inside_polygon()``).
Polygons with a tiny gap in between do not touch.

Around a point, the obstacles leave open "free wedges" (angle ranges) of free space.
Wedges are represented by the angle representations ``(start, end)`` of their two bounding rays,
the wedge reaching counter clockwise from ``start`` to ``end``.
A line of sight may pass through the point only when it enters and leaves through the same wedge,
the bounding rays included (paths may run along obstacle edges).
At most one wedge of a point can be larger than 180 degree.
A path can only bend around a point within such a wedge: the point is an extremity.
"""

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Set, Tuple

import numpy as np

from extremitypathfinder.utils_numba import _compute_repr_n_dist

Wedge = Tuple[float, float]
# polygon index, angle representation of the rays to the previous and the next vertex,
# indices of the previous and the next vertex
Incidence = Tuple[int, float, float, int, int]


@dataclass
class Contacts:
    """the points where passing through is decided by their free wedges

    * ``check_idxs``: one vertex index per point: all contact points and all vertices
      with a free angle of at most 180 degree (within the map)
    * ``exactly_checked``: mask of all vertices lying on these points
    * ``wedges``: the free wedges of every point in ``check_idxs``
    * ``contact_of_vertex``: maps every vertex lying on a contact point to the index in ``check_idxs``
    * ``incident_edges``: the edges having a vertex on or passing through a contact point
    * ``extremity_wedges``: the wedge > 180 degree of every contact point which is an extremity
    * ``wedge_neighbours``: the vertices in the direction of the two bounding rays of these wedges
    """

    check_idxs: np.ndarray
    exactly_checked: np.ndarray
    wedges: Dict[int, List[Wedge]] = field(default_factory=dict)
    contact_of_vertex: Dict[int, int] = field(default_factory=dict)
    incident_edges: Dict[int, Set[int]] = field(default_factory=dict)
    extremity_wedges: Dict[int, Wedge] = field(default_factory=dict)
    wedge_neighbours: Dict[int, Tuple[int, int]] = field(default_factory=dict)


def _rep(vector: np.ndarray) -> float:
    return _compute_repr_n_dist(vector)[0]


def in_closure(rep: float, wedge: Wedge) -> bool:
    """whether the direction lies within the wedge or on one of its bounding rays"""
    start, end = wedge
    if start == end:
        # a single ray: the whole circle
        return True
    return (rep - start) % 4.0 <= (end - start) % 4.0


def wedge_size(wedge: Wedge) -> float:
    start, end = wedge
    if start == end:
        return 4.0
    return (end - start) % 4.0


def _group_identical(coords: np.ndarray) -> List[List[int]]:
    """groups the indices of all vertices with exactly equal coordinates

    Note: np.unique(axis=0) would compare the bytes and distinguish 0.0 and -0.0
    """
    order = np.lexsort((coords[:, 1], coords[:, 0]))
    coords_sorted = coords[order]
    is_new = np.ones(len(coords), dtype=bool)
    is_new[1:] = np.any(coords_sorted[1:] != coords_sorted[:-1], axis=1)
    starts = np.flatnonzero(is_new).tolist() + [len(coords)]
    order = order.tolist()
    return [order[start:end] for start, end in zip(starts, starts[1:])]


def _find_edges_through_points(
    points: np.ndarray, coords: np.ndarray, edge_vertex_idxs: np.ndarray
) -> Dict[int, List[int]]:
    """finds the edges passing through the points (excluding the edge vertices)

    :return: the indices of the edges passing through every point (only for points with such edges)
    """
    p1 = coords[edge_vertex_idxs[:, 0]]
    p2 = coords[edge_vertex_idxs[:, 1]]
    x_min = np.minimum(p1[:, 0], p2[:, 0])
    x_max = np.maximum(p1[:, 0], p2[:, 0])
    y_min = np.minimum(p1[:, 1], p2[:, 1])
    y_max = np.maximum(p1[:, 1], p2[:, 1])
    edges_through = defaultdict(list)
    # limit the memory used by the bounding box candidates
    chunk_size = max(1, 2**22 // max(1, len(edge_vertex_idxs)))
    for chunk_start in range(0, len(points), chunk_size):
        chunk = points[chunk_start : chunk_start + chunk_size]
        x = chunk[:, 0, None]
        y = chunk[:, 1, None]
        in_box = (x_min <= x) & (x <= x_max) & (y_min <= y) & (y <= y_max)
        point_idxs, edge_idxs = np.nonzero(in_box)
        pts = chunk[point_idxs]
        is_vertex = np.all(pts == p1[edge_idxs], axis=1) | np.all(
            pts == p2[edge_idxs], axis=1
        )
        for point_idx, edge_idx, p in zip(
            point_idxs[~is_vertex], edge_idxs[~is_vertex], pts[~is_vertex]
        ):
            # the same computation as in _inside_polygon()
            if abs(_rep(p1[edge_idx] - p) - _rep(p2[edge_idx] - p)) == 2.0:
                edges_through[chunk_start + int(point_idx)].append(int(edge_idx))
    return edges_through


def _neighbours(
    i: int, vertex_edge_idxs: np.ndarray, edge_vertex_idxs: np.ndarray
) -> Tuple[int, int]:
    edge_idx1, edge_idx2 = vertex_edge_idxs[i]
    return int(edge_vertex_idxs[edge_idx1, 0]), int(edge_vertex_idxs[edge_idx2, 1])


def _incidences(
    point: np.ndarray,
    vertex_idxs: List[int],
    edge_idxs: List[int],
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    vertex_polygon_idxs: np.ndarray,
) -> List[Incidence]:
    # vertices: the rays point to the previous and to the next vertex
    # edges passing through: the rays point to the start and to the end of the edge
    # in both cases the free side (left of the edges) reaches counter clockwise from the second to the first ray
    pairs = [_neighbours(i, vertex_edge_idxs, edge_vertex_idxs) for i in vertex_idxs]
    pairs += [tuple(int(i) for i in edge_vertex_idxs[e]) for e in edge_idxs]
    return [
        (
            int(vertex_polygon_idxs[i_a]),
            _rep(coords[i_a] - point),
            _rep(coords[i_b] - point),
            i_a,
            i_b,
        )
        for i_a, i_b in pairs
    ]


def _on_free_side(rep: float, incidence: Incidence) -> bool:
    _, rep_a, rep_b, _, _ = incidence
    width = (rep_a - rep_b) % 4.0
    if width == 0.0:
        # zero-width spike (an invalid polygon): consistent with _fill_extremity_mask(),
        # the obstacle is the ray only
        return True
    return (rep - rep_b) % 4.0 < width


def free_wedges(
    incidences: List[Incidence], boundary_polygon_idx: Optional[int]
) -> List[Tuple[Wedge, Tuple[int, int]]]:
    """computes the free wedges around a point

    The rays of all incidences split the surrounding into elementary angle ranges ("arcs").
    Per polygon, the number of its incidences having an arc on their free (left) side
    equals the winding number of the polygon up to a constant.
    Holes block the arcs with less than the maximal number (clockwise: nonzero winding),
    the boundary polygon blocks the arcs with the minimal number (outside the boundary).
    Touching polygons act as their union. The free arcs are the wedges:
    between two free arcs there always is a ray blocking the passage (e.g. a zero-width spike).

    :return: the free wedges and the vertex indices in the direction of their bounding rays
    """
    ray_vertices: Dict[float, int] = {}
    for _, rep_a, rep_b, i_a, i_b in incidences:
        ray_vertices.setdefault(rep_a, i_a)
        ray_vertices.setdefault(rep_b, i_b)
    ray_reps = sorted(ray_vertices)
    arcs = [(r, ray_reps[(k + 1) % len(ray_reps)]) for k, r in enumerate(ray_reps)]

    polygon_idxs = {incidence[0] for incidence in incidences}
    arc_counts = []
    for arc in arcs:
        middle = (arc[0] + wedge_size(arc) / 2.0) % 4.0
        counts = dict.fromkeys(polygon_idxs, 0)
        for incidence in incidences:
            if _on_free_side(middle, incidence):
                counts[incidence[0]] += 1
        arc_counts.append(counts)

    blocking_counts = {}
    for polygon_idx in polygon_idxs:
        polygon_counts = {counts[polygon_idx] for counts in arc_counts}
        if polygon_idx == boundary_polygon_idx:
            blocking_counts[polygon_idx] = {min(polygon_counts)}
        else:
            blocking_counts[polygon_idx] = polygon_counts - {max(polygon_counts)}

    return [
        (arc, (ray_vertices[arc[0]], ray_vertices[arc[1]]))
        for arc, counts in zip(arcs, arc_counts)
        if not any(counts[p] in blocking_counts[p] for p in polygon_idxs)
    ]


def compute_contacts(
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    polygon_extremity_mask: np.ndarray,
    vertex_polygon_idxs: np.ndarray,
    boundary_polygon_idx: Optional[int],
    within_map: Callable[[np.ndarray], bool],
) -> Tuple[Contacts, np.ndarray]:
    """finds all contacts and the free wedges at all vertices where passing through might be blocked

    :param polygon_extremity_mask: the extremities of the single polygons (within the map), ignoring contacts
    :param vertex_polygon_idxs: the index of the polygon of every vertex
    :param boundary_polygon_idx: the index of the boundary polygon, ``None`` for unbounded environments
    :param within_map: a function checking whether a point lies within the map
    :return: the contacts and the extremity mask taking the contacts into account
    """
    extremity_mask = polygon_extremity_mask.copy()
    groups = _group_identical(coords)
    points = coords[[group[0] for group in groups]]
    edges_through = _find_edges_through_points(points, coords, edge_vertex_idxs)

    check_idxs = []
    contacts = Contacts(np.empty(0, dtype=int), np.zeros(len(coords), dtype=bool))
    for point_idx, (group, point) in enumerate(zip(groups, points)):
        edge_idxs = edges_through.get(point_idx, [])
        is_contact = len(group) + len(edge_idxs) > 1
        if not is_contact and polygon_extremity_mask[group[0]]:
            # a regular extremity: passing through is decided by the edges
            continue
        if not within_map(point):
            # Note: also not an extremity
            continue
        idx = min(group)
        check_idxs.append(idx)
        contacts.exactly_checked[group] = True
        if not is_contact:
            # a single vertex with a free angle <= 180 degree:
            # the wedge reaches counter clockwise from the next to the previous vertex
            prev_idx, next_idx = _neighbours(idx, vertex_edge_idxs, edge_vertex_idxs)
            wedge = (_rep(coords[next_idx] - point), _rep(coords[prev_idx] - point))
            contacts.wedges[idx] = [wedge]
            continue

        incidences = _incidences(
            point,
            group,
            edge_idxs,
            coords,
            edge_vertex_idxs,
            vertex_edge_idxs,
            vertex_polygon_idxs,
        )
        wedges = free_wedges(incidences, boundary_polygon_idx)
        contacts.wedges[idx] = [wedge for wedge, _ in wedges]
        extremity_mask[group] = False
        for i in group:
            contacts.contact_of_vertex[i] = idx
        incident_edges = {int(e) for i in group for e in vertex_edge_idxs[i]}
        contacts.incident_edges[idx] = incident_edges.union(edge_idxs)
        for wedge, ray_vertices in wedges:
            if wedge_size(wedge) > 2.0:
                extremity_mask[idx] = True
                contacts.extremity_wedges[idx] = wedge
                contacts.wedge_neighbours[idx] = ray_vertices

    contacts.check_idxs = np.array(sorted(check_idxs), dtype=int)
    return contacts, extremity_mask


def _passable(
    point_idx: int, origin: int, candidate: int, coords: np.ndarray, contacts: Contacts
) -> bool:
    """whether the line of sight from the origin to the candidate may pass through the point"""
    point = coords[point_idx]
    rep_in = _rep(coords[origin] - point)
    rep_out = _rep(coords[candidate] - point)
    return any(
        in_closure(rep_in, wedge) and in_closure(rep_out, wedge)
        for wedge in contacts.wedges[point_idx]
    )


def filter_visible(
    origin: int,
    visible: Set[int],
    coords: np.ndarray,
    representations: np.ndarray,
    distances: np.ndarray,
    contacts: Contacts,
    query_contacts: Optional[Dict[int, int]] = None,
) -> Set[int]:
    """removes the visible candidates which are blocked by contacts

    * a line of sight to a contact extremity must arrive within its wedge
    * a line of sight to a query point on a contact point must arrive within one of its wedges
    * a line of sight may pass through a point only within one of its free wedges

    :param representations: the angle representations of all coordinates seen from the origin
    :param distances: the distances of all coordinates to the origin
    :param query_contacts: the contact points of the query points (start, goal) lying on a contact point
    """
    if len(visible) == 0:
        return visible
    visible = set(visible)
    for candidate in visible.intersection(contacts.extremity_wedges):
        rep = _rep(coords[origin] - coords[candidate])
        if not in_closure(rep, contacts.extremity_wedges[candidate]):
            visible.discard(candidate)
    if query_contacts:
        for candidate in visible.intersection(query_contacts):
            rep = _rep(coords[origin] - coords[candidate])
            wedges = contacts.wedges[query_contacts[candidate]]
            if not any(in_closure(rep, wedge) for wedge in wedges):
                visible.discard(candidate)

    check_idxs = contacts.check_idxs
    if len(check_idxs) == 0 or len(visible) == 0:
        return visible
    check_reps = representations[check_idxs]
    # Note: a python set is faster than np.isin() for the typical small number of candidates
    check_rep_set = set(check_reps.tolist())
    for candidate in [i for i in visible if representations[i] in check_rep_set]:
        # the points on the line of sight. Note: the points identical to the origin are not passed
        check_dists = distances[check_idxs]
        on_line = check_reps == representations[candidate]
        passed = np.logical_and(on_line, check_dists < distances[candidate])
        passed = np.logical_and(passed, check_dists > 0.0)
        for point_idx in check_idxs[passed]:
            if not _passable(int(point_idx), origin, candidate, coords, contacts):
                visible.discard(candidate)
                break
    return visible


def contact_at(
    point: np.ndarray, coords: np.ndarray, contacts: Contacts
) -> Optional[int]:
    """the contact point identical to the given point, ``None`` if there is no such contact

    :param coords: the coordinates of all polygon vertices
    """
    if not contacts.contact_of_vertex:
        return None
    for i in np.flatnonzero(np.all(coords == point, axis=1)):
        idx = contacts.contact_of_vertex.get(int(i))
        if idx is not None:
            return idx
    return None


def check_polygon_contacts(coords: np.ndarray):
    """ensures that a polygon touching itself does not cross itself and has no overlapping edges

    * crossing at a vertex: a vertex lying on another edge with its neighbours on opposite sides
      or identical vertices whose corners interleave by angle
    * collinear overlapping edges (zero-width spikes or slits)

    https://github.com/jannikmi/extremitypathfinder/issues/135
    Note: proper intersections of two edges are detected by _no_self_intersection()
    """
    nr_vertices = len(coords)
    vertex_idxs = np.arange(nr_vertices)
    # edge i connects the vertices i-1 and i, vertex i connects the edges i and i+1
    edge_vertex_idxs = np.stack([(vertex_idxs - 1) % nr_vertices, vertex_idxs], axis=1)
    vertex_edge_idxs = np.stack([vertex_idxs, (vertex_idxs + 1) % nr_vertices], axis=1)
    vertex_polygon_idxs = np.zeros(nr_vertices, dtype=int)
    groups = _group_identical(coords)
    points = coords[[group[0] for group in groups]]
    edges_through = _find_edges_through_points(points, coords, edge_vertex_idxs)
    for point_idx, (group, point) in enumerate(zip(groups, points)):
        incidences = _incidences(
            point,
            group,
            edges_through.get(point_idx, []),
            coords,
            edge_vertex_idxs,
            vertex_edge_idxs,
            vertex_polygon_idxs,
        )
        rays = [rep for _, rep_a, rep_b, _, _ in incidences for rep in (rep_a, rep_b)]
        if len(set(rays)) < len(rays):
            raise ValueError("The given polygon has overlapping edges")
        for k, (_, rep_a1, rep_b1, _, _) in enumerate(incidences):
            corner = (rep_a1, rep_b1)
            for _, rep_a2, rep_b2, _, _ in incidences[k + 1 :]:
                # Note: all rays are distinct
                if in_closure(rep_a2, corner) != in_closure(rep_b2, corner):
                    raise ValueError("The given polygon has self intersections")
