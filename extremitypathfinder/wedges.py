"""The free space around points: free wedges

Obstacles are the holes and everything outside the boundary polygon.
A *point* is a position where polygons meet: the position of one or more polygon vertices
(other polygons might touch it), or a query point (start, goal) lying on a polygon edge.
Around a point the obstacles leave angle ranges of free space, the *free wedges*.
A wedge is represented by the angle representations ``(start, end)`` of its two bounding rays,
it reaches counter clockwise from ``start`` to ``end``.
The bounding rays belong to the wedge: paths may run along obstacle edges.

Examples: a convex vertex of a single hole has one free wedge larger than 180 degree.
A reflex vertex has one free wedge smaller than 180 degree.
Obstacles that touch (a *contact*: several polygon vertices with identical coordinates, or a vertex on an edge)
split the free space around the point into several wedges, at most one of them is larger than 180 degree.

The same rules hold at every point:

1. A point is an extremity when it has a free wedge larger than 180 degree.
   Only there a shortest path can bend around an obstacle.
2. A line of sight leaves a point within a free wedge:
   an extremity within its wedge larger than 180 degree, a query point within any of its wedges.
3. A line of sight passes through a point only when it enters and leaves within the same wedge.
   Hence a path never passes between two obstacles that touch.
4. A line of sight arrives at a point within a free wedge, like rule 2.
   Note: arriving in free space at a point of a single polygon always means arriving within its wedge
   (otherwise the line of sight would have crossed the polygon). It is only checked at contacts.

A contact means exactly equal coordinates or a vertex exactly on an edge (cf. ``_lies_on_edge()``).
Polygons with a tiny gap in between do not touch.
Rays of exactly the same direction share one angle representation, although the representations computed
from vectors of different length might differ in the last bit (e.g. along overlapping edges of different length).
https://github.com/jannikmi/extremitypathfinder/issues/24
"""

import itertools
from collections import defaultdict
from collections.abc import Callable, Iterable
from dataclasses import dataclass

import numpy as np

from extremitypathfinder.utils_numba import (
    _compute_repr_n_dist,
    _lies_on_edge,
    get_neighbour_idxs,
)

Wedge = tuple[float, float]
# the direction of a ray and its angle representation
Ray = tuple[np.ndarray, float]
# a polygon passing a point: the polygon index and the angle representations of its two rays
# (vertex: the rays to the previous and the next vertex, edge passing through: to the start and the end of the edge).
# The free side of the polygon (left of its edges) reaches counter clockwise from the second to the first ray.
Incidence = tuple[int, float, float]

# directions with representations this close to a bounding ray are compared exactly
RAY_TOLERANCE = 1e-12


def _rep(vector: np.ndarray) -> float:
    return _compute_repr_n_dist(vector)[0]


def _cross(vector1: np.ndarray, vector2: np.ndarray) -> float:
    return vector1[0] * vector2[1] - vector1[1] * vector2[0]


def in_closure(rep: float, wedge: Wedge) -> bool:
    """whether the direction lies within the wedge or on one of its bounding rays"""
    start, end = wedge
    if start == end:
        # a single ray: the whole circle
        return True
    return (rep - start) % 4.0 <= (end - start) % 4.0


def wedge_size(wedge: Wedge) -> float:
    """the angle of the wedge as angle representation difference (2.0: 180 degree)"""
    start, end = wedge
    if start == end:
        return 4.0
    return (end - start) % 4.0


def _ray_rep(vector: np.ndarray, rays: list[Ray]) -> float | None:
    """the angle representation of the ray with exactly the same direction, ``None`` if there is none"""
    for ray, ray_rep in rays:
        if _cross(ray, vector) == 0.0 and np.dot(ray, vector) > 0.0:
            return ray_rep
    return None


def _add_ray(vector: np.ndarray, rays: list[Ray]) -> float:
    """the angle representation of the ray, identical for all rays with exactly the same direction"""
    rep = _ray_rep(vector, rays)
    if rep is None:
        rep = _rep(vector)
        rays.append((vector, rep))
    return rep


def _on_free_side(rep: float, incidence: Incidence) -> bool:
    _, rep_a, rep_b = incidence
    width = (rep_a - rep_b) % 4.0
    if width == 0.0:
        # zero-width spike (an invalid polygon): the obstacle is the ray only
        return True
    return (rep - rep_b) % 4.0 < width


def free_wedges(
    incidences: list[Incidence], boundary_polygon_idx: int | None
) -> list[Wedge]:
    """computes the free wedges around a point

    The rays of all incidences split the surrounding into elementary angle ranges ("arcs").
    Per polygon, the number of its incidences having an arc on their free (left) side
    equals the winding number of the polygon up to a constant.
    Holes block the arcs with less than the maximal number (clockwise: nonzero winding),
    the boundary polygon blocks the arcs with the minimal number (outside the boundary).
    Touching polygons act as their union. The free arcs are the wedges:
    between two free arcs there always is a ray blocking the passage (e.g. a zero-width spike).
    """
    ray_reps = sorted({rep for _, rep_a, rep_b in incidences for rep in (rep_a, rep_b)})
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
        if len(polygon_counts) == 1:
            # only zero-width spikes: they block their rays only (the arcs are split by the rays)
            blocking_counts[polygon_idx] = set()
        elif polygon_idx == boundary_polygon_idx:
            blocking_counts[polygon_idx] = {min(polygon_counts)}
        else:
            blocking_counts[polygon_idx] = polygon_counts - {max(polygon_counts)}

    return [
        arc
        for arc, counts in zip(arcs, arc_counts)
        if not any(counts[p] in blocking_counts[p] for p in polygon_idxs)
    ]


@dataclass
class Point:
    """a position where polygons meet and the free wedges around it

    * ``idx``: the index of the coordinates of the point (the lowest index of identical vertices)
    * ``nr_incidences``: the number of polygon vertices and edges passing the point
    * ``incident_edges``: the edges having a vertex at or passing through the point
    * ``wedges``: the free wedges
    * ``extremity``: the index of the wedge larger than 180 degree, ``None`` if there is none
    * ``rays``: the directions of the bounding rays and their angle representations
    """

    idx: int
    nr_incidences: int
    incident_edges: set[int]
    wedges: list[Wedge]
    extremity: int | None
    rays: list[Ray]

    @property
    def is_contact(self) -> bool:
        return self.nr_incidences > 1

    @property
    def extremity_wedge(self) -> Wedge | None:
        return None if self.extremity is None else self.wedges[self.extremity]

    def rep(self, vector: np.ndarray) -> float:
        """the angle representation of the direction, identical to a ray with exactly the same direction"""
        rep = _ray_rep(vector, self.rays)
        return _rep(vector) if rep is None else rep

    def in_wedges(
        self, rep: float, vector: np.ndarray, wedges: Iterable[Wedge]
    ) -> bool:
        """whether the direction lies within (the closure of) one of the wedges

        :param rep: the angle representation of the direction
        :param vector: the direction, to compare it exactly with the bounding rays
        """
        for wedge in wedges:
            if any(abs(rep - bound) < RAY_TOLERANCE for bound in wedge):
                rep = self.rep(vector)
            if in_closure(rep, wedge):
                return True
        return False

    def passable(self, vector_in: np.ndarray, vector_out: np.ndarray) -> bool:
        """whether a line of sight may pass through the point (rule 3)

        :param vector_in: the direction from the point back to the start of the line of sight
        :param vector_out: the direction from the point to the end of the line of sight
        """
        rep_in = self.rep(vector_in)
        rep_out = self.rep(vector_out)
        return any(
            in_closure(rep_in, wedge) and in_closure(rep_out, wedge)
            for wedge in self.wedges
        )


def _group_identical(coords: np.ndarray) -> list[list[int]]:
    """groups the indices of all vertices with exactly equal coordinates

    Note: np.unique(axis=0) would compare the bytes and distinguish 0.0 and -0.0
    """
    order = np.lexsort((coords[:, 1], coords[:, 0]))
    coords_sorted = coords[order]
    is_new = np.ones(len(coords), dtype=bool)
    is_new[1:] = np.any(coords_sorted[1:] != coords_sorted[:-1], axis=1)
    starts = np.flatnonzero(is_new).tolist() + [len(coords)]
    order = order.tolist()
    return [order[start:end] for start, end in itertools.pairwise(starts)]


def _find_edges_through_points(
    positions: np.ndarray, coords: np.ndarray, edge_vertex_idxs: np.ndarray
) -> dict[int, list[int]]:
    """finds the edges passing through the positions (excluding their vertices)

    :return: the indices of the edges passing through every position (only for positions with such edges)
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
    for chunk_start in range(0, len(positions), chunk_size):
        chunk = positions[chunk_start : chunk_start + chunk_size]
        x = chunk[:, 0, None]
        y = chunk[:, 1, None]
        in_box = (x_min <= x) & (x <= x_max) & (y_min <= y) & (y <= y_max)
        position_idxs, edge_idxs = np.nonzero(in_box)
        pts = chunk[position_idxs]
        is_vertex = np.all(pts == p1[edge_idxs], axis=1) | np.all(
            pts == p2[edge_idxs], axis=1
        )
        for position_idx, edge_idx, p in zip(
            position_idxs[~is_vertex], edge_idxs[~is_vertex], pts[~is_vertex]
        ):
            if _lies_on_edge(p, p1[edge_idx], p2[edge_idx]):
                edges_through[chunk_start + int(position_idx)].append(int(edge_idx))
    return edges_through


def _incidences(
    position: np.ndarray,
    vertex_idxs: list[int],
    edge_idxs: list[int],
    coords: np.ndarray,
    edge_vertex_idxs: np.ndarray,
    vertex_edge_idxs: np.ndarray,
    vertex_polygon_idxs: np.ndarray,
    rays: list[Ray],
) -> list[Incidence]:
    """the polygons passing a position

    :param vertex_idxs: the polygon vertices at the position
    :param edge_idxs: the polygon edges passing through the position
    :param rays: gets filled with the directions and angle representations of the rays
    """
    pairs = [
        get_neighbour_idxs(i, vertex_edge_idxs, edge_vertex_idxs) for i in vertex_idxs
    ]
    pairs += [tuple(int(i) for i in edge_vertex_idxs[e]) for e in edge_idxs]
    return [
        (
            int(vertex_polygon_idxs[i_a]),
            _add_ray(coords[i_a] - position, rays),
            _add_ray(coords[i_b] - position, rays),
        )
        for i_a, i_b in pairs
    ]


class Points:
    """all points of an environment: the positions of the polygon vertices within the map

    Every vertex position is one point, identified by the lowest index of the vertices at the position.
    Vertex positions outside of the map (e.g. inside another hole) are no points.

    :param vertex_polygon_idxs: the index of the polygon of every vertex
    :param boundary_polygon_idx: the index of the boundary polygon, ``None`` for unbounded environments
    :param within_map: a function checking whether a position lies within the map
    """

    def __init__(
        self,
        coords: np.ndarray,
        edge_vertex_idxs: np.ndarray,
        vertex_edge_idxs: np.ndarray,
        vertex_polygon_idxs: np.ndarray,
        boundary_polygon_idx: int | None,
        within_map: Callable[[np.ndarray], bool],
    ):
        self.coords = coords
        self.edge_vertex_idxs = edge_vertex_idxs
        self.vertex_edge_idxs = vertex_edge_idxs
        self.vertex_polygon_idxs = vertex_polygon_idxs
        self.boundary_polygon_idx = boundary_polygon_idx
        # all points by their index
        self.by_idx: dict[int, Point] = {}
        # the point index of every vertex position, None for positions outside of the map
        self._idx_at: dict[tuple[float, float], int | None] = {}

        groups = _group_identical(coords)
        positions = coords[[group[0] for group in groups]]
        edges_through = _find_edges_through_points(positions, coords, edge_vertex_idxs)
        for position_idx, (group, position) in enumerate(zip(groups, positions)):
            key = (float(position[0]), float(position[1]))
            if not within_map(position):
                self._idx_at[key] = None
                continue
            idx = min(group)
            self._idx_at[key] = idx
            self.by_idx[idx] = self._make_point(
                idx, position, group, edges_through.get(position_idx, [])
            )
        # the indices of all points (sorted)
        self.idxs = np.array(sorted(self.by_idx), dtype=int)

    def _make_point(
        self,
        idx: int,
        position: np.ndarray,
        vertex_idxs: list[int],
        edge_idxs: list[int],
    ) -> Point:
        rays: list[Ray] = []
        incidences = _incidences(
            position,
            vertex_idxs,
            edge_idxs,
            self.coords,
            self.edge_vertex_idxs,
            self.vertex_edge_idxs,
            self.vertex_polygon_idxs,
            rays,
        )
        wedges = free_wedges(incidences, self.boundary_polygon_idx)
        extremities = [k for k, wedge in enumerate(wedges) if wedge_size(wedge) > 2.0]
        incident_edges = {int(e) for i in vertex_idxs for e in self.vertex_edge_idxs[i]}
        incident_edges.update(edge_idxs)
        return Point(
            idx=idx,
            nr_incidences=len(incidences),
            incident_edges=incident_edges,
            wedges=wedges,
            extremity=extremities[0] if extremities else None,
            rays=rays,
        )

    @property
    def extremity_idxs(self) -> np.ndarray:
        """the indices of all points which are extremities (sorted)"""
        return np.array(
            [i for i in self.idxs if self.by_idx[i].extremity is not None], dtype=int
        )

    def at(self, position: np.ndarray, idx: int) -> Point | None:
        """the point at the position of a query point (start or goal)

        :param idx: the index of the query point in the coordinates
        :return: the point of the vertices at the position or a new point when the position lies on edges.
            ``None`` when no polygon passes the position (all directions are free).
        """
        key = (float(position[0]), float(position[1]))
        if key in self._idx_at:
            point_idx = self._idx_at[key]
            if point_idx is None:
                # a vertex position outside of the map: no free space
                return Point(idx, 0, set(), [], None, [])
            return self.by_idx[point_idx]
        edge_idxs = _find_edges_through_points(
            position[None, :], self.coords, self.edge_vertex_idxs
        ).get(0)
        if not edge_idxs:
            return None
        return self._make_point(idx, position, [], edge_idxs)

    def filter_visible(
        self,
        origin: int,
        visible: set[int],
        coords: np.ndarray,
        representations: np.ndarray,
        distances: np.ndarray,
        query_points: dict[int, Point | None] | None = None,
    ) -> set[int]:
        """removes the visible candidates whose lines of sight violate rule 3 or 4

        :param coords: the coordinates of all vertices (and query points)
        :param representations: the angle representations of all coordinates seen from the origin
        :param distances: the distances of all coordinates to the origin
        :param query_points: the points of the query points among the candidates
        """
        query_points = query_points or {}
        origin_position = coords[origin]

        def arrives(candidate: int) -> bool:
            # rule 4: only checked at contacts
            if candidate in query_points:
                point = query_points[candidate]
                if point is None or not point.is_contact:
                    return True
                wedges = point.wedges
            else:
                point = self.by_idx[candidate]
                if not point.is_contact:
                    return True
                wedges = [point.extremity_wedge]
            vector = origin_position - coords[candidate]
            return point.in_wedges(_rep(vector), vector, wedges)

        def passes(candidate: int) -> bool:
            # rule 3: all points on the line of sight (between, excluding the origin and the candidate)
            on_line = point_reps == representations[candidate]
            between = (point_dists > 0.0) & (point_dists < distances[candidate])
            for point_idx in self.idxs[on_line & between]:
                point_position = coords[point_idx]
                if not self.by_idx[int(point_idx)].passable(
                    origin_position - point_position, coords[candidate] - point_position
                ):
                    return False
            return True

        visible = {i for i in visible if arrives(i)}
        if len(visible) == 0:
            return visible
        point_reps = representations[self.idxs]
        point_dists = distances[self.idxs]
        candidates = np.fromiter(visible, dtype=int, count=len(visible))
        on_lines = candidates[np.isin(representations[candidates], point_reps)]
        return visible.difference(int(i) for i in on_lines if not passes(int(i)))


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
    positions = coords[[group[0] for group in groups]]
    edges_through = _find_edges_through_points(positions, coords, edge_vertex_idxs)
    for position_idx, (group, position) in enumerate(zip(groups, positions)):
        incidences = _incidences(
            position,
            group,
            edges_through.get(position_idx, []),
            coords,
            edge_vertex_idxs,
            vertex_edge_idxs,
            vertex_polygon_idxs,
            [],
        )
        reps = [rep for _, rep_a, rep_b in incidences for rep in (rep_a, rep_b)]
        if len(set(reps)) < len(reps):
            raise ValueError("The given polygon has overlapping edges")
        for k, (_, rep_a1, rep_b1) in enumerate(incidences):
            corner = (rep_a1, rep_b1)
            for _, rep_a2, rep_b2 in incidences[k + 1 :]:
                # Note: all rays are distinct
                if in_closure(rep_a2, corner) != in_closure(rep_b2, corner):
                    raise ValueError("The given polygon has self intersections")
