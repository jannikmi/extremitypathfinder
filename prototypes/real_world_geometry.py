"""robustness of extremitypathfinder on real-world-like geometry (triangulated obstacle maps)

Generates environments like the ones of HGARL (Zapata Rodriguez et al., TMLR 2026,
https://github.com/Andresrzr/HGARL), which uses extremitypathfinder as an oracle for exact shortest paths:
obstacles are triangulated (Shapely Delaunay, clipped, fan triangulation), stored as float32,
merged again with Shapely (union, morphological close by 1e-6, clipping to the unit square,
splitting of polygons with holes, simplification), cast to float32 and snapped to 1e-8.
This produces many touching or nearly touching polygons, near-degenerate and collinear vertices and tiny gaps.

Obstacle sources (``SOURCES``): jagged polygons (forest), city blocks between streets, hexagonal tiles.
Variants (``VARIANTS``): every float32 triangle as an obstacle, the triangles shifted by a few float32 ulps
(overlaps, gaps and contacts at the scale of 1e-8), their plain union, and the HGARL pipeline
(with and without its morphological close and its ring cleanup).

Every environment is stored with validation (errors are reported) and without, and queried with random
start and goal points (``--queries`` pairs per environment). The paths are checked independently:

* exceptions while storing or querying
* paths through an obstacle, between touching obstacles or outside the boundary (Shapely)
* missing or longer paths than a reference: Dijkstra on the naive visibility graph of all vertices
  (a segment is visible if the closed free space computed by Shapely covers it)

Shapely decides with floating point overlays, which is unreliable at the scale of float32 seams.
Every failure is therefore checked exactly with rational arithmetic (``ExactObstacles``):
an infeasible path is "confirmed" when it is blocked exactly, a missing or longer path when the reference path
is free exactly. Unconfirmed failures are errors of the reference, e.g. a reference path through a point
where obstacles touch, which extremitypathfinder correctly forbids.

Needs Shapely (``make install``). Runs with any installed version of extremitypathfinder (2.x and 3.x API),
e.g. in a separate virtual environment with ``extremitypathfinder==2.7.2``.

usage:
    python prototypes/real_world_geometry.py --seeds 40 --queries 15
    python prototypes/real_world_geometry.py --seeds 5 --sources hex --variants jitter --json results.json
"""

import argparse
import importlib.metadata
import importlib.util
import itertools
import json
import math
import time
import traceback
from collections import Counter
from fractions import Fraction

import networkx as nx
import numpy as np
import shapely
from shapely import ops
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box

from extremitypathfinder import PolygonEnvironment

UNIT_BOX = box(0.0, 0.0, 1.0, 1.0)
AREA_TOL = 1e-9
LENGTH_RTOL = 1e-9
INTERIOR = "T********"


# ---------------------------------------------------------------- obstacle sources


def jagged_polygon(rng, cx, cy, r, verts, spike=0.2) -> Polygon:
    """a random convex(ish) polygon (HGARL organic forest)"""
    angles = np.sort(
        np.linspace(0, 2 * np.pi, verts, endpoint=False) + rng.uniform(-0.1, 0.1, verts)
    )
    radii = r * (1.0 + rng.uniform(-spike, spike, verts))
    return Polygon(
        np.stack([cx + radii * np.cos(angles), cy + radii * np.sin(angles)], axis=1)
    )


def forest(rng) -> list[Polygon]:
    """overlapping, touching and separate jagged polygons, partly outside of the unit square"""
    obstacles = []
    for _ in range(rng.integers(8, 25)):
        cx, cy = rng.uniform(-0.05, 1.05, 2)
        r = float(np.clip(rng.normal(0.08, 0.05), 0.02, 0.2))
        obstacles.append(jagged_polygon(rng, cx, cy, r, int(rng.integers(3, 10))))
    return obstacles


def city(rng) -> list[Polygon]:
    """blocks between streets (partly diagonal) on a grid of tiles, touching the unit square (HGARL city)"""
    lines = []
    for _ in range(rng.integers(2, 5)):
        x = rng.uniform(0.1, 0.9)
        lines.append(LineString([(x, 0), (x, 1)]))
        y = rng.uniform(0.1, 0.9)
        lines.append(LineString([(0, y), (1, y)]))
    for _ in range(rng.integers(0, 3)):
        p, q = rng.uniform(0, 1, (2, 2))
        lines.append(LineString([p, q]))
    streets = ops.unary_union(
        [line.buffer(0.025, cap_style=2, join_style=2) for line in lines]
    )
    n_tiles = int(rng.integers(2, 5))
    blocks = []
    for i, j in itertools.product(range(n_tiles), repeat=2):
        tile = box(i / n_tiles, j / n_tiles, (i + 1) / n_tiles, (j + 1) / n_tiles)
        edge_tile = i in (0, n_tiles - 1) or j in (0, n_tiles - 1)
        for fragment in as_polygons(tile.difference(streets)):
            if fragment.area < 1e-4:
                continue
            block = fragment if edge_tile else fragment.buffer(-0.01, join_style=2)
            blocks.extend(p for p in as_polygons(block) if p.area > 1e-4)
    return blocks


def hex_tiles(rng) -> list[Polygon]:
    """a random subset of a hexagonal tiling: neighbouring tiles share edges (HGARL hex forest)"""
    r = rng.uniform(0.04, 0.09)
    w, h = math.sqrt(3) * r, 1.5 * r
    tiles = []
    for row in range(int(1 / h) + 2):
        for col in range(int(1 / w) + 2):
            if rng.random() > 0.45:
                continue
            cx = col * w + (w / 2 if row % 2 else 0.0)
            cy = row * h
            angles = np.pi / 6 + np.arange(6) * np.pi / 3
            tiles.append(
                Polygon(
                    np.stack([cx + r * np.cos(angles), cy + r * np.sin(angles)], axis=1)
                )
            )
    return tiles


SOURCES = {"forest": forest, "city": city, "hex": hex_tiles}


# ---------------------------------------------------------------- HGARL pipeline


def as_polygons(geometry) -> list[Polygon]:
    if isinstance(geometry, Polygon):
        return [] if geometry.is_empty else [geometry]
    if isinstance(geometry, MultiPolygon):
        return [g for g in geometry.geoms if not g.is_empty]
    return [
        g
        for g in getattr(geometry, "geoms", [])
        if isinstance(g, Polygon) and not g.is_empty
    ]


def fan_triangulate(polygon: Polygon) -> list[Polygon]:
    coords = list(polygon.exterior.coords[:-1])
    c = (polygon.centroid.x, polygon.centroid.y)
    triangles = [
        Polygon([coords[i], coords[(i + 1) % len(coords)], c])
        for i in range(len(coords))
    ]
    return [t for t in triangles if t.area > 1e-12]


def triangulate(polygon: Polygon) -> list[Polygon]:
    """Delaunay triangles clipped to the polygon, non-triangular pieces fan triangulated (HGARL)"""
    triangles = []
    for t in ops.triangulate(polygon):
        if t.area <= AREA_TOL:
            continue
        for piece in as_polygons(t.intersection(polygon)):
            if piece.area <= AREA_TOL:
                continue
            if len(piece.exterior.coords) == 4:
                triangles.append(piece)
            else:
                triangles.extend(fan_triangulate(piece))
    return triangles


def obstacle_triangles(obstacles: list[Polygon]) -> np.ndarray:
    """the triangles of the obstacles (clipped to the unit square, consolidated) as float32 (K,3,2)"""
    merged = ops.unary_union([o.buffer(0) for o in obstacles]).intersection(UNIT_BOX)
    triangles = [t for p in as_polygons(merged) for t in triangulate(p)]
    return np.array(
        [np.asarray(t.exterior.coords, dtype=np.float32)[:3] for t in triangles],
        dtype=np.float32,
    )


def decompose_holey(polygon: Polygon) -> list[Polygon]:
    """split polygons with holes into pieces without holes (HGARL), the pieces touch along the cuts"""
    work, result = [polygon], []
    for _ in range(50):
        next_work = []
        for piece in work:
            if not piece.interiors:
                result.append(piece)
                continue
            p_hole, p_ext = ops.nearest_points(
                LineString(piece.interiors[0].coords), LineString(piece.exterior.coords)
            )
            dx, dy = p_ext.x - p_hole.x, p_ext.y - p_hole.y
            length = math.hypot(dx, dy)
            if length < 1e-12:
                result.append(Polygon(piece.exterior))
                continue
            ux, uy = dx / length, dy / length
            mx, my = (p_hole.x + p_ext.x) / 2, (p_hole.y + p_ext.y) / 2
            cut = LineString([(mx - 2 * ux, my - 2 * uy), (mx + 2 * ux, my + 2 * uy)])
            parts = ops.split(piece, cut)
            if len(parts.geoms) > 1:
                next_work.extend(
                    g for g in parts.geoms if isinstance(g, Polygon) and not g.is_empty
                )
            else:
                result.append(piece)
        work = next_work
        if not work:
            break
    return result + work


def clean_ring(xy: np.ndarray) -> np.ndarray:
    """remove consecutive near duplicates and near-collinear vertices (HGARL ``_clean_ring``)"""
    keep = [0]
    for i in range(1, len(xy)):
        if np.linalg.norm(xy[i] - xy[keep[-1]]) > 1e-8:
            keep.append(i)
    xy = xy[keep]
    if len(xy) >= 3 and np.linalg.norm(xy[-1] - xy[0]) < 1e-8:
        xy = xy[:-1]
    for _ in range(3):
        if len(xy) < 4:
            break
        n = len(xy)
        out = []
        for i in range(n):
            a, b, c = xy[i - 1], xy[i], xy[(i + 1) % n]
            area2 = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
            if abs(area2) > 1e-12:
                out.append(b)
        if 3 <= len(out) < len(xy):
            xy = np.asarray(out)
        else:
            break
    return xy


def ring(polygon: Polygon) -> np.ndarray:
    return np.asarray(polygon.exterior.coords, dtype=np.float64)[:-1]


def hgarl_holes(
    triangles: np.ndarray, close: bool = True, clean: bool = True
) -> list[np.ndarray]:
    """the holes HGARL passes to extremitypathfinder

    cf. ``_triangles_to_obstacle_polys()`` and ``_build_holes_from_obstacles()`` in HGARL
    """
    union = ops.unary_union(
        [Polygon(t).buffer(0) for t in triangles.astype(np.float64)]
    )
    if close:
        union = union.buffer(1e-6, join_style=2).buffer(-1e-6, join_style=2).buffer(0)
    obstacles = [p for p in as_polygons(union) if p.area > AREA_TOL]
    obstacles = [
        g
        for p in obstacles
        for g in as_polygons(p.intersection(UNIT_BOX).buffer(0))
        if g.area > AREA_TOL
    ]
    obstacles = [
        g for p in obstacles for g in (decompose_holey(p) if p.interiors else [p])
    ]
    obstacles = [p.buffer(0).simplify(1e-6, preserve_topology=True) for p in obstacles]
    holes = []
    for p in obstacles:
        p = p.simplify(1e-5, preserve_topology=True).buffer(0)
        for g in as_polygons(p):
            if g.area < AREA_TOL:
                continue
            xy = ring(g)
            if clean:
                xy = clean_ring(xy)
            xy = xy.astype(np.float32).astype(np.float64)
            xy = np.round(xy / 1e-8) * 1e-8
            if len(xy) >= 3:
                holes.append(xy)
    return holes


def variant_triangles(triangles: np.ndarray) -> list[np.ndarray]:
    """every float32 triangle as its own hole: many obstacles touching along edges and at vertices"""
    return [t.astype(np.float64) for t in triangles]


def variant_union(triangles: np.ndarray) -> list[np.ndarray]:
    """the plain union of the float32 triangles: collinear vertices, slivers and tiny gaps of the float32 seams"""
    union = ops.unary_union(
        [Polygon(t).buffer(0) for t in triangles.astype(np.float64)]
    )
    # the interiors of the union are enclosed free space: dropped (as HGARL)
    return [ring(p) for p in as_polygons(union)]


def variant_jitter(triangles: np.ndarray, rng) -> list[np.ndarray]:
    """the triangles shifted by a few float32 ulps: nearly touching and slightly overlapping obstacles"""
    eps = np.float32(np.finfo(np.float32).eps)
    noise = rng.integers(-3, 4, triangles.shape).astype(np.float32) * eps
    return [t.astype(np.float64) for t in (triangles * (1 + noise)).astype(np.float32)]


VARIANTS = ["triangles", "jitter", "union", "hgarl", "hgarl_noclose", "hgarl_noclean"]


def build_variant(name: str, triangles: np.ndarray, rng) -> list[np.ndarray]:
    if name == "triangles":
        return variant_triangles(triangles)
    if name == "jitter":
        return variant_jitter(triangles, rng)
    if name == "union":
        return variant_union(triangles)
    if name == "hgarl":
        return hgarl_holes(triangles)
    if name == "hgarl_noclose":
        return hgarl_holes(triangles, close=False)
    if name == "hgarl_noclean":
        return hgarl_holes(triangles, clean=False)
    raise ValueError(name)


BOUNDARY = np.array([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)])


def generate(seed: int) -> list[dict]:
    """all environments of one seed: per obstacle source and variant a boundary and holes"""
    environments = []
    for source_name, source in SOURCES.items():
        rng = np.random.default_rng(seed)
        triangles = obstacle_triangles(source(rng))
        if len(triangles) == 0:
            continue
        for variant in VARIANTS:
            holes = build_variant(variant, triangles, rng)
            environments.append(
                {
                    "seed": seed,
                    "source": source_name,
                    "variant": variant,
                    "boundary": BOUNDARY,
                    "holes": holes,
                }
            )
    return environments


# ---------------------------------------------------------------- independent validation


class Reference:
    """the free space of an environment with Shapely, and shortest paths on the naive visibility graph"""

    def __init__(self, boundary, holes):
        self.boundary = Polygon(boundary)
        self.polygons = [Polygon(h) for h in holes]
        valid = [p if p.is_valid else p.buffer(0) for p in self.polygons]
        self.obstacles = ops.unary_union(valid)
        self.free = self.boundary.difference(self.obstacles)
        shapely.prepare(self.free)
        shapely.prepare(self.obstacles)
        self.tree = shapely.STRtree(self.polygons)
        vertices = [tuple(v) for v in boundary] + [tuple(v) for h in holes for v in h]
        self.vertices = np.unique(np.array(vertices), axis=0)
        self._graph = None
        # points where the union of the obstacles touches itself (pinch points of the free space)
        self.contacts = self._contacts()

    def _contacts(self) -> set[tuple[float, float]]:
        coords = Counter()
        for g in as_polygons(self.free):
            for r in [g.exterior, *g.interiors]:
                coords.update(tuple(c) for c in np.asarray(r.coords)[:-1])
        return {c for c, n in coords.items() if n > 1}

    def visible(self, a, b) -> np.ndarray:
        segments = shapely.linestrings(np.stack([a, b], axis=1))
        return shapely.covers(self.free, segments)

    def graph(self) -> nx.Graph:
        if self._graph is None:
            v = self.vertices
            i, j = np.triu_indices(len(v), k=1)
            vis = self.visible(v[i], v[j])
            g = nx.Graph()
            g.add_nodes_from(range(len(v)))
            for a, b in zip(i[vis], j[vis], strict=True):
                g.add_edge(int(a), int(b), weight=float(np.hypot(*(v[a] - v[b]))))
            self._graph = g
        return self._graph

    def shortest(self, start, goal):
        """the reference length and path ((None, None) if unreachable)"""
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if self.visible(start[None], goal[None])[0]:
            return float(np.hypot(*(start - goal))), [tuple(start), tuple(goal)]
        g = self.graph().copy()
        s, t = len(self.vertices), len(self.vertices) + 1
        for node, p in ((s, start), (t, goal)):
            vis = self.visible(np.repeat(p[None], len(self.vertices), 0), self.vertices)
            for k in np.flatnonzero(vis):
                g.add_edge(
                    node, int(k), weight=float(np.hypot(*(self.vertices[k] - p)))
                )
        try:
            length, nodes = nx.single_source_dijkstra(g, s, t)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return None, None
        coords = {s: tuple(start), t: tuple(goal)}
        return length, [
            coords[n] if n in coords else tuple(self.vertices[n]) for n in nodes
        ]

    def through_contact(self, path) -> bool:
        return any(tuple(p) in self.contacts for p in path[1:-1])

    def violations(self, path) -> list[str]:
        """why the path is infeasible (empty: feasible)"""
        reasons = []
        for p, q in itertools.pairwise(path):
            if tuple(p) == tuple(q):
                continue
            segment = LineString([p, q])
            if not self.boundary.covers(segment):
                reasons.append(f"leaves boundary {p}->{q}")
            for k in self.tree.query(segment):
                if segment.relate_pattern(self.polygons[k], INTERIOR):
                    inside = segment.intersection(self.polygons[k]).length
                    reasons.append(
                        f"through obstacle {k} ({inside:.3g} inside) {p}->{q}"
                    )
            if not reasons and segment.relate_pattern(self.obstacles, INTERIOR):
                reasons.append(f"between touching obstacles {p}->{q}")
        return reasons

    def random_free_points(self, rng, n, clearance=1e-3) -> list[tuple[float, float]]:
        points = []
        for _ in range(100 * n):
            p = Point(rng.uniform(0, 1, 2))
            if self.free.contains(p) and self.obstacles.distance(p) > clearance:
                points.append((p.x, p.y))
                if len(points) == n:
                    break
        return points


# ---------------------------------------------------------------- exact validation (rational arithmetic)
# Shapely decides with floating point overlays, which is not reliable at the scale of float32 seams (1e-8).
# A path claimed by a reference (or by extremitypathfinder) is therefore checked exactly with Fractions:
# every float coordinate is a rational number. A path is blocked when
# * a part of a segment lies strictly inside an obstacle or outside of the boundary,
# * a part of a segment runs between two obstacles touching along it (obstacles on both sides), or
# * the path passes through a point (inside a segment or at a waypoint) with obstacles touching this point
#   on both sides of the path ("touching obstacles form a wall", #24).


def _f(p):
    return (Fraction(float(p[0])), Fraction(float(p[1])))


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _orient(a, b, c):
    return _cross(_sub(b, a), _sub(c, a))


def _on_open_segment(x, a, b) -> bool:
    """whether x lies on the segment a-b, its end points excluded"""
    return _orient(a, b, x) == 0 and _dot(_sub(x, a), _sub(x, b)) < 0


def _angle_before(ref, w1, w2) -> bool:
    """whether direction w1 comes strictly before w2 counter clockwise starting at ref (angles in [0, 2 pi))"""

    def half(w):
        c = _cross(ref, w)
        return 0 if c > 0 or (c == 0 and _dot(ref, w) > 0) else 1

    h1, h2 = half(w1), half(w2)
    if h1 != h2:
        return h1 < h2
    c = _cross(w1, w2)
    if c != 0:
        return c > 0
    # collinear: same direction (equal) or, only possible in half 1 at angle pi, opposite
    return False


def _open_arcs_intersect(s1, e1, s2, e2) -> bool:
    """whether the open counter clockwise arcs (s1, e1) and (s2, e2) of directions intersect"""

    def pos(w):  # angle of w relative to s1 is > 0
        return _angle_before(s1, s1, w)

    b0_before_l1 = _angle_before(s1, s2, e1)  # ang(s2) < ang(e1)
    wraps = not _angle_before(s1, s2, e2)  # ang(s2) >= ang(e2)
    if not wraps:
        return b0_before_l1
    return pos(e2) or b0_before_l1


class ExactObstacles:
    """the polygons of an environment in rational arithmetic"""

    def __init__(self, boundary, holes):
        self.polygons = []  # (vertices, interior on the left side, is the boundary)
        for k, h in enumerate([boundary, *holes]):
            v = [_f(p) for p in h]
            area2 = sum(_cross(v[i], v[(i + 1) % len(v)]) for i in range(len(v)))
            # obstacles: the holes and the outside of the boundary
            obstacle_left = (area2 > 0) != (k == 0)
            self.polygons.append((v, obstacle_left, k == 0))
        self.bboxes = [
            (
                min(float(x) for x, _ in v),
                min(float(y) for _, y in v),
                max(float(x) for x, _ in v),
                max(float(y) for _, y in v),
            )
            for v, _, _ in self.polygons
        ]

    def _candidates(self, a, b):
        x0, x1 = sorted((float(a[0]), float(b[0])))
        y0, y1 = sorted((float(a[1]), float(b[1])))
        for k, (bx0, by0, bx1, by1) in enumerate(self.bboxes):
            if k == 0 or not (bx1 < x0 or bx0 > x1 or by1 < y0 or by0 > y1):
                yield k

    @staticmethod
    def _location(v, m) -> int:
        """1: strictly inside, 0: on the boundary, -1: outside of the polygon v"""
        inside = False
        n = len(v)
        for i in range(n):
            a, b = v[i], v[(i + 1) % n]
            within_x = min(a[0], b[0]) <= m[0] <= max(a[0], b[0])
            within_y = min(a[1], b[1]) <= m[1] <= max(a[1], b[1])
            if _orient(a, b, m) == 0 and within_x and within_y:
                return 0
            if (a[1] > m[1]) != (b[1] > m[1]):
                x = a[0] + (m[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
                if x > m[0]:
                    inside = not inside
        return 1 if inside else -1

    def _sectors(self, x, candidates):
        """the open counter clockwise sectors of directions (start, end) occupied by obstacles at the point x"""
        sectors = []
        for k in candidates:
            v, obstacle_left, _ = self.polygons[k]
            n = len(v)
            for i in range(n):
                a, b = v[i], v[(i + 1) % n]
                if a == x:
                    prev = v[i - 1]
                    to_next, to_prev = _sub(b, x), _sub(prev, x)
                    # obstacle left of the edges: from the next vertex counter clockwise to the previous one
                    sectors.append(
                        (to_next, to_prev) if obstacle_left else (to_prev, to_next)
                    )
                elif b != x and _on_open_segment(x, a, b):
                    forward, backward = _sub(b, a), _sub(a, b)
                    sectors.append(
                        (forward, backward) if obstacle_left else (backward, forward)
                    )
        return sectors

    def _blocked_at(self, x, back, out, candidates) -> bool:
        """whether obstacles touch the point x on both sides of the path

        :param back: the direction the path arrives from
        :param out: the direction the path leaves to
        """
        left = right = False
        for s, e in self._sectors(x, candidates):
            left = left or _open_arcs_intersect(out, back, s, e)
            right = right or _open_arcs_intersect(back, out, s, e)
            if left and right:
                return True
        return False

    def segment_blocked(self, p, q) -> str | None:
        P, Q = _f(p), _f(q)
        d = _sub(Q, P)
        dd = _dot(d, d)
        candidates = list(self._candidates(P, Q))
        params = {Fraction(0), Fraction(1)}
        for k in candidates:
            v = self.polygons[k][0]
            for i in range(len(v)):
                a, b = v[i], v[(i + 1) % len(v)]
                oa, ob = _orient(P, Q, a), _orient(P, Q, b)
                points = []
                if oa == 0:
                    points.append(a)
                if ob == 0:
                    points.append(b)
                if oa * ob < 0:
                    t = oa / (oa - ob)
                    points.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
                for x in points:
                    t = _dot(_sub(x, P), d) / dd
                    if 0 < t < 1:
                        params.add(t)
        params = sorted(params)
        for t0, t1 in itertools.pairwise(params):
            tm = (t0 + t1) / 2
            m = (P[0] + tm * d[0], P[1] + tm * d[1])
            sides = set()
            for k in candidates:
                v, obstacle_left, is_boundary = self.polygons[k]
                loc = self._location(v, m)
                if loc == (-1 if is_boundary else 1):
                    where = (
                        "outside of the boundary"
                        if is_boundary
                        else f"inside obstacle {k - 1}"
                    )
                    return f"{where} at {tuple(map(float, m))}"
                if loc == 0:
                    for i in range(len(v)):
                        a, b = v[i], v[(i + 1) % len(v)]
                        # an edge running along the segment through m (m is no vertex: it is no breakpoint)
                        if _orient(P, Q, a) == 0 and _on_open_segment(m, a, b):
                            same = _dot(_sub(b, a), d) > 0
                            sides.add("left" if obstacle_left == same else "right")
            if len(sides) == 2:
                return f"between obstacles touching along the segment at {tuple(map(float, m))}"
        for t in params[1:-1]:
            x = (P[0] + t * d[0], P[1] + t * d[1])
            if self._blocked_at(x, (-d[0], -d[1]), d, candidates):
                return f"through a contact point {tuple(map(float, x))}"
        return None

    def path_blocked(self, path) -> str | None:
        """why the path is blocked (None: free)"""
        path = [
            p for i, p in enumerate(path) if i == 0 or tuple(p) != tuple(path[i - 1])
        ]
        for i, (p, q) in enumerate(itertools.pairwise(path)):
            reason = self.segment_blocked(p, q)
            if reason:
                return f"segment {i}: {reason}"
        for i in range(1, len(path) - 1):
            x, a, b = _f(path[i]), _f(path[i - 1]), _f(path[i + 1])
            candidates = list(self._candidates(x, x))
            if self._blocked_at(x, _sub(a, x), _sub(b, x), candidates):
                return f"waypoint {i}: through a contact point {path[i]}"
        return None


# ---------------------------------------------------------------- running extremitypathfinder

VERSION = importlib.metadata.version("extremitypathfinder")
API_2 = hasattr(PolygonEnvironment, "store")


def build(boundary, holes, validate):
    if API_2:
        env = PolygonEnvironment()
        env.store(
            boundary, holes, validate=validate
        )  # also prepares the visibility graph
        return env
    return PolygonEnvironment(boundary, holes, validate=validate)


def short_error(e: Exception) -> str:
    return f"{type(e).__name__}: {str(e).splitlines()[0][:120] if str(e) else ''}"


# the results of all queries (e.g. to compare the paths with and without Numba)
QUERIES = []


def check_environment(environment: dict, n_queries: int, rng) -> list[dict]:
    """all failures of one environment"""
    failures = []
    base = {k: environment[k] for k in ("seed", "source", "variant")}
    boundary, holes = environment["boundary"], environment["holes"]
    shapely_invalid = sum(not Polygon(h).is_valid for h in holes)
    base["nr_holes"] = len(holes)
    base["shapely_invalid_holes"] = shapely_invalid
    try:
        build(boundary, holes, validate=True)
        base["validation"] = "ok"
    except Exception as e:  # noqa: BLE001
        base["validation"] = short_error(e)
        failures.append(dict(base, kind="validation error", error=short_error(e)))
    try:
        t = time.perf_counter()
        env = build(boundary, holes, validate=False)
        base["build_s"] = round(time.perf_counter() - t, 3)
    except Exception as e:  # noqa: BLE001
        failures.append(
            dict(
                base,
                kind="exception building",
                error=short_error(e),
                trace=traceback.format_exc(),
            )
        )
        return failures
    reference = Reference(boundary, holes)
    exact = ExactObstacles(boundary, holes)
    points = reference.random_free_points(rng, 2 * n_queries)
    for start, goal in zip(points[::2], points[1::2], strict=False):
        query = dict(base, start=start, goal=goal)
        try:
            path, length = env.find_shortest_path(start, goal, verify=False)
        except Exception as e:  # noqa: BLE001
            failures.append(
                dict(
                    query,
                    kind="exception querying",
                    error=short_error(e),
                    trace=traceback.format_exc(),
                )
            )
            continue
        path = [tuple(map(float, p)) for p in path]
        QUERIES.append(dict(query, length=length, path=path))
        ref_length, ref_path = reference.shortest(start, goal)
        failure = None
        if not path:
            if ref_length is not None:
                failure = {"kind": "missing path", "reference_length": ref_length}
                failure["reference_path"] = ref_path
        elif reference.violations(path) or exact.path_blocked(path):
            failure = {"kind": "infeasible path", "path": path, "length": length}
            failure["shapely"] = reference.violations(path)[:3]
        elif ref_length is not None and length > ref_length * (1 + LENGTH_RTOL) + 1e-12:
            failure = {"kind": "longer", "length": length, "path": path}
            failure.update(reference_length=ref_length, reference_path=ref_path)
        elif ref_length is None:
            failure = {"kind": "path where reference has none", "path": path}
            failure["length"] = length
        if failure is None:
            continue
        # confirm exactly: an infeasible path must be blocked, the reference path of a missing or longer path free
        if failure["kind"] == "infeasible path":
            reason = exact.path_blocked(path)
            failure["exact"] = reason
            failure["confirmed"] = reason is not None
        elif failure["kind"] in ("missing path", "longer"):
            reason = exact.path_blocked(ref_path)
            failure["exact"] = reason
            failure["confirmed"] = reason is None
        else:
            failure["confirmed"] = exact.path_blocked(path) is None
        failures.append(dict(query, **failure))
    return failures


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--seeds", type=int, default=10)
    parser.add_argument("--first-seed", type=int, default=0)
    parser.add_argument("--queries", type=int, default=10)
    parser.add_argument("--sources", nargs="*", default=list(SOURCES))
    parser.add_argument("--variants", nargs="*", default=VARIANTS)
    parser.add_argument("--json", help="write all failures to this file")
    args = parser.parse_args()

    numba = importlib.util.find_spec("numba") is not None
    print(
        f"extremitypathfinder {VERSION} ({'2.x' if API_2 else '3.x'} API), Numba installed: {numba}"
    )
    all_failures = []
    counts = Counter()
    for seed in range(args.first_seed, args.first_seed + args.seeds):
        for environment in generate(seed):
            selected = environment["source"] in args.sources
            if not selected or environment["variant"] not in args.variants:
                continue
            counts["environments"] += 1
            rng = np.random.default_rng(1000 + seed)
            failures = check_environment(environment, args.queries, rng)
            for f in failures:
                confirmed = {True: "confirmed", False: "not confirmed", None: ""}[
                    f.get("confirmed")
                ]
                details = f"{f['kind']} {confirmed} {f.get('error', '')} {f.get('exact') or ''}"
                print(
                    f"seed {f['seed']} {f['source']}/{f['variant']}: {details}",
                    flush=True,
                )
                counts[f"{f['kind']} {confirmed}".strip()] += 1
            all_failures.extend(failures)
    print(dict(counts))
    if args.json:
        with open(args.json, "w") as file:
            json.dump(
                {"failures": all_failures, "queries": QUERIES},
                file,
                indent=1,
                default=str,
            )


if __name__ == "__main__":
    main()
