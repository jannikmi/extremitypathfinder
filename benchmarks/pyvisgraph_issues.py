"""Minimal reproductions of the pyvisgraph 0.2.1 defects found by the benchmark validation.

1. Invalid paths through obstacles (``invalid``):
   ``edge_in_polygon`` decides whether the segment between two vertices of the same polygon
   runs through its interior by a point-in-polygon test of the segment midpoint
   (``polygon_crossing``, a crossing count along a horizontal ray to ``x = INF = 10000``).
   That test mixes exact and tolerant comparisons: edges are skipped with exact ``<``/``>``
   comparisons of the y-coordinates, but a vertex counts as lying on the ray when
   ``ccw()`` rounds the cross product to zero (absolute tolerance 1e-10; with a ray of length ~1e4
   that is a vertical distance of ~1e-14). When the midpoint lies level with another vertex
   of the polygon in exact arithmetic, the floating point midpoint misses that vertex
   by a rounding error (e.g. ``(0.1 + 0.7) / 2 == 0.39999999999999997``):
   one of the two edges at the vertex is skipped, the other counts as touching the ray
   and waits for a partner. The crossing is lost, the parity flips, the midpoint is
   classified as outside, and the visibility graph gets an edge through the obstacle.
   Coordinates on a decimal grid (snapped data, integer or rounded coordinates)
   make such coincidences frequent. Using an exact crossing count fixes all cases found.
2. Build crash (``crash``): vertices about 1e-6 apart make ``angle2`` compute a cosine
   of -1.0000000014. The truncation ``int(cos * 1e10) / 1e10`` keeps it below -1
   and ``acos`` raises ``ValueError: math domain error``.

Usage::

    uv run --group benchmark python -m benchmarks.pyvisgraph_issues [--crash]
"""

import argparse

import pyvisgraph as vg
from shapely.geometry import LineString, Polygon

from extremitypathfinder import PolygonEnvironment

# convex on the right, the left vertex (-1, 0.4) and the right vertex (3, 0.4)
# lie level with the midpoint of the vertices (0, 0.1) and (0, 0.7)
POLYGON = [(0.0, 0.1), (2.0, -1.0), (3.0, 0.4), (2.0, 2.0), (0.0, 0.7), (-1.0, 0.4)]
QUERY = ((-0.1, -0.3), (-0.1, 1.1))


def invalid_path():
    graph = vg.VisGraph()
    graph.build([[vg.Point(*p) for p in POLYGON]], status=False)
    polygon = Polygon(POLYGON)
    through = [
        ((e.p1.x, e.p1.y), (e.p2.x, e.p2.y))
        for e in graph.visgraph.get_edges()
        if LineString([(e.p1.x, e.p1.y), (e.p2.x, e.p2.y)]).relate_pattern(
            polygon, "T********"
        )
    ]
    print("polygon:", POLYGON)
    print("visibility graph edges through the polygon:", through)
    start, goal = QUERY
    path = graph.shortest_path(vg.Point(*start), vg.Point(*goal))
    print("pyvisgraph path:         ", [(p.x, p.y) for p in path])
    env = PolygonEnvironment(None, [POLYGON])
    path, length = env.find_shortest_path(start, goal)
    print(
        "extremitypathfinder path:",
        [tuple(map(float, p)) for p in path],
        f"length {float(length):.6f}",
    )


def crash():
    """the build of the raw (unsnapped) Natural Earth 110m data crashes"""
    from benchmarks import world_map

    polygons = world_map.load_polygons(world_map.download("110m"))
    cleaned, _ = world_map.clean(polygons, grid=1e-12)
    graph = vg.VisGraph()
    try:
        graph.build(
            [[vg.Point(*c) for c in p.exterior.coords[:-1]] for p in cleaned],
            status=False,
        )
    except ValueError as exc:
        print(f"pyvisgraph build crashed: {type(exc).__name__}: {exc}")
    else:
        print("pyvisgraph build succeeded")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--crash",
        action="store_true",
        help="also reproduce the build crash (raw 110m data, a build of several minutes)",
    )
    args = parser.parse_args()
    invalid_path()
    if args.crash:
        crash()
