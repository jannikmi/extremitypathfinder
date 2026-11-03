"""Independent validation of the computed paths with Shapely.

1. Feasibility: the path starts at the start point, ends at the goal and no segment
   intersects the interior of an obstacle (running along edges and through vertices is allowed).
2. Optimality agreement: the lengths of both packages agree within a relative tolerance.
   Routes of equal length may differ.
"""

import itertools
import math

from shapely.geometry import LineString, Polygon
from shapely.strtree import STRtree

from benchmarks.adapters import Path, path_length

LENGTH_RTOL = 1e-9
# interior of the segment intersects the interior of the polygon
INTERIOR_PATTERN = "T********"


class Obstacles:
    """the obstacles of a map with a spatial index"""

    def __init__(self, holes):
        self.polygons = [Polygon(h) for h in holes]
        self.tree = STRtree(self.polygons)

    def blocked_segments(self, path: Path) -> list[dict]:
        """the segments of the path running through the interior of an obstacle"""
        failures = []
        for i, (p, q) in enumerate(itertools.pairwise(path)):
            if p == q:
                continue
            segment = LineString([p, q])
            for j in self.tree.query(segment):
                polygon = self.polygons[j]
                if segment.relate_pattern(polygon, INTERIOR_PATTERN):
                    failures.append(
                        {
                            "segment": i,
                            "from": list(p),
                            "to": list(q),
                            "polygon": int(j),
                            "length_inside": segment.intersection(polygon).length,
                        }
                    )
        return failures


def feasibility(obstacles: Obstacles, start, goal, path: Path) -> list[str]:
    """the reasons why the path is infeasible (empty: feasible)"""
    if not path:
        # the free space is unbounded and the obstacles are disjoint: a path always exists
        return ["no path"]
    reasons = []
    if not math.isclose(path[0][0], start[0]) or not math.isclose(path[0][1], start[1]):
        reasons.append("does not start at the start point")
    if not math.isclose(path[-1][0], goal[0]) or not math.isclose(path[-1][1], goal[1]):
        reasons.append("does not end at the goal")
    blocked = obstacles.blocked_segments(path)
    if blocked:
        reasons.append(f"{len(blocked)} segment(s) through obstacles")
    return reasons


def lengths_agree(length_a: float, length_b: float, rtol: float = LENGTH_RTOL) -> bool:
    return abs(length_a - length_b) <= rtol * max(1.0, length_a)


def validate_paths(
    holes, queries, paths: dict[str, list[Path | None]], rtol: float = LENGTH_RTOL
) -> dict[str, dict]:
    """validate the paths of all packages for the queries

    :param paths: per package the paths in the order of the queries.
        ``None`` for queries which were not run (query time limit).
    :return: per package the number of checked and failed queries and the failures.
        A package fails a query if its path is infeasible, or if the feasible path
        of another package is shorter (not optimal).
    """
    obstacles = Obstacles(holes)
    report = {
        name: {"checked": 0, "infeasible": 0, "not_optimal": 0, "failures": []}
        for name in paths
    }
    for i, (start, goal) in enumerate(queries):
        feasible_lengths = {}
        for name, package_paths in paths.items():
            if i >= len(package_paths) or package_paths[i] is None:
                continue
            path = package_paths[i]
            report[name]["checked"] += 1
            reasons = feasibility(obstacles, start, goal, path)
            if reasons:
                report[name]["infeasible"] += 1
                report[name]["failures"].append(
                    {
                        "query": i,
                        "start": list(start),
                        "goal": list(goal),
                        "reasons": reasons,
                        "length": path_length(path),
                        "path": [list(p) for p in path],
                        "blocked_segments": obstacles.blocked_segments(path),
                    }
                )
            else:
                feasible_lengths[name] = path_length(path)
        if len(feasible_lengths) < 2:
            continue
        shortest = min(feasible_lengths.values())
        for name, length in feasible_lengths.items():
            if not lengths_agree(length, shortest, rtol):
                report[name]["not_optimal"] += 1
                report[name]["failures"].append(
                    {
                        "query": i,
                        "start": list(start),
                        "goal": list(goal),
                        "reasons": [f"not optimal: {length!r} > {shortest!r}"],
                        "length": length,
                        "path": [list(p) for p in paths[name][i]],
                        "blocked_segments": [],
                    }
                )
    for entry in report.values():
        entry["failed"] = len({f["query"] for f in entry["failures"]})
    return report
