"""Uniform interface to the benchmarked packages: build an environment, query a shortest path.

Both adapters return the path as a list of (x, y) float tuples.
The path length is computed from the path with the same code for both packages
(pyvisgraph returns no length, extremitypathfinder a numpy float).
"""

import itertools
import math
from collections.abc import Sequence

Coords = Sequence[tuple[float, float]]
Path = list[tuple[float, float]]


def path_length(path: Path) -> float:
    return sum(math.dist(p, q) for p, q in itertools.pairwise(path))


class Adapter:
    name: str
    label: str

    def version(self) -> str:
        raise NotImplementedError

    def build(self, holes: list[Coords]):
        """the prepared environment of the unbounded map with the polygons as obstacles"""
        raise NotImplementedError

    def query(self, env, start: tuple[float, float], goal: tuple[float, float]) -> Path:
        """the shortest path from start to goal, an empty list if there is none"""
        raise NotImplementedError

    def graph_size(self, env) -> tuple[int, int]:
        """the number of nodes and edges of the precomputed graph"""
        raise NotImplementedError


class ExtremityPathfinder(Adapter):
    name = "extremitypathfinder"
    label = "extremitypathfinder"

    def __init__(self):
        from extremitypathfinder import PolygonEnvironment

        self._cls = PolygonEnvironment

    def version(self) -> str:
        from importlib.metadata import version

        return version("extremitypathfinder")

    @staticmethod
    def numba_active() -> bool:
        from extremitypathfinder import utils_numba

        return hasattr(utils_numba._lies_behind_inner, "nopython_signatures")

    def build(self, holes):
        # the input is valid by construction (checked with Shapely when the maps are generated)
        return self._cls(None, holes, validate=False)

    def query(self, env, start, goal):
        # the query points are in free space by construction, pyvisgraph does not check either
        path, _ = env.find_shortest_path(start, goal, verify=False)
        return [(float(x), float(y)) for x, y in path]

    def graph_size(self, env):
        return env.graph.number_of_nodes(), env.graph.number_of_edges()


class PyVisGraph(Adapter):
    name = "pyvisgraph"
    label = "pyvisgraph"

    def __init__(self):
        import pyvisgraph

        self._vg = pyvisgraph

    def version(self) -> str:
        from importlib.metadata import version

        return version("pyvisgraph")

    def build(self, holes):
        polygons = [[self._vg.Point(x, y) for x, y in hole] for hole in holes]
        graph = self._vg.VisGraph()
        graph.build(polygons, workers=1, status=False)
        return graph

    def query(self, env, start, goal):
        path = env.shortest_path(self._vg.Point(*start), self._vg.Point(*goal))
        return [(p.x, p.y) for p in path]

    def graph_size(self, env):
        return len(env.visgraph.graph), len(env.visgraph.edges)


ADAPTERS = {cls.name: cls for cls in (ExtremityPathfinder, PyVisGraph)}
PACKAGES = tuple(ADAPTERS)


def get_adapter(name: str) -> Adapter:
    return ADAPTERS[name]()
