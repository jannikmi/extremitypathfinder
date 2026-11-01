"""Benchmark workloads: the world map size sweep, small examples and seeded query generation.

All maps are unbounded (``boundary=None``) with simple, pairwise disjoint polygons as obstacles,
the common input semantics of extremitypathfinder and pyvisgraph (see ``benchmarks/PLAN.md``).
"""

import json
import random
from dataclasses import asdict, dataclass
from pathlib import Path

import shapely
from shapely.geometry import Point, Polygon

from benchmarks import world_map

# queries keep at least this distance (degrees) to the land of every map of the sweep
QUERY_EPS = 0.1
# query points are drawn uniformly from this region (longitude, latitude)
QUERY_REGION = ((-180.0, 180.0), (-60.0, 75.0))
DEFAULT_NR_QUERIES = 200
DEFAULT_SEED = 0
MAP_DIR = world_map.DEFAULT_CACHE_DIR / "maps"

Coords = list[tuple[float, float]]
Query = tuple[tuple[float, float], tuple[float, float]]


@dataclass(frozen=True)
class MapSpec:
    """parameters of a world map of the sweep (deterministic for a given Natural Earth release)"""

    name: str
    # None: the coarsest resolution with enough vertices
    resolution: str | None = None
    # None: no downsampling
    target_vertices: int | None = None
    grid: float = world_map.DEFAULT_GRID


def _spec(target: int) -> MapSpec:
    label = f"{target // 1000}k" if target >= 1000 else str(target)
    return MapSpec(f"world_{label}", None, target)


# the raw 110m data has 4,954 vertices after cleaning: it is the 5k step of the sweep
WORLD_SWEEP = (
    *(_spec(n) for n in (250, 500, 1000, 2000)),
    MapSpec("world_5k", "110m", None),
    *(_spec(n) for n in (10_000, 20_000, 50_000, 100_000, 200_000, 400_000)),
)
QUICK_SWEEP = WORLD_SWEEP[:2]
SWEEP_BY_NAME = {spec.name: spec for spec in WORLD_SWEEP}


def parse_size(size: str) -> MapSpec:
    """a sweep step from its name or vertex count: ``world_2k``, ``2k``, ``2000``, ``5k`` (raw 110m)"""
    if size in SWEEP_BY_NAME:
        return SWEEP_BY_NAME[size]
    if f"world_{size}" in SWEEP_BY_NAME:
        return SWEEP_BY_NAME[f"world_{size}"]
    number = size.lower().replace("_", "")
    target = int(float(number[:-1]) * 1000) if number.endswith("k") else int(number)
    for spec in WORLD_SWEEP:
        if spec.target_vertices == target:
            return spec
    return _spec(target)


def load_world(spec: MapSpec, cache_dir: Path = MAP_DIR) -> dict:
    """the cleaned (and downsampled) world map, cached as JSON with its parameters in the header

    The cache is regenerated when its parameters differ from ``spec``.
    """
    path = cache_dir / f"{spec.name}.json"
    params = asdict(spec)
    if path.exists():
        with open(path) as f:
            data = json.load(f)
        if (data.get("spec"), data.get("ne_release")) == (params, world_map.NE_RELEASE):
            return data
    data = world_map.build_map(
        spec.resolution,
        spec.target_vertices,
        spec.grid,
        cache_dir=world_map.DEFAULT_CACHE_DIR,
    )
    data["spec"] = params
    data["ne_release"] = world_map.NE_RELEASE
    data["name"] = spec.name
    cache_dir.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f)
    return data


def map_summary(data: dict) -> dict:
    """the map parameters without the coordinates"""
    return {k: v for k, v in data.items() if k not in ("boundary", "holes")}


def _small_examples() -> dict[str, list[Coords]]:
    """the test environments of the package which fit the common semantics, as unbounded maps

    The boundaries are dropped. Intersecting holes are separated (the smaller one is dropped).
    """
    from tests import test_cases

    envs = {
        "separated": test_cases.SEPARATED_ENV,
        "poly": test_cases.POLY_ENV_PARAMS,
        "overlap": test_cases.OVERLAP_POLY_ENV_PARAMS,
    }
    examples = {}
    for name, (_, holes) in envs.items():
        polygons, _ = world_map._separate([Polygon(h) for h in holes])
        world_map.check(polygons)
        examples[name] = [list(p.exterior.coords[:-1]) for p in polygons]
    return examples


def small_example_maps() -> list[dict]:
    """small examples (smoke test, thin wrapper): unbounded maps in the format of :func:`load_world`"""
    maps = []
    for name, holes in _small_examples().items():
        maps.append(
            {
                "name": f"example_{name}",
                "source": "tests/test_cases.py (boundary dropped)",
                "polygons": len(holes),
                "vertices": sum(len(h) for h in holes),
                "boundary": None,
                "holes": [[list(c) for c in h] for h in holes],
            }
        )
    return maps


def land(maps: list[dict]):
    """the union of the land (all polygons) of the maps"""
    polygons = [Polygon(h) for data in maps for h in data["holes"]]
    return shapely.union_all(polygons)


def generate_queries(
    maps: list[dict],
    nr_queries: int = DEFAULT_NR_QUERIES,
    seed: int = DEFAULT_SEED,
    eps: float = QUERY_EPS,
    region: tuple[tuple[float, float], tuple[float, float]] | None = QUERY_REGION,
) -> list[Query]:
    """seeded random (start, goal) pairs in the free space of all the maps

    A point is rejected if it lies within ``eps`` of the land of any map:
    every point is strictly in free space at every size.
    The pairs are in random order, so any prefix is an unbiased sample.

    :param region: the sampling region ((x_min, x_max), (y_min, y_max)).
        None: the bounding box of the land, extended by 10 %.
    """
    blocked = land(maps).buffer(eps)
    shapely.prepare(blocked)
    if region is None:
        x_min, y_min, x_max, y_max = blocked.bounds
        dx, dy = 0.1 * (x_max - x_min), 0.1 * (y_max - y_min)
        region = ((x_min - dx, x_max + dx), (y_min - dy, y_max + dy))
    (x_min, x_max), (y_min, y_max) = region
    rng = random.Random(seed)

    def free_point() -> tuple[float, float]:
        while True:
            point = (rng.uniform(x_min, x_max), rng.uniform(y_min, y_max))
            if not blocked.intersects(Point(point)):
                return point

    return [(free_point(), free_point()) for _ in range(nr_queries)]
