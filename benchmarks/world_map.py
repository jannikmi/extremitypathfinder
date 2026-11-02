"""World land polygons as obstacles: download, clean and downsample Natural Earth data.

Produces unbounded polygon environments (land masses as holes, the sea is the free space)
which both extremitypathfinder and pyvisgraph accept without changing their meaning:

* simple polygons only: multipolygons are split, interior rings (lakes) are dropped
* no two polygons intersect or touch (touching obstacles block paths in extremitypathfinder only)
* coordinates snapped to a grid and consecutive duplicate vertices removed.
  Near-identical vertices crash the pyvisgraph build (``math domain error`` in ``angle2``).
* self-touching polygons are repaired with a tiny outward buffer

Coordinates are longitude/latitude degrees used as planar coordinates:
distances are not geodesic and paths do not wrap around the antimeridian.

Optionally the polygons are downsampled (simplified) to a target number of vertices,
which allows a sweep over environment sizes with realistic geometry.

Data: Natural Earth land polygons (public domain), https://www.naturalearthdata.com,
GeoJSON from https://github.com/nvkelso/natural-earth-vector (pinned release, checksums verified).

Usage::

    uv run --group benchmark python -m benchmarks.world_map --resolution 110m -o world_110m.json
    uv run --group benchmark python -m benchmarks.world_map --target-vertices 2000 -o world_2k.json
"""

import argparse
import hashlib
import json
import math
import sys
import urllib.request
from pathlib import Path

import shapely
from shapely.geometry import Polygon, shape
from shapely.strtree import STRtree
from shapely.validation import explain_validity

NE_RELEASE = "v5.1.2"
NE_URL = (
    "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/"
    f"{NE_RELEASE}/geojson/ne_{{resolution}}_land.geojson"
)
NE_SHA256 = {
    "110m": "9e0729ee253ca7d7a5c4ae9395fb1902264c5377c52e224d13dd85010e2835d9",
    "50m": "e874b27a51d146452be360cafb3cc50c86001074a67d534113e6534682f9826b",
    "10m": "1ac90796408bc6ad6911d69448485d3c4dbf2190370080368a09976e1c9f7416",
}
# from coarse to fine
RESOLUTIONS = ("110m", "50m", "10m")
DEFAULT_CACHE_DIR = Path(__file__).parent / "data"
# ~11 m at the equator
DEFAULT_GRID = 1e-4
# relative tolerance of the reached number of vertices when downsampling
TARGET_RTOL = 0.02
MAX_BISECTION_STEPS = 60

Coords = list[tuple[float, float]]


class DownloadError(OSError):
    """the source data could not be downloaded or has the wrong checksum (an external failure)"""


def download(resolution: str, cache_dir: Path = DEFAULT_CACHE_DIR) -> Path:
    """download the Natural Earth land polygons (once) and verify the checksum"""
    if resolution not in NE_SHA256:
        raise ValueError(f"resolution must be one of {RESOLUTIONS}, got {resolution}")
    cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / f"ne_{resolution}_land_{NE_RELEASE}.geojson"
    if not path.exists():
        url = NE_URL.format(resolution=resolution)
        print(f"downloading {url}", file=sys.stderr)
        tmp = path.with_suffix(".part")
        try:
            urllib.request.urlretrieve(url, tmp)
        except OSError as exc:
            raise DownloadError(f"cannot download {url}: {exc}") from exc
        tmp.rename(path)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != NE_SHA256[resolution]:
        raise DownloadError(
            f"checksum mismatch of {path}: {digest} != {NE_SHA256[resolution]}. "
            "Delete the file to download it again."
        )
    return path


def load_polygons(path: Path) -> list[Polygon]:
    """all polygons of a GeoJSON file, multipolygons split, interior rings (lakes) dropped"""
    with open(path) as f:
        collection = json.load(f)
    polygons = []
    for feature in collection["features"]:
        geometry = shape(feature["geometry"])
        parts = getattr(geometry, "geoms", [geometry])
        polygons.extend(Polygon(p.exterior) for p in parts)
    return polygons


def nr_vertices(polygons: list[Polygon]) -> int:
    return sum(len(p.exterior.coords) - 1 for p in polygons)


def _snap(polygon: Polygon, grid: float) -> Coords:
    """exterior coordinates snapped to the grid, without closing and consecutive duplicate vertices"""
    decimals = max(0, round(-math.log10(grid)))
    coords = [
        (
            round(round(x / grid) * grid, decimals),
            round(round(y / grid) * grid, decimals),
        )
        for x, y in polygon.exterior.coords[:-1]
    ]
    return [c for i, c in enumerate(coords) if c != coords[i - 1]]


def _simple_parts(polygon: Polygon, grid: float) -> list[Polygon]:
    """valid polygons without self contacts (each vertex position used once) on the grid"""
    result = []
    pending = [polygon]
    while pending:
        candidate = pending.pop()
        coords = _snap(candidate, grid)
        if len(set(coords)) < 3:
            continue  # collapsed
        snapped = Polygon(coords)
        if snapped.is_valid and len(set(coords)) == len(coords) and snapped.area > 0:
            result.append(snapped)
            continue
        if snapped.is_valid and snapped.area > 0:
            # self contact (repeated vertex): merge the touching parts with a tiny outward buffer
            repaired = snapped.buffer(grid, join_style="mitre")
        else:
            repaired = shapely.make_valid(snapped)
        parts = [
            Polygon(p.exterior)
            for p in getattr(repaired, "geoms", [repaired])
            if isinstance(p, Polygon) and p.area > 0
        ]
        if parts == [snapped]:
            raise RuntimeError(f"cannot repair polygon: {explain_validity(snapped)}")
        pending.extend(parts)
    return result


def _separate(polygons: list[Polygon]) -> tuple[list[Polygon], int]:
    """drop the smaller polygon of every intersecting or touching pair

    :return: the remaining polygons and the number of dropped polygons
    """
    order = sorted(range(len(polygons)), key=lambda i: polygons[i].area, reverse=True)
    polygons = [polygons[i] for i in order]
    tree = STRtree(polygons)
    dropped = set()
    for i, polygon in enumerate(polygons):
        if i in dropped:
            continue
        for j in tree.query(polygon, predicate="intersects"):
            if j > i and j not in dropped:
                dropped.add(int(j))
    return [p for i, p in enumerate(polygons) if i not in dropped], len(dropped)


def clean(
    polygons: list[Polygon], grid: float = DEFAULT_GRID, tolerance: float = 0.0
) -> tuple[list[Polygon], dict]:
    """simplify (optional), snap, repair and separate the polygons

    :param tolerance: Douglas-Peucker simplification tolerance in degrees (0: no simplification).
        Polygons with an area below ``tolerance**2`` are dropped (slivers, tiny islands).
    :return: the cleaned polygons and statistics of the cleaning
    """
    stats = {"input_polygons": len(polygons), "input_vertices": nr_vertices(polygons)}
    if tolerance > 0:
        polygons = [p.simplify(tolerance, preserve_topology=True) for p in polygons]
        polygons = [p for p in polygons if p.area >= tolerance**2]
    stats["after_simplify_polygons"] = len(polygons)
    cleaned = [part for p in polygons for part in _simple_parts(p, grid)]
    cleaned, nr_dropped = _separate(cleaned)
    stats["dropped_intersecting"] = nr_dropped
    stats["polygons"] = len(cleaned)
    stats["vertices"] = nr_vertices(cleaned)
    return cleaned, stats


def downsample(
    polygons: list[Polygon], target_vertices: int, grid: float = DEFAULT_GRID
) -> tuple[list[Polygon], dict]:
    """clean the polygons with the simplification tolerance reaching the target number of vertices

    bisection of the tolerance in log space, the result has at most ``target_vertices`` vertices
    and lies within ``TARGET_RTOL`` of it if possible
    """
    cleaned, stats = clean(polygons, grid)
    if stats["vertices"] <= target_vertices:
        stats["tolerance"] = 0.0
        return cleaned, stats
    lo, hi = math.log10(grid), 2.0  # tolerances: grid size ... 100 degrees
    best = None
    for _ in range(MAX_BISECTION_STEPS):
        mid = (lo + hi) / 2
        cleaned, stats = clean(polygons, grid, 10**mid)
        stats["tolerance"] = 10**mid
        if stats["vertices"] > target_vertices:
            lo = mid
            continue
        hi = mid
        best = cleaned, stats
        if stats["vertices"] >= (1 - TARGET_RTOL) * target_vertices:
            break
    if best is None:
        raise RuntimeError(f"cannot reach {target_vertices} vertices")
    return best


def check(polygons: list[Polygon], validate_epf: bool = False):
    """verify the requirements of both packages, raises ValueError otherwise

    The Shapely checks (simple polygons, pairwise disjoint) are fast and always run.
    :param validate_epf: additionally run the input validation of extremitypathfinder.
        Redundant with the Shapely checks and slow on large maps (94 s at 20k vertices).
    """
    for i, polygon in enumerate(polygons):
        coords = polygon.exterior.coords[:-1]
        if not polygon.is_valid or len(set(coords)) != len(coords):
            raise ValueError(f"polygon {i} is not simple: {explain_validity(polygon)}")
    _, nr_dropped = _separate(polygons)
    if nr_dropped:
        raise ValueError(f"{nr_dropped} polygons intersect or touch")
    if validate_epf:
        import numpy as np

        from extremitypathfinder import utils

        holes = [np.array(p.exterior.coords[:-1], dtype=float) for p in polygons]
        utils.check_hole_data_requirements(holes)


def build_map(
    resolution: str | None = None,
    target_vertices: int | None = None,
    grid: float = DEFAULT_GRID,
    cache_dir: Path = DEFAULT_CACHE_DIR,
    validate_epf: bool = False,
) -> dict:
    """the cleaned world map as JSON serialisable dict (``holes`` like the command line input format)

    :param resolution: the Natural Earth resolution. Default: the coarsest resolution
        with at least ``target_vertices`` vertices (110m without target).
    :param target_vertices: downsample to (at most) this number of vertices
    :param validate_epf: also run the (slow) input validation of extremitypathfinder
    """
    if resolution is None:
        resolution = RESOLUTIONS[0]
        if target_vertices is not None:
            for resolution in RESOLUTIONS:
                polygons = load_polygons(download(resolution, cache_dir))
                if nr_vertices(polygons) >= target_vertices:
                    break
    polygons = load_polygons(download(resolution, cache_dir))
    if target_vertices is None:
        cleaned, stats = clean(polygons, grid)
        stats["tolerance"] = 0.0
    else:
        cleaned, stats = downsample(polygons, target_vertices, grid)
    check(cleaned, validate_epf)
    return {
        "source": f"Natural Earth {NE_RELEASE} ne_{resolution}_land (public domain)",
        "resolution": resolution,
        "grid": grid,
        "target_vertices": target_vertices,
        **stats,
        "boundary": None,
        "holes": [[list(c) for c in p.exterior.coords[:-1]] for p in cleaned],
    }


def load_map(path: str | Path) -> tuple[None, list[Coords]]:
    """read a map written by this script: (boundary, holes) for PolygonEnvironment"""
    with open(path) as f:
        data = json.load(f)
    return None, [[tuple(c) for c in hole] for hole in data["holes"]]


def main(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--resolution", choices=RESOLUTIONS, default=None)
    parser.add_argument("--target-vertices", type=int, default=None)
    parser.add_argument(
        "--grid", type=float, default=DEFAULT_GRID, help="snapping grid size in degrees"
    )
    parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR)
    parser.add_argument(
        "--validate-epf",
        action="store_true",
        help="also run the input validation of extremitypathfinder (slow on large maps)",
    )
    parser.add_argument("-o", "--output", type=Path, required=True)
    args = parser.parse_args(argv)
    world = build_map(
        args.resolution,
        args.target_vertices,
        args.grid,
        args.cache_dir,
        args.validate_epf,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(world, f)
    summary = {k: v for k, v in world.items() if k not in ("boundary", "holes")}
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
