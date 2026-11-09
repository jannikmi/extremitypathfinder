"""audit of the floating point decisions against exact rational arithmetic (#3)

Every decision of the package that depends on floating point rounding is checked on random inputs:
the float decision of the package against the exact decision for the same float64 values,
computed with ``fractions.Fraction`` (``tests/exact.py``, shared with ``tests/test_exact_decisions.py``,
which asserts the result on integer coordinates). Reports the number of misdecisions per function and kind of input.

Decisions:

* the order of the angle representations (``_compute_repr_n_dist()``, ``reps_n_distances()``),
  the basis of the angle ranges, the free wedges and the comparisons of directions
* ``_lies_on_edge()``: contacts and query points on edges
* ``_inside_polygon()``: the ray casting point in polygon test (and the edge test above)
* ``_has_clockwise_numbering()``: the orientation of the input polygons
* ``Point.in_wedges()``: directions near a bounding ray, compared exactly within ``RAY_TOLERANCE``
* ``_lies_behind_inner()``: whether a vertex lies behind an edge (Cramer's rule, #185)
* the validation (#193): ``utils._crossing_parameters()``, whether two edges cross in their interiors
  (Cramer's rule without division). Its bounding box prefilter compares coordinates only (exact).
  The contact checks (``check_polygon_contacts()``) decide with ``_lies_on_edge()`` and the angle representations,
  audited above

Inputs: integers (small and up to 2^20, e.g. grids), a decimal grid (0.1), projected coordinates
(1e6 plus a 0.01 grid), random floats of several magnitudes, and near-degenerate cases
(directions closer than the precision of the angle representations, points next to an edge).
Misdecisions of the angle representations are reported with the largest exact difference of the representations.

Measured (November 2026, -n 100000, identical with and without Numba): no misdecisions when all coordinates,
query points included, are integers.
Not covered: ``Point.in_front()`` and ``free_wedges()`` (#212).
All misdecisions concern directions or points closer than about one unit in the last place (1e-16 relative)
to a decision boundary, cf. the Numerical Policy in docs/8_design.rst.

usage:
    python prototypes/float_audit.py -n 100000
    python prototypes/float_audit.py -n 100000 --no-numba
"""

import argparse
import importlib
import sys
from collections import Counter
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument(
    "-n", type=int, default=100_000, help="cases per function and input kind"
)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument(
    "--no-numba", action="store_true", help="run as if Numba was not installed"
)
args = parser.parse_args()
if args.no_numba:
    sys.modules["numba"] = None

# imported after blocking Numba (--no-numba). The exact references are shared with the tests
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
exact = importlib.import_module("tests.exact")
utils = importlib.import_module("extremitypathfinder.utils")
utils_numba = importlib.import_module("extremitypathfinder.utils_numba")
wedges = importlib.import_module("extremitypathfinder.wedges")
_compute_repr_n_dist = utils_numba._compute_repr_n_dist
_has_clockwise_numbering = utils_numba._has_clockwise_numbering
_inside_polygon = utils_numba._inside_polygon
_lies_behind_inner = utils_numba._lies_behind_inner
_lies_on_edge = utils_numba._lies_on_edge
reps_n_distances = utils_numba.reps_n_distances
Point = wedges.Point
_rep = wedges._rep

N = args.n
rng = np.random.default_rng(args.seed)
MODE = "numba" if hasattr(utils_numba._lies_on_edge, "py_func") else "python"
print(f"mode: {MODE}, n={N}, seed={args.seed}", file=sys.stderr)


# --- input generators: arrays of shape (n, k, 2) -----------------------------------------
def ints(n, k, lim=20):
    return rng.integers(-lim, lim + 1, size=(n, k, 2)).astype(float)


def large_ints(n, k):
    return ints(n, k, lim=2**20)


def decimal_grid(n, k):
    return rng.integers(-200, 201, size=(n, k, 2)) * 0.1


def decimal_offset(n, k):
    # projected coordinates: large offsets and decimals
    return 1e6 + rng.integers(-2000, 2001, size=(n, k, 2)) * 0.01


def floats(n, k):
    return rng.uniform(-1e3, 1e3, size=(n, k, 2)) * 10.0 ** rng.integers(
        -3, 3, size=(n, 1, 1)
    )


GENERATORS = {
    "integers (±20)": ints,
    "integers (±2^20)": large_ints,
    "decimal grid 0.1": decimal_grid,
    "1e6 + 0.01 grid": decimal_offset,
    "random floats": floats,
}


def sign(x):
    return int(x > 0) - int(x < 0)


# --- audits ------------------------------------------------------------------------------------
results = []


def report(function, inputs, counts, n):
    results.append((function, inputs, n, dict(counts)))
    print(f"{function:<32} {inputs:<20} n={n:>8} {dict(counts)}", file=sys.stderr)


def near_degenerate_pairs(n):
    """pairs of directions differing by one integer unit at a large scale, and parallel pairs"""
    base = rng.integers(-(2**20), 2**20 + 1, size=(n, 2)).astype(float)
    k = rng.integers(1, 1000, size=(n, 1)).astype(float)
    choice = rng.integers(0, 3, size=n)
    other = base.copy()
    other[choice == 0] = (
        base[choice == 0] * k[choice == 0]
    )  # parallel, different length
    other[choice == 1] = base[choice == 1] + rng.integers(
        -1, 2, size=(int((choice == 1).sum()), 2)
    )
    other[choice == 2] = base[choice == 2] * (
        1 + 1e-12 * rng.standard_normal((int((choice == 2).sum()), 1))
    )
    return np.stack([base, other], axis=1)


def audit_reps():
    gens = dict(GENERATORS)
    gens["near-degenerate pairs"] = lambda n, k: near_degenerate_pairs(n)
    for name, gen in gens.items():
        vs = gen(N, 2)
        c = Counter()
        vec_reps, _ = reps_n_distances(vs.reshape(-1, 2))
        vec_reps = vec_reps.reshape(-1, 2)
        for i, (u, v) in enumerate(vs):
            eu, ev = exact.rep(u), exact.rep(v)
            if eu is None or ev is None:
                continue
            ru = _compute_repr_n_dist(u)[0]
            rv = _compute_repr_n_dist(v)[0]
            if ru != vec_reps[i, 0] or rv != vec_reps[i, 1]:
                c["vectorised differs"] += 1
            s_exact, s_float = sign(eu - ev), sign(ru - rv)
            if s_exact == s_float:
                continue
            c.setdefault("max |exact diff|", 0.0)
            c["max |exact diff|"] = max(c["max |exact diff|"], float(abs(eu - ev)))
            if s_exact == 0:
                c["same direction, different rep"] += 1
            elif s_float == 0:
                c["different directions, same rep"] += 1
            else:
                c["order inverted"] += 1
        report("angle representation order", name, c, N)


def audit_lies_on_edge():
    gens = dict(GENERATORS)

    def on_edge_points(n, k):
        # points on the edge: integer edges and integer points on them, and rounded interpolations
        p1 = ints(n, 1, 1000)[:, 0]
        d = ints(n, 1, 50)[:, 0]
        t = rng.integers(0, 4, size=(n, 1))
        p2 = p1 + 3 * d
        p_exact = p1 + t * d
        frac = rng.uniform(0, 1, size=(n, 1))
        p_round = (p1 * 0.1) + frac * (p2 * 0.1 - p1 * 0.1)
        use_round = rng.integers(0, 2, size=(n, 1)).astype(bool)
        p = np.where(use_round, p_round, p_exact)
        a = np.where(use_round, p1 * 0.1, p1)
        b = np.where(use_round, p2 * 0.1, p2)
        return np.stack([p, a, b], axis=1)

    gens["on / near the edge"] = on_edge_points
    for name, gen in gens.items():
        pts = gen(N, 3)
        c = Counter()
        for p, p1, p2 in pts:
            e = exact.on_edge(p, p1, p2)
            f = _lies_on_edge(p, p1, p2)
            if e != f:
                c["false positive" if f else "false negative"] += 1
            elif e:
                c["(on edge)"] += 1
        report("_lies_on_edge()", name, c, N)


def random_star_polygon(gen, nr_vertices):
    """a simple polygon: vertices sorted by angle around their mean"""
    while True:
        pts = gen(1, nr_vertices)[0]
        pts = np.unique(pts, axis=0)
        if len(pts) < 3:
            continue
        c = pts.mean(axis=0)
        ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
        order = np.lexsort((np.hypot(*(pts - c).T), ang))
        poly = pts[order]
        if exact.signed_area2(poly) != 0:
            return np.ascontiguousarray(poly)


def audit_inside_polygon():
    nr_polygons = max(N // 200, 10)
    for name, gen in GENERATORS.items():
        c = Counter()
        n = 0
        for _ in range(nr_polygons):
            poly = random_star_polygon(gen, int(rng.integers(3, 12)))
            # random points, vertices, edge midpoints and points one unit in the last place next to them
            queries = [(False, q) for q in gen(100, 1)[:, 0]]
            for i in range(len(poly)):
                p1, p2 = poly[i - 1], poly[i]
                queries.append((False, p1.copy()))
                queries.append((False, (p1 + p2) / 2))
                queries.append((True, np.nextafter((p1 + p2) / 2, np.inf)))
            for one_ulp_off, p in queries:
                p = np.ascontiguousarray(p, dtype=float)
                for border in (True, False):
                    n += 1
                    e = exact.inside_polygon(p, poly, border)
                    f = _inside_polygon(p, poly, border)
                    if e != f:
                        on_edge_rule = any(
                            _lies_on_edge(p, *edge) != exact.on_edge(p, *edge)
                            for edge in zip(np.roll(poly, 1, axis=0), poly)
                        )
                        kind = "via _lies_on_edge()" if on_edge_rule else "ray casting"
                        if one_ulp_off:
                            kind += " (1 ulp off an edge)"
                        c[kind] += 1
        report("_inside_polygon()", name, c, n)


def audit_clockwise():
    nr = max(N // 10, 10)
    gens = dict(GENERATORS)

    def thin(n, k):
        # thin triangles far from the origin: area tiny compared with the coordinates
        base = 1e6 + rng.integers(-1000, 1000, size=(n, 1, 2)) * 0.01
        d = rng.uniform(-1, 1, size=(n, 1, 2))
        eps = 10.0 ** rng.integers(-12, -6, size=(n, 1, 1))
        perp = np.stack([-d[..., 1], d[..., 0]], axis=-1)
        t = rng.uniform(0.1, 0.9, size=(n, 1, 1))
        return np.concatenate([base, base + d, base + t * d + eps * perp], axis=1)

    gens["thin triangles at 1e6"] = thin
    for name, gen in gens.items():
        c = Counter()
        n = 0
        for _ in range(nr):
            if name == "thin triangles at 1e6":
                poly = np.ascontiguousarray(gen(1, 3)[0])
            else:
                poly = random_star_polygon(gen, int(rng.integers(3, 12)))
                if rng.integers(0, 2):
                    poly = np.ascontiguousarray(poly[::-1])
            a = exact.signed_area2(poly)
            if a == 0:
                continue
            n += 1
            if (a > 0) != _has_clockwise_numbering(poly):
                c["wrong"] += 1
        report("_has_clockwise_numbering()", name, c, n)


def audit_in_wedges():
    """Point.in_wedges(): the RAY_TOLERANCE selection of exact comparisons with the bounding rays"""
    gens = {
        "integers (±20)": lambda n: ints(n, 3)[:, :, :],
        "integers (±2^20)": lambda n: large_ints(n, 3),
        "decimal grid 0.1": lambda n: decimal_grid(n, 3),
    }

    def near_bound(n):
        a = rng.integers(-(2**20), 2**20 + 1, size=(n, 2)).astype(float)
        b = rng.integers(-(2**20), 2**20 + 1, size=(n, 2)).astype(float)
        bound = np.where(rng.integers(0, 2, size=(n, 1)).astype(bool), a, b)
        k = rng.integers(1, 1000, size=(n, 1)).astype(float)
        choice = rng.integers(0, 3, size=(n, 1))
        d = np.where(choice == 0, bound * k, bound + rng.integers(-1, 2, size=(n, 2)))
        d = np.where(choice == 2, bound * 0.1 * k, d)
        return np.stack([a, b, d], axis=1)

    gens["near a bounding ray"] = near_bound
    for name, gen in gens.items():
        c = Counter()
        n = 0
        for a, b, d in gen(N):
            if not (a.any() and b.any() and d.any()):
                continue
            ra, rb = _rep(a), _rep(b)
            if ra == rb:
                continue
            n += 1
            point = Point(0, 2, set(), [(ra, rb)], None, [(a, ra), (b, rb)])
            f = point.in_wedges(_rep(d), d, [(ra, rb)])
            e = exact.in_closure(d, a, b)
            if e != f:
                c["false positive" if f else "false negative"] += 1
                c.setdefault("max |exact diff|", 0.0)
                er = exact.rep(d)
                diff = min(abs(er - exact.rep(a)), abs(er - exact.rep(b)))
                c["max |exact diff|"] = max(c["max |exact diff|"], float(diff))
        report("Point.in_wedges() (RAY_TOLERANCE)", name, c, n)


def audit_lies_behind():
    for name, gen in GENERATORS.items():
        c = Counter()
        n = 0
        for p1, p2, v in gen(N, 3):
            e = exact.lies_behind(p1, p2, v)
            if e is None:
                continue
            n += 1
            a, b = p1[0] - p2[0], p1[1] - p2[1]
            if a * v[1] - v[0] * b == 0.0:
                # the exact determinant is not 0: decided as parallel ("not behind")
                c["float determinant 0, exact not"] += 1
            f = _lies_behind_inner(p1, p2, v)
            if e != f:
                c["wrong"] += 1
        report("_lies_behind_inner()", name, c, n)


def audit_crossing():
    """the crossing test of the validation, on random edge pairs and on edges through a vertex of the other edge"""
    gens = dict(GENERATORS)

    def through_vertex(scale):
        def generator(n, k):
            # q1 lies on p1-p2 (exactly for integers, rounded for the decimals), q2 off the edge:
            # crossings close to a vertex, decided by lambda or mu close to 0 or 1
            p1 = ints(n, 1, 1000)[:, 0]
            d = ints(n, 1, 50)[:, 0]
            p2 = p1 + 3 * d
            q1 = p1 + rng.integers(0, 4, size=(n, 1)) * d
            q2 = q1 + ints(n, 1, 50)[:, 0]
            return np.stack([p1, p2, q1, q2], axis=1) * scale

        return generator

    gens["through a vertex (integers)"] = through_vertex(1.0)
    gens["through a vertex (0.1 grid)"] = through_vertex(0.1)
    for name, gen in gens.items():
        edges = gen(N, 4)
        crossing, _, _ = utils._crossing_parameters(
            edges[:, 0], edges[:, 1], edges[:, 2], edges[:, 3]
        )
        c = Counter()
        for (p1, p2, q1, q2), f in zip(edges, crossing):
            e = exact.crossing(p1, p2, q1, q2)
            if e != f:
                c["false positive" if f else "false negative"] += 1
                # every decision is the side of a vertex of one edge relative to the line of the other edge
                # (d, t, d - t, u, d - u in utils._crossing_parameters()):
                # the smallest exact relative distance (sine of the angle) of a vertex to the other line
                distances = []
                for (a, b), others in (((p1, p2), (q1, q2)), ((q1, q2), (p1, p2))):
                    for v in others:
                        if np.any(v != a):
                            lengths = float(np.hypot(*(b - a)) * np.hypot(*(v - a)))
                            distances.append(abs(exact.cross(b - a, v - a)) / lengths)
                distance = min(distances)
                c.setdefault("max |exact diff|", 0.0)
                c["max |exact diff|"] = max(c["max |exact diff|"], float(distance))
            elif e:
                c["(crossing)"] += 1
        report("_crossing_parameters()", name, c, N)


for audit in (
    audit_reps,
    audit_lies_on_edge,
    audit_inside_polygon,
    audit_clockwise,
    audit_in_wedges,
    audit_lies_behind,
    audit_crossing,
):
    audit()

print("\n| Function | Inputs | Cases | Misdecisions |")
print("|---|---|---|---|")
for function, inputs, n, counts in results:
    shown = [
        f"{k}: {v:.1e}" if isinstance(v, float) else f"{k}: {v:,}"
        for k, v in counts.items()
        if not k.startswith("(")
    ]
    shown = ", ".join(shown) if shown else "0"
    print(f"| {function} | {inputs} | {n:,} | {shown} |")
