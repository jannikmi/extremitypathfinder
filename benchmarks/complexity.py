"""The runtime complexity terms of the compared packages, evaluated for the benchmarked maps.

The terms are the ``O(...)`` expressions without the constant factor, with ``n`` vertices, ``m`` extremities
(the nodes of the extremitypathfinder graph) and ``E`` edges of the visibility graph of the package.
They are defined here only in code: update them here when the complexities change, and the text of
``docs/7_performance.rst`` (cf. the ``benchmark-derived`` directive there).

* extremitypathfinder: ``docs/8_design.rst`` ("Complexity and Memory"). Typical case: build ``m n log n``
  for the visibility graph plus ``n^2`` for the wedges, query ``n log n + m`` for the visibility of start and goal
  plus ``E log E`` for the A* search. Worst case (many edges spanning the directions of many extremities,
  AREA with a tree over the candidates): ``m n log^2 n`` and ``n log^2 n``.
* pyvisgraph: Lee's rotational sweep, build ``n^2 log n``. A query computes the visibility of start and goal
  (two sweeps, ``n log n`` each) and runs a heap based Dijkstra, ``E log n``
  (derived from the source of pyvisgraph 0.2.1).
  The implementation takes ``n^3`` in the worst case (``docs/8_design.rst``).
  The worst cases (``WORST_CASE``) are shown as references.

The terms carry no constants: they say how the work grows, not how fast an implementation is.
:func:`series` fits one constant per package (at its smallest map) to put the terms on the scale of the measured times.
"""

import math
from collections.abc import Callable

from benchmarks import measure

EPF = "extremitypathfinder"
PVG = "pyvisgraph"

# kind -> package -> (formula as text, term of n, m, E)
Term = Callable[[int, int, int], float]
TERMS: dict[str, dict[str, tuple[str, Term]]] = {
    "build": {
        EPF: ("m·n·log n + n²", lambda n, m, e: m * n * math.log2(n) + n * n),
        PVG: ("n²·log n", lambda n, m, e: n * n * math.log2(n)),
    },
    "query": {
        EPF: (
            "n·log n + m + E·log E",
            lambda n, m, e: n * math.log2(n) + m + e * math.log2(e),
        ),
        PVG: (
            "2·n·log n + E·log n",
            lambda n, m, e: 2 * n * math.log2(n) + e * math.log2(n),
        ),
    },
}
# reference: the documented worst cases (``docs/8_design.rst``, "Complexity and Memory"). Only the terms the
# documentation states: the worst case of the visibility of a query point of pyvisgraph is not stated
WORST_CASE: dict[str, dict[str, tuple[str, Term]]] = {
    "build": {
        EPF: (
            "m·n·log² n + n²",
            lambda n, m, e: m * n * math.log2(n) ** 2 + n * n,
        ),
        PVG: ("n³", lambda n, m, e: n**3),
    },
    "query": {
        EPF: (
            "n·log² n + m + E·log E",
            lambda n, m, e: n * math.log2(n) ** 2 + m + e * math.log2(e),
        ),
    },
}
# build: invalid runs built a complete graph (their time is valid), the query terms need the valid graph edges
STATUSES = {"build": (measure.OK, measure.INVALID), "query": (measure.OK,)}


def _measured(result: dict, kind: str) -> float:
    if kind == "build":
        return result.get("build_time")
    return (result.get("query_stats") or {}).get("p50")


def rows(results: dict, kind: str, package: str) -> list[dict]:
    """per map where the package has results, ascending in vertices: vertices, extremities, edges, term, time (s)

    The edges and extremities are needed (and the extremitypathfinder result must be valid for queries)
    only by the terms that use them: the build term of pyvisgraph needs the vertices only.
    """
    term = TERMS[kind][package][1]
    uses_graph = (kind, package) != ("build", PVG)
    out = []
    for entry in results["world"]:
        result = entry["results"].get(package)
        epf = entry["results"].get(EPF) or {}
        if result is None or result["status"] not in STATUSES[kind]:
            continue
        if uses_graph and epf.get("status") not in STATUSES[kind]:
            continue
        n, m, e = (
            entry["map"]["vertices"],
            epf.get("graph_nodes"),
            result.get("graph_edges"),
        )
        time = _measured(result, kind)
        graph_missing = uses_graph and (m is None or e is None or min(m, e) < 2)
        if time is None or n < 2 or graph_missing:
            continue
        out.append(
            {
                "vertices": n,
                "extremities": m,
                "edges": e,
                "term": term(n, m or 0, e or 0),
                "time": time,
            }
        )
    return sorted(out, key=lambda row: row["vertices"])


def series(results: dict, kind: str, package: str) -> tuple[list[dict], float]:
    """the rows and the constant ``c`` of the package: ``c * term`` equals the measured time at the smallest map"""
    data = rows(results, kind, package)
    c = data[0]["time"] / data[0]["term"] if data else 1.0
    return data, c


def ratios(results: dict, kind: str) -> list[dict]:
    """per map where both packages have results: the ratio pyvisgraph / extremitypathfinder of terms and of times"""
    epf = {r["vertices"]: r for r in rows(results, kind, EPF)}
    pvg = {r["vertices"]: r for r in rows(results, kind, PVG)}
    return [
        {
            "vertices": n,
            "term": pvg[n]["term"] / epf[n]["term"],
            "time": pvg[n]["time"] / epf[n]["time"],
        }
        for n in sorted(epf.keys() & pvg.keys())
    ]
