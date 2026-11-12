"""The runtime complexity terms of the compared packages, evaluated for the benchmarked maps.

The terms are the ``O(...)`` expressions without the constant factor, with ``n`` vertices, ``m`` extremities
(the nodes of the extremitypathfinder graph) and ``E`` edges of the visibility graph of the package.
They are the *only* place of the formulas: update them here when the complexities change.

* extremitypathfinder: typical case of ``docs/8_design.rst`` ("Complexity and Memory"),
  build ``m n log n`` for the visibility graph plus ``n^2`` for the wedges,
  query ``n log n + m`` for the visibility of start and goal plus ``E log E`` for the A* search.
* pyvisgraph: Lee's rotational sweep, build ``n^2 log n``. A query computes the visibility of start and goal
  (two sweeps, ``n log n`` each) and runs a heap based Dijkstra, ``E log n``
  (derived from the source of pyvisgraph 0.2.1).
  The implementation takes ``n^3`` in the worst case (``docs/8_design.rst``), shown as a reference.

The terms carry no constants: they say how the work grows, not how fast an implementation is.
:func:`series` fits one constant per package (at its smallest map) to put the terms on the scale of the measured times.
"""

import math
from collections.abc import Callable

from benchmarks import measure
from benchmarks.adapters import ADAPTERS

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
# reference: the worst case of the implementation of pyvisgraph
PVG_WORST_CASE_BUILD = ("n³", lambda n: n**3)
# build: invalid runs built a complete graph (their time is valid), the query terms need the valid graph edges
STATUSES = {"build": (measure.OK, measure.INVALID), "query": (measure.OK,)}


def _measured(result: dict, kind: str) -> float:
    return result["build_time"] if kind == "build" else result["query_stats"]["p50"]


def rows(results: dict, kind: str, package: str) -> list[dict]:
    """per map where the package has results: vertices, extremities, edges, term and measured time (s)"""
    term = TERMS[kind][package][1]
    out = []
    for entry in results["world"]:
        result = entry["results"].get(package)
        epf = entry["results"].get(EPF)
        if result is None or result["status"] not in STATUSES[kind]:
            continue
        if epf is None or epf["status"] not in (measure.OK, measure.INVALID):
            continue
        n, m, e = (
            entry["map"]["vertices"],
            epf.get("graph_nodes"),
            result.get("graph_edges"),
        )
        if m is None or e is None or min(n, m, e) < 2:
            continue
        out.append(
            {
                "vertices": n,
                "extremities": m,
                "edges": e,
                "term": term(n, m, e),
                "time": _measured(result, kind),
            }
        )
    return out


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


def label(package: str) -> str:
    return ADAPTERS[package].label
