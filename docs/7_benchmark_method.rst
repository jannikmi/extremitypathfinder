
.. _benchmark-method:

Benchmark method
================

How the numbers on the :ref:`performance page <speed-tests>` are measured, and what limits them.
The code is in ``benchmarks/``, how to run it is described in :ref:`running-benchmarks`.


Common input semantics
----------------------

The packages solve the same problem only on a subset of the inputs. All benchmark maps stay inside this subset:

* **Unbounded maps, obstacles only.** pyvisgraph has no boundary polygon.
  ``extremitypathfinder`` is used with ``boundary_coordinates=None``.
* **Simple, pairwise disjoint polygons.** Touching obstacles form a wall in ``extremitypathfinder``.
  pyvisgraph merges vertices with identical coordinates into one graph node and its behaviour at contacts is undefined.
* **No nearly identical vertices.** They crash the pyvisgraph build (see :ref:`pyvisgraph-defects`).
* **Query points strictly in free space**, at least 0.1° away from every polygon.
  On an edge, the two packages define "inside" differently.

The polygon orientation needs no preprocessing: both packages accept either orientation.

Maps with a boundary polygon, grids and touching or intersecting obstacles are supported by ``extremitypathfinder`` only
and are not benchmarked.


Workload: the world map
-----------------------

pyvisgraph's own documentation computes ocean routes on world shorelines.
The benchmark uses a comparable, freely redistributable dataset:
the `Natural Earth <https://www.naturalearthdata.com>`__ land polygons (public domain, release ``v5.1.2``,
SHA-256 verified), the land masses as obstacles and the sea as free space.

* **Cleaning** (``benchmarks/world_map.py``): multipolygons are split and lakes dropped,
  the coordinates snapped to a 1e-4° grid (about 11 m) without consecutive duplicates,
  self-touching polygons repaired with a 1e-4° buffer,
  and the smaller polygon of any intersecting or touching pair dropped.
  Every map is checked with Shapely: simple polygons, pairwise disjoint.
* **Size sweep**: Douglas-Peucker simplification (``preserve_topology=True``) to a target number of vertices
  (at least the target and at most 2 % more), from the coarsest Natural Earth resolution with enough vertices
  after the cleaning: 250, 500, 1k, 2k, 3k, 4k, 5k and 6k vertices.
  A map of the sweep is named after its target: e.g. the 6k map has 6,000 vertices or more.
  The tolerance is found by bisection. The simplification comes before the cleaning steps above,
  and polygons with an area below the squared tolerance are dropped (slivers, tiny islands).
  The polygons dropped because they intersect or touch another one are counted in the table below.
  This keeps realistic geometry, a realistic distribution of obstacle sizes and narrow passages at every size.
* **Queries**: one shared set of 200 seeded random point pairs for all sizes and both packages,
  so query latencies are directly comparable across the sweep.
  The points are uniform over longitude [-180, 180] and latitude [-60, 75]
  and at least 0.1° away from the land of every map of the sweep
  (the coarse maps move the coastline by up to the simplification tolerance).
  A path always exists (unbounded free space, disjoint obstacles).
* **Limitations**: longitude and latitude are used as planar coordinates.
  Distances are not geodesic and paths do not wrap around the antimeridian (like pyvisgraph's own example).
  Both packages solve the same planar problem, so the comparison is fair, but the routes are not real shipping routes.
  The query points keep a distance from the coasts, which is less representative of near-coast queries.

.. include:: ../benchmarks/results/benchmark_workloads.rst


.. _benchmark-measurements:

Measurements
------------

* **extremitypathfinder with and without Numba, warmed up**: an environment is built and queried in the same process
  before timing starts. Compilation and import times are excluded.
  Without Numba, Numba is blocked in the process, as if it was not installed. This setup is measured
  like the others, with the same budget and validation, and takes part in the check that all lengths agree.
  The comparison with pyvisgraph (ratios, graph size) uses the setup with Numba.
  pyvisgraph is pure Python. All run single-threaded (pyvisgraph ``workers=1``).
* **Build (preprocessing) time**: wall time of ``PolygonEnvironment(None, holes)``
  and ``VisGraph().build(polygons, workers=1)``.
  One build per package and size. Builds under 1 s are repeated until 5 s or 5 runs and the minimum is reported.
* **Query latency**: ``time.perf_counter_ns`` per query on the prepared environment (p50, p95, p99, mean, queries/s).
  ``extremitypathfinder`` is queried with ``verify=False``: pyvisgraph does not check the query points either.
* **Peak memory**: peak memory of the subprocess doing the build and the queries,
  reset after the imports, the warm-up and loading the input, minus its baseline at that point.
  Linux: peak resident memory (``VmHWM``), macOS: peak physical footprint (``proc_pid_rusage``,
  the memory figure of the Activity Monitor). A coarse figure: the allocator keeps freed memory.
  It is measured in the same run as the timings, without ``tracemalloc``, whose overhead would distort the timings.
  The baseline is reported separately, and the total is the sum of both: the memory of the process after
  the imports, the warm-up and loading the input (the interpreter, the package, its dependencies and,
  for ``extremitypathfinder``, Numba with the compiled functions). The peak above the baseline compares
  the data structures, the total is what a process needs.
* **Retained memory**: what the prepared environment keeps, without the temporary data of the build:
  the bytes allocated and not freed after the build, minus the same figure before it.
  The bytes are the sum of the C allocator's statistics (``malloc_zone_statistics`` on macOS,
  ``mallinfo2`` on Linux) and Python's allocator for small objects (``sys._debugmallocstats``), after a garbage collection.
  The resident memory or the footprint cannot show this: allocators keep freed memory, and macOS does not return it
  on request. The figure counts bytes and the peak counts pages of the process, so on tiny maps
  the retained memory can slightly exceed the peak. Reading it takes place after the build
  and before the queries, outside of all timings.
* **Baselines without input data**: each run measures the baselines without input data in separate processes:
  ``extremitypathfinder`` with Numba, ``extremitypathfinder`` without Numba and pyvisgraph.
  They are listed below the memory table.
* **Graph size**: nodes and edges of ``extremitypathfinder``'s reduced visibility graph
  and of pyvisgraph's full visibility graph.
* **Budget**: 10 min per build. Each package and size runs in its own subprocess, which is killed when the build
  exceeds the budget. The sizes run in ascending order. Once a package exceeds the budget it is not run at larger sizes,
  and the sweep ends when ``extremitypathfinder`` exceeds it. Data points beyond the budget are left out:
  no extrapolation. The queries are not limited by default.
  With ``--query-limit``, the queries stop after that time per package and size: the query set is in random order,
  so the queries completed are an unbiased prefix, and the statistics are reported over it
  with the number of completed queries.
* **Crashes**: a package raising an exception is ``crash``, with the message recorded.
  Crashed runs are left out of the results and listed under :ref:`benchmark-correctness`.
* **Validation** of every path of both packages, independent of both with Shapely, before any timing is accepted:
  the path starts at the start point, ends at the goal and no segment runs through the interior of an obstacle
  (along edges and through vertices is allowed), and the lengths of both packages agree within a relative
  tolerance of 1e-9. Packages and sizes failing a query are ``invalid``: their query times and the query ratio
  are left out. Their build was measured like any other: build time, memory and the build ratio are shown.
  Their graph size is marked as invalid, because the graph contains wrong edges (cf. :ref:`pyvisgraph-defects`).


.. _pyvisgraph-defects:

Known pyvisgraph 0.2.1 defects
------------------------------

Found by the validation while preparing the benchmark, reproducible with
``python -m benchmarks.pyvisgraph_issues`` and reported upstream
(`issue #59 <https://github.com/TaipanRex/pyvisgraph/issues/59>`__, `pull request #33 <https://github.com/TaipanRex/pyvisgraph/pull/33>`__):

.. benchmark-derived: the figures of the defects (from ``benchmarks.pyvisgraph_issues`` and the cleaned map). Update this text with every full benchmark run
   (docs/contributing/performance.md, "After a full run").
   TODO at the next full run (#234): the paragraph "Paths through obstacles" still describes the unsimplified 110m map,
   which is no longer part of the sweep. Re-derive it on a map of the sweep (e.g. world_5k): an invalid pyvisgraph
   path from the correctness results with its length and the valid length, and the number of pyvisgraph's edges
   through land. Then remove this TODO.

**Paths through obstacles.** On the cleaned 110m world map without simplification (4,954 vertices),
the query from the North Sea (3, 56) to the South China Sea (105, 5) returns a path of length 150.26
with two segments across land: Lisbon to Valencia across Iberia and Algeria to Bab-el-Mandeb across Africa.
The valid shortest path (``extremitypathfinder``) has length 211.14, around the Cape of Good Hope.
14 of the 277,886 edges of pyvisgraph's visibility graph of this map run through land.

All of them connect two vertices of the same polygon whose midpoint lies level with another vertex of that polygon.
pyvisgraph rejects such an edge by a point-in-polygon test of its midpoint
(a crossing count along a horizontal ray, ``polygon_crossing``), which mixes exact and tolerant comparisons:
edges are skipped with exact comparisons of the y-coordinates, but a vertex counts as lying on the ray
when the cross product rounds to zero (absolute tolerance 1e-10 with a ray of length 10\ :sup:`4`).
In exact arithmetic the midpoint lies level with the vertex, in floating point it misses it by a rounding error,
e.g. ``(0.1 + 0.7) / 2 == 0.39999999999999997``.
One of the two edges at that vertex is then skipped and the other one treated as touching the ray:
the crossing is lost, the midpoint is classified as outside and the edge through the obstacle is kept.
Coordinates on a decimal grid (snapped data, rounded or integer coordinates) make such coincidences frequent.
A consistent half-open crossing rule (an edge counts when ``(a.y > p.y) != (b.y > p.y)``
and the ray hits it at the exact x of the intersection) classifies the midpoints as inside and rejects all 14 edges.
Reported upstream: `pyvisgraph issue #59 <https://github.com/TaipanRex/pyvisgraph/issues/59>`__.
The smallest example is a single polygon with six vertices:

.. code-block:: python

    import pyvisgraph as vg

    polygon = [(0.0, 0.1), (2.0, -1.0), (3.0, 0.4), (2.0, 2.0), (0.0, 0.7), (-1.0, 0.4)]
    graph = vg.VisGraph()
    graph.build([[vg.Point(*p) for p in polygon]])
    graph.shortest_path(vg.Point(-0.1, -0.3), vg.Point(-0.1, 1.1))
    # [Point(-0.10, -0.30), Point(0.00, 0.10), Point(0.00, 0.70), Point(-0.10, 1.10)]
    # through the polygon from (0, 0.1) to (0, 0.7).
    # extremitypathfinder: around the vertex (-1, 0.4), length 2.28
    # with the half-open crossing rule, pyvisgraph returns the same path

**Build crash on nearly identical vertices.** Two vertices about 1e-6 apart
(in the raw Natural Earth 110m data near (-122.85, 76.12) in the Canadian Arctic) make ``angle2``
compute a cosine of -1.0000000014: the side opposite the angle is 1e-6 long and the law of cosines cancels.
The truncation ``int(cos * 1e10) / 1e10`` keeps it below -1 and ``acos`` raises ``ValueError: math domain error``
(from Python 3.14 on: ``expected a number in range from -1 up to 1, got -1.0000000014``).
Two valid, disjoint triangles of that data suffice (found by removing vertices); the build crashes within milliseconds:

.. code-block:: python

    import pyvisgraph as vg

    polygons = [
        [(137.51176, 75.94917), (138.831075, 76.13676), (141.471615, 76.09289)],
        [(-121.499995, 75.900019), (-122.854924, 76.116543), (-122.854925, 76.116543)],
    ]
    vg.VisGraph().build([[vg.Point(*p) for p in polygon] for polygon in polygons])
    # ValueError: math domain error (Python 3.14: expected a number in range from -1 up to 1, ...)

Clamping the cosine to [-1, 1], as proposed upstream in
`pyvisgraph pull request #33 <https://github.com/TaipanRex/pyvisgraph/pull/33>`__ (unmerged), makes the build succeed.
The benchmark maps avoid the crash by snapping to a grid.
