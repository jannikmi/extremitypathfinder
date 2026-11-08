
.. _speed-tests:

Performance
===========

.. benchmark-derived: what Numba costs. Update this text with every full benchmark run
   (docs/contributing/performance.md, "After a full run").

This page compares ``extremitypathfinder`` with `pyvisgraph <https://github.com/TaipanRex/pyvisgraph>`__,
the closest alternative (see :ref:`alternatives`), on realistic maps of increasing size.
``extremitypathfinder`` is compared with Numba, its best case, and is also measured without Numba:
Numba is optional, and it costs memory.

The results come from one full run on the machine stated in :ref:`benchmark-results`.
The raw results of that run are in ``benchmarks/results/benchmark.json``.
How everything is measured, and what limits the comparison, is described in :ref:`benchmark-method`.
All of it is reproducible with one command (see :ref:`running-benchmarks`).


Strengths, weaknesses and limitations
-------------------------------------

Written from the measured data of this run.

.. benchmark-derived: the take-aways of the results. Update this text with every full benchmark run
   (docs/contributing/performance.md, "After a full run").

Strengths of ``extremitypathfinder``:

- **Faster preprocessing and queries.** It builds the environment several times faster on all maps both packages
  built within the budget. On the maps where both packages return valid paths, it also answers queries several times
  faster (median). Both ratios grow with the size of the map.
- **Correct paths.** All its paths are valid and agree in length with the valid paths of pyvisgraph.
  On larger maps pyvisgraph returned paths through land (cf. :ref:`benchmark-correctness`).
- **The graph reduction pays off at query time.** Its visibility graph has several times fewer edges than
  pyvisgraph's full visibility graph, and the difference grows with the size of the map.
  The queries search this smaller graph.
- **Larger maps within the budget.** It built maps for which pyvisgraph exceeded the build budget.
- **Less retained memory.** On every map its prepared environment keeps less memory than pyvisgraph's,
  and the difference grows with the size of the map.
- **Fast without Numba too.** Without Numba, builds take more than twice as long and queries moderately longer.
  It is still faster than pyvisgraph, and it built the same maps within the budget.
  The baseline drops by more than half, the retained memory stays the same.
  When to install without Numba: see :ref:`with-or-without-numba`.

Weaknesses and limitations:

- **The preprocessing limits the map size.** The build time grows steeply with the map,
  and the sweep ends where it exceeds the build budget (cf. the build table).
  Going from the raw 110m data to the finer source data adds many polygons, and the visibility graph
  grows much faster than the number of vertices there.
- **Memory on small maps.** Its peak above the baseline is close to pyvisgraph's on small maps, despite the lower
  retained memory: the build needs temporary data. On large maps the peak is much lower than pyvisgraph's.
  Its baseline is much larger, so its process needs several times as much memory in total on the smallest maps.
  The gap shrinks with the size of the map.
  Most of the difference in the baseline is Numba (which is optional, cf. the baselines without input data).
- **Query time grows with the map.** Connecting start and goal to the graph checks the visibility of all vertices
  (cf. :ref:`design`).
- **Scope of the comparison.** One machine, single-threaded (pyvisgraph can build its graph with several processes,
  which was not measured). Longitude and latitude are used as
  planar coordinates, and the query points keep a distance from the coasts.


.. _benchmark-results:

Results
-------

.. include:: ../benchmarks/results/benchmark_system.rst

.. include:: ../benchmarks/results/benchmark_limits.rst

Speed
~~~~~

.. figure:: _static/benchmark_build_time.svg
   :width: 100%
   :alt: build time over the number of vertices, log-log, one line per package

   Build (preprocessing) time. The dashed line is the budget.

.. include:: ../benchmarks/results/benchmark_build.rst

.. figure:: _static/benchmark_query_time.svg
   :width: 100%
   :alt: query latency over the number of vertices, log-log, median and 5th to 95th percentile band

   Query latency: median and 5th to 95th percentile band.

.. include:: ../benchmarks/results/benchmark_query.rst

.. figure:: _static/benchmark_ratio.svg
   :width: 100%
   :alt: ratio of pyvisgraph to extremitypathfinder time for build and query

   Speed ratio pyvisgraph / extremitypathfinder: build time for the sizes where both completed the build,
   query time for the sizes where both are valid.

.. figure:: _static/benchmark_numba_ratio.svg
   :width: 100%
   :alt: ratio of the time of extremitypathfinder without Numba to with Numba for build and query

   Speed-up of ``extremitypathfinder`` by Numba: time without Numba / with Numba.

Memory
~~~~~~

.. figure:: _static/benchmark_memory.svg
   :width: 100%
   :alt: peak memory over the number of vertices

   Peak memory of the build and the queries: above the baseline (solid) and the total including the imports
   and the warm-up (dashed).

.. figure:: _static/benchmark_memory_retained.svg
   :width: 100%
   :alt: peak memory and retained memory above the baseline over the number of vertices

   Above the baseline: the peak (solid) and what is retained after the build (dashed).
   The difference is temporary data of the build and the queries, and memory the allocator keeps.

.. include:: ../benchmarks/results/benchmark_memory.rst

.. figure:: _static/benchmark_memory_composition.svg
   :width: 100%
   :alt: stacked bars of the peak memory per map size: interpreter and imports, Numba, retained and temporary data

   What the peak memory consists of (the totals of the table).
   Numba is the difference of ``extremitypathfinder``'s baselines with and without Numba (without input data),
   the rest of the baseline is the interpreter, the imports and the input.
   The data parts are measured with Numba: a build without Numba may peak differently.


.. _benchmark-correctness:

Correctness
-----------

Invalid, crashed and over budget runs are listed here. The query times of invalid runs are left out of the results,
their build results are shown (graph size marked as invalid):

.. include:: ../benchmarks/results/benchmark_correctness.rst

.. figure:: _static/benchmark_world_example.svg
   :width: 100%
   :alt: world map with the paths of both packages for one query

   The paths of both packages for one query on the smallest map with an invalid path (if any).

The causes of the invalid paths and of the build crash of pyvisgraph are described in :ref:`pyvisgraph-defects`.
