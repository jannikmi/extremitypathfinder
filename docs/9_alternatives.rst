
.. _alternatives:

Alternatives
============

Only few packages solve the same problem: exact Euclidean shortest paths in a 2D environment
bounded by a polygon with polygonal holes, for repeated queries in a static environment.


What sets this package apart
----------------------------

Not the raw speed: compiled tools (cf. `Other packages`_) are faster. What no other package offers together:

- **Python first**: installable with ``pip``, only NumPy and NetworkX as dependencies,
  optional Numba acceleration with identical results. The only other package on PyPI, pyvisgraph, is unmaintained
  and returns wrong paths for some inputs (cf. `Other packages`_).
- **Real-world input as it is**: polygons may touch, overlap and intersect, a boundary polygon is optional,
  grids are converted. No triangulation, navigation mesh, merging or cleaning of the geometries is needed.
- **Well-defined, tested behaviour at the edge cases**: contacts, points on edges and collinear vertices
  follow one set of rules (cf. :ref:`touching`). The floating point decisions are audited against exact arithmetic
  (cf. :ref:`engineering design <design>`), paths are validated independently in the benchmarks.
- **Built once, queried many times**: only the protruding corners become graph nodes, and the graph is reduced
  further, so the many queries in a static environment stay fast.


When to use this package
------------------------

This package fits best when:

- you work in Python and want a package installable with ``pip``, with few dependencies
- your environment is given as polygons (or a grid with blocked cells) and should be used as it is:
  polygons may touch and overlap, and no triangulation or navigation mesh has to be built first
- the environment is static and you query many paths in it: the visibility graph is computed once,
  each query only connects its start and goal to it
- you need exact shortest paths, identical with and without the optional Numba acceleration
- you need predictable results at touching obstacles and other edge cases

Consider an alternative when:

- the environment is very large (many protruding corners): the visibility graph can have up to quadratically many
  edges in the number of extremities :math:`m`, and computing it typically takes :math:`O(m \, n \log n)` time
  for :math:`n` vertices, :math:`O(m \, n \log^2 n)` in the worst case (cf. :ref:`engineering design <design>`).
  Polyanya needs no visibility graph, but a navigation mesh of the free space, which is likely cheaper to build.
- the environment changes often: every change requires computing the visibility graph again
- the input is a grid and approximate (grid or any-angle) paths are good enough: grid planners are simpler
  and can be faster on dense grids with many small obstacles (cf. :ref:`usage <usage>`)


Other packages
--------------

`pyvisgraph <https://github.com/TaipanRex/pyvisgraph>`__ (Python, MIT)
    The closest alternative: pure Python, installable from PyPI.
    **Unmaintained**: the last release (0.2.1) dates from 2018, the last commit from 2020.
    Builds the complete visibility graph of all obstacle vertices with Lee's algorithm
    (optionally with multiple worker processes) and finds paths with Dijkstra's algorithm.
    Lee's algorithm takes :math:`O(n^2 \log n)` with a balanced tree of the edges. The implementation of pyvisgraph
    takes :math:`O(n^3)` in the worst case: it keeps the edges in a sorted list and tests every visible vertex
    of the same polygon against all edges of the polygon (cf. :ref:`engineering design <design>`).
    Obstacles are polygons in the unbounded plane (no boundary polygon).
    Differences to this package:

    .. benchmark-derived: the comparison with pyvisgraph. Update this text with every full benchmark run
       (docs/contributing/performance.md, "After a full run").

    - this package only keeps the extremities (convex vertices) and the edges which can be part of a shortest path:
      a much smaller graph and faster queries (cf. :ref:`speed-tests`)
    - A* search with the euclidean distance as heuristic instead of Dijkstra's algorithm
    - boundary polygon, touching and intersecting polygons and grids with blocked cells as input
    - this package does not support parallel graph construction (so far)

    Measured comparison on world maps of increasing size: :ref:`speed-tests`.
    pyvisgraph returns paths through obstacles for some inputs
    (`issue #59 <https://github.com/TaipanRex/pyvisgraph/issues/59>`__)
    and its build crashes on nearly identical vertices
    (`pull request #33 <https://github.com/TaipanRex/pyvisgraph/pull/33>`__, unmerged; :ref:`pyvisgraph-defects`).

`Polyanya <https://docs.rs/polyanya>`__ (Rust, C++ original; no Python package)
    Exact any-angle search on a navigation mesh (Cui, Harabor and Grastien, IJCAI 2017)
    without precomputing a visibility graph. State of the art for this problem.
    Requires a triangulation (navigation mesh) of the free space as input.
    Differences to this package:

    - using Polyanya from Python requires writing bindings
    - Polyanya needs a triangulation of the free space, this package takes the polygons directly
      (also touching and overlapping ones)
    - Polyanya needs no visibility graph, only the navigation mesh. It likely scales better to very large
      environments, and a change of the environment only requires a new mesh (not measured yet, cf. below)

`VisiLibity1 <https://karlobermeyer.github.io/VisiLibity1/>`__ (C++)
    Library for 2D floating-point visibility computations and shortest paths in polygonal environments.
    Python binding: `PyVisiLibity <https://github.com/tsaoyu/PyVisiLibity>`__ (not on PyPI, has to be built).

`TřiVis <https://github.com/janmikulacz/trivis>`__ (C++, free for research use, commercial use by arrangement)
    Library for visibility queries in polygonal environments with holes, based on the triangular expansion algorithm
    (cf. :ref:`related work <related_work>`), with floating point arithmetic and robust predicates.
    Its Pathfinder extension builds a visibility graph of the reflex vertices and computes shortest paths
    between arbitrary points or all pairs of points.
    Differences to this package:

    - no Python package or bindings (built with CMake)
    - the environment is one boundary polygon with simple holes, triangulated by the library;
      touching and overlapping polygons are not described
    - fast visibility queries according to its authors; not compared with this package yet

CGAL based packages, e.g. `pyvispoly <https://pypi.org/project/pyvispoly/>`__ (GPL)
    Compute exact visibility polygons, but no shortest paths.
    Useful as reference for the correctness of the visibility computation.

Grid planners, e.g. `pythetastar <https://pypi.org/project/pythetastar/>`__ (Theta*) or A* in ``networkx``
    Only applicable to grid inputs. A* on the grid graph finds paths along the grid edges,
    any-angle planners like Theta* find shorter paths,
    but both are in general longer than the euclidean shortest paths found by this package.

A speed comparison exists for pyvisgraph only (:ref:`speed-tests`).
