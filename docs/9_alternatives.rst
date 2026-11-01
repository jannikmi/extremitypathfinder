
.. _alternatives:

Alternatives
============

Only few packages solve the same problem: exact Euclidean shortest paths in a 2D environment
bounded by a polygon with polygonal holes, for repeated queries in a static environment.


When to use this package
------------------------

This package fits best when:

- you work in Python and want a package installable with ``pip``, with few dependencies
- your environment is given as polygons (or a grid with blocked cells) and should be used as it is:
  polygons may touch and overlap, no triangulation or navigation mesh has to be built first
- the environment is static and you query many paths in it: the visibility graph is computed once,
  each query only connects its start and goal to it
- you need exact shortest paths, identical with and without the optional Numba acceleration

Consider an alternative when:

- the environment is very large (many protruding corners): in the worst case the visibility graph grows
  quadratically with the number of extremities and its computation takes :math:`O(n^2 \log n)` time
  (cf. :ref:`engineering design <design>`). Polyanya searches a navigation mesh instead and needs no such graph.
- the environment changes often: every change requires computing the visibility graph again
- the input is a grid and approximate (grid or any-angle) paths are good enough: grid planners are simpler


Other packages
--------------

`pyvisgraph <https://github.com/TaipanRex/pyvisgraph>`__ (Python, MIT)
    The closest alternative: pure Python, installable from PyPI.
    **Unmaintained**: the last release (0.2.1) dates from 2018, the last commit from 2020.
    Builds the complete visibility graph of all obstacle vertices with Lee's :math:`O(n^2 \log n)` algorithm
    (optionally with multiple worker processes) and finds paths with Dijkstra's algorithm.
    Obstacles are polygons in the unbounded plane (no boundary polygon).
    Differences to this package:

    - this package only keeps the extremities (convex vertices) and the edges which can be part of a shortest path,
      a much smaller graph (time and memory)
    - A* search with the euclidean distance as heuristic instead of Dijkstra's algorithm
    - boundary polygon, touching and intersecting polygons and grids with blocked cells as input
    - this package does not support parallel graph construction (so far)

`Polyanya <https://docs.rs/polyanya>`__ (Rust, C++ original; no Python package)
    Exact any-angle search on a navigation mesh (Cui, Harabor and Grastien, IJCAI 2017)
    without precomputing a visibility graph. State of the art for this problem.
    Requires a triangulation (navigation mesh) of the free space as input.
    Differences to this package:

    - no Python package: using it from Python requires writing bindings
    - this package takes the polygons directly (also touching and overlapping ones), no triangulation is needed
    - no quadratic preprocessing: better suited for very large or changing environments

`VisiLibity1 <https://karlobermeyer.github.io/VisiLibity1/>`__ (C++)
    Library for 2D floating-point visibility computations and shortest paths in polygonal environments.
    Python binding: `PyVisiLibity <https://github.com/tsaoyu/PyVisiLibity>`__ (not on PyPI, has to be built).

CGAL based packages, e.g. `pyvispoly <https://pypi.org/project/pyvispoly/>`__ (GPL)
    Compute exact visibility polygons, but no shortest paths.
    Useful as reference for the correctness of the visibility computation.

Grid planners, e.g. `pythetastar <https://pypi.org/project/pythetastar/>`__ (Theta*) or A* in ``networkx``
    Only applicable to grid inputs. A* on the grid graph finds paths along the grid edges,
    any-angle planners like Theta* find shorter paths,
    but both are in general longer than the euclidean shortest paths found by this package.

A speed comparison with these alternatives does not exist yet.
