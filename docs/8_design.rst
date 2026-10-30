.. _design:

==================
Engineering Design
==================

This document describes how ``extremitypathfinder`` is built: its goals, the structure of the code,
the data model, the processing pipelines and the reasoning behind the main design decisions.
It is meant for contributors and for users who want to understand the behaviour and the limits of the package.
The underlying algorithms are explained in :ref:`the about section <algorithm>`, this document refers to them
instead of repeating them.


Goals and Scope
===============

Goals
-----

- **Exact shortest paths**: the returned path is a shortest Euclidean path in the free space of a 2D polygon environment
  (up to floating point precision), not an approximation.
- **Fast repeated queries**: the environment is static. Expensive work is done once when the environment is created,
  so that many queries (start, goal) can be answered quickly afterwards.
- **Small graphs**: only extremities become graph nodes, and the graph is reduced further without losing optimality
  (cf. :ref:`algorithm`). A small graph makes the queries fast and keeps the memory low.
- **Well defined behaviour at the edge cases**: touching obstacles, points on edges and collinear vertices
  are handled by one consistent set of rules (cf. :ref:`touching`), not by special cases spread over the code.
- **Few dependencies**: ``numpy`` and ``networkx`` only. ``numba`` is an optional accelerator, the package
  produces the same results without it.

Non-goals
---------

- **Dynamic environments**: the polygons cannot be changed after the environment is created.
  A changed environment has to be created anew.
- **Geometry processing**: polygons are used exactly as given. Simplifying, repairing, buffering or merging
  geometries is left to dedicated libraries such as Shapely.
- **Robust geometric predicates**: computations use double precision floating point numbers.
  Contacts are only detected for exactly equal coordinates or vertices exactly on an edge (cf. `Numerical Policy`_).
- **Other distance metrics, 3D, agents with a size**: the package computes Euclidean shortest paths for a point
  in the plane. Agents with a size can be supported by buffering the obstacles beforehand.
- **Parallel computation**: the visibility graph is computed in a single thread.


Architecture
============

The package is a small library with a single entry point, the class ``PolygonEnvironment``.

.. code-block:: text

    extremitypathfinder/
    ├── __init__.py              public API
    ├── extremitypathfinder.py   PolygonEnvironment, SearchResult
    ├── wedges.py                points and free wedges
    ├── utils.py                 visibility (AREA), graph, A*, validation, grids, I/O
    ├── utils_numba.py           numerical kernels (optionally compiled)
    ├── numba_replacements.py    stand-ins for numba decorators and types
    ├── plotting.py              plots of environments and searches (matplotlib)
    ├── command_line.py          console command (JSON input)
    ├── configs.py               constants
    └── types.py                 type aliases

The modules form layers, each one only imports from the layers below it:

.. code-block:: text

    command_line, plotting       user interfaces
            │
    extremitypathfinder          PolygonEnvironment, SearchResult
            │
    utils                        visibility, graph, A*, validation
            │
    wedges                       points and free wedges
            │
    utils_numba                  numerical kernels
            │
    numba | numba_replacements

``PolygonEnvironment`` orchestrates the computation and owns all state.
``utils`` and ``wedges`` contain stateless functions and the immutable ``Points`` structure,
``utils_numba`` contains the innermost numerical loops.


Data Model
==========

All geometry is stored in flat numpy arrays and referenced by integer indices.
Graph nodes, path elements and edges are such indices, coordinates are only looked up when needed.

Vertices and edges
------------------

- ``coords`` (``n x 2``, ``float64``): the vertices of all polygons, the boundary polygon first (if any),
  followed by the holes in the given order.
- ``edge_vertex_idxs`` (``n x 2``): edge ``i`` connects the vertices ``i-1`` and ``i`` of its polygon.
  Every polygon has as many edges as vertices, so edges and vertices share one index range.
- ``vertex_edge_idxs`` (``n x 2``): vertex ``i`` connects the edges ``i`` and ``i+1`` of its polygon.

The orientation convention (boundary counter clockwise, holes clockwise) puts the free space on the left side of
every edge. The free wedges and the point in polygon test rely on it.

Points and free wedges
----------------------

``Points`` (``wedges.py``) describes the free space around every distinct vertex position within the map.
Vertices with identical coordinates form one *point*, identified by the lowest vertex index.
Vertex positions outside of the map (e.g. a hole vertex inside another hole) are no points.
Each ``Point`` stores:

- the free wedges: the angle ranges of free space around the position,
  computed from the polygons passing it (*incidences*: vertices at, and edges through the position)
  by counting on which side of each incidence an angle range lies (a winding number argument, ``free_wedges()``)
- the extremity wedge: the wedge larger than 180 degree, if there is one. A point is an extremity exactly then.
- the incident edges and the bounding rays (exact directions, used for exact comparisons along rays)

The four rules of the free wedges (cf. the module docstring of ``wedges.py`` and :ref:`algorithm`)
are the single source of truth for everything that happens at vertices:
which points are graph nodes, from where a line of sight may start,
where it may pass through and where it may arrive.

Angle representations and distances
-----------------------------------

``reprs_n_distances`` maps every extremity to two arrays of length ``n``: the angle representations
and the distances of all vertices seen from that extremity. They are computed once with vectorised numpy code
and reused for the visibility computation, the graph edge weights and the A* heuristic.

The visibility graph
--------------------

``graph`` is a ``networkx.Graph`` with one node per extremity (the vertex index of its point) and
undirected edges weighted with the Euclidean distance. Extremities without any edge are kept as nodes:
start and goal might connect to them.

Query data
----------

Start and goal get the indices ``n`` and ``n + 1``. A query appends their coordinates to a *copy* of
``coords`` and computes their angle representations and distances in a query local dictionary.
It never writes to the environment. The result, ``SearchResult``, carries the path, its length and the data
needed to inspect the search (temporary edges, node indices). The search graph is only built when requested.


Processing Pipelines
====================

Construction
------------

``PolygonEnvironment(boundary, holes, validate)`` performs all preprocessing in the constructor,
so an environment object is always complete and ready for queries.

1. **Conversion**: the input is converted to ``float64`` numpy arrays (independent copies of the input).
2. **Validation** (optional, ``validate=True``): shapes, minimum vertex count, finite coordinates,
   no repeated consecutive vertices, no self intersections, non-zero area, orientation,
   no crossings or overlapping edges at self contacts
   (``check_data_requirements()``, ``check_polygon_contacts()``).
3. **Compilation** (``compile_polygon_datastructs()``): the flat coordinate array and the edge/vertex index mappings.
4. **Points** (``Points``): vertices with identical coordinates are grouped (``np.lexsort``),
   edges passing through vertex positions are found with a vectorised bounding box prefilter
   and an exact on-edge test, then the free wedges of every point are computed.
   The extremities are the points with an extremity wedge.
5. **Angle representations** of all vertices seen from every extremity (``cmp_reps_n_distance_dict()``).
6. **Visibility graph** (``compute_graph()``): for every extremity in index order

   - the extremities "in front" of it are removed as candidates and their existing edges to it are deleted
     ("Property One", cf. :ref:`algorithm`)
   - the remaining extremities with a higher index are checked for visibility with ``find_visible_from()``.
     Visibility is symmetric, so each pair is checked once.

Grids (``from_grid()``) are converted into polygons following the borders of the blocked cells first
(``convert_grid()``) and then take the same path.

Query
-----

``search(start, goal, verify=True)`` (``find_shortest_path()`` returns only path and length):

1. **Verification** (optional): start and goal must lie within the map (point in polygon tests).
2. **Trivial case**: identical start and goal.
3. **Query points**: the ``Point`` at start and goal is looked up (on a vertex position) or created
   (on an edge). A query point identical to an extremity is merged with that graph node.
4. **Goal visibility**: the graph nodes and the start visible from the goal.
   The goal is checked first because two cheap exits follow:

   - nothing visible: there is no path.
   - the start is visible: the direct connection is the shortest path, no graph search is needed.

5. **Start visibility**: the graph nodes visible from the start.
6. **Graph search**: ``astar_path()`` searches the precomputed graph extended by the temporary edges of
   start and goal, with the Euclidean distance as heuristic. The temporary edges are kept in a separate
   adjacency dictionary, the precomputed graph is neither changed nor copied.
7. **Result**: the vertex indices of the path are mapped back to coordinates, the length is summed up.

Visibility from one origin
--------------------------

``find_visible_from()`` is the one visibility routine, used for the extremities during construction and
for start and goal during queries:

1. Candidates outside of the allowed free wedges of the origin are dropped (rule 2).
2. The edges incident to the origin are excluded, all other edges are checked with
   the *Angle Range Elimination Algorithm* (``find_visible()``, cf. :ref:`algorithm`):
   edges and candidates are sorted by angle representation, every edge only checks the candidates within its
   angle range. Edges spanning the angle representation ``0.0`` are checked separately in a rotated frame,
   so the main loop never handles the wrap around.
3. The remaining candidates are filtered by ``Points.filter_visible()``: lines of sight must pass through
   points within one wedge (rule 3) and arrive at contacts within a free wedge (rule 4).

Keeping the edge checks (step 2) free of vertex rules, and the vertex rules (steps 1, 3) free of edge checks,
keeps both parts simple and separately testable.


Design Decisions
================

Precompute once, query many times
---------------------------------

The visibility between extremities only depends on the environment and is computed in the constructor.
A query only has to compute the visibility of two points. This favours use cases with a static map and many queries
(games, robotics, simulations) and is the reason why the environment is immutable.

Precomputing all shortest paths between extremities was rejected: memory grows quadratically with the number of
extremities, and a fully connected graph would slow down A* (it would visit every node).

Angle representations instead of angles
---------------------------------------

Directions are compared with a monotone representation in ``[0, 4)`` (one unit per quadrant,
``_compute_repr_n_dist()``) instead of ``atan2``. It needs one square root and one division, keeps the
ordering of the angles and is computed without trigonometric functions.
Everything that sorts or compares directions uses it: the AREA sweep, the free wedges and the "in front" test.

One concept for vertices: free wedges
-------------------------------------

Before 3.0, contacts of polygons, vertices on edges and collinear edges were handled in separate places.
Since 3.0 all of these are expressed with free wedges (``wedges.py``, #24):
an extremity, a convex vertex, a contact point and a query point on an edge are all points with wedges,
and four rules decide every vertex related question. New edge cases are handled by computing the right wedges,
not by adding exceptions to the visibility code.

Queries do not modify the environment
-------------------------------------

Queries keep all their data local: coordinates are appended to a copy, the angle representations of start and goal
are stored in a ``ChainMap`` layered over the precomputed dictionary, and temporary edges live in a separate
adjacency dictionary. The precomputed graph is never copied, which made up most of the query time on large
environments before (#136). As a consequence queries have no side effects, environments can be reused
and pickled at any time, and ``SearchResult`` is independent of later queries.

Own port of ``networkx.astar_path()``
-------------------------------------

``utils.astar_path()`` is a port of the ``networkx`` A* search that additionally follows the temporary query edges.
Adding and removing the edges on the shared graph, other graph libraries and Dijkstra searches were compared
and rejected (#152, details in the :ref:`contribution guidelines <contributing>`).
The port is under the 3-clause BSD license of ``networkx`` (``LICENSES/networkx.txt``).
``networkx`` is kept as graph data structure because it is a familiar, inspectable public attribute
(``environment.graph``).

Optional numba acceleration
---------------------------

The numerical kernels in ``utils_numba.py`` are decorated with ``numba.njit`` and explicit signatures (``cache=True``).
When numba is not installed, ``numba_replacements.py`` provides decorators and types that return the plain Python
functions, so the package works and gives the same results without numba.
Code paths that operate on whole arrays use vectorised numpy instead
(e.g. ``reps_n_distances()``, which performs the same floating point operations as its scalar counterpart).
CI runs the tests with and without numba (``tests/test_acceleration.py`` checks which backend is active).

Optional validation
-------------------

Validating the input (orientation, self intersections, crossings at self contacts) is costly and skipped by default
(``validate=False``). Callers with trusted or generated input do not pay for it, callers with external input
should enable it. Invalid input without validation leads to undefined results.

Versioned pickles
-----------------

Environments are cached with ``pickle``. Every pickle stores ``configs.PICKLE_FORMAT_VERSION``,
loading a pickle with a different version raises a ``ValueError`` with instructions instead of producing an
environment with missing or outdated attributes. Increase the version whenever the stored attributes change
incompatibly.


Numerical Policy
================

- All coordinates are ``float64``. Any finite coordinate scale is supported.
- **Contacts are exact**: polygons touch when coordinates are exactly equal or a vertex lies exactly on an edge
  (exact cross product test, or angle representations exactly 180 degree apart, ``_lies_on_edge()``).
  Near-contacts do not block. This makes the behaviour reproducible and independent of a tolerance parameter.
  For integer coordinates (e.g. grids) the tests are exact.
- **Rays are exact**: directions that are exactly the same (zero cross product) share one angle representation,
  even when the representations computed from vectors of different length differ in the last bit
  (``Point.rep()``). A tiny tolerance (``RAY_TOLERANCE``) only selects which directions are compared exactly.
- **Lines of sight touching edges**: a line of sight touching an edge only in one of its vertices, or running along it,
  is not blocked by the edge. What happens at the vertex is decided by its wedges.
- Intersections are only computed (a 2x2 linear system, ``_lies_behind_inner()``) when cheaper tests
  (angle range, distance) cannot decide.


Complexity and Memory
=====================

With ``n`` vertices and ``m`` extremities (usually ``m << n``):

=================================  ==========================  =====================================================
Step                               Time                        Notes
=================================  ==========================  =====================================================
points and wedges                  ``O(n²)``                   vectorised bounding box test: edges through vertices
angle representations              ``O(m n)``                  vectorised, stored for all extremities
visibility graph                   ``O(m n log n)``            AREA for every extremity, cf. :ref:`algorithm`
query: visibility of start, goal   ``O(n log n + m)``          dominates the query time
query: A*                          ``O(E log E)``              ``E``: graph and temporary edges
=================================  ==========================  =====================================================

Memory is dominated by the angle representations and distances (``2 m n`` floats) and the graph
(at most ``m (m - 1) / 2`` edges, in practice far fewer thanks to the graph reduction).
A query uses ``O(n + m)`` additional memory.

Profiling large environments shows that the A* search takes about 5% of the query time,
most of the time is spent on the visibility of start and goal. Optimisations should target ``find_visible()``.
Benchmarks: :ref:`speed-tests`, ``scripts/speed_benchmarks.py`` and ``prototypes/large_env_query_profile.py``.


Testing
=======

The tests (``tests/``) are organised by concern:

- ``main_test.py``, ``test_cases.py``: end-to-end shortest paths on polygon and grid environments
  with known results
- ``test_find_visible.py``, ``helper_fcts_test.py``: the visibility algorithm and the geometric helpers
- ``test_contacts.py``: touching obstacles and the free wedges (#24)
- ``test_core_regressions.py``, ``test_geometry_regressions.py``: regressions of the core algorithms
  and of documented geometric edge cases
- ``test_pickle.py``, ``cli_test.py``: persistence and the command line interface
- ``test_acceleration.py``: the active backend (with or without numba)
- ``test_release.py``, ``test_validate_release.py``: the release tooling

``tox`` runs the tests for all supported Python versions with the minimum and the latest dependencies,
each with and without numba. Release artifacts are additionally installed and smoke tested in isolation
(``make release-check``). Bug fixes should come with a regression test reproducing the bug.


Limitations and Possible Extensions
===================================

- **Dynamic environments**: changes require a new environment. Computing visibility on demand,
  or updating only the affected part of the graph, would allow changing obstacles.
- **Parallel graph construction**: the visibility of each extremity can be computed independently
  (the symmetry optimisation would have to be given up or the results merged), which would parallelise well.
- **Query speed**: the visibility of start and goal is computed from scratch for every query.
  Spatial indices over the edges could reduce the number of edges checked per query.
- **A* specialised for visibility graphs**: the triangle inequality of the visibility graph could allow
  an earlier termination than for general graphs (cf. :ref:`algorithm`).
- **Dense grids**: environments with many small obstacles have many extremities. A grid based search can be
  faster there (cf. :ref:`usage`).

Ideas and planned changes are tracked as `GitHub Issues <https://github.com/jannikmi/extremitypathfinder/issues>`__.
