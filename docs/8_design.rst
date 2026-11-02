.. _design:

==================
Engineering Design
==================

This document describes how ``extremitypathfinder`` is built: its goals, the structure of the code,
the data model, the processing pipelines, the algorithms and the reasoning behind the main design decisions.
It is meant for contributors and for users who want to understand the behaviour and the limits of the package.
:ref:`The about section <algorithm>` gives an illustrated overview of the approach,
this document describes the algorithms as they are implemented.


Goals and Scope
===============

Goals
-----

- **Exact shortest paths**: the returned path is a shortest Euclidean path in the free space of a 2D polygon environment
  (up to floating point precision), not an approximation.
- **Fast repeated queries**: the environment is static. Expensive work is done once when the environment is created,
  so that many queries (start, goal) can be answered quickly afterwards.
- **Small graphs**: only extremities become graph nodes, and the graph is reduced further without losing optimality
  (cf. `Visibility graph reduction`_). A small graph makes the queries fast and keeps the memory low.
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
The input may use either vertex numbering: polygons with the opposite numbering are reversed when they are stored
(``orient_polygons()``).

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

The four rules of the free wedges (cf. `Free wedges`_ and the module docstring of ``wedges.py``)
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
2. **Orientation** (``orient_polygons()``): polygons with the opposite vertex numbering are reversed
   (boundary counter clockwise, holes clockwise). The test is cheap (linear in the number of vertices),
   so it always runs. Input which is no polygon (shape, fewer than 3 vertices) is left to the validation.
3. **Validation** (optional, ``validate=True``): shapes, minimum vertex count, finite coordinates,
   no repeated consecutive vertices, no self intersections, non-zero area,
   no crossings or overlapping edges at self contacts
   (``check_data_requirements()``, ``check_polygon_contacts()``).
4. **Compilation** (``compile_polygon_datastructs()``): the flat coordinate array and the edge/vertex index mappings.
5. **Points** (``Points``): vertices with identical coordinates are grouped (``np.lexsort``),
   edges passing through vertex positions are found with a vectorised bounding box prefilter
   and an exact on-edge test, then the free wedges of every point are computed.
   The extremities are the points with an extremity wedge.
6. **Angle representations** of all vertices seen from every extremity (``cmp_reps_n_distance_dict()``).
7. **Visibility graph** (``compute_graph()``): for every extremity in index order

   - the extremities "in front" of it are removed as candidates and their existing edges to it are deleted
     ("Property One", cf. `Visibility graph reduction`_)
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

All visibility computations, during construction and during queries, go through one routine
(`Visibility from one origin`_).


Algorithms
==========

Notation: ``n`` vertices (= edges), ``m`` extremities, ``k`` candidates of a visibility computation.

Why extremities suffice
-----------------------

A shortest path in a polygon environment is a polygonal chain: between two bends it runs straight
(otherwise it could be shortened), and it only bends where an obstacle forces it to, at a vertex around which
the path "wraps". The path can only wrap around a vertex whose free space is larger than 180 degree on the
side the path bends towards, i.e. at an **extremity**. At a vertex with a free angle of at most 180 degree
the path could be shortened by cutting the corner.
Hence the shortest path consists of start, a sequence of extremities and goal, consecutive elements being
mutually visible. Searching the visibility graph of the extremities (extended by start and goal) is exact.

Angle representation
--------------------

``_compute_repr_n_dist()`` maps a vector :math:`(dx, dy)` with length :math:`d` to a value in :math:`[0, 4)`:

====================================  ===========  ================================
quadrant                              offset       added value
====================================  ===========  ================================
:math:`dx \geq 0, dy \geq 0`          0            :math:`dy / d`
:math:`dx < 0, dy \geq 0`             1            :math:`-dx / d`
:math:`dx < 0, dy < 0`                2            :math:`-dy / d`
:math:`dx \geq 0, dy < 0`             3            :math:`dx / d`
====================================  ===========  ================================

Within each quadrant the added value is the sine of the angle measured from the start of the quadrant,
which grows monotonically from 0 to 1. The representation is therefore strictly monotonic in the angle
(counter clockwise from the direction ``(1, 0)``): comparing representations is comparing angles,
and a difference of exactly ``2.0`` means opposite directions. Angle ranges are handled modulo ``4.0``.
The zero vector has no direction (``NaN``), callers exclude it.
``reps_n_distances()`` is the vectorised variant for all vertices at once, with the same floating point operations.

Free wedges
-----------

``free_wedges()`` computes the free space around a position from its *incidences*:
every polygon vertex at the position contributes the rays to its previous and next vertex,
every edge passing through the position contributes the rays to the two edge vertices.
Because of the orientation convention the free side of an incidence is the angle range to the left of the polygon,
counter clockwise from its second ray to its first ray.

1. The distinct ray directions split the full circle into elementary angle ranges (*arcs*).
   Rays with exactly the same direction get one representation (``_add_ray()``, exact cross product test),
   so they do not create spurious tiny arcs.
2. For every arc and every polygon, count the incidences of that polygon having the arc (its middle direction)
   on their free side. Around a position, this count equals the winding number of the polygon up to a constant.
3. An arc is blocked by a hole when its count is lower than the maximum count of that hole
   (inside the hole), and blocked by the boundary polygon when its count equals the minimum
   (outside of the boundary). A polygon whose count is the same for all arcs only touches the position with
   zero-width spikes, which block their rays only.
4. The free arcs are the free wedges. Two neighbouring free arcs remain separate wedges:
   the ray between them is an obstacle (an edge running along it).

Because the arcs are classified per polygon and an arc is free only when no polygon blocks it,
overlapping and touching polygons act as their union.
A point is an **extremity** if one wedge is larger than 180 degree (representation difference ``> 2.0``,
there can be at most one). This is its *extremity wedge*.

Example: two square holes touching diagonally at one corner contribute the rays 0°, 90° (first hole)
and 180°, 270° (second hole). The arcs 0°–90° and 180°–270° are blocked, the free wedges are
90°–180° and 270°–360°. No wedge is larger than 180 degree, so the corner is no extremity,
and a line of sight from 135° to 315° through the corner enters and leaves in different wedges: it is blocked.

The rules applied with the wedges (``wedges.py``):

1. a point is an extremity when it has a free wedge larger than 180 degree
2. a line of sight leaves a point within a free wedge
   (an extremity within its extremity wedge, a query point within any of its wedges)
3. a line of sight passes through a point only when its incoming and outgoing directions lie in the same wedge
   (``Point.passable()``)
4. a line of sight arrives at a contact within a free wedge (at a single polygon vertex this always holds
   for a line of sight that did not cross the polygon, so it is only checked at contacts)

The bounding rays belong to the wedges (closed angle ranges, ``in_closure()``): paths may run along edges.
Directions within ``RAY_TOLERANCE`` of a bounding ray are compared with the exact ray directions instead of their
representations (``Point.rep()``), so that collinear edges of different length behave consistently.

Visibility from one origin
--------------------------

``find_visible_from()`` decides which candidates are visible from an origin. It is used for every extremity
during construction and for start and goal during queries:

1. **Leaving the origin (rule 2)**: candidates outside of the allowed wedges of the origin are dropped.
   The edges incident to the origin are then excluded from the edge checks: their effect is fully described
   by the wedges.
2. **Edges (AREA)**: the remaining edges are checked with ``find_visible()`` (below).
3. **Passing and arriving (rules 3 and 4)**: ``Points.filter_visible()`` removes candidates whose
   line of sight passes through a point outside of one wedge or arrives at a contact outside of its wedges.

Keeping the edge checks free of vertex rules, and the vertex rules free of edge checks,
keeps both parts simple and separately testable.

Angle Range Elimination Algorithm (AREA)
----------------------------------------

AREA is the visibility algorithm developed for this package.
The idea: a candidate is visible when no edge runs in front of it, "blocking the view".
``find_visible()`` removes every candidate that lies behind an edge, seen from the origin,
the candidates remaining at the end are visible.
A line of sight only touching an edge in one of its vertices, or running along it, is not blocked by that edge.

.. code-block:: text

    keep only the closest candidate per angle representation
    sort the candidates by angle representation
    per edge: range [min_rep, max_rep] and max_dist of its two vertices
    crossing edges: max_rep - min_rep > 2 (the range contains 0)

    check(regular edges)
    shift all representations: no crossing edge contains 0 anymore
    check(crossing edges)

    check(edges):
      sort the edges by min_rep
      ptr = 0                   # first candidate possibly in a range
      for edge in edges:
        stop if no candidates are left
          or the last candidate lies before min_rep
        move ptr past the candidates with rep < min_rep
        for candidate from ptr while rep <= max_rep:
          rep == min_rep or max_rep  -> keep   (touches a vertex)
          candidate lies on edge     -> keep   (query point)
          distance > max_dist        -> remove (behind both vertices)
          lies_behind(edge, cand.)   -> remove (2x2 linear system)
          otherwise                  -> keep

Details:

- **Identical directions**: of candidates with exactly the same representation only the closest is kept.
  A path to a farther one passes through the closer one at no extra cost (and the vertex rules decide
  whether it may pass at all).
- **Sweep**: since the edges are sorted by ``min_rep`` and the candidates by representation,
  candidates before the current edge's range can never be blocked by a following edge.
  The pointer only moves forward, removed candidates shrink the list.
- **Crossing edges**: an edge spans less than 180 degree, so a representation difference larger than ``2.0``
  means the range wraps around ``0``. Instead of handling the wrap in the sweep, all representations are shifted
  (modulo ``4.0``) so that the smallest upper end of the crossing edges moves to ``0``, which turns the crossing edges
  into regular ones. The same sweep is reused.
- **Distance shortcut**: a segment lies within the disk around the origin with the radius of its farther vertex.
  A candidate within the angle range but farther away than both edge vertices therefore lies behind the edge.
  Only the remaining candidates need an intersection computation.
- **Lies behind**: with all vectors relative to the origin, ``_lies_behind_inner()`` solves
  :math:`p_1 + \lambda (p_2 - p_1) = \mu v` for the intersection of the line of sight :math:`v` with the edge.
  The candidate is blocked when :math:`\mu < 1` (the edge is closer than the candidate).
  Within the angle range the system is regular.

Properties:

- every edge is checked at most once
- only a subset of the vertices has to be checked (the candidates, e.g. the extremities, not all vertices)
- the number of candidates decreases with every origin: visibility is symmetric, each pair is checked once
- no trigonometric functions
- intersections (solving a linear system) are only computed for a fraction of the candidates
- it would also work with line segments instead of polygons (the package however only accepts polygons)

Cost per origin: sorting the edges, :math:`O(n \log n)`, and the candidates, :math:`O(k \log k)`.
Every edge is visited at most once, and on average only the fraction of candidates within an edge's angle range
is checked against it, :math:`O(n \cdot k / n) = O(k)`.
With :math:`k \leq m` this gives :math:`O(n \log n + m)` per origin and :math:`O(m \, n \log n)` for all
extremities, i.e. :math:`O(n^2 \log n)` in the worst case (:math:`m \sim n`).

Comparison with `Lee's visibility graph algorithm <https://dav.ee/papers/Visibility_Graph_Algorithm.pdf>`__
(also :math:`O(n^2 \log n)`), which `pyvisgraph <https://github.com/TaipanRex/pyvisgraph>`__ uses:

- Lee checks all edges for intersection initially, AREA only the edges whose angle range contains candidates
- Lee computes the visibility of all vertices in every run, AREA only of the candidates
- Lee needs an intersection computation for most vertices (whenever its edge tree is not empty),
  AREA avoids most of them with the angle range and distance tests
- Lee sorts all vertices by angle on startup and keeps the edges in a binary tree,
  AREA sorts edges and candidates
- both work with line segments, not only polygons

Passing through and arriving at points
--------------------------------------

``Points.filter_visible()`` applies rules 3 and 4 to the candidates left by AREA:

- **Arriving (rule 4)**: at a contact, the direction back to the origin must lie in the extremity wedge
  (for a query point candidate: in any of its wedges).
- **Passing (rule 3)**: the points are sorted by their representation seen from the origin. For each candidate,
  a binary search (``np.searchsorted``) finds the points with nearly the same representation
  (within ``RAY_TOLERANCE``). Those closer than the candidate and exactly on the line of sight
  (identical representation or zero cross product) must be passable: the directions back to the origin and on to
  the candidate must lie in the same wedge.

Only the few points on a line of sight are checked, so this filter costs :math:`O(p \log p)` for sorting the
``p`` points plus a binary search per candidate.

Visibility graph reduction
--------------------------

``compute_graph()`` builds the graph with the reduction described in
`[1, Ch. II 4.4.2 "Property One"] <http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__:
an extremity :math:`e_1` lying *in front of* an extremity :math:`e` is never the next node after :math:`e`
on a shortest path (and vice versa), so they are not connected.

*In front* (``Point.in_front()``): the obstacles at :math:`e` occupy the angle range outside of its extremity wedge.
The front is the opposite range (rotated by 180 degree), excluding its bounding directions.
A path arriving at :math:`e` from the front cannot wrap around :math:`e`, so it would go on more directly
without visiting :math:`e`.

.. code-block:: text

    add all extremities as nodes
    for each extremity e in index order:
      later = the extremities with a higher index
      front = { x in later ∪ neighbours(e) : x lies in front of e }
      remove the edges between e and front  # added from the other side
      visible = find_visible_from(e, later - front, [extremity wedge])
      add the edges e - visible, weighted with their distance

Visibility is symmetric, so each pair is checked once (from the extremity with the lower index).
The front condition is not symmetric: an edge added from :math:`e_1`'s side is removed again when :math:`e_1`
lies in front of :math:`e`. That is why the current neighbours are included in the front test.
Extremities without edges remain in the graph, start and goal may connect to them.

The reduction does not apply to start and goal: they are endpoints of the path, not intermediate nodes.
A query point connects to all visible graph nodes, including those in front of it.

Query
-----

``search()`` computes the visibility of start and goal with the same routine and then searches the graph:

- **Merging**: a query point at the position of an extremity is mapped to that graph node.
  When the start is merged, its graph node is excluded from the candidates visible from the goal:
  it has the same direction and distance as the start and would hide it (only one candidate per direction is kept),
  although the start may leave through any of its wedges and the graph node only through its extremity wedge.
- **Goal first**: when nothing is visible from the goal there is no path. When the start is visible from the goal,
  the straight segment is the shortest path (no path can be shorter than the Euclidean distance) and the graph is
  not searched.
- **A***: ``astar_path()`` uses the Euclidean distance to the goal as heuristic. Edge weights are Euclidean distances,
  so the heuristic is admissible and consistent (triangle inequality): the first time the goal is taken from the
  priority queue, its path is a shortest one.
  The implementation follows ``networkx.astar_path()``: a binary heap with lazy deletion
  (outdated queue entries are skipped when popped), a counter as tie breaker (nodes are never compared),
  and caching of the heuristic per enqueued node. The neighbours of a node are those in the precomputed graph
  followed by the temporary edges of start and goal.
- **Distances**: ``get_distance()`` looks up the precomputed distances of the node with the higher index.
  Start and goal have the highest indices, their distances are computed for the query.

Point in polygon
----------------

``is_within_map()`` accepts a point when it lies within the boundary polygon (its border included)
and not strictly inside any hole (hole borders are part of the map).
``_inside_polygon()`` first checks for every edge whether the point lies exactly on it (``_lies_on_edge()``)
and then returns the given border value. Otherwise it counts the crossings of a horizontal ray from the point
in positive ``x`` direction with the polygon edges (crossing number). Edges with exactly one vertex to the right
are decided by comparing slopes as products (no division).

Contact detection
-----------------

Contacts are found while building the points (``Points``):

- **Identical vertices**: the coordinates are sorted lexicographically (``np.lexsort``)
  and consecutive equal rows are grouped. Unlike ``np.unique`` this treats ``0.0`` and ``-0.0`` as equal.
- **Vertices on edges**: for all distinct vertex positions, the edges whose bounding box contains the position are
  selected with numpy broadcasting (processed in chunks to bound the memory), the edge's own vertices are excluded,
  and ``_lies_on_edge()`` decides exactly: a zero cross product with the position between the edge vertices,
  or angle representations of the edge vertices exactly 180 degree apart.

The bounding box test compares all positions with all edges, :math:`O(n^2)` vectorised operations.

Input validation
----------------

With ``validate=True`` every polygon is checked (``_check_polygon()``):

- shape, at least 3 vertices, finite coordinates, no identical consecutive vertices
- **proper self intersections**: every pair of non-neighbouring edges is tested by solving a 2x2 linear system
  (``_get_intersection_status()``), intersections strictly inside both edges are rejected, :math:`O(n^2)` per polygon
- **non-zero area**: shoelace formula on coordinates translated to the first vertex (avoids cancellation)
- **crossings at self contacts** (``check_polygon_contacts()``): at every position where the polygon touches itself,
  two rays with the same direction mean overlapping edges (zero-width spikes or slits), and two corners
  whose rays interleave (exactly one ray of the second corner lies within the first corner) mean the polygon
  crosses itself

The orientation is not validated: ``orient_polygon()`` reverses polygons with the opposite numbering,
using the sign of :math:`\sum (x_2 - x_1)(y_2 + y_1)` over all edges (``_has_clockwise_numbering()``, positive: clockwise).

Grid conversion
---------------

``convert_grid()`` turns blocked cells into polygons whose vertices are cell corners
(cell ``(x, y)`` spans :math:`[x, x+1] \times [y, y+1]`):

1. **Boundary**: starting at the lowest, leftmost free cell, a wall follower walks along the border between free cells
   and blocked cells (or the grid border), keeping the blocked side on its left, and adds a vertex at every turn
   until it is back at the start. The vertex order is reversed to make it counter clockwise.
2. **Holes**: the blocked cells inside the boundary (tested at the cell centre) are processed repeatedly:
   the outline of the lowest, leftmost remaining blocked cell is traced in the same way (clockwise),
   and all blocked cells inside this new hole are removed from the remaining cells.

The polygons follow the cell borders exactly (no simplification, no diagonal edges).
Diagonally adjacent blocked cells produce polygons (or a polygon) touching at a corner, which block paths through it
(no corner cutting).


.. _design_decisions:

Design Decisions
================

Decisions made from #179 on are also listed, with the rejected options and who made them,
in the :doc:`decision log <contributing/decisions>`.

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

Validating the input (self intersections, crossings at self contacts) is costly and skipped by default
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
visibility graph                   ``O(m n log n)``            AREA for every extremity, cf. `Algorithms`_
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
- **A* specialised for visibility graphs**: in the visibility graph it is always shortest to reach a node directly
  instead of visiting other nodes first (triangle inequality). A custom A* could exploit this to terminate earlier
  than for general graphs: nodes never have to be revisited (the path only gets longer),
  and not all neighbours of the current node have to be checked before continuing with the next node.
- **Dense grids**: environments with many small obstacles have many extremities. A grid based search can be
  faster there (cf. :ref:`usage`).

Ideas and planned changes are tracked as `GitHub Issues <https://github.com/jannikmi/extremitypathfinder/issues>`__.
