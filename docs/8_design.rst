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
  (cf. `Visibility graph reduction`_). A small graph makes the queries fast.
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
  in the plane. Agents with a size are supported by inflating the obstacles beforehand
  (cf. :ref:`agent clearance <clearance>`).
- **Parallel computation**: the visibility graph is computed in a single thread.


Architecture
============

The package is a small library with a single entry point, the class ``PolygonEnvironment``.

.. code-block:: text

    extremitypathfinder/
    ├── __init__.py              public API
    ├── extremitypathfinder.py   PolygonEnvironment: the public API, delegating to the modules below
    ├── queries.py               queries: start and goal visibility, graph search, distance matrix
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
    extremitypathfinder          PolygonEnvironment
            │
    queries                      search, distance matrix, SearchResult
            │
    utils                        visibility, graph, A*, validation
            │
    wedges                       points and free wedges
            │
    utils_numba                  numerical kernels
            │
    numba | numba_replacements

``PolygonEnvironment`` owns all state and is the public API: it checks the input and delegates the algorithms
to the modules below (``utils.compute_graph()`` for the preparation, ``queries`` for the queries).
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

``cmp_reps_n_distances()`` computes two arrays of length ``n`` with vectorised numpy code: the angle representations
and the distances of all vertices seen from one origin. They are computed for every extremity while the visibility
graph is built (for the visibility computation and the edge weights) and for start and goal in every query.
They are not stored: for all extremities this would take ``2 m n`` floats, about 1 GiB for 10,000 vertices.

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
3. **Validation** (on by default, skipped with ``validate=False``): shapes, minimum vertex count, finite coordinates,
   no repeated consecutive vertices, no self intersections, non-zero area,
   no crossings or overlapping edges at self contacts
   (``check_data_requirements()``, ``check_polygon_contacts()``).
4. **Compilation** (``compile_polygon_datastructs()``): the flat coordinate array and the edge/vertex index mappings.
5. **Points** (``Points``): vertices with identical coordinates are grouped (``np.lexsort``),
   edges passing through vertex positions are found with a vectorised bounding box prefilter
   and an exact on-edge test, then the free wedges of every point are computed.
   The extremities are the points with an extremity wedge.
6. **Visibility graph** (``compute_graph()``): for every extremity in index order

   - the angle representations and distances of all vertices seen from it are computed
     (cf. `Angle representations and distances`_)
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

``_compute_repr_n_dist()`` maps a vector :math:`(dx, dy)` to a value in :math:`[0, 4)`, with :math:`s = |dx| + |dy|`:

====================================  ===========  ================================
quadrant                              offset       added value
====================================  ===========  ================================
:math:`dx \geq 0, dy \geq 0`          0            :math:`dy / s`
:math:`dx < 0, dy \geq 0`             1            :math:`-dx / s`
:math:`dx < 0, dy < 0`                2            :math:`-dy / s`
:math:`dx \geq 0, dy < 0`             3            :math:`dx / s`
====================================  ===========  ================================

Within each quadrant the added value (the "diamond angle") grows monotonically from 0 to 1
with the angle measured from the start of the quadrant. The representation is therefore strictly monotonic in the angle
(counter clockwise from the direction ``(1, 0)``): comparing representations is comparing angles,
and a difference of exactly ``2.0`` means opposite directions. Angle ranges are handled modulo ``4.0``.
The zero vector has no direction (``NaN``), callers exclude it.
The slope of the diamond angle is at least :math:`1/2` per radian everywhere,
so directions close to the axes are resolved as finely as all others. The sine of the angle (division by the length),
used before, is flat at the axes: directions up to about 1e-8 rad apart got the same representation there,
which let lines of sight cut obstacle corners and hid visible vertices on float32 maps (#3).
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
Directions exactly on a bounding ray (zero cross product, same direction) take the representation of that ray
(``Point.rep()``), so that collinear edges of different length behave consistently. The check only runs for directions
within ``RAY_WINDOW`` of a bound (cf. `Numerical Policy`_).

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
      keep only the edges with a candidate in [min_rep, max_rep]
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
- **Edges without candidates**: an edge whose angle range contains no candidate cannot block one,
  candidates are only removed during the sweep. Two vectorised binary searches (``np.searchsorted()``)
  select the other edges before sorting (``_edges_with_candidates()``), so the Python loop only runs over
  the few edges that can block a candidate, not over all edges of every origin.
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

Cost per origin with :math:`k \leq m` candidates: selecting the edges with candidates, :math:`O(n \log k)`
(vectorised), sorting them, :math:`O(n \log n)`, and the candidates, :math:`O(k \log k)`,
plus one check per edge and candidate within its angle range.

- **Typical case**: the angle ranges of the edges overlap little, so every candidate is only checked against
  a few edges. This gives :math:`O(n \log n + m)` per origin and :math:`O(m \, n \log n)`
  for all extremities.
- **Worst case**: a candidate in front of an edge is kept and checked again by every following edge
  whose angle range contains it, e.g. many long edges behind many candidates, all spanning the same angle range.
  This gives :math:`O(n \log n + n k)` per origin and :math:`O(m \, n \log n + n \, m^2)` for all extremities
  (the number of candidates decreases from :math:`m` with every origin), i.e. :math:`O(n^3)` for :math:`m \sim n`.
  In such an environment, doubling both the number of edges and of candidates quadruples
  the time of ``find_visible()``.
  An angular sweep would avoid this worst case, but is slower on the measured maps
  (`#216 <https://github.com/jannikmi/extremitypathfinder/issues/216>`__).

AREA builds on known techniques: sorting by angle around the origin (as Lee's algorithm),
a trigonometry-free angle representation (the "diamond angle") and culling by angle range and distance.
Its own part is the edge-centric sweep: every edge eliminates the candidates behind it,
instead of a rotating ray keeping the edges in a balanced tree.

Comparison with `Lee's visibility graph algorithm <https://dav.ee/papers/Visibility_Graph_Algorithm.pdf>`__,
which `pyvisgraph <https://github.com/TaipanRex/pyvisgraph>`__ uses.
Lee's algorithm takes :math:`O(n^2 \log n)` in the worst case when it keeps the edges crossing the sweep ray
in a balanced search tree. The implementation of pyvisgraph (0.2.1, ``visible_vertices.py``) takes :math:`O(n^3)`
in the worst case, like AREA for :math:`m \sim n`, for two reasons: it keeps the edges in a sorted Python list
(``insort()`` and ``del``, :math:`O(n)` per insertion and removal), and for every visible vertex of the same polygon
it tests whether the line of sight runs through the polygon with a crossing test over all edges of that polygon
(``edge_in_polygon()``), :math:`O(n)` per pair.
The list operations are fast memory moves, and the measured maps keep AREA in its typical case (#215).
The differences:

- Lee checks all edges for intersection initially, AREA only the edges whose angle range contains candidates
- Lee computes the visibility of all vertices in every run, AREA only of the candidates
- Lee needs an intersection computation for most vertices (whenever its edge tree is not empty),
  AREA avoids most of them with the angle range and distance tests
- Lee sorts all vertices by angle on startup and keeps the edges in a binary tree (pyvisgraph: a sorted list),
  AREA sorts edges and candidates
- both work with line segments, not only polygons

Passing through and arriving at points
--------------------------------------

``Points.filter_visible()`` applies rules 3 and 4 to the candidates left by AREA:

- **Arriving (rule 4)**: at a contact, the direction back to the origin must lie in the extremity wedge
  (for a query point candidate: in any of its wedges).
- **Passing (rule 3)**: the points are sorted by their representation seen from the origin. For each candidate,
  a binary search (``np.searchsorted``) finds the points with nearly the same representation
  (within ``RAY_WINDOW``). Those closer than the candidate and exactly on the line of sight
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

The same argument holds for the edges of start and goal: it is about the extremity :math:`e` being an
intermediate node, whether the path comes from a query point or from another extremity
(the "taut path" pruning of `Oh and Leong (2017) <https://arxiv.org/abs/1702.01524>`__ is the same).
A query point is therefore not connected to the graph nodes it lies in front of (#222).
The reverse does not hold: start and goal are endpoints of the path, so a query point connects to the visible graph
nodes lying in front of *it*, also when it is merged with an extremity.

Query
-----

``search()`` (``queries.search()``) computes the visibility of start and goal with the same routine and then searches the graph:

- **Merging**: a query point at the position of an extremity is mapped to that graph node.
  When the start is merged, its graph node is excluded from the candidates visible from the goal:
  it has the same direction and distance as the start and would hide it (only one candidate per direction is kept),
  although the start may leave through any of its wedges and the graph node only through its extremity wedge.
- **Skipping the nodes in front**: before the visibility of a query point is computed, the graph nodes it lies
  in front of are removed from the candidates (cf. `Visibility graph reduction`_). The visibility of start and goal
  takes most of the query time: fewer candidates make it faster, and A* gets fewer temporary edges.
  The test must be cheaper than what it saves, so it is vectorised (``Points.in_front_of()``):
  the front range of every extremity is precomputed, and the direction from an extremity to the query point is the
  opposite of the one seen from the query point (its angle representation plus 2). Directions within
  ``RAY_WINDOW`` of a bound of the range are decided by ``Point.in_front()``, like in the graph
  (it excludes the directions exactly opposite of a bounding ray). The graph node merged with the query point is not
  skipped (it is the query point). The one merged with the other query point need not be kept: the edge to it is the
  straight segment between start and goal, found by the "goal first" check. So the skipped nodes depend only on the
  query point, and ``distance_matrix()`` skips the same ones.
  AREA keeps only the closest candidate per direction, so skipping a node can make a farther candidate in the same
  direction the closest one. Whether its line of sight passes through the skipped node is decided by the points
  (rule 3, ``Points.filter_visible()``), not by the candidates.
- **Goal first**: when nothing is visible from the goal there is no path. When the start is visible from the goal,
  the straight segment is the shortest path (no path can be shorter than the Euclidean distance) and the graph is
  not searched.
- **A***: ``astar_path()`` uses the Euclidean distance to the goal as heuristic. Edge weights are Euclidean distances,
  so the heuristic is admissible and consistent (triangle inequality): the first time the goal is taken from the
  priority queue, its path is a shortest one.
  The search is an own implementation (#157): a binary heap with lazy deletion
  (a node whose distance improves is inserted again, outdated queue entries are skipped when popped)
  and an insertion counter as tie breaker (nodes are never compared).
  A node is expanded again if a shorter path to it is found after its expansion. With the consistent heuristic
  this does not happen, but the search also stays exact with a heuristic that is only admissible
  (e.g. for custom costs, #162) and when rounding violates the triangle inequality.
  The neighbours of a node are those in the precomputed graph followed by the temporary edges of start and goal.
- **Distances**: the heuristic uses the distances seen from the goal (computed for the query; a graph node merged
  with the goal has the same coordinates). The length of the path is the sum of its edge weights (``edge_weight()``).

Distance matrix
---------------

``distance_matrix()`` (``queries.distance_matrix()``) gives the same lengths as ``search()`` for every pair of points,
bit for bit (#175),
but computes the visibility of every point only once instead of twice per pair:

- The query points are appended to the coordinates (identical positions once) and merged with graph nodes
  as in ``search()``.
- From every point the visible graph nodes are computed, from every goal point additionally the visible start points.
  The graph nodes a point lies in front of are skipped as in ``search()``.
  This reproduces the visibility of ``search()`` with that point as goal: a start point is hidden by a closer graph
  node in the same direction (the path then passes through that node and has the same sum of edge weights),
  but not by the graph node at its own position or by other query points. Query points sharing a direction with
  another candidate are therefore checked in separate calls of ``find_visible_from()``,
  so that no query point hides a graph node or another query point.
- A start visible from a goal gives the straight segment. For the other pairs, the graph is searched with A*
  exactly as in ``search()`` (``queries.search_graph()``, shared by both): the temporary edges of start and goal
  and the distances seen from the goal as heuristic. The heuristic is computed once per goal.
  Paths and lengths are therefore those of ``search()`` by construction.
- Neither the environment nor its graph is changed: the temporary edges are passed to the search.

The visibility takes most of the query time (cf. :doc:`7_performance`), the A* searches only a small part.
Computing the visibility once per point gives most of the speed-up, so the graph search was not replaced:
one Dijkstra search per start point instead of A* per pair gave identical lengths but was not consistently faster
(cf. the :doc:`decision log <contributing/decisions>`).

Point in polygon
----------------

``is_within_map()`` accepts a point when it lies within the boundary polygon (its border included)
and not strictly inside any hole (hole borders are part of the map).
Only the holes whose closed bounding box contains the point are tested (``polygon_bounds()``):
a point outside of it lies neither inside the hole nor on its border.
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

By default (``validate=True``) every polygon is checked (``_check_polygon()``):

- shape, at least 3 vertices, finite coordinates, no identical consecutive vertices
- **proper self intersections** (``_find_crossing()``): only pairs of non-neighbouring edges whose bounding boxes
  overlap are tested. A sweep along x over the edges sorted by their smallest x coordinate selects the pairs
  overlapping in x, a vectorised comparison those overlapping in y (in chunks, to bound the memory).
  For these pairs the 2x2 linear system is solved with Cramer's rule and compared without division
  (``_crossing_parameters()``, exact for integer coordinates), intersections strictly inside both edges are rejected.
  :math:`O(n \log n + k)` vectorised operations for :math:`k` pairs of edges overlapping in x,
  :math:`O(n^2)` in the worst case (e.g. a zigzag of edges all overlapping in x). Coordinate differences
  beyond about :math:`10^{154}` overflow in the cross products and go undetected (the package does not work at
  such magnitudes anyway, e.g. the area check overflows too)
- **non-zero area**: shoelace formula on coordinates translated to the first vertex (avoids cancellation)
- **crossings at self contacts** (``check_polygon_contacts()``): at every position where the polygon touches itself,
  two rays with the same direction mean overlapping edges (zero-width spikes or slits), and two corners
  whose rays interleave (exactly one ray of the second corner lies within the first corner) mean the polygon
  crosses itself

The errors name the polygon (``boundary polygon`` or ``hole <index>``) and the location: the crossing edges
(with the vertex indices of the input, also for polygons stored reversed) and their crossing,
or the position of a crossing or an overlap at a self contact.

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

Newer decisions are also listed in the :doc:`decision log <contributing/decisions>`.

Precompute once, query many times
---------------------------------

The visibility between extremities only depends on the environment and is computed in the constructor.
A query only has to compute the visibility of two points. This favours use cases with a static map and many queries
(games, robotics, simulations) and is the reason why the environment is immutable.

Precomputing all shortest paths between extremities was rejected: memory grows quadratically with the number of
extremities, and a fully connected graph would slow down A* (it would visit every node).

AREA instead of a different algorithm
-------------------------------------

The visibility stays computed with AREA, other algorithms are not used as a replacement (cf. `Related Work`_).
The cost of the package is the visibility from many points, one at a time: from every extremity during the
construction, from start and goal per query. Any algorithm of this kind costs at least linear time per point,
so replacing AREA would mainly change constant factors. A rotational sweep with a tree of edges (Lee) is slower
in Python on the measured maps (#216). Triangular expansion, a navigation mesh (Polyanya) or continuous Dijkstra
need a triangulation of the free space: the polygons would have to be merged first (they may touch and overlap),
the exact contact semantics re-established, and either a dependency added or a lot of code written.
AREA carries the tested behaviour at the edge cases, the audit against exact arithmetic
and the identical results with and without Numba.

Changes of the scaling are added next to AREA, not instead of it: precomputed data for faster queries (#223)
or an optional engine for very large or changing environments (#177). AREA then serves as the reference
for their results.

Angle representations instead of angles
---------------------------------------

Directions are compared with a monotone representation in ``[0, 4)`` (one unit per quadrant,
``_compute_repr_n_dist()``) instead of ``atan2``. It needs one division, keeps the
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

Own A* search
-------------

``utils.astar_path()`` is an own A* search that additionally follows the temporary query edges.
Adding and removing the edges on the shared graph, other graph libraries and Dijkstra searches were compared
and rejected (#152, details in the :ref:`contribution guidelines <contributing>`).
It replaced a port of ``networkx.astar_path()`` (#157), so the package is licensed under MIT only.
Ideas specific to visibility graphs were evaluated and not adopted, as they gave no measurable gain
(the search takes a few percent of the query time) or would only hold for Euclidean costs:

- **a closed set** (never expanding a node twice): with the consistent heuristic the distance check already
  prevents expanding a node again, and expanding again keeps the search exact with admissible heuristics.
- **skipping neighbours** (not measured): a neighbour of the current node that is also a neighbour of its predecessor
  cannot be reached shorter through the current node (triangle inequality). The distance check already rejects it,
  skipping it would need an extra check per neighbour instead.
- **earlier termination**: the goal is only final when it is taken from the queue.
  Stopping earlier, e.g. when the goal is first reached, is not exact.
- **a precomputed adjacency structure** (plain dicts or arrays): costs memory per environment and changes the pickled
  state, the iteration over the networkx dicts is not the bottleneck (cf. #182).

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

Validation by default
---------------------

The input (self intersections, crossings at self contacts) is validated by default (``validate=True``), because invalid
input without validation silently leads to undefined results, e.g. no path although start and goal are in the free space.
The validation is cheap compared to the preparation of the environment (measured on the benchmark world maps with
1,000 to 5,000 vertices, with and without Numba: within the noise of the measurement), and it accepted every
benchmark world map (1,009 to 20,262 vertices) and all but one of 180 real-world-like environments of
``prototypes/real_world_geometry.py``. The rejected one has a zero-width slit about :math:`10^{-17}` wide
(a Shapely union artefact) whose free side cannot be decided in floating point: without validation, 35 of 300
random paths ran through an obstacle. Callers with trusted or generated input can skip the validation with
``validate=False``.

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
- **Rays are exact**: directions that are exactly the same (zero cross product, same direction) share one angle
  representation, even when the representations computed from vectors of different length differ in the last bit
  (``Point.rep()``). ``RAY_WINDOW`` (1e-12) is the only numeric constant in the decisions. It is not a tolerance on any
  decision: it only limits the check for exactly collinear directions to those within 1e-12 of a bounding ray, far above
  the last-bit differences of their representations. The regression test ``test_in_wedges_collinear_last_bit`` covers
  directions whose representations differ from the ray's in the last bit. Directions that are not exactly collinear
  with a ray are decided by their representations, as everywhere else. The 1e-16 bound of the audit
  (**Audit against exact arithmetic**, below) concerns these non-collinear comparisons.
- **Lines of sight touching edges**: a line of sight touching an edge only in one of its vertices, or running along it,
  is not blocked by the edge. What happens at the vertex is decided by its wedges.
- Intersections are only computed (a 2x2 linear system, ``_lies_behind_inner()``) when cheaper tests
  (angle range, distance) cannot decide. A determinant rounding to 0 (a direction parallel to the edge in floating
  point) means "not behind the edge", in both modes.
- **Audit against exact arithmetic** (#3, ``prototypes/float_audit.py``, the integer case is tested in
  ``tests/test_exact_decisions.py``): every decision above was compared with
  the exact decision (``fractions.Fraction``) for the same ``float64`` values, on integer, decimal grid, projected
  (1e6 plus decimals), random and near-degenerate inputs (up to 100,000 cases per function and kind of input),
  with identical results with and without Numba. There were no misdecisions when all coordinates, query points
  included, are integers (up to 2^20). All misdecisions concern directions
  or points within about one unit in the last place of a decision boundary: directions whose exact angle
  representations differ by less than 1e-16 may compare equal or in the wrong order, and points about one unit in the
  last place next to an edge may count as on the edge (by the 180 degree rule of ``_lies_on_edge()``) or fall on the
  wrong side of it in the point in polygon test. Such inputs are within the rounding errors of the coordinates
  themselves; exact predicates everywhere would be a large and slower change to the core and are not used.
- **Query points next to an edge**: a query point within rounding error of an edge, but not exactly on it, may be
  rejected by ``within_map()``. With ``verify=False`` the visibility from it can also be wrong, so its path and length
  are undefined and can depend on the query direction (#239).
  Points exactly on an edge are exact (``_lies_on_edge()``): move such a point onto the edge or away from it.
  A tolerance snapping near points onto edges is not used: it would contradict "near-contacts do not block".


Complexity and Memory
=====================

With ``n`` vertices and ``m`` extremities (usually ``m << n``):

=================================  ==========================  =====================================================
Step                               Time                        Notes
=================================  ==========================  =====================================================
points and wedges                  ``O(n²)``                   vectorised bounding box test: edges through vertices
visibility graph                   ``O(m n log n)``            AREA for every extremity (typical case;
                                                               worst case ``O(n m²)``), cf. `Algorithms`_
query: visibility of start, goal   ``O(n log n + m)``          dominates the query time (typical case;
                                                               worst case ``O(n m)``)
query: A*                          ``O(E log E)``              ``E``: graph and temporary edges
=================================  ==========================  =====================================================

Memory is dominated by the graph (at most ``m (m - 1) / 2`` edges, in practice far fewer thanks to the graph reduction)
and the ``O(n)`` data of the vertices. The angle representations and distances are computed per extremity
and not stored (cf. `Angle representations and distances`_).
A query uses ``O(n + m)`` additional memory.

Profiling large environments shows that the A* search takes about 5% of the query time,
most of the time is spent on the visibility of start and goal. Optimisations should target ``find_visible()``.
``prototypes/complexity_profile.py`` measures the steps of the preparation and how often AREA checks a candidate
(the measured maps stay in its typical case, #215).
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


.. _related_work:

Related Work
============

How the approach of this package relates to the literature on Euclidean shortest paths among polygonal obstacles.
Packages solving the same problem are compared in :ref:`alternatives`.

Constructing visibility graphs
    Lee's rotational sweep (1978) computes the visibility graph of :math:`n` vertices in :math:`O(n^2 \log n)` time.
    Welzl (1985), Asano et al. (1986) and Overmars and Welzl (1988) reduced this to :math:`O(n^2)`,
    `Ghosh and Mount (1991) <https://www.cs.umd.edu/~mount/Papers/sicomp91-visib.pdf>`__ to the output sensitive
    :math:`O(n \log n + E)` for :math:`E` visibility edges, and
    `Chen and Wang (2015) <https://doaj.org/article/83ae71cdf1754c2d838667b17b0f2432>`__ to
    :math:`O(n + h \log h + E)` for :math:`h` obstacles, given a triangulation of the free space.
    AREA does not improve these bounds, its worst case, :math:`O(n^3)` for :math:`m \sim n`, is worse than Lee's
    (cf. `Angle Range Elimination Algorithm (AREA)`_, which also names the techniques it builds on).
    It trades the worst case guarantee for fewer and simpler operations on typical maps.
    We are not aware of a published description of its edge-centric sweep.

Shortest paths without a visibility graph
    The continuous Dijkstra method computes a single shortest path in :math:`O(n \log n)` time
    (Hershberger and Suri, 1999; Wang, SODA 2021, with :math:`O(n)` space),
    and in :math:`O(n + h \log h)` given a triangulation of the free space
    (`Wang, STOC 2021 <https://digitalcommons.usu.edu/computer_science_facpubs/20>`__).
    These algorithms are optimal for single queries, but complex to implement. This package instead precomputes a
    graph for many queries in a static environment.

Reduced visibility graphs
    Restricting the graph to the extremities (cf. `Why extremities suffice`_) and to the edges that can be part of
    shortest paths (bitangents and obstacle edges between extremities) is known as the *tangent graph* (Liu and Arimoto, 1992) or *reduced visibility graph*
    (`LaValle, Planning Algorithms, 6.2.4 <https://lavalle.pl/planning/node271.html>`__).
    The graph reduction of this package follows Vinther et al. (2015, cf. `Visibility graph reduction`_).
    `Sparse visibility graphs <https://arxiv.org/abs/1702.01524>`__ (Oh and Leong, 2017) prune edges
    with the same taut path argument and build a hierarchy of them for any-angle paths on grids.

Queries
    Like this package, End Point Search (Shen et al., 2020) and
    `Euclidean hub labelling <https://ojs.aaai.org/index.php/AAAI/article/view/26463>`__ (Du, Shen and Cheema, 2023)
    connect start and goal to the extremities visible from them. Instead of searching a graph, they look up
    distances between the extremities precomputed on the reduced visibility graph (hub labelling also stores the
    visible extremities of grid cells): much faster queries at the cost of a lot of memory
    (reduced by `EHL* <https://arxiv.org/abs/2408.11341>`__).
    Polyanya (Cui, Harabor and Grastien, 2017) searches a navigation mesh instead.
    `RayScan <https://ojs.aaai.org/index.php/ICAPS/article/view/6654>`__ (Hechenberger et al., 2020)
    needs no preprocessing at all: it computes the visible vertices during the A* search with ray shooting,
    which suits changing environments.

Visibility from a point
    The `triangular expansion algorithm <https://arxiv.org/abs/1403.3905>`__ (Bungiu et al., 2014) computes the
    visibility from a point by expanding the triangles of a triangulation of the free space.
    `TřiVis <https://arxiv.org/abs/2410.08752>`__ (Mikula, Kulich and Přeučil, 2024) implements it,
    vertex to vertex visibility graphs included. It is the most relevant alternative to AREA for both construction
    and queries, but is based on a triangulation of the free space (C++, not compared yet).


Limitations and Possible Extensions
===================================

- **Dynamic environments**: changes require a new environment. Computing visibility on demand,
  or updating only the affected part of the graph, would allow changing obstacles.
- **Parallel graph construction**: the visibility of each extremity can be computed independently
  (the symmetry optimisation would have to be given up or the results merged), which would parallelise well.
- **Query speed**: the visibility of start and goal is computed from scratch for every query.
  Spatial indices over the edges could reduce the number of edges checked per query.
- **Dense grids**: environments with many small obstacles have many extremities. A grid based search can be
  faster there (cf. :ref:`usage`).

Ideas and planned changes are tracked as `GitHub Issues <https://github.com/jannikmi/extremitypathfinder/issues>`__.
