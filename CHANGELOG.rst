Changelog
=========

Unreleased
----------

- documentation: engineering design document (goals, architecture, data model, pipelines, algorithms and design decisions).
  The implementation details moved there from the about page.

bugfixes:

- documentation: the API documentation lists the plotting functions
  (matplotlib was missing from the documentation dependencies, their import failed).

internal:

- the documentation build in CI (``tox -e docs``) fails on warnings.


3.0.0 (2026-10-29)
------------------

A major release: the environment is set up in one step, and touching obstacles now block paths.
See the `upgrade guide <https://extremitypathfinder.readthedocs.io/en/latest/2_upgrading.html>`__
for how to port code written for 2.x.

BREAKING: the environment is initialised in one step:

- ``PolygonEnvironment(boundary_coordinates, list_of_hole_coordinates, validate=False)`` stores the polygons
  and computes the visibility graph. It replaces ``PolygonEnvironment()`` followed by ``.store()``.
- ``PolygonEnvironment.from_grid(size_x, size_y, obstacle_iter)`` replaces ``.store_grid_world()``.
  ``utils.convert_gridworld()`` is renamed to ``utils.convert_grid()``.
- removed the ``simplify`` parameter of ``from_grid()`` and ``utils.convert_grid()``.
  Polygon simplification was never implemented: ``store_grid_world()`` raised a ``NotImplementedError``
  with its default ``simplify=True``. Simplifying geometries is out of scope of this package (#68).
- removed the ``.prepare()`` method and the ``prepared`` attribute
  (a separate ``.prepare()`` call has not been needed since 2.5.0).
- removed the ``free_space_after`` parameter of ``find_shortest_path()`` and the ``temp_graph`` attribute.
  The search graph of a query is returned by ``search()`` instead of being stored in the environment.
- ``verify`` is a keyword-only parameter of ``find_shortest_path()`` and ``search()``: pass ``verify=False``.
  A positional third argument (e.g. ``free_space_after`` of 2.x) raises a ``TypeError``.
- new ``search()`` method: performs the same search as ``find_shortest_path()``, but returns a ``SearchResult``
  with the search data (start and goal nodes, their temporary edges, coordinates) in addition to the path
  and its length. The search graph (``SearchResult.graph``) is only built on request.
- removed ``PlottingEnvironment``. Use the functions ``plot_environment(environment, plotting_dir)``
  and ``plot_search(environment, result, plotting_dir)`` in ``extremitypathfinder.plotting`` instead.
  The plotting directory is a parameter of these functions instead of a global setting.
- environments pickled with earlier versions cannot be loaded anymore (new attributes): recreate them.
  Loading them raises a ``ValueError`` explaining this (pickles now store a format version).

BREAKING: touching obstacles block paths (#24):

- obstacles (the holes and everything outside the boundary polygon) that touch form a wall.
  A path never passes between two obstacles that touch, and never through a point where an obstacle touches itself.
  Previously contacts did not block. Every kind of contact blocks: shared vertices, a vertex on an edge,
  overlapping collinear edges, holes touching the boundary polygon and polygons touching themselves.
  Paths may still run along obstacle edges and through the vertices of a single obstacle.
- grids: diagonally adjacent blocked cells block paths (no corner cutting).
- a contact means exactly equal coordinates or a vertex exactly on an edge
  (an exact cross product test, e.g. for integer coordinates). Near-contacts (tiny gaps) do not block.
- query points exactly on a contact point may leave through any free space next to it.
  Query points on an edge shared by two touching obstacles have no path.
- at most one extremity per point: coincident vertices are no longer merged into one graph node.
  At a contact, a vertex is only an extremity when the free angle there is larger than 180 degree
  (e.g. two corners touching at a grid diagonal are no extremities anymore).
- removed the attribute ``PolygonEnvironment.extremity_mask``. New attribute ``points``:
  the vertex positions within the map and the free space around them (``extremitypathfinder.wedges``).
- ``validate=True`` detects crossings at vertices (#135): a vertex lying on another edge of the same polygon
  with its neighbours on opposite sides, and identical vertices whose corners interleave.
  Collinear overlapping edges within a polygon (zero-width spikes or slits) are rejected as well.
  Polygons touching themselves without crossing stay valid.

other changes:

- faster queries: the visibility graph is no longer copied for every query (#136).
  The A* search uses the precomputed graph and the temporary edges of start and goal directly.
  On large environments copying made up most of the query time (900 extremities: 72 ms -> 16 ms per query).
- license: the A* search is adapted from ``networkx.astar_path()`` and under the 3-clause BSD license of networkx
  (``LICENSES/networkx.txt``). The package license is now ``MIT AND BSD-3-Clause``.
- support CPython 3.15: tested in CI with the latest and the oldest dependencies providing 3.15 wheels.
- documentation: upgrade guide from 2.x (porting code and the changed handling of touching obstacles).
- documentation: grid example in the README.

bugfixes:

- lines of sight no longer pass straight through vertices with a free angle of at most 180 degree.
  Previously paths could run through a hole along collinear edges and reflex vertices (#84).
- a goal identical to an extremity is merged with it in the search graph.
  Previously such queries could return a longer path or no path at all.
- a query point identical to a polygon vertex sees along the edges of that vertex,
  and a query point on an edge sees along that edge (paths along polygon edges were partly blocked before).
  This also holds for edges whose vertex coordinates make the point only approximately lie on the edge.
- queries no longer store data in the environment (the angle representations of every query point were kept).
- ``validate=True`` no longer rejects some valid triangles as self-intersecting
  (the first and the last edge were not treated as neighbours).
- ``from_grid()`` supports grids without obstacles.
- documentation: ``load_pickle`` is imported from ``extremitypathfinder`` (the documented import failed).
- documentation: describe the A* search correctly, list the exceptions raised by the ``PolygonEnvironment`` constructor.

internal:

- one concept for the free space at the vertices (``wedges.py``): every vertex position is a point with free wedges.
  Extremities, the directions leaving an origin and passing or arriving at vertices follow the same rules at every
  point (contacts included). ``find_visible()`` only checks the edges (no special cases for edges touching the origin),
  ``find_visible_from()`` applies the rules of the free wedges. Shared point on edge test ``_lies_on_edge()``.
- core review follow-up: the graph reduction ("in front of each other") also applies to the last extremity
  and uses the free wedges of the points (``Point.in_front()``). Passing a contact point is detected despite
  rounding errors of the angle representations. Vectorised the angle representation computation,
  removed unused helper functions, added regression tests (``tests/test_core_regressions.py``).
- added tests for symmetric visibility, symmetric shortest paths and the graph construction.
- resolved or removed the ``TODO`` comments in the code, open topics are tracked as GitHub issues.
- switched the project tooling from Poetry to uv: standard ``[project]`` metadata and ``[dependency-groups]``,
  ``uv_build`` build backend, ``uv.lock`` (#142).
- automated releases: "prepare release" and "tag release" GitHub workflows, ``scripts/release.py``.
- Dependabot keeps the GitHub Actions (grouped into one PR) and the pre-commit hooks up to date (#76).
- updated the pre-commit hooks (``make hookup``, e.g. ruff 0.17) and applied their fixes.


2.8.0 (2026-09-18)
-------------------

- Add support for unbounded polygon environments without an explicit boundary.
- Support CPython 3.12–3.14 and NumPy 2.3.3+.
- Drop Python <3.12 and NumPy <2.3.3; use shared dependency floors across Python versions.
- Test minimum and latest dependencies with and without Numba in isolated
  wheel installations; update Numba and SciPy version constraints.
- Validate malformed, non-finite, and zero-area polygon inputs predictably and
  add deterministic geometry correctness regressions.

internal:

- Build and install-test wheel and source artifacts before publishing, including
  the public API, CLI, README example, and optional Numba acceleration.
- Gate publishing on a matching version tag and finalize the GitHub release only
  after PyPI publication succeeds.


2.7.2 (2024-04-10)
-------------------

- added support for python 3.12

internal:

- added tests for python 3.12
- use ruff pre-commit hooks
- made dependency groups docs and plot optional
- added tox test for documentation build



2.7.1 (2023-06-16)
-------------------

internal:

- JIT compile more utility functions (including numpy.linalg.solve)
- add scipy dependency for JIT compiling numpy.linalg.solve
- updated supported python versions to ">=3.8,<3.12" (required by scipy)
- remove debug print statements and assertions


2.7.0 (2023-06-08)
-------------------

- optional Numba JIT compilation offering significant speedup
- extra: `pip install extremitypathfinder[numba]`


2.6.0 (2023-06-04)
-------------------

internal:

- implemented an optimised visibility graph algorithm: sort edges and candidates after their representation to always only check the relevant fraction of candidates for each edge. Runtime complexity O(n^2 log_2 n).
- added visibility computation tests
- automatically skip GitHub actions publishing when the version already exists. useful for minor improvements without publishing a version. build would always fail otherwise
- updated pinned dependencies to fix security alerts
- minor code refactoring


2.5.0 (2023-05-05)
-------------------

- removed need for separate ``.prepare()`` call. Storing environment boundary data automatically triggers the preparation of the visibility graph. This is a non-breaking change. The ``.prepare()`` method is still available, but it is not needed anymore.

internal:

- updated dependency specification: networkx>=3, relaxed development dependency version requirements
- included tests for python 3.11
- minor code refactoring


2.4.1 (2022-08-22)
-------------------

- bugfix: catch the case where no path is possible in the graph in the ``networkx`` A* implementation
- added speed benchmarks and performance section in the documentation with benchmark results

internal:

- optimisation: checking edges with the biggest angle range first
- optimisation: skipping visibility checks for the last extremity
- using optimised point in polygon check algorithm
- using undirected Graph: The precomputed graph usually makes up the majority of the visibility graph (in comparison to the temporarily added edges for query start and goal nodes) and this precomputed part has to be undirected. Use undirected graph everywhere.
- added test cases


2.4.0 (2022-08-19)
-------------------

- A* and graph representation based on ``networkx`` library -> new dependency



2.3.0 (2022-08-18)
-------------------

- major overhaul of all functionality from OOP to functional programming/numpy based

internal:

- added test cases




2.2.3 (2022-08-11)
-------------------

- reverting changes of version ``2.2.2``


2.2.2 (2022-07-10)
-------------------

- yanked: release was faulty, changes reverted in ``2.2.3``


2.2.1 (2022-07-10)
-------------------

- packaging completely based on ``pyproject.toml`` (poetry)
- CI/CD: automatic publishing based on GitHub Actions

2.2.0  (2021-01-25)
-------------------

- Included a command line interface
- Improved testing routines and codestyle


2.1.0 (2021-01-07)
------------------

IMPORTANT BUGFIX: in some cases the visibility computation was faulty (fix #23)

- added new test case

2.0.0 (2020-12-22)
------------------

- IMPROVEMENT: Different polygons may now intersect each other. Thanks to `Georg Hess <https://github.com/georghess>`__!
- BUGFIX: Fixed a bug that caused "dangling" extremities in the graph to be left out
- ``TypeError`` and ``ValueError`` are being raised instead of ``AssertionError`` in case of invalid input parameters with ``validate=True``. Thanks to `Andrew Costello <https://github.com/Andrewsyl>`__!

1.5.0 (2020-06-18)
------------------

- BUGFIX: fix #16. introduce unique ordering of A* search heap queue with custom class ``SearchState`` (internal)


1.4.0 (2020-05-25)
------------------

- BUGFIX: fix clockwise polygon numbering test (for input data validation, mentioned in #12)



1.3.0 (2020-05-19)
------------------

- FIX #11: added option ``verify`` to ``find_shortest_path()`` for skipping the 'within map' test for goal and start points



1.2.0 (2020-05-18)
------------------

- supporting only python 3.7+
- fix #10: Memory leak in DirectedHeuristicGraph
- fix BUG where "dangling" extremities in the visibility graph would be deleted
- using generators to refer to the polygon properties (vertices,...) of an environment (save memory and remove redundancy)
- enabled plotting the test results, at the same time this is testing the plotting functionality
- added typing

internal:

- added sphinx documentation, included auto api documentation, improved docstrings
- added contribution guidelines
- add sponsor button
- updated publishing routine
- split up requirement files (basic, tests)
- specific tags for supported python versions in wheel
- testing all different python versions with tox
- added coverage tests
- added editorconfig
- specify version in VERSION file
- added new tests


1.1.0 (2018-10-17)
------------------

- optimised A*-algorithm to not visit all neighbours of the current node before continuing



1.0.0 (2018-10-07)
------------------

- first stable public version



0.0.1 (2018-09-27)
------------------

- birth of this package
