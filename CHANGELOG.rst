Changelog
=========


2.7.2 (2024-04-10)
-------------------

- added support for python 3.12

internal:

- added tests for python 3.12
- use ruff pre-commit hooks
- made dependency groups docs and plot optional
- added tox tes for documentation build



2.7.1 (2023-05-16)
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

* implemented an optimised visibility graph algorithm: sort edges and candidates after their representation to always only check the relevant fraction of candidates for each edge. Runtime complexity O(n^2 log_2 n).
* added visibility computation tests
* automatically skip GitHub actions publishing when the version already exists. useful for minor improvements without publishing a version. build would always fail otherwise
* updated pinned dependencies to fix security alerts
* minor code refactoring


2.5.0 (2023-05-05)
-------------------

* removed need for separate ``.prepare()`` call. Storing environment boundary data automatically triggers the preparation of the visibility graph. This is a non-breaking change. The ``.prepare()`` method is still available, but it is not needed anymore.

internal:

* updated dependency specification: networkx>=3, relaxed development dependency version requirements
* included tests for python 3.11
* minor code refactoring


2.4.1 (2022-08-22)
-------------------

* bugfix: catch the case where no path is possible in the graph in the ``networkx`` A* implementation
* added speed benchmarks and performance section in the documentation with benchmark results

internal:

* optimisation: checking edges with the biggest angle range first
* optimisation: skipping visibility checks for the last extremity
* using optimised point in polygon check algorithm
* using undirected Graph: The precomputed graph usually makes up the majority of the visibility graph (in comparison to the temporarily added edges for query start and goal nodes) and this precomputed part has to be undirected. Use undirected graph everywhere.
* added test cases


2.4.0 (2022-08-18)
-------------------

* A* and graph representation based on ``networkx`` library -> new dependency



2.3.0 (2022-08-18)
-------------------

* major overhaul of all functionality from OOP to functional programming/numpy based

internal:

* added test cases




2.2.3 (2022-10-11)
-------------------

* reverting changes of version ``2.2.2``


2.2.2 (2022-07-10)
-------------------

* [DEPRECATED]


2.2.1 (2022-07-10)
-------------------

* packaging completely based on ``pyproject.toml`` (poetry)
* CI/CD: automatic publishing based on GitHub Actions

2.2.0  (2021-01-25)
-------------------

* Included a command line interface
* Improved testing routines and codestyle


2.1.0 (2021-01-07)
------------------

IMPORTANT BUGFIX: in some cases the visibility computation was faulty (fix #23)

* added new test case

2.0.0 (2020-12-22)
------------------

* IMPROVEMENT: Different polygons may now intersect each other. Thanks to `Georg Hess <https://github.com/georghess>`__!
* BUGFIX: Fixed a bug that caused "dangling" extremities in the graph to be left out
* ``TypeError`` and ``ValueError`` are being raised instead of ``AssertionError`` in case of invalid input parameters with ``validate=True``. Thanks to `Andrew Costello <https://github.com/Andrewsyl>`__!

1.5.0 (2020-06-18)
------------------

* BUGFIX: fix #16. introduce unique ordering of A* search heap queue with custom class ``SearchState`` (internal)


1.4.0 (2020-05-25)
------------------

* BUGFIX: fix clockwise polygon numbering test (for input data validation, mentioned in #12)



1.3.0 (2020-05-19)
------------------

* FIX #11: added option ``verify`` to ``find_shortest_path()`` for skipping the 'within map' test for goal and start points



1.2.0 (2020-05-18)
------------------

* supporting only python 3.7+
* fix #10: Memory leak in DirectedHeuristicGraph
* fix BUG where "dangling" extremities in the visibility graph would be deleted
* using generators to refer to the polygon properties (vertices,...) of an environment (save memory and remove redundancy)
* enabled plotting the test results, at the same time this is testing the plotting functionality
* added typing

internal:

* added sphinx documentation, included auto api documentation, improved docstrings
* added contribution guidelines
* add sponsor button
* updated publishing routine
* split up requirement files (basic, tests)
* specific tags for supported python versions in wheel
* testing all different python versions with tox
* added coverage tests
* added editorconfig
* specify version in VERSION file
* added new tests


1.1.0 (2018-10-17)
------------------

* optimised A*-algorithm to not visit all neighbours of the current node before continuing



1.0.0 (2018-10-07)
------------------

* first stable public version



0.0.1 (2018-09-27)
------------------

* birth of this package
