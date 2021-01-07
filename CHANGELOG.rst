Changelog
=========


TODO python 3.6 support. tests not passing so far only for this python version



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

