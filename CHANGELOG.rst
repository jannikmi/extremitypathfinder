Changelog
=========


TODO python 3.6 support. tests not passing so far only for this python version


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

