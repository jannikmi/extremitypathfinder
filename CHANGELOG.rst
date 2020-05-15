Changelog
=========


1.2.0 (2020-05-15)
------------------

* supporting only python 3.6+
* fix #10: Memory leak in DirectedHeuristicGraph
* fix BUG where extremities would be deleted from the visibility graph
* using generators to refer to the polygon properties (vertices,...) of an environment (save memory and remove redundancy)


todo copy deep
todo docstrings
TODO sphinx documentation
TODO api doc


internal:

* add sponsor button
* updated publishing routine
* split up requirement files (basic, tests)
* specific tags for supported python versions in wheel
* testing all different python versions with tox
* added coverage tests
* added editorconfig
* specify version in VERSION file


1.1.0 (2018-10-17)
------------------

* optimised A*-algorithm to not visit all neighbours of the current node before continuing



1.0.0 (2018-10-07)
------------------

* first stable public version



0.0.1 (2018-09-27)
------------------

* birth of this package

