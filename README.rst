===================
extremitypathfinder
===================

.. image:: https://travis-ci.org/MrMinimal64/extremitypathfinder.svg?branch=master
    :target: https://travis-ci.org/MrMinimal64/extremitypathfinder


.. image:: https://img.shields.io/pypi/wheel/extremitypathfinder.svg
    :target: https://pypi.python.org/pypi/extremitypathfinder


.. image:: https://img.shields.io/pypi/v/extremitypathfinder.svg
    :target: https://pypi.python.org/pypi/extremitypathfinder

.. image:: https://anaconda.org/conda-forge/extremitypathfinderr/badges/version.svg
    :target: https://anaconda.org/conda-forge/extremitypathfinder

Python package for geometric shortest path computation for given 2D multi-polygon maps based on visibility.

Also see:
`GitHub <https://github.com/MrMinimal64/extremitypathfinder>`__,
`PyPI <https://pypi.python.org/pypi/extremitypathfinder/>`__,
`conda-forge feedstock <https://github.com/conda-forge/extremitypathfinder-feedstock>`__,


Dependencies
============

(``python``),
``numpy``,
``matplotlib``,


Installation
============


Installation with conda: see instructions at `conda-forge feedstock <https://github.com/conda-forge/extremitypathfinder-feedstock>`__ (NOTE: The newest version of extremitypathfinder might not be available via conda yet)



Installation with pip:
in the command line:

::

    pip install extremitypathfinder





Usage
=====

Data format:
____________



::

    TODO


plotting


**Note:** As mentioned in `[1, Ch. III 6.3] <http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__ in Chessboard like Gridworlds it can be better to use A* right away.


Basic Idea
==========

TODO map plot


Well described in `[1, Ch. II 3.2] <http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__:

An map/environment/world of a given shortest path problem can be represented by one boundary polygon with holes (themselves polygons).

IDEA: Two categories of vertices/corners can be distinguished in polygons:

* protruding corners (hereafter called **"Extremities"**, marked in red)
* all others


Extremities have an inner angle (facing towards the inside of the map) of > 180 degree.
As long as there are no obstacles between two points present, it is obviously always best (=shortest) to move to the goal point directly.
When obstacles obstruct the direct path (goal is not directly 'visible' from the start) however, extremities (and only extremities!) have to be visited until the goal is directly visible.

*Improvement:* As described in `[1, Ch. II 4.4.2 "Property One"] <http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__ during preprocessing time the visibility graph can be reduced further without the loss of guaranteed optimality of the algorithm:
All extremities lying "in front of" an extremity e such that they


Algorithm
=========

This package pretty much implements the Visibility Graph Optimized (VGO) Algorithm described in `[1, Ch. II 4.4.2] <http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__, just with a few computational tweaks:


Rough Procedure:
________________

`` TODO include plots for each step``


- **1. Preprocessing the map:** Independently of any query start and goal points the optimized visibility graph is being computed for the static environment once with ``map.prepare()``. Later versions might include a faster approach to compute visibility on the fly, for use cases where the map is changing dynamically

- **2. Including start and goal:** The start and goal points are being connected to the graph depending on their visibility

- **3. A-star shortest path computation :** Finding the shortest path on graphs is a well known problem. Use a version of the popular ``A*-Algorthm`` optimized for this special use case.


Tweaks (my contribution):
_________________________

**Visibility detection:**
To my knowledge the was no previous algorithm for computing the visibility of points that is visiting edges at most once without any intersection, distance or trigonometric computations and without sorting.

todo explain angle representation
todo explain algorithm

todo link other algorithms


**Modifications to A-star**: The basic algorithm has been modified to exploit the following geometrical property of this specific task (and hence also the extracted graph):
it is always shortest to directly reach a node instead of visiting other nodes first
(there is never an advantage through reduced edge weight).

This can be exploited in a lot of cases to make a* terminate earlier than for general graphs:

- when the goal is directly reachable, there can be no other shorter path to it. Terminate.

- when always only expanding the nodes with the lowest estimated cost (lower bound), there is no need to revisit nodes (path only gets longer)



Comparison to pyvisgraph
========================

todo link


Pros:
- computationally superior procedure in theory


Cons:

- so far some missing features
- new package, might contain bugs
- no existing speed comparison


Contact
=======

Most certainly there is stuff I missed, things I could have optimized even further or explained more clearly, etc. I would be really glad to get some feedback on my code.

If you encounter any bugs, have suggestions, criticism, etc.
feel free to **open an Issue**, **add a Pull Requests** on Git or ...

contact me: *[python] {*-at-*} [michelfe] {-*dot*-} [it]*



License
=======

``timezonefinder`` is distributed under the terms of the MIT license
(see LICENSE.txt).


References
==========

[1] Vinther, Anders Strand-Holm, Magnus Strand-Holm Vinther, and Peyman Afshani. `"Pathfinding in Two-dimensional Worlds" <http://www.cs.au.dk/~gerth/advising/thesis/anders-strand-holm-vinther_magnus-strand-holm-vinther.pdf>`__. no. June (2015).