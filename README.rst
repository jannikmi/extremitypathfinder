===================
extremitypathfinder
===================

..
    Note: can't include the badges file from the docs here, as it won't render on PyPI -> sync manually


.. image:: https://github.com/jannikmi/extremitypathfinder/actions/workflows/build.yml/badge.svg?branch=master
    :alt: CI status
    :target: https://github.com/jannikmi/extremitypathfinder/actions?query=branch%3Amaster

.. image:: https://readthedocs.org/projects/extremitypathfinder/badge/?version=latest
    :alt: documentation status
    :target: https://extremitypathfinder.readthedocs.io/en/latest/?badge=latest

.. image:: https://img.shields.io/pypi/v/extremitypathfinder.svg
    :alt: latest version on PyPI
    :target: https://pypi.python.org/pypi/extremitypathfinder

.. image:: https://img.shields.io/pypi/pyversions/extremitypathfinder.svg
    :alt: supported Python versions
    :target: https://pypi.python.org/pypi/extremitypathfinder

.. image:: https://static.pepy.tech/badge/extremitypathfinder
    :alt: total PyPI downloads
    :target: https://pepy.tech/projects/extremitypathfinder

.. image:: https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white
    :alt: pre-commit
    :target: https://github.com/pre-commit/pre-commit

.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :alt: code style: ruff
    :target: https://github.com/astral-sh/ruff


Python package for fast geometric shortest path computation in 2D multi-polygon or grid environments
based on visibility graphs.

.. image:: https://raw.githubusercontent.com/jannikmi/extremitypathfinder/master/docs/_static/demo.svg
    :alt: shortest path through a polygon environment


Features
--------

- **Exact shortest paths**: the Euclidean shortest path between any two points of the free space,
  not an approximation on a grid or mesh.
- **Fast repeated queries**: the visibility graph of an environment is computed once,
  each query then only connects its start and goal to it.
  Only the protruding corners ("extremities") become graph nodes, which keeps the graph small.
- **Polygon environments**: a boundary polygon with holes, or holes only (unbounded environments).
  Polygons may touch and overlap each other. Touching obstacles form a wall that paths cannot pass.
  The polygons are used directly: no triangulation or navigation mesh is needed.
- **Grids**: convert a grid with blocked cells into a polygon environment.
- **Few dependencies**: NumPy and NetworkX. Optional `Numba <https://numba.pydata.org/>`__ acceleration.
- **Extras**: input validation, plotting (Matplotlib), caching environments with pickle,
  and a command line interface.


Installation
------------

.. code-block:: console

    pip install extremitypathfinder

With the optional Numba extra for a significant speedup
(larger installation, initial compilation time):

.. code-block:: console

    pip install "extremitypathfinder[numba]"

Python 3.12 or newer is required (CI tests CPython 3.12–3.15).
Dependencies: NumPy >=2.3.3,<3 and NetworkX 3.x.
Cf. `supported versions and dependencies
<https://extremitypathfinder.readthedocs.io/en/latest/0_getting_started.html#supported-versions-and-dependencies>`__
for the support policy.


Quick Start
-----------

Polygon environment
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

    from extremitypathfinder import PolygonEnvironment

    # the boundary polygon (clockwise or counter clockwise vertex numbering)
    boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # the holes (obstacles)
    list_of_holes = [
        [
            (3.0, 7.0),
            (5.0, 9.0),
            (4.5, 7.0),
            (5.0, 4.0),
        ],
    ]
    # computes the visibility graph once
    environment = PolygonEnvironment(boundary_coordinates, list_of_holes, validate=True)

    # fast consecutive queries
    path, length = environment.find_shortest_path((4.5, 1.0), (4.0, 8.5))
    # path: [(4.5, 1.0), (3.0, 7.0), (4.0, 8.5)], length: 7.987...

``path`` is ``[]`` and ``length`` is ``None`` when there is no path between start and goal.
``validate=True`` checks the input polygons (vertex order, self intersections, ...).
For trusted input it can be skipped to save time (``validate=False``, the default).

Grid environment
~~~~~~~~~~~~~~~~

Coordinates refer to the corners of the grid cells, i.e. cell ``(x, y)`` spans ``[x, x+1] x [y, y+1]``:

.. code-block:: python

    size_x, size_y = 5, 3
    obstacle_iter = [(2, 1), (3, 1)]  # blocked grid cells (x, y)
    environment = PolygonEnvironment.from_grid(size_x, size_y, obstacle_iter)
    path, length = environment.find_shortest_path((0.5, 0.5), (4.5, 2.5))
    # path: [(0.5, 0.5), (2.0, 2.0), (4.5, 2.5)], length: 4.670...

Command line
~~~~~~~~~~~~

Environments stored as JSON (cf. ``example.json``) can be queried from the command line:

.. code-block:: console

    extremitypathfinder ./example.json -s 2.5 3.2 -g 7.9 6.8


Documentation
-------------

The `documentation <https://extremitypathfinder.readthedocs.io/en/latest/>`__ covers:

- `Usage <https://extremitypathfinder.readthedocs.io/en/latest/1_usage.html>`__:
  all features, the input requirements, plotting and caching
- `Upgrading from 2.x <https://extremitypathfinder.readthedocs.io/en/latest/2_upgrading.html>`__
- `About <https://extremitypathfinder.readthedocs.io/en/latest/3_about.html>`__:
  how it works and the handling of touching obstacles
- `Engineering Design <https://extremitypathfinder.readthedocs.io/en/latest/8_design.html>`__:
  the architecture and the algorithms
- `Alternatives <https://extremitypathfinder.readthedocs.io/en/latest/9_alternatives.html>`__:
  similar packages and how they differ
- `API reference <https://extremitypathfinder.readthedocs.io/en/latest/4_api.html>`__


Contributing and License
------------------------

Contributions are welcome: see the
`contribution guidelines <https://extremitypathfinder.readthedocs.io/en/latest/5_contributing.html>`__
and the `issue tracker <https://github.com/jannikmi/extremitypathfinder/issues>`__.

``extremitypathfinder`` is distributed under the terms of the MIT license
(`LICENSE <https://github.com/jannikmi/extremitypathfinder/blob/master/LICENSE>`__).
The A* search (``utils.astar_path()``) is adapted from NetworkX and distributed under its 3-clause BSD license
(`LICENSES/networkx.txt <https://github.com/jannikmi/extremitypathfinder/blob/master/LICENSES/networkx.txt>`__).

Also see:
`GitHub <https://github.com/jannikmi/extremitypathfinder>`__,
`PyPI <https://pypi.python.org/pypi/extremitypathfinder/>`__,
`Changelog <https://extremitypathfinder.readthedocs.io/en/latest/6_changelog.html>`__
