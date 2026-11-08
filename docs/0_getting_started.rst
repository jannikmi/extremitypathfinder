.. _getting_started:

===============
Getting started
===============


Installation
------------

Installation with pip:


.. code-block:: console

    pip install extremitypathfinder


.. benchmark-derived: speed and memory with and without Numba. Update this text with every full benchmark run
   (docs/contributing/performance.md, "After a full run").

Installation with Numba for a significant speedup, with the tradeoff of a larger installation footprint,
more memory and an initial compilation time (see :ref:`with-or-without-numba`):

.. code-block:: console

    pip install "extremitypathfinder[numba]"


.. _with-or-without-numba:

With or without Numba
~~~~~~~~~~~~~~~~~~~~~

Numba is optional. The choice is mainly speed against memory:

- **Results:** paths and lengths are identical with and without Numba.
- **Speed:** without Numba, building an environment takes about twice as long,
  and queries take moderately longer (cf. the speed-up by Numba on the :ref:`performance page <speed-tests>`).
- **Memory:** Numba and the compiled functions are loaded when the package is imported, in every process using it,
  even if it only loads a pickled environment. This adds tens of MiB to each process (more in the resident memory
  on Linux than in the footprint on macOS, where the benchmarks ran) and roughly doubles the memory of the process
  after the imports. On small maps, it is a large part of the memory a process needs
  (cf. the memory composition on the :ref:`performance page <speed-tests>`).
- **First use:** the first import compiles the functions, which takes several seconds and temporarily more memory.
  Later imports load them from Numba's cache. The cache belongs to the installation:
  e.g. every fresh container compiles again, unless the cache is built into its image.

Install without Numba when memory matters more than speed, e.g. on small maps, with many worker processes,
in memory-limited containers or in short-lived processes. Install with Numba for large maps and many queries.
``extremitypathfinder`` uses Numba whenever it can be imported: to run without it, install the package
without the ``numba`` extra into an environment without Numba.



Supported versions and dependencies
-----------------------------------

Python >=3.12,<4 is accepted; CI currently tests CPython 3.12–3.15.
Dependencies are NetworkX 3.x and NumPy >=2.3.3,<3.
Python <3.12 and NumPy <2.3.3 are no longer supported.

The Python and NumPy floors stay within the September 2026 downstream support
window in `NEP 29 <https://numpy.org/neps/nep-0029-deprecation_policy>`__,
now superseded by `SPEC 0 <https://scientific-python.org/specs/spec-0000/>`__.
Both policies exclude Python 3.11 by this date. This compatibility window is
narrower than CPython's security support lifetime. Review the floors for future
releases; SPEC 0 recommends dropping Python 3.12 in October 2026.
The declared dependency minimums are the same for all Python versions.
On Python 3.15 they provide no wheels, so CI tests the oldest releases that do instead:
NumPy 2.5.2 and Numba 0.68.0.
These recommendations do not promise upstream bug fixes for every included
NumPy release.
CI runs the full suite with minimum and latest compatible dependencies,
both with and without the ``numba`` extra, on every supported Python version.

The optional ``numba`` extra installs Numba >=0.63,<1 (SciPy is not needed).
Pip selects compatible versions; Numba may constrain
NumPy more tightly than the ordinary installation.
Acceleration depends on Numba/llvmlite platform support and adds installation
size and initial compilation time. Standard, GIL-enabled CPython is tested;
free-threaded builds and alternative interpreters are not covered by CI.

See ``pyproject.toml`` for the complete dependency constraints.



Basics
------



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
    environment = PolygonEnvironment(boundary_coordinates, list_of_holes, validate=False)
    start_coordinates = (4.5, 1.0)
    goal_coordinates = (4.0, 8.5)
    path, length = environment.find_shortest_path(start_coordinates, goal_coordinates)



All available features of this package are explained :ref:`HERE <usage>`.
Upgrading from version 2.x: see the :ref:`upgrade guide <upgrading>`.
