

===============
Getting started
===============


Installation
------------

Installation with pip:


.. code-block:: console

    pip install extremitypathfinder


Installation with Numba for a significant speedup, with the tradeoff of a larger installation footprint and slight initial compilation time (until caching kicks in):

.. code-block:: console

    pip install "extremitypathfinder[numba]"



Supported versions and dependencies
-----------------------------------

Python >=3.12,<4 is accepted; CI currently tests CPython 3.12–3.14.
Dependencies are NetworkX 3.x and NumPy >=2.3.3,<3.
Python <3.12 and NumPy <2.3.3 are no longer supported.

The Python and NumPy floors stay within the September 2026 downstream support
window in `NEP 29 <https://numpy.org/neps/nep-0029-deprecation_policy>`__,
now superseded by `SPEC 0 <https://scientific-python.org/specs/spec-0000/>`__.
Both policies exclude Python 3.11 by this date. This compatibility window is
narrower than CPython's security support lifetime. Review the floors for future
releases; SPEC 0 recommends dropping Python 3.12 in October 2026.
The dependency minimums are shared across all tested Python versions.
These recommendations do not promise upstream bug fixes for every included
NumPy release.
CI runs the full suite with minimum and latest compatible dependencies,
both with and without the ``numba`` extra, on every supported Python version.

The optional ``numba`` extra installs Numba >=0.63,<1 and SciPy >=1.16.1,<2.
SciPy supplies the compiled
linear algebra routines. Pip selects compatible versions; Numba may constrain
NumPy more tightly than the ordinary installation.
Acceleration depends on Numba/llvmlite platform support and adds installation
size and initial compilation time. Standard, GIL-enabled CPython is tested;
free-threaded builds and alternative interpreters are not covered by CI.

See ``pyproject.toml`` for the complete dependency constraints.



Basics
------



.. code-block:: python

    from extremitypathfinder import PolygonEnvironment

    environment = PolygonEnvironment()
    # counter clockwise vertex numbering!
    boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # clockwise numbering!
    list_of_holes = [
        [
            (3.0, 7.0),
            (5.0, 9.0),
            (4.5, 7.0),
            (5.0, 4.0),
        ],
    ]
    environment.store(boundary_coordinates, list_of_holes, validate=False)
    start_coordinates = (4.5, 1.0)
    goal_coordinates = (4.0, 8.5)
    path, length = environment.find_shortest_path(start_coordinates, goal_coordinates)



All available features of this package are explained :ref:`HERE <usage>`.
