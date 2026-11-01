=======================
Contribution Guidelines
=======================

Contributions are welcome, and they are greatly appreciated! Every little bit
helps, and credit will always be given.

You can contribute in many ways:

Types of Contributions
----------------------

Report Bugs
~~~~~~~~~~~

Report bugs via `Github Issues`_.

If you are reporting a bug, please include:

* Your version of this package, python and Numba (if you use it)
* Any other details about your local setup that might be helpful in troubleshooting, e.g. operating system.
* Detailed steps to reproduce the bug.
* Detailed description of the bug (error log etc.).


Fix Bugs
~~~~~~~~

Look through the GitHub issues for bugs. Anything tagged with "bug" is open to whoever wants to implement it.

Implement Features
~~~~~~~~~~~~~~~~~~

Look through the GitHub issues for features. Anything tagged with "help wanted"
and not assigned to anyone is open to whoever wants to implement it - please
leave a comment to say you have started working on it, and open a pull request
as soon as you have something working, so that GitHub Actions starts building it.

Issues without "help wanted" generally already have some code ready in the
background (maybe it's not yet open source), but you can still contribute to
them by saying how you'd find the fix useful, linking to known prior art, or
other such help.

Write Documentation
~~~~~~~~~~~~~~~~~~~

Probably for some features the documentation is missing or unclear. You can help with that!


Submit Feedback
~~~~~~~~~~~~~~~

The best way to send feedback is to file an issue via `Github Issues`_.

If you are proposing a feature:

* Explain in detail how it would work.
* Keep the scope as narrow as possible, to make it easier to implement. Create multiple issues if necessary.
* Remember that this is a volunteer-driven project, and that contributions  are welcome :)


Get Started!
------------

Ready to contribute? Here's how to set up this package for local development.

*  Fork this repo on GitHub.
*  Clone your fork locally

* To make changes, create a branch for local development:

   .. code-block:: sh

       $ git checkout -b name-of-your-bugfix-or-feature

* The project is managed with `uv <https://docs.astral.sh/uv/>`__.
  Install the package with all extras and development dependencies into a local virtual environment:

   .. code-block:: sh

       $ make install  # uv sync --all-extras --all-groups

* Check out the instructions and notes in ``publish.py``
* Install ``tox`` and run the tests:

   .. code-block:: sh

       $ pip install tox
       $ tox

   The tox configuration in ``pyproject.toml`` tests Python 3.12–3.15
   with minimum and latest dependencies, with and without Numba. During
   development, select a single environment:

   .. code-block:: sh

       $ tox -e py312-numba

   The release tooling (``scripts/release.py``) needs uv and is tested in a separate environment,
   which also runs in CI:

   .. code-block:: sh

       $ tox -e release


* Commit your changes and push your branch to GitHub:

   .. code-block:: sh

       $ git add .
       $ git commit -m "Your detailed description of your changes."
       $ git push origin name-of-your-bugfix-or-feature

* Submit a pull request through the GitHub website. This will trigger the GitHub Actions build which runs the tests against all supported versions of Python.


Performance
-----------

Measure before and after optimising, and include the numbers in the pull request:

* ``make benchmark`` (``benchmarks/``) compares the package with pyvisgraph on world maps of increasing size,
  validates all paths and renders the tables and figures of the performance section of the documentation.
  The full run takes hours: run it on an otherwise idle machine and state the machine.
  ``make benchmark ARGS="--quick"`` is a smoke test of a few seconds (also run in CI).
  Requires the ``benchmark`` dependency group (``uv sync --all-extras --group benchmark``).
* ``scripts/speed_benchmarks.py`` measures environment preparation and queries on the small test environments
  with the same harness (``uv run --group benchmark python scripts/speed_benchmarks.py``).
* ``prototypes/large_env_query_profile.py`` profiles queries on a large, configurable environment
  and compares the query time with the time a copy of the visibility graph would take (queries do not copy it):

   .. code-block:: sh

       $ python prototypes/large_env_query_profile.py --holes-per-side 10 --profile

``prototypes/`` contains such development scripts. They are not part of the package and not run by the tests.

Shortest path search
~~~~~~~~~~~~~~~~~~~~

``utils.astar_path()`` is a port of ``networkx.astar_path()`` that also follows the temporary edges of start and goal,
so queries neither copy nor change the visibility graph (#136).
It is under the 3-clause BSD license of networkx (``LICENSES/networkx.txt``):
keep its copyright notice above the function.
Alternatives compared in #152 and rejected, mainly for maintainability:

* ``networkx.astar_path()`` on the visibility graph with the temporary edges added and removed again:
  changes shared state during a query (not thread-safe, edges of extremities merged with start or goal must be kept)
  and was about twice as slow as the port.
* other graph libraries (scipy ``csgraph``, igraph, rustworkx): an additional dependency
  and a second graph representation (or ``environment.graph`` is no networkx graph anymore).
  Their Dijkstra searches were slower than the port (no heuristic, adding the temporary edges is expensive),
  the A* of rustworkx saved less than 1 ms per query.
* precomputed shortest paths between all extremities: memory grows quadratically
  with the number of extremities (75 MB for 2,500 extremities).

The A* search takes about 5% of the query time (900 and 2,500 extremities).
Most of the time is spent on the visibility of start and goal (``utils.find_visible()``),
so that is where optimising pays off.


Changelog
---------

``CHANGELOG.rst`` lists changes only, newest release first. Describe user-facing changes at the top
of a release section, bugfixes under a ``bugfixes:`` sub-list and purely internal changes under an
``internal:`` sub-list.
Add entries for changes that are merged but not yet released to an ``Unreleased`` section at the
top. When releasing, it becomes the ``<version> (<release date>)`` section (s. below).
CI checks that the changelog has a dated section for the version in ``pyproject.toml``.
Ideas, TODOs and planned changes do not belong in the changelog: track them as `Github Issues`_.


Release validation
------------------

Before publishing a release candidate, build and validate the exact artifacts users will install:

.. code-block:: sh

   $ make release-check

This command cleans ``dist/``, builds one wheel and one source distribution, and installs each in
an isolated virtual environment outside the source checkout. It smoke-tests package metadata, the
public API, the README example, the console command, dependency consistency, and the wheel with the
optional Numba extra. The same validation gates pull requests and runs again on the artifacts built
immediately before publishing.



Releasing
---------

Releases are automated with GitHub Actions:

1. Run the "prepare release" workflow from the Actions tab and enter ``patch``, ``minor``, ``major``
   or an explicit version. It sets the new version with ``uv version``, turns the ``Unreleased``
   changelog section into the section of the new version dated today, and pushes the branch
   ``release/<version>``. The job summary links to the pull request to open.
   Locally the same is done by ``make release-prepare BUMP=patch``.
2. Open the pull request, let CI pass and merge it.
3. The "tag release" workflow notices the new version on ``master``: it checks the changelog,
   pushes a tag exactly matching the version, and starts the build workflow on that tag.
4. The build workflow tests the tag and validates the release artifacts again. Its deploy job verifies
   that the version is not already on PyPI, uploads the validated artifacts to a draft GitHub release,
   publishes them to PyPI, and makes the GitHub release public only after the PyPI upload succeeds.

If a step fails after the tag exists, fix the cause and re-run the build workflow on the tag
(Actions tab: "build", "Run workflow", select the tag).
Pushing a tag manually still triggers the build workflow as well.



.. _Github Issues: https://github.com/MrMinimal64/extremitypathfinder/issues
