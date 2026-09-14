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



* Check out the instructions and notes in ``publish.py``
* Install ``tox`` and run the tests:

   .. code-block:: sh

       $ pip install tox
       $ tox

   The tox configuration in ``pyproject.toml`` tests Python 3.12–3.14
   with minimum and latest dependencies, with and without Numba. During
   development, select a single environment:

   .. code-block:: sh

       $ tox -e py312-numba


* Commit your changes and push your branch to GitHub:

   .. code-block:: sh

       $ git add .
       $ git commit -m "Your detailed description of your changes."
       $ git push origin name-of-your-bugfix-or-feature

* Submit a pull request through the GitHub website. This will trigger the GitHub Actions build which runs the tests against all supported versions of Python.


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



.. _Github Issues: https://github.com/MrMinimal64/extremitypathfinder/issues
