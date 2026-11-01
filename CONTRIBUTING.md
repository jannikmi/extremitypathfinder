# Contributing

Contributions are welcome and greatly appreciated! Every little bit helps, and credit will always be given.
This guide is for human contributors and coding agents alike.
Agents start at [`AGENTS.md`](https://github.com/jannikmi/extremitypathfinder/blob/master/AGENTS.md),
which links back to the topics below.

## Ways to contribute

- **Report bugs** via [GitHub Issues](https://github.com/jannikmi/extremitypathfinder/issues).
  Include your versions of the package, Python and Numba (if installed), your operating system,
  the steps to reproduce the bug (ideally a minimal environment) and the full error message.
- **Fix bugs**: issues labelled "bug" are open to whoever wants to fix them.
- **Implement features**: issues labelled "help wanted" and not assigned to anyone are open.
  Comment on the issue that you have started, and open a (draft) pull request early, so that CI runs.
  Issues without "help wanted" may already have code in the background. You can still help by describing
  how the feature would be useful to you or by linking to prior art.
- **Write documentation**: fix unclear or missing parts of the documentation.
- **Give feedback** by opening an issue. When proposing a feature, explain in detail how it would work and keep the scope
  narrow (split it into several issues if necessary). This is a volunteer-driven project.

## Your first pull request

1. Fork and clone the repository, then set up the development environment: [Setup](docs/contributing/setup.md).
2. Create a branch and make your change: [Workflow](docs/contributing/workflow.md).
3. Add or update tests and run the checks: [Testing](docs/contributing/testing.md).
4. Add a changelog entry for user-facing changes: [Changelog](docs/contributing/changelog.md).
5. Open a pull request and fill in the template.

## Guide

| Topic | Read it when you … |
| --- | --- |
| [Setup](docs/contributing/setup.md) | set up the development environment |
| [Workflow](docs/contributing/workflow.md) | work on an issue, open a pull request or review one |
| [Testing](docs/contributing/testing.md) | run or write tests |
| [Code style](docs/contributing/code-style.md) | write code |
| [Codebase map](docs/contributing/codebase-map.md) | look for the place to make a change |
| [Invariants](docs/contributing/invariants.md) | change anything in the package (read this first) |
| [Performance](docs/contributing/performance.md) | optimise or measure the speed |
| [Documentation](docs/contributing/documentation.md) | change the documentation or its figures |
| [Changelog](docs/contributing/changelog.md) | describe your change for users |
| [Releasing](docs/contributing/releasing.md) | validate or publish a release (maintainers) |
| [AI-assisted contributions](docs/contributing/ai-assisted-contributions.md) | use a coding agent or AI assistant |

The background of the code is in the [Engineering Design](docs/8_design.rst) page.
