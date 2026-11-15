# AGENTS.md

Instructions for coding agents working on this repository. Humans: see `CONTRIBUTING.md`.
This file is short on purpose. The details are in the contributing guide (`docs/contributing/`), linked below.

## Project

`extremitypathfinder` is a Python package for exact Euclidean shortest paths in 2D polygon or grid environments,
based on visibility graphs. Entry point: `PolygonEnvironment` (prepare an environment once, then query paths quickly).
Runtime dependencies: numpy, networkx. Numba is an optional accelerator. Managed with uv, flat layout (no `src/`).
Supported Python versions: `requires-python` in `pyproject.toml`.

## Commands

```sh
make install                       # uv sync --all-extras --all-groups
make check                         # pre-commit hooks + tests: run before declaring work done
uv run pytest tests/<file>.py -k <name>
uv run tox -e py312-numba          # one environment of the CI matrix (py312 = without Numba)
uv run tox -e docs                 # strict documentation build
make release-check                 # build and validate the release artifacts
```

## Repository map

- `extremitypathfinder/`: the package. Layers, each importing only from below:
  `command_line`, `plotting` → `extremitypathfinder` (PolygonEnvironment) → `queries` → `utils` → `wedges` →
  `utils_numba` → `numba` | `numba_replacements`
- `tests/`: pytest suite. `scripts/`: release tooling, docs figures, benchmarks. `benchmarks/`: comparison with pyvisgraph.
  `prototypes/`: untested dev scripts
- `docs/`: Sphinx (reStructuredText), contributing guide in Markdown (`docs/contributing/`)

## Hard rules

1. Read `docs/contributing/invariants.md` before changing the package. Do not change an invariant without an issue
   that decides it.
2. Results must be identical with and without Numba. Run affected tests in both modes.
3. Keep the module layering and the root module lean: `PolygonEnvironment` only checks the input and delegates,
   algorithms go into the modules below. New dependencies only if justified and accepted by a maintainer.
4. Queries never modify the environment or its visibility graph.
5. Every bugfix comes with a regression test that fails before the fix.
6. User-facing changes get an entry in the `Unreleased` section of `CHANGELOG.rst`.
7. Never edit `uv.lock` by hand (`make lock`). Never bump the version, push tags or publish releases unless explicitly asked.
   Editing `.github/workflows/` is allowed, but point out every edit and its reasoning in the pull request
   (`docs/contributing/workflow.md#pull-requests`). Never touch CI secrets.
8. The package is licensed under MIT only: do not copy or adapt code from other projects (`docs/contributing/invariants.md`).
9. Performance claims need before/after numbers in the issue or pull request (`docs/contributing/performance.md`).
   Static pages, docstrings and comments state results qualitatively, without measured numbers (they go stale).
10. Stay in the scope of the task: no unrelated refactoring or reformatting. Never weaken or skip tests to pass.
11. Before a larger change, present the trade-off with a recommendation. Unless one option is clearly better,
    a maintainer decides. Record and summarise it (`docs/contributing/workflow.md#trade-offs-and-decisions`).
12. After opening a pull request, run an independent review of the diff (fresh context) and fix clear findings.
13. CI must not fail because of errors in pyvisgraph or other external packages and services,
    only because of extremitypathfinder (`docs/contributing/testing.md`).
14. Amend, don't append: update issue descriptions, decision log entries, documentation and guide rules in place,
    so that they state the current state. No follow-up comments, "superseded" markers or histories: git and GitHub
    keep the history. Exceptions: `CHANGELOG.rst`, and comments that ask someone a question.

## Read next

| Task | Read |
| --- | --- |
| any change to the package | `docs/contributing/invariants.md`, `docs/contributing/codebase-map.md` |
| writing or running tests | `docs/contributing/testing.md` |
| style and linting | `docs/contributing/code-style.md` |
| prioritising issues, branches, commits, pull requests | `docs/contributing/workflow.md` |
| unblocking issues and re-ranking their priorities | `docs/contributing/unblock-and-rerank.md` |
| picking, claiming and implementing the next issue | `docs/contributing/implement-next.md` |
| documentation, figures and this contributing guide | `docs/contributing/documentation.md` |
| changelog entries | `docs/contributing/changelog.md` |
| optimising | `docs/contributing/performance.md` |
| releases (only when asked) | `docs/contributing/releasing.md` |
| rules for AI-assisted work | `docs/contributing/ai-assisted-contributions.md` |
| algorithms and design decisions | `docs/8_design.rst` |

When a correction repeats, amend the relevant guide page (or this file, if it applies to almost every change).
