# Plan: setting up extremitypathfinder for human and AI contributors

## Goals

1. **One source of truth, many entry points.** Humans arrive via GitHub or the docs.
   Agents arrive via `AGENTS.md` (Codex, Cursor, Copilot, Gemini CLI, Aider, …) or `CLAUDE.md` (Claude Code).
   All of these point to the same set of topic files. No rule is written twice.
2. **Progressive disclosure.** Agents load the entry file on every turn, so it stays short (~100–150 lines):
   commands, a repo map, hard rules, and an index of what to read next.
   The detail lives in small topic files that are read only when the task needs them.
3. **Tool-agnostic first, tool-specific second.** Procedures are plain Markdown in `docs/contributing/`.
   Claude-specific config (`.claude/`) and other tool configs only *point* to them or automate them.
4. **Fast, deterministic feedback loops.** An agent works only as well as the checks it can run.
   It needs one quick command ("did I break anything?") and one full command (the CI equivalent).
5. **Fix the stale parts of the current guide while migrating:** the issue link points to `MrMinimal64/…`,
   `publish.py` no longer exists, and the `make clean` target has the typo `.mypyp_cache`.

## Target file tree

```
AGENTS.md                         # agent entry point (tool-agnostic, open AGENTS.md convention)
CLAUDE.md                         # "@AGENTS.md" import + the few Claude-Code-only notes
CONTRIBUTING.md                   # human entry point; GitHub links to it from issues/PRs
docs/contributing/
├── index.md                      # hub: reading paths by role ("first PR", "agent session") and by task
├── setup.md                      # uv, `make install`, pre-commit hooks, optional Numba
├── workflow.md                   # issues → branch → commits → PR → review; scope and size of PRs
├── testing.md                    # quick vs full checks, tox matrix, Numba on/off, minimum deps,
│                                 #   how to add a regression test (test_*_regressions.py, test_cases.py)
├── code-style.md                 # ruff/flake8 rules, 120 columns, type hints, docstrings, comments
├── codebase-map.md               # module → responsibility → "change X here", layering rule;
│                                 #   links to the Engineering Design page for the full rationale
├── invariants.md                 # the rules a change must not break (see below); agents read this first
├── performance.md                # benchmarks, profiling, A* port rationale (moved from CONTRIBUTING.rst)
├── documentation.md              # Sphinx build, rst vs md, docs_figures.py, README sync
├── changelog.md                  # Unreleased section, bugfixes/internal sub-lists
├── releasing.md                  # release validation + the automated release workflow
└── ai-assisted-contributions.md  # policy for agent-made changes (see below)
.github/
├── PULL_REQUEST_TEMPLATE.md      # checklist: tests, changelog, docs, benchmarks, AI disclosure
└── ISSUE_TEMPLATE/
    ├── bug_report.yml            # version, Python, Numba, minimal environment JSON, traceback
    ├── feature_request.yml
    └── config.yml                # links to docs and discussions
.claude/
├── settings.json                 # shared permission allowlist (uv run pytest, make check, git read cmds, …)
└── skills/                       # optional, added later on demand (see "Claude Code specifics")
```

Why `docs/contributing/` and not a top-level `contributing/` folder: the docs build renders the files,
so the guide is on Read the Docs as well as on GitHub. GitHub renders Markdown anywhere, so nothing is lost.

## Content of the entry points

### `AGENTS.md` (always loaded, keep it short)

- One paragraph: what the package does, the core abstraction (`PolygonEnvironment`), Python ≥ 3.12, uv-managed.
- **Commands**:
  `make install`, `make check` (new, quick), `uv run pytest tests/<file>::<test>`, `tox -e py312-numba`, `tox -e docs`.
- **Repo map**: one line per top-level directory and package module.
- **Hard rules** (one line each, link to the details):
  - results must be identical with and without Numba; run the affected tests in both modes
  - keep the module layering (no imports from higher layers), cf. the Engineering Design page
  - do not change the orientation convention or the numerical policy without an issue that decides it
  - every bugfix comes with a regression test that fails before the fix
  - user-facing change → entry in the `Unreleased` section of `CHANGELOG.rst`
  - never edit `uv.lock` by hand; never bump the version, tag, or edit the release workflows unless asked
  - keep the networkx copyright notice above `utils.astar_path()`
  - performance claims need before/after numbers from the benchmark scripts
- **Read next** table: task → topic file.

### `CLAUDE.md`

```markdown
@AGENTS.md

## Claude Code
- Shared permissions live in `.claude/settings.json`; personal overrides in `.claude/settings.local.json` (git-ignored).
```

The `@` import keeps it in sync with `AGENTS.md`. A symlink would also work, but it breaks on Windows checkouts.
Copilot, Cursor and Codex read `AGENTS.md` directly, so no extra copies are needed.
Add a tool-specific file only when a tool cannot read `AGENTS.md`, and make it a pointer.

### `CONTRIBUTING.md` (humans)

Short and welcoming: ways to contribute (bugs, fixes, features, docs, feedback),
a 5-step "first PR" path, and links to `docs/contributing/`. It replaces `CONTRIBUTING.rst`.

### `invariants.md`: the most valuable file for agents

Agents break implicit rules. This file makes them explicit and checkable. It summarises the design doc and links to it:

- the orientation convention (boundary counter-clockwise, holes clockwise; input in either direction is normalised)
- the numerical policy: double precision; contacts only for exactly equal coordinates or vertices exactly on an edge
- the touching-obstacle rules and the four free-wedge rules (`wedges.py`)
- Numba parity: `utils_numba.py` must run under `numba_replacements.py` too, and the signatures must stay Numba-typable
- queries must not mutate the visibility graph (thread safety, #136)
- the public API surface (`__init__.py`, the CLI) and the deprecation policy

### `ai-assisted-contributions.md`

- The human who opens the PR is responsible for it: they understand, review and can explain every line.
- Disclose substantial AI assistance in the PR (a checkbox in the template, no tool names required).
- Verification is mandatory: the PR states which checks ran (quick check, tox envs, benchmarks).
- Scope: one concern per PR. No drive-by reformatting or "improvements" outside the task.
- No generated issue spam or unverified bug reports; reproduce first.
- Agents never publish releases, change CI secrets or workflows, or merge PRs.
- Session hygiene: start from an issue or an explicit task, and keep notes and plans out of the repo.
  Durable learnings go into the topic files instead.

## Tooling changes

1. **`make check`**: `pre-commit run --all-files` + `uv run pytest -x -q` in the current environment.
   It is the agent's inner loop (seconds, not the full tox matrix).
   Keep `make tox` as the CI-equivalent outer loop.
2. **Move the lint configuration into `pyproject.toml`** (`[tool.ruff]`, line length 120).
   Then running `ruff` directly, from an editor, or from an agent gives the same result as pre-commit.
   Optional follow-up: replace flake8 (+ bugbear, comprehensions, tidy-imports) with the equivalent ruff rules
   (`B`, `C4`, `TID`), which leaves one linter.
3. **`[tool.pytest.ini_options]`**: `testpaths = ["tests"]`, and set `MPLBACKEND=Agg`.
   A bare `pytest` then behaves like tox and does not pick up `scripts/` or `prototypes/` by accident.
4. **Markdown in Sphinx**: add `myst-parser` to the `docs` dependency group, `docs/requirements.txt` and the tox
   `docs` env. Set `source_suffix` to `.rst` and `.md`, and enable `myst_heading_anchors`.
   `docs/5_contributing.rst` becomes a toctree over `docs/contributing/`.
   The CI docs build runs with `-W`, so it catches broken cross-references.
   Links from the topic files to repo files outside `docs/` use code literals or absolute GitHub URLs,
   because relative links outside the Sphinx source break there.
5. **Packaging**: replace `CONTRIBUTING.rst` with `CONTRIBUTING.md` (and add `AGENTS.md`) in
   `[tool.uv.build-backend].source-include`. Update the check-manifest ignore list.
   `scripts/validate_release.py` / `tests/test_validate_release.py` may assert on the sdist contents.
6. **`.gitignore`**: `.claude/settings.local.json`, `CLAUDE.local.md`.
7. **Optional, for cloud agent sessions**: a SessionStart hook (or environment setup script) that runs
   `uv sync --all-extras --all-groups` so that tests run on the first try.

## Claude Code specifics (thin layer)

- `.claude/settings.json`: allow the read-only and check commands (`uv run pytest*`, `make check`, `make test`,
  `pre-commit run*`, `git status/diff/log`). Deny `uv publish`, and pushing tags.
- Skills, added only when a procedure repeats. Each skill points to the topic file instead of copying it:
  - `regression-test`: turn a bug report into a failing test in the right test module
  - `benchmark-compare`: run the benchmarks on `master` and on the branch, then format the table for the PR
- Other agents get the same procedures from the topic files, so nothing is Claude-only except the automation.

## Migration steps (one PR, reviewable commit by commit)

1. Add myst-parser and the Sphinx config. Create `docs/contributing/` by splitting `CONTRIBUTING.rst` by topic
   (performance, changelog, release validation, releasing move almost verbatim). Fix the stale links.
2. Write the new topic files: `setup`, `workflow`, `testing`, `code-style`, `codebase-map`, `invariants`,
   `documentation`, `ai-assisted-contributions`.
3. Add `CONTRIBUTING.md` and delete `CONTRIBUTING.rst`. Update `docs/5_contributing.rst`, `docs/index.rst` and the packaging.
4. Add `AGENTS.md` and `CLAUDE.md`.
5. Tooling: `make check`, the ruff and pytest config in `pyproject.toml`, and `.gitignore`.
6. Add the GitHub PR and issue templates.
7. Add `.claude/settings.json`. Skills and the SessionStart hook come later, in separate PRs.

Verification: `make check`, `tox -e docs` (strict), `tox -e release`, `make release-check`
(the sdist contents changed), and a manual read of the rendered contributing pages on GitHub and in the HTML docs.

## Keeping it healthy

- Rule of thumb in `workflow.md`: when an agent (or a reviewer) repeats a correction, it becomes a line in the
  relevant topic file or in the hard rules. When a rule stops mattering, delete it.
- Keep `AGENTS.md` under ~150 lines. Anything longer moves to a topic file.
- Optional CI check: a link checker for the Markdown files (e.g. `lychee`, or Sphinx `linkcheck`).

## Decided

- Location: `docs/contributing/`, rendered on Read the Docs via myst-parser.

## Open decisions

1. Replace flake8 with ruff rules now, or keep both?
2. Disclosure of AI assistance in PRs: required checkbox (recommended) or optional?
3. Commit `.claude/settings.json` (shared allowlist), or leave Claude config entirely personal?
