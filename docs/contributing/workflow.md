# Workflow

## Before you start

- Work on an issue. For a larger change (cf. [Trade-offs and decisions](#trade-offs-and-decisions)),
  open or comment on an issue first, so that the approach can be agreed on before you write the code.
- Changes of the [invariants](invariants.md) (e.g. the numerical policy or the orientation convention)
  need an issue that decides them.

## Trade-offs and decisions

Before implementing a larger change (new behaviour, API, algorithm, data structure, dependency or anything that
touches the [invariants](invariants.md)), consider the trade-offs:

1. **List the options**, including "leave it as it is", with their advantages and disadvantages
   (correctness, performance with numbers, maintainability, API stability, dependencies, effort).
2. **Recommend one option and justify it.**
3. **Decide.** Post the trade-off and the recommendation in the issue.
   If one option is clearly better on all criteria that matter, the contributor decides and goes ahead.
   Otherwise a maintainer decides: wait for the decision before implementing.
   Changes of the invariants are always decided by a maintainer.
4. **Document the trade-off and the decision** and who made it (the contributor or a maintainer)
   where the next person will look for it:
   - design decisions about the package: the Design Decisions section of `docs/8_design.rst`
   - rejected alternatives and lessons about working on the code: the relevant page of this guide
     (e.g. the rejected search alternatives in [Performance](performance.md#shortest-path-search))
   - local decisions about a single function: a comment or the docstring next to the code

   Link the issue or pull request with the full discussion. Record rejected options too,
   so that they are not proposed again without new arguments.
5. **Summarise the trade-off in the pull request** (cf. [Pull requests](#pull-requests)), so that the reviewer can
   check the decision, not only the code.

## Branch and commits

```sh
git checkout -b name-of-your-bugfix-or-feature
```

- One concern per pull request. Do not mix a fix with refactoring, reformatting or unrelated "improvements".
- Write commit messages that say what changed and why. Prefixes like `docs:`, `fix:` or `tests:` are welcome.
- Run `make check` before you push.

## Pull requests

Push your branch and open a pull request against `master`. The template asks for:

- what changed and why, with a link to the issue
- the considered trade-off of a larger change: the options, the recommendation, the decision and who made it
  (cf. [Trade-offs and decisions](#trade-offs-and-decisions))
- the checks you ran (`make check`, tox environments, benchmarks for performance changes)
- a changelog entry for user-facing changes (cf. [Changelog](changelog.md))
- documentation updates for changed behaviour (cf. [Documentation](documentation.md))
- whether AI assistance was used substantially (cf. [AI-assisted contributions](ai-assisted-contributions.md))

CI (`.github/workflows/build.yml`) runs the pre-commit hooks, the tests on all supported Python versions
with minimum and latest dependencies, with and without Numba, the strict documentation build,
the release tooling tests and the release artifact validation.

## Reviews

- **Agents review their own pull request.** After opening a pull request, an agent runs an independent review
  of the diff: a fresh session, subagent or review tool without the context of the implementation
  (e.g. `/code-review` in Claude Code). It fixes the clear findings and pushes them.
  Findings that need a decision go to the human in charge, not into the code.
- Address every review comment, either with a change or with an answer why not.
- Push follow-up commits instead of force-pushing, so that reviewers see what changed since their review.

## Keeping this guide useful

When a reviewer (or an agent's user) has to repeat the same correction, add it as a rule to the relevant page
of this guide, or to the hard rules in `AGENTS.md` if it applies to almost every change.
Delete rules that stopped mattering. Keep `AGENTS.md` short: details belong here.
