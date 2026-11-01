# Workflow

## Before you start

- Work on an issue. For anything larger than a small fix, open or comment on an issue first,
  so that the approach can be agreed on before you write the code.
- Changes of the [invariants](invariants.md) (e.g. the numerical policy or the orientation convention)
  need an issue that decides them.

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
- the checks you ran (`make check`, tox environments, benchmarks for performance changes)
- a changelog entry for user-facing changes (cf. [Changelog](changelog.md))
- documentation updates for changed behaviour (cf. [Documentation](documentation.md))
- whether AI assistance was used substantially (cf. [AI-assisted contributions](ai-assisted-contributions.md))

CI (`.github/workflows/build.yml`) runs the pre-commit hooks, the tests on all supported Python versions
with minimum and latest dependencies, with and without Numba, the strict documentation build,
the release tooling tests and the release artifact validation.

## Reviews

- Address every review comment, either with a change or with an answer why not.
- Push follow-up commits instead of force-pushing, so that reviewers see what changed since their review.

## Keeping this guide useful

When a reviewer (or an agent's user) has to repeat the same correction, add it as a rule to the relevant page
of this guide, or to the hard rules in `AGENTS.md` if it applies to almost every change.
Delete rules that stopped mattering. Keep `AGENTS.md` short (about 150 lines at most): details belong here.
