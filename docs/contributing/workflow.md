# Workflow

## Before you start

- Work on an issue. For a larger change (cf. [Trade-offs and decisions](#trade-offs-and-decisions)),
  open or comment on an issue first, so that the approach can be agreed on before you write the code.
- Changes of the [invariants](invariants.md) (e.g. the numerical policy or the orientation convention)
  need an issue that decides them.

## Prioritising issues

Open issues carry one of the labels `priority: high`, `priority: medium` and `priority: low`.
Pick work from `priority: high` down. Maintainers label new issues when triaging them.
Contributors can suggest a priority or a dependency in a comment. The criteria are:

1. **Stability:** correct results, no crashes, a reliable installation and no regressions for existing users.
2. **Popularity:** easier to find, install, adopt and cite, or one common reason less to choose another package.

Combine both with the effort and the risk of breaking something (core algorithms, the [invariants](invariants.md))
into one label. Within a label, stability issues come first, then the ones with less effort.

- **Demand counts:** a thumbs-up reaction, a use case in a comment or a report by a user raises the priority.
- **An issue inherits the highest priority of the issues it blocks,** e.g. a benchmark that other work waits for.
- **Decision issues before implementations:** decide the issues that change other issues
  (e.g. replacing a dependency) before implementing those.
- **Large changes of the core** (new engines, optimisations) stay low until a benchmark or a user shows the need,
  unless they block a higher-priority issue.

Maintainers record the dependencies on GitHub, not in a file: "blocked by" for an issue that must be done first,
sub-issues for the parts of a larger issue. They update the labels when an issue changes its scope or what it blocks.

## Trade-offs and decisions

A larger change is new behaviour, API, algorithm, data structure or dependency, or anything that touches
the [invariants](invariants.md). Before implementing one:

1. **List the options**, including "leave it as it is", with their advantages and disadvantages
   (correctness, performance with numbers, maintainability, API stability, dependencies, effort).
2. **Recommend one** and justify it in the issue.
3. **Decide.** If one option is clearly better on all criteria that matter, the contributor decides.
   Otherwise, and always for the invariants, a maintainer decides before anything is implemented.
4. **Record** the decision in the [Decision log](decisions.md), in the pull request that implements it.
   The full reasoning goes where it is read: the {ref}`Design Decisions <design_decisions>` (package design),
   the relevant page of this guide (working on the code) or a comment next to the code (a single function,
   no log entry needed).
5. **Summarise** the trade-off in the pull request, so that the reviewer checks the decision, not only the code.

### Clearing open decisions

Decisions that wait for a maintainer block work. Collect them and put them to a maintainer in batches,
on request (e.g. `/decisions` in Claude Code) or before work that depends on them.

1. **Collect** the open decisions. Each one is a question no rule of this guide answers and no maintainer
   has answered yet:
   - the current task: trade-offs found while working on it, review findings that need a decision
     (cf. [Reviews](#reviews));
   - open pull requests: an undecided or agent-decided **Trade-off** section, review threads that ask
     for a decision;
   - the [Decision log](decisions.md): entries **Decided by: an agent** that a maintainer has not confirmed;
   - open issues: changes of the [invariants](invariants.md), new dependencies, issues titled "Discussion",
     issues that compare options without a maintainer answer and issues that block others.
2. **Leave out** what is not a decision: questions answered by this guide, the issue or the code,
   and choices with one option clearly better on all criteria that matter (the contributor decides those,
   step 3 of [Trade-offs and decisions](#trade-offs-and-decisions)). Agent decisions that a maintainer has not confirmed
   stay in.
3. **Order** the decisions like the issues (cf. [Prioritising issues](#prioritising-issues)):
   decisions that block others first, then by the priority label of the work they block.
4. **Prepare** each one: the question in one sentence, links to its sources, the options including
   "leave it as it is" with their advantages and disadvantages, the recommendation and what the decision unblocks.
   Check the facts (code, numbers, links) before presenting them. Merge duplicates found in several places.
5. **Ask** a maintainer, a few decisions at a time, most important first. Accept "defer" and "need more information"
   as answers: the decision stays open, with what is missing noted.
6. **Record** each decision where the workflow expects it (step 4 of
   [Trade-offs and decisions](#trade-offs-and-decisions)): the issue or pull request (a comment with the decision
   and the rejected options), and the [Decision log](decisions.md) in the pull request that implements it.
   Confirming an agent decision changes its **Decided by**, in the open pull request about the same work
   or else in a small pull request of its own. Show the maintainer the text before posting it.
   Do not change labels, "blocked by" links or the scope of an issue unless the maintainer asks for it.
   If the person you are working with is not a maintainer, record nothing: the list from step 4 is the result.
7. **Report** what was decided, deferred and still open. Implementing a decision is a separate task.

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
- the trade-off of a larger change (cf. [Trade-offs and decisions](#trade-offs-and-decisions))
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
