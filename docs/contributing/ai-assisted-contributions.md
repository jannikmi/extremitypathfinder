# AI-assisted contributions

Coding agents and AI assistants are welcome tools. The same standards apply to their output as to any other code.

## For the person opening the pull request

- **You are responsible for the pull request.** You have read and understood every line and can explain it in review.
- **Disclose substantial AI assistance.** Tick the checkbox in the pull request template when an AI tool wrote
  a substantial part of the change. Naming the tool is optional. Small completions need no disclosure.
- **Verify.** State in the pull request which checks ran (`make check`, tox environments, benchmarks),
  and run them yourself, do not rely on the agent's summary.
- **Reproduce before reporting.** Do not open issues for AI-found bugs without a reproduction.

## For agents

An agent working on this repository is a contributor: the whole guide applies to it,
with the person who directs it (cf. above) being responsible for its work.
The rules below are additions for the risks specific to agents, not a separate rule set.

- Start from an issue or an explicit task, and stay in its scope:
  no drive-by refactoring, reformatting or "improvements" elsewhere.
- Read the [Invariants](invariants.md) before changing the package.
- Before implementing a larger change, prepare the trade-off with a justified recommendation and follow
  [Trade-offs and decisions](workflow.md#trade-offs-and-decisions). Unless one option is clearly better on all
  criteria that matter, present it in the issue (or to the human in charge, if there is none)
  and wait for a maintainer's decision instead of implementing. Never pick a side of an open trade-off silently.
- If you decided without a maintainer because one option was clearly better, say so explicitly in the pull request
  and record it in the [Decision log](decisions.md) as decided by an agent, so that the reviewer checks this decision
  first. A person who directs you is not a maintainer decision, unless that person is a maintainer
  and decided this trade-off.
- After opening a pull request, run an independent review of the diff and fix the clear findings
  (cf. [Workflow](workflow.md#reviews)).
- Run `make check` before declaring the work done. Report failing checks honestly, never weaken or skip a test
  to make it pass.
- Never bump the version, push tags, publish releases, edit CI secrets or the release workflows, or merge pull requests.
- Keep plans, notes and scratch files out of the repository. Lasting insights about how to work on this code belong
  in this guide (cf. [Workflow](workflow.md)).
- Tool-specific configuration (e.g. `CLAUDE.md`, `.claude/`) only points to or automates this guide.
  Procedures are written here, once, for every tool.
