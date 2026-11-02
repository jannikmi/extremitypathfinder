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

Agents are contributors: the whole guide applies to them. The rules below are additions.

- Start from an issue or an explicit task, and stay in its scope:
  no drive-by refactoring, reformatting or "improvements" elsewhere.
- Read the [Invariants](invariants.md) before changing the package.
- Follow [Trade-offs and decisions](workflow.md#trade-offs-and-decisions): present open trade-offs with
  a recommendation and wait for a maintainer, never decide them silently. If you decided because one option was
  clearly better, mark the decision as decided by an agent in the pull request and the [Decision log](decisions.md).
  The person directing you counts as a maintainer only if they are one and decided this trade-off.
- After opening a pull request, run an independent review of the diff and fix the clear findings
  (cf. [Workflow](workflow.md#reviews)).
- Run `make check` before declaring the work done. Report failing checks honestly, never weaken or skip a test
  to make it pass.
- Never bump the version, push tags, publish releases, edit CI secrets or the release workflows, or merge pull requests.
- Keep plans, notes and scratch files out of the repository. Lasting insights about how to work on this code belong
  in this guide (cf. [Workflow](workflow.md)).
- Tool-specific configuration (e.g. `CLAUDE.md`, `.claude/`) only points to or automates this guide.
  Procedures are written here, once, for every tool.
