---
name: unblock
description: Unblock and re-rank the issues of extremitypathfinder, one at a time, until the maintainer stops. Picks the most important blocked issue, compiles, measures or audits the missing information, prepares options with trade-offs and a recommendation if a decision is needed, asks the maintainer, amends the issue and the affected repository files, and re-ranks the priorities. Use when asked for blocked issues, open or pending decisions, re-prioritising, before starting work that waits for a blocked issue, or via /unblock.
argument-hint: "[#<issue> to start with]"
---

# Unblock and re-rank

Automates [Unblock and re-rank](../../../docs/contributing/unblock-and-rerank.md).
That page is the procedure; read it and [Prioritising issues](../../../docs/contributing/workflow.md#prioritising-issues)
first. This file only adds how to run it in Claude Code. An issue number as argument is the issue to start with.

## Who is a maintainer

`mcp__github__get_me` gives the login of the GitHub token, `mcp__github__list_repository_collaborators` its role
(`maintain` or `admin` is a maintainer). The token owner need not be the person in the chat: if the two may differ, ask.

## Rank and pick (steps 1 and 2)

Use the GitHub MCP tools (`mcp__github__*`, load them with ToolSearch):

- `list_issues` (open, `fields` without `body`) for the labels.
- `issue_read` for the candidates, from the top of the ranking down: method `get` (body), `get_comments`,
  `get_sub_issues`, `get_parent`. These tools do not return GitHub's "blocked by" links: take the dependencies from
  the body and the comments ("blocked by #…", "depends on #…"), and ask the maintainer when they are unclear.
- `list_pull_requests` (open), then `pull_request_read` (`get`, `get_review_comments`) for a **Trade-off** section
  or review threads waiting for a decision.

Show the ranking as a short Markdown table in the chat.

## Compile, measure or audit (step 3)

Use the repository commands of `AGENTS.md` (`uv run pytest`, `uv run python -m cProfile`, the benchmark commands of
`docs/contributing/performance.md`). A cloud session runs on a virtual machine: label its numbers as indicative.
Run `make install` first if Numba is missing, and run audits and measurements with and without Numba where they
differ.

Perform the audit an issue needs now, not as a future task. E.g. call the function on integer, decimal grid and
near-degenerate inputs and compare it with a `fractions.Fraction` reference, or transform the test environments
(`tests/test_cases.py`) and compare the results. Check a surprising result (e.g. a reported bug) against an
independent reference before presenting it: the input generator can be wrong too.
Write scripts and outputs to the scratchpad directory, never into the repository.
Delegate wide searches to a subagent if that keeps the conversation short.

## Ask (step 5)

Per issue, one `AskUserQuestion` call (load it with ToolSearch if needed) with one question.
Put the compiled information and the full comparison in the chat message before the call, not in the dialog.

- The question stands alone: `#<number>`, the decision or what is missing in one sentence, what it unblocks.
- 2 to 4 options: the recommended one first, labelled "(Recommended)", each with its main advantage and disadvantage.
  Add "Defer" while there is room.
  "Need more information" and "stop" come as free text with the built-in "Other".
- Never group issues into one call, and do not stop after a fixed number: continue until the maintainer stops
  (dismisses the dialog or says so) or no blocked issue is left.

## Update and re-rank (steps 6 and 7)

Make the changes without asking. Do not post comments (`add_issue_comment`) for them; the only exception is a
question to someone else (e.g. the reporter), ending with the attribution footer of the session.

- **Issue:** `issue_write` (method `update`) with the amended `title`, `body` and `labels`, `sub_issue_write` for
  sub-issues. `labels` replaces all labels: pass the complete list. Read the current body right before writing it,
  and keep the sections that are still valid (e.g. an "Interested in this feature?" banner) and the original report.
  Write the decision into the body as a section "Decision (<date>)": what was decided, by whom, the rejected options
  with the main reason; put the audit or measurement in its own section with the method and the machine, and the
  reason of a priority change in a "Priority" line. Rewrite superseded plans instead of striking them out.
  The tools cannot set GitHub's "blocked by" links: write "Blocked by #<n>" into the body and list the links to set
  in the report. Show in the chat the changed sections (old → new), labels and links as "old → new".
- **Repository files:** work in a separate worktree from `origin/master` (`git worktree add` or `EnterWorktree`),
  so that work in progress stays untouched; use the session's designated branch if it has one. One commit per issue
  (`docs: record the decision of #<number>`), edited with the file tools. Run `uv run tox -e docs` and `make check`
  before each push. Open one draft pull request for the run after the first decision that changes a file
  (the template's **Trade-off** section lists the decisions); later decisions add commits to it.
  Link the issues with `Refs #<n>`, never `closes` or `fixes`: the issues stay open until implemented.
  Show each changed file in the chat.

## Report (step 8)

Run the independent review of the whole pull request diff once (`AGENTS.md` rule 12), fix the clear findings,
and push. End with a table: issue, what blocked it, answer, changes made (description, labels, links, files),
still open, the "blocked by" links to set, and the link to the pull request. Do not start implementing.
