---
name: unblock
description: Unblock and re-rank the issues of extremitypathfinder, one at a time, until the maintainer stops. Picks the most important blocked issue, compiles or measures the missing information, prepares options with trade-offs and a recommendation if a decision is needed, asks the maintainer, updates the issue and re-ranks the priorities. Use when asked for blocked issues, open or pending decisions, re-prioritising, before starting work that waits for a blocked issue, or via /unblock.
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

## Compile or measure (step 3)

Use the repository commands of `AGENTS.md` (`uv run pytest`, `uv run python -m cProfile`, the benchmark commands of
`docs/contributing/performance.md`). A cloud session runs on a virtual machine: label its numbers as indicative.
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

Make the changes without asking, with `add_issue_comment`, `issue_write` (labels, body) and `sub_issue_write`,
and show each one in the chat: the comment as posted with its link, labels and links as "old → new".
Comments end with the attribution footer of the session.

## Report (step 8)

End with a table: issue, what blocked it, answer, changes made, still open. Do not start implementing.
