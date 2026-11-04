---
name: implement-next
description: Implement the highest-priority issue of extremitypathfinder that is fully unblocked and not claimed by anyone else, bundled with the issues best shipped with it. Ranks the open issues, claims the picked ones on GitHub (safe with several sessions running at once), implements them with tests, changelog and documentation, opens a pull request, reviews it independently and sees it through CI and review. Use when asked to work on the next or most important issue, to pick something to implement, or via /implement-next.
argument-hint: "[#<issue> to implement] [--dry-run]"
---

# Implement the next issue

Automates [Implement the next issue](../../../docs/contributing/implement-next.md).
That page is the procedure; read it, `AGENTS.md` and [Prioritising issues](../../../docs/contributing/workflow.md#prioritising-issues)
first. This file only adds how to run it in Claude Code.

Arguments: an issue number is the issue to implement (still checked and claimed).
`--dry-run` stops after step 3 and shows the ranking, the pick and the bundle, without claiming anything.
One run implements one bundle; for continuous work, run it in a loop (`/loop /implement-next`).

## Rank and pick (steps 1 and 2)

Gather the issues, their dependencies and the open pull requests as in "Rank and pick" of the
[unblock skill](../unblock/SKILL.md), with the GitHub MCP tools (`mcp__github__*`, load them with ToolSearch).
In addition, for each candidate from the top down:

- read all pages of `issue_read` `get_comments` and evaluate the claims: comments with `<!-- claim branch=… -->`
  and `<!-- release branch=… -->` markers, ordered by creation time, and the assignees.
  Check a claim's pull request with `list_pull_requests` (`head`, `state: all`) for merged, closed or stale.
- check the open pull requests' bodies for "closes #…", "fixes #…" or "resolves #…".

Show the top of the ranking as a short Markdown table (issue, priority, ready or what blocks it, claimed by)
and the bundle with one sentence per issue on why it belongs in it.

## Claim (step 4)

- Branch: the branch this session must push to, if the environment designates one; otherwise
  `implement-<issue numbers>-<short slug>` from the latest `origin/master`.
  The agent session link goes into the claim comment.
- Post the claim with `add_issue_comment`, then add `in progress` with `issue_write`: it replaces all labels,
  so pass the labels just read by `issue_read` `get_labels` plus `in progress`.
- Read the comments again, after posting, and keep only the issues where the session's claim is the oldest active one.
- Comments end with the attribution footer of the session. Release with the same tools and the release marker.

## Plan and decide (step 5)

A maintainer is identified as in the unblock skill. If one is in the chat, ask with `AskUserQuestion`
like step 5 of the unblock skill: one question per decision, the comparison in the chat message before the call.
In a run without a person (scheduled or looped), never wait: follow the page (decide if clearly better,
otherwise record the options in the issue and continue without it).

## Implement and check (steps 6 and 7)

- Use the commands of `AGENTS.md`. With and without Numba: `uv run tox -e py312` and `uv run tox -e py312-numba`.
- Write scratch scripts and outputs to the scratchpad directory, never into the repository.
- Push with `git push -u origin <branch>`. Create the draft pull request with `create_pull_request`, filling in
  `.github/PULL_REQUEST_TEMPLATE.md` (a **Trade-off** section for every decision, with who made it),
  then `subscribe_pr_activity` for it.

## Review and follow through (step 8)

- Run `/code-review` on the diff (or a subagent without this conversation's context), fix the clear findings,
  push them, and put findings that need a decision into the pull request for the maintainer.
- When the checks of the latest commit pass, mark the pull request ready with `update_pull_request` (`draft: false`).
- PR events wake the session: fix CI failures, merge conflicts and review comments as they come.
  Never merge, approve, bump the version or edit `.github/workflows/`.

## Report (step 9)

End with: the pull request link, a table of the issues (closed by the pull request, dropped and why, claim released),
the decisions with who made them, the follow-up issues created and the checks that ran with their results.
