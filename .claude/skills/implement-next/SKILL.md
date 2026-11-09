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
A prompt handed off by another session names the bundle and its claimed branch: check that its claims are still
the oldest active ones and continue with step 5 on that branch.
One run implements one bundle and ends when its pull request is ready for review; for continuous work, run it in
a loop (`/loop /implement-next`). PR events of earlier runs wake the session in between.

## Rank and pick (steps 1 and 2)

Gather the issues, their dependencies and the open pull requests as in "Rank and pick" of the
[unblock skill](../unblock/SKILL.md), with the GitHub MCP tools (`mcp__github__*`, load them with ToolSearch).
In addition, for each candidate from the top down:

- `issue_read` `get`: the assignees and `closed_by_pull_requests` (an open one means not ready).
- all pages of `issue_read` `get_comments`: the comments with `<!-- claim branch=… -->` and
  `<!-- release branch=… -->` markers, ordered by comment id. A release cancels the earlier claims of its branch.
  Find a claim's pull request with `list_pull_requests` (`head: "<owner>:<branch>"`, `state: all`).

Show the top of the ranking as a short Markdown table (issue, priority, ready or what blocks it, claimed by)
and the bundle with one sentence per issue on why it belongs in it.

## Claim (step 4)

- Branch: a new branch for each run, `implement-<issue numbers>-<short slug>` from the latest `origin/master`,
  even if the environment designates a branch (that one may carry the open pull request of an earlier run).
  If pushing to a new branch is refused, stop with that reason instead of reusing a branch.
- Post the claims with `add_issue_comment`, with the agent session link. Wait a few seconds (`sleep 5`),
  read the comments again and keep only the issues where this run's claim is the oldest active one.
  Post the release on every issue dropped. Comments end with the attribution footer of the session.
- Do not change labels or assignees for a claim.

## Fresh session or this one (after step 4)

If the conversation already holds earlier work, check whether a fresh session suits the implementation better.
Skip this with `--dry-run`.

- **Fresh** if most of the context is unrelated to the bundle (other issues, earlier runs, long explorations):
  it costs on every turn, gets compacted and can bring stale assumptions into the work.
  In a loop, this holds from the second run on: the looping session then only picks, claims and hands off,
  and each fresh session watches its own pull request.
- **This session** if its context is about the bundle, e.g. the issue was just unblocked or investigated here.
  A decision a maintainer made only in this chat is a reason to stay, unless it is written into the issue first:
  a fresh session gets its prompt from a peer, not from the maintainer.

Decide by yourself, also with a person in the chat, and state the decision with a sentence why.
To hand off, keep the claims of step 4 and start the session with `create_session` (load it with ToolSearch),
`outcome_branch` set to the claimed branch and a prompt with `/implement-next`, the bundle, the claimed branch
and what it needs from here (findings, links, decisions with who made them and where they are recorded).
Report the session and end this run. Without `create_session` (e.g. a local CLI), give the person that prompt
to paste into a new session and end the run; without a person, continue here.

## Plan and decide (step 5)

A maintainer is identified as in the unblock skill. If one is in the chat, ask with `AskUserQuestion`
like step 5 of the unblock skill: one question per decision, the comparison in the chat message before the call.
In a run without a person (scheduled or looped), never wait: follow the page (decide if clearly better,
otherwise write the options and the recommendation into the issue and continue without it).
Unlike `/unblock`, this posts options without asking, but never a decision a maintainer must make.

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
