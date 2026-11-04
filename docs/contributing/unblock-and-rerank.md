# Unblock and re-rank

Issues that wait for a decision, for information or for another issue block work.
Unblock them one at a time, most important first, and update the ranking after each one
(cf. [Prioritising issues](workflow.md#prioritising-issues)).
Run this on request (e.g. `/unblock` in Claude Code) or before work that depends on a blocked issue.

1. **Rank** the open issues by their priority label, with the rules of [Prioritising issues](workflow.md#prioritising-issues):
   an issue inherits the highest priority of the issues it blocks, within a label stability comes first,
   then less effort. An open pull request that waits for a decision counts for its issue.
2. **Pick** the most important blocked issue. An issue is blocked if it needs
   - a decision: options without a maintainer answer, a change of the [invariants](invariants.md),
     a new dependency, a "Discussion" issue, a trade-off in its pull request;
   - information: a reproduction, a use case, measurements, a missing comparison or an unclear scope;
   - another issue ("blocked by"): then that issue is the one to unblock, with the inherited priority.

   Skip issues that are blocked only by work already in progress, or by a decision that was deferred
   and nothing about it changed since.
3. **Compile or measure** what is missing: read the issue, its links, the code and the documentation;
   reproduce the problem; profile or benchmark it (cf. [Performance](performance.md): before/after numbers,
   the machine, the commands). Keep scratch files out of the repository. Ask before measurements that take
   more than a few minutes (e.g. a full benchmark run). Check every fact before presenting it.
4. **Prepare the options**, if a decision is needed: the question in one sentence, the options including
   "leave it as it is" with their trade-offs (cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)),
   the recommendation and what the decision unblocks. If the information shows one option clearly better
   on all criteria that matter, say so: the maintainer confirms it like any other.
5. **Ask** a maintainer about this one issue. Possible answers: an option, "defer", "need more information"
   (back to step 3 with what is missing) or "stop".
6. **Update** the issue: a comment with the compiled information and, if decided, the decision and the rejected options.
   Change its scope, sub-issues or "blocked by" links if the decision changes them. Record the decision in the
   [Decision log](decisions.md) only in the pull request that implements it, and only if it meets the criteria there.
7. **Re-rank:** the decision or the new information can change the effort, the risk, the demand or what an issue blocks.
   Propose the new priority labels for this issue and for the issues it blocks or that block it, each with the reason.
8. **Repeat** from step 2 with the new ranking, until no blocked issue is left or the maintainer stops.
   Then report each issue handled: what blocked it, the answer, what changed (comments, links, labels)
   and what is still open. Implementing a decision is a separate task.

Only a maintainer decides, changes priorities or approves the texts of steps 6 and 7. Show each comment and
each change of labels or links before making it. If the person you work with is not a maintainer, stop after step 4
for each issue and hand over the result, without posting or changing anything.
