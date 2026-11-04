# Open decisions

Decisions that wait for a maintainer (cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)) block work.
Collect them and put them to a maintainer one by one, on request (e.g. `/decisions` in Claude Code)
or before work that depends on them.

1. **Collect** the open decisions. Each one is a question no rule of this guide answers and no maintainer
   has answered yet:
   - the current task: trade-offs found while working on it, review findings that need a decision
     (cf. [Reviews](workflow.md#reviews));
   - open pull requests: an undecided or agent-decided **Trade-off** section, review threads that ask
     for a decision;
   - the [Decision log](decisions.md): entries **Decided by: an agent** that a maintainer has not confirmed;
   - open issues: changes of the [invariants](invariants.md), new dependencies, issues titled "Discussion",
     issues that compare options without a maintainer answer and issues that block others.
2. **Leave out** what is not a decision: questions answered by this guide, the issue or the code,
   and choices with one option clearly better on all criteria that matter (the contributor decides those,
   step 3 of [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Agent decisions that a maintainer
   has not confirmed stay in.
3. **Order** the decisions like the issues (cf. [Prioritising issues](workflow.md#prioritising-issues)).
   A decision inherits the highest priority of the work it blocks. Within a priority, decisions that block others
   come first.
4. **Prepare** each one: the question in one sentence, links to its sources, the options including
   "leave it as it is" with their advantages and disadvantages, the recommendation and what the decision unblocks.
   Check the facts (code, numbers, links) before presenting them. Merge duplicates found in several places.
5. **Ask** a maintainer one decision at a time, in this order, without a limit, until all are asked
   or the maintainer stops. Accept "defer" and "need more information" as answers:
   the decision stays open, with what is missing noted.
6. **Record** each decision where the workflow expects it (step 4 of
   [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)): the issue or pull request (a comment with
   the decision and the rejected options), and the [Decision log](decisions.md) in the pull request that implements it.
   Confirming an agent decision changes its **Decided by**, in the open pull request about the same work
   or else in a small pull request of its own. Show the maintainer the text before posting it.
   Do not change labels, "blocked by" links or the scope of an issue unless the maintainer asks for it.
   If the person you are working with is not a maintainer, record nothing: the list from step 4 is the result.
7. **Report** what was decided, deferred and still open. Implementing a decision is a separate task.
