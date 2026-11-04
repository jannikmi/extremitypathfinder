# Implement the next issue

Pick the most important issue that is ready to be implemented, claim it, implement it together with the issues
that are best shipped with it, and see the pull request through review.
Several contributors and agents can do this at the same time: [claims](#claims) keep them off each other's issues.
Run it on request (e.g. `/implement-next` in Claude Code), optionally for a given issue.

## Procedure

1. **Rank** the open issues as in step 1 of [Unblock and re-rank](unblock-and-rerank.md),
   with the rules of [Prioritising issues](workflow.md#prioritising-issues).
2. **Pick** the highest-ranked issue that is ready. An issue is ready if it is
   - not blocked in the sense of step 2 of [Unblock and re-rank](unblock-and-rerank.md): no open decision,
     no missing information, every "blocked by" issue closed. For an issue with open sub-issues, consider the sub-issues;
   - not claimed by anyone else and not closed by an open pull request;
   - possible without actions only a maintainer can take (accounts, credentials, publishing, releases).
     Treat such an issue as blocked by the maintainer.

   For a given issue, check the same and stop with the reason if it is not ready.
   If no issue is ready, report the most important blocked ones and recommend [Unblock and re-rank](unblock-and-rerank.md).
3. **Bundle** ready issues that are cheaper to ship together: they change the same code, tests or documentation,
   one prepares or simplifies the other, or they are small changes of the same kind that one review covers.
   Split only unrelated changes that would interfere with each other, e.g. one could be held up, rejected or reverted
   without the other, or they need a decision the other does not. The size of a pull request is no reason to split.
4. **Claim** every issue of the bundle. Drop an issue that someone else claimed first from the bundle;
   if it was the picked issue, release the other claims and go back to step 2.
5. **Plan:** read the [Invariants](invariants.md), the [Codebase map](codebase-map.md) and each issue with its comments
   and links. For a larger change not yet decided in the issue, follow
   [Trade-offs and decisions](workflow.md#trade-offs-and-decisions): decide if one option is clearly better,
   otherwise ask a maintainer. If no maintainer can answer now, write the options and the recommendation into the issue
   (as in steps 4 and 6 of [Unblock and re-rank](unblock-and-rerank.md)), release its claim and continue without it;
   with an empty bundle, go back to step 2.
6. **Implement** with the rules of this guide: tests (a regression test that fails before each fix),
   results identical with and without Numba, changelog, documentation and decision log.
   Commit per issue where the changes allow it, so that reviewers can follow them.
   Open a draft pull request after the first commit, with "closes #…" for every issue of the bundle.
7. **Check:** `make check`, the tox environments with and without Numba if the package changed,
   the strict documentation build if the documentation changed, benchmarks for performance changes
   (cf. [Testing](testing.md), [Performance](performance.md)).
8. **Review:** run the independent review (cf. [Reviews](workflow.md#reviews)) and fix the clear findings.
   Mark the pull request ready for review when CI passes. Then fix CI failures, merge conflicts and review comments
   until the pull request is merged or closed. Merge `master` into the branch for conflicts, do not force-push.
   Findings outside the bundle become new issues.
9. **Report** the pull request, the issues it closes, the decisions made and by whom, the follow-up issues
   and what is still open.

## Claims

A claim is a comment on the issue. Comments work for every tool and person, and GitHub orders them in time,
so everyone reading them sees the same order. Agents of one maintainer often share a GitHub account:
the branch, not the account, identifies a claim.

```markdown
<!-- claim branch=<branch> -->
Claimed for implementation on branch `<branch>` by <name or agent session link>, together with #… (if bundled).
```

- **Claim** before writing code: post the comment and add the label `in progress`.
  Then read all comments of the issue again. The oldest active claim wins: if another one is older, release yours.
- **Active:** a claim is active until it is released, until its pull request is merged or closed, or until it is stale:
  no pull request one day after the claim, or no commit and no comment in its pull request for 14 days.
  To take over a stale claim, name it in your claim comment.
- **Release** when you give up an issue or your pull request is closed without merging:
  `<!-- release branch=<branch> -->` and the reason. Remove the label if no other active claim is left.
  A merged pull request closes the issue, no release is needed.
- An issue assigned to a person is claimed by that person. The label `in progress` only helps to find claimed issues:
  the comments decide.
