# Process decision log

Decided trade-offs about how we work on the project (contributing guide, issues, pull requests), newest first.
Entry format and rules: [Decision log](decisions.md).

## Static typing is checked with mypy `--strict` in pre-commit and CI (#226)

- **Decided by:** a maintainer
- **Rejected:** pyright (needs Node in the pre-commit hook, less common in scientific Python packages);
  checking with both mypy and pyright (double maintenance for little gain);
  priority low (wrong annotations go unnoticed meanwhile, users get no type information).
- **Details:** [Code style](code-style.md#conventions)

## The next issue is implemented by a guide page and the `/implement-next` skill, claimed by a comment (#195)

- **Decided by:** a maintainer (bundling related issues, claims for concurrent work); an agent (comment claims
  where the oldest active one wins, a new branch per run as the claim id, the stale-claim limit)
- **Rejected:** one concern per pull request (more pull requests, reviews and conflicts for related changes);
  claiming by assignment (agents share the maintainer's account); by a label or an issue edit (the last writer wins;
  a label goes stale when a session dies); by pushing a lock branch (invisible in the issue, needs cleanup);
  a `ready` label (needs triage; readiness follows from the issue and its links).
- **Details:** [Implement the next issue](implement-next.md)

## Blocked issues are unblocked and re-ranked by a guide page and the `/unblock` skill, which performs the audits and amends the issues and files (#188, #194)

- **Decided by:** a maintainer
- **Rejected:** leaving it as it is (blocked issues and open decisions stay spread over issues and pull requests);
  the procedure only in the Claude Code skill (other tools and humans would not find it);
  a `decision needed` label (needs triage, finds no missing information);
  asking in batches or up to a limit (the maintainer decides when to stop);
  blocking decisions before all others (they inherit the priority of the work they block instead);
  approving each change separately (slow; showing the changes suffices to check and revert them);
  changes without showing them (cannot be checked);
  a comment per decision (the scope must be reconstructed from the thread, decisions reach the
  repository only with their implementation); rescoping an issue into a later audit (defers the information the
  decision needs); asking before longer audits (stalls the run, a heads-up lets the maintainer intervene).
- **Details:** [Unblock and re-rank](unblock-and-rerank.md)

## Trade-offs are decided before larger changes and recorded in a short log (#179)

- **Decided by:** a maintainer (the rule); an agent (the form of the log)
- **Rejected:** no explicit rule (decisions, in particular by agents, stay invisible to reviewers);
  only the existing pages (no place for development decisions, "who decided" not recorded,
  past decisions spread out); full records in the log (duplicate the design page).
