# Decision log

Decided trade-offs, newest first. The log answers "was this considered before, what was decided and by whom?"
in one place, including options that were rejected, so that they are not proposed again without new arguments.
How to decide and when to add an entry: [Trade-offs and decisions](workflow.md#trade-offs-and-decisions).

Each entry is short and links to the details instead of repeating them.
In the heading, reference the issue, or the pull request once it is open:

```markdown
## <decision as a statement> (#<issue or pull request>)

- **Decided by:** a maintainer (link to the comment) | the contributor | an agent, without a maintainer decision
- **Options:** <chosen option>, rejected: <other options, each with the main reason>
- **Reason:** <one or two sentences>
- **Details:** <link to the design page section, guide page or code with the full reasoning, if any>
```

Entries are not edited after the fact, with two exceptions: a maintainer confirms a decision of an agent by replacing
its **Decided by** line with `a maintainer (link)`, and a revised decision gets a new entry while the old entry
is marked **superseded by** the new one.

## Trade-offs are decided before larger changes and recorded in this log (#179)

- **Decided by:** a maintainer (requested the rule implemented in #179)
- **Options:** consider, decide and document trade-offs, agents present a justified recommendation;
  rejected: no explicit rule (decisions made silently in the code, in particular by agents)
- **Reason:** reviewers check a decision only if they see it, and documented rejected options save repeated discussions.
- **Details:** [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)

## Decisions are recorded in this log, the reasoning where it is read (#179)

- **Decided by:** an agent, without a maintainer decision (the maintainer asked for better places "if beneficial")
- **Options:** a log with short entries linking to the reasoning on the design page, a guide page or in the code;
  rejected: only the existing pages (the decisions about the development, e.g. tooling or this process, have no home,
  "who decided" is not recorded, and finding a past decision needs searching several pages),
  full records in the log (duplicates the design page, which users read too)
- **Reason:** one place to look up past decisions and who made them, without duplicating the reasoning.
- **Details:** [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)

## Earlier decisions

Decisions made before this log have no entries. They are described with their reasoning in the
{ref}`Design Decisions <design_decisions>` section of the Engineering Design page and in the issues linked there,
for example:

- queries do not modify the environment (#136)
- own port of `networkx.astar_path()`, other searches and graph libraries rejected (#152,
  details in [Performance](performance.md#shortest-path-search))
- one concept for vertices: free wedges (#24)
