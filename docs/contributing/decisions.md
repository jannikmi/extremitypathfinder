# Decision log

Decided trade-offs, newest first, including the rejected options
(cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Entry format:

```markdown
## <decision> (#<issue or pull request>)

- **Decided by:** a maintainer (link) | the contributor | an agent
- **Rejected:** <options, each with the main reason>
- **Details:** <link to the full reasoning, if any>
```

Record only decisions that outlive their pull request and whose rejected options are likely to be proposed again.
Record the final state of a pull request, not the decisions it revised along the way.
Leave out details that are fully explained on the page they affect.

Keep the entries correct: update links and wording when the referenced content moves or changes.
A maintainer confirms an agent decision by changing **Decided by**.
A decision revised by a later pull request gets a new entry that marks the old one **superseded**.
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## Contributing pages have a configurable word limit and must be referenced

- **Decided by:** a maintainer (the limit, justified exceptions, amending, splitting, references);
  an agent (counting like `wc -w`, the toctree not counting as a reference, the configuration in `pyproject.toml`)
- **Rejected:** counting lines or characters (depend on line wrapping and language); excluding code blocks
  (they cost reading time and agent context too); counting the hidden toctree as a reference
  (every page is in it, the check would never fail).
- **Details:** [Size and structure of this guide](documentation.md#size-and-structure-of-this-guide)

## Blocked issues are unblocked and re-ranked by a guide page and the `/unblock` skill (#188)

- **Decided by:** a maintainer
- **Rejected:** leaving it as it is (blocked issues and open decisions stay spread over issues and pull requests);
  the procedure only in the Claude Code skill (other tools and humans would not find it);
  a `decision needed` label (needs triage by a maintainer and finds no missing information);
  asking in batches or up to a limit (the maintainer decides when to stop);
  handling blocking decisions before all others (they inherit the priority of the work they block instead);
  approving each comment and label change separately (slows down every issue; showing the changes is enough to check
  and revert them); changes without showing them (the maintainer could not check them).
- **Details:** [Unblock and re-rank](unblock-and-rerank.md)

## The 2×2 system in `_lies_behind_inner()` is solved in closed form, SciPy is dropped from the numba extra (#185)

- **Decided by:** a maintainer ([#185](https://github.com/jannikmi/extremitypathfinder/issues/185))
- **Rejected:** leaving `numpy.linalg.solve` (LAPACK) and SciPy: Numba compiles it only with SciPy installed
  (about 110 MB, more memory in every process with Numba), two different LAPACK builds with and without Numba,
  slower without Numba, and not exact on integer coordinates.
- **Details:** the measurements in #185 and its pull request, the comment in `utils_numba._lies_behind_inner()`

## Benchmarks against pyvisgraph on Natural Earth world maps of increasing size (#165)

- **Decided by:** a maintainer
- **Rejected:** GSHHG data (LGPL and less accessible); a separate query set per size (latencies not comparable
  across sizes); measuring memory with `tracemalloc` (distorts the timings or needs a second, much slower build);
  a separate run without Numba (the results would mix two runs).
- **Details:** [Benchmark method](../7_benchmark_method.rst)

## The benchmarks do not run in CI, and CI never fails because of external packages (#165)

- **Decided by:** a maintainer
- **Rejected:** a `--quick` smoke test in CI: it fails on errors of pyvisgraph, dependency installations or downloads,
  which are not errors of this package. `make benchmark` runs the smoke test locally.
- **Details:** [Testing](testing.md#ci-and-external-packages)

## Published benchmark results come from one full run, whose raw results are committed (#165)

- **Decided by:** a maintainer
- **Rejected:** rerunning only the changed package (the results would mix two runs: conditions, commit);
  committing only the tables and figures (the published numbers cannot be checked or rendered again without a rerun
  of hours on that machine); a release asset (separated from the commit that produced the docs).
- **Details:** [Running the benchmarks](performance.md#running-the-benchmarks)

## Trade-offs are decided before larger changes and recorded in a short log (#179)

- **Decided by:** a maintainer (the rule); an agent (the form of the log)
- **Rejected:** no explicit rule (decisions, in particular by agents, stay invisible to reviewers);
  only the existing pages (no place for decisions about the development, "who decided" not recorded,
  past decisions spread over several pages); full records in the log (duplicate the design page).
