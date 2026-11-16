# Benchmark decision log

Decided trade-offs about the benchmarks and releases, newest first.
Entry format and rules: [Decision log](decisions.md).

## A release requires up-to-date benchmark results, checked when it is prepared, on its pull request and when it is tagged (#191)

- **Decided by:** a maintainer (the rule and closing the gap of manual version changes); an agent (how it is checked)
- **Rejected:** running the benchmarks in CI (CI must not fail because of external packages, cf.
  [Testing](testing.md#ci-and-external-packages); shared runners give noisy timings on changing hardware);
  checking every pull request (code still changes until the release);
  only checking when the release is prepared (misses manual version changes and later merges);
  comparing `pyproject.toml` and `uv.lock` as files (they change with every release, the measured dependency
  versions are compared instead).
- **Details:** [Releasing](releasing.md#benchmark-results)

## Published benchmark results come from one full run, whose raw results are committed (#165)

- **Decided by:** a maintainer
- **Rejected:** rerunning only the changed package (mixes two runs: conditions, commit);
  committing only the tables and figures (the numbers cannot be checked or rendered again without a rerun
  of hours on that machine); a release asset (separated from the commit that produced the docs).
- **Details:** [Running the benchmarks](performance.md#running-the-benchmarks)
