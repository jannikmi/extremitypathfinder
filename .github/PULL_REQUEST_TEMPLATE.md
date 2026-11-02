## What and why

<!-- What does this change and why? Link the issue, e.g. "closes #123". -->

## Trade-off

<!--
Larger changes only (cf. the "Trade-offs and decisions" section of the workflow guide), otherwise delete this section.
Options considered (with advantages and disadvantages), the recommendation, the decision and who made it:
a maintainer (link the comment), or the contributor or an agent because one option was clearly better (no maintainer decision).
Link the entry in the decision log (docs/contributing/decisions.md) and where the reasoning is documented.
-->

## Checks

- [ ] `make check` passes (pre-commit hooks and tests)
- [ ] tests added or updated (a regression test for a bugfix)
- [ ] tested with and without Numba, if numerical code changed
- [ ] before/after benchmark numbers included, if this is a performance change
- [ ] changelog entry in the `Unreleased` section of `CHANGELOG.rst`, if user-facing
- [ ] documentation updated, if behaviour changed

## AI assistance

- [ ] An AI tool wrote a substantial part of this change. I have reviewed and understood every line
  (cf. the [AI-assisted contributions](https://extremitypathfinder.readthedocs.io/en/latest/contributing/ai-assisted-contributions.html) guide).
