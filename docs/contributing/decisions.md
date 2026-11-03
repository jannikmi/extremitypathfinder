# Decision log

Decided trade-offs, newest first, including the rejected options
(cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Entry format:

```markdown
## <decision> (#<issue or pull request>)

- **Decided by:** a maintainer (link) | the contributor | an agent
- **Rejected:** <options, each with the main reason>
- **Details:** <link to the full reasoning, if any>
```

Entries are not edited, except that a maintainer confirms an agent decision by changing **Decided by**,
and a revised decision gets a new entry that marks the old one **superseded**.
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## The 2×2 system in `_lies_behind_inner()` is solved in closed form, SciPy is dropped from the numba extra (#185)

- **Decided by:** a maintainer ([#185](https://github.com/jannikmi/extremitypathfinder/issues/185))
- **Rejected:** leaving `numpy.linalg.solve` (LAPACK) and SciPy: Numba compiles it only with SciPy installed
  (about 110 MB, more memory in every process with Numba), two different LAPACK builds with and without Numba,
  slower without Numba, and not exact on integer coordinates.
- **Details:** the measurements in #185 and its pull request, the comment in `utils_numba._lies_behind_inner()`

## Trade-offs are decided before larger changes and recorded in this log (#179)

- **Decided by:** a maintainer
- **Rejected:** no explicit rule: decisions, in particular by agents, stay invisible to reviewers.

## A short log linking to the reasoning, which stays where it is read (#179)

- **Decided by:** an agent
- **Rejected:** only the existing pages: no place for decisions about the development, "who decided" not recorded,
  past decisions spread over several pages. Full records in the log: duplicate the design page.
