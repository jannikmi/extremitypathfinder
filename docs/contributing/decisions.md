# Decision log

Decided trade-offs of the package, its benchmarks and releases, newest first, including the rejected options
(cf. [Trade-offs and decisions](workflow.md#trade-offs-and-decisions)). Entry format:

```markdown
## <decision> (#<issue or pull request>)

- **Decided by:** a maintainer (link) | the contributor | an agent
- **Rejected:** <options, each with the main reason>
- **Details:** <link to the full reasoning, if any>
```

Record only decisions that outlive their pull request and whose rejected options are likely to be proposed again,
in the final state of the pull request. Leave out details fully explained on the page they affect.
Leave out decisions whose outcome is documented elsewhere and that are not important enough to be remembered,
e.g. the scope of a feature (a documentation recipe now, a parameter later on demand): the issue records them.

Keep the entries correct when the referenced content moves or changes.
A maintainer confirms an agent decision by changing **Decided by**.
Amend the entry of a revised decision instead of adding a new one:
the log states the current decisions (git keeps the history).
Decisions about how we work on the project are in the [process decision log](decisions-process.md).
Earlier decisions are described in the {ref}`Design Decisions <design_decisions>` and the issues linked there.

## The distance matrix searches the graph with A* per pair, like single queries (#175)

- **Decided by:** a maintainer
- **Rejected:** one Dijkstra search per start point (stopped when the graph nodes visible from all goals are settled).
  It gave identical lengths, but it is a second search algorithm to maintain, paths would need their own
  tie-breaking to equal those of `find_shortest_path()` (#237), and it was not consistently faster.
  Measured on the environments of `prototypes/distance_matrix_profile.py` (random points, with Numba;
  without Numba the same pattern), the time of `distance_matrix()` with Dijkstra / A* per pair:

  | vertices | 20 points | 100 points |
  | ---: | ---: | ---: |
  | 104 | 27 / 27 ms | 209 / 373 ms |
  | 328 | 109 / 66 ms | 715 / 1,036 ms |
  | 904 | 554 / 173 ms | 2,932 / 2,895 ms |

  Dijkstra settles most of the graph to reach goals all over the environment, A* only searches towards one goal.
  A* is slower only for many points in small environments (up to 1.8 times).
- **Consequences:** `distance_matrix()` and `search()` share the graph search (`_search_graph()`),
  so their paths and lengths are identical by construction, and returning paths (#237) needs no new search.
  The speed-up comes from computing the visibility once per point. Revisit Dijkstra only if many-to-many queries
  with many points in small environments become a bottleneck for users.

## Own A* search instead of the networkx port, expanding a node again when its distance improves (#157)

- **Decided by:** a maintainer ([#157](https://github.com/jannikmi/extremitypathfinder/issues/157): replacing the port, MIT only); an agent (no closed set, with #162 in mind)
- **Rejected:** keeping the port (BSD-3-Clause next to MIT); a closed set (no measured gain, wrong paths with heuristics
  that are only admissible, e.g. custom costs of #162); skipping neighbours, earlier termination and a precomputed
  adjacency structure (no gain, or only valid for Euclidean costs).
- **Details:** {ref}`Design Decisions <design_decisions>`, [Performance](performance.md#shortest-path-search)

## AREA stays the visibility algorithm, others are added next to it (#224)

- **Decided by:** a maintainer
- **Rejected:** triangular expansion, Polyanya or continuous Dijkstra instead; an angular sweep, also only for
  the worst case of AREA (#216: slower on the measured maps, no fitting package, none works with Numba).
- **Details:** {ref}`Design Decisions <design_decisions>`, `prototypes/sweep_tree_overhead.py` (#216)

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
