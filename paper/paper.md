---
title: "extremitypathfinder: exact Euclidean shortest paths in 2D polygon environments with reduced visibility graphs"
tags:
  - Python
  - path planning
  - shortest path
  - visibility graph
  - computational geometry
authors:
  - name: Jannik Kissinger
    orcid: 0000-0000-0000-0000 # TODO
    affiliation: 1
affiliations:
  - name: TODO
    index: 1
date: TODO
bibliography: paper.bib
---

<!--
Skeleton of the JOSS paper (issue #190). The plan and the open tasks are in the pull request that added this file.
Sections and order as required by JOSS (https://joss.readthedocs.io/en/latest/paper.html).
Length of the whole paper: 750-1750 words. Each bullet below is a content note, to be replaced by prose.
No API documentation in the paper: refer to the documentation instead.
-->

# Summary

<!-- ~150 words, for non-specialists. -->

- The problem: the shortest path between two points in a plane with polygonal obstacles
  (robots, ships, game characters, pedestrians in a building plan).
- What the package does: prepare an environment once (boundary polygon with holes, or a grid),
  then answer many shortest path queries quickly. Exact Euclidean paths, not approximations on a grid or mesh.
- How, in one sentence: only the protruding corners ("extremities") become nodes of a reduced
  visibility graph [@vinther2015pathfinding], searched with A\* [@hart1968astar].

# Statement of need

<!-- ~250 words. Problem, target audience, relation to other work. -->

- Target audience: researchers and engineers in robotics, simulation, GIS and routing (e.g. maritime),
  and game AI, who need exact shortest paths in Python with `pip install` and few dependencies.
- Use cases with many queries in a static environment: distance matrices, simulations, route planning.
- Inputs as they come from practice: touching and overlapping polygons, grids with blocked cells,
  no triangulation or navigation mesh to build first.
- Reproducibility: results are identical with and without the optional Numba accelerator [@lam2015numba].

# State of the field

<!-- ~300 words. Source: docs/9_alternatives.rst. Must contain a "build vs. contribute" justification. -->

- Visibility graphs [@lozanoperez1979; @lee1978proximity]; any-angle search on navigation meshes, Polyanya
  [@cui2017polyanya]; grid planners.
- pyvisgraph [@pyvisgraph]: the closest Python alternative. Build vs. contribute: unmaintained since 2020,
  full visibility graph (no extremity reduction), Dijkstra instead of A\*, no boundary polygon, returns invalid paths
  on some inputs. The reduced graph and the input model (boundary polygon, touching polygons) would require
  a rewrite of pyvisgraph's core; justify this in the prose.
- Polyanya: state of the art, but no Python package, and it needs a triangulation of the free space.
- VisiLibity1 (C++): visibility and shortest paths, its Python binding is not on PyPI and has to be built.
  CGAL based packages (e.g. pyvispoly): visibility polygons only, no shortest paths.
- Measured comparison with pyvisgraph (docs/7_performance.rst, #111). Polyanya not measured yet (#170).

# Software design

<!-- ~350 words. Source: docs/8_design.rst and docs/contributing/decisions.md. Trade-offs, not code structure. -->

- Preprocessing vs. query time: the visibility graph is computed once, a query only connects start and goal.
- Graph size vs. generality: only extremities, and only edges that can be part of a shortest path
  ("Property One" in @vinther2015pathfinding). Smaller graph, faster queries, more preprocessing logic.
- Visibility algorithm: Angle Range Elimination (AREA), which checks only candidate extremities and the edges
  whose angle range contains them, instead of Lee's rotational sweep with an edge tree [@lee1978proximity]
  (docs/8_design.rst).
- Numerics: `float64` with exact contacts (polygons touch only at exactly equal coordinates or a vertex exactly
  on an edge), no tolerance parameter (docs/contributing/invariants.md). Audit against exact arithmetic: #3.
- Acceleration: Numba is optional, results must be identical with and without it (tested in CI).
- Dependencies: NumPy [@harris2020numpy] and NetworkX [@hagberg2008networkx] only; flat arrays referenced by index.
- Queries never modify the prepared environment: safe for repeated queries and caching with pickle.

# Research impact statement

<!-- ~200 words. Only verified evidence: publications, external use, integrations. No aspirational statements. -->

- Use by other research groups, shown by the official code of their papers (`PolygonEnvironment` imported):
  - Chalmers (Åkesson group), multi-robot planning and navigation: @ceder2024bev (states that reference paths
    come from visibility graphs and A\*), @zhang2025future, @roselli2026scheduling, @xue2026proactive;
    also master theses and courses since 2020.
  - University of Southern Denmark, UAV search and rescue: @grontved2023decentralized (`trajallocpy`).
  - Integrations: SiTT, the traffic simulation of the ERC project STRADA (University of Trier),
    declares the package as a dependency; `ros_sgp_tools` (SGP-Tools, informative path planning) uses it.
- The papers link their code repository, but none names the package in its text
  (checked: IROS 2024 and the arXiv versions of the others): ask these groups to cite the JOSS paper.
- Direct citations in master theses:
  - @laitala2024docking (Chalmers, marine vessel docking) cites the package and uses it as its path planner;
    the follow-up thesis @persson2026docking names it in its text.
  - @gaba2022uav (MIT, UAV last-mile delivery routing): "This thesis leverages the extremitypathfinder
    Python package" for shortest paths around obstacles.
- Journal paper: @zapatarodriguez2026hierarchical (TMLR) uses the package for its oracle planner,
  "the exact shortest Euclidean path through the free space of a 2D polygonal environment".
- To check: IV 2020 (Guido), found by Google Scholar, behind the IEEE paywall.
- Former author name: citations before the name change use Jannik Michelfeit ("Michelfeit et al., 2021")
  or the former GitHub username MrMinimal64. Mention this once, so that editors can match the citations.
- Releases on PyPI since 2018, download numbers, GitHub dependents (29 repositories, 1 package in November 2026).
- Reproducible benchmark against pyvisgraph with one command (`benchmarks/`).

# AI usage disclosure

<!-- Required. Tools used, for what, how the output was verified; human authors made the core design decisions. -->

- TODO: tools and the parts of the code, tests, documentation and paper they were used for
  (since 2026, commits with `Co-Authored-By` trailers; pull requests with the AI disclosure checkbox).
- Verification: review of every change by the maintainer, test suite with and without Numba in CI,
  regression tests for every bugfix, independent validation of the benchmark paths with Shapely.
- Design decisions and the invariants were decided by the maintainer (docs/contributing/decisions.md).

# Acknowledgements

- TODO: contributors (cf. the GitHub contributors page), funding (if any).

# References
