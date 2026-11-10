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
    orcid: 0000-0002-1819-6975
    affiliation: 1
affiliations:
  - name: Independent Researcher, Germany
    index: 1
date: TODO
bibliography: paper.bib
---

<!--
JOSS paper (issue #190). The plan and the open tasks are in the pull request that added this file.
Sections and order as required by JOSS (https://joss.readthedocs.io/en/latest/paper.html).
Length of the whole paper: 750-1750 words. No API documentation in the paper: refer to the documentation instead.
Initial prose version, drafted by an AI tool from the earlier content notes and the documentation:
the maintainer edits it, and checks every statement, before submission.
-->

# Summary

Many applications need the shortest route between two points in a plane with obstacles:
a mobile robot in a factory hall, a vessel approaching a dock, a drone avoiding no-fly zones
or a character in a game. Among polygonal obstacles, the shortest path is a sequence of straight segments
that bends only at protruding obstacle corners.
`extremitypathfinder` is a Python package that computes such paths exactly.
The environment is described once, as a boundary polygon with polygonal holes or as a grid with blocked cells,
and the package precomputes a visibility graph that contains only the protruding corners, the *extremities*,
and only the edges that can be part of a shortest path [@vinther2015pathfinding], see \autoref{fig:reduction}.
Each query connects its start and goal to this graph and searches it with A\* [@hart1968astar].
The paths are Euclidean shortest paths in the free space, not approximations on a grid or a mesh.

![The complete visibility graph of all vertices (top) and the reduced graph of the extremities (bottom),
with the same shortest path.\label{fig:reduction}](graph_reduction.png)

# Statement of need

The package is meant for researchers and engineers in robotics, simulation, geographic information systems and
game development who need exact shortest paths in Python. It is installed with `pip` and requires only
NumPy [@harris2020numpy] and NetworkX [@hagberg2008networkx]. Numba [@lam2015numba] is an optional accelerator
with identical results, which the test suite checks in continuous integration. Shapely and GeoJSON geometries,
for example from GeoPandas, are accepted directly with an optional extra.

Typical use cases ask many queries in one static environment: reference paths for trajectory planners,
distance matrices for task allocation, oracle planners in reinforcement learning, simulations that move many agents
through the same map. Environments are therefore prepared once, and can be stored and reused.

Polygons are used as they are: they may touch, overlap and intersect, vertices may lie on edges of other polygons,
and either vertex order is accepted. Grids are converted into polygons along the borders of the blocked cells.
Contacts, points on edges and collinear vertices follow one documented set of rules; touching obstacles block paths.
No triangulation or navigation mesh has to be built first. Coordinates are planar, so geographic data is projected
first, and agents with a size are handled by inflating the obstacles, for which the documentation gives a recipe.

# State of the field

Visibility graphs are the classical exact method for Euclidean shortest paths among polygonal obstacles
[@lozanoperez1979]. Lee's rotational sweep computes the visibility graph of $n$ vertices in $O(n^2 \log n)$ time
with a balanced tree of edges [@lee1978proximity]; later algorithms are faster asymptotically. Polyanya [@cui2017polyanya] computes exact any-angle paths on a navigation mesh without
a visibility graph and is the state of the art for single queries, but it needs a triangulation of the free space
and has no Python package. TřiVis [@mikula2024trivis] (C++) computes visibility graphs by triangular expansion,
likewise on a triangulation. Grid planners such as A\* or Theta\* are simpler, but their paths are in general longer.

In Python, the closest alternative is pyvisgraph [@pyvisgraph], unmaintained since 2020. It builds the complete
visibility graph of all obstacle vertices with Lee's algorithm and searches it with Dijkstra's algorithm.
Its implementation takes $O(n^3)$ in the worst case: it keeps the edges in a sorted list instead of a tree and tests
every visible vertex of the same polygon against all edges of that polygon. It has no boundary polygon. A contribution to pyvisgraph would have been a rewrite, because the reduction to
extremities, the pruning of graph edges, the boundary polygon and touching polygons change its data model,
visibility computation and search.

A reproducible benchmark on world maps of up to 9,952 vertices, validating every path independently with Shapely,
measured `extremitypathfinder` (with Numba) as 4.6 to 7.2 times faster than pyvisgraph in building the
environment, with several times fewer graph edges and faster queries (\autoref{fig:benchmark}).
On the map with 4,954 vertices, 59 of 200 pyvisgraph paths crossed land (\autoref{fig:invalid}),
because its point-in-polygon test mixes exact and tolerant comparisons.
Polyanya and TřiVis have not been measured yet.

![Benchmark on world maps (Apple M1 Pro, one thread). pyvisgraph exceeded the budget at 9,952 vertices;
its queries on 4,954 vertices are left out, because they are invalid.\label{fig:benchmark}](benchmark_sweep.png)

![A pyvisgraph path through land (dashed) and the path of `extremitypathfinder`.\label{fig:invalid}](benchmark_world_example.png)

# Software design

The design trades preprocessing time for query time. The visibility between extremities depends only on the
environment and is computed once. A query only computes the visibility of its start and goal, keeps its data local
and never modifies the environment, so environments are immutable and queries can run in any order.

The graph is kept small without losing optimality. A shortest path bends only at extremities, so other vertices
are not graph nodes. In addition, an extremity lying "in front of" another one, seen from its obstacle side,
is never its successor on a shortest path, and the edge between them is omitted
[Property One in @vinther2015pathfinding].

Visibility is computed with the Angle Range Elimination Algorithm (AREA), developed for this package.
From each origin it sorts the candidate vertices and the edges by angle, and every edge eliminates the candidates
behind it, instead of a rotating ray keeping the edges in a tree. Cheap tests on angle ranges and distances decide
most cases, and directions are compared with a trigonometry-free angle representation.
With $m$ extremities, the visibility graph typically takes $O(m\,n \log n)$ time, because a candidate is checked
against few edges: about 1.5 on the benchmark maps, independent of their size. The worst case,
$O(n\,m^2)$ or $O(n^3)$ for $m \sim n$, is worse than Lee's. AREA was kept, because a rotational sweep with
a tree of edges was slower in Python on the measured maps.

All questions at vertices, such as which vertices are extremities and where a line of sight may pass,
are answered by the free angle ranges ("wedges") around each vertex position, computed with a winding number.
Touching and overlapping polygons are thus treated as their union, without special cases.

The package computes with double precision and has no tolerance parameter: polygons touch only where coordinates
are exactly equal or a vertex lies exactly on an edge. Every floating point decision was audited against exact
rational arithmetic. With integer coordinates up to $2^{20}$, such as grids,
no decision differed. Otherwise, differences occurred only within about one unit in the last place of
a decision boundary, which is within the rounding of the coordinates themselves.
The A\* search is adapted from NetworkX to follow the temporary edges of start and goal,
so the shared graph is neither changed nor copied.

# Research impact statement

<!--
Stability (decided in #196): submit a few months after the next release, and name that release here.
Replace the TODOs at submission.
-->
The package has been developed iteratively since 2018: 21 releases up to 2024 (versions 0.0.1 to 2.7.2),
with code from four further contributors. Versions 2.8 to TODO (2026) modernised it, with a new API in 3.0:
touching obstacles now block paths, input is validated faster and accepted from Shapely, memory use is lower,
and benchmarks are reproducible. This paper describes version TODO (released TODO).
Since 2020, eleven publications from at least four institutions have used the package,
as shown by their text or by the planner in their official code:

- Trajectory planning for mobile robots at Chalmers University of Technology, 2021 to 2024: the planners of
  @berlin2021trajectory, @zhang2021motion, @zhang2023prescient, @zhang2023collision and
  @ceder2024bev compute their reference paths with the package. A co-author of @berlin2021trajectory, Georg Hess, also contributed
  to the package.
- Multi-UAV task allocation for search and rescue at the University of Southern Denmark:
  @grontved2023decentralized, whose library `trajallocpy` calls the package, and @grontved2025communication,
  which uses `trajallocpy`.
- @zapatarodriguez2026hierarchical (TMLR, Technical University of Munich) uses it as an oracle planner for
  "the exact shortest Euclidean path through the free space of a 2D polygonal environment".
  Its code works around a singular matrix error of version 2.7.2; that test is now solved in closed form.
- @cankiri2020guido builds the visibility graphs of an augmented reality indoor navigation system with it,
  stating that the package uses Lee's algorithm; it uses AREA.
- Master theses: @laitala2024docking (Chalmers, autonomous docking of vessels) uses it as its path planner,
  and @gaba2022uav (MIT, last-mile delivery with drones) computes shortest paths around obstacles with it.

Research software builds on it as well: SiTT, the transport simulation of the ERC project STRADA at the University of
Trier, and `ros_sgp_tools`, a ROS 2 package for informative path planning.
GitHub lists 29 dependent repositories, and the package was downloaded about 114,000 times
(November 2026, including mirrors and automated installations).
Earlier citations use the author's former name, Jannik Michelfeit, or the former GitHub username MrMinimal64.
A survey of all known uses is part of the repository.

# AI usage disclosure

<!--
The maintainer confirms every statement of this section before submission, in particular:
which parts the agent wrote and the commit counts (update them on the submission date).
Confirmed by the maintainer: no AI tools were used from 2018 to 2024.
-->
The core approach and its original implementation (2018 to 2024) were written by the author without AI tools:
the extremity graph, its edge reduction, AREA and the precomputation.
Since September 2026, the author has used Claude Code (Anthropic), an AI coding agent, for a substantial part of the
modernisation: about 130 of roughly 205 commits from September to November 2026 are co-authored by it
(`Co-Authored-By` trailers), and their pull requests carry an AI disclosure. The agent wrote most of the new code
of versions 2.8 to TODO, such as the wedges for touching obstacles, the input validation, the memory reduction,
the Shapely input and the plotting, as well as tests, documentation and the floating point audit.
It built the benchmark, surveyed the known uses and drafted this paper, which the author edited.
Trade-offs were decided by the author; smaller decisions of the agent are marked in the decision log.
Every change was reviewed by the author, the tests run with and without Numba in continuous integration,
every bug fix comes with a regression test, and the benchmark paths are validated independently.
The instructions for coding agents in the repository are contributor tooling, not part of the package.

# Acknowledgements

The author thanks the contributors of code: Andrew Costello (exceptions instead of assertions in the input validation),
Georg Hess (support for overlapping polygons), Ivan Doria (the command line interface) and GitHub user gpapadok
(tests and the plotting documentation), as well as everyone who reported issues and requested features.
The package was started in 2018 during the author's studies at TU Dresden. Its development has received no funding.

# References
