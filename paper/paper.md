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
or a character in a game. When the obstacles are polygons, the shortest path is a sequence of straight segments
that bends only at protruding obstacle corners.
`extremitypathfinder` is a Python package that computes such paths exactly.
The user describes the environment once, as a boundary polygon with polygonal holes or as a grid with blocked cells.
The package then precomputes a visibility graph that contains only the protruding corners, the *extremities*,
and only the edges that can be part of a shortest path [@vinther2015pathfinding].
Each query connects its start and goal to this graph and searches it with A\* [@hart1968astar].
The returned paths are Euclidean shortest paths in the free space, not approximations on a grid or a mesh.

# Statement of need

The package is meant for researchers and engineers in robotics, simulation, geographic information systems and
game development who need exact shortest paths in Python. It is installed with `pip`, requires only
NumPy [@harris2020numpy] and NetworkX [@hagberg2008networkx], and uses Numba [@lam2015numba] as an optional
accelerator. The results are identical with and without Numba, which is checked by the test suite in continuous
integration.

Typical use cases ask many queries in one static environment: reference paths for trajectory planners,
distance matrices for task allocation, oracle planners in reinforcement learning, simulations that move many agents
through the same map. The package therefore separates the preparation of an environment from the queries,
and a prepared environment can be stored with `pickle` and reused.

The input is accepted as it comes from practice. Obstacles may touch and overlap, vertices may lie on edges of other
polygons, polygons may be numbered clockwise or counter clockwise, and grids are converted into polygons that follow
the borders of the blocked cells. Touching obstacles block paths, so a path never squeezes through a gap of
zero width. No triangulation or navigation mesh has to be built first.
The documentation describes the input model, the algorithms and the limits of the package.

# State of the field

Visibility graphs are the classical exact method for Euclidean shortest paths among polygonal obstacles
[@lozanoperez1979]. Lee's rotational sweep computes the visibility graph of $n$ vertices in $O(n^2 \log n)$ time,
keeping the edges crossing the sweep ray in a balanced search tree [@lee1978proximity]. Polyanya [@cui2017polyanya]
computes exact any-angle paths on a navigation mesh without a visibility graph and is the state of the art for this
problem, but it needs a triangulation of the free space and is not available as a Python package. Grid planners such
as A\* or Theta\* are simpler, but their paths are in general longer than Euclidean shortest paths.

In Python, the closest alternative is pyvisgraph [@pyvisgraph]. It builds the complete visibility graph of all
obstacle vertices with Lee's algorithm, in an implementation that takes $O(n^3)$ in the worst case (a sorted list
instead of a tree, and an interior test per visible vertex), and searches it with Dijkstra's algorithm. Its obstacles
are polygons in the unbounded plane; it has no boundary polygon. VisiLibity1 (C++) computes visibility and shortest
paths, but its Python binding is not on PyPI. CGAL-based packages such as pyvispoly compute visibility polygons, not
shortest paths.

Building a new package instead of contributing to pyvisgraph was justified by the differences in its core:
the reduction to extremities, the pruning of graph edges, the boundary polygon and the handling of touching polygons
change the data model, the visibility computation and the search, so a contribution would have been a rewrite.
pyvisgraph has not been maintained since 2020.
A reproducible benchmark on world maps of up to 9,952 vertices, in which every returned path is validated
independently with Shapely, measured `extremitypathfinder` (with Numba) as 4.6 to 7.2 times faster than pyvisgraph
in building the environment. Its visibility graphs had several times fewer edges, and its queries were faster.
On the map with 4,954 vertices, 59 of 200 pyvisgraph paths crossed land; the cause, mixed exact and tolerant
comparisons in a point-in-polygon test, is described in the documentation. A comparison with Polyanya
has not been measured yet.

# Software design

The design trades preprocessing time for query time. The visibility between extremities depends only on the
environment and is computed once, when the environment is created. A query only computes the visibility of its start
and goal. Environments are therefore immutable: a changed map requires a new environment.
Precomputing all shortest paths between extremities was rejected, because memory would grow quadratically
with the number of extremities.

The graph is kept small without losing optimality. A shortest path bends only at extremities, so other vertices
are not graph nodes. In addition, an extremity lying "in front of" another one, seen from its obstacle side,
is never its successor on a shortest path, and the edge between them is omitted
[Property One in @vinther2015pathfinding]. The reduction makes the preprocessing more complex but speeds up the
queries.

Visibility is computed with the Angle Range Elimination Algorithm (AREA), developed for this package,
not with Lee's algorithm. From each origin, AREA sorts the candidate vertices and the edges by angle and removes
the candidates that lie behind an edge. Cheap tests on the angle range and the distance decide most cases,
so an intersection is computed only for few candidate-edge pairs, and no trigonometric functions are used.
A candidate is only checked against the edges whose angle range contains it. Usually these are few,
which gives $O(m\,n \log n)$ time for the visibility graph of $m$ extremities; adversarial maps with $m \sim n$
reach $O(n^3)$. On the benchmark maps, each candidate was checked against about 1.5 edges, independent of the map size.
Directions are compared with a monotone representation of the angle in $[0, 4)$ instead of `atan2`.

All questions at vertices, such as which vertices are extremities and where a line of sight may start,
pass through or end, are answered by one concept: the free angle ranges ("wedges") around each vertex
position, computed with a winding number argument. Touching and overlapping polygons are thus treated as
their union, without special cases in the visibility code.

The package computes with double precision floating point numbers and has no tolerance parameter. Polygons touch
only where coordinates are exactly equal or a vertex lies exactly on an edge, which makes the behaviour reproducible
and exact for integer coordinates such as grids. The numerical kernels are optionally compiled with Numba; without
it, the same functions run as plain Python or vectorised NumPy code. Queries keep all their data local, so they never
modify the prepared environment and can be repeated in any order. The A\* search is an adapted copy of the NetworkX
implementation that additionally follows the temporary edges of start and goal, so the shared graph is neither
changed nor copied.

# Research impact statement

The package has been on PyPI since 2018 and has been used in published research since 2020.
The following works use it, as shown by their text or by the planner in their official code:

- Trajectory planning for mobile robots at Chalmers University of Technology, 2021 to 2024: the reference paths of
  the planners in @berlin2021trajectory, @zhang2021motion, @zhang2023prescient, @zhang2023collision and
  @ceder2024bev are computed with the package. A co-author of @berlin2021trajectory, Georg Hess, also contributed
  to the package.
- Multi-UAV task allocation for search and rescue at the University of Southern Denmark:
  @grontved2023decentralized, whose library `trajallocpy` calls the package, and @grontved2025communication,
  which uses `trajallocpy`.
- @zapatarodriguez2026hierarchical (TMLR) uses it as an oracle planner for
  "the exact shortest Euclidean path through the free space of a 2D polygonal environment".
- @cankiri2020guido builds the visibility graphs of an augmented reality indoor navigation system with it.
  This paper states that the package uses Lee's algorithm; as described above, it uses AREA.
- Master theses: @laitala2024docking (Chalmers, autonomous docking of vessels) uses it as its path planner,
  and @gaba2022uav (MIT, last-mile delivery with drones) computes shortest paths around obstacles with it.

Research software builds on it as well: SiTT, the transport simulation of the ERC project STRADA at the University of
Trier, and `ros_sgp_tools`, a ROS 2 package for informative path planning.
GitHub lists 29 dependent repositories, and the package was downloaded about 114,000 times in total
(November 2026, including mirrors and automated installations).
Earlier citations use the author's former name, Jannik Michelfeit, or the former GitHub username MrMinimal64.
A survey of all known uses is part of the repository.

# AI usage disclosure

Since September 2026, the author has used Claude Code (Anthropic), an AI coding agent, for a substantial part of the
development: about 130 of roughly 205 commits from September to November 2026 are co-authored by it, marked with
`Co-Authored-By` trailers, and the corresponding pull requests carry an AI disclosure. It was used to write code,
tests and documentation, to build the benchmark, to survey the known uses of the package, and to draft this paper,
which the author edited. The core approach (extremity graph, AREA, precomputation) predates these tools (2018).
Trade-offs were decided by the author; smaller decisions taken by the agent are marked as such in the decision log
of the repository. Every change was reviewed by the author, the test suite runs with and without Numba in
continuous integration, every bug fix comes with a regression test, and the paths of the benchmark are validated
independently with Shapely.

# Acknowledgements

The author thanks the contributors of code: Andrew Costello (exceptions instead of assertions in the input validation),
Georg Hess (support for overlapping polygons), Ivan Doria (the command line interface) and GitHub user gpapadok
(tests and the plotting documentation), as well as everyone who reported issues and requested features.
The package was started in 2018 during the author's studies at TU Dresden. Its development has received no funding.

# References
