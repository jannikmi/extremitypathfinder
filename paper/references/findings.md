# What the known uses tell us

A survey of how extremitypathfinder is used in research, industry, teaching and community projects (November 2026).
The individual references and the method of the survey are listed in the [index](README.md). Thanks to everyone who
published their code: it made this survey possible.

## Summary

- **Use in research:** 11 publications from 2020 to 2026 use the package and one more names it: papers in IEEE RA-L,
  IROS, CASE, ICAR, ICUAS, IV and TMLR, and master theses at Chalmers and MIT. They come from at least four
  institutions (Chalmers University of Technology, University of Southern Denmark, MIT, Technical University of Munich),
  with a concentration at Chalmers.
- **Software that builds on it:** SiTT (University of Trier, simulation of historic transport), `trajallocpy`
  (UAV task allocation) and `ros_sgp_tools` (informative path planning in ROS 2).
- **Outside research:** an agriculture demo and workshop by Red Hat AI, competition code (RoboMaster AI Challenge,
  ICRA 2020), maps for a GoldenEye 007 speedrunning wiki, and student projects.
- **Application domains:** mobile robots in factories and warehouses, multi-robot and UAV planning, marine vessel
  docking, delivery drones, reinforcement learning, indoor navigation, games, historic transport.

## 1. Compatibility with version 3.0

Every user found calls the 2.x API: `PolygonEnvironment()`, `store()`, mostly `prepare()`, then `find_shortest_path()`.
Version 3.0 sets up the environment in the constructor and removes `store()`, `prepare()`, the `free_space_after`
parameter and the old plotting helpers.

- Most research code pins a 2.x version (2.1.0 to 2.7.2) and keeps working.
- Two dependents declare the package without a version bound. On Python 3.12 or newer, where pip installs 3.0,
  they are incompatible:
  - `trajallocpy` calls `store()` and `find_shortest_path(..., free_space_after=False)`.
  - `ros_sgp_tools` calls `store()` and `prepare()`. It catches the error and then plans straight lines to the
    waypoints, without the geofence.
  On Python older than 3.12 pip still installs 2.7.2.

**Actions:** contact both projects (an upper bound `<3` or a port to the 3.0 API); check the
[upgrade guide](https://extremitypathfinder.readthedocs.io/en/latest/2_upgrading.html) against these exact calls;
announce future breaking changes with a deprecation release first.

## 2. Robustness on real-world geometry

The most demanding input comes from triangulated obstacle maps (TMLR, Technical University of Munich). The paper uses
the package as an oracle for exact shortest paths and falls back to a grid-based A\* "when numerical instability is
detected". Its code repairs the input with Shapely, rounds coordinates, retries without validation and rebuilds
obstacles from an occupancy grid. Among the errors it handles is a singular matrix: version 2.7.2 calls
`numpy.linalg.solve` in a test that assumes the system is always solvable. Other users met the bug fixed in #84
(a path through a hole) or describe the input rules they had to satisfy.

**Actions:** reproduce these failures with such maps and use them in the floating point audit (#3) and the random
query tests (#138); check what the exact contact tests of 3.0 already fix. The package's main promise, exact shortest
paths, is what these users rely on.

## 3. Feature requests confirmed by use

| Need | Seen in | Issue | Status (2026-11-06) |
| --- | --- | --- | --- |
| Keep a minimum distance to obstacles (robot size) | obstacles inflated by hand in three projects; requested in a thesis on vessel docking | #167 | decided: a documentation recipe built on `from_shapely()`, blocked by #166; priority high |
| Shapely geometries as input, with validation and repair | four users convert from Shapely, one repairs geometries | #166 | decided: convert only, own validation; priority high |
| Distances between many points | a distance matrix for drone routing, task costs for UAV task allocation | #175 | decided: `distance_matrix()`, visibility once per point; priority medium |
| Fast preparation of many small environments | one environment per training episode in reinforcement learning | #80 | 1 to 8 ms per environment of 12 to 52 vertices (with Numba), fast enough; evidence added to #80, priority low |
| Visibility queries in the public API | field of view in the GoldenEye maps, via internal functions | #205 | new issue; priority low |
| Non-convex boundaries | a geofence modelled as convex hull minus holes | #206 | documentation note |

The evidence is in the issue descriptions, with links to the code of the projects.

## 4. Documentation

- Geographic data: one user passes longitude and latitude as planar coordinates. A note on projected coordinates
  prevents wrong distances (#206).
- Input rules: a thesis lists the polygon orientation among the rules it had to satisfy. The next release accepts both orientations (#161);
  the documentation should say so clearly.
- Visibility algorithm: one paper states that the package uses Lee's algorithm. It uses Angle Range Elimination
  (AREA) instead ([design document](https://extremitypathfinder.readthedocs.io/en/latest/8_design.html)).

## 5. Citations and community

- Citations differ: the former username MrMinimal64, the former name Jannik Michelfeit, the contributors as authors,
  or only the URL. A citation file (`CITATION.cff`) and a JOSS paper give one reference to cite.
- Several users would make good examples for the gallery (#168): vessel docking, agriculture, historic transport,
  UAV task allocation, reinforcement learning.
- The group at Chalmers used the package from 2021 to 2024 and has since used an implementation of its own.
  Its experience would be valuable feedback on what is missing.

## 6. For the JOSS paper (#190)

- Count only the references with the status "Direct use" as evidence; the others are marked in the index and in
  `../paper.bib`.
- Disclose that a co-author of one Chalmers paper contributed to the package.
- State that the package uses AREA, not Lee's algorithm.

Do you use extremitypathfinder? Tell us in [#168](https://github.com/jannikmi/extremitypathfinder/issues/168).
