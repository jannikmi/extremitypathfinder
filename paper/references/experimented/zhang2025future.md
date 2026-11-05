# zhang2025future

- **Status:** Experimented: the code contains a module that calls the package, but the default planner does not use it
- **Bib key:** `zhang2025future` in `paper.bib` (title, authors, venue, year, DOI or URL), marked there as experimented: do not cite as evidence of use
- **Full text:** <https://research.chalmers.se/publication/547361>
- **Institution:** Chalmers University of Technology
- **Topic:** dynamic obstacle avoidance for mobile robots in warehouses and hospitals with energy-based multimodal motion
  prediction and MPC
- **Code:** [Woodenonez/DyObAv-MPCnEBM-Warehouse](https://github.com/Woodenonez/DyObAv-MPCnEBM-Warehouse) (MIT, 20 stars)
- **Package version:** `extremitypathfinder==2.7.2` (`requirements.txt`)
- **API used:** 2.x: `PolygonEnvironment()`, `store()`, `find_shortest_path()` in
  `src/pkg_motion_plan/path_plan_cspace/visibility_extremitypathfinder.py`
- **Names the package:** no (full text checked). The text does not say how the reference path is computed, so it does
  not settle which visibility module the experiments used; footnote 1 links the code
- **Last checked:** 2026-11-06 (Crossref, full text, code, default branch)

## Use of the package

`local_traj_plan.py` imports `visibility.py`, an own visibility graph implementation with networkx and Shapely with the
same class name `VisibilityPathFinder`. Nothing imports `visibility_extremitypathfinder.py`. Both files are in the first
commit (2024-09), so the history does not show which one the experiments used.

## Maintenance notes

- The Chalmers group switched from the package to its own implementation around 2024. Their reasons (features,
  dependencies, performance) would be useful feedback for the roadmap.
- Co-author Georg Hess contributed to the package.
- The same module layout (unused `visibility_extremitypathfinder.py`) is in `roselli2026scheduling`, `xue2026proactive`,
  `zhang2025gradient`, `wang2025collaboration` and student projects (`chalmers-ssy226-2025.md`).
