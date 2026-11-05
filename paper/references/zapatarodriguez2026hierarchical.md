# zapatarodriguez2026hierarchical

- **Status:** Direct use (stated in the text with a footnote linking the repository, and in the official code)
- **Bib key:** `zapatarodriguez2026hierarchical` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Institution:** not checked
- **Topic:** geometric observation interface for spatial planning with RL; an oracle planner as the reference
- **Code:** [Andresrzr/HGARL](https://github.com/Andresrzr/HGARL) (created 2026-08)
- **Package version:** `extremitypathfinder==2.7.2` (`pyproject.toml`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(boundary, holes, validate=...)`, `find_shortest_path(start, goal, verify=...)`
  (`rl/envs/map/solvers/map_solver.py`, also `rl/evaluator.py`). Import inside a `try`: it fails gracefully if missing.
  Input from triangulated obstacle maps via Shapely; option `retry_make_valid` repairs invalid polygons.
- **Names the package:** yes, footnote 11: the oracle computes "the exact shortest Euclidean path through the free space
  of a 2D polygonal environment"
- **Last checked:** 2026-11-05 (metadata on OpenReview, passage checked by the maintainer, code; OpenReview refuses
  automated PDF downloads)

## Use of the package

Ground truth (oracle) for the RL agent's spatial planning: optimal path lengths for the evaluation.

## Maintenance notes

- Relies on the claim "exact shortest paths": the main reason for #138 (random query tests) and #3 (floating point audit).
- They repair input geometries with Shapely before storing them: a use case for Shapely input (#166).
- Pinned to 2.7.2, unaffected by 3.0.
