# zapatarodriguez2026hierarchical

- **Status:** Direct use (stated in the text with a footnote linking the repository, and in the official code)
- **Bib key:** `zapatarodriguez2026hierarchical` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Institution:** Technical University of Munich, TUM School of Engineering and Design (all four authors)
- **Topic:** a hierarchical, set-valued observation interface on triangulated obstacle geometry for sparse-reward
  RL navigation, compared with raster (CNN) observations; published in TMLR 08/2026
- **Code:** [Andresrzr/HGARL](https://github.com/Andresrzr/HGARL) (created 2026-08)
- **Package version:** `extremitypathfinder==2.7.2` (`pyproject.toml`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(boundary, holes, validate=...)`, `find_shortest_path(start, goal, verify=...)`
  (`rl/envs/map/solvers/map_solver.py`, also `rl/evaluator.py`). Import inside a `try`: it fails gracefully if missing.
- **Names the package:** yes, appendix "Oracle planner" with footnote 11 linking the repository: A\* on a visibility
  graph over the extremity vertices, "the exact shortest Euclidean path through the free space of a 2D polygonal
  environment". The package solves the search with `networkx.astar_path`.
- **Last checked:** 2026-11-06 (full text, code)

## Use of the package

Oracle for the evaluation: path efficiency of the trained agent relative to the shortest feasible path, over successful
episodes. Obstacles come from triangulated maps, merged and simplified with Shapely before they are stored.

## Maintenance notes

- **Robustness problems reported in the paper:** footnote 6 and the appendix say the oracle falls back to a discrete
  grid A\* "when numerical instability is detected (e.g., degenerate geometry or precision failures)".
  The code shows what fails: it retries after `make_valid()` and stronger simplification when the error message
  contains "self intersection", "given polygon" or "singular" (numpy's "Singular matrix"; 2.7.2 calls
  `np.linalg.solve`, uncaught in `utils_numba.py`), then without validation, then with obstacles rebuilt from the
  occupancy grid. It also snaps coordinates (`epf_snap_precision`) and checks the returned path (`_check_path_quality`).
  Worth reproducing with their maps: input for #3 (floating point audit), #138 (random query tests) and #166
  (Shapely input, validation and repair). Some of it may be fixed since 3.0 (exact contact tests, touching obstacles).
- Relies on the claim "exact shortest paths", so the paper's results depend on it.
- Inflates the merged obstacles with `buffer(inflate, join_style=2)` (mitred joins): a workaround for agent clearance (#167).
  One environment per evaluation episode: evidence for the preparation time of small environments (#80).
- Pinned to 2.7.2, unaffected by 3.0.
