# zhang2023collision

- **Status:** Direct use (official code calls the package)
- **Bib key:** `zhang2023collision` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Full text:** <https://research.chalmers.se/publication/538118>
- **Institution:** Chalmers University of Technology
- **Topic:** a DQN agent proposes actions that are turned into a reference trajectory, tracked by MPC (DQN-MPC hybrid)
- **Code:** [Woodenonez/TrajTrack-MPCnDQN-RLBoost](https://github.com/Woodenonez/TrajTrack-MPCnDQN-RLBoost)
  (MIT, 48 stars, archived)
- **Package version:** `extremitypathfinder==2.4.1`
- **API used:** 2.x: `PolygonEnvironment()`, `store(..., validate=False)`, `prepare()`, `find_shortest_path()`,
  in the path planner and in the RL training environment (`src/pkg_dqn/environment/environment.py`)
- **Names the package:** no (full text checked). "The reference path in each random scene is generated through the
  visibility graph and A\* algorithm as in [25]."
- **Last checked:** 2026-11-05 (Crossref, full text, code)

## Use of the package

Reference paths for random training and test scenes of the RL agent, so the package runs once per episode.

## Maintenance notes

- Many queries in many small, newly generated environments: preparation time matters here, not only query time.
- Pinned to 2.4.1, unaffected by 3.0.
