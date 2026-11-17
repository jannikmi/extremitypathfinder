# ceder2024bev

- **Status:** Direct use (official code calls the package)
- **Bib key:** `ceder2024bev` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Institution:** Chalmers University of Technology
- **Topic:** multi-robot trajectory planning from bird's-eye-view images with continuous DRL (DDPG, TD3) and MPC
- **Code:** [kristianceder/DRL-Traj-Planner](https://github.com/kristianceder/DRL-Traj-Planner) (MIT, 14 stars).
  Copies: [ChloeXue00/Navigation-of-Mobile-Robots-via-MPC-DRL](https://github.com/ChloeXue00/Navigation-of-Mobile-Robots-via-MPC-DRL),
  [ChloeXue00/safe_area_navigation_for_robots](https://github.com/ChloeXue00/safe_area_navigation_for_robots)
  (the second pins 2.7.2 in one file)
- **Package version:** `extremitypathfinder==2.4.1`
- **API used:** 2.x: `PolygonEnvironment()`, `store(..., validate=False)`, `prepare()`, `find_shortest_path()`
  (path planner and the DDPG/TD3 and DQN training environments)
- **Names the package:** no (full text checked). States that reference paths come from visibility graphs and A\*.
- **Last checked:** 2026-11-05 (Crossref, full text, code)

## Use of the package

Reference paths for the robots, also inside the RL training environments.

## Maintenance notes

- Pinned to 2.4.1, unaffected by 3.0.
