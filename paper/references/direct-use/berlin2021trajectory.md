# berlin2021trajectory

- **Status:** Direct use (official code calls the package, its README names it)
- **Bib key:** `berlin2021trajectory` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Preprint:** TechRxiv [10.36227/techrxiv.14215862](https://doi.org/10.36227/techrxiv.14215862)
- **Institution:** Chalmers University of Technology (Åkesson group); Per-Lage Götvall: AB Volvo (funded by the Volvo project ViMCoR, per the paper)
- **Topic:** long-range trajectory generation for a mobile robot in a factory: visibility graph + A\* for the global
  path, followed by a nonlinear MPC (OpEn/PANOC) for a smooth, collision-free trajectory
- **Code:** [wljungbergh/mpc-trajectory-generator](https://github.com/wljungbergh/mpc-trajectory-generator)
  (64 stars, created 2020-10). Started as a project in the Chalmers course SSY226 (autumn 2020).
  Restructured later as [Woodenonez/TrajGenAvo-NMPC-OpEn](https://github.com/Woodenonez/TrajGenAvo-NMPC-OpEn), see `zhang2021motion.md`.
- **Package version:** installed from source (commit "added extremitypathfinder from source"), `env/environment.yml`
- **API used:** 2.x: `PolygonEnvironment()`, `store()`, `prepare()`, `find_shortest_path()`
- **Names the package:** the README ("find the shortest path using A\*, implemented in the extremitypathfinder python package").
  The paper (TechRxiv version in the repository) describes the visibility graph and A\* but does not name it.
- **Last checked:** 2026-11-05 (Crossref, README, code, PDF in the repository)

## Use of the package

Obstacles are modelled as polygons. The package returns the shortest path, which serves as the reference for the NMPC.

## Maintenance notes

- Co-author Georg Hess contributed to the package (intersecting polygons, see `CHANGELOG.rst` and the About page).
  Not a fully independent use: disclose this if the paper is cited in the JOSS paper.
- Root of the Chalmers lineage: `zhang2021motion`, `zhang2023prescient`, `zhang2023collision`, `ceder2024bev`
  reuse its path planning module. Ze Zhang's PhD thesis cites this paper for "a visibility graph with A\*".
- Uses the 2.x API: breaks with 3.0 (cf. the upgrade guide).
