# zhang2021motion

- **Status:** Direct use (code), weaker link: the repository that calls the package names this paper as "related"
- **Bib key:** `zhang2021motion` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Full text:** <https://research.chalmers.se/publication/526674>
- **Institution:** Chalmers University of Technology
- **Topic:** multimodal motion prediction (mixture density networks) of dynamic obstacles, combined with NMPC
  trajectory generation for mobile robots
- **Code:** [Woodenonez/TrajGenAvo-NMPC-OpEn](https://github.com/Woodenonez/TrajGenAvo-NMPC-OpEn)
  (GPL-3.0, archived), restructured from the code of `berlin2021trajectory`. The evaluation of the prediction model is in
  [Woodenonez/M3P-MDN-CASE2021](https://github.com/Woodenonez/M3P-MDN-CASE2021) (does not use the package).
- **Package version:** `extremitypathfinder>=2.1.0` (`env/environment.yml`, unpinned upper bound)
- **API used:** 2.x: `PolygonEnvironment()`, `store()`, `prepare()`, `find_shortest_path()`,
  plotting: `PlottingEnvironment`, `draw_prepared_map` (`src/path_advisor/visibility.py`)
- **Names the package:** README yes ("implemented in the extremitypathfinder Python package"). The paper does not name it
  and does not describe the global planner (full text checked)
- **Last checked:** 2026-11-05 (Crossref, full text, README, code)

## Use of the package

Global reference path (A\* on the visibility graph), followed by an NMPC that avoids predicted obstacles.

## Maintenance notes

- Uses plotting functions removed in 3.0 (`PlottingEnvironment`, `draw_prepared_map`) and the 2.x API.
  With the unpinned requirement a fresh install on Python ≥ 3.12 gets 3.0 and fails.
