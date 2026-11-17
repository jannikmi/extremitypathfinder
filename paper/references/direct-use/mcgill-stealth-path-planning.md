# Stealth path planning (McGill University honours project)

- **Status:** Direct use (student research project)
- **Bib key:** none (no publication found)
- **Source type:** undergraduate honours research project (COMP 400)
- **Authors:** Allison Meikle, supervised by Clark Verbrugge, McGill University (from the README)
- **Code:** [allisonmeikle/stealth-path-planning](https://github.com/allisonmeikle/stealth-path-planning)
  (created 2025-09, last push 2026-03)
- **Topic:** path planning in stealth games: an agent stays hidden from a guard's line of sight, with visibility
  polygons, shadow regions and Monte Carlo Tree Search
- **Package version:** `extremitypathfinder==2.7.2` (`requirements.txt`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(boundary, obstacles, True)` (validation on), `find_shortest_path()` (`map.py`)
- **Last checked:** 2026-11-05 (README, code)

## Use of the package

Shortest paths between points of the game map; obstacles and the boundary are inflated with Shapely first.

## Maintenance notes

- Game AI domain (#168). Inflation workaround for clearance (#167).
