# SiTT: Simulation of Traffic and Transport (STRADA, University of Trier)

- **Status:** Direct use (dependent package; used in an example and earlier precalculation code)
- **Bib key:** none (software)
- **Source type:** research software (Python package on PyPI) of a research project in the digital humanities
- **Project:** STRADA (Simulation of Transport between the Adriatic Sea and the Danube), EU/ERC project 101001835,
  University of Trier; see [CORDIS](https://cordis.europa.eu/project/id/101001835) and <https://strada.uni-trier.de/>.
  A chapter on STRADA is in the Oxford Handbook of Digital Classical Studies (2026, abstract only checked).
- **Authors:** Maximilian Kalus (package author)
- **Code:** [Simulation-of-Traffic-and-Transport/SiTT](https://github.com/Simulation-of-Traffic-and-Transport/SiTT) (MIT);
  [PyPI](https://pypi.org/project/sitt/)
- **Topic:** agent-based simulation of pre-industrial (Roman) trade routes over land and water, with PostGIS
- **Package version:** `extremitypathfinder[numba]~=2.7.1` (`pyproject.toml`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(shore_line, holes, validate=True)`, `find_shortest_path()`
  (`examples/calculate_lake_routes.py`, `precalculation/old_files/connect_lake_harbors.py`)
- **Last checked:** 2026-11-05 (code, PyPI)

## Use of the package

Routes between harbours across lakes: the shore line is the boundary polygon, islands are holes (Shapely geometries
from PostGIS).

## Maintenance notes

- A code comment points to #84 ("Path going straight through hole", closed): they hit that bug.
- Pinned to `~=2.7.1`, so 3.0 does not break it. Unlike the other users, it validates its input (`validate=True`).
- Use case for Shapely input (#166) and the GIS application domain (#168).
