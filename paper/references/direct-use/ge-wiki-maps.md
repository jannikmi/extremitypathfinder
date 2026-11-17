# GE Wiki Maps (GoldenEye 007 speedrunning wiki)

- **Status:** Direct use (community project, outside research)
- **Bib key:** none
- **Source type:** hobby / community repository
- **Authors:** GitHub `whiteted-strats`
- **Code:** [whiteted-strats/GE_Wiki_Maps](https://github.com/whiteted-strats/GE_Wiki_Maps) (created 2020-10, last push 2026-09)
- **Topic:** top-down maps of GoldenEye 007 (N64) levels for the speedrunning wiki <https://wiki.the-elite.net>,
  generated from data dumped from the running game (guards, lines of sight, noise ranges)
- **Package version:** `extremitypathfinder==2.2.1` (`requirements.txt`, pinned in a September 2026 fix commit)
- **API used:** 2.2-era internals: `PolygonEnvironment`, `store()`, `prepare()`, `within_map()`, `translate()`,
  `extremitypathfinder.helper_classes.Vertex`, `extremitypathfinder.helper_fcts.find_visible` (`lib/fov.py`)
- **Last checked:** 2026-11-05 (code)

## Use of the package

Field of view of guards: the visibility computation (`find_visible`) finds the area a guard can see, comment:
"the visibility in this should be sufficient since our LOS is a straight line".

## Maintenance notes

- Uses internal modules that no longer exist (`helper_classes`, `helper_fcts`); they had to pin 2.2.1.
  A user of the visibility computation itself, not of the shortest paths: a use case for exposing visibility
  queries in the public API.
