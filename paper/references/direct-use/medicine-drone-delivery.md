# Medicine drone delivery (route service)

- **Status:** Direct use (project of unknown context)
- **Bib key:** none
- **Source type:** personal or team project (Kotlin backend, Python gRPC route service)
- **Authors:** GitHub `BardinPetr`
- **Code:** [BardinPetr/medicine-drone-delivery](https://github.com/BardinPetr/medicine-drone-delivery)
  (created 2024-10, last push 2025-12)
- **Topic:** drone delivery of medicine with no-flight zones
- **Package version:** `extremitypathfinder==2.7.2` (`python/requirements.txt`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(..., validate=False)`, `find_shortest_path()` (`python/router.py`)
- **Last checked:** 2026-11-05 (code)

## Use of the package

The boundary is the whole globe in longitude/latitude (−180…180, −90…90), the holes are bounding boxes of no-flight zones.
The environment is rebuilt on every zone update. A `ValueError` (no path) falls back to the straight line.

## Maintenance notes

- Treats longitude/latitude as planar coordinates: distances are not geodesic. A pitfall worth a sentence in the docs
  (projected coordinates for geographic data).
