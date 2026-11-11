# Red Hat AI: Intelligent Agriculture Demo

- **Status:** Direct use (industry demo and workshop)
- **Bib key:** none (software, outside research)
- **Source type:** demo and workshop repository of a company organisation
- **Authors:** GitHub organisation `rh-aiservices-bu` ("Red Hat AI")
- **Code:** [rh-aiservices-bu/intelligent-agriculture-demo](https://github.com/rh-aiservices-bu/intelligent-agriculture-demo)
  (GPL-3.0, 3 stars, created 2022-08, last push 2023-05)
- **Topic:** showcase of AI model training and serving (crop disease recognition), 5G slices, edge computing and
  "Path optimization using Optapy and a PathFinding algorithm"
- **Package version:** `extremitypathfinder==2.4.1` (`workshop/requirements.txt`, `pathservice/container/Pipfile`)
- **API used:** 2.x: `PolygonEnvironment()`, `store()`, `prepare()`, `find_shortest_path()`, `PlottingEnvironment`
  (`pathservice/container/app.py`, `workshop/04_pathfinder.ipynb`)
- **Last checked:** 2026-11-05 (code)

## Use of the package

A containerised path service ("Path and Route services API") finds paths on a farm map whose holes are areas that cannot be crossed
(a river). Workshop notebook 04 explains the package and its plots to participants.

## Maintenance notes

- Industrial adoption, in a teaching context (workshop). Good example for the gallery (#168).
- Pinned to 2.4.1, unaffected by 3.0; the plots it describes come from the 2.x plotting module.
