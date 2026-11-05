# gaba2022uav

- **Status:** Direct use (stated in the text, cited as a reference)
- **Bib key:** `gaba2022uav` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Institution:** MIT, Institute for Data, Systems, and Society and Department of Electrical Engineering and Computer
  Science (dual degree: S.M. Technology and Policy, S.M. EECS)
- **Supervisors:** Matthias Winkenbach (Research Scientist, MIT Center for Transportation & Logistics), Duane Boning (EECS)
- **Topic:** routing of delivery drones (last mile, case study Greater Boston) with societal constraints:
  no-fly zones, noise, cruise altitude, maximum take-off weight, maintenance
- **Code:** none found
- **Package version, API:** not stated
- **Names the package:** yes, section 3.3 (flight restrictions): "This thesis leverages the extremitypathfinder Python
  package (Michelfeit et al., 2021)". Bibliography: "Jannik Michelfeit, Ivan Doria, Andrew Costello, and georghess
  Georg Hess. extremitypathfinder. 2021."
- **Last checked:** 2026-11-06 (full text)

## Use of the package

The FAA no-fly zones and additional geofenced zones (drawn by hand as polygons, more of them for stronger constraint
scenarios) are the obstacles. Flight trajectories go laterally around them on the visibility graph, and the shortest
paths between all nodes give an asymmetric distance matrix, the input of the routing models (GURP).

## Maintenance notes

- Use case for a distance matrix between many points (#175), with environments rebuilt per scenario.
- The citation lists the GitHub contributors of 2021 as authors and uses the former name: one more reason for
  `CITATION.cff`.
