# grontved2025communication

- **Status:** Direct use (official code calls the package through `trajallocpy`)
- **Bib key:** `grontved2025communication` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Institution:** University of Southern Denmark (SDU)
- **Topic:** decentralized UAV-to-UAV communication with ESP-NOW; the demo planner allocates coverage tasks
- **Code:** [kasperg3/swarmtalk](https://github.com/kasperg3/swarmtalk) (4 stars), predecessor
  [kasperg3/DroneComms](https://github.com/kasperg3/DroneComms)
- **Package version:** `extremitypathfinder[numba]`, unpinned (README, `planning/Dockerfile` based on `ros:humble`)
- **API used:** indirectly, `trajallocpy.CoverageProblem` (see `trajallocpy.md`)
- **Names the package:** paper text not checked
- **Last checked:** 2026-11-05 (Crossref, code)

## Maintenance notes

- The Docker image uses ROS 2 Humble (Python 3.10), so pip resolves to 2.7.2 and 3.0 does not break it.
  A manual install on Python ≥ 3.12 would break through `trajallocpy`.
