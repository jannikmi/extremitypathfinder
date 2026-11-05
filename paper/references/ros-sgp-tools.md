# ros_sgp_tools (ROS 2 companion of SGP-Tools)

- **Status:** Direct use (integration, research software)
- **Bib key:** none (software). The library SGP-Tools is described in papers by Kalvik Jakkala and Srinivas Akella
  (e.g. multi-robot informative path planning, [arXiv:2309.07050](https://arxiv.org/abs/2309.07050));
  whether their experiments used this path follower was not checked.
- **Source type:** ROS 2 package
- **Authors:** Kalvik Jakkala (GitHub `itskalvik`)
- **Code:** [itskalvik/ros_sgp_tools](https://github.com/itskalvik/ros_sgp_tools) (Apache-2.0); website <http://sgp-tools.com>
- **Topic:** sensor placement and informative path planning (IPP) for robots (e.g. ArduPilot vehicles) in ROS 2
- **Package version:** `extremitypathfinder`, unpinned (`requirements.txt`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(boundary, holes)`, `prepare()`, `find_shortest_path()`
  (`scripts/path_follower.py`, last change 2026-01)
- **Last checked:** 2026-11-05 (code)

## Use of the package

The path follower keeps the vehicle inside a geofence: the boundary is the convex hull of the fence, the holes are the
hull minus the fence (a way to model a non-convex fence). Coordinates are scaled from latitude/longitude first.

## Maintenance notes

- **Breaks silently with 3.0:** the dependency is unpinned and the code calls `store()` and `prepare()`, removed in 3.0.
  The exception is caught and the node "falls back to direct go2waypoint", i.e. straight lines that ignore the geofence.
  This affects installations on Python ≥ 3.12 (e.g. ROS 2 Jazzy); Python < 3.12 resolves to 2.7.2.
  Worth an issue upstream (pin `<3` or port to the 3.0 API).
