# Chalmers course SSY226, 2025: local coordinator for a fleet of mobile robots (Duckiebots)

- **Status:** Experimented: the code contains a module that calls the package, but the default planner does not use it
- **Bib key:** none (course project)
- **Source type:** course project, SSY226 "Design project in systems, control and mechatronics", Chalmers
- **Authors:** project group 8 (GitHub `filipSand` and others)
- **Code:** [filipSand/ssy226-group8-2025](https://github.com/filipSand/ssy226-group8-2025) (MIT, created 2025-11);
  related Duckietown version [blizzt44/DuckieTownROS](https://github.com/blizzt44/DuckieTownROS) (created 2025-11)
- **Topic:** local coordination of a fleet of autonomous mobile robots (Duckiebots, ROS)
- **API used:** 2.x in `pkg_motion_plan/path_plan_cspace/visibility_extremitypathfinder.py`, not imported
- **Last checked:** 2026-11-05 (code)

## Use of the package

Copies the module layout of the Chalmers research code (`zhang2025future.md`), where the networkx/Shapely `visibility.py`
is used. The 2020 edition of the same course produced `berlin2021trajectory`, which used the package.
