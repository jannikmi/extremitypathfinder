# Jilin University TARS_GO: decision code for the RoboMaster AI Challenge (ICRA 2020)

- **Status:** Direct use, in an archived part of the code
- **Bib key:** none (competition code)
- **Source type:** open source code of a student robotics competition team
- **Authors:** Jilin University team (吉林大学吉甲大师团队), GitHub `Junking1`
- **Code:** [Junking1/ICRA2020-JLU-TARS_GO-Decision](https://github.com/Junking1/ICRA2020-JLU-TARS_GO-Decision)
  (42 stars, 10 forks, created 2020-08)
- **Topic:** decision making (decision tree and reinforcement learning) for robots in the RoboMaster AI Challenge
  held with ICRA 2020
- **Package version:** not declared
- **API used:** 2.x: `PolygonEnvironment()`, `store(..., validate=False)`, `prepare()`, `find_shortest_path()`,
  plotting `draw_prepared_map` and other 2.x drawing helpers
  (`强化学习部分/archieved/GlobalLocalPlanner.py`, i.e. "reinforcement learning part / archived")
- **Last checked:** 2026-11-05 (code)

## Use of the package

Global planner of the robot on the battlefield map, with a dynamic window approach for local planning.
Obstacles are inflated by the robot size before they are stored.

## Maintenance notes

- Inflating obstacles by the robot size is a workaround for agent clearance (#167); the same pattern appears in
  `mcgill-stealth-path-planning.md` and as a complaint in `laitala2024docking.md`.
