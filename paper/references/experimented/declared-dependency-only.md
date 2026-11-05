# Repositories that only declare the package as a dependency

- **Status:** Experimented: the package is declared or installed, no code calls it (on the default branch)
- **Bib key:** none
- **Last checked:** 2026-11-05 (code)

Kept in one file because each entry has little information.

| Repository | Context | Declared as |
| --- | --- | --- |
| [matthewyjiang/srpgt](https://github.com/matthewyjiang/srpgt) | simulation of safe reactive navigation in granular terrain | `extremitypathfinder>=2.7.1` (`pyproject.toml`) |
| [systut/mobile_robot_packages](https://github.com/systut/mobile_robot_packages) | "Python packages for mobile robot development", account "System Engineering Laboratory" | `extremitypathfinder` (`requirements.txt`) |
| [hjcwuhuqifei/rl_demo](https://github.com/hjcwuhuqifei/rl_demo) | reinforcement learning demo | `extremitypathfinder=2.1.0` (`requirements.txt`) |
| [kasperg3/RosTrajAlloc](https://github.com/kasperg3/RosTrajAlloc) | ROS wrapper of `trajallocpy` (SDU) | `extremitypathfinder[numba]` (`requirements.txt`) |
| [kasperg3/DroneComms](https://github.com/kasperg3/DroneComms) | predecessor of SwarmTalk (SDU), see `../direct-use/grontved2025communication.md` | installed in `planning/README.md` and `planning/Dockerfile` |

## Maintenance notes

- `srpgt` declares `>=2.7.1` without an upper bound: harmless as long as no code calls the package.
