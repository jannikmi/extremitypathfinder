# trajallocpy (library)

- **Status:** Direct use (dependent package)
- **Bib key:** none (software; the paper is `grontved2023decentralized`)
- **Source type:** Python library on PyPI and GitHub
- **Authors:** Kasper A. R. Grøntved (GitHub `kasperg3`), University of Southern Denmark
- **Code:** [kasperg3/trajallocpy](https://github.com/kasperg3/trajallocpy) (MIT, 13 stars, created 2022-12, last push 2026-05);
  [PyPI](https://pypi.org/project/trajallocpy/)
- **Topic:** trajectory task allocation (CBBA) for multiple UAVs in coverage and search and rescue missions
- **Package version:** `extremitypathfinder[numba]`, unpinned (`setup.py`, `requirements.txt`)
- **API used:** 2.x: `PolygonEnvironment()`, `store(..., validate=False)` (`CoverageProblem.py`),
  `find_shortest_path(start, end, free_space_after=False, verify=False)` (`Agent.py`)
- **Used by:** `grontved2023decentralized`, `grontved2025communication` (SwarmTalk), `kasperg3/RosTrajAlloc`, `kasperg3/DroneComms`
- **Last checked:** 2026-11-05 (code)

## Use of the package

The coverage problem stores the search area and obstacles once; the agents query paths between tasks.

## Maintenance notes

- **Breaks with 3.0:** unpinned dependency, and it uses `store()` and `free_space_after`, both removed in 3.0.
  A fresh install on Python ≥ 3.12 gets 3.0 and fails; Python < 3.12 still resolves to 2.7.2.
  Worth an issue or pull request upstream (pin `<3` or port to the 3.0 API).
- The only dependent package found on PyPI apart from SiTT (`sitt.md`).
