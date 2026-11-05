# Known uses of extremitypathfinder

One file per reference: publications, theses, software and projects that use, name or experimented with the package.
Title, authors, venue, year and DOI of the publications are in `../paper.bib`.
Collected for the JOSS paper (#190, research impact statement) and for maintenance: who uses which API and version,
which application domains and feature requests come from real users, and whom to contact.

Only entries with the status "Direct use" are evidence of use in the paper.

## Status

- **Direct use:** the text or the official code of the work calls the package.
- **Names the package:** the text names it, the use is not shown.
- **Experimented:** the code contains a module that calls the package or declares it as a dependency, but the default code
  path does not call it. Relevant because the authors tried the package, but not evidence of use.
- **Indirect:** builds on or cites work that used the package.
- **Not confirmed:** found by a search, could not be confirmed.

## Publications and theses

| File | Status | Institution | Topic |
| --- | --- | --- | --- |
| [cankiri2020guido](cankiri2020guido.md) | Direct use (text) | Bilkent (not checked) | AR indoor navigation |
| [berlin2021trajectory](berlin2021trajectory.md) | Direct use (code) | Chalmers | NMPC trajectories, factory robots |
| [zhang2021motion](zhang2021motion.md) | Direct use (code) | Chalmers | motion prediction + NMPC |
| [gaba2022uav](gaba2022uav.md) | Direct use (text) | MIT | UAV last-mile delivery routing |
| [grontved2023decentralized](grontved2023decentralized.md) | Direct use (code) | SDU | multi-UAV task allocation, search and rescue |
| [zhang2023prescient](zhang2023prescient.md) | Direct use (code) | Chalmers | warehouse navigation, motion prediction |
| [zhang2023collision](zhang2023collision.md) | Direct use (code) | Chalmers | DRL + MPC trajectory planning |
| [laitala2024docking](laitala2024docking.md) | Direct use (text) | Chalmers, CPAC Systems | autonomous docking of vessels |
| [ceder2024bev](ceder2024bev.md) | Direct use (code) | Chalmers | multi-robot DRL + MPC |
| [grontved2025communication](grontved2025communication.md) | Direct use (code, via trajallocpy) | SDU | UAV swarm communication |
| [zapatarodriguez2026hierarchical](zapatarodriguez2026hierarchical.md) | Direct use (text and code) | TU Munich | RL spatial planning, oracle planner |
| [persson2026docking](persson2026docking.md) | Names the package | Chalmers, CPAC Systems | autonomous docking of vessels |
| [zhang2025future](zhang2025future.md) | Experimented | Chalmers | energy-based motion prediction + MPC |
| [zhang2025gradient](zhang2025gradient.md) | Experimented | Chalmers | gradient field DWA |
| [wang2025collaboration](wang2025collaboration.md) | Experimented | Chalmers | robot fleets, latency, distributed MPC |
| [mathiesen2025drone](mathiesen2025drone.md) | Experimented | SDU | drone swarm RL |
| [roselli2026scheduling](roselli2026scheduling.md) | Experimented | Chalmers | fleet scheduling + MPC |
| [xue2026proactive](xue2026proactive.md) | Experimented | Chalmers, University of Pennsylvania | motion prediction + safe control |
| [zhang2025phd](zhang2025phd.md) | Indirect | Chalmers | future-oriented robot navigation |

## Software, industry, teaching and community

| File | Source type | Status | Context |
| --- | --- | --- | --- |
| [trajallocpy](trajallocpy.md) | library (PyPI) | Direct use | SDU, UAV task allocation |
| [sitt](sitt.md) | research software (PyPI) | Direct use | University of Trier, STRADA, historic transport simulation |
| [ros-sgp-tools](ros-sgp-tools.md) | ROS 2 package | Direct use | informative path planning, geofence |
| [redhat-intelligent-agriculture-demo](redhat-intelligent-agriculture-demo.md) | industry demo and workshop | Direct use | Red Hat AI, agriculture |
| [jlu-robomaster-icra2020](jlu-robomaster-icra2020.md) | competition code | Direct use (archived part) | Jilin University, RoboMaster AI Challenge |
| [ge-wiki-maps](ge-wiki-maps.md) | community project | Direct use (visibility only) | GoldenEye 007 speedrunning wiki |
| [mcgill-stealth-path-planning](mcgill-stealth-path-planning.md) | honours project | Direct use | McGill University, stealth games |
| [tamu-escort-robotics](tamu-escort-robotics.md) | course project | Direct use | Texas A&M University |
| [strozberg-pedestrian-transformers](strozberg-pedestrian-transformers.md) | personal project | Direct use | pedestrian prediction |
| [medicine-drone-delivery](medicine-drone-delivery.md) | personal or team project | Direct use | drone delivery, no-flight zones |
| [chalmers-ssy226-2025](chalmers-ssy226-2025.md) | course project | Experimented | Chalmers, Duckiebot fleet |
| [declared-dependency-only](declared-dependency-only.md) | five repositories | Experimented | dependency declared only |
| [pyvisigraph](pyvisigraph.md) | alternative package | Not confirmed (mention) | criticises approximate values |

## Findings

What the maintainers take from these references (compatibility, robustness, feature requests, documentation,
citations): [findings.md](findings.md).

## Sources checked (November 2026)

| Source | Query | Result |
| --- | --- | --- |
| GitHub code search | `extremitypathfinder` with `language:Python`, `language:Markdown`, `language:"Jupyter Notebook"`, `filename:requirements.txt`, `filename:pyproject.toml`, `filename:setup.py`, `filename:environment.yml`, `filename:visibility_extremitypathfinder.py`, `"from extremitypathfinder"`, `MrMinimal64/extremitypathfinder`, `jannikmi/extremitypathfinder` | about 33 repositories, all read (files in this folder) |
| GitHub dependents | <https://github.com/jannikmi/extremitypathfinder/network/dependents> | counter: 29 repositories, 1 package. Only 13 repositories are listed (the counter apparently includes private ones), all covered here, plus `alphavector/all` and `Lyn4ever29/pipy_server`, which only install all PyPI packages (a mirror), no use. Package: SiTT. |
| GitHub repositories, issues, commits | search for `extremitypathfinder` outside this repository | commits led to `whiteted-strats/GE_Wiki_Maps`; issues only in pyvisgraph and a Factorio project (no use) |
| Google Scholar | `"extremitypathfinder"` (by the maintainer; automated access is blocked) | Guido, Laitala; one mismatch (a timezonefinder citation) |
| Crossref | `query=extremitypathfinder`; every DOI of `paper.bib` | no hits for the name; metadata of all DOIs checked |
| arXiv API | `all:extremitypathfinder` | none |
| OpenAlex | `filter=fulltext.search:extremitypathfinder`, `"extremity pathfinder"`, `MrMinimal64` | none for the package (rate limited without an API key) |
| Semantic Scholar | website search; snippet API (full text) | none on the website; the API needs a key (rate limit) |
| Zenodo, DOAJ, HAL, OpenAIRE, Europe PMC | `extremitypathfinder` | none |
| Chalmers ODR | API `discover/search/objects?query=extremitypathfinder` | Laitala, Persson (full text index) |
| CORE, Mojeek, DuckDuckGo | `"extremitypathfinder"` | not reachable (bot checks) |
| Web search (Bing, Brave, general) | `"extremitypathfinder"` with `thesis`, `paper`, `pdf`, `university` | Berlin et al. (via the README of its code), pyvisigraph |
| Stack Exchange, Hacker News | `extremitypathfinder` | none |
| Full texts read | Chalmers theses and papers, arXiv preprints, PDFs in the code repositories | cf. "Last checked" of each file |
| Downloads | <https://pypistats.org/api/packages/extremitypathfinder/recent>, <https://pepy.tech/projects/extremitypathfinder> | 853 last month, about 114,000 in total |

## Reproduce or update

1. **Find candidates.** Run the GitHub code searches of the table, e.g.
   `gh api -X GET search/code -f q='extremitypathfinder language:Python' -f per_page=100 --jq '.items[].repository.full_name' | sort -u`
   (code search covers default branches only, at most 100 results per query, so split the queries as above).
   Open the dependents page. Repeat the literature searches, Google Scholar by hand, with all spellings:
   `extremitypathfinder`, `"extremity pathfinder"`, `ExtremityPathfinder`, `MrMinimal64`, `Michelfeit`.
2. **Check each repository.** Download it (`gh api repos/<owner>/<repo>/tarball > repo.tgz`, unpack, never run it) and
   find the calls: `grep -rn -E 'extremitypathfinder|PolygonEnvironment' --include='*.py' --include='*.ipynb'`.
   Trace from the entry point: a module that calls the package counts only if the code actually imports it
   (the Chalmers code has an unused `visibility_extremitypathfinder.py` next to the used `visibility.py`).
   Note the declared version (`requirements*.txt`, `pyproject.toml`, `setup.py`, `environment*.yml`) and the API calls.
3. **Check each publication.** Metadata from Crossref (`curl https://api.crossref.org/works/<doi>`), otherwise from the
   repository of the thesis. Read the full text for a mention (search for "extremity", "pathfinder", "visibility graph").
   Link the publication to the code only through the README of the code or the paper itself.
4. **Record.** One file per reference with the fields used here; publications also get an entry in `../paper.bib`,
   in the block of their status. Update "Last checked", this index and the findings above.

Search hits for the author's other packages (e.g. timezonefinder) are not uses of this package: check the reference.
