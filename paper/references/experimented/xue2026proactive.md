# xue2026proactive

- **Status:** Experimented: the code contains a module that calls the package, but the default planner does not use it
- **Bib key:** `xue2026proactive` in `paper.bib` (title, authors, venue, year, DOI or URL), marked there as experimented: do not cite as evidence of use
- **Preprint:** [arXiv:2601.10233](https://arxiv.org/abs/2601.10233)
- **Institution:** Chalmers University of Technology (Zhang, Åkesson) and University of Pennsylvania (Xue, Figueroa)
- **Topic:** local-minima-free navigation: multimodal motion prediction with control barrier functions (MCBF)
- **Code:** [yifanxueseas/mmp_mcbf_control](https://github.com/yifanxueseas/mmp_mcbf_control) (MIT)
- **Package version:** `extremitypathfinder==2.4.1` (`requirements.txt`, `requirements_socialnav.txt`)
- **API used:** 2.x in `src/pkg_motion_plan/path_plan_cspace/visibility_extremitypathfinder.py`, not imported
- **Names the package:** no (arXiv version checked by the earlier search)
- **Last checked:** 2026-11-05 (Crossref, arXiv title page, code, default branch)

## Use of the package

Same layout as `zhang2025future`: the planner uses the networkx/Shapely `visibility.py`. The package is still installed
from the requirements. Both files are in the first commit (2026-02).
