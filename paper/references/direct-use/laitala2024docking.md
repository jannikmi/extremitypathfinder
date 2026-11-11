# laitala2024docking

- **Status:** Direct use (stated in the text, cited as reference [26])
- **Bib key:** `laitala2024docking` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Supervisors, examiner:** Axel Måneskiöld and Daniel Söderberg (CPAC Systems AB), Peter Forsberg (Chalmers)
- **Industry partner:** CPAC Systems AB
- **Topic:** trajectory planning and route adaptation for autonomously docking a marine vessel with MPC
- **Code:** none found
- **Package version, API:** not stated (cited as "GitHub-name MrMinimal64. Extremitypathfinder. Accessed: May 03, 2024", PyPI URL)
- **Names the package:** yes, in sections 3 (methods), 4 (results, figures 4.1, 4.2) and 5.1 (discussion)
- **Last checked:** 2026-11-05 (full text)

## Use of the package

Path planner of the docking system: boundary polygon counter clockwise, obstacles clockwise, A\* on the visibility graph,
followed by a smoothing algorithm and an MPC. The thesis lists the input requirements it had to meet
(no self-intersections, no consecutive identical vertices, at least three vertices).

## Maintenance notes

- Feedback from the discussion (5.1): "functioned as expected", but the path does not consider the vessel's dimensions
  (it may pass through gaps too narrow for the vessel) and needs smoothing. The first is tracked as agent clearance (#167).
- The next release accepts polygons in either orientation (#161, `Unreleased` in `CHANGELOG.rst`),
  so the orientation requirement they describe goes away.
- Cites the former username MrMinimal64: an example why `CITATION.cff` is needed.
- Follow-up thesis: `persson2026docking`.
