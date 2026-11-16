# cankiri2020guido

- **Status:** Direct use (stated in the text, cited as reference [11])
- **Bib key:** `cankiri2020guido` in `paper.bib` (title, authors, venue, year, DOI or URL)
- **Institution:** not checked (Uğur Güdükbay is at Bilkent University)
- **Topic:** augmented reality indoor navigation on smartphones; visibility graphs of rooms for route guidance
- **Code:** none found
- **Package version, API:** unknown
- **Names the package:** yes. Google Scholar shows the passage
  "We utilize Extremity Pathfinder Python library [11] to create visibility graphs of the places for indoor navigation".
- **Citations of the paper:** 17 (Google Scholar, November 2026)
- **Last checked:** 2026-11-05 (passage checked by the maintainer, Scholar snippet)

## Use of the package

Builds visibility graphs from the saved borders and obstacles of a place, which the AR app uses for navigation.

## Maintenance notes

- Earliest citation found. Spells the name "Extremity Pathfinder" (two words): include this spelling in future searches.
- States that the package uses Lee's algorithm and its complexity, which is wrong (the package uses AREA,
  cf. `docs/8_design.rst`). The JOSS paper should say this clearly.
