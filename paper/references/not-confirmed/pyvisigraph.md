# pyvisigraph (alternative package that names extremitypathfinder)

- **Status:** Not confirmed (mention, not a use)
- **Bib key:** none
- **Source type:** Python package (PyPI, 2024), now removed from PyPI (`pypi.org/pypi/pyvisigraph/json` returns 404)
- **Source of the information:** a search engine snippet of the copy on pydigger.com (2024-04-26); the text itself
  could not be opened
- **Last checked:** 2026-11-05

## Content

The README (as shown in the snippet) says the project is based on pyvisgraph and that, "in reviewing projects such as
pivisgraph and extremitypathfinder", the main issue was that these programs "operate with approximate values"
(`acos`, `atan`, divisions).

## Maintenance notes

- Relevant for the numerics argument of the JOSS paper and the floating point audit (#3): since 3.0 the contact tests
  are exact cross product tests (`CHANGELOG.rst`), the other computations use `float64`.
- Do not cite: the source is not available anymore.
