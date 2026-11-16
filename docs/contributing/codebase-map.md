# Codebase map

Where to make a change. The [Engineering Design](../8_design.rst) page explains the architecture,
the data model and the algorithms in detail.

## Repository

| Path | Content |
| --- | --- |
| `extremitypathfinder/` | the package (flat layout, no `src/`) |
| `tests/` | the test suite (cf. [Testing](testing.md)) |
| `docs/` | Sphinx documentation (reStructuredText, the contributing guide in Markdown) |
| `scripts/` | release tooling, documentation figures, speed benchmarks |
| `benchmarks/` | comparison with pyvisgraph (cf. [Performance](performance.md)), not part of the package |
| `prototypes/` | development scripts, e.g. profiling. Not part of the package, not tested |
| `example.json` | example input of the command line interface (used by the tests and the docs) |
| `.github/workflows/` | CI (`build.yml`) and the release automation |

## Package modules

The modules form layers. Each module only imports from the layers below it:

```text
command_line, plotting       user interfaces
        │
extremitypathfinder          PolygonEnvironment
        │
queries                      search, distance matrix, SearchResult
        │
utils                        visibility, graph, A*, validation
        │
wedges                       points and free wedges
        │
li_chao_tree                 Li Chao tree of edges over the candidates
        │
utils_numba                  numerical kernels
        │
numba | numba_replacements
```

| Module | Change it to … |
| --- | --- |
| `__init__.py` | change the public API (`PolygonEnvironment`, `SearchResult`, `load_pickle`) |
| `extremitypathfinder.py` | change the public methods of `PolygonEnvironment` (it owns all state, checks the input and delegates) |
| `queries.py` | change how a query is run: start and goal visibility, graph search, distance matrix, `SearchResult` |
| `utils.py` | change input validation and orientation, visibility (`find_visible()`), the graph, A* (`astar_path()`), grids, JSON and pickle I/O |
| `wedges.py` | change what happens at vertices: points, free wedges, which vertices are extremities |
| `li_chao_tree.py` | change how AREA finds the candidates behind an edge: the Li Chao tree (`blocked_candidates()`) |
| `utils_numba.py` | change the innermost numerical loops (angle representations, intersection tests) |
| `numba_replacements.py` | change the stand-ins used when Numba is not installed |
| `plotting.py` | change the drawings and plot exports (Matplotlib) |
| `command_line.py` | change the console command |
| `configs.py`, `types.py` | add constants or type aliases |

**Keep the root module lean.** `extremitypathfinder.py` holds the public API: the methods of `PolygonEnvironment`
with their docstrings, input checks and the state. Algorithms go into the module of their layer as functions that
take the data they need (e.g. `utils.compute_graph()`, `queries.search()`), and the method only delegates.
Add a new module of the right layer for a new group of algorithms instead of growing the root module or `utils.py`.

## Where the time goes

Most of the query time is spent on the visibility of start and goal (`utils.find_visible()`).
The A* search takes only a small part (cf. [Performance](performance.md)).
