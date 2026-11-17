.. _upgrading:

=========
Upgrading
=========

This page lists what to change in code written for an earlier major version.
The :ref:`changelog <changelog>` lists all changes.

* **From 3.x:** follow :ref:`upgrading-4`.
* **From 2.x:** port your code directly to 4.0. First follow :ref:`upgrading-from-2`,
  then :ref:`upgrading-4`. Version 3 changed how an environment is set up and how touching obstacles are handled.
  The supported Python and NumPy versions were raised in 2.8.0 (cf. :ref:`getting started <getting_started>`).
  Code that has to keep running on older versions can stay on its pinned 2.x release.


.. _upgrading-4:

Upgrading from 3.x to 4.0
=========================

Version 4 stores the visibility graph as plain dicts, redesigns the plotting module, replaces pickles with a stable
file format and validates the input by default. Paths and lengths stay the same, except where bugs were fixed
(cf. the :ref:`changelog <changelog>`).


Visibility graph
----------------

``env.graph`` and ``result.graph`` are dicts instead of ``networkx.Graph`` objects, and NetworkX is no dependency anymore.
Install it with ``pip install "extremitypathfinder[networkx]"`` to convert the graphs (cf. :ref:`usage`).

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - 2.x and 3.x
     - 4.0
   * - ``env.graph`` (a ``networkx.Graph``), e.g. ``nx.connected_components(env.graph)``
     - ``env.to_networkx()``, e.g. ``nx.connected_components(env.to_networkx())``
   * - ``result.graph`` (a ``networkx.Graph``)
     - ``result.to_networkx()``
   * - ``env.graph.nodes``
     - ``env.graph`` (the keys)
   * - ``env.graph.neighbors(node)``, ``env.graph[node][neighbour]["weight"]``
     - ``env.graph[node]``, ``env.graph[node][neighbour]``
   * - ``env.graph.edges(data="weight")``
     - ``env.edges()``

Code that imports ``networkx`` itself needs it installed: it no longer comes with this package.


Plotting
--------

The plotting functions draw onto Matplotlib axes instead of the global Matplotlib state.
The figure can then be adjusted, shown and saved with Matplotlib (cf. :ref:`usage`).

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - 3.x
     - 4.0
   * - ``draw_with_path()``, ``draw_prepared_map()``, ``draw_graph()``, ``draw_path()`` and the other ``draw_*()``
       helpers, ``mark_points()``, ``set_limits()``
     - ``ax = draw(env, path)``, ``ax = draw(env, graph=False)`` without the visibility graph,
       ``ax = draw_search(env, result)`` for a search
   * - ``export_plot()``, ``get_plot_name()``, ``SHOW_PLOTS``
     - ``ax.figure.savefig(...)``, ``matplotlib.pyplot.show()``
   * - ``POLYGON_SETTINGS``, the ``EXPORT_*`` constants
     - ``style=PlotStyle(...)`` (colors and sizes), a parameter of all plotting functions
   * - ``plot_environment()``: two files (with and without the visibility graph)
     - one file with the visibility graph, its path is returned. Without the graph: ``draw(env, graph=False)``
   * - ``plot_search()``: up to three files
     - one file, its path is returned

``plot_environment()`` and ``plot_search()`` name their files ``environment_<timestamp>.svg`` and
``search_<timestamp>.svg`` unless a ``file_name`` is given (its extension selects the format).
The drawings look different: obstacles are filled grey, the visibility graph is blue and the path orange.


Stored environments
-------------------

``export_pickle()`` and ``load_pickle()`` are removed (also the import ``from extremitypathfinder import load_pickle``).
Store environments with ``environment.save(path)`` and load them with ``PolygonEnvironment.load(path)``:
the file format does not depend on the internal attributes, so later versions can load it (cf. :ref:`usage <usage>`).

Environments pickled with 3.x cannot be loaded anymore: ``pickle.load()`` raises a ``ValueError`` for them.
Recreate them from their polygons and store them with ``save()``.
Environments can still be pickled and copied (e.g. for worker processes),
but such pickles can only be loaded by the same version of this package.


Input validation
----------------

The input is validated by default: ``PolygonEnvironment()``, ``from_grid()``, ``from_shapely()``
and the command line interface use ``validate=True`` (in 3.x the default was ``validate=False``).
Input that violates the requirements (e.g. self intersecting polygons, collinear overlapping edges)
gave wrong results without an error before, e.g. no path although start and goal are in the free space.
It now raises a ``ValueError`` naming the polygon and the location.
Repair the input (cf. the ``shapely.make_valid()`` recipe of the usage page)
or pass ``validate=False`` to skip the validation for trusted input.

Malformed input is always rejected, also with ``validate=False`` and with ``verify=False``:

* polygons with fewer than 3 coordinate pairs or ragged lists (``TypeError``),
  non-finite coordinates (``ValueError``)
* start and goal points that do not consist of two values (``TypeError``) or are not finite (``ValueError``),
  also in ``distance_matrix()``
* ``from_grid()``: sizes that are no positive integers, blocked cells that are no integer pairs or lie outside of the
  grid (previously ignored: remove them from the input) and grids without any free cell
  (``TypeError`` or ``ValueError``)


Return values
-------------

``find_shortest_path()`` and ``search()`` always return the path as a list of ``(x, y)`` tuples of Python floats
and the length as a Python float. Previously they were ``numpy.float64`` values, and a goal directly visible from
the start returned the passed objects unchanged (e.g. tuples of ints, lists or arrays).
The values are the same. Only code that depends on the types needs to change.


.. _upgrading-from-2:

Upgrading from 2.x to 3.0
=========================

Then continue with :ref:`upgrading-4`.


Initialisation
--------------

The environment is set up in one step: the constructor stores the polygons and computes the visibility graph.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - 2.x
     - 3.0 and later
   * - ``env = PolygonEnvironment()``

       ``env.store(boundary, holes, validate=False)``
     - ``env = PolygonEnvironment(boundary, holes, validate=False)``
   * - ``env.store_grid_world(size_x, size_y, obstacle_iter, simplify=False)``
     - ``env = PolygonEnvironment.from_grid(size_x, size_y, obstacle_iter)``
   * - ``utils.convert_gridworld(size_x, size_y, obstacle_iter, simplify=False)``
     - ``utils.convert_grid(size_x, size_y, obstacle_iter)``
   * - ``env.prepare()``, ``env.prepared``
     - removed: an environment is always ready for queries (``store()`` already prepared it since 2.5.0)

The ``simplify`` parameter is gone. Simplify the polygons beforehand if needed, e.g. with Shapely.


Queries
-------

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - 2.x
     - 3.0 and later
   * - ``env.find_shortest_path(start, goal, free_space_after=True)``
     - ``env.find_shortest_path(start, goal)``
   * - ``env.find_shortest_path(start, goal, free_space_after=False)``,
       then reading ``env.temp_graph``
     - ``result = env.search(start, goal)``, then reading ``result.graph``
       (also ``result.path``, ``result.length``, ``result.idx_start``, ``result.idx_goal``)
   * - ``env.extremity_mask`` (boolean array over all vertices)
     - ``mask = np.zeros(env.nr_vertices, dtype=bool)``

       ``mask[env.extremity_indices] = True``

       or ``env.points`` for the free space around every vertex position

Queries no longer store any data in the environment.
``verify`` is a keyword-only parameter of ``find_shortest_path()`` and ``search()``.
A 2.x call passing ``free_space_after`` or ``verify`` positionally,
e.g. ``env.find_shortest_path(start, goal, False)``, raises a ``TypeError``:
drop the ``free_space_after`` argument and pass ``verify`` by keyword.

Coincident vertices are no longer merged into one extremity: ``extremity_indices`` holds at most one
vertex per position, and vertices at contacts are only extremities when the free angle there is larger than
180 degree. The extremities and the nodes of ``env.graph`` can therefore differ from 2.x.


Plotting
--------

``PlottingEnvironment`` is removed. Create a ``PolygonEnvironment`` and pass it to the plotting functions.
The plotting directory is an argument of these functions instead of a global setting.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - 2.x
     - 3.0 and later
   * - ``env = PlottingEnvironment(plotting_dir="plots")``

       ``env.store(boundary, holes)``
     - ``env = PolygonEnvironment(boundary, holes)``

       ``plot_environment(env, plotting_dir="plots")``
   * - ``env.find_shortest_path(start, goal)``
     - ``result = env.search(start, goal)``

       ``plot_search(env, result, plotting_dir="plots")``

Both functions are in ``extremitypathfinder.plotting``.
Version 4.0 changed the plotting module again (cf. :ref:`upgrading-4`).


Pickled environments
--------------------

Environments pickled with 2.x cannot be loaded anymore: ``pickle.load()`` raises a ``ValueError`` for them;
pickles of a 2.x ``PlottingEnvironment`` fail earlier, because that class no longer exists.
Recreate the environments from their polygons. Since 4.0, store them with ``save()`` instead of pickling them.


Different results: touching obstacles block paths
-------------------------------------------------

Code may run unchanged but return different paths. Obstacles (the holes and everything outside the
boundary polygon) that touch now form a wall:

* a path never passes between two obstacles that touch (shared vertices, a vertex on an edge,
  overlapping edges, holes touching the boundary), and never through a point where an obstacle touches itself.
  In 2.x such contacts did not block paths.
* grids: paths no longer cut corners between diagonally adjacent blocked cells
* paths no longer run through a hole along collinear edges and reflex vertices (a bug in 2.x)

A path can therefore be longer than before, or not exist at all (``([], None)``).
Only exact contacts block, tiny gaps between obstacles do not.
To keep a passage open, leave a gap between the obstacles.

``validate=True`` rejects more input than in 2.x:
polygons crossing themselves at a vertex and collinear overlapping edges within a polygon
(zero-width spikes or slits) are invalid. Since 4.0 the input is validated by default (cf. :ref:`upgrading-4`).
