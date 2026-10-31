.. _upgrading:

=====================
Upgrading from 2.x
=====================

Version 3 changes how an environment is set up and how touching obstacles are handled.
This page lists what to change in code written for version 2.x.
The :ref:`changelog <changelog>` lists all changes.


Initialisation
--------------

The environment is set up in one step: the constructor stores the polygons and computes the visibility graph.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - 2.x
     - 3.x
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
     - 3.x
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
     - 3.x
   * - ``env = PlottingEnvironment(plotting_dir="plots")``

       ``env.store(boundary, holes)``
     - ``env = PolygonEnvironment(boundary, holes)``

       ``plot_environment(env, plotting_dir="plots")``
   * - ``env.find_shortest_path(start, goal)``
     - ``result = env.search(start, goal)``

       ``plot_search(env, result, plotting_dir="plots")``

Both functions are in ``extremitypathfinder.plotting``. Each exports one file and returns its path.
``draw()`` and ``draw_search()`` draw onto Matplotlib axes instead (cf. :ref:`usage`).


Pickled environments
--------------------

Environments pickled with 2.x cannot be loaded anymore.
``load_pickle()`` raises a ``ValueError`` for them; pickles of a 2.x ``PlottingEnvironment``
fail earlier, because that class no longer exists.
Recreate the environments from their polygons and export them again with ``export_pickle()``.


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

Inputs that were accepted before may be rejected with ``validate=True``:
polygons crossing themselves at a vertex and collinear overlapping edges within a polygon
(zero-width spikes or slits) are invalid.
