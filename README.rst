===================
extremitypathfinder
===================


.. image:: https://api.travis-ci.org/MrMinimal64/extremitypathfinder.svg?branch=master
    :target: https://travis-ci.org/MrMinimal64/extremitypathfinder

.. image:: https://readthedocs.org/projects/extremitypathfinder/badge/?version=latest
    :alt: documentation status
    :target: https://extremitypathfinder.readthedocs.io/en/latest/?badge=latest

.. image:: https://img.shields.io/pypi/wheel/extremitypathfinder.svg
    :target: https://pypi.python.org/pypi/extremitypathfinder

.. image:: https://pepy.tech/badge/extremitypathfinder
    :alt: Total PyPI downloads
    :target: https://pepy.tech/project/extremitypathfinder

.. image:: https://img.shields.io/pypi/v/extremitypathfinder.svg
    :alt: latest version on PyPI
    :target: https://pypi.python.org/pypi/extremitypathfinder


python package for fast geometric shortest path computation in 2D multi-polygon or grid environments based on visibility graphs.


.. image:: ./docs/_static/title_demo_plot.png


Quick Guide:

::


    pip install extremitypathfinder


.. code-block:: python

    from extremitypathfinder import PolygonEnvironment
    environment = PolygonEnvironment()
    # counter clockwise vertex numbering!
    boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]
    # clockwise numbering!
    list_of_holes = [[(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0), ], ]
    environment.store(boundary_coordinates, list_of_holes, validate=False)
    environment.prepare()
    start_coordinates = (4.5, 1.0)
    goal_coordinates = (4.0, 8.5)
    path, length = environment.find_shortest_path(start_coordinates, goal_coordinates)


For more refer to the `documentation <https://extremitypathfinder.readthedocs.io/en/latest/>`__.


Also see:
`GitHub <https://github.com/MrMinimal64/extremitypathfinder>`__,
`PyPI <https://pypi.python.org/pypi/extremitypathfinder/>`__
