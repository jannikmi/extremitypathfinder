from extremitypathfinder.extremitypathfinder import PolygonEnvironment as Environment

# enable for plotting:
# from extremitypathfinder.plotting import PlottingEnvironment as Environment


if __name__ == "__main__":
    environment = Environment()

    # counter clockwise vertex numbering!
    boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]

    # clockwise numbering!
    list_of_holes = [[(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0), ]]
    # environment.store(boundary_coordinates, list_of_holes, validate=True, export_plots=True)
    environment.store(boundary_coordinates, list_of_holes, validate=False)

    environment.prepare()

    # environment.export_pickle()

    # from extremitypathfinder.extremitypathfinder import load_pickle
    # environment = load_pickle()

    start_coordinates = (4.5, 1.0)
    goal_coordinates = (4.0, 8.5)
    path, length = environment.find_shortest_path(start_coordinates, goal_coordinates)
    print(path, length)

    # grid world
    size_x, size_y = 19, 10
    obstacle_iter = [
        # obstacles changing boundary
        # (x,y),
        (0, 1),
        (1, 1),
        (2, 1),
        (3, 1),

        (17, 9),
        (17, 8),
        (17, 7),

        (17, 5),
        (17, 4),
        (17, 3),
        (17, 2),
        (17, 1),
        (17, 0),

        # hole 1
        (5, 5),
        (5, 6),
        (6, 6),
        (6, 7),
        (7, 7),

        # hole 2
        (7, 5),
    ]

    environment.store_grid_world(size_x, size_y, obstacle_iter, simplify=False, validate=False)
    environment.prepare()
    start_coordinates, goal_coordinates = (0.5, 6), (18.5, 0.5)
    path, length = environment.find_shortest_path(start_coordinates, goal_coordinates)
    print(path, length)
