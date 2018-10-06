from extremitypathfinder.extremitypathfinder import PolygonEnvironment, load_pickle

if __name__ == "__main__":
    environment = PolygonEnvironment()

    # counter clockwise vertex numbering!
    boundary_coordinates = [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)]

    # clockwise numbering!
    list_of_holes = [[(3.0, 7.0), (5.0, 9.0), (4.5, 7.0), (5.0, 4.0), ], ]
    environment.store(boundary_coordinates, list_of_holes, validate=True, export_plots=True)
    # environment.store(boundary_coordinates, list_of_holes, validate=False, export_plots=False)

    environment.prepare(export_plots=True)

    # environment.export_pickle()
    # environment = load_pickle()

    start_coordinates = (4.5, 1.0)
    goal_coordinates = (4.0, 8.5)
    # path, length = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=False)
    path, length = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=True)
    print(path)

    size_x, size_y = 19, 10
    obstacle_iter = [
        # (x,y),

        # obstacles changing boundary
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
    #
    # size_x, size_y = 5,4
    # obstacle_iter = [
    #     # (x,y),
    #
    #     # obstacles changing boundary
    #     (3, 0),
    #     (3, 1),
    #
    #     # hole 1
    #     (1,2),
    #
    # ]

    # environment.store_grid_world(size_x, size_y, obstacle_iter, simplify=False, validate=False, export_plots=True)
    # environment.prepare(export_plots=True)
    # # environment.export_pickle()
    # # environment = load_pickle()
    # start_coordinates, goal_coordinates = ((0.5, 6), (9,6))
    # # start_coordinates, goal_coordinates = ((0.5, 6), (18.5, 0.5))
    #
    # # start_coordinates = (17,9.0)
    # # goal_coordinates = (17,0.5)
    # # path, length = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=False)
    # path, length = environment.find_shortest_path(start_coordinates, goal_coordinates, export_plots=True)
    # print(path)
