# size_x, size_y, obstacle_iter
from math import sqrt

GRID_ENV_PARAMS = (
    19,
    10,
    [
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
    ],
)

INVALID_DESTINATION_DATA = [
    # outside of map region
    ((-1, 5.0), (17, 0.5)),
    ((17, 0.5), (-1, 5.0)),
    ((20, 5.0), (17, 0.5)),
    ((17, 0.5), (20, 5.0)),
    ((1, -5.0), (17, 0.5)),
    ((17, 0.5), (1, -5.0)),
    ((1, 11.0), (17, 0.5)),
    ((17, 0.5), (1, 11.0)),
    # outside boundary polygon
    ((17.5, 5.0), (17, 0.5)),
    ((17, 0.5), (17.5, 5.0)),
    ((1, 1.5), (17, 0.5)),
    ((17, 0.5), (1, 1.5)),
    # inside hole
    ((6.5, 6.5), (17, 0.5)),
    ((17, 0.5), (6.5, 6.5)),
]

TEST_DATA_GRID_ENV = [
    # ((start,goal),(path,distance))
    # shortest paths should be distinct (reverse will automatically be tested)
    # identical nodes
    (((15, 5), (15, 5)), ([(15, 5), (15, 5)], 0.0)),
    # directly reachable
    (((15, 5), (15, 6)), ([(15, 5), (15, 6)], 1.0)),
    (((15, 5), (16, 6)), ([(15, 5), (16, 6)], sqrt(2))),
    # points on the polygon edges (vertices) should be accepted!
    # on edge
    (((15, 0), (15, 6)), ([(15, 0), (15, 6)], 6.0)),
    (((17, 5), (16, 5)), ([(17, 5), (16, 5)], 1.0)),
    # on edge of hole
    (((7, 8), (7, 9)), ([(7, 8), (7, 9)], 1.0)),
    # on vertex
    (((4, 2), (4, 3)), ([(4, 2), (4, 3)], 1.0)),
    # on vertex of hole
    (((6, 8), (6, 9)), ([(6, 8), (6, 9)], 1.0)),
    # on two vertices
    # coinciding with edge (direct neighbour)
    (((4, 2), (4, 1)), ([(4, 2), (4, 1)], 1.0)),
    (((5, 5), (5, 7)), ([(5, 5), (5, 7)], 2.0)),
    # should have direct connection to all visible extremities! connected in graph
    (((6, 8), (5, 7)), ([(6, 8), (5, 7)], sqrt(2))),
    (((4, 1), (5, 7)), ([(4, 1), (5, 7)], sqrt(1**2 + 6**2))),
    # should have direct connection to all visible extremities! even if not connected in graph!
    (((4, 2), (5, 7)), ([(4, 2), (5, 7)], sqrt(1**2 + 5**2))),
    # mix of edges and vertices, directly visible
    (((2, 2), (5, 7)), ([(2, 2), (5, 7)], sqrt(3**2 + 5**2))),
    # also regular points should have direct connection to all visible extremities!
    (((10, 3), (17, 6)), ([(10, 3), (17, 6)], sqrt(7**2 + 3**2))),
    (((10, 3), (8, 8)), ([(10, 3), (8, 8)], sqrt(2**2 + 5**2))),
    # even if the query point lies in front of an extremity! (test if new query vertices are being created!)
    (((10, 3), (8, 5)), ([(10, 3), (8, 5)], sqrt(2**2 + 2**2))),
    # using a* graph search:
    # directly reachable through a single vertex (does not change distance!)
    (((5, 1), (3, 3)), ([(5, 1), (4, 2), (3, 3)], sqrt(2**2 + 2**2))),
    # If two Polygons have vertices with identical coordinates (this is allowed),
    #   paths through these vertices are theoretically possible!
    (
        ((6.5, 5.5), (7.5, 6.5)),
        ([(6.5, 5.5), (7, 6), (7.5, 6.5)], sqrt(1**2 + 1**2)),
    ),
    # distance should stay the same even if multiple extremities lie on direct path
    # test if path is skipping passed extremities
    (((8, 4), (8, 8)), ([(8, 4), (8, 5), (8, 6), (8, 7), (8, 8)], 4)),
    (((8, 4), (8, 9)), ([(8, 4), (8, 5), (8, 6), (8, 7), (8, 8), (8, 9)], 5)),
    # regular examples
    (
        ((0.5, 6), (18.5, 0.5)),
        (
            [(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (17, 6), (18, 6), (18.5, 0.5)],
            23.18783787537749,
        ),
    ),
    (
        ((0.5, 6), (9, 5.5)),
        ([(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (9.0, 5.5)], 8.727806217396338),
    ),
    (
        ((0.5, 6), (18.5, 9)),
        (
            [(0.5, 6.0), (5, 5), (6, 5), (7, 5), (8, 5), (18, 7), (18.5, 9.0)],
            19.869364068640845,
        ),
    ),
    (
        ((6.9, 4), (7, 9)),
        ([(6.9, 4.0), (7, 6), (8, 7), (8, 8), (7, 9)], 5.830925564196269),
    ),
    (
        ((6.5, 4), (7, 9)),
        ([(6.5, 4.0), (7, 6), (8, 7), (8, 8), (7, 9)], 5.889979937555021),
    ),
    # symmetric around the lower boundary obstacle
    (
        ((0.5, 0.5), (0.5, 2.5)),
        ([(0.5, 0.5), (4, 1), (4, 2), (0.5, 2.5)], 8.071067811865476),
    ),
    # symmetric around the lower right boundary obstacle
    (
        ((16.5, 0.5), (18.5, 0.5)),
        ([(16.5, 0.5), (17, 6), (18, 6), (18.5, 0.5)], 12.045361017187261),
    ),
    # symmetric around the top right boundary obstacle
    (
        ((16.5, 9.5), (18.5, 9.5)),
        ([(16.5, 9.5), (17, 7), (18, 7), (18.5, 9.5)], 6.0990195135927845),
    ),
]

POLY_ENV_PARAMS = (
    # boundary_coordinates
    [(0.0, 0.0), (10.0, 0.0), (9.0, 5.0), (10.0, 10.0), (0.0, 10.0)],
    # list_of_holes
    [
        [
            (3.0, 7.0),
            (5.0, 9.0),
            (4.6, 7.0),
            (5.0, 4.0),
        ],
    ],
)

TEST_DATA_POLY_ENV = [
    # # ((start,goal),(path,distance))
    # # identical nodes
    # (((1, 1), (1, 1)), ([(1, 1), (1, 1)], 0.0)),
    # # directly reachable
    # (((1, 1), (1, 2)), ([(1, 1), (1, 2)], 1.0)),
    # (((1, 1), (2, 1)), ([(1, 1), (2, 1)], 1.0)),
    # # points on the polygon edges (vertices) should be accepted!
    # # on edge (boundary polygon)
    # (((1, 0), (1, 1)), ([(1, 0), (1, 1)], 1.0)),
    # (((9.5, 2.5), (8.5, 2.5)), ([(9.5, 2.5), (8.5, 2.5)], 1.0)),
    # (((0, 2), (0, 1)), ([(0, 2), (0, 1)], 1.0)),  # both
    # (((1, 0), (5, 0)), ([(1, 0), (5, 0)], 4.0)),  # both
    # # on edge of hole
    # (((4, 8), (3, 8)), ([(4, 8), (3, 8)], 1.0)),
    # (((4, 8), (4.1, 8.1)), ([(4, 8), (4.1, 8.1)], sqrt(2 * (0.1**2)))),  # both
    # # on vertex
    # (((9, 5), (8, 5)), ([(9, 5), (8, 5)], 1.0)),
    # # on vertex of hole
    # (((3, 7), (2, 7)), ([(3, 7), (2, 7)], 1.0)),
    # # on two vertices
    # # coinciding with edge (direct neighbour)
    # (((3, 7), (5, 9)), ([(3, 7), (5, 9)], sqrt(8))),
    # (((4.6, 7), (5, 9)), ([(4.6, 7), (5, 9)], sqrt((0.4**2) + (2**2)))),
    # # should have direct connection to all visible extremities! connected in graph
    # (((5, 4), (5, 9)), ([(5, 4), (5, 9)], 5)),
    # # should have a direct connection to all visible extremities! even if not connected in graph!
    # (((9, 5), (5, 9)), ([(9, 5), (5, 9)], sqrt(2 * (4**2)))),
    # using a* graph search:
    # directly reachable through a single vertex (does not change distance!)
    (((9, 4), (9, 6)), ([(9, 4), (9, 5), (9, 6)], 2)),
    # slightly indented, path must go through right boundary extremity
    (((9.1, 4), (9.1, 6)), ([(9.1, 4.0), (9.0, 5.0), (9.1, 6.0)], 2.009975124224178)),
    # path must go through lower hole extremity
    (((4, 4.5), (6, 4.5)), ([(4.0, 4.5), (5.0, 4.0), (6.0, 4.5)], 2.23606797749979)),
    # path must go through top hole extremity
    (((4, 8.5), (6, 8.5)), ([(4.0, 8.5), (5.0, 9.0), (6.0, 8.5)], 2.23606797749979)),
]

OVERLAP_POLY_ENV_PARAMS = (
    # boundary_coordinates
    [
        (9.5, 10.5),
        (25.5, 10.5),
        (25.5, 0.5),
        (49.5, 0.5),
        (49.5, 49.5),
        (0.5, 49.5),
        (0.5, 16.5),
        (9.5, 16.5),
        (9.5, 45.5),
        (10.0, 45.5),
        (10.0, 30.5),
        (35.5, 30.5),
        (35.5, 14.5),
        (0.5, 14.5),
        (0.5, 0.5),
        (9.5, 0.5),
    ],
    # list_of_holes
    [
        [
            (40.5, 4.5),
            (29.5, 4.5),
            (29.5, 15.0),
            (40.5, 15.0),
        ],
        [
            (45.4, 14.5),
            (44.6, 14.5),
            (43.4, 20.5),
            (46.6, 20.5),
        ],
        # slightly right of the top boundary obstacle
        # goal: create an obstacle that obstructs two very close extremities
        # to check if visibility is correctly blocked in such cases
        [
            (40, 34),
            (10.5, 34),
            (10.5, 40),
            (40, 40),
        ],
        # on the opposite site close to top boundary obstacle
        [
            (9, 34),
            (5, 34),
            (5, 40),
            (9, 40),
        ],
        [
            (31.5, 5.390098048839718),
            (31.5, 10.909901951439679),
            (42.5, 13.109901951160282),
            (42.5, 7.590098048560321),
        ],
    ],
)

TEST_DATA_OVERLAP_POLY_ENV = [
    # ((start,goal),(path,distance))
    (
        ((1, 1), (5, 20)),
        (
            [
                (1, 1),
                (9.5, 10.5),
                (25.5, 10.5),
                (29.5, 4.5),
                (40.5, 4.5),
                (42.5, 7.590098048560321),
                (42.5, 13.109901951160282),
                (35.5, 30.5),
                (10.5, 34.0),
                (10.0, 45.5),
                (9.5, 45.5),
                (9, 34),
                (5, 20),
            ],
            138.23115155299263,
        ),
    ),
    (
        ((2, 38), (45, 45)),
        ([(2.0, 38.0), (9.5, 45.5), (10.0, 45.5), (45.0, 45.0)], 46.11017296417249),
    ),
    (
        ((2, 38), (45, 2)),
        (
            [
                (2.0, 38.0),
                (9.5, 45.5),
                (10.0, 45.5),
                (10.5, 34.0),
                (35.5, 30.5),
                (42.5, 13.109901951160282),
                (45.0, 2.0),
            ],
            77.99506635830616,
        ),
    ),
    (
        ((2, 38), (38, 2)),
        (
            [
                (2.0, 38.0),
                (9.5, 45.5),
                (10.0, 45.5),
                (10.5, 34.0),
                (35.5, 30.5),
                (42.5, 13.109901951160282),
                (42.5, 7.590098048560321),
                (40.5, 4.5),
                (38.0, 2.0),
            ],
            79.34355163003127,
        ),
    ),
    (
        ((2, 38), (28, 2)),
        (
            [
                (2.0, 38.0),
                (9.5, 45.5),
                (10.0, 45.5),
                (10.5, 34.0),
                (35.5, 30.5),
                (42.5, 13.109901951160282),
                (42.5, 7.590098048560321),
                (40.5, 4.5),
                (28.0, 2.0),
            ],
            88.55556650808049,
        ),
    ),
]

SEPARATED_ENV = (
    [(5, 5), (-5, 5), (-5, -5), (5, -5)],
    [[(-5.1, 1), (-5.1, 2), (5.1, 2), (5.1, 1)]],  # intersecting polygons -> no path possible
    # [[(-5, 1), (-5, 2), (5, 2), (5, 1)]], # hole lies on the edges -> path possible
)

TEST_DATA_SEPARATE_ENV = [
    # ((start,goal),(path,distance))
    (((0, 0), (0, 4)), ([], None)),  # unreachable
]

# ((start,goal),(path,distance))
