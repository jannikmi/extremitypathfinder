"""Contacts between obstacles block paths (#24).

Obstacles are the holes and everything outside the boundary polygon.
Obstacles that touch form a wall: a path never passes between two obstacles that touch,
and never through a point where an obstacle touches itself.
Every environment below has a query whose shortest path would pass through the contact
if contacts did not block.
"""

from math import hypot, sqrt

import pytest

from extremitypathfinder import PolygonEnvironment
from tests.test_geometry_regressions import _assert_valid_path

BOUNDARY = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]

# two holes sharing the vertex (5, 5): the upper left one is smaller
SHARED_VERTEX_HOLES = [
    [(3.0, 5.0), (3.0, 7.0), (5.0, 7.0), (5.0, 5.0)],
    [(5.0, 2.0), (5.0, 5.0), (8.0, 5.0), (8.0, 2.0)],
]
# the same shape as a single hole touching itself at (5, 5)
SELF_TOUCHING_HOLE = [
    (3.0, 5.0),
    (3.0, 7.0),
    (5.0, 7.0),
    (5.0, 5.0),
    (8.0, 5.0),
    (8.0, 2.0),
    (5.0, 2.0),
    (5.0, 5.0),
]
# around the smaller hole
SHARED_VERTEX_DETOUR = (
    [(4.0, 4.0), (3.0, 5.0), (3.0, 7.0), (5.0, 7.0), (6.0, 6.0)],
    4.0 + 2.0 * sqrt(2.0),
)

# the tip (5, 5) of the lower triangle lies on the bottom edge of the square
VERTEX_ON_EDGE_HOLES = [
    [(3.0, 5.0), (3.0, 7.0), (7.0, 7.0), (7.0, 5.0)],
    [(5.0, 5.0), (6.0, 3.0), (4.0, 3.0)],
]

# the two holes share the edge x = 5 for 4 <= y <= 5, the right one is wider
OVERLAPPING_EDGES_HOLES = [
    [(3.0, 4.0), (3.0, 6.0), (5.0, 6.0), (5.0, 4.0)],
    [(5.0, 3.0), (5.0, 5.0), (8.0, 5.0), (8.0, 3.0)],
]

# two thin triangles touching at their tips (5, 5): a small free pocket opens to the left,
# the other free wedge at (5, 5) is larger than 180 degree (an extremity)
TOUCHING_TIPS_HOLES = [
    [(5.0, 5.0), (2.0, 6.0), (2.0, 7.0)],
    [(5.0, 5.0), (2.0, 2.5), (2.0, 4.0)],
]


@pytest.mark.parametrize(
    ("holes", "start", "goal", "expected_path", "expected_length"),
    [
        pytest.param(
            SHARED_VERTEX_HOLES,
            (4.0, 4.0),
            (6.0, 6.0),
            *SHARED_VERTEX_DETOUR,
            id="holes sharing a vertex",
        ),
        pytest.param(
            [SELF_TOUCHING_HOLE],
            (4.0, 4.0),
            (6.0, 6.0),
            *SHARED_VERTEX_DETOUR,
            id="hole touching itself at a vertex",
        ),
        pytest.param(
            VERTEX_ON_EDGE_HOLES,
            (1.0, 4.0),
            (9.0, 4.0),
            [(1.0, 4.0), (4.0, 3.0), (6.0, 3.0), (9.0, 4.0)],
            2.0 + 2.0 * sqrt(10.0),
            id="vertex on the edge of another hole",
        ),
        pytest.param(
            OVERLAPPING_EDGES_HOLES,
            (5.0, 1.0),
            (5.0, 8.0),
            [(5.0, 1.0), (3.0, 4.0), (3.0, 6.0), (5.0, 8.0)],
            sqrt(13.0) + 2.0 + 2.0 * sqrt(2.0),
            id="holes with overlapping collinear edges",
        ),
        pytest.param(
            TOUCHING_TIPS_HOLES,
            (3.0, 5.0),
            (6.0, 5.0),
            [(3.0, 5.0), (2.0, 6.0), (2.0, 7.0), (6.0, 5.0)],
            sqrt(2.0) + 1.0 + 2.0 * sqrt(5.0),
            id="out of a pocket between touching holes",
        ),
        pytest.param(
            TOUCHING_TIPS_HOLES,
            (4.0, 7.0),
            (4.0, 3.0),
            [(4.0, 7.0), (5.0, 5.0), (4.0, 3.0)],
            2.0 * sqrt(5.0),
            id="around the larger free wedge of a contact",
        ),
    ],
)
def test_contacts_between_holes_block(
    holes, start, goal, expected_path, expected_length
):
    environment = PolygonEnvironment(BOUNDARY, holes, validate=True)
    for query, path_expected in (
        ((start, goal), expected_path),
        ((goal, start), list(reversed(expected_path))),
    ):
        path, length = environment.find_shortest_path(*query)
        assert path == path_expected
        assert length == pytest.approx(expected_length)
        _assert_valid_path(path, length, holes)


def test_hole_touching_the_boundary_at_a_vertex_blocks():
    # the tip (5, 1) of the triangle coincides with a vertex of the boundary
    boundary = [(0.0, 0.0), (5.0, 1.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    holes = [[(5.0, 1.0), (4.0, 4.0), (6.0, 4.0)]]
    environment = PolygonEnvironment(boundary, holes, validate=True)

    path, length = environment.find_shortest_path((2.0, 1.5), (8.0, 1.5))

    assert path == [(2.0, 1.5), (4.0, 4.0), (6.0, 4.0), (8.0, 1.5)]
    assert length == pytest.approx(2.0 + 2.0 * hypot(2.0, 2.5))


def test_hole_vertex_on_the_boundary_edge_blocks():
    # the tip (5, 0) of the triangle lies on the bottom edge of the boundary
    holes = [[(5.0, 0.0), (4.0, 3.0), (6.0, 3.0)]]
    environment = PolygonEnvironment(BOUNDARY, holes, validate=True)

    path, length = environment.find_shortest_path((2.0, 0.5), (8.0, 0.5))

    assert path == [(2.0, 0.5), (4.0, 3.0), (6.0, 3.0), (8.0, 0.5)]
    assert length == pytest.approx(2.0 + 2.0 * hypot(2.0, 2.5))


def test_hole_edge_on_the_boundary_edge_blocks():
    # the bottom edge of the square hole lies on the bottom edge of the boundary
    holes = [[(4.0, 0.0), (4.0, 2.0), (6.0, 2.0), (6.0, 0.0)]]
    environment = PolygonEnvironment(BOUNDARY, holes, validate=True)

    path, length = environment.find_shortest_path((2.0, 0.0), (8.0, 0.0))

    assert path == [(2.0, 0.0), (4.0, 2.0), (6.0, 2.0), (8.0, 0.0)]
    assert length == pytest.approx(2.0 + 4.0 * sqrt(2.0))


def test_hole_touching_two_boundary_edges_separates_the_map():
    # https://github.com/jannikmi/extremitypathfinder/issues/23:
    # the hole lies on the left and right edges of the boundary
    boundary = [(5.0, 5.0), (-5.0, 5.0), (-5.0, -5.0), (5.0, -5.0)]
    holes = [[(-5.0, 1.0), (-5.0, 2.0), (5.0, 2.0), (5.0, 1.0)]]
    environment = PolygonEnvironment(boundary, holes, validate=True)

    assert environment.find_shortest_path((0.0, 0.0), (0.0, 4.0)) == ([], None)


def test_boundary_touching_itself_separates_the_map():
    # two squares of free space touching at (5, 5)
    boundary = [
        (0.0, 0.0),
        (5.0, 0.0),
        (5.0, 5.0),
        (10.0, 5.0),
        (10.0, 10.0),
        (5.0, 10.0),
        (5.0, 5.0),
        (0.0, 5.0),
    ]
    environment = PolygonEnvironment(boundary, [], validate=True)

    assert environment.find_shortest_path((1.0, 1.0), (9.0, 9.0)) == ([], None)
    path, length = environment.find_shortest_path((1.0, 1.0), (4.0, 2.0))
    assert path == [(1.0, 1.0), (4.0, 2.0)]


def test_hole_vertex_touching_its_own_edge_encloses_a_pocket():
    # the vertex (7, 6) lies on the edge (8, 6) -> (4, 6) of the same hole:
    # the pocket left of it is enclosed, the pocket right of it opens to the right
    hole = [
        (2.0, 2.0),
        (2.0, 8.0),
        (8.0, 8.0),
        (8.0, 6.0),
        (4.0, 6.0),
        (4.0, 4.0),
        (6.0, 4.0),
        (7.0, 6.0),
        (8.0, 4.0),
        (8.0, 2.0),
    ]
    environment = PolygonEnvironment(BOUNDARY, [hole], validate=True)

    assert environment.find_shortest_path((5.0, 5.0), (7.5, 5.5)) == ([], None)
    assert environment.find_shortest_path((5.0, 5.0), (9.0, 5.0)) == ([], None)
    path, _ = environment.find_shortest_path((7.5, 5.5), (9.0, 5.0))
    assert path == [(7.5, 5.5), (9.0, 5.0)]


def test_grid_diagonal_obstacles_block():
    # the blocked cells (1, 1) and (2, 2) touch diagonally at (2, 2), no corner cutting
    environment = PolygonEnvironment.from_grid(5, 4, [(1, 1), (2, 2), (3, 2)])

    path, length = environment.find_shortest_path((2.5, 1.5), (1.5, 2.5))

    assert path == [(2.5, 1.5), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (1.5, 2.5)]
    assert length == pytest.approx(2.0 + sqrt(2.0))


def test_hole_touching_the_boundary_from_outside_has_no_effect():
    # the hole lies outside of the boundary and touches its vertex (5, 1):
    # it is part of the outside obstacle, the path still bends at (5, 1)
    boundary = [(0.0, 0.0), (5.0, 1.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    holes = [[(5.0, 1.0), (6.0, -1.0), (4.0, -1.0)]]
    environment = PolygonEnvironment(boundary, holes, validate=True)

    path, length = environment.find_shortest_path((2.0, 0.5), (8.0, 0.5))

    assert path == [(2.0, 0.5), (5.0, 1.0), (8.0, 0.5)]
    assert length == pytest.approx(2.0 * hypot(3.0, 0.5))
    assert environment.all_extremities.count((5.0, 1.0)) == 1


def test_near_contacts_do_not_block():
    # the lower right hole is shifted right by 1e-9: there is a tiny gap at (5, 5)
    holes = [
        SHARED_VERTEX_HOLES[0],
        [(x + 1e-9, y) for x, y in SHARED_VERTEX_HOLES[1]],
    ]
    environment = PolygonEnvironment(BOUNDARY, holes, validate=True)

    path, length = environment.find_shortest_path((4.0, 4.0), (6.0, 6.0))

    # through the gap, no detour around one of the holes
    assert len(path) <= 3
    assert length == pytest.approx(2.0 * sqrt(2.0))


def test_paths_along_the_edges_of_a_single_hole_stay_possible():
    environment = PolygonEnvironment(BOUNDARY, SHARED_VERTEX_HOLES[:1], validate=True)

    path, length = environment.find_shortest_path((1.0, 5.0), (9.0, 5.0))

    # along the bottom edge and through the lower right vertex (5, 5) of the hole
    assert path == [(1.0, 5.0), (3.0, 5.0), (5.0, 5.0), (9.0, 5.0)]
    assert length == pytest.approx(8.0)


@pytest.mark.parametrize(
    ("holes", "contact", "goals"),
    [
        (SHARED_VERTEX_HOLES, (5.0, 5.0), [(4.0, 4.0), (6.0, 6.0)]),
        ([SELF_TOUCHING_HOLE], (5.0, 5.0), [(4.0, 4.0), (6.0, 6.0)]),
        (VERTEX_ON_EDGE_HOLES, (5.0, 5.0), [(4.0, 4.5), (6.0, 4.5)]),
        (TOUCHING_TIPS_HOLES, (5.0, 5.0), [(3.0, 5.0), (6.0, 5.0)]),
    ],
)
def test_query_on_a_contact_may_use_any_free_wedge(holes, contact, goals):
    environment = PolygonEnvironment(BOUNDARY, holes, validate=True)
    for goal in goals:
        expected = hypot(goal[0] - contact[0], goal[1] - contact[1])
        assert environment.find_shortest_path(contact, goal) == (
            [contact, goal],
            pytest.approx(expected),
        )
        assert environment.find_shortest_path(goal, contact) == (
            [goal, contact],
            pytest.approx(expected),
        )


def test_query_inside_a_wall_has_no_path():
    # (5, 4.5) lies on the edge shared by both holes
    environment = PolygonEnvironment(BOUNDARY, OVERLAPPING_EDGES_HOLES, validate=True)

    assert environment.find_shortest_path((5.0, 4.5), (5.0, 1.0)) == ([], None)
    assert environment.find_shortest_path((5.0, 1.0), (5.0, 4.5)) == ([], None)


def test_contact_without_a_large_free_wedge_is_no_graph_node():
    environment = PolygonEnvironment(BOUNDARY, SHARED_VERTEX_HOLES, validate=True)
    assert (5.0, 5.0) not in environment.all_extremities
    # 8 square corners minus the 2 corners at the contact
    assert len(environment.all_extremities) == 6
    assert environment.graph.number_of_nodes() == 6


def test_contact_with_a_large_free_wedge_is_a_single_graph_node():
    environment = PolygonEnvironment(BOUNDARY, TOUCHING_TIPS_HOLES, validate=True)
    assert environment.all_extremities.count((5.0, 5.0)) == 1
    assert len(environment.all_extremities) == 5
    assert environment.graph.number_of_nodes() == 5
