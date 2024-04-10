import numpy as np

from extremitypathfinder import utils


def proto_test_case(data, fct):
    for input, expected_output in data:
        # print(input, expected_output, fct(input))
        actual_output = fct(input)
        if actual_output != expected_output:
            print(
                "input: {} expected: {} got: {}".format(
                    input, expected_output, actual_output
                )
            )
        assert actual_output == expected_output


def other_edge_intersects(
    n1: int, n2: int, edge_vertex_idxs: np.ndarray, coords: np.ndarray
) -> bool:
    p1 = coords[n1]
    p2 = coords[n2]
    for i1, i2 in edge_vertex_idxs:
        if i1 == n1 or i2 == n2:
            # no not check the same edge
            continue
        q1 = coords[i1]
        q2 = coords[i2]
        if utils._get_intersection_status(p1, p2, q1, q2) == 1:
            return True
    return False
