"""Concurrent use from several threads and free-threaded CPython (#254)

Queries do not write to the environment (invariant, #136): the same queries run from several threads on one
environment give the same results as run one after another. Environments constructed in several threads at once
are identical. On free-threaded CPython (``python3.14t``) the threads run in parallel, as long as no imported
extension module re-enables the GIL, which ``test_free_threading`` checks in the tox environments ``py314t*``.
"""

import itertools
import os
import sys
import sysconfig
import threading
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from extremitypathfinder import PolygonEnvironment
from tests.test_random_queries import ENV_IDS, ENVIRONMENTS, _random_query_points

NR_THREADS = 8


@pytest.fixture(autouse=True)
def _frequent_thread_switches():
    """with the GIL: switch threads as often as possible, so that the queries interleave"""
    interval = sys.getswitchinterval()
    sys.setswitchinterval(1e-5)
    yield
    sys.setswitchinterval(interval)


def _run_in_threads(fct, nr_threads=NR_THREADS):
    """calls ``fct()`` in several threads at once and returns their results"""
    barrier = threading.Barrier(nr_threads)

    def run():
        barrier.wait()
        return fct()

    with ThreadPoolExecutor(max_workers=nr_threads) as executor:
        futures = [executor.submit(run) for _ in range(nr_threads)]
        return [future.result() for future in futures]


def _graph_state(environment):
    graph = environment.graph
    return sorted(graph.nodes), sorted(
        (min(u, v), max(u, v), w) for u, v, w in graph.edges(data="weight")
    )


def test_free_threading():
    if os.environ.get("EXPECT_FREE_THREADING") is None:
        pytest.skip("the free-threading assertion is configured by tox")
    assert sysconfig.get_config_var("Py_GIL_DISABLED") == 1
    # all test modules are imported during the collection: none of the extension modules
    # (NumPy, Numba, Matplotlib, Shapely, ...) re-enabled the GIL
    assert not sys._is_gil_enabled()


@pytest.mark.parametrize("env_id", ENV_IDS)
def test_concurrent_queries(env_id):
    environment = PolygonEnvironment(*ENVIRONMENTS[env_id])
    graph_before = _graph_state(environment)
    points = _random_query_points(ENVIRONMENTS[env_id], seed=ENV_IDS.index(env_id))
    pairs = list(itertools.product(points, repeat=2))

    def queries():
        paths = [environment.find_shortest_path(start, goal) for start, goal in pairs]
        return paths, environment.distance_matrix(points)

    expected_paths, expected_matrix = queries()
    for paths, matrix in _run_in_threads(queries):
        assert paths == expected_paths
        np.testing.assert_array_equal(matrix, expected_matrix)
    assert _graph_state(environment) == graph_before


def test_concurrent_construction():
    def construct_all():
        return [
            _graph_state(PolygonEnvironment(*params))
            for params in ENVIRONMENTS.values()
        ]

    expected = construct_all()
    for graphs in _run_in_threads(construct_all):
        assert graphs == expected
