"""networkx is an optional dependency (extra ``networkx``, #182): only the conversions of the graphs need it"""

import subprocess
import sys

import networkx as nx
import pytest

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.utils import graph_edges
from tests.main_test import DIRECT_QUERY_POLY_ENV
from tests.test_cases import POLY_ENV_PARAMS, TEST_DATA_POLY_ENV


def _assert_same_graph(nx_graph, graph):
    assert list(nx_graph.nodes) == list(graph)
    assert {
        node: {neighbour: data["weight"] for neighbour, data in neighbours.items()}
        for node, neighbours in nx_graph.adj.items()
    } == graph


def test_environment_to_networkx():
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    nx_graph = environment.to_networkx()
    assert isinstance(nx_graph, nx.Graph)
    _assert_same_graph(nx_graph, environment.graph)
    assert nx.is_weighted(nx_graph)
    # an independent copy
    nx_graph.clear()
    assert any(environment.graph.values())


def test_empty_graph_to_networkx():
    # a convex boundary alone has no extremities
    environment = PolygonEnvironment([(0, 0), (4, 0), (4, 4), (0, 4)], [])
    assert environment.graph == {}
    assert environment.to_networkx().number_of_nodes() == 0


def test_search_result_to_networkx():
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    indirect_cases = [c for c in TEST_DATA_POLY_ENV if len(c[1][0]) > 2]
    assert indirect_cases
    for (start, goal), (_, length) in indirect_cases:
        result = environment.search(start, goal)
        nx_graph = result.to_networkx()
        _assert_same_graph(nx_graph, result.graph)
        for u, v, weight in result.query_edges:
            assert nx_graph.edges[u, v]["weight"] == weight
        # networkx finds a path of the same length in the search graph
        nx_length = nx.dijkstra_path_length(nx_graph, result.idx_start, result.idx_goal)
        assert nx_length == pytest.approx(length)


def test_search_result_without_graph_search_to_networkx():
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    # the goal is directly visible
    assert environment.search(*DIRECT_QUERY_POLY_ENV).to_networkx() is None


def test_graph_edges_are_each_edge_once():
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    edges = environment.edges()
    assert edges == list(graph_edges(environment.graph))
    # the order of networkx for the same dicts (the order of the file format)
    assert edges == list(environment.to_networkx().edges(data="weight"))
    assert len({frozenset((u, v)) for u, v, _ in edges}) == len(edges)
    assert 2 * len(edges) == sum(map(len, environment.graph.values()))


def test_import_does_not_import_networkx():
    code = (
        "import sys\n"
        "from extremitypathfinder import PolygonEnvironment\n"
        "env = PolygonEnvironment([(0, 0), (4, 0), (4, 4), (0, 4)], [[(1, 1), (3, 1), (2, 3)]])\n"
        "env.search((0.5, 2), (3.5, 2)).graph\n"
        "assert 'networkx' not in sys.modules\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_without_networkx(tmp_path):
    # networkx blocked as if it was not installed: everything but the conversions works
    code = (
        "import sys\n"
        "sys.modules['networkx'] = None\n"
        "from extremitypathfinder import PolygonEnvironment\n"
        "env = PolygonEnvironment([(0, 0), (4, 0), (4, 4), (0, 4)], [[(1, 1), (3, 1), (2, 3)]])\n"
        "result = env.search((0.5, 2), (3.5, 2))\n"
        "assert len(result.path) == 3, result.path\n"
        "assert result.graph\n"
        "env.distance_matrix([(0.5, 2), (3.5, 2)])\n"
        f"env.save({str(tmp_path / 'env.npz')!r})\n"
        f"assert PolygonEnvironment.load({str(tmp_path / 'env.npz')!r}).graph == env.graph\n"
        "for convert in (env.to_networkx, result.to_networkx):\n"
        "    try:\n"
        "        convert()\n"
        "    except ImportError as error:\n"
        "        assert 'extremitypathfinder[networkx]' in str(error), error\n"
        "    else:\n"
        "        raise AssertionError('no ImportError')\n"
    )
    subprocess.run([sys.executable, "-c", code], check=True)


def test_search_result_edges():
    environment = PolygonEnvironment(*POLY_ENV_PARAMS)
    assert environment.search(*DIRECT_QUERY_POLY_ENV).edges() is None
    indirect_cases = [c for c in TEST_DATA_POLY_ENV if len(c[1][0]) > 2]
    for (start, goal), _ in indirect_cases:
        result = environment.search(start, goal)
        edges = result.edges()
        assert edges == list(result.to_networkx().edges(data="weight"))
        # the visibility graph and the query edges, each once, the lengths as Python floats
        expected = {frozenset((u, v)): w for u, v, w in environment.edges()}
        expected |= {frozenset((u, v)): w for u, v, w in result.query_edges}
        assert {frozenset((u, v)): w for u, v, w in edges} == expected
        assert len(edges) == len(expected)
        assert all(type(w) is float for _, _, w in edges)
