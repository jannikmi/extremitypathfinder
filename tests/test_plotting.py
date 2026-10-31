"""Drawing environments and searches onto matplotlib axes and exporting them as files."""

import matplotlib.pyplot as plt
import numpy as np
import pytest
from matplotlib.colors import to_rgba

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import (
    PlotStyle,
    draw,
    draw_search,
    plot_environment,
    plot_search,
)
from scripts.docs_figures import main as generate_figures
from tests.test_cases import POLY_ENV_PARAMS, SEPARATED_ENV, TEST_DATA_SEPARATE_ENV

# start and goal requiring a graph search, and directly visible from each other
INDIRECT_QUERY = ((4.5, 1.0), (4.0, 8.5))
DIRECT_QUERY = ((0.5, 0.5), (1.5, 0.5))


@pytest.fixture
def environment():
    return PolygonEnvironment(*POLY_ENV_PARAMS)


@pytest.fixture(autouse=True)
def close_figures():
    yield
    plt.close("all")


def artists(ax):
    """the drawn elements by their group id"""
    return {
        artist.get_gid(): artist
        for artist in ax.get_children()
        if artist.get_gid() is not None
    }


def test_draws_all_elements(environment):
    path, _ = environment.find_shortest_path((4.5, 1.0), (4.0, 8.5))
    _, ax = plt.subplots()

    assert draw(environment, path, ax=ax, vertices=True) is ax

    drawn = artists(ax)
    assert set(drawn) == {
        "boundary",
        "obstacles",
        "visibility-graph",
        "vertices",
        "extremities",
        "path",
        "start",
        "goal",
    }
    assert len(drawn["obstacles"].get_paths()) == len(environment.holes)
    assert (
        len(drawn["visibility-graph"].get_segments())
        == environment.graph.number_of_edges()
    )
    assert len(drawn["extremities"].get_offsets()) == len(environment.extremity_indices)
    assert len(drawn["vertices"].get_offsets()) == environment.nr_vertices
    np.testing.assert_array_equal(drawn["path"].get_xydata(), np.array(path))
    np.testing.assert_array_equal(drawn["start"].get_offsets(), [path[0]])
    np.testing.assert_array_equal(drawn["goal"].get_offsets(), [path[-1]])
    assert ax.get_aspect() == 1.0


def test_optional_elements(environment):
    ax = draw(environment, graph=False, extremities=False)

    assert set(artists(ax)) == {"boundary", "obstacles"}


def test_unbounded_environment():
    holes = [[(0.0, 2.0), (2.0, 2.0), (2.0, 0.0), (0.0, 0.0)]]
    environment = PolygonEnvironment(None, holes)
    path, _ = environment.find_shortest_path((-1.0, 0.5), (3.0, 0.5))

    ax = draw(environment, path)

    drawn = artists(ax)
    assert "boundary" not in drawn
    assert {"obstacles", "path", "start", "goal"} <= set(drawn)
    # the view contains the path
    x_min, x_max = ax.get_xlim()
    assert x_min <= -1.0 and 3.0 <= x_max


def test_style(environment):
    style = PlotStyle(path_color="black", obstacle_color="#123456", path_width=5.0)
    path = [(4.5, 1.0), (4.0, 8.5)]

    drawn = artists(draw(environment, path, style=style))

    assert drawn["path"].get_color() == "black"
    assert drawn["path"].get_linewidth() == 5.0
    assert tuple(drawn["obstacles"].get_facecolor()[0]) == to_rgba("#123456")


def test_group_ids_in_svg_export(environment, tmp_path):
    path, _ = environment.find_shortest_path((4.5, 1.0), (4.0, 8.5))
    ax = draw(environment, path)
    output = tmp_path / "plot.svg"

    ax.figure.savefig(output)

    svg = output.read_text(encoding="utf-8")
    for gid in ("boundary", "obstacles", "visibility-graph", "path", "start", "goal"):
        assert f'id="{gid}"' in svg


def test_draw_search(environment):
    result = environment.search(*INDIRECT_QUERY)
    assert result.query_edges

    drawn = artists(draw_search(environment, result))

    assert {"visibility-graph", "query-edges", "path", "start", "goal"} <= set(drawn)
    assert len(drawn["query-edges"].get_segments()) == len(result.query_edges)
    assert drawn["query-edges"].get_linestyle() != [(0, None)]  # dashed
    np.testing.assert_array_equal(drawn["path"].get_xydata(), np.array(result.path))


def test_draw_search_without_drawing_the_path(environment):
    result = environment.search(*INDIRECT_QUERY)

    drawn = artists(draw_search(environment, result, path=False))

    assert "path" not in drawn
    assert "query-edges" in drawn
    np.testing.assert_array_equal(drawn["start"].get_offsets(), [INDIRECT_QUERY[0]])
    np.testing.assert_array_equal(drawn["goal"].get_offsets(), [INDIRECT_QUERY[1]])


def test_draw_search_without_graph_search(environment):
    result = environment.search(*DIRECT_QUERY)
    assert result.query_edges is None

    drawn = artists(draw_search(environment, result))

    assert "query-edges" not in drawn
    np.testing.assert_array_equal(drawn["path"].get_xydata(), np.array(DIRECT_QUERY))


def test_draw_search_without_path():
    environment = PolygonEnvironment(*SEPARATED_ENV)
    (start, goal), _ = TEST_DATA_SEPARATE_ENV[0]
    result = environment.search(start, goal)
    assert result.path == []

    drawn = artists(draw_search(environment, result))

    # start and goal are marked nevertheless
    assert "path" not in drawn
    np.testing.assert_array_equal(drawn["start"].get_offsets(), [start])
    np.testing.assert_array_equal(drawn["goal"].get_offsets(), [goal])


def test_plot_environment(environment, tmp_path):
    first = plot_environment(environment, tmp_path)
    second = plot_environment(environment, str(tmp_path))

    # unique default file names
    assert first != second
    assert sorted(tmp_path.iterdir()) == sorted([first, second])
    assert first.name.startswith("environment_") and first.suffix == ".svg"
    assert 'id="visibility-graph"' in first.read_text(encoding="utf-8")
    # the figures are closed
    assert plt.get_fignums() == []


@pytest.mark.parametrize("query", [INDIRECT_QUERY, DIRECT_QUERY])
def test_plot_search(environment, tmp_path, query):
    result = environment.search(*query)

    file_path = plot_search(
        environment, result, tmp_path / "plots", file_name="search.png"
    )

    # the extension selects the format
    assert file_path == (tmp_path / "plots" / "search.png").resolve()
    assert file_path.read_bytes().startswith(b"\x89PNG")
    assert plt.get_fignums() == []


def test_docs_figures(tmp_path):
    generate_figures(tmp_path)

    names = {file.name for file in tmp_path.iterdir()}
    assert names == {
        "demo.svg",
        "environment.svg",
        "environment_graph.svg",
        "search_graph.svg",
        "search.svg",
        "grid.svg",
        "touching_vertices.svg",
        "touching_vertex_edge.svg",
        "touching_edges.svg",
    }
    demo = (tmp_path / "demo.svg").read_text(encoding="utf-8")
    assert "prefers-color-scheme: dark" in demo
    assert 'id="path"' in demo
    # the search graph without the path
    search_graph = (tmp_path / "search_graph.svg").read_text(encoding="utf-8")
    assert 'id="query-edges"' in search_graph and 'id="path"' not in search_graph
    # deterministic: no creation date, regenerating gives identical files
    first = {name: (tmp_path / name).read_bytes() for name in names}
    generate_figures(tmp_path)
    assert {name: (tmp_path / name).read_bytes() for name in names} == first
