"""Drawing environments and paths onto matplotlib axes (``plotting.draw()``)."""

import matplotlib.pyplot as plt
import numpy as np
import pytest
from matplotlib.colors import to_rgba

from extremitypathfinder import PolygonEnvironment
from extremitypathfinder.plotting import PlotStyle, draw
from scripts.demo_plot import main as generate_demo
from tests.test_cases import POLY_ENV_PARAMS


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


def test_demo_plot(tmp_path):
    output = tmp_path / "demo.svg"

    generate_demo(output)

    svg = output.read_text(encoding="utf-8")
    assert "prefers-color-scheme: dark" in svg
    assert 'id="path"' in svg
    # deterministic: no creation date, regenerating gives an identical file
    first = output.read_bytes()
    generate_demo(output)
    assert output.read_bytes() == first
