"""Exporting and loading environments with pickle."""

import copy
import pickle

import pytest

from extremitypathfinder import PolygonEnvironment, configs, load_pickle

BOUNDARY = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
HOLES = [[(3.0, 4.0), (3.0, 6.0), (7.0, 6.0), (7.0, 4.0)]]
START, GOAL = (5.0, 1.0), (5.0, 9.0)


@pytest.fixture
def environment():
    return PolygonEnvironment(BOUNDARY, HOLES)


def test_export_and_load_pickle(environment, tmp_path):
    path = str(tmp_path / "env.pickle")
    environment.export_pickle(path=path)
    loaded = load_pickle(path=path)

    assert isinstance(loaded, PolygonEnvironment)
    assert loaded.find_shortest_path(START, GOAL) == environment.find_shortest_path(
        START, GOAL
    )
    assert not hasattr(loaded, "_pickle_format_version")


def test_pickle_without_format_version_is_rejected(environment, monkeypatch):
    # environments pickled with versions before the format version was introduced
    monkeypatch.setattr(
        PolygonEnvironment, "__getstate__", lambda self: self.__dict__.copy()
    )
    data = pickle.dumps(environment)
    monkeypatch.undo()

    with pytest.raises(ValueError, match="no pickle format version"):
        pickle.loads(data)


def test_pickle_with_other_format_version_is_rejected(environment, monkeypatch):
    data = pickle.dumps(environment)
    monkeypatch.setattr(
        configs, "PICKLE_FORMAT_VERSION", configs.PICKLE_FORMAT_VERSION + 1
    )

    with pytest.raises(ValueError, match="incompatible version"):
        pickle.loads(data)


@pytest.mark.parametrize("copy_fct", [copy.copy, copy.deepcopy])
def test_copy(environment, copy_fct):
    copied = copy_fct(environment)
    assert copied.find_shortest_path(START, GOAL) == environment.find_shortest_path(
        START, GOAL
    )
    assert not hasattr(copied, "_pickle_format_version")


class SlottedEnvironment(PolygonEnvironment):
    __slots__ = ("tag",)


@pytest.mark.parametrize(
    "copy_fct",
    [copy.copy, copy.deepcopy, lambda env: pickle.loads(pickle.dumps(env))],
)
def test_subclass_with_slots(copy_fct):
    environment = SlottedEnvironment(BOUNDARY, HOLES)
    environment.tag = "x"
    copied = copy_fct(environment)
    assert copied.tag == "x"
    assert copied.find_shortest_path(START, GOAL) == environment.find_shortest_path(
        START, GOAL
    )
