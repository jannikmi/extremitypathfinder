"""Exporting and loading environments with pickle."""

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

    with pytest.raises(ValueError, match="Recreate the environment"):
        pickle.loads(data)


def test_pickle_with_other_format_version_is_rejected(environment, monkeypatch):
    data = pickle.dumps(environment)
    monkeypatch.setattr(
        configs, "PICKLE_FORMAT_VERSION", configs.PICKLE_FORMAT_VERSION + 1
    )

    with pytest.raises(ValueError, match="incompatible version"):
        pickle.loads(data)
