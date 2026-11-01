"""Tests of the benchmark harness (``benchmarks/``), skipped without the benchmark dependencies."""

import pytest

pytest.importorskip("shapely")
pytest.importorskip("pyvisgraph")

from shapely.geometry import Point

from benchmarks import measure, run, validate, workloads, world_map
from benchmarks.adapters import get_adapter
from benchmarks.pyvisgraph_issues import POLYGON, QUERY

SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


def test_feasibility():
    obstacles = validate.Obstacles([SQUARE])
    start, goal = (-1.0, 0.5), (2.0, 0.5)
    assert validate.feasibility(obstacles, start, goal, [start, goal]) == [
        "1 segment(s) through obstacles"
    ]
    around = [start, (0.0, 1.0), (1.0, 1.0), goal]
    # along an edge and through vertices is allowed
    assert validate.feasibility(obstacles, start, goal, around) == []
    assert validate.feasibility(obstacles, start, goal, around[:-1]) == [
        "does not end at the goal"
    ]
    assert validate.feasibility(obstacles, start, goal, []) == ["no path"]


def test_validate_paths_marks_infeasible_and_suboptimal():
    start, goal = (-1.0, 0.5), (2.0, 0.5)
    shortest = [start, (0.0, 1.0), (1.0, 1.0), goal]
    detour = [start, (-1.0, 2.0), (2.0, 2.0), goal]
    through = [start, goal]
    report = validate.validate_paths(
        [SQUARE],
        [(start, goal)],
        {"a": [shortest], "b": [detour], "c": [through], "d": [None]},
    )
    assert report["a"]["failed"] == 0
    assert report["b"]["not_optimal"] == 1
    assert report["c"]["infeasible"] == 1
    assert report["d"]["checked"] == 0


@pytest.mark.parametrize("package", ["extremitypathfinder", "pyvisgraph"])
def test_adapters_agree_on_square(package):
    adapter = get_adapter(package)
    env = adapter.build([SQUARE])
    path = adapter.query(env, (-1.0, 0.5), (2.0, 0.5))
    report = validate.validate_paths(
        [SQUARE], [((-1.0, 0.5), (2.0, 0.5))], {package: [path]}
    )
    assert report[package]["failed"] == 0
    assert validate.lengths_agree(
        validate.path_length(path), 2 * (0.5**2 + 1) ** 0.5 + 1.0
    )


def test_pyvisgraph_invalid_path_reproduction():
    """the documented pyvisgraph defect: a visibility edge through the obstacle"""
    paths = {}
    for package in ("extremitypathfinder", "pyvisgraph"):
        adapter = get_adapter(package)
        paths[package] = [adapter.query(adapter.build([POLYGON]), *QUERY)]
    report = validate.validate_paths([POLYGON], [QUERY], paths)
    assert report["extremitypathfinder"]["failed"] == 0
    assert report["pyvisgraph"]["infeasible"] == 1


def test_queries_in_free_space():
    data = workloads.small_example_maps()[-1]
    queries = workloads.generate_queries([data], 20, seed=1, region=None)
    assert queries == workloads.generate_queries([data], 20, seed=1, region=None)
    obstacles = validate.Obstacles(data["holes"])
    for start, goal in queries:
        for point in (start, goal):
            assert all(p.distance(Point(point)) >= 0.1 for p in obstacles.polygons)


def test_measure_subprocess_and_budget():
    data = workloads.small_example_maps()[-1]
    queries = workloads.generate_queries([data], 5, region=None)
    result = measure.measure("extremitypathfinder", data["holes"], queries, budget=60)
    assert result["status"] == "done"
    assert result["queries_completed"] == 5
    assert run.query_stats(result["query_times_ns"])["n"] == 5
    over = measure.measure("pyvisgraph", data["holes"], queries, budget=0.0)
    assert over["status"] == measure.OVER_BUDGET


def test_parse_size():
    assert workloads.parse_size("5k").resolution == "110m"
    assert workloads.parse_size("2000").name == "world_2k"
    assert workloads.parse_size("world_250").target_vertices == 250


def test_quick_run_tolerates_download_failure(monkeypatch, tmp_path):
    """external failures (here: the map download) must not fail the smoke test"""

    def fail(*_args, **_kwargs):
        raise world_map.DownloadError("no network")

    monkeypatch.setattr(workloads, "load_world", fail)
    raw = tmp_path / "raw.json"
    argv = ["--quick", "--no-render", "--fail-on-invalid", "--raw-output", str(raw)]
    assert run.main([*argv, "--packages", "extremitypathfinder"]) == 0
    assert "no network" in raw.read_text()
    with pytest.raises(world_map.DownloadError):
        run.main(argv[1:])  # the full run fails


def test_hanging_query_phase_is_killed(monkeypatch):
    monkeypatch.setattr(measure, "QUERY_LIMIT_GRACE", 0.0)
    data = workloads.small_example_maps()[-1]
    queries = workloads.generate_queries([data], 1000, region=None)
    result = measure.measure(
        "pyvisgraph", data["holes"], queries, budget=60, query_limit=0.0
    )
    assert result["status"] == measure.CRASH
    assert "query phase killed" in result["error"]
