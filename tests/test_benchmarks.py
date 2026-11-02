"""Tests of the benchmark harness (``benchmarks/``), skipped without the benchmark dependencies."""

import sys

import pytest

pytest.importorskip("shapely")
pytest.importorskip("pyvisgraph")

from shapely.geometry import Point

from benchmarks import measure, run, validate, workloads, world_map
from benchmarks.adapters import get_adapter
from benchmarks.pyvisgraph_issues import CRASH_POLYGONS, POLYGON, QUERY

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


def test_pyvisgraph_build_crash_reproduction():
    """the documented pyvisgraph defect: math domain error in angle2 on vertices 1e-6 apart"""
    adapter = get_adapter("pyvisgraph")
    with pytest.raises(ValueError, match="math domain error"):
        adapter.build(CRASH_POLYGONS)
    env = get_adapter("extremitypathfinder").build(CRASH_POLYGONS)
    assert env.nr_vertices == 6


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
    """external failures (here: the map download) must not fail the quick run"""

    def fail(*_args, **_kwargs):
        raise world_map.DownloadError("no network")

    monkeypatch.setattr(workloads, "load_world", fail)
    raw = tmp_path / "raw.json"
    argv = ["--quick", "--no-render", "--fail-on-invalid", "--raw-output", str(raw)]
    assert run.main([*argv, "--packages", "extremitypathfinder"]) == 0
    assert "no network" in raw.read_text()
    with pytest.raises(world_map.DownloadError):
        run.main([*argv[1:], "--fresh"])  # the full run fails


def _fake_worker(monkeypatch, tmp_path, body: str):
    """replace the worker by a script: emits the given events, then runs the body"""
    script = tmp_path / "worker.py"
    script.write_text(
        "import os, signal, sys, time\n"
        "def emit(*events):\n"
        "    sys.stdout.buffer.write(b''.join(b'@epf-benchmark-event ' + e + b'\\n' for e in events))\n"
        "    sys.stdout.buffer.flush()\n" + body
    )
    monkeypatch.setattr(measure, "WORKER_COMMAND", [sys.executable, str(script)])


START = 'b\'{"event": "build_start"}\''
BUILT = 'b\'{"event": "built", "build_time": 0.0}\''


def test_hanging_query_phase_is_killed(monkeypatch, tmp_path):
    # both events in one chunk, unrelated output of a package ignored
    _fake_worker(
        monkeypatch,
        tmp_path,
        f"print('package output')\nemit({START}, {BUILT})\ntime.sleep(60)\n",
    )
    monkeypatch.setattr(measure, "QUERY_LIMIT_GRACE", 0.0)
    result = measure.measure("pyvisgraph", SQUARE, [], budget=10, query_limit=0.5)
    assert result["status"] == measure.CRASH
    assert "query phase killed" in result["error"]


def test_killed_build_is_a_crash_not_over_budget(monkeypatch, tmp_path):
    """e.g. the out of memory killer"""
    _fake_worker(
        monkeypatch, tmp_path, f"emit({START})\nos.kill(os.getpid(), signal.SIGKILL)\n"
    )
    result = measure.measure("pyvisgraph", SQUARE, [], budget=60)
    assert result["status"] == measure.CRASH
    assert "killed" in result["error"]


def test_slow_build_is_over_budget(monkeypatch, tmp_path):
    _fake_worker(monkeypatch, tmp_path, f"emit({START})\ntime.sleep(60)\n")
    result = measure.measure("pyvisgraph", SQUARE, [], budget=0.5)
    assert result["status"] == measure.OVER_BUDGET


def test_percentiles_nearest_rank():
    stats = run.query_stats([int(t * 1e9) for t in (1, 2, 3, 4, 5)])
    assert (stats["p50"], stats["p95"], stats["p5"]) == (3, 5, 1)


def test_render_tables_edge_cases():
    """no world results, only invalid results: every table keeps a body (strict docs build)"""
    meta = {
        "quick": False,
        "budget": 600,
        "query_limit": None,
        "nr_queries": 1,
        **_system_meta(),
    }
    entry = {
        "map": {
            "name": "m",
            "resolution": "110m",
            "polygons": 1,
            "vertices": 4,
            "tolerance": 0.0,
            "dropped_intersecting": 0,
        },
        # pyvisgraph not run (--packages)
        "results": {
            "extremitypathfinder": {
                "status": measure.INVALID,
                "validation": {
                    "failed": 1,
                    "checked": 1,
                    "infeasible": 1,
                    "not_optimal": 0,
                },
            }
        },
    }
    for world in ([], [entry]):
        tables = run.render_tables({"meta": meta, "world": world, "examples": []})
        for name, content in tables.items():
            if ".. list-table::" in content:
                assert content.count("   * - ") >= 2, name


def _system_meta():
    return {
        "system": {
            "cpu_model": "cpu",
            "cpu_count": 1,
            "memory_bytes": 2**30,
            "platform": "p",
            "python": "3",
            "python_implementation": "CPython",
        },
        "versions": {},
        "git": {"sha": None, "dirty": False},
        "date": "d",
        "command": "c",
        "seed": 0,
    }


def _fake_measure(calls: list, interrupt_at: int | None = None):
    """a measurement without subprocess: straight paths (infeasible around obstacles)"""

    def fake(package, holes, queries, budget, query_limit=None):
        calls.append(package)
        if len(calls) == interrupt_at:
            raise KeyboardInterrupt
        return {
            "status": "done",
            "build_time": 0.1,
            "query_times_ns": [1000] * len(queries),
            "paths": [[list(start), list(goal)] for start, goal in queries],
            "memory_peak": 0,
        }

    return fake


def test_resume_after_interruption(monkeypatch, tmp_path):
    raw = tmp_path / "raw.json"
    argv = [
        "--sizes",
        "none",
        "--queries",
        "3",
        "--raw-output",
        str(raw),
        "--no-render",
    ]
    calls = []
    monkeypatch.setattr(measure, "measure", _fake_measure(calls, interrupt_at=4))
    with pytest.raises(KeyboardInterrupt):
        run.main(argv)
    stored = run.json.loads(raw.read_text())
    assert [e["complete"] for e in stored["examples"]] == [True, False]
    with pytest.raises(SystemExit, match="--resume"):
        run.main(argv)  # never overwritten silently
    with pytest.raises(SystemExit, match="nr_queries"):
        run.main([*argv, "--resume", "--queries", "4"])  # other settings
    monkeypatch.setattr(run, "measured_code_changed", lambda sha: True)
    with pytest.raises(SystemExit, match="code of the package"):
        run.main([*argv, "--resume"])  # other code
    calls.clear()
    monkeypatch.setattr(measure, "measure", _fake_measure(calls))
    # independent of uncommitted changes in this checkout (tested separately)
    monkeypatch.setattr(run, "measured_code_changed", lambda sha: False)
    assert run.main([*argv, "--resume"]) == 0
    # only the interrupted and the remaining measurements: 3 examples x 2 packages - 3 stored
    assert len(calls) == 3
    results = run.json.loads(raw.read_text())
    assert all(e["complete"] for e in results["examples"])
    assert len(results["meta"]["resumed"]) == 1
    # the stored paths are validated again: straight paths through the obstacles
    assert results["examples"][0]["results"]["pyvisgraph"]["status"] in (
        measure.OK,
        measure.INVALID,
    )


def test_measurement_during_sleep_is_repeated(monkeypatch):
    calls = []
    monkeypatch.setattr(measure, "measure", _fake_measure(calls))
    clock = iter(
        [0.0, 3600.0, 4000.0, 4000.0]
    )  # wall clock: first measurement spans a sleep
    monkeypatch.setattr(run.time, "time", lambda: next(clock))
    result = run.measure_awake("pyvisgraph", SQUARE, [], None, None, "test")
    assert len(calls) == 2
    assert "suspended_s" not in result


def test_default_outputs():
    """the full run's raw results are committed, quick runs and the paths are not"""
    full, quick = run.parse_args([]), run.parse_args(["--quick"])
    assert full.raw_output == run.RESULTS_DIR / "benchmark.json"
    assert quick.raw_output.parent == run.RAW_DIR
    paths_dir = run.Checkpoint({}, full.raw_output).paths_dir
    assert paths_dir == run.RAW_DIR / "benchmark_paths"


def test_resume_refused_after_code_change():
    """a resumed run must not mix measurements of different code"""
    head = run._git()["sha"]
    assert not run.measured_code_changed(head) or run._git()["dirty"]
    assert run.measured_code_changed(None)
    assert run.measured_code_changed("0" * 40)  # unknown commit


def test_battery_only_warns(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(run, "on_battery", lambda: True)
    monkeypatch.setattr(run.sys, "platform", "darwin")
    monkeypatch.setattr(run.shutil, "which", lambda name: None)  # no caffeinate here
    monkeypatch.setattr(measure, "measure", _fake_measure([]))
    argv = [
        "--sizes",
        "none",
        "--no-render",
        "--raw-output",
        str(tmp_path / "raw.json"),
    ]
    assert run.main(argv) == 0
    assert "on battery power" in capsys.readouterr().err
