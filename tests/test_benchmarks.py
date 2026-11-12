"""Tests of the benchmark harness (``benchmarks/``), skipped without the benchmark dependencies."""

import json
import math
import sys
from dataclasses import asdict
from pathlib import Path

import pytest

pytest.importorskip("shapely")
pytest.importorskip("pyvisgraph")

from shapely.geometry import Point, Polygon

from benchmarks import measure, run, validate, workloads, world_map
from benchmarks.adapters import NO_NUMBA, get_adapter
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
    """the documented pyvisgraph defect: acos fails in angle2 on vertices 1e-6 apart"""
    adapter = get_adapter("pyvisgraph")
    # the message of math.acos: "math domain error" up to Python 3.13,
    # "expected a number in range from -1 up to 1, got -1.0000000014" from Python 3.14 on
    with pytest.raises(ValueError, match="math domain error|range from -1 up to 1"):
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
    assert workloads.parse_size("5k").target_vertices == 5000
    assert workloads.parse_size("2000").name == "world_2k"
    assert workloads.parse_size("world_250").target_vertices == 250


def test_downsample_reaches_at_least_the_target():
    """a map of n vertices has at least n vertices (e.g. world_6k: not 5,952)"""
    polygons = [
        Polygon(
            [
                (10 * i + 4 * math.cos(a), 4 * math.sin(a))
                for a in (2 * math.pi * k / 400 for k in range(400))
            ]
        )
        for i in range(5)
    ]
    for target in (100, 250, 1000):
        _, stats = world_map.downsample(polygons, target)
        assert target <= stats["vertices"] <= (1 + world_map.TARGET_RTOL) * target
    with pytest.raises(ValueError):
        world_map.downsample(polygons, 2001)
    # unreachable within the tolerance: a larger simplification drops all polygons
    with pytest.warns(UserWarning, match="above the target"):
        _, stats = world_map.downsample(polygons, 5)
    assert stats["vertices"] >= 5


def test_load_world_rebuilds_maps_of_another_map_version(monkeypatch, tmp_path):
    """cached maps of an earlier map generation (e.g. at most the target) are not reused"""
    spec = workloads.SWEEP_BY_NAME["world_5k"]
    old = {"spec": asdict(spec), "ne_release": world_map.NE_RELEASE, "vertices": 4954}
    (tmp_path / f"{spec.name}.json").write_text(json.dumps(old))
    monkeypatch.setattr(world_map, "build_map", lambda *_a, **_k: {"vertices": 5081})
    assert workloads.load_world(spec, tmp_path)["vertices"] == 5081
    monkeypatch.setattr(world_map, "build_map", lambda *_a, **_k: pytest.fail("cached"))
    assert workloads.load_world(spec, tmp_path)["vertices"] == 5081


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


def test_invalid_runs_show_build_results_but_no_query_results():
    """invalid paths: no query results, the graph size is marked, the build is measured like any other"""
    meta = {
        "quick": False,
        "budget": 600,
        "query_limit": None,
        "nr_queries": 1,
        **_system_meta(),
    }
    stats = dict.fromkeys(("p5", "p50", "p95", "p99", "mean"), 1e-3)
    built = {
        "build_time": 2.0,
        "graph_nodes": 4,
        "graph_edges": 6,
        "memory_peak": 3 * 2**20,
        "memory_baseline": 10 * 2**20,
        "query_stats": {**stats, "queries_per_s": 1000.0, "n": 1},
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
        "results": {
            "extremitypathfinder": {"status": measure.OK, **built},
            "pyvisgraph": {
                "status": measure.INVALID,
                **built,
                "validation": {
                    "failed": 1,
                    "checked": 1,
                    "infeasible": 1,
                    "not_optimal": 0,
                },
            },
        },
    }
    tables = run.render_tables({"meta": meta, "world": [entry], "examples": []})
    build = tables["benchmark_build.rst"]
    # build time, memory and build ratio as for valid runs, only the graph size is marked
    assert build.count("(invalid)") == 1
    assert "4 / 6 (invalid)" in build
    assert build.count("     - 2 s\n") == 2
    assert "     - 1\n" in build  # build ratio
    # memory above the baseline, baseline and total for both packages, also for the invalid run
    memory = tables["benchmark_memory.rst"]
    assert "(invalid)" not in memory
    for cell in ("3 MiB", "10 MiB", "13 MiB"):
        assert memory.count(f"     - {cell}\n") == 2
    assert "pyvisgraph" not in tables["benchmark_query.rst"].split("Queries\n", 1)[1]


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


def test_baselines_carry_the_commit_of_the_run(monkeypatch, tmp_path):
    """a run rewrites its tracked raw results: the working tree is modified when the baselines are measured"""
    states = iter([{"sha": "a", "dirty": False}, {"sha": "a", "dirty": True}])
    monkeypatch.setattr(run, "_git", lambda: next(states))
    monkeypatch.setattr(measure, "measure", _fake_measure([]))
    monkeypatch.setattr(
        measure,
        "measure_baseline",
        lambda package, without_numba=False: {"status": "done", "memory_baseline": 1},
    )
    raw = tmp_path / "raw.json"
    argv = [
        "--sizes",
        "none",
        "--queries",
        "1",
        "--raw-output",
        str(raw),
        "--no-render",
    ]
    assert run.main(argv) == 0
    results = run.json.loads(raw.read_text())
    assert (
        results["baselines"]["git"]
        == results["meta"]["git"]
        == {"sha": "a", "dirty": False}
    )


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
    assert [e["complete"] for e in stored["examples"]] == [True]
    assert len(stored["examples"][0]["results"]) == 3
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
    # only the interrupted and the remaining measurements: 3 examples x 3 setups - 3 stored
    assert len(calls) == 6
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


def test_raw_results_compact_and_lossless(tmp_path):
    """the committed raw results: valid JSON, identical data, a list of numbers or a path per line"""
    results = {
        "meta": {"sizes": ["a", "b"], "nested": {}},
        "world": [{"times": [1, 2, 3], "path": [[0.1, 0.2], [0.3, 0.4]], "x": None}],
        "world_queries": [[[0.0, 1.0], [2.0, 3.0]], [[4.0, 5.0], [6.0, 7.0]]],
        "paths": {"0": [[1.0, 2.0], [3.0, 4.0]]},
        "empty": [],
    }
    path = tmp_path / "raw.json"
    run.write_raw(results, path)
    text = path.read_text()
    assert run.json.loads(text) == results
    assert text.endswith("}\n")
    assert '"times": [1, 2, 3],' in text
    assert '"path": [[0.1, 0.2], [0.3, 0.4]],' in text
    assert "  [[0.0, 1.0], [2.0, 3.0]]," in text  # one query per line
    assert not any(line != line.rstrip() for line in text.splitlines())


def test_baseline_without_numba():
    """the worker blocks Numba: extremitypathfinder runs as if Numba was not installed"""
    result = measure.measure_baseline("extremitypathfinder", without_numba=True)
    assert result["status"] == "done", result.get("error")
    assert result["numba"] is False
    assert result["memory_baseline"] > 0


def test_setup_without_numba_and_retained_memory():
    """the same paths as with Numba, Numba blocked in the worker, the retained memory within the peak"""
    data = workloads.small_example_maps()[-1]
    queries = workloads.generate_queries([data], 5, region=None)
    with_numba = measure.measure("extremitypathfinder", data["holes"], queries, 60)
    without = measure.measure(NO_NUMBA, data["holes"], queries, 60)
    assert without["status"] == "done", without.get("error")
    assert without["numba"] is False
    report = validate.validate_paths(
        data["holes"],
        queries,
        {"with": with_numba["paths"], "without": without["paths"]},
    )
    assert report["with"]["failed"] == report["without"]["failed"] == 0
    assert (without["graph_nodes"], without["graph_edges"]) == (
        with_numba["graph_nodes"],
        with_numba["graph_edges"],
    )
    if measure.allocated_memory_bytes() is not None:
        assert without["memory_retained"] > 0


@pytest.mark.skipif(
    measure.allocated_memory_bytes() is None,
    reason="allocator statistics not available",
)
@pytest.mark.parametrize(
    "allocate",
    [lambda: bytearray(50 * 2**20), lambda: [(float(i),) for i in range(500_000)]],
    ids=["C allocator", "small Python objects"],
)
def test_allocated_memory_excludes_freed_memory(allocate):
    """the retained memory counts what is in use, not the freed memory an allocator keeps"""
    before = measure.allocated_memory_bytes()
    data = allocate()
    assert measure.allocated_memory_bytes() - before > 30 * 2**20
    del data
    assert measure.allocated_memory_bytes() - before < 2**20


def test_memory_cells_with_retained_memory():
    result = {
        "memory_peak": 3 * 2**20,
        "memory_retained": 2 * 2**20,
        "memory_baseline": 10 * 2**20,
    }
    assert run.memory_cells(result) == ["3 MiB", "2 MiB", "10 MiB", "13 MiB"]
    # less than before the build, not available (results without it)
    assert run.memory_cells({**result, "memory_retained": -5})[1] == "< 1 MiB"
    assert run.memory_cells({**result, "memory_retained": None})[1] == "–"


def test_failures_without_numba_are_failures_of_the_package(monkeypatch, tmp_path):
    raw = tmp_path / "raw.json"
    entry = {"map": {"name": "m"}, "results": {NO_NUMBA: {"status": measure.CRASH}}}
    raw.write_text(run.json.dumps({"meta": {}, "world": [entry], "examples": []}))
    argv = ["--render-only", str(raw), "--no-render", "--fail-on-invalid"]
    assert run.main(argv) == 1


def test_baselines_rendered_and_added_on_the_same_machine_only(monkeypatch, tmp_path):
    setups = [
        {
            "label": "a",
            "package": "extremitypathfinder",
            "status": "done",
            "memory_baseline": 5 * 2**20,
        },
        {"label": "b", "package": "pyvisgraph", "status": measure.CRASH, "error": "e"},
    ]
    assert run.baseline_note({}) == ""
    note = run.baseline_note({"baselines": {"setups": setups}})
    assert "a 5 MiB" in note and "b failed" in note

    raw = tmp_path / "raw.json"
    raw.write_text(run.json.dumps({"meta": {"system": {"cpu_model": "other"}}}))
    with pytest.raises(SystemExit, match="another machine"):
        run.add_baselines(raw, ["pyvisgraph"])
    monkeypatch.setattr(run, "measure_baselines", lambda packages: {"setups": setups})
    monkeypatch.setattr(run.measure, "system_info", lambda: {"cpu_model": "other"})
    assert run.add_baselines(raw, ["pyvisgraph"])["baselines"]["setups"] == setups
    assert run.json.loads(raw.read_text())["baselines"]["setups"] == setups


def test_memory_composition_needs_matching_baselines(tmp_path):
    from benchmarks import plot

    def setup(numba, mib):
        return {
            "label": "x",
            "package": "extremitypathfinder",
            "status": "done",
            "numba": numba,
            "memory_baseline": mib * 2**20,
        }

    results = {
        "world": [
            {
                "results": {
                    "extremitypathfinder": {
                        "status": measure.OK,
                        "memory_baseline": 80 * 2**20,
                    }
                }
            }
        ]
    }
    assert plot.numba_share(results) is None
    results["baselines"] = {"setups": [setup(True, 86), setup(False, 26)]}
    assert plot.numba_share(results) == 60 * 2**20
    # a share larger than a baseline of the run (80 MiB): measured elsewhere, no bars
    results["baselines"] = {"setups": [setup(True, 160), setup(False, 46)]}
    assert plot.numba_share(results) is None
    # only the baseline with Numba
    results["baselines"] = {"setups": [setup(True, 86)]}
    assert plot.numba_share(results) is None


def test_derived_text_is_marked(tmp_path):
    # the text derived from the results must stay marked, so that a full run lists it for updating
    marked = {str(path) for path in run.derived_text_files()}
    assert {
        "README.rst",
        "docs/0_getting_started.rst",
        "docs/7_benchmark_method.rst",
        "docs/7_performance.rst",
        "docs/9_alternatives.rst",
    } <= marked
    (tmp_path / "docs").mkdir()
    (tmp_path / "docs" / "page.rst").write_text(f".. {run.DERIVED_MARKER}: claim\n")
    # the word in prose is no marker
    (tmp_path / "docs" / "other.rst").write_text(
        f"mentions {run.DERIVED_MARKER}: in prose\n"
    )
    assert run.derived_text_files(tmp_path) == [Path("docs/page.rst")]


def test_rendering_a_full_run_lists_the_derived_text(tmp_path, capsys):
    results = run.REPO_ROOT / "benchmarks" / "results" / "benchmark.json"
    out = ["--tables-dir", str(tmp_path), "--figures-dir", str(tmp_path)]
    assert run.main(["--render-only", str(results), *out]) == 0
    assert "docs/7_performance.rst" in capsys.readouterr().err


def test_correctness_table_omits_maps_which_no_package_built():
    """over budget for every package: no information on correctness; over budget for one: shown"""
    meta = {
        "quick": False,
        "budget": 600,
        "query_limit": None,
        "nr_queries": 1,
        **_system_meta(),
    }

    def entry(name, statuses):
        return {
            "map": {
                "name": name,
                "resolution": "110m",
                "polygons": 1,
                "vertices": 4,
                "tolerance": 0.0,
                "dropped_intersecting": 0,
            },
            "results": {
                package: {"status": status, "budget": 600, **built}
                if status == measure.OK
                else {"status": status, "budget": 600}
                for package, status in statuses.items()
            },
        }

    stats = dict.fromkeys(("p5", "p50", "p95", "p99", "mean"), 1e-3)
    built = {
        "build_time": 2.0,
        "graph_nodes": 4,
        "graph_edges": 6,
        "memory_peak": 3 * 2**20,
        "memory_baseline": 10 * 2**20,
        "query_stats": {**stats, "queries_per_s": 1000.0, "n": 1},
    }
    world = [
        entry(
            "both_over",
            {
                "extremitypathfinder": measure.OVER_BUDGET,
                "pyvisgraph": measure.OVER_BUDGET,
            },
        ),
        entry(
            "one_over",
            {
                "extremitypathfinder": measure.OK,
                "pyvisgraph": measure.OVER_BUDGET,
            },
        ),
    ]
    table = run.render_tables({"meta": meta, "world": world, "examples": []})[
        "benchmark_correctness.rst"
    ]
    assert "both_over" not in table
    assert "one_over" in table
