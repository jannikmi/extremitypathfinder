"""Benchmark of extremitypathfinder against pyvisgraph: command line entry point.

Runs the world map size sweep (and the small examples), validates all paths,
writes the raw results (JSON, gitignored) and renders the RST tables and SVG figures
of the documentation.

Usage::

    make benchmark                                                    # --quick: smoke test, seconds
    uv run --all-extras --group benchmark python -m benchmarks.run    # full run, hours
    uv run --all-extras --group benchmark python -m benchmarks.run --sizes 250,1k,5k ...

See ``docs/7_performance.rst`` for the method and the results.
"""

import argparse
import datetime
import importlib.util
import json
import math
import os
import shlex
import shutil
import statistics
import subprocess
import sys
import time
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from benchmarks import measure, validate, workloads, world_map
from benchmarks.adapters import PACKAGES

REPO_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = Path(__file__).parent / "results"
RAW_DIR = RESULTS_DIR / "raw"
FIGURES_DIR = REPO_ROOT / "docs" / "_static"
# outputs of --quick runs: gitignored, the committed results stay untouched
QUICK_DIR = RAW_DIR / "quick"
DEFAULT_BUDGET = 600.0  # s per build
QUICK_NR_QUERIES = 20
# --quick: a hanging or slow external package must not block the run
QUICK_BUDGET = 120.0  # s
QUICK_QUERY_LIMIT = 60.0  # s
DEPENDENCIES = (
    "extremitypathfinder",
    "numpy",
    "networkx",
    "numba",
    "llvmlite",
    "scipy",
    "pyvisgraph",
    "shapely",
)
# statuses of a package and map
NOT_RUN = "not run"


def _versions() -> dict:
    versions = {}
    for name in DEPENDENCIES:
        try:
            versions[name] = version(name)
        except PackageNotFoundError:
            versions[name] = None
    return versions


def _git() -> dict:
    def git(*args):
        return subprocess.run(
            ["git", *args], check=False, cwd=REPO_ROOT, capture_output=True, text=True
        ).stdout.strip()

    return {
        "sha": git("rev-parse", "HEAD") or None,
        "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
    }


def metadata(args, argv) -> dict:
    return {
        "date": datetime.datetime.now(datetime.UTC).isoformat(timespec="seconds"),
        "command": shlex.join(["python", "-m", "benchmarks.run", *argv]),
        "quick": args.quick,
        "seed": args.seed,
        "nr_queries": args.queries,
        "query_eps": workloads.QUERY_EPS,
        "query_region": workloads.QUERY_REGION,
        "budget": args.budget,
        "query_limit": args.query_limit,
        "allow_no_numba": args.allow_no_numba,
        "sizes": [spec.name for spec in args.sizes],
        "length_rtol": validate.LENGTH_RTOL,
        "repeat_builds_below": measure.REPEAT_BELOW,
        "natural_earth": {
            "release": world_map.NE_RELEASE,
            "sha256": world_map.NE_SHA256,
        },
        "system": measure.system_info(),
        "versions": _versions(),
        "git": _git(),
    }


def query_stats(times_ns: list[int]) -> dict | None:
    if not times_ns:
        return None
    times = sorted(t * 1e-9 for t in times_ns)

    def percentile(q):
        # nearest rank
        return times[max(0, math.ceil(q / 100 * len(times)) - 1)]

    mean = statistics.fmean(times)
    return {
        "p5": percentile(5),
        "p50": percentile(50),
        "p95": percentile(95),
        "p99": percentile(99),
        "mean": mean,
        "queries_per_s": 1.0 / mean if mean > 0 else None,
        "n": len(times),
    }


def _log(*args, **kwargs):
    print(*args, file=sys.stderr, flush=True, **kwargs)


# results which are kept when a run is resumed (validated again with the stored paths)
MEASURED = {"done", measure.OK, measure.INVALID, measure.CRASH, measure.OVER_BUDGET}
# a measurement during which the machine slept is repeated
SUSPEND_TOLERANCE = 2.0  # s
MAX_ATTEMPTS = 3
# settings which must not change when a run is resumed
RESUME_KEYS = (
    "quick",
    "seed",
    "nr_queries",
    "budget",
    "query_limit",
    "allow_no_numba",
    "sizes",
)


class Checkpoint:
    """the raw results, written after every measurement, and the paths of the measurements

    The paths are needed for the validation, but too large for the raw results:
    they are stored next to them (``<raw>_paths/``).
    """

    def __init__(self, results: dict, path: Path):
        self.results = results
        self.path = path
        self.paths_dir = path.with_name(path.stem + "_paths")

    def _paths_file(self, section: str, name: str, package: str) -> Path:
        return self.paths_dir / f"{section}__{name}__{package}.json"

    def save(self):
        write_raw(self.results, self.path)

    def save_paths(self, section, name, package, paths):
        self.paths_dir.mkdir(parents=True, exist_ok=True)
        with open(self._paths_file(section, name, package), "w") as f:
            json.dump(paths, f)

    def load_paths(self, section, name, package):
        path = self._paths_file(section, name, package)
        if not path.exists():
            return None
        with open(path) as f:
            return json.load(f)

    def entry(self, section: str, data: dict) -> dict:
        """the (possibly partial) entry of a map, appended to the results if new"""
        for entry in self.results[section]:
            if entry["map"]["name"] == data["name"]:
                return entry
        entry = {"map": workloads.map_summary(data), "results": {}, "complete": False}
        self.results[section].append(entry)
        return entry


def measure_awake(package, holes, queries, budget, query_limit, label) -> dict:
    """measure, repeated if the machine was suspended (sleep) during the measurement

    The monotonic clocks of the timings stop during a suspension, the wall clock does not.
    """
    for attempt in range(1, MAX_ATTEMPTS + 1):
        wall, started = time.time(), time.perf_counter()
        result = measure.measure(package, holes, queries, budget, query_limit)
        suspended = (time.time() - wall) - (time.perf_counter() - started)
        if suspended < SUSPEND_TOLERANCE:
            return result
        result["suspended_s"] = suspended
        _log(
            f"[{label}] {package}: the machine was suspended for {suspended:.0f} s, "
            f"measurement {'repeated' if attempt < MAX_ATTEMPTS else 'kept'} "
            f"(attempt {attempt} of {MAX_ATTEMPTS})"
        )
    return result


def run_map(
    checkpoint: Checkpoint,
    section: str,
    data: dict,
    queries,
    packages,
    active: set,
    budget,
    query_limit,
) -> dict:
    """measure and validate all packages on one map, resuming stored measurements"""
    entry = checkpoint.entry(section, data)
    name = data["name"]
    paths = {}
    for package in packages:
        if package not in active:
            entry["results"][package] = {"status": NOT_RUN}
            continue
        previous = entry["results"].get(package, {})
        if previous.get("status") in MEASURED:
            stored = checkpoint.load_paths(section, name, package)
            failed = previous["status"] in (measure.CRASH, measure.OVER_BUDGET)
            if failed or stored is not None:
                _log(f"[{section}] {package} ({data['vertices']:,} vertices): resumed")
                if stored is not None:
                    paths[package] = stored
                continue
        _log(f"[{section}] {package} ({data['vertices']:,} vertices) ...")
        started = time.perf_counter()
        result = measure_awake(
            package, data["holes"], queries, budget, query_limit, section
        )
        _log(
            f"[{section}] {package}: {result['status']}"
            f" ({time.perf_counter() - started:.1f} s) {result.get('error', '')}"
        )
        if result["status"] == "done":
            paths[package] = result.pop("paths")
            checkpoint.save_paths(section, name, package, paths[package])
            result["query_stats"] = query_stats(result["query_times_ns"])
        entry["results"][package] = result
        entry["complete"] = False
        checkpoint.save()
    report = validate.validate_paths(data["holes"], queries, paths)
    # paths kept in the raw results (for the figures): the first query and all failed queries
    kept = {0} | {f["query"] for c in report.values() for f in c["failures"]}
    for package, package_paths in paths.items():
        entry["results"][package]["example_paths"] = {
            str(i): [list(p) for p in package_paths[i]]
            for i in sorted(kept)
            if i < len(package_paths)
        }
    for package, checks in report.items():
        result = entry["results"][package]
        result["validation"] = checks
        result["status"] = measure.INVALID if checks["failed"] else measure.OK
        if checks["failed"]:
            _log(f"[{section}] {package}: {checks['failed']} invalid queries")
    entry["complete"] = True
    checkpoint.save()
    return entry


def load_resumable(args, argv) -> dict:
    """the stored raw results to resume, checked against the settings of this run"""
    with open(args.raw_output) as f:
        results = json.load(f)
    stored, current = results["meta"], metadata(args, argv)
    changed = [key for key in RESUME_KEYS if stored.get(key) != current.get(key)]
    for key in ("cpu_model", "cpu_count", "platform", "python"):
        if stored["system"].get(key) != current["system"].get(key):
            changed.append(f"system.{key}")
    changed += [
        f"versions.{name}"
        for name, version in current["versions"].items()
        if stored["versions"].get(name) != version
    ]
    if changed:
        raise SystemExit(
            f"cannot resume {args.raw_output}: changed {', '.join(changed)}. "
            "Run with the settings of the interrupted run, or with --fresh to start over."
        )
    if stored["git"] != current["git"]:
        _log(
            f"WARNING: the code changed since the interrupted run: {stored['git']} -> {current['git']}"
        )
    stored.setdefault("resumed", []).append(
        {"date": current["date"], "git": current["git"], "command": current["command"]}
    )
    return results


def keep_awake():
    """macOS: prevent idle sleep while this process runs (sleep pauses the measurements)"""
    if sys.platform != "darwin" or shutil.which("caffeinate") is None:
        return
    # -i: idle sleep, -s: system sleep (on AC power), -w: until this process exits
    subprocess.Popen(["caffeinate", "-i", "-s", "-w", str(os.getpid())])
    _log("caffeinate: idle and system sleep prevented while the benchmark runs")
    if measure.power_source() == "Battery Power":
        _log("WARNING: on battery power: connect the power adapter (throttling, sleep)")


def run(args, argv) -> dict:
    packages = args.packages
    # the plan: extremitypathfinder is only compared with Numba (best case)
    numba_missing = importlib.util.find_spec("numba") is None
    if "extremitypathfinder" in packages and numba_missing and not args.allow_no_numba:
        raise SystemExit(
            "Numba is not installed: install the numba extra (uv sync --all-extras) "
            "or pass --allow-no-numba"
        )
    if args.resume:
        results = load_resumable(args, argv)
        _log(f"resuming {args.raw_output}")
    else:
        if args.raw_output.exists() and not (args.quick or args.fresh):
            raise SystemExit(
                f"{args.raw_output} exists (an interrupted or earlier run): "
                "pass --resume to continue it or --fresh to start over"
            )
        results = {"meta": metadata(args, argv), "world": [], "examples": []}
    checkpoint = Checkpoint(results, args.raw_output)
    if args.fresh and checkpoint.paths_dir.exists():
        shutil.rmtree(checkpoint.paths_dir)
    keep_awake()

    if args.examples:
        for data in workloads.small_example_maps():
            entry = checkpoint.entry("examples", data)
            queries = entry.setdefault(
                "queries",
                workloads.generate_queries(
                    [data], args.queries, args.seed, eps=0.1, region=None
                ),
            )
            run_map(
                checkpoint,
                "examples",
                data,
                queries,
                packages,
                set(packages),
                args.budget,
                None,
            )

    specs = args.sizes
    if specs:
        _log(f"loading {len(specs)} world maps ...")
        try:
            maps = [workloads.load_world(spec) for spec in specs]
            maps.sort(key=lambda data: data["vertices"])
        except world_map.DownloadError as exc:
            if not args.quick:
                raise
            # external failure: the smoke test continues with the small examples
            _log(f"WARNING: world maps skipped: {exc}")
            results["world_skipped"] = str(exc)
            checkpoint.save()
            return results
        if "world_queries" not in results:
            results["world_queries"] = workloads.generate_queries(
                maps, args.queries, args.seed
            )
        queries = results["world_queries"]
        active = set(packages)
        for data in maps:
            entry = run_map(
                checkpoint,
                "world",
                data,
                queries,
                packages,
                active,
                args.budget,
                args.query_limit,
            )
            for package, result in entry["results"].items():
                if result["status"] == measure.OVER_BUDGET:
                    active.discard(package)
            if {"extremitypathfinder"} & set(packages) - active:
                _log("extremitypathfinder exceeded the build budget: end of the sweep")
                break
    results["meta"]["completed"] = datetime.datetime.now(datetime.UTC).isoformat(
        timespec="seconds"
    )
    checkpoint.save()
    return results


def preflight(args) -> int:
    """check this machine for the full run, print a report. 0: ready"""
    problems = []

    def check(ok: bool, message: str, problem: str | None = None):
        print(f"{'ok  ' if ok else 'FAIL'} {message}")
        if not ok:
            problems.append(problem or message)

    system = measure.system_info()
    print(
        f"machine: {system['cpu_model']}, {system['cpu_count']} logical cores, "
        f"{format_bytes(system['memory_bytes'])}, {system['platform']}, Python {system['python']}"
    )
    tiny = [[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]]
    query = [((-1.0, 0.5), (2.0, 0.5))]
    for package in args.packages:
        result = measure.measure(package, tiny, query, budget=60)
        check(
            result["status"] == "done",
            f"{package} runs in a worker: {result['status']} {result.get('error', '')}",
        )
        if package == "extremitypathfinder" and result["status"] == "done":
            check(
                result["numba"] or args.allow_no_numba,
                f"Numba active: {result['numba']}",
            )
    # the peak memory of a worker must not include the peak of this (large) process
    ballast = bytearray(300 * 2**20)
    for k in range(0, len(ballast), 4096):
        ballast[k] = 1
    output = subprocess.run(
        [*measure.WORKER_COMMAND, "--memory-probe", str(100 * 2**20)],
        cwd=measure.REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    ).stdout
    del ballast
    try:
        probe = json.loads(output)
    except ValueError:
        probe = None
    if probe is None:
        check(False, "memory probe failed")
    else:
        peak, baseline = probe["peak"] / 2**20, probe["baseline"] / 2**20
        check(
            80 <= peak <= 150 and baseline < 250,
            f"peak memory ({probe['method']}): 100 MiB allocated, {peak:.0f} MiB measured, "
            f"worker baseline {baseline:.0f} MiB with 300 MiB in the parent",
            "the peak memory measurement is wrong on this machine",
        )
    if sys.platform == "darwin":
        check(
            shutil.which("caffeinate") is not None,
            "caffeinate available (prevents sleep)",
        )
        check(
            system.get("power_source") == "AC Power",
            f"power source: {system.get('power_source')}",
            "connect the power adapter",
        )
    if not args.quick:
        for spec in args.sizes:
            try:
                workloads.load_world(spec)
            except world_map.DownloadError as exc:
                check(False, f"map data: {exc}")
                break
        else:
            check(
                True,
                f"world maps generated and cached: {', '.join(s.name for s in args.sizes)}",
            )
    free = shutil.disk_usage(
        args.raw_output.parent if args.raw_output.parent.exists() else REPO_ROOT
    ).free
    check(free > 2**30, f"free disk space: {format_bytes(free)}")
    if args.raw_output.exists() and not args.quick:
        print(
            f"note: {args.raw_output} exists: continue it with --resume, or start over with --fresh"
        )
    print(
        "ready for the full run"
        if not problems
        else "not ready:\n  " + "\n  ".join(problems)
    )
    return 1 if problems else 0


def write_raw(results: dict, path: Path):
    """write atomically: an interruption while writing keeps the previous file"""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w") as f:
        json.dump(results, f, indent=1)
    os.replace(tmp, path)


# rendering of the RST tables

PLACEHOLDER_NOTE = (
    "PLACEHOLDER: generated by a ``--quick`` run (smoke test of the harness) "
    "on a shared, noisy machine. The numbers are not representative and must not be cited."
)


def format_time(seconds: float | None) -> str:
    if seconds is None:
        return "–"
    for unit, factor in (("s", 1.0), ("ms", 1e-3), ("µs", 1e-6)):
        if seconds >= factor:
            return f"{seconds / factor:.3g} {unit}"
    return f"{seconds * 1e9:.3g} ns"


def format_bytes(nr_bytes: int | None) -> str:
    if nr_bytes is None:
        return "–"
    for unit, factor in (("GiB", 2**30), ("MiB", 2**20), ("KiB", 2**10)):
        if nr_bytes >= factor:
            return f"{nr_bytes / factor:.3g} {unit}"
    return f"{nr_bytes} B"


def _list_table(title: str, header: list[str], rows: list[list], widths=None) -> str:
    lines = [f".. list-table:: {title}", "   :header-rows: 1", ""]
    if not rows:
        # docutils rejects tables without a body
        rows = [["–"] * (len(header) - 1) + ["no results"]]
    for row in [header, *rows]:
        lines.append(f"   * - {row[0]}")
        lines.extend(f"     - {cell}" for cell in row[1:])
    return "\n".join(lines) + "\n"


def _note(results: dict) -> str:
    if results["meta"]["quick"]:
        return f".. warning::\n\n   {PLACEHOLDER_NOTE}\n\n"
    return ""


def is_ok(result: dict) -> bool:
    return result.get("status") == measure.OK


def largest_ok(results: dict) -> dict:
    """per package the largest map within the budget and the first one over the budget"""
    summary = {}
    for package in PACKAGES:
        ok = [
            e["map"]["vertices"]
            for e in results["world"]
            if e["results"].get(package, {}).get("status")
            in (measure.OK, measure.INVALID)
        ]
        over = [
            e["map"]["vertices"]
            for e in results["world"]
            if e["results"].get(package, {}).get("status") == measure.OVER_BUDGET
        ]
        summary[package] = {
            "largest_within_budget": max(ok) if ok else None,
            "first_over_budget": min(over) if over else None,
        }
    return summary


def system_summary(meta: dict) -> str:
    system = meta["system"]
    memory = format_bytes(system.get("memory_bytes"))
    versions = ", ".join(f"{k} {v}" for k, v in meta["versions"].items() if v)
    git = meta["git"]
    sha = (git["sha"] or "unknown")[:10] + (" (dirty)" if git["dirty"] else "")
    limit = (
        f"query time limit {meta['query_limit']:g} s"
        if meta["query_limit"]
        else "no query time limit"
    )
    return (
        f"- Machine: {system['cpu_model']}, {system['cpu_count']} logical cores, "
        f"{memory} RAM, {system['platform']}\n"
        f"- Python {system['python']} ({system['python_implementation']}), {versions}\n"
        f"- Commit ``{sha}``, {meta['date']}, command: ``{meta['command']}``\n"
        f"- Build budget {meta['budget']:g} s per package and size, {limit}, "
        f"{meta['nr_queries']} queries (seed {meta['seed']})\n"
    )


def render_tables(results: dict) -> dict[str, str]:
    """the RST tables of the results: file name -> content"""
    note = _note(results)
    meta = results["meta"]
    tables = {"benchmark_system.rst": note + system_summary(meta)}
    world = results["world"]

    rows = []
    for e in world:
        m = e["map"]
        rows.append(
            [
                m["name"],
                f"{m['resolution']}",
                f"{m['polygons']:,}",
                f"{m['vertices']:,}",
                f"{m['tolerance']:.2g}",
                f"{m['dropped_intersecting']}",
            ]
        )
    tables["benchmark_workloads.rst"] = note + _list_table(
        "World map sizes",
        [
            "Map",
            "Source",
            "Polygons",
            "Vertices",
            "Simplification tolerance (°)",
            "Dropped (intersecting)",
        ],
        rows,
    )

    rows = []
    for e in world:
        epf = e["results"].get("extremitypathfinder", {})
        pvg = e["results"].get("pyvisgraph", {})
        row = [f"{e['map']['vertices']:,}"]
        for r in (epf, pvg):
            row.append(
                format_time(r["build_time"]) if is_ok(r) else r.get("status", "–")
            )
        ratio = (
            f"{pvg['build_time'] / epf['build_time']:.3g}"
            if is_ok(epf) and is_ok(pvg)
            else "–"
        )
        row.append(ratio)
        for r in (epf, pvg):
            row.append(
                f"{r['graph_nodes']:,} / {r['graph_edges']:,}" if is_ok(r) else "–"
            )
        for r in (epf, pvg):
            if not is_ok(r):
                row.append("–")
            elif r["memory_peak"] < 2**20:
                # below the resolution of the peak resident memory
                row.append("< 1 MiB")
            else:
                row.append(format_bytes(r["memory_peak"]))
        rows.append(row)
    tables["benchmark_build.rst"] = note + _list_table(
        "Preprocessing (build) time, graph size and peak memory",
        [
            "Vertices",
            "Build extremitypathfinder",
            "Build pyvisgraph",
            "Ratio",
            "Graph extremitypathfinder (nodes / edges)",
            "Graph pyvisgraph (nodes / edges)",
            "Memory extremitypathfinder",
            "Memory pyvisgraph",
        ],
        rows,
    )

    rows = []
    for e in world:
        for package in PACKAGES:
            r = e["results"].get(package)
            if r is None or not is_ok(r):
                continue
            q = r["query_stats"]
            rows.append(
                [
                    f"{e['map']['vertices']:,}",
                    package,
                    format_time(q["p50"]),
                    format_time(q["p95"]),
                    format_time(q["p99"]),
                    format_time(q["mean"]),
                    f"{q['queries_per_s']:.3g}",
                    f"{q['n']} / {meta['nr_queries']}",
                ]
            )
    tables["benchmark_query.rst"] = note + _list_table(
        "Query latency",
        ["Vertices", "Package", "p50", "p95", "p99", "Mean", "Queries/s", "Queries"],
        rows,
    )

    rows = []
    for e in world + results["examples"]:
        for package, r in e["results"].items():
            if r["status"] in (measure.OK, NOT_RUN):
                continue
            if r["status"] == measure.INVALID:
                v = r["validation"]
                detail = (
                    f"{v['failed']} of {v['checked']} queries failed "
                    f"({v['infeasible']} infeasible, {v['not_optimal']} not optimal)"
                )
            elif r["status"] == measure.OVER_BUDGET:
                detail = f"build exceeded {r['budget']:g} s"
            else:
                detail = f"``{r.get('error', '')}``"
            rows.append(
                [
                    e["map"]["name"],
                    f"{e['map']['vertices']:,}",
                    package,
                    r["status"],
                    detail,
                ]
            )
    if not rows:
        rows.append(["–", "–", "–", "–", "all runs valid and within the budget"])
    tables["benchmark_correctness.rst"] = note + _list_table(
        "Invalid, crashed and over budget runs (left out of the speed results)",
        ["Map", "Vertices", "Package", "Status", "Details"],
        rows,
    )

    lines = []
    for package, s in largest_ok(results).items():
        if s["largest_within_budget"] is None and s["first_over_budget"] is None:
            continue
        line = f"- {package}: largest map within the budget: "
        line += (
            f"{s['largest_within_budget']:,} vertices"
            if s["largest_within_budget"]
            else "none"
        )
        if s["first_over_budget"]:
            line += (
                f". Exceeded the {meta['budget']:g} s build budget at "
                f"{s['first_over_budget']:,} vertices, larger maps were not run"
            )
        lines.append(line + ".")
    tables["benchmark_limits.rst"] = note + "\n".join(lines) + "\n"
    return tables


def write_tables(results: dict, directory: Path):
    directory.mkdir(parents=True, exist_ok=True)
    for name, content in render_tables(results).items():
        (directory / name).write_text(content)
    _log(f"tables written to {directory}")


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--quick",
        action="store_true",
        help=f"small maps ({', '.join(s.name for s in workloads.QUICK_SWEEP)}) "
        f"and {QUICK_NR_QUERIES} queries: smoke test of the harness (make benchmark). "
        "Failures to download the map data are warnings",
    )
    parser.add_argument(
        "--sizes",
        type=lambda s: (
            [] if s == "none" else [workloads.parse_size(x) for x in s.split(",") if x]
        ),
        default=None,
        help="comma separated world map sizes, e.g. 250,1k,5k, run in ascending order "
        "(default: the full sweep, 'none': only the small examples). "
        "The query set depends on the maps of the run: compare only runs with the same sizes",
    )
    parser.add_argument(
        "--budget",
        type=float,
        default=None,
        help=f"build time budget per package and size in seconds "
        f"(default: {DEFAULT_BUDGET:g}, --quick: {QUICK_BUDGET:g})",
    )
    parser.add_argument(
        "--query-limit",
        type=float,
        default=None,
        help="time limit of the query phase per package and size in seconds "
        f"(default: none, --quick: {QUICK_QUERY_LIMIT:g})",
    )
    parser.add_argument(
        "--queries",
        type=int,
        default=None,
        help=f"number of queries (default: {workloads.DEFAULT_NR_QUERIES}, "
        f"--quick: {QUICK_NR_QUERIES})",
    )
    parser.add_argument("--seed", type=int, default=workloads.DEFAULT_SEED)
    parser.add_argument(
        "--packages",
        type=lambda s: s.split(","),
        default=list(PACKAGES),
        help="comma separated packages (default: all)",
    )
    parser.add_argument(
        "--no-examples",
        dest="examples",
        action="store_false",
        help="skip the small examples",
    )
    parser.add_argument(
        "--raw-output",
        type=Path,
        default=None,
        help="raw results JSON (default: benchmarks/results/raw/benchmark[_quick].json)",
    )
    parser.add_argument(
        "--tables-dir",
        type=Path,
        default=None,
        help="output directory of the RST tables (default: benchmarks/results/, "
        "--quick: benchmarks/results/raw/quick/, never the committed results)",
    )
    parser.add_argument(
        "--figures-dir",
        type=Path,
        default=None,
        help="output directory of the SVG figures (default: docs/_static/, "
        "--quick: benchmarks/results/raw/quick/)",
    )
    parser.add_argument(
        "--no-render", action="store_true", help="only write the raw results"
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="continue an interrupted run (same settings): keeps the stored measurements",
    )
    parser.add_argument(
        "--fresh",
        action="store_true",
        help="start over, replacing the raw results of an earlier or interrupted run",
    )
    parser.add_argument(
        "--preflight",
        action="store_true",
        help="check this machine for the full run (Numba, memory measurement, power) and exit",
    )
    parser.add_argument(
        "--render-only",
        type=Path,
        default=None,
        metavar="RAW_JSON",
        help="render the tables and figures of existing raw results",
    )
    parser.add_argument(
        "--allow-no-numba",
        action="store_true",
        help="run extremitypathfinder without Numba (not comparable with the documented results)",
    )
    parser.add_argument(
        "--fail-on-invalid",
        action="store_true",
        help="exit with code 1 if extremitypathfinder is invalid or crashes. "
        "Errors of pyvisgraph and other external packages never change the exit code",
    )
    args = parser.parse_args(argv)
    if args.resume and args.fresh:
        parser.error("--resume and --fresh exclude each other")
    if args.query_limit is not None and args.query_limit <= 0:
        parser.error("--query-limit must be positive")
    for package in args.packages:
        if package not in PACKAGES:
            parser.error(f"unknown package {package}, choose from {PACKAGES}")
    if args.tables_dir is None:
        args.tables_dir = QUICK_DIR if args.quick else RESULTS_DIR
    if args.figures_dir is None:
        args.figures_dir = QUICK_DIR if args.quick else FIGURES_DIR
    if args.quick:
        if args.budget is None:
            args.budget = QUICK_BUDGET
        if args.query_limit is None:
            args.query_limit = QUICK_QUERY_LIMIT
    elif args.budget is None:
        args.budget = DEFAULT_BUDGET
    if args.queries is None:
        args.queries = QUICK_NR_QUERIES if args.quick else workloads.DEFAULT_NR_QUERIES
    if args.sizes is None:
        args.sizes = list(
            workloads.QUICK_SWEEP if args.quick else workloads.WORLD_SWEEP
        )
    if args.raw_output is None:
        name = "benchmark_quick.json" if args.quick else "benchmark.json"
        args.raw_output = RAW_DIR / name
    return args


def main(argv: list[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    args = parse_args(argv)
    if args.preflight:
        return preflight(args)
    if args.render_only is not None:
        with open(args.render_only) as f:
            results = json.load(f)
    else:
        results = run(args, argv)
    if not args.no_render:
        from benchmarks import plot

        write_tables(results, args.tables_dir)
        plot.write_figures(results, args.figures_dir)
    report_failures = [
        f"{entry['map']['name']}: {result['status']}"
        for entry in results["world"] + results["examples"]
        for package, result in entry["results"].items()
        if (package, result["status"])
        in {
            ("extremitypathfinder", measure.INVALID),
            ("extremitypathfinder", measure.CRASH),
        }
    ]
    if report_failures:
        _log("extremitypathfinder failures:", *report_failures, sep="\n  ")
        if args.fail_on_invalid:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
