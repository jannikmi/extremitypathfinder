"""Timing, memory and budgets: each package and map is measured in its own subprocess.

The worker subprocess (``python -m benchmarks.measure TASK.json``)

1. imports the package and warms it up: one small environment is built and queried
   (compiles the Numba functions of extremitypathfinder; compilation is not measured),
2. records its baseline peak memory (``ru_maxrss``),
3. builds the environment. Builds faster than ``REPEAT_BELOW`` are repeated
   until ``REPEAT_TOTAL`` seconds or ``REPEAT_MAX_RUNS`` runs, the minimum is reported,
4. reports the build on stdout (the parent kills the worker if this takes longer than the budget),
5. runs the queries, each timed with ``perf_counter_ns``,
   optionally stopping after a query time limit (the queries done are a prefix of the query set),
6. writes the timings, the paths, the graph size and the peak memory above the baseline to a JSON file.

Validation of the paths happens in the parent (``benchmarks.validate``), not in the measured process.
"""

import json
import os
import platform
import resource
import selectors
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

from benchmarks.adapters import get_adapter, path_length

REPEAT_BELOW = 1.0  # s
REPEAT_TOTAL = 5.0  # s
REPEAT_MAX_RUNS = 5
QUERY_LIMIT_GRACE = 60.0  # s
REPO_ROOT = Path(__file__).resolve().parents[1]

# warm-up environment: two disjoint squares, the query has to pass between them
WARMUP_HOLES = [
    [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
    [(2.0, 0.0), (3.0, 0.0), (3.0, 1.0), (2.0, 1.0)],
]
WARMUP_QUERY = ((-1.0, 0.5), (4.0, 0.5))

OK = "ok"
INVALID = "invalid"
OVER_BUDGET = "over budget"
CRASH = "crash"


def peak_memory_bytes() -> int:
    """the peak resident memory of this process"""
    maxrss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    # Linux: kilobytes, macOS: bytes
    return maxrss if sys.platform == "darwin" else maxrss * 1024


def _emit(event: dict):
    print(json.dumps(event), flush=True)


def timed_build(adapter, holes) -> tuple[object, float, list[float]]:
    """the environment and the minimum build time (repeated if faster than REPEAT_BELOW)"""
    times = []
    total_start = time.perf_counter()
    while True:
        env = None  # free the previous environment before the next build
        start = time.perf_counter_ns()
        env = adapter.build(holes)
        times.append((time.perf_counter_ns() - start) * 1e-9)
        elapsed = time.perf_counter() - total_start
        done = (
            times[0] >= REPEAT_BELOW,
            len(times) >= REPEAT_MAX_RUNS,
            elapsed >= REPEAT_TOTAL,
        )
        if any(done):
            return env, min(times), times


def run_task(task: dict) -> dict:
    """the measurement of one package on one map (runs in the worker process)"""
    adapter = get_adapter(task["package"])
    warmup_env = adapter.build(WARMUP_HOLES)
    adapter.query(warmup_env, *WARMUP_QUERY)
    del warmup_env
    info = {"version": adapter.version()}
    if task["package"] == "extremitypathfinder":
        info["numba"] = adapter.numba_active()
    baseline = peak_memory_bytes()

    holes = [[tuple(c) for c in hole] for hole in task["holes"]]
    _emit({"event": "build_start"})
    env, build_time, build_times = timed_build(adapter, holes)
    _emit({"event": "built", "build_time": build_time})
    nodes, edges = adapter.graph_size(env)

    query_limit = task.get("query_limit")
    query_times = []
    paths = []
    limit_reached = False
    queries_start = time.perf_counter()
    for start, goal in task["queries"]:
        elapsed = time.perf_counter() - queries_start
        if query_limit is not None and elapsed > query_limit:
            limit_reached = True
            break
        t0 = time.perf_counter_ns()
        path = adapter.query(env, tuple(start), tuple(goal))
        query_times.append(time.perf_counter_ns() - t0)
        paths.append(path)

    return {
        **info,
        "build_time": build_time,
        "build_times": build_times,
        "graph_nodes": nodes,
        "graph_edges": edges,
        "query_times_ns": query_times,
        "queries_completed": len(query_times),
        "query_limit_reached": limit_reached,
        "lengths": [path_length(p) for p in paths],
        "paths": [[list(p) for p in path] for path in paths],
        "memory_baseline": baseline,
        "memory_peak": peak_memory_bytes() - baseline,
    }


def worker_main(task_path: str):
    with open(task_path) as f:
        task = json.load(f)
    try:
        result = {"status": "done", **run_task(task)}
    except Exception as exc:  # noqa: BLE001 any failure of the package is a result
        result = {
            "status": CRASH,
            "error": f"{type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        }
    with open(task["output"], "w") as f:
        json.dump(result, f)


def measure(
    package: str,
    holes,
    queries,
    budget: float | None,
    query_limit: float | None = None,
) -> dict:
    """measure one package on one map in a subprocess

    :param budget: build time budget in seconds: the subprocess is killed when the build
        takes longer (status ``over budget``). None: no limit.
    :param query_limit: time limit of the query phase in seconds. None: no limit.
    :return: the measurement with ``status`` ``done``, ``crash`` or ``over budget``
    """
    with tempfile.TemporaryDirectory(prefix="epf_benchmark_") as tmp:
        task_path = Path(tmp) / "task.json"
        output = Path(tmp) / "result.json"
        with open(task_path, "w") as f:
            json.dump(
                {
                    "package": package,
                    "holes": holes,
                    "queries": queries,
                    "query_limit": query_limit,
                    "output": str(output),
                },
                f,
            )
        env = {**os.environ, "PYTHONHASHSEED": "0"}
        process = subprocess.Popen(
            [sys.executable, "-m", "benchmarks.measure", str(task_path)],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            text=True,
            env=env,
        )
        started = time.perf_counter()
        built = _wait_for_build(process, started, budget)
        if not built:
            process.kill()
            process.wait()
            if process.returncode is not None and output.exists():
                with open(output) as f:
                    return json.load(f)  # crashed during the build
            return {
                "status": OVER_BUDGET,
                "budget": budget,
                "elapsed": time.perf_counter() - started,
            }
        # hard limit of the query phase: the limit is checked between queries,
        # a hanging query is killed after a grace period
        timeout = None if query_limit is None else query_limit + QUERY_LIMIT_GRACE
        try:
            process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
            return {
                "status": CRASH,
                "error": f"query phase killed after {timeout:g} s (query time limit {query_limit:g} s)",
            }
        if not output.exists():
            return {
                "status": CRASH,
                "error": f"worker exited with code {process.returncode} without result",
            }
        with open(output) as f:
            return json.load(f)


def _wait_for_build(process: subprocess.Popen, started: float, budget) -> bool:
    """wait for the build of the worker. False: over budget or exited without build

    The budget starts with the build, after the imports and the warm-up.
    """
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    deadline = None
    try:
        while True:
            timeout = None
            if deadline is not None:
                timeout = deadline - time.perf_counter()
                if timeout <= 0:
                    return False
            if not selector.select(timeout):
                return False
            line = process.stdout.readline()
            if not line:
                return False  # exited
            event = json.loads(line)
            if event.get("event") == "build_start" and budget is not None:
                deadline = time.perf_counter() + budget
            if event.get("event") == "built":
                return budget is None or event["build_time"] <= budget
    finally:
        selector.close()


def system_info() -> dict:
    """hardware and software of this machine"""
    info = {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "cpu_model": _cpu_model(),
        "cpu_count": os.cpu_count(),
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
    }
    try:
        pages = os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE")
        info["memory_bytes"] = pages
    except (ValueError, OSError, AttributeError):
        info["memory_bytes"] = None
    return info


def _cpu_model() -> str:
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    if sys.platform == "darwin":
        try:
            return subprocess.check_output(
                ["sysctl", "-n", "machdep.cpu.brand_string"], text=True
            ).strip()
        except (OSError, subprocess.CalledProcessError):
            pass
    return platform.processor()


if __name__ == "__main__":
    worker_main(sys.argv[1])
