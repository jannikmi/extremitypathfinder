"""Timing, memory and budgets: each package and map is measured in its own subprocess.

The worker subprocess (``python -m benchmarks.measure TASK.json``)

1. imports the package and warms it up: one small environment is built and queried
   (compiles the Numba functions of extremitypathfinder; compilation is not measured),
2. records its baseline peak memory (Linux: ``VmHWM``, reset after the warm-up; elsewhere ``ru_maxrss``),
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
import select
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
WORKER_COMMAND = [sys.executable, "-m", "benchmarks.measure"]
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
# worker progress
BUILT = "built"
EXITED = "exited"
EVENT_PREFIX = b"@epf-benchmark-event "


def _proc_status_bytes(key: str) -> int | None:
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith(key + ":"):
                    return int(line.split()[1]) * 1024  # kB
    except OSError:
        pass
    return None


def reset_peak_memory() -> bool:
    """reset the peak resident memory of this process to its current value (Linux)"""
    try:
        with open("/proc/self/clear_refs", "w") as f:
            f.write("5")
        return True
    except OSError:
        return False


def peak_memory_bytes() -> int:
    """the peak resident memory of this process

    Linux: ``VmHWM``. Unlike ``ru_maxrss`` it is not inherited from the parent process
    (``ru_maxrss`` survives ``execve``: the worker would start with the peak of the benchmark runner).
    Other platforms: ``ru_maxrss`` (macOS: bytes), possibly including the peak of the parent.
    """
    peak = _proc_status_bytes("VmHWM")
    if peak is not None:
        return peak
    maxrss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return maxrss if sys.platform == "darwin" else maxrss * 1024


def _emit(event: dict):
    sys.stdout.buffer.write(EVENT_PREFIX + json.dumps(event).encode() + b"\n")
    sys.stdout.buffer.flush()


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
    holes = [[tuple(c) for c in hole] for hole in task["holes"]]
    # the baseline: after the imports, the warm-up and the input data
    info["memory_peak_reset"] = reset_peak_memory()
    baseline = peak_memory_bytes()

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
            [*WORKER_COMMAND, str(task_path)],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            env=env,
        )
        started = time.perf_counter()
        progress = _wait_for_build(process, budget)
        if progress != BUILT:
            process.kill()
            process.communicate()
            if progress == OVER_BUDGET:
                return {
                    "status": OVER_BUDGET,
                    "budget": budget,
                    "elapsed": time.perf_counter() - started,
                }
            if output.exists():
                with open(output) as f:
                    return json.load(f)  # exception during the build
            killed = " (killed, e.g. out of memory)" if process.returncode < 0 else ""
            return {
                "status": CRASH,
                "error": f"worker exited with code {process.returncode} during the build{killed}",
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


def _wait_for_build(process: subprocess.Popen, budget) -> str:
    """wait for the build of the worker: ``BUILT``, ``OVER_BUDGET`` or ``EXITED``

    The budget starts with the build, after the imports and the warm-up.
    Reads the pipe unbuffered (no lost wake-up when both events arrive in one chunk)
    and ignores any output of the packages which is not an event of the worker.
    """
    fd = process.stdout.fileno()
    buffer = b""
    deadline = None
    while True:
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            if not line.startswith(EVENT_PREFIX):
                continue
            event = json.loads(line[len(EVENT_PREFIX) :])
            if event["event"] == "build_start" and budget is not None:
                deadline = time.perf_counter() + budget
            if event["event"] == "built":
                if budget is None or event["build_time"] <= budget:
                    return BUILT
                return OVER_BUDGET
        timeout = None
        if deadline is not None:
            timeout = deadline - time.perf_counter()
            if timeout <= 0:
                return OVER_BUDGET
        ready, _, _ = select.select([fd], [], [], timeout)
        if not ready:
            return OVER_BUDGET
        chunk = os.read(fd, 65536)
        if not chunk:
            return EXITED
        buffer += chunk


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
