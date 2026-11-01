"""Timing, memory and budgets: each package and map is measured in its own subprocess.

The worker subprocess (``python -m benchmarks.measure TASK.json``)

1. imports the package and warms it up: one small environment is built and queried
   (compiles the Numba functions of extremitypathfinder; compilation is not measured),
2. resets and records its baseline peak memory (see :func:`peak_memory_bytes`),
3. builds the environment. Builds faster than ``REPEAT_BELOW`` are repeated
   until ``REPEAT_TOTAL`` seconds or ``REPEAT_MAX_RUNS`` runs, the minimum is reported,
4. reports the build on stdout (the parent kills the worker if this takes longer than the budget),
5. runs the queries, each timed with ``perf_counter_ns``,
   optionally stopping after a query time limit (the queries done are a prefix of the query set),
6. writes the timings, the paths, the graph size and the peak memory above the baseline to a JSON file.

Validation of the paths happens in the parent (``benchmarks.validate``), not in the measured process.
"""

import ctypes
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


class _RusageInfoV4(ctypes.Structure):
    """``struct rusage_info_v4`` of macOS (``<sys/resource.h>``)"""

    _fields_ = [
        ("ri_uuid", ctypes.c_uint8 * 16),
        *(
            (name, ctypes.c_uint64)
            for name in [
                "ri_user_time",
                "ri_system_time",
                "ri_pkg_idle_wkups",
                "ri_interrupt_wkups",
                "ri_pageins",
                "ri_wired_size",
                "ri_resident_size",
                "ri_phys_footprint",
                "ri_proc_start_abstime",
                "ri_proc_exit_abstime",
                "ri_child_user_time",
                "ri_child_system_time",
                "ri_child_pkg_idle_wkups",
                "ri_child_interrupt_wkups",
                "ri_child_pageins",
                "ri_child_elapsed_abstime",
                "ri_diskio_bytesread",
                "ri_diskio_byteswritten",
                "ri_cpu_time_qos_default",
                "ri_cpu_time_qos_maintenance",
                "ri_cpu_time_qos_background",
                "ri_cpu_time_qos_utility",
                "ri_cpu_time_qos_legacy",
                "ri_cpu_time_qos_user_initiated",
                "ri_cpu_time_qos_user_interactive",
                "ri_billed_system_time",
                "ri_serviced_system_time",
                "ri_logical_writes",
                "ri_lifetime_max_phys_footprint",
                "ri_instructions",
                "ri_cycles",
                "ri_billed_energy",
                "ri_serviced_energy",
                "ri_interval_max_phys_footprint",
                "ri_runnable_time",
            ]
        ),
    ]


RUSAGE_INFO_V4 = 4


def _darwin_rusage() -> "_RusageInfoV4 | None":
    try:
        libc = ctypes.CDLL("/usr/lib/libSystem.dylib")
        info = _RusageInfoV4()
        if libc.proc_pid_rusage(os.getpid(), RUSAGE_INFO_V4, ctypes.byref(info)) != 0:
            return None
    except (OSError, AttributeError):
        return None
    if not 0 < info.ri_phys_footprint <= info.ri_interval_max_phys_footprint:
        return None  # implausible: unexpected structure layout
    return info


def memory_method() -> str:
    """how the peak memory is measured on this machine"""
    if _proc_status_bytes("VmHWM") is not None:
        return "VmHWM"
    if sys.platform == "darwin" and _darwin_rusage() is not None:
        return "phys_footprint"
    return "ru_maxrss"


def reset_peak_memory() -> bool:
    """reset the peak memory of this process to its current value (Linux, macOS)"""
    method = memory_method()
    try:
        if method == "VmHWM":
            with open("/proc/self/clear_refs", "w") as f:
                f.write("5")
            return True
        if method == "phys_footprint":
            libc = ctypes.CDLL("/usr/lib/libSystem.dylib")
            return libc.proc_reset_footprint_interval(os.getpid()) == 0
    except (OSError, AttributeError):
        pass
    return False


def peak_memory_bytes() -> int:
    """the peak memory of this process

    - Linux: ``VmHWM`` (peak resident memory).
    - macOS: the peak physical footprint since the last reset (``proc_pid_rusage``),
      the memory figure of the Activity Monitor.
    - otherwise ``ru_maxrss``. It may include the peak of the parent process:
      ``ru_maxrss`` survives ``execve`` on Linux, the methods above do not.
    """
    method = memory_method()
    if method == "VmHWM":
        return _proc_status_bytes("VmHWM")
    if method == "phys_footprint":
        return _darwin_rusage().ri_interval_max_phys_footprint
    maxrss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return maxrss if sys.platform == "darwin" else maxrss * 1024


def memory_probe(nr_bytes: int) -> dict:
    """allocate and touch ``nr_bytes``: the measured peak must grow by about that much"""
    reset = reset_peak_memory()
    baseline = peak_memory_bytes()
    block = bytearray(nr_bytes)
    for i in range(0, nr_bytes, 4096):
        block[i] = 1
    peak = peak_memory_bytes() - baseline
    del block
    return {
        "method": memory_method(),
        "reset": reset,
        "baseline": baseline,
        "peak": peak,
    }


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
    info["memory_method"] = memory_method()
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
        try:
            return _supervise(process, output, budget, query_limit)
        finally:
            # also on an interruption of the benchmark (Ctrl-C): no orphaned worker
            if process.poll() is None:
                process.kill()
                process.wait()


def _supervise(process: subprocess.Popen, output: Path, budget, query_limit) -> dict:
    """wait for the worker within the budget and the query time limit, its result"""
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
        info["memory_bytes"] = os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE")
    except (ValueError, OSError, AttributeError):
        info["memory_bytes"] = _sysctl("hw.memsize", int)
    if sys.platform == "darwin":
        info["power_source"] = power_source()
    return info


def _sysctl(name: str, cast=str):
    try:
        return cast(subprocess.check_output(["sysctl", "-n", name], text=True).strip())
    except (OSError, ValueError, subprocess.CalledProcessError):
        return None


def power_source() -> str | None:
    """macOS: "AC Power" or "Battery Power" (laptops throttle on battery)"""
    try:
        output = subprocess.check_output(["pmset", "-g", "ps"], text=True)
    except (OSError, subprocess.CalledProcessError):
        return None
    first = output.splitlines()[0] if output else ""
    return first.split("'")[1] if first.count("'") >= 2 else None


def _cpu_model() -> str:
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    if sys.platform == "darwin":
        # e.g. "Apple M2 Pro"
        model = _sysctl("machdep.cpu.brand_string")
        if model:
            return model
    return platform.processor()


if __name__ == "__main__":
    if sys.argv[1] == "--memory-probe":
        print(json.dumps(memory_probe(int(sys.argv[2]))))
    else:
        worker_main(sys.argv[1])
