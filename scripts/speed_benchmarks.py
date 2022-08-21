import random
import timeit
from typing import Callable

from extremitypathfinder import PolygonEnvironment
from tests.test_cases import POLYGON_ENVS

RUNS_ENV_PREP = int(1e3)
RUNS_QUERY = int(1e3)


def timefunc(function: Callable, nr_runs: int, *args):
    def wrap():
        function(*args)

    timer = timeit.Timer(wrap)
    t_in_sec = timer.timeit(nr_runs)
    return t_in_sec


def get_random_env():
    return random.choice(POLYGON_ENVS)


def get_prepared_env(env_data):
    env = PolygonEnvironment()
    env.store(*env_data)
    env.prepare()
    return env


def get_rnd_query_pts(env):
    # return any of the environments points
    start_idx = random.randint(0, env.nr_vertices - 1)
    goal_idx = random.randint(0, env.nr_vertices - 1)
    start = env.coords[start_idx]
    goal = env.coords[goal_idx]
    return goal, start


def eval_time_fct():
    global tf, point_list
    for point in point_list:
        tf.timezone_at(lng=point[0], lat=point[1])


def test_env_preparation_speed():
    def run_func():
        env_data = get_random_env()
        _ = get_prepared_env(env_data)

    print()
    t = timefunc(run_func, RUNS_ENV_PREP)
    t_avg = t / RUNS_ENV_PREP
    pts_p_sec = t_avg**-1
    print(f"avg. environment preparation time {t_avg:.1e} s/run, {pts_p_sec:.1e} runs/s")
    print(f"averaged over {RUNS_ENV_PREP:,} runs")


def test_query_speed():
    prepared_envs = [get_prepared_env(d) for d in POLYGON_ENVS]

    def run_func():
        env = random.choice(prepared_envs)
        goal, start = get_rnd_query_pts(env)
        p, d = env.find_shortest_path(start, goal, verify=False)
        x = 1

    print()
    t = timefunc(run_func, RUNS_QUERY)
    t_avg = t / RUNS_QUERY
    pts_p_sec = t_avg**-1
    print(f"avg. query time {t_avg:.1e} s/run, {pts_p_sec:.1e} runs/s")
    print(f"averaged over {RUNS_QUERY:,} runs")
