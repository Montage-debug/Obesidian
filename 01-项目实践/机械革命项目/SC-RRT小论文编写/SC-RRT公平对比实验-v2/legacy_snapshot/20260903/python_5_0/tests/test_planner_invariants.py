"""规划器端点、碰撞与预算公平性回归测试。"""

from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "planning" / "src"))

from baselines import InformedRRTStar, RRTConnect
from sc_rrt.geometry import is_collision_free
from sc_rrt.sc_rrt_adaptive import SCRRTAdaptive


def _env() -> dict:
    start = np.array([0.45, -0.20, 0.24])
    goal = np.array([0.72, 0.20, 0.36])
    return {
        "dim": 3,
        "dimension": 3,
        "bounds": [0.42, 0.78, -0.30, 0.30, 0.12, 0.45],
        "start": start,
        "goal": goal,
        "start_point": start,
        "goal_point": goal,
        "obstacles": np.array([[0.585, 0.0, 0.30, 0.045]]),
        "step_size": 0.02,
        "goal_threshold": 0.04,
    }


def _mm_env(env: dict) -> dict:
    obs_mm = np.hstack([env["obstacles"][:, :3] * 1000.0, env["obstacles"][:, 3:4] * 1000.0])
    return {
        "dim": 3,
        "dimension": 3,
        "bounds": [b * 1000.0 for b in env["bounds"]],
        "start": env["start"] * 1000.0,
        "goal": env["goal"] * 1000.0,
        "obstacles": obs_mm,
        "step_size": 20.0,
        "goal_threshold": 40.0,
    }


def test_sc_rrt_uses_geometry_collision_not_fast_path():
    env = _mm_env(_env())
    planner = SCRRTAdaptive(
        env=env,
        max_iterations=50,
        mode="adaptive",
        safety_margin=6.0,
        verbose=False,
    )
    assert not hasattr(planner, "segment_collision_free")
    p1 = env["start"]
    p2 = env["goal"]
    assert planner._collision_free(p1, p2) == is_collision_free(p1, p2, env["obstacles"], 3)


def test_sc_rrt_stop_at_first_solution_when_extra_iters_zero():
    env = _mm_env(_env())
    np.random.seed(20260726)
    planner = SCRRTAdaptive(
        env=env,
        max_iterations=800,
        mode="adaptive",
        post_success_extra_iters=0,
        safety_margin=6.0,
        verbose=False,
    )
    _, _, success, metrics = planner.plan()
    if success and np.isfinite(metrics.get("first_solution_iter", np.inf)):
        assert metrics["iterations"] <= metrics["first_solution_iter"] + 1


def test_bound_prune_disabled_before_first_solution():
    env = _mm_env(_env())
    np.random.seed(7)
    planner = SCRRTAdaptive(
        env=env,
        max_iterations=120,
        mode="adaptive",
        enable_connect_prune=True,
        post_success_extra_iters=0,
        safety_margin=6.0,
        verbose=False,
    )
    _, _, _, metrics = planner.plan()
    if not metrics.get("success_rate"):
        assert metrics.get("bound_pruned_count", 0) == 0


def test_sc_rrt_path_endpoints_when_successful():
    env = _mm_env(_env())
    np.random.seed(99)
    path, _, success, _ = SCRRTAdaptive(
        env=env, max_iterations=3000, mode="adaptive", safety_margin=6.0, verbose=False,
    ).plan()
    if success and path is not None:
        assert np.allclose(path[0], env["start"], atol=1e-6)
        assert np.allclose(path[-1], env["goal"], atol=1e-6)


def test_workload_counters_consistent():
    env = _mm_env(_env())
    np.random.seed(11)
    _, _, _, m = SCRRTAdaptive(
        env=env, max_iterations=200, mode="adaptive", safety_margin=6.0, verbose=False,
    ).plan()
    att = m.get("expansion_attempts", 0)
    added = m.get("added_nodes", 0)
    assert added <= att
    assert m.get("collision_rejection_rate", 0) <= 1.0 + 1e-9


def test_rrt_connect_path_has_correct_endpoints_and_is_collision_free():
    env = _env()
    np.random.seed(20260724)
    result = RRTConnect(env, max_iterations=10000, safety_margin=0.006).plan()

    assert result["success"]
    path = np.asarray(result["path"], dtype=float)
    assert np.allclose(path[0], env["start_point"], atol=1e-9)
    assert np.allclose(path[-1], env["goal_point"], atol=1e-9)

    inflated = np.asarray(env["obstacles"], dtype=float).copy() * 1000.0
    inflated[:, 3] += 6.0
    path_mm = path * 1000.0
    assert all(
        is_collision_free(path_mm[i], path_mm[i + 1], inflated, 3)
        for i in range(len(path_mm) - 1)
    )


def test_informed_rrt_uses_requested_iteration_budget():
    planner = InformedRRTStar(_env(), max_iterations=10000, safety_margin=0.006)
    assert planner.max_iterations == 10000


def test_informed_rrt_plan_runs_without_numpy_math_gamma():
    np.random.seed(20260724)
    result = InformedRRTStar(_env(), max_iterations=500, safety_margin=0.006).plan()
    assert "path" in result and "success" in result
