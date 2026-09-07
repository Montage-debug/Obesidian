from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from core import Environment, path_is_valid, segment_is_free
from environments import generate_environment, grid_path_exists
from planners import build_planner


def config() -> dict:
    return json.loads((ROOT / "config" / "protocol.json").read_text(encoding="utf-8"))


def easy_environment(dim: int) -> Environment:
    bounds = np.tile([0.0, 300.0], (dim, 1))
    start = np.full(dim, 15.0)
    goal = np.full(dim, 285.0)
    center = np.full(dim, 150.0)
    obstacles = np.asarray([np.r_[center, 18.0]])
    return Environment("test", dim, bounds, start, goal, obstacles, "test", 1)


def test_segment_collision_2d_and_3d() -> None:
    for dim in (2, 3):
        center = np.zeros(dim)
        obstacle = np.asarray([np.r_[center, 1.0]])
        assert not segment_is_free(np.full(dim, -2.0), np.full(dim, 2.0), obstacle)
        a = np.full(dim, 2.0)
        b = np.full(dim, 3.0)
        assert segment_is_free(a, b, obstacle)


def test_all_variants_use_exact_budget_and_return_valid_paths() -> None:
    cfg = config()
    cfg["max_extension_attempts"] = 400
    cfg["checkpoints"] = [100, 200, 400]
    algorithms = cfg["main_algorithms"] + cfg["ablation_algorithms"]
    for dim in (2, 3):
        env = easy_environment(dim)
        for algorithm in algorithms:
            result = build_planner(algorithm, env, 12345, cfg).plan()
            assert result.budget.attempts == 400
            assert set(result.budget.trace) == {100, 200, 400}
            costs = [result.budget.trace[x] for x in (100, 200, 400)]
            assert all(costs[i + 1] <= costs[i] for i in range(2))
            if result.success:
                assert path_is_valid(result.path, env)
            for value in result.extra.values():
                if isinstance(value, float):
                    assert not np.isnan(value)


def test_reproducible_geometry_and_cost() -> None:
    cfg = config()
    cfg["max_extension_attempts"] = 300
    cfg["checkpoints"] = [300]
    env = easy_environment(2)
    first = build_planner("SC-RRT", env, 88, cfg).plan()
    second = build_planner("SC-RRT", env, 88, cfg).plan()
    assert first.budget.attempts == second.budget.attempts
    assert first.budget.added_nodes == second.budget.added_nodes
    assert first.best_cost == second.best_cost
    if first.success:
        assert np.array_equal(first.path, second.path)


def test_generated_maps_meet_locked_constraints() -> None:
    cfg = config()
    for dim in (2, 3):
        env = generate_environment(dim, 0, "calibration", cfg)
        spec = cfg["dimensions"][str(dim)]
        assert len(env.obstacles) == spec["obstacle_count"]
        assert not segment_is_free(env.start, env.goal, env.obstacles)
        assert grid_path_exists(env, spec["feasibility_grid_mm"])
