"""multivia 拼接与 has_multivia 回归。"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "planning" / "src"))

from migration_multivia_planner import _concat_paths, has_multivia, plan_multivia, stage_chain_from_case
from massage_robot_env import MassageRobotEnv


def test_has_multivia():
    assert not has_multivia({"start": {"pos": [0, 0, 0]}, "goal": {"pos": [1, 0, 0]}})
    assert has_multivia({
        "start": {"pos": [0, 0, 0]},
        "goal": {"pos": [1, 0, 0]},
        "via_points": [{"pos": [0.5, 0, 0.2]}],
    })


def test_stage_chain_labels():
    case = {
        "start": {"pos": [0.44, -0.12, 0.18]},
        "goal": {"pos": [0.72, -0.24, 0.26]},
        "via_points": [{"pos": [0.5, 0.06, 0.34], "label": "s1"}],
    }
    chain, labels = stage_chain_from_case(case)
    assert len(chain) == 3
    assert labels == ["start", "s1", "goal"]


def test_concat_dedupe():
    a = [np.array([0, 0, 0]), np.array([1, 0, 0])]
    b = [np.array([1, 0, 0]), np.array([2, 0, 0])]
    out = _concat_paths([a, b])
    assert len(out) == 3


def test_plan_multivia_failure_propagates():
    case = {
        "start": {"pos": [0.44, -0.12, 0.18]},
        "goal": {"pos": [0.72, -0.24, 0.26]},
        "obstacle_set": "scene_s3_narrow",
        "via_points": [{"pos": [0.5, 0.06, 0.34]}],
    }
    obs_path = ROOT / "config/obstacles_app_v1.yaml"
    eb = MassageRobotEnv(obs_path, case["obstacle_set"])

    def fail_rrt(*_a, **_k):
        return {"success": False, "path": [], "planning_time": 0.0, "tree_nodes": 0}

    def identity_path(path, *_a, **_k):
        return path

    out = plan_multivia(
        case, eb, obs_path, 100, 0.006, {}, {"enabled": False},
        0.02, 0.04, fail_rrt, identity_path, {},
    )
    assert out["success"] is False
    assert "segment_0_failed" in out["failure_mode"]
