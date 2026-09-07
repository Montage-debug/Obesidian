#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""迁移 case SC-RRT 规划封装（单段 / multivia）。"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, Optional

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "planning" / "src"))
sys.path.insert(0, str(ROOT / "planning" / "scripts"))

from arm_segment_checker import attach_arm_checker_to_env
from migration_multivia_planner import has_multivia, plan_multivia
from migration_session_context import MigrationSessionContext


def plan_migration_case(
    ctx: MigrationSessionContext,
    max_iterations: int,
    safety_margin: float,
    adaptive_cfg: Optional[dict] = None,
    arm_cfg: Optional[dict] = None,
    step_size: float = 0.02,
    goal_threshold: float = 0.04,
    pp_cfg: Optional[dict] = None,
    env_builder=None,
) -> Dict[str, Any]:
    from run_massage_robot_experiment import finalize_path, run_sc_rrt

    case = ctx.case
    obstacles_config = ctx.obstacles_config

    if has_multivia(case):
        if env_builder is None:
            from massage_robot_env import MassageRobotEnv
            env_builder = MassageRobotEnv(obstacles_config, case.get("obstacle_set"))
        return plan_multivia(
            case, env_builder, obstacles_config, max_iterations, safety_margin,
            adaptive_cfg, arm_cfg, step_size, goal_threshold,
            run_sc_rrt, finalize_path, pp_cfg or {},
        )

    env = ctx.build_planner_env(
        step_size=step_size, goal_threshold=goal_threshold, safety_margin_m=safety_margin,
    )
    env["case"] = case
    if arm_cfg:
        attach_arm_checker_to_env(env, arm_cfg, spheres=ctx.spheres)
        checker = env.get("arm_segment_checker")
        if checker is not None:
            checker.reset_seed()
    out = run_sc_rrt(env, max_iterations, safety_margin, adaptive_cfg or {})
    out["planning_mode"] = "single"
    out["num_stages"] = 1
    return out
