#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""迁移验证会话上下文：与 ROS MigrationSessionInfo 字段一致。"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np

from exp_paths import EXP_ROOT
from massage_robot_env import MassageRobotEnv
from scene_obstacles import load_scene_spheres


ALGO_FILE_SLUG = {
    "sc_rrt": "sc_rrt",
    "sc-rrt": "sc_rrt",
    "baseline_moveto": "baseline_moveto",
    "dynamic_rrt": "dynamic_rrt",
    "informed_rrtstar": "informed_rrtstar",
}


def _normalize_algorithm(algorithm: str) -> str:
    key = algorithm.lower().replace("-", "_")
    return ALGO_FILE_SLUG.get(key, key.replace("-", "_"))


def _load_cases(cases_file: Path) -> List[dict]:
    with open(cases_file, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data.get("cases", data)


def _find_case(cases: List[dict], case_id: int) -> dict:
    for c in cases:
        cid = int(c.get("selected_id", c.get("global_id", -1)))
        if cid == case_id:
            return c
    raise KeyError(f"case_id {case_id} not in cases file")


def _read_world_meta_obstacle_set(world_meta: Path) -> str:
    if not world_meta.is_file():
        return ""
    try:
        meta = json.loads(world_meta.read_text(encoding="utf-8"))
        return str(meta.get("obstacle_set", "") or "")
    except (json.JSONDecodeError, OSError):
        return ""


@dataclass
class MigrationSessionContext:
    case_id: int
    scene_id: str
    obstacle_set: str
    algorithm: str
    start_tcp: Tuple[float, float, float]
    goal_tcp: Tuple[float, float, float]
    path_file: str
    experiment_root: str
    gazebo_world_obstacle_set: str = ""
    num_tcp_waypoints: int = 0
    num_raw_waypoints: int = 0
    session_id: str = ""
    path_source: str = "disk"
    sync_warning: str = ""
    plan_ok: bool = True
    case: dict = field(repr=False, default_factory=dict)
    spheres: List[Tuple[np.ndarray, float]] = field(repr=False, default_factory=list)
    obstacles_config: Path = field(repr=False, default_factory=lambda: EXP_ROOT / "config" / "obstacles_app_v1.yaml")

    def build_planner_env(
        self,
        step_size: float = 0.02,
        goal_threshold: float = 0.04,
        safety_margin_m: float = 0.006,
    ) -> dict:
        """SC-RRT 唯一输入：起终点、障碍集与碰撞世界。"""
        return MassageRobotEnv.from_case(
            self.case,
            obstacles_config=self.obstacles_config,
            step_size=step_size,
            goal_threshold=goal_threshold,
            safety_margin_m=safety_margin_m,
        )


def load_session(
    case_id: int,
    algorithm: str = "sc_rrt",
    experiment_root: Optional[Path] = None,
    cases_file: Optional[Path] = None,
    path_file: Optional[Path] = None,
) -> MigrationSessionContext:
    """从磁盘组装迁移会话（与 broadcaster 共用）。"""
    root = Path(experiment_root or EXP_ROOT).resolve()
    algo_slug = _normalize_algorithm(algorithm)
    cases_path = cases_file or (root / "data" / "processed" / "app_single_wlzc_v1.json")
    case = _find_case(_load_cases(cases_path), int(case_id))

    pf = path_file or (root / "data" / "paths" / f"case_{int(case_id):02d}_{algo_slug}.json")
    try:
        path_rel = str(pf.relative_to(root))
    except ValueError:
        path_rel = str(pf)

    num_tcp = 0
    num_raw = 0
    path_obstacle_set = case.get("obstacle_set", "")
    if pf.is_file():
        pdata = json.loads(pf.read_text(encoding="utf-8"))
        num_tcp = len(pdata.get("poses", []))
        num_raw = len(pdata.get("raw_poses", pdata.get("poses", [])))
        path_obstacle_set = pdata.get("obstacle_set", path_obstacle_set) or path_obstacle_set

    obs_cfg = root / "config" / "obstacles_app_v1.yaml"
    spheres = load_scene_spheres(path_obstacle_set, obs_cfg)
    world_meta = root / "simulation" / "ros2" / "sc_rrt_gazebo" / "worlds" / "massage_scene.world.meta"
    gazebo_set = _read_world_meta_obstacle_set(world_meta)

    start = tuple(float(x) for x in case["start"]["pos"])
    goal = tuple(float(x) for x in case["goal"]["pos"])
    warnings: List[str] = []
    if path_obstacle_set and gazebo_set and path_obstacle_set != gazebo_set:
        warnings.append(f"path obstacle_set={path_obstacle_set} != gazebo meta={gazebo_set}")
    if case.get("obstacle_set") and path_obstacle_set != case.get("obstacle_set"):
        warnings.append(
            f"path obstacle_set={path_obstacle_set} != case={case.get('obstacle_set')}"
        )

    session_id = datetime.now().strftime("%Y%m%dT%H%M%S")
    return MigrationSessionContext(
        case_id=int(case_id),
        scene_id=str(case.get("scene_id", "")),
        obstacle_set=str(path_obstacle_set),
        algorithm=algo_slug,
        start_tcp=start,
        goal_tcp=goal,
        path_file=path_rel,
        experiment_root=str(root),
        gazebo_world_obstacle_set=gazebo_set,
        num_tcp_waypoints=num_tcp,
        num_raw_waypoints=num_raw,
        session_id=session_id,
        path_source="disk",
        sync_warning="; ".join(warnings),
        plan_ok=True,
        case=case,
        spheres=spheres,
        obstacles_config=obs_cfg,
    )
