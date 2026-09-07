#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""臂体胶囊段碰撞：沿 TCP 段 IK 采样并检验连杆净空。"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parent.parent.parent
ROS2_PKG = ROOT / "simulation" / "ros2" / "sc_rrt_gazebo"
if ROS2_PKG.is_dir():
    sys.path.insert(0, str(ROS2_PKG))

URDF_FALLBACK = ROS2_PKG / "urdf" / "GJ1_aubo_i5_ik.urdf"


def _load_ik_solver(urdf_path: Optional[Path] = None):
    try:
        from sc_rrt_gazebo.ik_solver import AuboIKSolver, MASSAGE_TRANSIT_JOINTS
    except ImportError as exc:
        raise ImportError(
            "无法导入 sc_rrt_gazebo.ik_solver；请 colcon build sc_rrt_gazebo 并 source install"
        ) from exc

    urdf = Path(urdf_path) if urdf_path else URDF_FALLBACK
    if not urdf.is_file():
        from sc_rrt_gazebo.ik_solver import default_ik_urdf_path

        urdf = default_ik_urdf_path()
    return AuboIKSolver(urdf, orientation_joints=MASSAGE_TRANSIT_JOINTS)


class ArmSegmentChecker:
    """TCP 段几何碰撞通过后，用 IK 采样检验臂体胶囊。"""

    def __init__(
        self,
        spheres: Sequence[Tuple[Sequence[float], float]],
        min_arm_clearance_m: float = 0.006,
        segment_samples: int = 8,
        urdf_path: Optional[Path] = None,
    ):
        self.spheres = [(np.asarray(c, dtype=float), float(r)) for c, r in spheres]
        self.min_arm_clearance_m = float(min_arm_clearance_m)
        self.segment_samples = max(2, int(segment_samples))
        self._solver = _load_ik_solver(urdf_path)
        self._last_q: List[float] = list(self._solver.transit_joints)

    def reset_seed(self) -> None:
        self._last_q = list(self._solver.transit_joints)

    def set_seed(self, q: Sequence[float]) -> None:
        self._last_q = list(q)

    def segment_free(self, p1_m: Sequence[float], p2_m: Sequence[float]) -> bool:
        a = np.asarray(p1_m, dtype=float)
        b = np.asarray(p2_m, dtype=float)
        if not self.spheres:
            return True
        seed = list(self._last_q)
        for k in range(self.segment_samples + 1):
            t = k / self.segment_samples
            pos = a + t * (b - a)
            q, ok = self._solver.solve_position(pos, seed)
            if not ok:
                return False
            clear = self._solver.arm_clearance_to_spheres(q, self.spheres)
            if clear < self.min_arm_clearance_m:
                return False
            seed = q
        self._last_q = seed
        return True


def attach_arm_checker_to_env(
    env: dict,
    arm_cfg: dict,
    spheres: Optional[Sequence[Tuple[Sequence[float], float]]] = None,
    urdf_path: Optional[Path] = None,
) -> Optional[ArmSegmentChecker]:
    if not arm_cfg or not arm_cfg.get("enabled", False):
        return None
    if spheres is None:
        from scene_obstacles import load_scene_spheres

        case = env.get("case") or {}
        obs_set = case.get("obstacle_set")
        spheres = load_scene_spheres(obs_set) if obs_set else []
    checker = ArmSegmentChecker(
        spheres,
        min_arm_clearance_m=float(arm_cfg.get("min_arm_clearance_m", 0.006)),
        segment_samples=int(arm_cfg.get("segment_samples", 8)),
        urdf_path=urdf_path,
    )
    env["arm_segment_checker"] = checker
    return checker
