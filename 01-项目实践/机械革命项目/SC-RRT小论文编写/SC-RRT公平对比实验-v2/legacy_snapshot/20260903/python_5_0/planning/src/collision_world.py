"""
四算法共用的 TCP 点碰撞世界：球体簇 + 可选 AABB，统一安全裕度膨胀。
"""

from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

from sc_rrt.geometry import is_collision_free


class CollisionWorld:
    """规划层碰撞模型（TCP 点 + 球/AABB，不含整臂 URDF）。"""

    def __init__(
        self,
        spheres: Sequence[Dict],
        boxes: Sequence[Dict] | None = None,
        safety_margin_m: float = 0.0,
    ):
        self.spheres = list(spheres or [])
        self.boxes = list(boxes or [])
        self.safety_margin_m = float(safety_margin_m)

    @classmethod
    def from_env_builder(
        cls,
        env_builder,
        safety_margin_m: float = 0.0,
    ) -> "CollisionWorld":
        return cls(env_builder.spheres, env_builder.boxes, safety_margin_m)

    def planner_obstacles_m(self) -> np.ndarray:
        """仅人体/场景球体，供 RRT 球碰撞（米）。"""
        obs = [
            [*s["center"], s["radius"] + self.safety_margin_m]
            for s in self.spheres
        ]
        return np.asarray(obs, dtype=float) if obs else np.zeros((0, 4))

    def _point_in_box(self, point: np.ndarray, box: Dict) -> bool:
        diff = np.abs(np.asarray(point, dtype=float) - box["center"])
        margin = self.safety_margin_m
        return bool(np.all(diff < box["half_size"] + margin))

    def segment_free_m(
        self,
        p1: np.ndarray,
        p2: np.ndarray,
        sphere_obstacles_mm: np.ndarray | None,
        dim: int,
        scale: float = 1000.0,
        num_checks: int = 32,
    ) -> bool:
        """球体用 geometry；方盒用线段采样（毫米坐标）。"""
        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        if sphere_obstacles_mm is not None and len(sphere_obstacles_mm) > 0:
            if not is_collision_free(p1, p2, sphere_obstacles_mm, dim):
                return False
        if not self.boxes:
            return True
        for t in np.linspace(0.0, 1.0, num_checks):
            pt_m = (p1 + t * (p2 - p1)) / scale
            for box in self.boxes:
                if self._point_in_box(pt_m, box):
                    return False
        return True
