"""
RRT 基线共享工具：毫米尺度环境、树操作、结果封装。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np

from collision_world import CollisionWorld
from massage_robot_env import MassageRobotEnv
from sc_rrt.geometry import (
    calculate_path_length,
    calculate_path_smoothness,
    find_nearest_node,
    is_collision_free,
    steer_point,
)


class RRTPlannerBase:
    """RRT 族算法公共基类（独立实现，不依赖 SCRRTAdaptive）。"""

    SCALE = 1000.0

    def __init__(
        self,
        env: Dict,
        max_iterations: int,
        algorithm: str,
        safety_margin: float = 0.0,
    ):
        self.env = env
        self.max_iterations = max_iterations
        self.algorithm = algorithm
        self.mm_env = MassageRobotEnv.to_mm_planner_env(env, self.SCALE)
        self.dim = int(self.mm_env.get("dim", 3))
        self.bounds = np.asarray(self.mm_env["bounds"], dtype=float)
        self.start = np.asarray(self.mm_env["start"], dtype=float)
        self.goal = np.asarray(self.mm_env["goal"], dtype=float)
        self.obstacles = self.mm_env.get("obstacles")
        self.safety_margin = float(safety_margin)
        if self.obstacles is not None and len(self.obstacles) > 0 and self.safety_margin > 0:
            self.obstacles = np.asarray(self.obstacles, dtype=float).copy()
            self.obstacles[:, 3] += self.safety_margin * self.SCALE
        self.step_size = float(self.mm_env.get("step_size", 20.0))
        self.goal_threshold = float(self.mm_env.get("goal_threshold", 40.0))
        cw = env.get("collision_world")
        if cw is None:
            cw = CollisionWorld([], env.get("boxes") or [], safety_margin_m=0.0)
        self._collision_world: CollisionWorld = cw

    def _segment_collision_free(self, p_near: np.ndarray, p_new: np.ndarray) -> bool:
        """球体 + 方盒统一碰撞检测（毫米坐标）。"""
        if self._collision_world.boxes:
            return self._collision_world.segment_free_m(
                p_near, p_new, self.obstacles, self.dim, self.SCALE,
            )
        return is_collision_free(p_near, p_new, self.obstacles, self.dim)

    def _new_tree(self, capacity: int = 2048) -> np.ndarray:
        return np.zeros((capacity, self.dim + 2), dtype=float)

    def _grow_tree(self, tree: np.ndarray, size: int, capacity: int) -> Tuple[np.ndarray, int, int]:
        if size < len(tree):
            return tree, size, capacity
        extra = max(capacity, 512)
        tree = np.vstack([tree, np.zeros((extra, self.dim + 2), dtype=float)])
        return tree, size, extra

    def _add_node(
        self,
        tree: np.ndarray,
        size: int,
        parent_idx: int,
        point: np.ndarray,
        cost: float,
    ) -> Tuple[np.ndarray, int]:
        tree[size, : self.dim] = point
        tree[size, self.dim] = parent_idx
        tree[size, self.dim + 1] = cost
        return tree, size + 1

    def _extend_toward(
        self,
        tree: np.ndarray,
        size: int,
        target: np.ndarray,
    ) -> Tuple[np.ndarray, int, Optional[int], np.ndarray]:
        nearest_idx = find_nearest_node(tree[:size], target, self.dim)
        x_near = tree[nearest_idx, : self.dim]
        x_new = steer_point(x_near, target, self.step_size)
        if not self._segment_collision_free(x_near, x_new):
            return tree, size, None, x_new
        cost = tree[nearest_idx, self.dim + 1] + float(np.linalg.norm(x_new - x_near))
        tree, size = self._add_node(tree, size, nearest_idx, x_new, cost)
        return tree, size, size - 1, x_new

    def _connect(
        self,
        tree: np.ndarray,
        size: int,
        target: np.ndarray,
    ) -> Tuple[np.ndarray, int, Optional[int]]:
        last_idx = None
        for _ in range(64):
            tree, size, new_idx, x_new = self._extend_toward(tree, size, target)
            if new_idx is None:
                break
            last_idx = new_idx
            if np.linalg.norm(x_new - target) <= self.goal_threshold:
                break
        return tree, size, last_idx

    def _backtrack_path(self, tree: np.ndarray, idx: int) -> List[np.ndarray]:
        pts = []
        current = idx
        while current >= 0:
            pts.append(tree[current, : self.dim].copy())
            parent = int(tree[current, self.dim])
            if parent == current:
                break
            current = parent
        pts.reverse()
        return pts

    def _build_result(
        self,
        path_mm: Optional[List[np.ndarray]],
        success: bool,
        planning_time: float,
        tree_nodes: int,
        failure_mode: str = "max_iterations",
    ) -> Dict:
        result_path = MassageRobotEnv.path_to_meters(path_mm, self.SCALE) if path_mm else []
        path_len = 0.0
        if success and path_mm:
            path_len = calculate_path_length(np.asarray(path_mm)) / self.SCALE

        straight = np.linalg.norm(self.env["goal_point"] - self.env["start_point"])
        path_eff = straight / path_len if success and path_len > 0 else 0.0
        smooth = 0.0
        if success and result_path:
            smooth = float(calculate_path_smoothness(np.asarray(result_path, dtype=float)))

        return {
            "success": bool(success),
            "path": result_path,
            "path_length": path_len if success else float("inf"),
            "planning_time": planning_time,
            "tree_nodes": tree_nodes,
            "path_efficiency": path_eff,
            "smoothness": smooth,
            "algorithm": self.algorithm,
            "failure_mode": failure_mode if not success else "none",
        }

    def plan(self) -> Dict:
        raise NotImplementedError
