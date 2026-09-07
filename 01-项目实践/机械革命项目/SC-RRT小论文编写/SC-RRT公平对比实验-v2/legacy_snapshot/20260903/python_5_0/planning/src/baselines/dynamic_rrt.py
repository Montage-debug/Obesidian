"""
Dynamic-RRT：双向树 + 动态交汇点 + 固定椭球约束（无自适应控制器）。
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from sc_rrt.geometry import is_collision_free, sample_in_ellipsoid, sample_point

from .rrt_planner_base import RRTPlannerBase


class DynamicRRT(RRTPlannerBase):
    """双向 Dynamic-RRT：meet_point 平滑更新 + 固定 gamma/p 椭球采样。"""

    def __init__(
        self,
        env: Dict,
        max_iterations: int = 8000,
        gamma: float = 4.0,
        p_informed: float = 0.3,
        meet_smooth: float = 0.7,
        safety_margin: float = 0.0,
    ):
        super().__init__(env, max_iterations, "Dynamic-RRT", safety_margin)
        self.gamma = gamma
        self.p_informed = p_informed
        self.meet_smooth = meet_smooth

    def _sample(
        self,
        focus1: np.ndarray,
        focus2: np.ndarray,
        c_best: float,
    ) -> np.ndarray:
        if np.random.rand() < self.p_informed:
            pt = sample_in_ellipsoid(
                focus1, focus2, c_best, self.dim, bounds=self.bounds, max_attempts=12,
            )
            if pt is not None:
                return pt
        return sample_point(self.bounds, self.dim)

    def _meet_point(
        self,
        tree_a: np.ndarray,
        size_a: int,
        tree_b: np.ndarray,
        size_b: int,
        current: np.ndarray,
    ) -> np.ndarray:
        if size_a < 2 or size_b < 2:
            return current
        min_d = np.inf
        mid = current.copy()
        for i in range(min(size_a, 40)):
            for j in range(min(size_b, 40)):
                d = float(np.linalg.norm(tree_a[i, : self.dim] - tree_b[j, : self.dim]))
                if d < min_d:
                    min_d = d
                    mid = (tree_a[i, : self.dim] + tree_b[j, : self.dim]) / 2.0
        centroid = (
            np.mean(tree_a[:size_a, : self.dim], axis=0)
            + np.mean(tree_b[:size_b, : self.dim], axis=0)
        ) / 2.0
        target = 0.5 * mid + 0.5 * centroid
        return self.meet_smooth * current + (1.0 - self.meet_smooth) * target

    def _try_connect(
        self,
        tree_from: np.ndarray,
        size_from: int,
        tree_to: np.ndarray,
        size_to: int,
        idx_from: int,
    ) -> Optional[List[np.ndarray]]:
        x_new = tree_from[idx_from, : self.dim]
        nearest_b = int(np.argmin(
            np.linalg.norm(tree_to[:size_to, : self.dim] - x_new, axis=1)
        ))
        x_near_b = tree_to[nearest_b, : self.dim]
        if not is_collision_free(x_new, x_near_b, self.obstacles, self.dim):
            return None
        path_a = self._backtrack_path(tree_from, idx_from)
        path_b = self._backtrack_path(tree_to, nearest_b)
        path_b.reverse()
        if path_b and np.allclose(path_b[0], path_a[-1]):
            path_b = path_b[1:]
        return path_a + path_b

    def plan(self) -> Dict:
        t0 = time.perf_counter()
        cap = max(512, self.max_iterations // 8)
        tree_a = self._new_tree(cap)
        tree_b = self._new_tree(cap)
        tree_a[0, : self.dim] = self.start
        tree_a[0, self.dim] = -1
        tree_a[0, self.dim + 1] = 0.0
        tree_b[0, : self.dim] = self.goal
        tree_b[0, self.dim] = -1
        tree_b[0, self.dim + 1] = 0.0
        size_a, size_b = 1, 1
        cap_a, cap_b = cap, cap

        meet = (self.start + self.goal) / 2.0
        success = False
        best_path: Optional[List[np.ndarray]] = None

        for it in range(1, self.max_iterations + 1):
            if it % 40 == 0:
                meet = self._meet_point(tree_a, size_a, tree_b, size_b, meet)

            c_min_a = float(np.linalg.norm(meet - self.start))
            c_min_b = float(np.linalg.norm(self.goal - meet))
            c_best_a = c_min_a * self.gamma
            c_best_b = c_min_b * self.gamma

            for side in ("a", "b"):
                if side == "a":
                    x_rand = self._sample(self.start, meet, c_best_a)
                    tree_a, size_a, cap_a = self._grow_tree(tree_a, size_a, cap_a)
                    tree_a, size_a, new_idx, x_new = self._extend_toward(tree_a, size_a, x_rand)
                    if new_idx is None:
                        continue
                    if np.linalg.norm(x_new - meet) <= self.goal_threshold:
                        path = self._try_connect(tree_a, size_a, tree_b, size_b, new_idx)
                        if path:
                            best_path = path
                            success = True
                            break
                else:
                    x_rand = self._sample(meet, self.goal, c_best_b)
                    tree_b, size_b, cap_b = self._grow_tree(tree_b, size_b, cap_b)
                    tree_b, size_b, new_idx, x_new = self._extend_toward(tree_b, size_b, x_rand)
                    if new_idx is None:
                        continue
                    if np.linalg.norm(x_new - meet) <= self.goal_threshold:
                        path = self._try_connect(tree_b, size_b, tree_a, size_a, new_idx)
                        if path:
                            best_path = list(reversed(path))
                            success = True
                            break
            if success:
                break

        elapsed = time.perf_counter() - t0
        return self._build_result(best_path, success, elapsed, size_a + size_b)
