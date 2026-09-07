"""
RRT-Connect：标准双向连接，无椭球约束、无自适应控制器。
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

import numpy as np

from sc_rrt.geometry import sample_point

from .rrt_planner_base import RRTPlannerBase


class RRTConnect(RRTPlannerBase):
    """经典 RRT-Connect 基线。"""

    def __init__(
        self,
        env: Dict,
        max_iterations: int = 8000,
        safety_margin: float = 0.0,
    ):
        super().__init__(env, max_iterations, "RRT-Connect", safety_margin)

    def _extract_bidir_path(
        self,
        tree_a: np.ndarray,
        idx_a: int,
        tree_b: np.ndarray,
        idx_b: int,
    ) -> List[np.ndarray]:
        path_a = self._backtrack_path(tree_a, idx_a)
        path_b = self._backtrack_path(tree_b, idx_b)
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
        # 目标树必须以 goal 为根；零初始化会把回溯路径错误地拼到原点。
        tree_b[0, : self.dim] = self.goal
        tree_b[0, self.dim] = -1
        tree_b[0, self.dim + 1] = 0.0
        size_a, size_b = 1, 1
        cap_a, cap_b = cap, cap

        success = False
        best_path: Optional[List[np.ndarray]] = None
        extend_a = True

        for _ in range(self.max_iterations):
            x_rand = sample_point(self.bounds, self.dim)
            if extend_a:
                tree_a, size_a, cap_a = self._grow_tree(tree_a, size_a, cap_a)
                tree_a, size_a, new_idx, x_new = self._extend_toward(tree_a, size_a, x_rand)
                if new_idx is not None:
                    tree_b, size_b, cap_b = self._grow_tree(tree_b, size_b, cap_b)
                    tree_b, size_b, conn_idx = self._connect(tree_b, size_b, x_new)
                    if conn_idx is not None and np.linalg.norm(
                        tree_b[conn_idx, : self.dim] - x_new
                    ) <= self.goal_threshold:
                        best_path = self._extract_bidir_path(tree_a, new_idx, tree_b, conn_idx)
                        success = True
                        break
            else:
                tree_b, size_b, cap_b = self._grow_tree(tree_b, size_b, cap_b)
                tree_b, size_b, new_idx, x_new = self._extend_toward(tree_b, size_b, x_rand)
                if new_idx is not None:
                    tree_a, size_a, cap_a = self._grow_tree(tree_a, size_a, cap_a)
                    tree_a, size_a, conn_idx = self._connect(tree_a, size_a, x_new)
                    if conn_idx is not None and np.linalg.norm(
                        tree_a[conn_idx, : self.dim] - x_new
                    ) <= self.goal_threshold:
                        best_path = self._extract_bidir_path(tree_a, conn_idx, tree_b, new_idx)
                        success = True
                        break
            extend_a = not extend_a

        elapsed = time.perf_counter() - t0
        return self._build_result(best_path, success, elapsed, size_a + size_b)
