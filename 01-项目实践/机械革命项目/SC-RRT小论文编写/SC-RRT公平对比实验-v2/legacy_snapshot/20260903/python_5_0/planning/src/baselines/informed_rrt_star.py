"""
Informed RRT* 基线：单树 informed 采样 + rewire（独立实现，不依赖 SCRRTAdaptive）。
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional

import numpy as np

from sc_rrt.geometry import is_collision_free, sample_in_ellipsoid, sample_point, steer_point

from .rrt_planner_base import RRTPlannerBase


class InformedRRTStar(RRTPlannerBase):
    """标准 Informed RRT*（gamma=1, p=1 的全 informed 椭球 + rewire）。"""

    REWIRE_FACTOR = 2.0
    GOAL_SAMPLE_RATE = 0.05

    def __init__(
        self,
        env: Dict,
        max_iterations: int = 8000,
        safety_margin: float = 0.0,
    ):
        super().__init__(env, max_iterations, "Informed-RRT*", safety_margin)

    def _near_radius(self, tree_size: int) -> float:
        vol = np.prod([self.bounds[2 * i + 1] - self.bounds[2 * i] for i in range(self.dim)])
        unit_ball = math.pi ** (self.dim / 2.0) / math.gamma(self.dim / 2.0 + 1.0)
        return self.REWIRE_FACTOR * (vol / max(tree_size, 1) / unit_ball) ** (1.0 / self.dim)

    def _propagate_costs(self, tree: np.ndarray, size: int, root_idx: int) -> None:
        """rewire 后沿子树刷新累积代价。"""
        children: Dict[int, List[int]] = {i: [] for i in range(size)}
        for idx in range(1, size):
            parent = int(tree[idx, self.dim])
            if parent >= 0:
                children[parent].append(idx)

        stack = [root_idx]
        while stack:
            parent = stack.pop()
            for child in children.get(parent, []):
                p_pt = tree[parent, : self.dim]
                c_pt = tree[child, : self.dim]
                tree[child, self.dim + 1] = (
                    tree[parent, self.dim + 1] + float(np.linalg.norm(c_pt - p_pt))
                )
                stack.append(child)

    def _sample(self, c_best: float) -> np.ndarray:
        if np.random.rand() < self.GOAL_SAMPLE_RATE:
            return self.goal.copy()
        pt = sample_in_ellipsoid(
            self.start, self.goal, c_best, self.dim, bounds=self.bounds, max_attempts=12,
        )
        if pt is not None:
            return pt
        return sample_point(self.bounds, self.dim)

    def plan(self) -> Dict:
        t0 = time.perf_counter()
        cap = max(512, self.max_iterations // 4)
        tree = self._new_tree(cap)
        tree[0, : self.dim] = self.start
        tree[0, self.dim] = -1
        tree[0, self.dim + 1] = 0.0
        size = 1
        cap_cur = cap

        success = False
        best_path: Optional[List[np.ndarray]] = None
        c_best = float("inf")
        best_goal_idx: Optional[int] = None
        first_found_iter = None
        max_extra = max(200, self.max_iterations // 20)

        for it in range(1, self.max_iterations + 1):
            x_rand = self._sample(c_best)
            nearest_idx = int(np.argmin(
                np.linalg.norm(tree[:size, : self.dim] - x_rand, axis=1)
            ))
            x_near = tree[nearest_idx, : self.dim]
            x_new = steer_point(x_near, x_rand, self.step_size)
            if not self._segment_collision_free(x_near, x_new):
                continue

            r = self._near_radius(size)
            dists = np.linalg.norm(tree[:size, : self.dim] - x_new, axis=1)
            near_idx = [int(i) for i in np.where(dists <= r)[0]]
            if not near_idx:
                near_idx = [nearest_idx]

            # 选最优父节点
            best_parent = nearest_idx
            best_cost = float("inf")
            for idx in near_idx:
                x_p = tree[idx, : self.dim]
                if not self._segment_collision_free(x_p, x_new):
                    continue
                cost = tree[idx, self.dim + 1] + float(np.linalg.norm(x_new - x_p))
                if cost < best_cost:
                    best_cost = cost
                    best_parent = idx
            if best_cost == float("inf"):
                best_cost = (
                    tree[nearest_idx, self.dim + 1]
                    + float(np.linalg.norm(x_new - x_near))
                )
                best_parent = nearest_idx

            tree, size, cap_cur = self._grow_tree(tree, size, cap_cur)
            tree, size = self._add_node(tree, size, best_parent, x_new, best_cost)
            new_idx = size - 1

            # rewire 邻域节点
            for idx in near_idx:
                if idx == new_idx:
                    continue
                x_nb = tree[idx, : self.dim]
                if not self._segment_collision_free(x_new, x_nb):
                    continue
                alt = tree[new_idx, self.dim + 1] + float(np.linalg.norm(x_nb - x_new))
                if alt + 1e-6 < tree[idx, self.dim + 1]:
                    tree[idx, self.dim] = new_idx
                    tree[idx, self.dim + 1] = alt
                    self._propagate_costs(tree, size, idx)

            # 显式尝试连接 goal（任意新节点）
            if self._segment_collision_free(x_new, self.goal):
                goal_cost = tree[new_idx, self.dim + 1] + float(np.linalg.norm(self.goal - x_new))
                if goal_cost < c_best:
                    c_best = goal_cost
                    best_goal_idx = new_idx
                    path = self._backtrack_path(tree, new_idx)
                    path.append(self.goal.copy())
                    best_path = path
                    success = True
                    if first_found_iter is None:
                        first_found_iter = it
                    elif it - first_found_iter >= max_extra:
                        break

            if float(np.linalg.norm(x_new - self.goal)) <= self.goal_threshold:
                goal_cost = tree[new_idx, self.dim + 1] + float(np.linalg.norm(self.goal - x_new))
                if goal_cost < c_best:
                    c_best = goal_cost
                    best_goal_idx = new_idx
                    path = self._backtrack_path(tree, new_idx)
                    path.append(self.goal.copy())
                    best_path = path
                    success = True
                    if first_found_iter is None:
                        first_found_iter = it
                    elif it - first_found_iter >= max_extra:
                        break

        elapsed = time.perf_counter() - t0
        return self._build_result(best_path, success, elapsed, size)
