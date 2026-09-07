"""
Baseline-MoveTo: 当前按摩机器人系统的 3 点直线空移策略。
"""

import time
from typing import Dict, List

import numpy as np


class BaselineMoveTo:
    """工程基线：抬起-平移-下降 三点直线"""

    def __init__(self, env: Dict):
        self.env = env
        self.dim = env.get("dim", 3)

    def plan(self) -> Dict:
        """直接返回轨迹中的 baseline 路径（计时覆盖完整构造与指标计算）。"""
        t0 = time.perf_counter()
        baseline = self.env.get("baseline_path", [])
        if not baseline:
            start = self.env["start_point"]
            goal = self.env["goal_point"]
            mid = [(s + g) / 2.0 for s, g in zip(start, goal)]
            mid[2] = max(start[2], goal[2]) + 0.05
            baseline = [start.tolist(), mid, goal.tolist()]

        path = [np.array(p, dtype=float) for p in baseline]

        # 计算路径长度与效率（纳入公平计时）
        length = sum(
            np.linalg.norm(path[i + 1] - path[i])
            for i in range(len(path) - 1)
        )
        straight_dist = np.linalg.norm(
            self.env["goal_point"] - self.env["start_point"]
        )
        elapsed = time.perf_counter() - t0

        return {
            "success": True,
            "path": path,
            "path_length": length,
            "planning_time": elapsed,
            "tree_nodes": len(path),
            "path_efficiency": straight_dist / length if length > 0 else 0,
            "algorithm": "Baseline-MoveTo",
        }
