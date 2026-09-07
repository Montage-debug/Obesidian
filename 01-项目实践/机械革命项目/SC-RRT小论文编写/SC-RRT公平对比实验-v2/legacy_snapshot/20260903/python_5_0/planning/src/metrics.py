"""
实验指标计算模块
"""

from typing import Dict, List, Optional

import numpy as np


def compute_path_length(path: List[np.ndarray]) -> float:
    """计算路径总长度"""
    if len(path) < 2:
        return 0.0
    return sum(
        np.linalg.norm(np.asarray(path[i + 1]) - np.asarray(path[i]))
        for i in range(len(path) - 1)
    )


def compute_path_efficiency(path: List[np.ndarray], start: np.ndarray, goal: np.ndarray) -> float:
    """路径效率 = 直线距离 / 实际路径长度"""
    length = compute_path_length(path)
    straight = np.linalg.norm(np.asarray(goal) - np.asarray(start))
    return straight / length if length > 0 else 0.0


def aggregate_results(records: List[Dict]) -> Dict:
    """聚合多次运行结果为 Mean ± Std（路径类指标仅统计成功 run）。"""
    import pandas as pd

    df = pd.DataFrame(records)
    summary = {}
    for algo in df["algorithm"].unique():
        sub = df[df["algorithm"] == algo]
        ok = sub[sub["success"] == True]  # noqa: E712
        summary[algo] = {
            "success_rate": sub["success"].mean() * 100,
            "path_length_mean": ok["path_length"].mean(),
            "path_length_std": ok["path_length"].std(),
            "planning_time_mean": sub["planning_time"].mean(),
            "planning_time_std": sub["planning_time"].std(),
            "clearance_mean": ok["clearance"].mean(),
            "clearance_std": ok["clearance"].std(),
            "path_efficiency_mean": ok["path_efficiency"].mean(),
            "path_efficiency_std": ok["path_efficiency"].std(),
            "smoothness_mean": ok["smoothness"].mean(),
            "smoothness_std": ok["smoothness"].std(),
            "tree_nodes_mean": sub["tree_nodes"].mean(),
        }
    return summary
