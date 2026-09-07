#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 obstacles_app_v1 按 obstacle_set 加载场景球簇。"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import numpy as np
import yaml

from exp_paths import EXP_ROOT


def load_scene_spheres(
    obstacle_set: str,
    obstacles_config: Path | None = None,
) -> List[Tuple[np.ndarray, float]]:
    """返回 (center, radius) 列表，与规划层 MassageRobotEnv 一致。"""
    cfg_path = obstacles_config or (EXP_ROOT / "config" / "obstacles_app_v1.yaml")
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    stress = cfg.get("stress_cases", {})
    if obstacle_set not in stress:
        raise KeyError(f"Unknown obstacle_set: {obstacle_set}")
    selected = stress[obstacle_set]
    out: List[Tuple[np.ndarray, float]] = []
    for s in selected.get("spheres", []):
        out.append((np.asarray(s["center"], dtype=float), float(s["radius"])))
    return out


def min_tcp_clearance_on_path(
    positions: List[List[float]],
    spheres: List[Tuple[np.ndarray, float]],
    samples_per_seg: int = 24,
) -> float:
    """沿折线采样，TCP 到球簇最小间隙 (m)。"""
    if not positions or not spheres:
        return float("inf")
    pts = [np.asarray(p, dtype=float) for p in positions]
    min_clear = float("inf")
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        for k in range(samples_per_seg + 1):
            t = k / samples_per_seg
            p = a + t * (b - a)
            for center, radius in spheres:
                d = float(np.linalg.norm(p - center) - radius)
                min_clear = min(min_clear, d)
    return min_clear
