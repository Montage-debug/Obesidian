#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""关节轨迹 jerk / 速度指标（与 trajectory_executor 口径一致）。"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


def joint_jerk_profiles(
    points: List[List[float]],
    delta_ts: List[float],
    num_joints: int | None = None,
) -> Tuple[np.ndarray, List[np.ndarray]]:
    """由关节轨迹与时间参数化计算各关节 jerk 序列。"""
    if len(points) < 4 or not delta_ts:
        return np.array([]), []
    n_j = num_joints or len(points[0])
    times = [0.0]
    for dt in delta_ts:
        times.append(times[-1] + dt)
    times_arr = np.array(times, dtype=float)
    profiles = []
    for j in range(n_j):
        pos = np.array([p[j] for p in points], dtype=float)
        vel = np.gradient(pos, times_arr)
        acc = np.gradient(vel, times_arr)
        jerk = np.gradient(acc, times_arr)
        profiles.append(jerk)
    return times_arr, profiles


def compute_jerk_rms_from_plan(
    points: List[List[float]],
    delta_ts: List[float],
) -> float:
    """各关节 jerk RMS 再对关节取均值。"""
    _times, profiles = joint_jerk_profiles(points, delta_ts)
    if not profiles:
        return 0.0
    jerks = [float(np.sqrt(np.mean(j ** 2))) for j in profiles]
    return float(np.mean(jerks))


def compute_max_jerk_from_plan(
    points: List[List[float]],
    delta_ts: List[float],
) -> float:
    """跨关节与时间的最大绝对 jerk。"""
    _times, profiles = joint_jerk_profiles(points, delta_ts)
    if not profiles:
        return 0.0
    return float(max(np.max(np.abs(j)) for j in profiles))


def compute_max_joint_velocity(
    points: List[List[float]],
    delta_ts: List[float],
) -> float:
    """相邻路点间最大关节角速度 (rad/s)。"""
    if len(points) < 2 or not delta_ts:
        return 0.0
    max_vel = 0.0
    for i in range(len(points) - 1):
        dt = delta_ts[i] if i < len(delta_ts) else delta_ts[-1]
        if dt <= 0:
            continue
        q0 = np.asarray(points[i], dtype=float)
        q1 = np.asarray(points[i + 1], dtype=float)
        max_vel = max(max_vel, float(np.max(np.abs(q1 - q0) / dt)))
    return max_vel


def count_joint_limit_violations(
    points: Sequence[Sequence[float]],
    joint_limits: dict[str, Tuple[float, float]],
    joint_names: List[str],
) -> int:
    """统计路点上超出限位的关节-点次数。"""
    violations = 0
    for pt in points:
        for j, name in enumerate(joint_names):
            lo, hi = joint_limits[name]
            v = float(pt[j])
            if v < lo - 1e-6 or v > hi + 1e-6:
                violations += 1
    return violations
