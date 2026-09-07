"""四算法共用的部署路径后处理：捷径、简化、重采样、间隙推离。"""

from __future__ import annotations

from typing import Callable, List, Optional, Sequence

import numpy as np

from .geometry import is_collision_free


def _segment_collision_free(
    p1: np.ndarray,
    p2: np.ndarray,
    obstacles: np.ndarray,
    dim: int,
    segment_free: Optional[Callable[[np.ndarray, np.ndarray], bool]] = None,
) -> bool:
    if not is_collision_free(p1, p2, obstacles, dim):
        return False
    if segment_free is not None and not segment_free(p1, p2):
        return False
    return True


def _as_array(path: Sequence[Sequence[float]]) -> np.ndarray:
    if path is None or len(path) == 0:
        return np.zeros((0, 3))
    return np.asarray(path, dtype=float)


def _inflate_obstacles(obstacles: np.ndarray, margin: float) -> np.ndarray:
    if margin <= 0 or obstacles is None or len(obstacles) == 0:
        return obstacles
    obs = obstacles.copy()
    obs[:, 3] = obs[:, 3] + margin
    return obs


def optimize_clearance_path(
    path: Sequence[Sequence[float]],
    obstacles: np.ndarray,
    dim: int = 3,
    target_clearance: float = 0.016,
    max_passes: int = 15,
) -> np.ndarray:
    """沿路径内点推离最近球体，提升最小间隙。"""
    pts = _as_array(path)
    if len(pts) < 3 or obstacles is None or len(obstacles) == 0:
        return pts

    optimized = pts.copy()
    for _ in range(max_passes):
        improved = False
        for i in range(1, len(optimized) - 1):
            pt = optimized[i]
            dists = np.linalg.norm(obstacles[:, :3] - pt, axis=1) - obstacles[:, 3]
            nearest = int(np.argmin(dists))
            gap = float(dists[nearest])
            if gap >= target_clearance:
                continue
            center = obstacles[nearest, :3]
            direction = pt - center
            norm = float(np.linalg.norm(direction))
            if norm < 1e-8:
                direction = np.array([0.0, 0.0, 1.0])
                norm = 1.0
            push = direction / norm * (target_clearance - gap + 0.001)
            candidate = pt + push
            if _segment_collision_free(optimized[i - 1], candidate, obstacles, dim) and _segment_collision_free(
                candidate, optimized[i + 1], obstacles, dim
            ):
                optimized[i] = candidate
                improved = True
        if not improved:
            break
    return optimized


def shortcut_path(
    path: Sequence[Sequence[float]],
    obstacles: np.ndarray,
    dim: int = 3,
    max_passes: int = 4,
    clearance_margin: float = 0.0,
    segment_free: Optional[Callable[[np.ndarray, np.ndarray], bool]] = None,
) -> np.ndarray:
    """视线捷径：跳过中间拐点（障碍含安全裕度）。"""
    pts = _as_array(path)
    if len(pts) < 3:
        return pts

    obs = _inflate_obstacles(obstacles, clearance_margin)

    for _ in range(max_passes):
        i = 0
        out = [pts[0]]
        while i < len(pts) - 1:
            j = len(pts) - 1
            while j > i + 1:
                if _segment_collision_free(pts[i], pts[j], obs, dim, segment_free):
                    break
                j -= 1
            out.append(pts[j])
            i = j
        pts = np.asarray(out, dtype=float)
    return pts


def _perp_dist(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    if np.linalg.norm(ab) < 1e-12:
        return float(np.linalg.norm(point - a))
    t = np.clip(np.dot(point - a, ab) / np.dot(ab, ab), 0.0, 1.0)
    proj = a + t * ab
    return float(np.linalg.norm(point - proj))


def simplify_path(path: Sequence[Sequence[float]], epsilon: float = 0.01) -> np.ndarray:
    """Douglas-Peucker 路径简化。"""
    pts = _as_array(path)
    if len(pts) < 3:
        return pts

    def rdp(indices: List[int]) -> List[int]:
        if len(indices) < 3:
            return indices
        start, end = indices[0], indices[-1]
        max_d, idx = 0.0, -1
        for k in indices[1:-1]:
            d = _perp_dist(pts[k], pts[start], pts[end])
            if d > max_d:
                max_d, idx = d, k
        if max_d > epsilon:
            left = rdp(indices[: indices.index(idx) + 1])
            right = rdp(indices[indices.index(idx) :])
            return left[:-1] + right
        return [start, end]

    keep = rdp(list(range(len(pts))))
    return pts[keep]


def resample_path(path: Sequence[Sequence[float]], step: float = 0.015) -> np.ndarray:
    """沿折线均匀重采样。"""
    pts = _as_array(path)
    if len(pts) < 2:
        return pts

    seg_lens = [np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)]
    total = sum(seg_lens)
    if total < step:
        return pts

    samples = [pts[0]]
    seg_i = 0
    seg_start = 0.0
    target = step
    while target < total - 1e-9:
        while seg_i < len(seg_lens) and seg_start + seg_lens[seg_i] < target:
            seg_start += seg_lens[seg_i]
            seg_i += 1
        if seg_i >= len(seg_lens):
            break
        local = (target - seg_start) / max(seg_lens[seg_i], 1e-12)
        p = pts[seg_i] + local * (pts[seg_i + 1] - pts[seg_i])
        samples.append(p)
        target += step
    samples.append(pts[-1])
    return np.asarray(samples, dtype=float)


def postprocess_path(
    path: Sequence[Sequence[float]],
    obstacles: np.ndarray,
    dim: int = 3,
    epsilon: float = 0.012,
    step: float = 0.025,
    clearance_margin: float = 0.010,
    target_clearance: float = 0.016,
    segment_free: Optional[Callable[[np.ndarray, np.ndarray], bool]] = None,
) -> np.ndarray:
    """公共部署后处理：捷径 → 简化 → 重采样 → 间隙推离。"""
    pts = _as_array(path)
    if len(pts) < 2:
        return pts
    pts = shortcut_path(
        pts, obstacles, dim, clearance_margin=clearance_margin, segment_free=segment_free,
    )
    pts = simplify_path(pts, epsilon)
    pts = resample_path(pts, step)
    pts = optimize_clearance_path(pts, obstacles, dim, target_clearance=target_clearance)
    return pts


# 兼容旧调用；新实验必须对全部规划器使用 postprocess_path。
postprocess_sc_rrt_path = postprocess_path
