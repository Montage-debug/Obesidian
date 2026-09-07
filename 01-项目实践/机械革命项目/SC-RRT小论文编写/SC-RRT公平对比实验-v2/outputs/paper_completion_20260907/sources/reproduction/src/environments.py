"""可重现的 2D/3D 障碍环境生成与网格可行性验证。

地图生成只使用几何约束和网格连通性，不运行任何待比较规划器，
避免按某一算法的表现筛选地图。
"""

from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import numpy as np

from core import Environment, point_is_free, segment_is_free


FAMILIES = ("uniform_clutter", "central_cluster", "slab_clutter")


def _random_center(
    rng: np.random.Generator,
    bounds: np.ndarray,
    radius: float,
    family: str,
) -> np.ndarray:
    low = bounds[:, 0] + radius
    high = bounds[:, 1] - radius
    if family == "uniform_clutter":
        return rng.uniform(low, high)
    if family == "central_cluster" and rng.random() < 0.7:
        center = np.mean(bounds, axis=1)
        scale = 0.18 * (bounds[:, 1] - bounds[:, 0])
        return np.clip(rng.normal(center, scale), low, high)
    if family == "slab_clutter" and rng.random() < 0.65:
        point = rng.uniform(low, high)
        midpoint = float(np.mean(bounds[0]))
        point[0] = np.clip(rng.normal(midpoint, 0.09 * (high[0] - low[0])), low[0], high[0])
        return point
    return rng.uniform(low, high)


def _straight_line_blockers(
    rng: np.random.Generator,
    start: np.ndarray,
    goal: np.ndarray,
    radius_range: tuple[float, float],
    count: int,
) -> list[np.ndarray]:
    direction = goal - start
    direction /= np.linalg.norm(direction)
    blockers: list[np.ndarray] = []
    for t in np.linspace(0.25, 0.75, count):
        radius = float(rng.uniform(*radius_range))
        jitter = rng.normal(size=len(start))
        jitter -= np.dot(jitter, direction) * direction
        norm = float(np.linalg.norm(jitter))
        if norm > 1e-12:
            jitter *= rng.uniform(0.0, 0.35 * radius) / norm
        blockers.append(np.r_[start + t * (goal - start) + jitter, radius])
    return blockers


def grid_path_exists(env: Environment, resolution: float) -> bool:
    """用 8/26 邻域体素网格验证存在一条保守的无碰通道。"""
    axes = [
        np.linspace(lo, hi, int(np.ceil((hi - lo) / resolution)) + 1)
        for lo, hi in env.bounds
    ]
    shape = tuple(len(axis) for axis in axes)
    mesh = np.meshgrid(*axes, indexing="ij")
    points = np.stack([part.ravel() for part in mesh], axis=1)
    free = np.ones(len(points), dtype=bool)
    for begin in range(0, len(points), 4096):
        chunk = points[begin : begin + 4096]
        delta = chunk[:, None, :] - env.obstacles[None, :, :-1]
        squared = np.einsum("ijk,ijk->ij", delta, delta)
        free[begin : begin + len(chunk)] = np.all(
            squared > env.obstacles[None, :, -1] ** 2,
            axis=1,
        )
    free = free.reshape(shape)

    def closest_index(point: np.ndarray) -> tuple[int, ...]:
        return tuple(int(np.argmin(np.abs(axis - value))) for axis, value in zip(axes, point))

    source = closest_index(env.start)
    target = closest_index(env.goal)
    free[source] = True
    free[target] = True
    offsets = [
        offset
        for offset in np.ndindex(*(3,) * env.dim)
        if any(value != 1 for value in offset)
    ]
    offsets = [tuple(value - 1 for value in offset) for offset in offsets]
    queue = deque([source])
    visited = np.zeros(shape, dtype=bool)
    visited[source] = True
    while queue:
        current = queue.popleft()
        if current == target:
            return True
        current_point = np.array([axes[d][current[d]] for d in range(env.dim)])
        for offset in offsets:
            nxt = tuple(current[d] + offset[d] for d in range(env.dim))
            if any(nxt[d] < 0 or nxt[d] >= shape[d] for d in range(env.dim)):
                continue
            if visited[nxt] or not free[nxt]:
                continue
            next_point = np.array([axes[d][nxt[d]] for d in range(env.dim)])
            if not segment_is_free(current_point, next_point, env.obstacles):
                continue
            visited[nxt] = True
            queue.append(nxt)
    return False


def generate_environment(dim: int, index: int, split: str, config: dict) -> Environment:
    spec = config["dimensions"][str(dim)]
    seed_base = 271_828 if split == "calibration" else 314_159
    family = FAMILIES[index % len(FAMILIES)]
    bounds = np.asarray(spec["bounds_mm"], dtype=float)
    start = np.asarray(spec["start_mm"], dtype=float)
    goal = np.asarray(spec["goal_mm"], dtype=float)
    radius_range = tuple(float(x) for x in spec["radius_range_mm"])
    obstacle_count = int(spec["obstacle_count"])

    for retry in range(100):
        generation_seed = seed_base + dim * 100_000 + index * 1_000 + retry
        rng = np.random.default_rng(generation_seed)
        blocker_count = 3 if dim == 2 else 6
        obstacles = _straight_line_blockers(rng, start, goal, radius_range, blocker_count)
        trials = 0
        while len(obstacles) < obstacle_count and trials < obstacle_count * 500:
            trials += 1
            radius = float(rng.uniform(*radius_range))
            center = _random_center(rng, bounds, radius, family)
            clearance = radius + 1.5 * float(config["goal_threshold_mm"])
            if np.linalg.norm(center - start) <= clearance or np.linalg.norm(center - goal) <= clearance:
                continue
            obstacles.append(np.r_[center, radius])
        if len(obstacles) != obstacle_count:
            continue
        env = Environment(
            env_id=f"{split}-{dim}d-{index:02d}",
            dim=dim,
            bounds=bounds,
            start=start,
            goal=goal,
            obstacles=np.asarray(obstacles, dtype=float),
            family=family,
            generation_seed=generation_seed,
        )
        if point_is_free(start, env.obstacles) and point_is_free(goal, env.obstacles) and not segment_is_free(start, goal, env.obstacles):
            if grid_path_exists(env, float(spec["feasibility_grid_mm"])):
                return env
    raise RuntimeError(f"无法为 {split} {dim}D 索引 {index} 生成合格地图")


def environment_to_dict(env: Environment) -> dict:
    return {
        "environment_id": env.env_id,
        "dimension": env.dim,
        "family": env.family,
        "generation_seed": env.generation_seed,
        "bounds_mm": env.bounds.tolist(),
        "start_mm": env.start.tolist(),
        "goal_mm": env.goal.tolist(),
        "obstacles": env.obstacles.tolist(),
    }


def environment_from_dict(data: dict) -> Environment:
    return Environment(
        env_id=data["environment_id"],
        dim=int(data["dimension"]),
        family=data["family"],
        generation_seed=int(data["generation_seed"]),
        bounds=np.asarray(data["bounds_mm"], dtype=float),
        start=np.asarray(data["start_mm"], dtype=float),
        goal=np.asarray(data["goal_mm"], dtype=float),
        obstacles=np.asarray(data["obstacles"], dtype=float),
    )


def ensure_environment_files(root: Path, config: dict) -> dict[tuple[str, int], list[Environment]]:
    output = root / "data" / "environments"
    output.mkdir(parents=True, exist_ok=True)
    result: dict[tuple[str, int], list[Environment]] = {}
    for split, count_key in (("calibration", "calibration_maps_per_dimension"), ("evaluation", "evaluation_maps_per_dimension")):
        count = int(config[count_key])
        for dim in (2, 3):
            path = output / f"{split}_{dim}d.json"
            if path.exists():
                payload = json.loads(path.read_text(encoding="utf-8"))
                environments = [environment_from_dict(item) for item in payload["environments"]]
            else:
                environments = [generate_environment(dim, index, split, config) for index in range(count)]
                payload = {
                    "schema_version": "1.0",
                    "split": split,
                    "dimension": dim,
                    "selection_rule": "geometry_and_grid_connectivity_only",
                    "environments": [environment_to_dict(env) for env in environments],
                }
                path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
            result[(split, dim)] = environments
    return result
