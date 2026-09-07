"""统一几何、树结构、预算计数和指标记录。

公平预算定义：一次 atomic extension attempt 包含一次目标选择后的
nearest + steer + edge collision check，并且最多插入一个新节点。Connect
策略内部的每一步推进都单独消耗一次预算。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class Environment:
    env_id: str
    dim: int
    bounds: np.ndarray
    start: np.ndarray
    goal: np.ndarray
    obstacles: np.ndarray
    family: str
    generation_seed: int

    @property
    def straight_distance(self) -> float:
        return float(np.linalg.norm(self.goal - self.start))


def steer(source: np.ndarray, target: np.ndarray, step_size: float) -> np.ndarray:
    delta = target - source
    distance = float(np.linalg.norm(delta))
    if distance <= step_size:
        return target.copy()
    return source + delta * (step_size / max(distance, 1e-12))


def point_is_free(point: np.ndarray, obstacles: np.ndarray, margin: float = 0.0) -> bool:
    if len(obstacles) == 0:
        return True
    radii = obstacles[:, -1] + margin
    return bool(np.all(np.linalg.norm(obstacles[:, :-1] - point, axis=1) > radii))


def segment_is_free(
    a: np.ndarray,
    b: np.ndarray,
    obstacles: np.ndarray,
    margin: float = 0.0,
) -> bool:
    """精确线段-圆/球体碰撞检测，2D 和 3D 使用同一实现。"""
    if len(obstacles) == 0:
        return True
    ab = b - a
    denom = float(np.dot(ab, ab))
    centers = obstacles[:, :-1]
    if denom <= 1e-18:
        closest = np.repeat(a[None, :], len(obstacles), axis=0)
    else:
        t = np.clip(((centers - a) @ ab) / denom, 0.0, 1.0)
        closest = a + t[:, None] * ab
    distances = np.linalg.norm(centers - closest, axis=1)
    return bool(np.all(distances > obstacles[:, -1] + margin))


def path_length(path: Iterable[np.ndarray]) -> float:
    pts = np.asarray(list(path), dtype=float)
    if len(pts) < 2:
        return float("inf")
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())


def path_is_valid(path: np.ndarray, env: Environment, atol: float = 1e-7) -> bool:
    if path is None or len(path) < 2:
        return False
    if not np.allclose(path[0], env.start, atol=atol):
        return False
    if not np.allclose(path[-1], env.goal, atol=atol):
        return False
    return all(
        segment_is_free(path[i], path[i + 1], env.obstacles)
        for i in range(len(path) - 1)
    )


def sample_uniform(rng: np.random.Generator, bounds: np.ndarray) -> np.ndarray:
    return rng.uniform(bounds[:, 0], bounds[:, 1])


def sample_unit_ball(rng: np.random.Generator, dim: int) -> np.ndarray:
    direction = rng.normal(size=dim)
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-12:
        direction[0] = 1.0
        norm = 1.0
    return direction / norm * (rng.random() ** (1.0 / dim))


def rotation_from_x_axis(direction: np.ndarray) -> np.ndarray:
    """构造把第一坐标轴映射到 direction 的正交矩阵。"""
    dim = len(direction)
    unit = direction / max(float(np.linalg.norm(direction)), 1e-12)
    basis = np.eye(dim)
    basis[:, 0] = unit
    q, _ = np.linalg.qr(basis)
    if float(np.dot(q[:, 0], unit)) < 0:
        q[:, 0] *= -1.0
    return q


def sample_in_ellipsoid(
    rng: np.random.Generator,
    focus_a: np.ndarray,
    focus_b: np.ndarray,
    major_axis: float,
    bounds: np.ndarray,
    max_attempts: int = 32,
) -> Optional[np.ndarray]:
    focal_distance = float(np.linalg.norm(focus_b - focus_a))
    if not np.isfinite(major_axis) or major_axis <= focal_distance + 1e-9:
        return None
    center = 0.5 * (focus_a + focus_b)
    semi_major = major_axis / 2.0
    semi_minor = np.sqrt(max(major_axis * major_axis - focal_distance * focal_distance, 0.0)) / 2.0
    scales = np.full(len(focus_a), semi_minor, dtype=float)
    scales[0] = semi_major
    rotation = rotation_from_x_axis(focus_b - focus_a)
    for _ in range(max_attempts):
        sample = center + rotation @ (scales * sample_unit_ball(rng, len(focus_a)))
        if np.all(sample >= bounds[:, 0]) and np.all(sample <= bounds[:, 1]):
            return sample
    return None


class Tree:
    """预分配的紧凑 RRT 树；所有算法使用相同存储与近邻实现。"""

    def __init__(self, root: np.ndarray, capacity: int):
        self.dim = len(root)
        self.points = np.empty((capacity, self.dim), dtype=float)
        self.parents = np.full(capacity, -1, dtype=np.int32)
        self.costs = np.full(capacity, np.inf, dtype=float)
        self.children_count = np.zeros(capacity, dtype=np.int32)
        self.points[0] = root
        self.costs[0] = 0.0
        self.size = 1

    def add(self, point: np.ndarray, parent: int, cost: float) -> int:
        if self.size >= len(self.points):
            raise RuntimeError("树容量超出公平预算上限")
        idx = self.size
        self.points[idx] = point
        self.parents[idx] = parent
        self.costs[idx] = cost
        self.children_count[parent] += 1
        self.size += 1
        return idx

    def nearest(self, point: np.ndarray) -> int:
        return int(np.argmin(np.einsum("ij,ij->i", self.points[: self.size] - point, self.points[: self.size] - point)))

    def near(self, point: np.ndarray, radius: float) -> np.ndarray:
        delta = self.points[: self.size] - point
        return np.flatnonzero(np.einsum("ij,ij->i", delta, delta) <= radius * radius)

    def backtrack(self, idx: int) -> list[np.ndarray]:
        result: list[np.ndarray] = []
        seen: set[int] = set()
        while idx >= 0:
            if idx in seen:
                raise RuntimeError("检测到树父指针环")
            seen.add(idx)
            result.append(self.points[idx].copy())
            idx = int(self.parents[idx])
        result.reverse()
        return result


@dataclass
class Budget:
    maximum: int
    checkpoints: tuple[int, ...]
    attempts: int = 0
    added_nodes: int = 0
    collision_checks: int = 0
    collision_rejections: int = 0
    rewire_attempts: int = 0
    rewire_successes: int = 0
    trace: dict[int, float] = field(default_factory=dict)

    @property
    def exhausted(self) -> bool:
        return self.attempts >= self.maximum

    def consume(self) -> bool:
        if self.exhausted:
            return False
        self.attempts += 1
        return True

    def record(self, best_cost: float) -> None:
        if self.attempts in self.checkpoints:
            self.trace[self.attempts] = float(best_cost)

    def finalize_trace(self, best_cost: float) -> None:
        last = float("inf")
        for checkpoint in self.checkpoints:
            if checkpoint in self.trace:
                last = min(last, self.trace[checkpoint])
            elif checkpoint <= self.attempts:
                self.trace[checkpoint] = min(last, float(best_cost))
        if self.maximum not in self.trace and self.attempts >= self.maximum:
            self.trace[self.maximum] = float(best_cost)


@dataclass
class PlannerResult:
    algorithm: str
    success: bool
    path: Optional[np.ndarray]
    best_cost: float
    runtime_s: float
    first_solution_time_s: float
    first_solution_attempt: float
    first_solution_nodes: float
    first_solution_cost: float
    total_nodes: int
    budget: Budget
    extra: dict = field(default_factory=dict)

    def as_record(self, env: Environment, seed: int) -> dict:
        efficiency = env.straight_distance / self.best_cost if np.isfinite(self.best_cost) else np.nan
        first_eff = env.straight_distance / self.first_solution_cost if np.isfinite(self.first_solution_cost) else np.nan
        return {
            "environment_id": env.env_id,
            "family": env.family,
            "dimension": env.dim,
            "seed": seed,
            "algorithm": self.algorithm,
            "success": int(self.success),
            "budget_limit": self.budget.maximum,
            "expansion_attempts": self.budget.attempts,
            "added_nodes": self.budget.added_nodes,
            "total_nodes": self.total_nodes,
            "collision_checks": self.budget.collision_checks,
            "collision_rejections": self.budget.collision_rejections,
            "rewire_attempts": self.budget.rewire_attempts,
            "rewire_successes": self.budget.rewire_successes,
            "first_solution_attempt": self.first_solution_attempt,
            "first_solution_time_s": self.first_solution_time_s,
            "first_solution_nodes": self.first_solution_nodes,
            "first_solution_cost_mm": self.first_solution_cost,
            "first_solution_efficiency": first_eff,
            "fixed_budget_time_s": self.runtime_s,
            "fixed_budget_cost_mm": self.best_cost,
            "fixed_budget_efficiency": efficiency,
            "path_valid": int(path_is_valid(self.path, env)) if self.success else 0,
            **self.extra,
        }


class PlannerBase:
    def __init__(
        self,
        algorithm: str,
        env: Environment,
        seed: int,
        maximum: int,
        checkpoints: Iterable[int],
        step_size: float,
        goal_threshold: float,
        collision_margin: float = 0.0,
    ):
        self.algorithm = algorithm
        self.env = env
        self.rng = np.random.default_rng(seed)
        self.budget = Budget(maximum, tuple(int(x) for x in checkpoints))
        self.step_size = float(step_size)
        self.goal_threshold = float(goal_threshold)
        self.collision_margin = float(collision_margin)
        self.best_cost = float("inf")
        self.best_path: Optional[np.ndarray] = None
        self.first_solution_time_s = float("nan")
        self.first_solution_attempt = float("nan")
        self.first_solution_nodes = float("nan")
        self.first_solution_cost = float("nan")
        self._started = perf_counter()

    def elapsed(self) -> float:
        return perf_counter() - self._started

    def edge_free(self, a: np.ndarray, b: np.ndarray, rewire: bool = False) -> bool:
        self.budget.collision_checks += 1
        if rewire:
            self.budget.rewire_attempts += 1
        ok = segment_is_free(a, b, self.env.obstacles, self.collision_margin)
        if rewire and ok:
            pass
        return ok

    def attempt_extend(self, tree: Tree, target: np.ndarray) -> tuple[Optional[int], Optional[np.ndarray]]:
        if not self.budget.consume():
            return None, None
        parent = tree.nearest(target)
        new_point = steer(tree.points[parent], target, self.step_size)
        if float(np.linalg.norm(new_point - tree.points[parent])) <= 1e-10:
            self.budget.record(self.best_cost)
            return None, new_point
        if not self.edge_free(tree.points[parent], new_point):
            self.budget.collision_rejections += 1
            self.budget.record(self.best_cost)
            return None, new_point
        idx = tree.add(new_point, parent, tree.costs[parent] + float(np.linalg.norm(new_point - tree.points[parent])))
        self.budget.added_nodes += 1
        self.budget.record(self.best_cost)
        return idx, new_point

    def consider_solution(self, path: list[np.ndarray], total_nodes: int, extra: Optional[dict] = None) -> bool:
        candidate = np.asarray(path, dtype=float)
        cost = path_length(candidate)
        improved = cost + 1e-9 < self.best_cost
        if not improved and np.isfinite(self.first_solution_attempt):
            return False
        if not path_is_valid(candidate, self.env):
            return False
        if not np.isfinite(self.first_solution_attempt):
            self.first_solution_attempt = float(self.budget.attempts)
            self.first_solution_time_s = self.elapsed()
            self.first_solution_nodes = float(total_nodes)
            self.first_solution_cost = cost
        if improved:
            self.best_cost = cost
            self.best_path = candidate
            if extra:
                self.on_solution_improved(extra)
            self.budget.record(self.best_cost)
        return improved

    def on_solution_improved(self, extra: dict) -> None:
        del extra

    def result(self, total_nodes: int, extra: Optional[dict] = None) -> PlannerResult:
        self.budget.finalize_trace(self.best_cost)
        return PlannerResult(
            algorithm=self.algorithm,
            success=self.best_path is not None,
            path=self.best_path,
            best_cost=self.best_cost,
            runtime_s=self.elapsed(),
            first_solution_time_s=self.first_solution_time_s,
            first_solution_attempt=self.first_solution_attempt,
            first_solution_nodes=self.first_solution_nodes,
            first_solution_cost=self.first_solution_cost,
            total_nodes=total_nodes,
            budget=self.budget,
            extra=extra or {},
        )
