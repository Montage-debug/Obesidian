"""六种主算法与两种 SC-RRT 消融变体的统一预算实现。"""

from __future__ import annotations

import math
from typing import Callable, Optional

import numpy as np

from core import (
    Environment,
    PlannerBase,
    PlannerResult,
    Tree,
    path_length,
    sample_in_ellipsoid,
    sample_uniform,
    steer,
)


def _join_bidirectional(
    start_tree: Tree,
    start_idx: int,
    goal_tree: Tree,
    goal_idx: int,
) -> list[np.ndarray]:
    left = start_tree.backtrack(start_idx)
    right = goal_tree.backtrack(goal_idx)
    right.reverse()
    if left and right and np.allclose(left[-1], right[0], atol=1e-8):
        right = right[1:]
    return left + right


class RRTPlanner(PlannerBase):
    def __init__(self, *args, goal_bias: float = 0.05, **kwargs):
        super().__init__("RRT", *args, **kwargs)
        self.goal_bias = float(goal_bias)

    def _try_goal(self, tree: Tree, idx: int) -> None:
        point = tree.points[idx]
        distance = float(np.linalg.norm(point - self.env.goal))
        if distance > self.goal_threshold:
            return
        if not self.edge_free(point, self.env.goal):
            return
        path = tree.backtrack(idx)
        if distance > 1e-9:
            path.append(self.env.goal.copy())
        self.consider_solution(path, tree.size)

    def plan(self) -> PlannerResult:
        tree = Tree(self.env.start, self.budget.maximum + 2)
        while not self.budget.exhausted:
            target = self.env.goal if self.rng.random() < self.goal_bias else sample_uniform(self.rng, self.env.bounds)
            idx, _ = self.attempt_extend(tree, target)
            if idx is not None:
                self._try_goal(tree, idx)
        return self.result(tree.size)


class RRTStarPlanner(PlannerBase):
    def __init__(
        self,
        algorithm: str,
        *args,
        goal_bias: float = 0.05,
        max_rewire_radius_steps: float = 6.0,
        **kwargs,
    ):
        super().__init__(algorithm, *args, **kwargs)
        self.goal_bias = float(goal_bias)
        self.max_rewire_radius = max_rewire_radius_steps * self.step_size
        self.goal_candidates: set[int] = set()

    def _sample(self) -> np.ndarray:
        if self.rng.random() < self.goal_bias:
            return self.env.goal.copy()
        return sample_uniform(self.rng, self.env.bounds)

    def _near_radius(self, n: int) -> float:
        volume = float(np.prod(self.env.bounds[:, 1] - self.env.bounds[:, 0]))
        unit_ball = math.pi ** (self.env.dim / 2.0) / math.gamma(self.env.dim / 2.0 + 1.0)
        gamma_star = 2.0 * ((1.0 + 1.0 / self.env.dim) * volume / unit_ball) ** (1.0 / self.env.dim)
        radius = gamma_star * (math.log(max(n, 2)) / max(n, 2)) ** (1.0 / self.env.dim)
        return float(min(self.max_rewire_radius, max(self.step_size * 1.01, radius)))

    def _propagate_costs(self, tree: Tree, root: int) -> None:
        queue = [root]
        while queue:
            parent = queue.pop()
            children = np.flatnonzero(tree.parents[: tree.size] == parent)
            for child in children:
                tree.costs[child] = tree.costs[parent] + float(
                    np.linalg.norm(tree.points[child] - tree.points[parent])
                )
                queue.append(int(child))

    def _refresh_goal(self, tree: Tree) -> None:
        if not self.goal_candidates:
            return
        idx = min(
            self.goal_candidates,
            key=lambda i: tree.costs[i] + float(np.linalg.norm(tree.points[i] - self.env.goal)),
        )
        path = tree.backtrack(idx)
        if not np.allclose(path[-1], self.env.goal, atol=1e-9):
            path.append(self.env.goal.copy())
        self.consider_solution(path, tree.size)

    def _iteration(self, tree: Tree) -> None:
        if not self.budget.consume():
            return
        target = self._sample()
        nearest = tree.nearest(target)
        new_point = steer(tree.points[nearest], target, self.step_size)
        if float(np.linalg.norm(new_point - tree.points[nearest])) <= 1e-10:
            self.budget.record(self.best_cost)
            return
        if not self.edge_free(tree.points[nearest], new_point):
            self.budget.collision_rejections += 1
            self.budget.record(self.best_cost)
            return

        near = tree.near(new_point, self._near_radius(tree.size))
        best_parent = nearest
        best_cost = tree.costs[nearest] + float(np.linalg.norm(new_point - tree.points[nearest]))
        for candidate in near:
            candidate = int(candidate)
            if candidate == nearest:
                continue
            proposal = tree.costs[candidate] + float(np.linalg.norm(new_point - tree.points[candidate]))
            if proposal + 1e-9 >= best_cost:
                continue
            if self.edge_free(tree.points[candidate], new_point, rewire=True):
                best_parent = candidate
                best_cost = proposal

        new_idx = tree.add(new_point, best_parent, best_cost)
        self.budget.added_nodes += 1
        rewired = False
        for candidate in near:
            candidate = int(candidate)
            if candidate in (0, best_parent, new_idx):
                continue
            proposal = tree.costs[new_idx] + float(np.linalg.norm(tree.points[candidate] - new_point))
            if proposal + 1e-9 >= tree.costs[candidate]:
                continue
            if self.edge_free(new_point, tree.points[candidate], rewire=True):
                old_parent = int(tree.parents[candidate])
                tree.children_count[old_parent] = max(0, tree.children_count[old_parent] - 1)
                tree.children_count[new_idx] += 1
                tree.parents[candidate] = new_idx
                tree.costs[candidate] = proposal
                self.budget.rewire_successes += 1
                self._propagate_costs(tree, candidate)
                rewired = True

        distance = float(np.linalg.norm(new_point - self.env.goal))
        if distance <= self.goal_threshold and self.edge_free(new_point, self.env.goal):
            self.goal_candidates.add(new_idx)
        if rewired or new_idx in self.goal_candidates:
            self._refresh_goal(tree)
        self.budget.record(self.best_cost)

    def plan(self) -> PlannerResult:
        tree = Tree(self.env.start, self.budget.maximum + 2)
        while not self.budget.exhausted:
            self._iteration(tree)
        return self.result(tree.size)


class InformedRRTStarPlanner(RRTStarPlanner):
    def __init__(self, *args, **kwargs):
        super().__init__("Informed-RRT*", *args, **kwargs)

    def _sample(self) -> np.ndarray:
        if self.rng.random() < self.goal_bias:
            return self.env.goal.copy()
        if np.isfinite(self.best_cost):
            informed = sample_in_ellipsoid(
                self.rng,
                self.env.start,
                self.env.goal,
                self.best_cost,
                self.env.bounds,
            )
            if informed is not None:
                return informed
        return sample_uniform(self.rng, self.env.bounds)


class BidirectionalConnectBase(PlannerBase):
    def _connect(self, tree: Tree, target: np.ndarray) -> tuple[Optional[int], bool]:
        nearest = tree.nearest(target)
        if float(np.linalg.norm(tree.points[nearest] - target)) <= 1e-8:
            return nearest, True
        last_idx: Optional[int] = None
        while not self.budget.exhausted:
            idx, point = self.attempt_extend(tree, target)
            if idx is None:
                return last_idx, False
            last_idx = idx
            if point is not None and float(np.linalg.norm(point - target)) <= 1e-8:
                return idx, True
        return last_idx, False

    def _expand_and_connect(
        self,
        active: Tree,
        other: Tree,
        active_is_start: bool,
        target: np.ndarray,
    ) -> None:
        idx, new_point = self.attempt_extend(active, target)
        if idx is None or new_point is None:
            return
        other_idx, reached = self._connect(other, new_point)
        if not reached or other_idx is None:
            return
        if active_is_start:
            path = _join_bidirectional(active, idx, other, other_idx)
        else:
            path = _join_bidirectional(other, other_idx, active, idx)
        self.consider_solution(path, active.size + other.size)


class RRTConnectPlanner(BidirectionalConnectBase):
    def __init__(self, *args, **kwargs):
        super().__init__("RRT-Connect", *args, **kwargs)

    def plan(self) -> PlannerResult:
        start_tree = Tree(self.env.start, self.budget.maximum + 2)
        goal_tree = Tree(self.env.goal, self.budget.maximum + 2)
        active_is_start = True
        while not self.budget.exhausted:
            target = sample_uniform(self.rng, self.env.bounds)
            if active_is_start:
                self._expand_and_connect(start_tree, goal_tree, True, target)
            else:
                self._expand_and_connect(goal_tree, start_tree, False, target)
            active_is_start = not active_is_start
        return self.result(start_tree.size + goal_tree.size)


class DynamicRRTPlanner(RRTPlanner):
    """按 Zhao et al. (2023) 的单树结构复现 Dynamic-RRT。"""

    def __init__(
        self,
        *args,
        pareto_interval: int = 8,
        non_pareto_probability: float = 0.1,
        ellipsoid_inflation: float = 1.2,
        goal_bias: float = 0.15,
        **kwargs,
    ):
        PlannerBase.__init__(self, "Dynamic-RRT", *args, **kwargs)
        self.pareto_interval = int(pareto_interval)
        self.non_pareto_probability = float(non_pareto_probability)
        self.ellipsoid_inflation = float(ellipsoid_inflation)
        self.goal_bias = float(goal_bias)

    def _f_hat(self, tree: Tree, idx: int, current_start_idx: int) -> float:
        point = tree.points[idx]
        current_start = tree.points[current_start_idx]
        d_start = float(np.linalg.norm(point - current_start))
        d_goal = float(np.linalg.norm(point - self.env.goal))
        if d_start <= 1e-9 or idx == 0:
            return float("inf")
        bend = max(1.0, tree.costs[idx] / max(float(np.linalg.norm(point - self.env.start)), 1e-9))
        estimate = tree.costs[idx] + max(d_goal, bend * d_goal)
        return max(estimate * self.ellipsoid_inflation, float(np.linalg.norm(current_start - self.env.goal)))

    def _pareto_start(self, tree: Tree, current_start_idx: int) -> int:
        distances = np.linalg.norm(tree.points[: tree.size] - self.env.goal, axis=1)
        candidate_count = min(tree.size, 150)
        candidates = np.argpartition(distances, candidate_count - 1)[:candidate_count]
        values = np.empty((candidate_count, 2), dtype=float)
        for j, idx in enumerate(candidates):
            d_start = float(np.linalg.norm(tree.points[idx] - self.env.start))
            bend = tree.costs[idx] / max(d_start, 1e-9) if idx != 0 else 1.2
            f_hat = tree.costs[idx] + max(distances[idx], bend * distances[idx])
            values[j] = (-float(tree.children_count[idx]), f_hat)
        optimal = np.ones(candidate_count, dtype=bool)
        for i in range(candidate_count):
            dominates = np.all(values <= values[i], axis=1) & np.any(values < values[i], axis=1)
            dominates[i] = False
            if np.any(dominates):
                optimal[i] = False
        pool = np.flatnonzero(~optimal if self.rng.random() < self.non_pareto_probability else optimal)
        if len(pool) == 0:
            pool = np.arange(candidate_count)
        selected = int(candidates[int(self.rng.choice(pool))])
        if selected == current_start_idx and tree.size > 1:
            alternatives = candidates[candidates != current_start_idx]
            if len(alternatives):
                selected = int(self.rng.choice(alternatives))
        return selected

    def plan(self) -> PlannerResult:
        tree = Tree(self.env.start, self.budget.maximum + 2)
        current_start_idx = 0
        successful_extensions = 0
        while not self.budget.exhausted:
            closest_goal_idx = int(np.argmin(np.linalg.norm(tree.points[: tree.size] - self.env.goal, axis=1)))
            if self.rng.random() < self.goal_bias:
                target = self.env.goal.copy()
            else:
                major = self._f_hat(tree, closest_goal_idx, current_start_idx)
                target = sample_in_ellipsoid(
                    self.rng,
                    tree.points[current_start_idx],
                    self.env.goal,
                    major,
                    self.env.bounds,
                )
                if target is None:
                    target = sample_uniform(self.rng, self.env.bounds)
            idx, _ = self.attempt_extend(tree, target)
            if idx is not None:
                successful_extensions += 1
                self._try_goal(tree, idx)
                if successful_extensions % self.pareto_interval == 0:
                    current_start_idx = self._pareto_start(tree, current_start_idx)
        return self.result(tree.size, {"pareto_start_index": current_start_idx})


class SCRRTPlanner(BidirectionalConnectBase):
    def __init__(
        self,
        *args,
        variant: str = "full",
        meet_update_interval: int = 50,
        feedback_interval: int = 50,
        target_improvement_rate: float = 0.02,
        gamma_initial: float = 4.0,
        gamma_min: float = 1.0,
        gamma_max: float = 4.0,
        p_before_first_solution: float = 0.0,
        p_min: float = 0.2,
        p_max: float = 0.95,
        pareto_bias_probability: float = 0.15,
        pid_window_updates: int = 50,
        pid_kp: float = 2.0,
        pid_ki: float = 0.2,
        pid_kd: float = 0.8,
        pid_rho_y: float = 0.9,
        pid_rho_d: float = 0.8,
        pid_integral_min: float = -3.0,
        pid_integral_max: float = 3.0,
        pid_gamma_0: float = 1.5,
        pid_alpha_gamma: float = 0.5,
        pid_p_0: float = 0.8,
        pid_alpha_p: float = 0.5,
        connect_trigger_steps: float = 12.0,
        connect_max_steps: int = 8,
        rewire_radius_steps: float = 2.5,
        **kwargs,
    ):
        names = {
            "full": "SC-RRT",
            "no_adcs": "SC-RRT-no-ADCS",
            "no_ssfor": "SC-RRT-no-SSFOR",
        }
        super().__init__(names[variant], *args, **kwargs)
        self.variant = variant
        self.meet_update_interval = int(meet_update_interval)
        self.feedback_interval = int(feedback_interval)
        self.target_improvement_rate = float(target_improvement_rate)
        self.gamma = [float(gamma_initial), float(gamma_initial)]
        self.gamma_min = float(gamma_min)
        self.gamma_max = float(gamma_max)
        self.p_before = float(p_before_first_solution)
        self.p_informed = [self.p_before, self.p_before]
        self.p_min = float(p_min)
        self.p_max = float(p_max)
        self.pareto_bias_probability = float(pareto_bias_probability)
        self.pid_window_updates = int(pid_window_updates)
        self.pid_kp = float(pid_kp)
        self.pid_ki = float(pid_ki)
        self.pid_kd = float(pid_kd)
        self.pid_rho_y = float(pid_rho_y)
        self.pid_rho_d = float(pid_rho_d)
        self.pid_integral_min = float(pid_integral_min)
        self.pid_integral_max = float(pid_integral_max)
        self.pid_gamma_0 = float(pid_gamma_0)
        self.pid_alpha_gamma = float(pid_alpha_gamma)
        self.pid_p_0 = float(pid_p_0)
        self.pid_alpha_p = float(pid_alpha_p)
        self.connect_trigger_distance = float(connect_trigger_steps) * self.step_size
        self.connect_max_steps = int(connect_max_steps)
        self.rewire_radius = float(rewire_radius_steps) * self.step_size
        self.meet = 0.5 * (self.env.start + self.env.goal)
        self.best_side_costs = [float("inf"), float("inf")]
        self.pid_cost_history: list[float] = []
        self.pid_error_previous = 0.0
        self.pid_integral = 0.0
        self.pid_derivative_filtered = 0.0
        self.pid_improvement_ema = 0.0
        self.feedback_updates = 0

    def _update_meet(self, start_tree: Tree, goal_tree: Tree) -> None:
        a0 = max(0, start_tree.size - 128)
        b0 = max(0, goal_tree.size - 128)
        a = start_tree.points[a0 : start_tree.size]
        b = goal_tree.points[b0 : goal_tree.size]
        if len(a) == 0 or len(b) == 0:
            return
        distances = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
        i, j = np.unravel_index(int(np.argmin(distances)), distances.shape)
        target = 0.5 * (a[i] + b[j])
        self.meet = 0.7 * self.meet + 0.3 * target

    def _pareto_target(self, other: Tree, toward: np.ndarray) -> np.ndarray:
        count = min(other.size, 96)
        distances = np.linalg.norm(other.points[: other.size] - toward, axis=1)
        candidates = np.argpartition(distances, count - 1)[:count]
        child = other.children_count[candidates].astype(float)
        score = distances[candidates] / max(float(np.max(distances[candidates])), 1e-9) - 0.15 * child
        if self.rng.random() < 0.2:
            idx = int(self.rng.choice(candidates))
        else:
            idx = int(candidates[int(np.argmin(score))])
        jitter = self.rng.normal(scale=0.15 * self.step_size, size=self.env.dim)
        return np.clip(other.points[idx] + jitter, self.env.bounds[:, 0], self.env.bounds[:, 1])

    def _sample_side(self, side: int, own: Tree, other: Tree) -> np.ndarray:
        del own
        draw = self.rng.random()
        p = self.p_informed[side] if self.variant != "no_adcs" else 0.0
        if draw < p and np.isfinite(self.best_side_costs[side]):
            focus = self.env.start if side == 0 else self.env.goal
            focal_distance = float(np.linalg.norm(focus - self.meet))
            major = max(self.best_side_costs[side] * self.gamma[side], focal_distance + 1e-6)
            sample = sample_in_ellipsoid(self.rng, focus, self.meet, major, self.env.bounds)
            if sample is not None:
                return sample
        cross_threshold = p + (1.0 - p) * self.pareto_bias_probability
        if draw < cross_threshold:
            return self._pareto_target(other, self.meet)
        return sample_uniform(self.rng, self.env.bounds)

    def _feedback(self) -> None:
        if self.variant != "full":
            return
        self.pid_cost_history.append(self.best_cost)
        self.feedback_updates += 1
        if not np.isfinite(self.best_cost) or len(self.pid_cost_history) < self.pid_window_updates + 1:
            self.gamma = [self.gamma_max, self.gamma_max]
            self.p_informed = [0.0, 0.0]
            return
        old = self.pid_cost_history[-self.pid_window_updates - 1]
        if not np.isfinite(old) and np.isfinite(self.best_cost):
            improvement = 1.0
        else:
            improvement = float(np.clip((old - self.best_cost) / max(old, 1e-6), 0.0, 1.0))
        self.pid_improvement_ema = (
            self.pid_rho_y * self.pid_improvement_ema
            + (1.0 - self.pid_rho_y) * improvement
        )
        error = self.target_improvement_rate - self.pid_improvement_ema
        derivative = error - self.pid_error_previous
        self.pid_derivative_filtered = (
            self.pid_rho_d * self.pid_derivative_filtered
            + (1.0 - self.pid_rho_d) * derivative
        )
        candidate_integral = float(np.clip(
            self.pid_integral + error,
            self.pid_integral_min,
            self.pid_integral_max,
        ))

        def map_output(integral: float) -> tuple[float, float]:
            output = (
                self.pid_kp * error
                + self.pid_ki * integral
                + self.pid_kd * self.pid_derivative_filtered
            )
            gamma = float(np.clip(
                self.pid_gamma_0 * np.exp(self.pid_alpha_gamma * output),
                self.gamma_min,
                self.gamma_max,
            ))
            p_value = float(np.clip(
                self.pid_p_0 - self.pid_alpha_p * np.tanh(output),
                self.p_min,
                self.p_max,
            ))
            return gamma, p_value

        gamma, p_value = map_output(candidate_integral)
        saturated = (
            (gamma >= self.gamma_max - 1e-6 and error > 0)
            or (gamma <= self.gamma_min + 1e-6 and error < 0)
            or (p_value <= self.p_min + 1e-6 and error > 0)
            or (p_value >= self.p_max - 1e-6 and error < 0)
        )
        if saturated:
            gamma, p_value = map_output(self.pid_integral)
        else:
            self.pid_integral = candidate_integral
        self.gamma = [gamma, gamma]
        self.p_informed = [p_value, p_value]
        self.pid_error_previous = error

    def on_solution_improved(self, extra: dict) -> None:
        side_costs = extra.get("side_costs")
        if side_costs is not None:
            self.best_side_costs = [float(side_costs[0]), float(side_costs[1])]
        if self.variant == "no_ssfor" and self.p_informed[0] == self.p_before:
            self.gamma = [self.pid_gamma_0, self.pid_gamma_0]
            self.p_informed = [self.pid_p_0, self.pid_p_0]

    def _propagate_costs(self, tree: Tree, root: int) -> None:
        queue = [root]
        while queue:
            parent = queue.pop()
            children = np.flatnonzero(tree.parents[: tree.size] == parent)
            for child in children:
                tree.costs[child] = tree.costs[parent] + float(
                    np.linalg.norm(tree.points[child] - tree.points[parent])
                )
                queue.append(int(child))

    def _extend_sc(self, tree: Tree, target: np.ndarray) -> tuple[Optional[int], Optional[np.ndarray]]:
        """一次原子扩展，保留原 SC-RRT 的局部最优父节点与重布线。"""
        if not self.budget.consume():
            return None, None
        nearest = tree.nearest(target)
        new_point = steer(tree.points[nearest], target, self.step_size)
        if float(np.linalg.norm(new_point - tree.points[nearest])) <= 1e-10:
            self.budget.record(self.best_cost)
            return None, new_point
        if not self.edge_free(tree.points[nearest], new_point):
            self.budget.collision_rejections += 1
            self.budget.record(self.best_cost)
            return None, new_point
        near = tree.near(new_point, self.rewire_radius)
        best_parent = nearest
        best_cost = tree.costs[nearest] + float(np.linalg.norm(new_point - tree.points[nearest]))
        for candidate in near:
            candidate = int(candidate)
            proposal = tree.costs[candidate] + float(np.linalg.norm(new_point - tree.points[candidate]))
            if proposal + 1e-9 < best_cost and self.edge_free(tree.points[candidate], new_point, rewire=True):
                best_parent, best_cost = candidate, proposal
        new_idx = tree.add(new_point, best_parent, best_cost)
        self.budget.added_nodes += 1
        for candidate in near:
            candidate = int(candidate)
            if candidate in (0, best_parent, new_idx):
                continue
            proposal = best_cost + float(np.linalg.norm(tree.points[candidate] - new_point))
            if proposal + 1e-9 >= tree.costs[candidate]:
                continue
            if self.edge_free(new_point, tree.points[candidate], rewire=True):
                old_parent = int(tree.parents[candidate])
                tree.children_count[old_parent] = max(0, tree.children_count[old_parent] - 1)
                tree.children_count[new_idx] += 1
                tree.parents[candidate] = new_idx
                tree.costs[candidate] = proposal
                self.budget.rewire_successes += 1
                self._propagate_costs(tree, candidate)
        self.budget.record(self.best_cost)
        return new_idx, new_point

    def _try_bridge(
        self,
        active: Tree,
        active_idx: int,
        other: Tree,
        other_idx: int,
        start_side: bool,
    ) -> bool:
        if not self.edge_free(active.points[active_idx], other.points[other_idx]):
            return False
        if start_side:
            path = _join_bidirectional(active, active_idx, other, other_idx)
            bridge_half = 0.5 * float(np.linalg.norm(active.points[active_idx] - other.points[other_idx]))
            side_costs = (active.costs[active_idx] + bridge_half, other.costs[other_idx] + bridge_half)
        else:
            path = _join_bidirectional(other, other_idx, active, active_idx)
            bridge_half = 0.5 * float(np.linalg.norm(active.points[active_idx] - other.points[other_idx]))
            side_costs = (other.costs[other_idx] + bridge_half, active.costs[active_idx] + bridge_half)
        self.consider_solution(
            path,
            active.size + other.size,
            {"side_costs": side_costs},
        )
        return True

    def _expand_sc(
        self,
        start_tree: Tree,
        goal_tree: Tree,
        start_side: bool,
    ) -> None:
        active = start_tree if start_side else goal_tree
        other = goal_tree if start_side else start_tree
        side = 0 if start_side else 1
        target = self._sample_side(side, active, other)
        idx, new_point = self._extend_sc(active, target)
        if idx is None or new_point is None:
            return
        other_idx = other.nearest(new_point)
        distance = float(np.linalg.norm(other.points[other_idx] - new_point))
        if distance <= self.goal_threshold:
            self._try_bridge(active, idx, other, other_idx, start_side)
            return
        if distance >= self.connect_trigger_distance:
            return
        current_idx = idx
        for _ in range(self.connect_max_steps):
            if self.budget.exhausted:
                break
            current_idx, point = self._extend_sc(active, other.points[other_idx])
            if current_idx is None or point is None:
                break
            distance = float(np.linalg.norm(point - other.points[other_idx]))
            if distance <= self.goal_threshold:
                self._try_bridge(active, current_idx, other, other_idx, start_side)
                break

    def plan(self) -> PlannerResult:
        start_tree = Tree(self.env.start, self.budget.maximum + 2)
        goal_tree = Tree(self.env.goal, self.budget.maximum + 2)
        start_side = True
        last_meet_update = 0
        last_feedback = 0
        while not self.budget.exhausted:
            self._expand_sc(start_tree, goal_tree, start_side)
            start_side = not start_side
            if self.budget.attempts - last_meet_update >= self.meet_update_interval:
                self._update_meet(start_tree, goal_tree)
                last_meet_update = self.budget.attempts
            if self.budget.attempts - last_feedback >= self.feedback_interval:
                self._feedback()
                last_feedback = self.budget.attempts
        return self.result(
            start_tree.size + goal_tree.size,
            {
                "gamma_a_final": self.gamma[0],
                "gamma_b_final": self.gamma[1],
                "p_a_final": self.p_informed[0],
                "p_b_final": self.p_informed[1],
                "feedback_updates": self.feedback_updates,
                "pid_improvement_ema_final": self.pid_improvement_ema,
            },
        )


def build_planner(
    algorithm: str,
    env: Environment,
    seed: int,
    config: dict,
) -> PlannerBase:
    common = dict(
        env=env,
        seed=seed,
        maximum=int(config["max_extension_attempts"]),
        checkpoints=config["checkpoints"],
        step_size=float(config["step_size_mm"]),
        goal_threshold=float(config["goal_threshold_mm"]),
        collision_margin=float(config.get("collision_margin_mm", 0.0)),
    )
    if algorithm == "RRT":
        return RRTPlanner(**common, goal_bias=float(config["goal_bias"]))
    if algorithm == "RRT-Connect":
        return RRTConnectPlanner(**common)
    if algorithm == "RRT*":
        return RRTStarPlanner("RRT*", **common, goal_bias=float(config["goal_bias"]))
    if algorithm == "Informed-RRT*":
        return InformedRRTStarPlanner(**common, goal_bias=float(config["goal_bias"]))
    if algorithm == "Dynamic-RRT":
        return DynamicRRTPlanner(**common, **config["dynamic_rrt"])
    if algorithm == "SC-RRT":
        return SCRRTPlanner(**common, variant="full", **config["sc_rrt"])
    if algorithm == "SC-RRT-no-ADCS":
        return SCRRTPlanner(**common, variant="no_adcs", **config["sc_rrt"])
    if algorithm == "SC-RRT-no-SSFOR":
        return SCRRTPlanner(**common, variant="no_ssfor", **config["sc_rrt"])
    raise KeyError(f"未知算法: {algorithm}")
