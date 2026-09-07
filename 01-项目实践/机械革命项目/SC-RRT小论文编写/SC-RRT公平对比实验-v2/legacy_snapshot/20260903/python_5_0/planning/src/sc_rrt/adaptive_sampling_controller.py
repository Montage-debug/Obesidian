"""
自适应采样控制器 - 搜索状态反馈（SSFOR / EER-SSFOR）
"""

import numpy as np
from typing import Dict, Tuple, Optional
from enum import Enum


class SearchPhase(Enum):
  """搜索阶段枚举"""
  EXPLORATION = "exploration"
  EXPLOITATION = "exploitation"
  CONVERGENCE = "convergence"


class AdaptiveSamplingController:
    """
    双侧独立的 gamma / p_informed 调节器。
    use_eer_feedback=True 时使用窗口有效扩展率（EER-SSFOR）；
    否则保留代价历史闭环（Original SC 消融）。
    """

    def __init__(
        self,
        max_iterations: int = 1000,
        gamma_initial: float = 2.5,
        p_initial: float = 0.2,
        gamma_nominal: float = 1.5,
        p_nominal: float = 0.8,
        gamma_min: float = 1.0,
        gamma_max: float = 4.0,
        p_min: float = 0.2,
        p_max: float = 0.95,
        feedback_interval: int = 20,
        cost_window: int = 5,
        improvement_threshold: float = 0.005,
        gamma_step: float = 0.15,
        p_step: float = 0.04,
        sample_efficiency_window: int = 50,
        min_efficiency_threshold: float = 0.2,
        global_p_floor: float = 0.12,
        frontier_progress_threshold: float = 0.015,
        use_eer_feedback: bool = True,
    ):
        self.max_iterations = max_iterations
        self.gamma_initial = gamma_initial
        self.p_initial = p_initial
        self.gamma_nominal = gamma_nominal
        self.p_nominal = p_nominal
        self.gamma_min = gamma_min
        self.gamma_max = gamma_max
        self.p_min = max(p_min, global_p_floor)
        self.p_max = p_max
        self.feedback_interval = max(1, feedback_interval)
        self.cost_window = max(2, cost_window)
        self.improvement_threshold = improvement_threshold
        self.gamma_step = gamma_step
        self.p_step = p_step
        self.sample_efficiency_window = sample_efficiency_window
        self.min_efficiency_threshold = min_efficiency_threshold
        self.global_p_floor = global_p_floor
        self.frontier_progress_threshold = frontier_progress_threshold
        self.use_eer_feedback = use_eer_feedback

        self.recent_samples = []
        self.cost_history = []
        self.current_phase = SearchPhase.EXPLORATION
        self.iteration_count = 0
        self.has_solution = False
        self.solution_found_iter = None
        self.current_gamma = gamma_initial
        self.current_p_informed = p_initial
        self.phase_history = []
        self.gamma_history = []
        self.p_history = []
        self.efficiency_history = []

        # EER 窗口（由规划器在 feedback 边界写入）
        self._win_attempts = 0
        self._win_added = 0
        self._win_collisions = 0
        self._win_frontier = 0

    def accumulate_eer_window(
        self,
        attempts: int,
        added: int,
        collision_rejections: int,
        frontier_progress: int,
    ) -> None:
        """规划器每步累加窗口统计，在 feedback_interval 边界调用 update。"""
        self._win_attempts += int(attempts)
        self._win_added += int(added)
        self._win_collisions += int(collision_rejections)
        self._win_frontier += int(frontier_progress)

    def _consume_eer_window(self) -> Tuple[float, float, float]:
        att = max(1, self._win_attempts)
        ext_rate = self._win_added / att
        col_rate = self._win_collisions / att
        prog_rate = self._win_frontier / att
        self._win_attempts = 0
        self._win_added = 0
        self._win_collisions = 0
        self._win_frontier = 0
        return ext_rate, col_rate, prog_rate

    def update(
        self,
        iteration: int,
        has_solution: bool,
        tree_size_a: int,
        tree_size_b: int,
        recent_valid_samples: int = 0,
        recent_total_samples: int = 0,
        best_path_cost: float = np.inf,
        force_feedback: bool = False,
    ) -> Tuple[float, float, Dict]:
        """调节 gamma 与 informed 概率；窗口边界才应用 EER 规则。"""
        self.iteration_count = iteration
        if has_solution and not self.has_solution:
            self.has_solution = True
            self.solution_found_iter = iteration

        if recent_total_samples > 0:
            self.recent_samples.append((recent_total_samples, recent_valid_samples))
            if len(self.recent_samples) > self.sample_efficiency_window:
                self.recent_samples.pop(0)
        sample_efficiency = self._calculate_sample_efficiency()

        relative_improvement = 0.0
        adjustment = "hold"
        ext_rate = col_rate = prog_rate = 0.0

        if not has_solution or not np.isfinite(best_path_cost):
            self.current_phase = SearchPhase.EXPLORATION
            gamma = self.gamma_initial
            p_informed = self.p_initial
            adjustment = "explore"
        else:
            progress = iteration / max(self.max_iterations, 1)
            self.current_phase = (
                SearchPhase.CONVERGENCE if progress >= 0.7 else SearchPhase.EXPLOITATION
            )
            gamma = self.current_gamma
            p_informed = self.current_p_informed

            if self.use_eer_feedback and (force_feedback or iteration % self.feedback_interval == 0):
                ext_rate, col_rate, prog_rate = self._consume_eer_window()
                # 高碰撞、低扩展：放宽椭球、提高全局探索
                if col_rate > 0.55 and ext_rate < 0.25:
                    gamma += self.gamma_step * 1.2
                    p_informed -= self.p_step * 1.5
                    adjustment = "eer_high_collision"
                elif ext_rate > 0.45 and prog_rate < self.frontier_progress_threshold:
                    gamma -= self.gamma_step
                    p_informed += self.p_step
                    adjustment = "eer_low_frontier"
                elif prog_rate >= self.frontier_progress_threshold:
                    gamma -= 0.5 * self.gamma_step
                    p_informed += 0.5 * self.p_step
                    adjustment = "eer_frontier_ok"
                elif ext_rate < self.min_efficiency_threshold:
                    gamma += self.gamma_step
                    p_informed -= self.p_step
                    adjustment = "eer_low_extension"
            elif not self.use_eer_feedback and iteration % self.feedback_interval == 0:
                self.cost_history.append(float(best_path_cost))
                self.cost_history = self.cost_history[-self.cost_window:]
                if len(self.cost_history) >= 2:
                    old = self.cost_history[0]
                    new = self.cost_history[-1]
                    relative_improvement = max(0.0, (old - new) / max(abs(old), 1e-12))
                    if relative_improvement > self.improvement_threshold:
                        gamma -= self.gamma_step
                        p_informed += self.p_step
                        adjustment = "tighten"
                    else:
                        gamma += self.gamma_step
                        p_informed -= self.p_step
                        adjustment = "relax"
                else:
                    gamma = 0.5 * (gamma + self.gamma_nominal)
                    p_informed = 0.5 * (p_informed + self.p_nominal)

        if not self.use_eer_feedback and sample_efficiency < self.min_efficiency_threshold:
            gamma += self.gamma_step
            p_informed -= self.p_step
            adjustment = "efficiency_relax"

        gamma = float(np.clip(gamma, self.gamma_min, self.gamma_max))
        p_informed = float(np.clip(p_informed, self.p_min, self.p_max))
        # 知情概率上限内仍保证全局采样下限（1-p 不低于 floor）
        if (1.0 - p_informed) < self.global_p_floor:
            p_informed = 1.0 - self.global_p_floor

        self.current_gamma = gamma
        self.current_p_informed = p_informed

        self.phase_history.append(self.current_phase.value)
        self.gamma_history.append(gamma)
        self.p_history.append(p_informed)
        self.efficiency_history.append(sample_efficiency)
        info = {
            "phase": self.current_phase.value,
            "gamma": gamma,
            "p_informed": p_informed,
            "sample_efficiency": sample_efficiency,
            "relative_cost_improvement": relative_improvement,
            "effective_extension_rate": ext_rate,
            "collision_rejection_rate": col_rate,
            "frontier_progress_rate": prog_rate,
            "has_solution": has_solution,
            "tree_size_total": tree_size_a + tree_size_b,
            "adjustment": adjustment,
        }
        return gamma, p_informed, info

    def _calculate_sample_efficiency(self) -> float:
        if len(self.recent_samples) == 0:
            return 0.5
        total_total = sum(s[0] for s in self.recent_samples)
        total_valid = sum(s[1] for s in self.recent_samples)
        if total_total == 0:
            return 0.5
        return total_valid / total_total

    def reset(self):
        self.current_phase = SearchPhase.EXPLORATION
        self.iteration_count = 0
        self.has_solution = False
        self.solution_found_iter = None
        self.current_gamma = self.gamma_initial
        self.current_p_informed = self.p_initial
        self.recent_samples = []
        self.cost_history = []
        self._win_attempts = 0
        self._win_added = 0
        self._win_collisions = 0
        self._win_frontier = 0
        self.phase_history = []
        self.gamma_history = []
        self.p_history = []
        self.efficiency_history = []

    def get_current_state(self) -> Dict:
        return {
            "phase": self.current_phase.value,
            "gamma": self.current_gamma,
            "p_informed": self.current_p_informed,
            "iteration": self.iteration_count,
            "has_solution": self.has_solution,
            "sample_efficiency": self._calculate_sample_efficiency(),
        }


class AdaptiveSamplingConfig:
    @staticmethod
    def get_config(scenario: str = "balanced") -> Dict:
        configs = {
            "balanced": {
                "gamma_initial": 2.5,
                "p_initial": 0.2,
                "gamma_nominal": 1.5,
                "p_nominal": 0.8,
                "min_efficiency_threshold": 0.2,
            },
        }
        return configs.get(scenario, configs["balanced"])
