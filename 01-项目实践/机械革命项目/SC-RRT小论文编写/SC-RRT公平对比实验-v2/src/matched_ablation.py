"""启动条件一致的 SSFOR 对照；不修改六算法已运行的实现。"""
from __future__ import annotations

import hashlib
import json
import numpy as np

from planners import SCRRTPlanner


class MatchedSCRRT(SCRRTPlanner):
    def __init__(self, *args, fixed=False, **kwargs):
        super().__init__(*args, variant="full", **kwargs)
        self.fixed = fixed
        self.algorithm = "SC-RRT-fixed-matched" if fixed else "SC-RRT"
        self.activation_attempt = None
        self.activation_snapshot = None
        self.control_trace = []

    def _feedback(self):
        if self.fixed:
            self.pid_cost_history.append(self.best_cost)
            self.feedback_updates += 1
            if not np.isfinite(self.best_cost) or len(self.pid_cost_history) < self.pid_window_updates + 1:
                self.gamma = [self.gamma_max] * 2
                self.p_informed = [0.0] * 2
            else:
                self.gamma = [self.pid_gamma_0] * 2
                self.p_informed = [self.pid_p_0] * 2
        else:
            super()._feedback()
        if self.p_informed[0] > 0 and self.activation_attempt is None:
            self.activation_attempt = self.budget.attempts
            self.activation_snapshot = {
                "attempt": self.budget.attempts,
                "nodes_added": self.budget.added_nodes,
                "checks": self.budget.collision_checks,
                "best_cost": self.best_cost,
                "rng_sha256": hashlib.sha256(json.dumps(self.rng.bit_generator.state, sort_keys=True).encode()).hexdigest(),
            }
        self.control_trace.append({
            "attempt": self.budget.attempts,
            "gamma": self.gamma[0], "p": self.p_informed[0],
            "best_cost_mm": self.best_cost if np.isfinite(self.best_cost) else None,
        })


def build_matched(env, seed, config, fixed=False):
    return MatchedSCRRT(
        env=env, seed=seed, maximum=config["max_extension_attempts"],
        checkpoints=config["checkpoints"], step_size=config["step_size_mm"],
        goal_threshold=config["goal_threshold_mm"],
        collision_margin=config["collision_margin_mm"], fixed=fixed,
        **config["sc_rrt"],
    )
