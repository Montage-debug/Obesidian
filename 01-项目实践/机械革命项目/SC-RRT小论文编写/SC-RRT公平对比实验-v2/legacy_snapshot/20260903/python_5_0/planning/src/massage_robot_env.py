"""
按摩机器人场景环境：加载障碍物、工作空间边界、测试案例。
"""

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

from collision_world import CollisionWorld
from exp_paths import EXP_ROOT as ROOT


class MassageRobotEnv:
    """按摩机器人笛卡尔任务空间环境"""

    def __init__(
        self,
        obstacles_config: Optional[Path] = None,
        obstacle_set: Optional[str] = None,
    ):
        config_path = obstacles_config or ROOT / "config" / "obstacles.yaml"
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = yaml.safe_load(f)
        if obstacle_set:
            self._select_obstacle_set(obstacle_set)

        self.bounds = np.array(self.config["workspace"]["bounds"], dtype=float)
        self.spheres = self._load_spheres()
        self.boxes = self._load_boxes()
        self.min_clearance = self.config["safety"]["min_clearance"]
        self.work_surface_z_min = self._work_surface_z_min()

    def _select_obstacle_set(self, obstacle_set: str) -> None:
        """从压力集单一配置源装载指定案例的球体与方盒。"""
        stress_cases = self.config.get("stress_cases", {})
        if obstacle_set not in stress_cases:
            raise KeyError(f"Unknown obstacle_set: {obstacle_set}")
        selected = stress_cases[obstacle_set]
        self.config["body"] = {
            "type": "sphere_cluster",
            "spheres": selected.get("spheres", []),
        }
        self.config["obstacles"] = {
            "type": "boxes",
            "boxes": selected.get("boxes", []),
        }

    @staticmethod
    def _work_surface_z_from_plane(work_plane: dict) -> float:
        center = work_plane["center"]
        size = work_plane["size"]
        return float(center[2]) + float(size[2]) * 0.5

    def _work_surface_z_min(self) -> float:
        """TCP 允许的最小 Z（base_link），防止穿入桌面下方。"""
        ws = self.config.get("work_surface", {})
        if "z_min" in ws:
            return float(ws["z_min"])
        wp = self.config.get("work_plane")
        if wp:
            return self._work_surface_z_from_plane(wp) - float(ws.get("margin", 0.01))
        return float(self.bounds[4])

    def clamp_to_work_surface(self, point: np.ndarray) -> np.ndarray:
        """将点 Z 抬升到台面以上。"""
        p = np.asarray(point, dtype=float).copy()
        p[2] = max(p[2], self.work_surface_z_min)
        return p

    def clamp_path_to_work_surface(self, path: List) -> List[np.ndarray]:
        """整段路径抬升到工作台面以上。"""
        return [self.clamp_to_work_surface(p) for p in path]

    def _load_spheres(self) -> List[Dict]:
        """加载球体障碍物（人体背部）"""
        spheres = []
        body = self.config.get("body", {})
        for s in body.get("spheres", []):
            spheres.append({
                "center": np.array(s["center"], dtype=float),
                "radius": float(s["radius"]),
                "type": "body",
            })
        return spheres

    def _load_boxes(self) -> List[Dict]:
        """加载长方体障碍物（床、柜体、额外方盒）"""
        boxes = []
        for section in ["bed", "cabinet", "obstacles"]:
            for b in self.config.get(section, {}).get("boxes", []):
                boxes.append({
                    "center": np.array(b["center"], dtype=float),
                    "half_size": np.array(b["size"], dtype=float) / 2.0,
                    "type": section,
                })
        return boxes

    @classmethod
    def planner_obstacles(
        cls,
        env_builder: "MassageRobotEnv",
        safety_margin_m: float = 0.0,
    ) -> np.ndarray:
        """规划器障碍物：球体簇 + 统一裕度（方盒走 CollisionWorld 线段检测）。"""
        world = CollisionWorld.from_env_builder(env_builder, safety_margin_m)
        return world.planner_obstacles_m()

    @staticmethod
    def bounds_with_margin(
        start: np.ndarray,
        goal: np.ndarray,
        base_bounds: np.ndarray,
        margin: float = 0.05,
        work_surface_z_min: float = 0.0,
    ) -> List[float]:
        """在配置边界与起终点包络之间取并集，保证采样覆盖实际案例"""
        pts = np.vstack([start, goal])
        merged = base_bounds.copy()
        merged[0] = min(merged[0], pts[:, 0].min() - margin)
        merged[1] = max(merged[1], pts[:, 0].max() + margin)
        merged[2] = min(merged[2], pts[:, 1].min() - margin)
        merged[3] = max(merged[3], pts[:, 1].max() + margin)
        merged[4] = min(merged[4], max(work_surface_z_min, pts[:, 2].min() - margin))
        merged[5] = max(merged[5], pts[:, 2].max() + margin)
        return merged.tolist()

    @classmethod
    def from_case(
        cls,
        case: Dict,
        obstacles_config: Optional[Path] = None,
        step_size: float = 0.02,
        goal_threshold: float = 0.04,
        safety_margin_m: float = 0.0,
    ) -> Dict:
        """从 moveto case 构建 SC-RRT 可用的 env 字典"""
        env_builder = cls(obstacles_config, case.get("obstacle_set"))
        start = np.array(case["start"]["pos"], dtype=float)
        goal = np.array(case["goal"]["pos"], dtype=float)
        collision_world = CollisionWorld.from_env_builder(
            env_builder, safety_margin_m=safety_margin_m,
        )
        obstacles = collision_world.planner_obstacles_m()
        bounds = cls.bounds_with_margin(
            start, goal, env_builder.bounds,
            work_surface_z_min=env_builder.work_surface_z_min,
        )

        env = {
            "dim": 3,
            "dimension": 3,
            "bounds": bounds,
            "start_point": start,
            "goal_point": goal,
            "start": start,
            "goal": goal,
            "obstacles": obstacles,
            "boxes": env_builder.boxes,
            "collision_world": collision_world,
            "start_pose": case["start"],
            "goal_pose": case["goal"],
            "baseline_path": case.get("baseline_path", []),
            "case_id": case.get("global_id", case.get("id", 0)),
            "session_id": case.get("session_id", ""),
            "min_clearance": env_builder.min_clearance,
            "step_size": step_size,
            "goal_threshold": goal_threshold,
        }
        return env

    @staticmethod
    def to_mm_planner_env(env: Dict, scale: float = 1000.0) -> Dict:
        """将米制环境缩放到毫米，匹配 SC-RRT 原始算法参数尺度"""
        mm = dict(env)
        mm["start_point"] = np.asarray(env["start_point"], dtype=float) * scale
        mm["goal_point"] = np.asarray(env["goal_point"], dtype=float) * scale
        mm["start"] = mm["start_point"].copy()
        mm["goal"] = mm["goal_point"].copy()
        obs = np.asarray(env["obstacles"], dtype=float).copy()
        if obs.size > 0:
            obs[:, :3] *= scale
            obs[:, 3] *= scale
        mm["obstacles"] = obs
        mm["bounds"] = [float(b) * scale for b in env["bounds"]]
        mm["step_size"] = float(env.get("step_size", 0.02)) * scale
        mm["goal_threshold"] = float(env.get("goal_threshold", 0.04)) * scale
        mm["_length_scale"] = scale
        return mm

    @staticmethod
    def path_to_meters(path: List, scale: float = 1000.0) -> List[np.ndarray]:
        """规划器输出路径转回米"""
        if path is None:
            return []
        if isinstance(path, np.ndarray):
            if path.size == 0:
                return []
            return [np.asarray(p, dtype=float) / scale for p in path]
        if len(path) == 0:
            return []
        return [np.asarray(p, dtype=float) / scale for p in path]

    def is_point_collision(self, point: np.ndarray) -> bool:
        """检查点是否与障碍物碰撞"""
        for s in self.spheres:
            if np.linalg.norm(point - s["center"]) < s["radius"]:
                return True
        for b in self.boxes:
            diff = np.abs(point - b["center"])
            if np.all(diff < b["half_size"]):
                return True
        return False

    def is_segment_collision(self, p1: np.ndarray, p2: np.ndarray, num_checks: int = 30) -> bool:
        """检查线段是否与障碍物（球体+长方体）碰撞"""
        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        for t in np.linspace(0, 1, num_checks):
            if self.is_point_collision(p1 + t * (p2 - p1)):
                return True
        return False

    def min_segment_body_clearance(
        self,
        p1: np.ndarray,
        p2: np.ndarray,
        num_checks: int = 40,
    ) -> float:
        """线段相对人体表面的最小间隙（米）"""
        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        body_spheres = [s for s in self.spheres if s["type"] == "body"]
        if not body_spheres:
            return float("inf")

        min_dist = float("inf")
        for t in np.linspace(0, 1, num_checks):
            pt = p1 + t * (p2 - p1)
            for s in body_spheres:
                dist = np.linalg.norm(pt - s["center"]) - s["radius"]
                min_dist = min(min_dist, dist)
        return min_dist

    def compute_clearance(self, path: List[np.ndarray]) -> float:
        """计算路径上 TCP 距人体表面的最小间隙（含线段采样）"""
        if not path:
            return 0.0

        body_spheres = [s for s in self.spheres if s["type"] == "body"]
        if not body_spheres:
            return float("inf")

        min_dist = float("inf")
        pts = [np.asarray(p, dtype=float) for p in path]
        for i in range(len(pts)):
            for s in body_spheres:
                dist = np.linalg.norm(pts[i] - s["center"]) - s["radius"]
                min_dist = min(min_dist, dist)
            if i + 1 < len(pts):
                seg_clear = self.min_segment_body_clearance(pts[i], pts[i + 1])
                min_dist = min(min_dist, seg_clear)

        return max(min_dist, 0.0)

    @staticmethod
    def load_cases(cases_file: Optional[Path] = None) -> List[Dict]:
        """加载测试案例"""
        path = cases_file or ROOT / "data" / "processed" / "selected_15_cases.json"
        if not path.exists():
            path = ROOT / "data" / "processed" / "selected_30_cases.json"
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data["cases"]

    @staticmethod
    def load_all_cases() -> List[Dict]:
        """加载全部案例"""
        path = ROOT / "data" / "processed" / "moveto_cases.json"
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data["cases"]
