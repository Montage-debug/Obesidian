#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多阶段 TCP 链：每段独立 SC-RRT，拼接后整链后处理。"""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from arm_segment_checker import attach_arm_checker_to_env
from massage_robot_env import MassageRobotEnv


def _pos_from_stage(entry: dict) -> np.ndarray:
    if "pos" in entry:
        return np.asarray(entry["pos"], dtype=float)
    return np.asarray(entry["position"], dtype=float)


def stage_chain_from_case(case: dict) -> Tuple[List[np.ndarray], List[str]]:
    labels: List[str] = ["start"]
    chain: List[np.ndarray] = [np.asarray(case["start"]["pos"], dtype=float)]
    for i, vp in enumerate(case.get("via_points") or []):
        chain.append(_pos_from_stage(vp))
        labels.append(str(vp.get("label") or vp.get("acupoint_code") or f"via_{i + 1}"))
    chain.append(np.asarray(case["goal"]["pos"], dtype=float))
    labels.append("goal")
    return chain, labels


def has_multivia(case: dict) -> bool:
    return len(case.get("via_points") or []) > 0


def env_for_segment(case, obstacles_config, p_start, p_goal, step_size, goal_threshold, safety_margin_m):
    seg_case = deepcopy(case)
    seg_case["start"] = {"pos": p_start.tolist(), "rpy": case["start"].get("rpy", [0, 0, 0])}
    seg_case["goal"] = {"pos": p_goal.tolist(), "rpy": case["goal"].get("rpy", [0, 0, 0])}
    return MassageRobotEnv.from_case(
        seg_case, obstacles_config=obstacles_config,
        step_size=step_size, goal_threshold=goal_threshold, safety_margin_m=safety_margin_m,
    )


def _concat_paths(segments: List[List]) -> List:
    if not segments:
        return []
    out: List = []
    for si, seg in enumerate(segments):
        if not seg:
            continue
        if si == 0:
            out.extend(seg)
        else:
            out.extend(seg[1:])
    return out


def _seed_checker_at_tcp(checker, tcp_m: Sequence[float]) -> None:
    if checker is None:
        return
    q, ok = checker._solver.solve_position(np.asarray(tcp_m, dtype=float), checker._last_q)
    if ok:
        checker.set_seed(q)


def plan_multivia(case, env_builder, obstacles_config, max_iterations, safety_margin,
                  adaptive_cfg, arm_cfg, step_size, goal_threshold, run_sc_rrt, finalize_path, pp_cfg):
    chain, labels = stage_chain_from_case(case)
    if len(chain) < 2:
        return {"success": False, "path": [], "failure_mode": "invalid_stage_chain"}

    segment_records = []
    raw_segments = []
    segment_envs = []
    total_time = 0.0
    total_nodes = 0
    spheres = None
    if case.get("obstacle_set"):
        from scene_obstacles import load_scene_spheres
        spheres = load_scene_spheres(case["obstacle_set"], obstacles_config)

    env_last = None
    for seg_i in range(len(chain) - 1):
        p0, p1 = chain[seg_i], chain[seg_i + 1]
        env = env_for_segment(case, obstacles_config, p0, p1, step_size, goal_threshold, safety_margin)
        env["case"] = case
        env_last = env
        checker = None
        if arm_cfg and arm_cfg.get("enabled"):
            attach_arm_checker_to_env(env, arm_cfg, spheres=spheres)
            checker = env.get("arm_segment_checker")
            if checker is not None and seg_i == 0:
                checker.reset_seed()

        seg_result = run_sc_rrt(env, max_iterations, safety_margin, adaptive_cfg or {})
        seg_path = seg_result.get("path") or []
        seg_ok = bool(seg_result.get("success") and seg_path)
        seg_time = float(seg_result.get("planning_time", 0.0))
        seg_nodes = int(seg_result.get("tree_nodes", 0))
        total_time += seg_time
        total_nodes += seg_nodes
        seg_len = 0.0
        if seg_ok and len(seg_path) >= 2:
            pts = [np.asarray(p, dtype=float) for p in seg_path]
            seg_len = float(sum(np.linalg.norm(pts[j + 1] - pts[j]) for j in range(len(pts) - 1)))
        segment_records.append({
            "index": seg_i, "from_label": labels[seg_i], "to_label": labels[seg_i + 1],
            "success": seg_ok, "raw_n": len(seg_path),
            "planning_time_s": round(seg_time, 6), "path_length_m": round(seg_len, 6),
            "tree_nodes": seg_nodes,
            "failure_mode": seg_result.get("failure_mode", "none" if seg_ok else "max_iterations"),
        })
        if not seg_ok:
            return {
                "success": False, "path": [], "raw_path": _concat_paths(raw_segments),
                "failure_mode": f"segment_{seg_i}_failed", "planning_mode": "multivia",
                "segments": segment_records, "num_stages": len(chain) - 1,
                "planning_time": total_time, "tree_nodes": total_nodes, "algorithm": "SC-RRT",
            }
        raw_segments.append(seg_path)
        segment_envs.append(env)
        _seed_checker_at_tcp(checker, np.asarray(seg_path[-1], dtype=float))

    full_raw = _concat_paths(raw_segments)
    # 按段后处理，避免整链 shortcut 把 multivia 绕障折线压成起终点直线
    deployed_segments = []
    for seg_path, seg_env in zip(raw_segments, segment_envs):
        if not seg_path:
            continue
        deployed_segments.append(
            finalize_path(seg_path, env_builder, pp_cfg, env=seg_env)
        )
    deployed = _concat_paths(deployed_segments)
    return {
        "success": bool(deployed), "path": deployed, "raw_path": full_raw, "deployed_path": deployed,
        "failure_mode": "none" if deployed else "postprocess_empty", "planning_mode": "multivia",
        "segments": segment_records, "num_stages": len(chain) - 1,
        "planning_time": total_time, "tree_nodes": total_nodes,
        "path_length": sum(r["path_length_m"] for r in segment_records), "algorithm": "SC-RRT",
    }


def stage_tcp_metadata(case: dict) -> List[dict]:
    chain, labels = stage_chain_from_case(case)
    return [{"position": pt.tolist(), "label": lab} for pt, lab in zip(chain, labels)]
