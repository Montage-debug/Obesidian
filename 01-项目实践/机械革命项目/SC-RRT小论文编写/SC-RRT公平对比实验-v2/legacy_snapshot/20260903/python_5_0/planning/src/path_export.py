#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TCP 路径 JSON 导出（migration_v1 代表路径 schema）。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, List, Sequence


def poses_to_json(path: Sequence) -> list[dict]:
    """笛卡尔路点列表 → poses 字段。"""
    out = []
    for p in path:
        if hasattr(p, "tolist"):
            pos = p.tolist()
        else:
            pos = list(p)
        out.append({"position": pos})
    return out


def export_representative_path(
    out_dir: Path,
    case_id: int,
    algorithm: str,
    case_meta: dict,
    run_id: int,
    planning_time_s: float,
    deployed_path: list,
    raw_path: list | None = None,
    planning_mode: str = "single",
    stage_tcp: list | None = None,
    segments: list | None = None,
    num_stages: int = 1,
) -> Path:
    """写入 case_XX_sc_rrt.json；poses 与 deployed 一致供 RViz/Gazebo 读取。"""
    out_dir.mkdir(parents=True, exist_ok=True)
    safe_name = algorithm.replace("*", "star").replace("-", "_").lower()
    out_file = out_dir / f"case_{case_id:02d}_{safe_name}.json"
    deployed = deployed_path or []
    raw = raw_path if raw_path is not None else deployed
    data: dict[str, Any] = {
        "case_id": case_id,
        "scene_id": case_meta.get("scene_id", ""),
        "obstacle_set": case_meta.get("obstacle_set", ""),
        "algorithm": algorithm,
        "frame": "base_link",
        "run_id": int(run_id),
        "planning_time_s": round(float(planning_time_s), 6),
        "poses": poses_to_json(deployed),
        "deployed_poses": poses_to_json(deployed),
        "raw_poses": poses_to_json(raw),
        "planning_mode": planning_mode,
        "num_stages": int(num_stages),
    }
    if stage_tcp:
        data["stage_tcp"] = stage_tcp
    if segments:
        data["segments"] = segments
    with open(out_file, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return out_file


def _poses_field(data: dict, key: str) -> list[list[float]]:
    return [list(p["position"]) for p in (data.get(key) or []) if p.get("position")]


def load_playback_poses(data: dict) -> list[list[float]]:
    """RViz/动画回放用 TCP 路点；multivia 若 deployed 过短则回退 raw_poses。"""
    deployed = _poses_field(data, "deployed_poses") or _poses_field(data, "poses")
    raw = _poses_field(data, "raw_poses")
    stage_tcp = data.get("stage_tcp") or []
    if data.get("planning_mode") == "multivia" and stage_tcp:
        min_len = max(len(raw), len(stage_tcp))
        if len(deployed) < min_len and raw:
            return raw
    return deployed if deployed else raw


def load_deployed_poses(path_json: Path) -> list[list[float]]:
    """从扁平或嵌套 JSON 读取部署路径坐标。"""
    data = json.loads(path_json.read_text(encoding="utf-8"))
    if "poses" in data or "deployed_poses" in data or "raw_poses" in data:
        poses = load_playback_poses(data)
        if poses:
            return poses
    if "paths" in data and "sc_rrt" in data["paths"]:
        poses = data["paths"]["sc_rrt"].get("poses", [])
        return [p["position"] for p in poses]
    raise KeyError(f"no path in {path_json}")
