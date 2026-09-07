#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按摩机器人 TechMoveTo 空移路径规划批量对比实验。

对比算法: SC-RRT, Dynamic-RRT, Informed-RRT*, Baseline-MoveTo
"""

import argparse
import atexit
import fcntl
import json
import multiprocessing as mp
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "planning" / "src"))

from massage_robot_env import MassageRobotEnv
from metrics import compute_path_efficiency
from sc_rrt.geometry import calculate_path_smoothness, is_collision_free
from sc_rrt.sc_rrt_adaptive import SCRRTAdaptive
from baselines import BaselineMoveTo, DynamicRRT, InformedRRTStar, RRTConnect
from sc_rrt.path_postprocess import postprocess_path
from path_export import export_representative_path
from migration_multivia_planner import has_multivia, plan_multivia, stage_tcp_metadata
from arm_segment_checker import attach_arm_checker_to_env


def audit_success_path(
    env: dict,
    path: list,
    env_builder: MassageRobotEnv,
    goal_threshold: float,
    safety_margin_m: float = 0.0,
) -> str | None:
    """成功路径端点与碰撞审计；返回 failure_mode 或 None。"""
    if not path:
        return "no_path"
    pts = [np.asarray(p, dtype=float) for p in path]
    start = np.asarray(env["start_point"], dtype=float)
    goal = np.asarray(env["goal_point"], dtype=float)
    if np.linalg.norm(pts[0] - start) > goal_threshold:
        return "invalid_endpoint"
    if np.linalg.norm(pts[-1] - goal) > goal_threshold:
        return "invalid_endpoint"
    obs = MassageRobotEnv.planner_obstacles(env_builder, safety_margin_m=safety_margin_m)
    obs_mm = obs.copy()
    if obs_mm.size > 0:
        obs_mm[:, :3] *= 1000.0
        obs_mm[:, 3] *= 1000.0
    cw = env.get("collision_world")
    for i in range(len(pts) - 1):
        p1 = pts[i] * 1000.0
        p2 = pts[i + 1] * 1000.0
        if cw is not None and cw.boxes:
            if not cw.segment_free_m(p1, p2, obs_mm, 3, 1000.0):
                return "collision_on_path"
        elif obs_mm.size > 0 and not is_collision_free(p1, p2, obs_mm, 3):
            return "collision_on_path"
    checker = env.get("arm_segment_checker")
    if checker is not None and len(pts) >= 2:
        checker.reset_seed()
        for i in range(len(pts) - 1):
            if not checker.segment_free(pts[i], pts[i + 1]):
                return "arm_collision_on_path"
    return None


def resolve_suite(cfg: dict, suite_name: str) -> tuple[list, Path | None, int, str]:
    """按套件名解析案例列表、障碍配置、运行次数与输出标签。"""
    suites = cfg.get("suites", {})
    suite = suites.get(suite_name, {})
    if not suite:
        return MassageRobotEnv.load_cases(), None, int(cfg["experiment"].get("num_runs", 50)), ""

    if not suite.get("cases_file"):
        raise ValueError(f"suite '{suite_name}' 缺少 cases_file")

    cases_path = ROOT / suite["cases_file"]
    with open(cases_path, "r", encoding="utf-8") as handle:
        cases = json.load(handle)["cases"]
    indices = suite.get("case_indices")
    if indices:
        cases = [cases[i] for i in indices if i < len(cases)]
    elif suite.get("num_cases"):
        cases = cases[: int(suite["num_cases"])]

    obs_cfg = suite.get("obstacles_config")
    obstacles_config = ROOT / obs_cfg if obs_cfg else None
    if obstacles_config and not obstacles_config.exists():
        raise FileNotFoundError(f"obstacles_config 不存在: {obstacles_config}")
    num_runs = int(suite.get("num_runs", cfg["experiment"].get("num_runs", 50)))
    return cases, obstacles_config, num_runs, suite_name


def _algorithm_worker(algo_fn, env, max_iter, case_id, case, queue):
    """子进程执行单次规划，便于对全部算法施加同一墙钟上限。"""
    try:
        queue.put(("ok", algo_fn(env, max_iter, case_id=case_id, case=case)))
    except Exception as exc:
        queue.put(("error", repr(exc)))


def run_with_wall_timeout(algo_fn, env, max_iter, case_id, case, timeout_s):
    """统一外层墙钟限制；超时与基础设施异常使用不同 failure_mode。"""
    if timeout_s <= 0:
        return algo_fn(env, max_iter, case_id=case_id, case=case)

    ctx = mp.get_context("fork")
    queue = ctx.Queue()
    proc = ctx.Process(
        target=_algorithm_worker,
        args=(algo_fn, env, max_iter, case_id, case, queue),
    )
    proc.start()
    proc.join(timeout_s)
    if proc.is_alive():
        proc.terminate()
        proc.join(2.0)
        return {
            "success": False,
            "path": [],
            "path_length": float("inf"),
            "planning_time": timeout_s,
            "tree_nodes": 0,
            "failure_mode": "wall_clock_timeout",
            "timed_out": True,
        }
    if queue.empty():
        return {
            "success": False,
            "path": [],
            "path_length": float("inf"),
            "planning_time": 0.0,
            "tree_nodes": 0,
            "failure_mode": "infrastructure_error",
        }
    status, payload = queue.get()
    if status == "error":
        raise RuntimeError(payload)
    return payload


def make_algorithms(
    safety_margin: float,
    informed_cfg: dict | None = None,
    adaptive_cfg: dict | None = None,
    dynamic_cfg: dict | None = None,
):
    """构建同预算、同碰撞膨胀的四个主比较算法。"""
    informed_cfg = informed_cfg or {}
    adaptive_cfg = adaptive_cfg or {}
    dynamic_cfg = dynamic_cfg or {}

    def run_informed(env, max_iter, case_id=0, case=None):
        return InformedRRTStar(env, max_iter, safety_margin=safety_margin).plan()

    def run_dynamic(env, max_iter, case_id=0, case=None):
        return DynamicRRT(
            env,
            max_iter,
            gamma=dynamic_cfg.get("gamma", 4.0),
            p_informed=dynamic_cfg.get("p_informed", 0.3),
            safety_margin=safety_margin,
        ).plan()

    def sc_variant(name: str, eer: bool, cp: bool):
        def _run(env, max_iter, case_id=0, case=None):
            cfg = dict(adaptive_cfg)
            cfg["enable_eer_ssfor"] = eer
            cfg["enable_connect_prune"] = cp
            out = run_sc_rrt(env, max_iter, safety_margin, cfg)
            out["algorithm"] = name
            return out
        return _run

    return {
        "SC-RRT": sc_variant("SC-RRT", True, True),
        "SC-RRT-Original": sc_variant("SC-RRT-Original", False, False),
        "SC-RRT+EER": sc_variant("SC-RRT+EER", True, False),
        "SC-RRT+CP": sc_variant("SC-RRT+CP", False, True),
        "Dynamic-RRT": run_dynamic,
        "Informed-RRT*": run_informed,
        "RRT-Connect": lambda env, mi, case_id=0, case=None: RRTConnect(
            env, mi, safety_margin=safety_margin
        ).plan(),
        "Baseline-MoveTo": lambda env, mi, case_id=0, case=None: BaselineMoveTo(env).plan(),
    }


def make_failure_mode(result: dict, success: bool) -> str:
    """从规划结果提取 failure_mode。"""
    if result.get("timed_out"):
        return "timeout"
    mode = result.get("failure_mode")
    if mode:
        return mode
    return "none" if success else "max_iterations"


def build_record(
    timestamp: str,
    case: dict,
    case_id: int,
    run_id: int,
    algo_name: str,
    result: dict,
    raw_clearance: float,
    final_clearance: float,
    eff: float,
    raw_smoothness: float,
    final_smoothness: float,
    raw_path_length: float,
    final_path_length: float,
) -> dict:
    """组装单条 CSV 记录。"""
    success = bool(result["success"])
    record = {
        "timestamp": timestamp,
        "case_id": case_id,
        "session_id": case.get("session_id", ""),
        "run_id": run_id,
        "algorithm": algo_name,
        "success": success,
        "path_length": final_path_length,
        "raw_path_length": raw_path_length,
        "deployed_path_length": final_path_length,
        "final_path_length": final_path_length,
        "planning_time": result["planning_time"],
        "core_planning_time": result.get("core_planning_time", result["planning_time"]),
        "first_solution_time": result.get("first_solution_time", np.nan),
        "first_solution_nodes": result.get("first_solution_nodes", np.nan),
        "first_solution_iterations": result.get("first_solution_iterations", np.nan),
        "collision_check_count": result.get("collision_check_count", 0),
        "expansion_attempts": result.get("expansion_attempts", 0),
        "added_nodes": result.get("added_nodes", 0),
        "effective_extension_rate": result.get("effective_extension_rate", np.nan),
        "collision_rejection_rate": result.get("collision_rejection_rate", np.nan),
        "bound_pruned_count": result.get("bound_pruned_count", 0),
        "rewire_attempts": result.get("rewire_attempts", 0),
        "rewire_successes": result.get("rewire_successes", 0),
        "tree_nodes": result.get("tree_nodes", 0),
        "path_efficiency": eff,
        "smoothness": final_smoothness,
        "raw_smoothness": raw_smoothness,
        "final_smoothness": final_smoothness,
        "clearance": final_clearance,
        "deployed_clearance": final_clearance,
        "raw_clearance": raw_clearance,
        "final_clearance": final_clearance,
        "failure_mode": make_failure_mode(result, success),
        "transit_distance_m": case.get("transit_distance_m", 0),
        "tier": case.get("tier", ""),
        "scene_id": case.get("scene_id", ""),
        "difficulty": case.get("difficulty", ""),
        "planning_mode": result.get("planning_mode", "single"),
        "num_stages": int(result.get("num_stages", 1)),
        "multivia": result.get("planning_mode") == "multivia",
    }
    return record


def run_sc_rrt(
    env: dict,
    max_iterations: int,
    safety_margin: float = 0.0,
    adaptive_cfg: dict | None = None,
) -> dict:
    """运行 SC-RRT（毫米尺度 + 与基线相同的碰撞原语）。"""
    scale = 1000.0
    mm_env = MassageRobotEnv.to_mm_planner_env(env, scale)
    if env.get("arm_segment_checker") is not None:
        mm_env["arm_segment_checker"] = env["arm_segment_checker"]
    cfg = dict(adaptive_cfg or {})
    meet_smooth = cfg.pop("meet_smoothing_factor", 0.7)
    post_success_extra = cfg.pop("post_success_extra_iters", 0)
    enable_eer = bool(cfg.pop("enable_eer_ssfor", True))
    enable_cp = bool(cfg.pop("enable_connect_prune", True))
    bound_eps = float(cfg.pop("bound_prune_epsilon", 1e-6))
    planner = SCRRTAdaptive(
        env=mm_env,
        max_iterations=max_iterations,
        mode="adaptive",
        step_size=mm_env["step_size"],
        goal_threshold=mm_env["goal_threshold"],
        safety_margin=safety_margin * scale,
        smoothing_factor=float(meet_smooth),
        post_success_extra_iters=post_success_extra,
        enable_eer_ssfor=enable_eer,
        enable_connect_prune=enable_cp,
        bound_prune_epsilon=bound_eps,
        adaptive_config=cfg or None,
        verbose=False,
    )
    path, tree, success, metrics = planner.plan()
    result_path = MassageRobotEnv.path_to_meters(path, scale) if path is not None else []
    path_len = metrics.get("path_length", 0.0)
    if np.isfinite(path_len):
        path_len = path_len / scale
    first_nodes = metrics.get("first_solution_nodes", metrics.get("tree_nodes", 0))
    if np.isfinite(first_nodes):
        pass
    else:
        first_nodes = metrics.get("tree_nodes", 0)
    return {
        "success": bool(success),
        "path": result_path,
        "path_length": path_len if success else float("inf"),
        "planning_time": metrics.get("planning_time", 0.0),
        "core_planning_time": metrics.get("core_planning_time", metrics.get("planning_time", 0.0)),
        "tree_nodes": metrics.get("tree_nodes", 0),
        "first_solution_time": metrics.get("first_solution_time", metrics.get("convergence_time", np.inf)),
        "first_solution_nodes": first_nodes,
        "first_solution_iterations": metrics.get("first_solution_iter", np.inf),
        "collision_check_count": metrics.get("collision_check_count", 0),
        "expansion_attempts": metrics.get("expansion_attempts", 0),
        "added_nodes": metrics.get("added_nodes", 0),
        "effective_extension_rate": metrics.get("effective_extension_rate", 0.0),
        "collision_rejection_rate": metrics.get("collision_rejection_rate", 0.0),
        "bound_pruned_count": metrics.get("bound_pruned_count", 0),
        "rewire_attempts": metrics.get("rewire_attempts", 0),
        "rewire_successes": metrics.get("rewire_successes", 0),
        "path_efficiency": metrics.get("path_efficiency", 0.0),
        "algorithm": "SC-RRT",
        "failure_mode": metrics.get("failure_mode", "none" if success else "max_iterations"),
    }


def compute_clearance(env_builder: MassageRobotEnv, path: list) -> float:
    """计算路径最小间隙（CSV 列名 clearance，即 Table I 的 Min Clearance）。"""
    if not path:
        return 0.0
    return env_builder.compute_clearance(path)


def finalize_path(
    path: list,
    env_builder: MassageRobotEnv,
    pp_cfg: dict,
    env: dict | None = None,
) -> list:
    """四算法使用完全相同的可选部署后处理。"""
    if not path or not pp_cfg.get("enabled", False):
        return path
    obs = MassageRobotEnv.planner_obstacles(env_builder)
    pp = pp_cfg or {}
    segment_free = None
    checker = (env or {}).get("arm_segment_checker")
    if checker is not None:
        segment_free = checker.segment_free
    return postprocess_path(
        path, obs, dim=3,
        epsilon=pp.get("simplify_epsilon", 0.012),
        step=pp.get("resample_step", 0.028),
        clearance_margin=pp.get("clearance_margin", 0.012),
        target_clearance=pp.get("target_clearance", 0.022),
        segment_free=segment_free,
    ).tolist()


def path_length_m(path: list) -> float:
    if len(path) < 2:
        return 0.0
    pts = [np.asarray(p, dtype=float) for p in path]
    return float(sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)))


def pick_median_planning_candidate(candidates: list[dict]) -> dict | None:
    """在成功 run 中按 planning_time 选中位数条目。"""
    ok = [c for c in candidates if c.get("success") and c.get("deployed_path")]
    if not ok:
        return None
    ok.sort(key=lambda c: float(c["planning_time"]))
    return ok[len(ok) // 2]


def save_results_csv(results: list, csv_dir: Path, timestamp: str, tag: str = "") -> Path:
    """增量保存实验结果到 CSV"""
    csv_dir.mkdir(parents=True, exist_ok=True)
    tag_suffix = f"_{tag}" if tag else ""
    if tag:
        csv_path = csv_dir / f"{tag}.csv"
    else:
        csv_path = csv_dir / f"massage_robot_results_{timestamp}.csv"
    df = pd.DataFrame(results)
    df.to_csv(csv_path, index=False)
    df.to_csv(csv_dir / f"massage_robot_results{tag_suffix}.csv", index=False)
    return csv_path


def acquire_runner_lock(tag: str = "") -> None:
    """单主进程写 CSV：禁止并行启动第二个 batch runner（按 output-tag 分锁）。"""
    lock_name = (
        "massage_robot_experiment.lock"
        if not tag
        else f"massage_robot_experiment_{tag}.lock"
    )
    lock_path = ROOT / "results" / "logs" / lock_name
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_fd = open(lock_path, "w", encoding="utf-8")
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(
            f"错误: 已有实验主进程在运行（锁文件 {lock_path}）",
            file=sys.stderr,
        )
        sys.exit(2)
    lock_fd.write(str(os.getpid()))
    lock_fd.flush()

    def _release() -> None:
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        lock_fd.close()

    atexit.register(_release)


def main():
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--output-tag", type=str, default="")
    pre.add_argument("--suite", type=str, default="")
    pre_args, _ = pre.parse_known_args()
    lock_tag = pre_args.output_tag or pre_args.suite or "default"
    acquire_runner_lock(lock_tag)
    parser = argparse.ArgumentParser(description="按摩机器人路径规划对比实验")
    parser.add_argument("--cases", type=int, default=15, help="测试案例数")
    parser.add_argument("--runs", type=int, default=50, help="每案例运行次数")
    parser.add_argument("--start-case", type=int, default=0, help="从指定案例索引开始（断点续跑）")
    parser.add_argument("--resume-csv", type=str, default="", help="续跑时加载已有 CSV 记录")
    parser.add_argument("--cases-file", type=str, default="", help="案例 JSON（覆盖默认 R15）")
    parser.add_argument("--obstacles-config", type=str, default="", help="障碍 YAML（C25 压力集）")
    parser.add_argument(
        "--suite",
        type=str,
        default="migration_v1",
        help="experiment.yaml suites 键名（默认 migration_v1）",
    )
    parser.add_argument(
        "--output-tag",
        type=str,
        default="",
        help="CSV 文件名标签，如 pilot_v2",
    )
    parser.add_argument("--quick", action="store_true", help="快速测试 (5案例×10次)")
    parser.add_argument(
        "--margin-profile",
        type=str,
        default="main",
        help="experiment.margin_profiles 键名（main / off_margin）",
    )
    parser.add_argument(
        "--algorithms",
        type=str,
        default="",
        help="逗号分隔，仅运行指定算法（续跑时仅替换对应行）",
    )
    args = parser.parse_args()

    # 读取配置
    with open(ROOT / "config" / "experiment.yaml", "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    num_cases = 5 if args.quick else args.cases
    num_runs = 10 if args.quick else args.runs
    exp_cfg = cfg["experiment"]
    user_output_tag = args.output_tag or ""
    output_tag = user_output_tag
    obstacles_path: Path | None = None

    if args.suite:
        cases, obstacles_config_path, num_runs, suite_tag = resolve_suite(cfg, args.suite)
        if obstacles_config_path:
            obstacles_path = obstacles_config_path
        output_tag = user_output_tag or suite_tag
        args.output_tag = output_tag
        start_idx = max(0, int(args.start_case))
        if args.cases < 15:
            n = max(1, int(args.cases))
            cases = cases[start_idx : start_idx + n]
        elif start_idx > 0:
            cases = cases[start_idx:]
        num_cases = len(cases)
    suite_cfg = {}
    suites = cfg.get("suites", {})
    if args.suite in suites:
        suite_cfg = suites[args.suite]
        if suite_cfg.get("num_runs") and args.runs == 50:
            num_runs = int(suite_cfg["num_runs"])
        elif args.runs != 50:
            num_runs = int(args.runs)
    informed_cfg = dict(cfg.get("informed_rrt", {}))
    max_iter = int(informed_cfg.get("max_iterations", exp_cfg["max_iterations"]))
    wall_clock_timeout_s = float(exp_cfg.get("wall_clock_timeout_s", 0))
    if suite_cfg.get("wall_clock_timeout_s") is not None:
        wall_clock_timeout_s = float(suite_cfg["wall_clock_timeout_s"])
    informed_wall_s = float(
        cfg.get("informed_rrt", {}).get("wall_clock_timeout_s", wall_clock_timeout_s)
    )
    step_size = exp_cfg.get("step_size", 0.02)
    goal_threshold = exp_cfg.get("goal_threshold", 0.04)
    margin_profiles = exp_cfg.get("margin_profiles", {})
    safety_margin = float(
        margin_profiles.get(args.margin_profile, exp_cfg.get("safety_margin", 0.006))
    )
    pp_cfg = dict(exp_cfg.get("common_postprocess", exp_cfg.get("sc_rrt_postprocess", {})))
    if suite_cfg.get("common_postprocess"):
        pp_cfg.update(suite_cfg["common_postprocess"])
    adaptive_cfg = exp_cfg.get("sc_rrt_adaptive", {})
    dynamic_cfg = exp_cfg.get("dynamic_rrt", {})
    arm_cfg_global = dict(exp_cfg.get("arm_collision_in_planning", {}))
    if suite_cfg.get("arm_collision_in_planning") is False:
        arm_cfg_global["enabled"] = False

    if not args.suite:
        cases_path = Path(args.cases_file) if args.cases_file else None
        if cases_path and not cases_path.is_absolute():
            cases_path = ROOT / cases_path
        cases = MassageRobotEnv.load_cases(cases_path)
        cases = cases[args.start_case:num_cases]

        if args.obstacles_config:
            obstacles_path = Path(args.obstacles_config)
            if not obstacles_path.is_absolute():
                obstacles_path = ROOT / obstacles_path
    elif args.obstacles_config and obstacles_path is None:
        obstacles_path = Path(args.obstacles_config)
        if not obstacles_path.is_absolute():
            obstacles_path = ROOT / obstacles_path

    def env_builder_for_case(case: dict) -> MassageRobotEnv:
        """按案例加载对应障碍场景（C25 每案例独立 obstacle_set）。"""
        if case.get("obstacle_set"):
            return MassageRobotEnv(obstacles_path, case["obstacle_set"])
        if obstacles_path:
            return MassageRobotEnv(obstacles_path)
        return MassageRobotEnv()

    algorithms = make_algorithms(safety_margin, informed_cfg, adaptive_cfg, dynamic_cfg)
    suite_cfg = suites.get(args.suite, {}) if args.suite else {}
    cfg_algos = suite_cfg.get("algorithms") or cfg.get("algorithms")
    if cfg_algos and not args.algorithms:
        algorithms = {k: v for k, v in algorithms.items() if k in cfg_algos}
    if args.algorithms:
        selected = {a.strip() for a in args.algorithms.split(",") if a.strip()}
        algorithms = {k: v for k, v in algorithms.items() if k in selected}
        if not algorithms:
            print(f"错误: --algorithms 无有效算法: {args.algorithms}", file=sys.stderr)
            sys.exit(1)

    results = []
    done_keys: set[tuple] = set()
    if args.resume_csv:
        resume_path = Path(args.resume_csv)
        if resume_path.exists():
            prev = pd.read_csv(resume_path)
            results = prev.to_dict("records")
            for r in results:
                done_keys.add(
                    (r.get("case_id"), r.get("run_id"), r.get("algorithm"))
                )
            print(f"续跑: 已加载 {len(results)} 条历史记录，跳过已完成 {len(done_keys)} 组 (case,run,algo)")


    paths_dir = ROOT / "data" / "paths"
    csv_dir = ROOT / "results" / "csv"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.resume_csv and results:
        timestamp = str(results[0].get("timestamp", timestamp))

    print(f"实验开始: {len(cases)} 案例 × {num_runs} 次 × {len(algorithms)} 算法 (start_case={args.start_case})")
    print(
        f"最大迭代: {max_iter}, 全算法安全裕度: {safety_margin} m, "
        f"统一墙钟上限: {wall_clock_timeout_s}s"
        + (f"（Informed-RRT*: {informed_wall_s}s）" if informed_wall_s != wall_clock_timeout_s else "")
    )

    for case in cases:
        case_id = case.get("selected_id", case.get("global_id", 0))
        env_builder = env_builder_for_case(case)
        env = MassageRobotEnv.from_case(
            case,
            obstacles_config=obstacles_path,
            step_size=step_size,
            goal_threshold=goal_threshold,
            safety_margin_m=safety_margin,
        )
        env["case"] = case
        if arm_cfg_global.get("enabled"):
            attach_arm_checker_to_env(
                env,
                arm_cfg_global,
                spheres=None,
            )

        export_candidates: dict[str, list[dict]] = {k: [] for k in algorithms}

        for algo_name, algo_fn in algorithms.items():
            for run_id in range(num_runs):
                if (case_id, run_id, algo_name) in done_keys:
                    continue
                formal_seeds = case.get("formal_seeds")
                if formal_seeds and run_id < len(formal_seeds):
                    np.random.seed(int(formal_seeds[run_id]))
                else:
                    np.random.seed(run_id * 1000 + case_id)

                try:
                    # 统一墙钟计时：plan() + 路径后处理 + 指标计算（全算法同口径）
                    t_wall0 = time.perf_counter()
                    algo_timeout = (
                        informed_wall_s if algo_name == "Informed-RRT*" else wall_clock_timeout_s
                    )
                    if has_multivia(case) and algo_name == "SC-RRT":
                        n_seg = len(case.get("via_points") or []) + 1
                        algo_timeout = max(float(algo_timeout), float(wall_clock_timeout_s) * n_seg)
                    if has_multivia(case) and algo_name == "SC-RRT":
                        def _run_multivia(_env, _mi, case_id=None, case=None, **_kw):
                            return plan_multivia(
                                case, env_builder, obstacles_path, max_iter, safety_margin,
                                adaptive_cfg, arm_cfg_global, step_size, goal_threshold,
                                run_sc_rrt, finalize_path, pp_cfg,
                            )
                        result = run_with_wall_timeout(
                            _run_multivia, env, max_iter, case_id, case, algo_timeout,
                        )
                    else:
                        result = run_with_wall_timeout(
                            algo_fn, env, max_iter, case_id, case, algo_timeout
                        )

                    if result.get("planning_mode") == "multivia":
                        raw_path = result.get("raw_path") or []
                        deployed_path = result.get("path") or []
                        result["path"] = deployed_path
                    else:
                        raw_path = result["path"]
                        deployed_path = finalize_path(raw_path, env_builder, pp_cfg, env=env)
                        result["path"] = deployed_path

                    raw_path_length = path_length_m(raw_path)
                    raw_clearance = compute_clearance(env_builder, raw_path)
                    raw_smoothness = (
                        float(calculate_path_smoothness(np.asarray(raw_path, dtype=float)))
                        if result["success"] and raw_path
                        else 0.0
                    )
                    if result.get("planning_mode") != "multivia":
                        pass  # deployed_path already set above for single
                    final_path_length = path_length_m(result["path"])
                    result["path_length"] = final_path_length
                    final_clearance = compute_clearance(env_builder, result["path"])

                    if result["success"] and result["path"]:
                        eff = compute_path_efficiency(
                            result["path"], env["start_point"], env["goal_point"]
                        )
                        final_smoothness = float(
                            calculate_path_smoothness(np.asarray(result["path"], dtype=float))
                        )
                    else:
                        eff = result.get("path_efficiency", 0.0)
                        final_smoothness = 0.0

                    result["planning_time"] = time.perf_counter() - t_wall0

                    if result["success"] and raw_path:
                        audit_fail = audit_success_path(
                            env, raw_path, env_builder, goal_threshold, safety_margin,
                        )
                        if not audit_fail and result["path"]:
                            audit_fail = audit_success_path(
                                env, result["path"], env_builder, goal_threshold, safety_margin,
                            )
                        if audit_fail:
                            result["success"] = False
                            result["failure_mode"] = audit_fail

                    record = build_record(
                        timestamp, case, case_id, run_id, algo_name,
                        result, raw_clearance, final_clearance, eff,
                        raw_smoothness, final_smoothness,
                        raw_path_length, final_path_length,
                    )
                    results.append(record)
                    done_keys.add((case_id, run_id, algo_name))

                    if result.get("timed_out"):
                        print(
                            f"  超时 case={case_id} algo={algo_name} run={run_id} "
                            f"t={result['planning_time']:.1f}s"
                        )

                    if result["success"] and result["path"]:
                        export_candidates[algo_name].append(
                            {
                                "success": True,
                                "run_id": run_id,
                                "planning_time": result["planning_time"],
                                "deployed_path": result["path"],
                                "raw_path": raw_path,
                                "export_meta": {
                                    "planning_mode": result.get("planning_mode", "single"),
                                    "num_stages": result.get("num_stages", 1),
                                    "segments": result.get("segments"),
                                    "stage_tcp": stage_tcp_metadata(case)
                                    if has_multivia(case) else None,
                                },
                            }
                        )

                except Exception as e:
                    print(f"  错误 case={case_id} algo={algo_name} run={run_id}: {e}")
                    results.append({
                        "timestamp": timestamp,
                        "case_id": case_id,
                        "session_id": case.get("session_id", ""),
                        "run_id": run_id,
                        "algorithm": algo_name,
                        "success": False,
                        "path_length": 0,
                        "planning_time": 0,
                        "tree_nodes": 0,
                        "path_efficiency": 0,
                        "smoothness": 0,
                        "clearance": 0,
                        "failure_mode": "error",
                        "transit_distance_m": case.get("transit_distance_m", 0),
                        "raw_path_length": float("nan"),
                        "final_path_length": float("nan"),
                        "raw_clearance": float("nan"),
                        "final_clearance": float("nan"),
                        "raw_smoothness": float("nan"),
                        "final_smoothness": float("nan"),
                    })

        for algo_name, candidates in export_candidates.items():
            rep = pick_median_planning_candidate(candidates)
            if rep:
                meta = rep.get("export_meta") or {}
                export_representative_path(
                    paths_dir,
                    case_id,
                    algo_name,
                    case,
                    rep["run_id"],
                    rep["planning_time"],
                    rep["deployed_path"],
                    rep["raw_path"],
                    planning_mode=meta.get("planning_mode", "single"),
                    stage_tcp=meta.get("stage_tcp"),
                    segments=meta.get("segments"),
                    num_stages=meta.get("num_stages", 1),
                )

        print(f"  案例 {case_id} 完成")
        save_results_csv(results, csv_dir, timestamp, output_tag)

    tag_suffix = f"_{output_tag}" if output_tag else ""
    csv_path = save_results_csv(results, csv_dir, timestamp, output_tag)
    df = pd.DataFrame(results)

    print(f"\n实验完成: {len(results)} 条记录")
    print(f"CSV: {csv_path}")

    # 打印摘要（路径类指标仅统计成功 run）
    ok_df = df[df["success"] == True]  # noqa: E712
    summary = ok_df.groupby("algorithm").agg({
        "path_length": ["mean", "std"],
        "planning_time": ["mean", "std"],
        "clearance": ["mean", "std"],
        "path_efficiency": ["mean", "std"],
        "smoothness": ["mean", "std"],
    }).round(4)
    success_rates = df.groupby("algorithm")["success"].mean().round(4)
    print("\n成功率:")
    print(success_rates)
    print("\n摘要统计 (successful runs):")
    print(summary)


if __name__ == "__main__":
    main()
