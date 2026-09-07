#!/usr/bin/env python3
"""执行配对随机种子的六算法主实验或 SC-RRT 消融实验。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "src"))

from environments import ensure_environment_files  # noqa: E402
from planners import build_planner  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", choices=("smoke", "calibration", "formal", "ablation"), default="smoke")
    parser.add_argument("--dimensions", default="2,3")
    parser.add_argument("--max-attempts", type=int)
    parser.add_argument("--maps-per-dim", type=int)
    parser.add_argument("--seeds-per-map", type=int)
    parser.add_argument("--algorithms", help="逗号分隔；缺省按 suite 选择")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--progress-every", type=int, default=1)
    return parser.parse_args()


RUN_FIELDS = [
    "environment_id", "family", "dimension", "map_index", "seed_index", "seed",
    "algorithm", "success", "budget_limit", "expansion_attempts", "added_nodes",
    "total_nodes", "collision_checks", "collision_rejections", "rewire_attempts",
    "rewire_successes", "first_solution_attempt", "first_solution_time_s",
    "first_solution_nodes", "first_solution_cost_mm", "first_solution_efficiency",
    "fixed_budget_time_s", "fixed_budget_cost_mm", "fixed_budget_efficiency",
    "path_valid", "pareto_start_index", "gamma_a_final", "gamma_b_final",
    "p_a_final", "p_b_final", "feedback_updates", "pid_improvement_ema_final",
]
TRACE_FIELDS = [
    "environment_id", "family", "dimension", "map_index", "seed_index", "seed",
    "algorithm", "checkpoint", "best_cost_mm",
]


def completed_keys(path: Path) -> set[tuple[str, int, str]]:
    if not path.exists():
        return set()
    with path.open(newline="", encoding="utf-8") as handle:
        return {
            (row["environment_id"], int(row["seed"]), row["algorithm"])
            for row in csv.DictReader(handle)
        }


def open_incremental_writer(path: Path, fields: list[str]):
    exists = path.exists() and path.stat().st_size > 0
    handle = path.open("a", newline="", encoding="utf-8")
    writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
    if not exists:
        writer.writeheader()
        handle.flush()
    return handle, writer


def main() -> int:
    args = parse_args()
    config_path = ROOT / "config" / "protocol.json"
    config_bytes = config_path.read_bytes()
    config = json.loads(config_bytes)
    if args.max_attempts:
        config["max_extension_attempts"] = args.max_attempts
        config["checkpoints"] = [x for x in config["checkpoints"] if x <= args.max_attempts]
        if args.max_attempts not in config["checkpoints"]:
            config["checkpoints"].append(args.max_attempts)

    all_environments = ensure_environment_files(ROOT, config)
    dims = [int(value) for value in args.dimensions.split(",")]
    if args.algorithms:
        algorithms = [value.strip() for value in args.algorithms.split(",") if value.strip()]
    elif args.suite == "ablation":
        algorithms = ["SC-RRT", *config["ablation_algorithms"]]
    else:
        algorithms = list(config["main_algorithms"])

    if args.suite == "smoke":
        split, map_count, seed_count = "calibration", 1, 1
    elif args.suite == "calibration":
        split = "calibration"
        map_count = int(config["calibration_maps_per_dimension"])
        seed_count = int(config["calibration_seeds_per_map"])
    else:
        split = "evaluation"
        map_count = int(config["evaluation_maps_per_dimension"])
        seed_count = int(config["planner_seeds_per_map"])
    map_count = args.maps_per_dim or map_count
    seed_count = args.seeds_per_map or seed_count

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or ROOT / "results" / f"{args.suite}_{stamp}"
    output.mkdir(parents=True, exist_ok=True)
    total = len(dims) * map_count * seed_count * len(algorithms)
    run_path = output / "runs.csv"
    trace_path = output / "traces.csv"
    done = completed_keys(run_path)
    completed = len(done)
    run_handle, run_writer = open_incremental_writer(run_path, RUN_FIELDS)
    trace_handle, trace_writer = open_incremental_writer(trace_path, TRACE_FIELDS)
    try:
        for dim in dims:
            environments = all_environments[(split, dim)][:map_count]
            if len(environments) < map_count:
                raise ValueError(f"{dim}D 只有 {len(environments)} 张地图，不足 {map_count} 张")
            for map_index, env in enumerate(environments):
                for seed_index in range(seed_count):
                    paired_seed = dim * 10_000_000 + map_index * 10_000 + seed_index
                    offset = (map_index * seed_count + seed_index) % len(algorithms)
                    ordered = algorithms[offset:] + algorithms[:offset]
                    for algorithm in ordered:
                        key = (env.env_id, paired_seed, algorithm)
                        if key in done:
                            continue
                        planner = build_planner(algorithm, env, paired_seed, config)
                        result = planner.plan()
                        record = result.as_record(env, paired_seed)
                        record["map_index"] = map_index
                        record["seed_index"] = seed_index
                        if result.budget.attempts != int(config["max_extension_attempts"]):
                            raise AssertionError(f"{algorithm} 未用满预算")
                        if result.success and not record["path_valid"]:
                            raise AssertionError(f"{algorithm} 返回无效路径")
                        run_writer.writerow(record)
                        for checkpoint, cost in sorted(result.budget.trace.items()):
                            trace_writer.writerow({
                                "environment_id": env.env_id,
                                "family": env.family,
                                "dimension": dim,
                                "map_index": map_index,
                                "seed_index": seed_index,
                                "seed": paired_seed,
                                "algorithm": algorithm,
                                "checkpoint": checkpoint,
                                "best_cost_mm": cost if np.isfinite(cost) else np.nan,
                            })
                        run_handle.flush()
                        trace_handle.flush()
                        done.add(key)
                        completed += 1
                        if completed % max(1, args.progress_every) == 0 or completed == total:
                            print(
                                f"[{completed:04d}/{total:04d}] {env.env_id} seed={seed_index:02d} "
                                f"{algorithm}: success={result.success} cost={result.best_cost:.3f} "
                                f"time={result.runtime_s:.3f}s",
                                flush=True,
                            )
    finally:
        run_handle.close()
        trace_handle.close()

    metadata = {
        "suite": args.suite,
        "started_from_protocol_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "effective_config": config,
        "dimensions": dims,
        "algorithms": algorithms,
        "map_count_per_dimension": map_count,
        "seeds_per_map": seed_count,
        "run_count": completed,
        "completed_at_utc": datetime.now(timezone.utc).isoformat(),
        "python": sys.version,
        "platform": platform.platform(),
        "numpy": np.__version__,
    }
    (output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"完成：{output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
