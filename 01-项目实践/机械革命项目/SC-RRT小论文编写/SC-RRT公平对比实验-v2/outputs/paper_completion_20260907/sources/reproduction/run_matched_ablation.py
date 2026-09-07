#!/usr/bin/env python3
"""同批顺序执行完整 SC-RRT 和等启动时刻固定参数对照，每对交替顺序。"""
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
from environments import environment_from_dict
from matched_ablation import build_matched


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    config = json.loads((ROOT / "config/protocol.json").read_text())
    output = ROOT / "results" / ("matched_smoke" if args.smoke else "formal_matched_ssfor")
    output.mkdir(exist_ok=True)
    manifest = {
        "protocol": config,
        "hashes": {str(p.relative_to(ROOT)): sha(p) for p in [
            ROOT / "src/core.py", ROOT / "src/planners.py", ROOT / "src/matched_ablation.py",
            ROOT / "run_matched_ablation.py", ROOT / "data/environments/evaluation_2d.json",
            ROOT / "data/environments/evaluation_3d.json",
        ]},
        "purpose": "Equal activation condition: finite best path and at least 51 feedback observations. Fixed gamma=1.5, p=0.8; no parameter tuning.",
        "python": sys.version, "numpy": np.__version__, "platform": platform.platform(),
        "expected_runs": 4 if args.smoke else 2000,
    }
    identity = hashlib.sha256(json.dumps(manifest, sort_keys=True).encode()).hexdigest()
    manifest_path = output / "manifest.json"
    if manifest_path.exists():
        assert json.loads(manifest_path.read_text())["identity"] == identity, "拒绝跨实现续跑"
    else:
        manifest["identity"] = identity
        manifest["started_utc"] = datetime.now(timezone.utc).isoformat()
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2))
    old = {}
    with (ROOT / "results/formal_main/runs.csv").open() as f:
        for r in csv.DictReader(f):
            if r["algorithm"] == "SC-RRT":
                old[(r["environment_id"], int(r["seed"]))] = r
    path = output / "records.jsonl"
    records = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    done = {(r["run"]["environment_id"], r["run"]["seed"], r["run"]["algorithm"]): r for r in records}
    with path.open("a", buffering=1) as file:
        for dim in (2, 3):
            maps = json.loads((ROOT / f"data/environments/evaluation_{dim}d.json").read_text())["environments"]
            for map_index, item in enumerate(maps[:1] if args.smoke else maps):
                env = environment_from_dict(item)
                for seed_index in range(1 if args.smoke else 50):
                    seed = dim * 10_000_000 + map_index * 10_000 + seed_index
                    for fixed in ([False, True] if seed_index % 2 == 0 else [True, False]):
                        name = "SC-RRT-fixed-matched" if fixed else "SC-RRT"
                        key = (env.env_id, seed, name)
                        if key in done:
                            continue
                        planner = build_matched(env, seed, config, fixed)
                        result = planner.plan()
                        run = result.as_record(env, seed)
                        run.update(map_index=map_index, seed_index=seed_index,
                                   activation_attempt=planner.activation_attempt)
                        assert run["expansion_attempts"] == 5000
                        assert not run["success"] or run["path_valid"] == 1
                        assert np.all(np.isfinite(planner.gamma + planner.p_informed))
                        if not fixed:
                            prior = old[(env.env_id, seed)]
                            assert abs(result.best_cost - float(prior["fixed_budget_cost_mm"])) < 1e-7
                            assert result.total_nodes == int(prior["total_nodes"])
                            assert result.budget.collision_checks == int(prior["collision_checks"])
                        record = {"run": run, "activation_snapshot": planner.activation_snapshot,
                                  "cost_trace": result.budget.trace,
                                  "control_trace": planner.control_trace}
                        other = done.get((env.env_id, seed, "SC-RRT" if fixed else "SC-RRT-fixed-matched"))
                        if other:
                            assert other["activation_snapshot"] == planner.activation_snapshot, "启动前状态不一致"
                            assert other["run"]["first_solution_attempt"] == run["first_solution_attempt"]
                        file.write(json.dumps(record, ensure_ascii=False) + "\n")
                        done[key] = record
                        if len(done) % 20 == 0 or args.smoke:
                            print(f"{len(done)}/{manifest['expected_runs']} {env.env_id} seed={seed_index} {name} L={result.best_cost:.3f} t={result.runtime_s:.3f}s activation={planner.activation_attempt}", flush=True)
    assert len(done) == manifest["expected_runs"]
    runs = [r["run"] for r in done.values()]
    fields = sorted(set().union(*(r.keys() for r in runs)))
    with (output / "runs.csv").open("w") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(runs)
    (output / "completion.json").write_text(json.dumps({"runs":len(done), "completed_utc":datetime.now(timezone.utc).isoformat(), "identity":identity}, indent=2))
    print("COMPLETE", output, flush=True)


if __name__ == "__main__":
    main()
