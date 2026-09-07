#!/usr/bin/env python3
"""生成首解/固定预算统计、配对检验和收敛图。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent
ALGORITHM_ORDER = ["RRT", "RRT-Connect", "RRT*", "Informed-RRT*", "Dynamic-RRT", "SC-RRT"]


def wilson(successes: int, count: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if count == 0:
        return float("nan"), float("nan")
    proportion = successes / count
    denominator = 1 + z * z / count
    center = (proportion + z * z / (2 * count)) / denominator
    half = z * math.sqrt(proportion * (1 - proportion) / count + z * z / (4 * count * count)) / denominator
    return center - half, center + half


def describe_runs(data: pd.DataFrame) -> pd.DataFrame:
    metrics = [
        "first_solution_attempt", "first_solution_time_s", "first_solution_nodes",
        "first_solution_cost_mm", "first_solution_efficiency", "fixed_budget_time_s",
        "fixed_budget_cost_mm", "fixed_budget_efficiency", "total_nodes", "collision_checks",
    ]
    rows = []
    for (dimension, algorithm), group in data.groupby(["dimension", "algorithm"], sort=False):
        successes = int(group["success"].sum())
        low, high = wilson(successes, len(group))
        row = {
            "dimension": dimension,
            "algorithm": algorithm,
            "n": len(group),
            "success_rate": successes / len(group),
            "success_ci95_low": low,
            "success_ci95_high": high,
        }
        for metric in metrics:
            values = pd.to_numeric(group[metric], errors="coerce").replace([np.inf, -np.inf], np.nan).dropna()
            row[f"{metric}_mean"] = values.mean()
            row[f"{metric}_std"] = values.std(ddof=1)
            row[f"{metric}_median"] = values.median()
            row[f"{metric}_q1"] = values.quantile(0.25)
            row[f"{metric}_q3"] = values.quantile(0.75)
        rows.append(row)
    return pd.DataFrame(rows)


def signed_rank_p(differences: np.ndarray) -> float:
    values = np.asarray(differences, dtype=float)
    values = values[np.isfinite(values) & (np.abs(values) > 1e-12)]
    n = len(values)
    if n == 0:
        return 1.0
    ranks = pd.Series(np.abs(values)).rank(method="average").to_numpy()
    w_plus = float(ranks[values > 0].sum())
    mean = n * (n + 1) / 4.0
    _, counts = np.unique(np.abs(values), return_counts=True)
    tie_correction = float(np.sum(counts**3 - counts))
    variance = (n * (n + 1) * (2 * n + 1) - tie_correction / 2.0) / 24.0
    if variance <= 0:
        return 1.0
    z_score = abs(w_plus - mean) / math.sqrt(variance)
    return math.erfc(z_score / math.sqrt(2.0))


def paired_bootstrap_ci(differences: np.ndarray, seed: int, repetitions: int = 10_000) -> tuple[float, float]:
    values = np.asarray(differences, dtype=float)
    values = values[np.isfinite(values)]
    if len(values) == 0:
        return float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    means = np.empty(repetitions)
    for begin in range(0, repetitions, 500):
        size = min(500, repetitions - begin)
        samples = rng.choice(values, size=(size, len(values)), replace=True)
        means[begin : begin + size] = samples.mean(axis=1)
    return tuple(np.quantile(means, [0.025, 0.975]))


def holm_adjust(p_values: list[float]) -> list[float]:
    count = len(p_values)
    order = np.argsort(p_values)
    adjusted = np.empty(count)
    running = 0.0
    for rank, index in enumerate(order):
        running = max(running, (count - rank) * p_values[index])
        adjusted[index] = min(1.0, running)
    return adjusted.tolist()


def paired_comparisons(data: pd.DataFrame, references: list[str]) -> pd.DataFrame:
    keys = ["dimension", "environment_id", "seed"]
    metrics = ["first_solution_attempt", "first_solution_time_s", "fixed_budget_cost_mm", "fixed_budget_time_s"]
    rows = []
    for dimension in sorted(data["dimension"].unique()):
        subset = data[data["dimension"] == dimension]
        sc = subset[subset["algorithm"] == "SC-RRT"].set_index(keys)
        for reference in references:
            other = subset[subset["algorithm"] == reference].set_index(keys)
            joined = sc.join(other, how="inner", lsuffix="_sc", rsuffix="_reference")
            for metric_index, metric in enumerate(metrics):
                paired = joined[[f"{metric}_sc", f"{metric}_reference"]].replace([np.inf, -np.inf], np.nan).dropna()
                difference = paired[f"{metric}_reference"].to_numpy() - paired[f"{metric}_sc"].to_numpy()
                low, high = paired_bootstrap_ci(difference, int(dimension) * 1000 + metric_index)
                denominator = paired[f"{metric}_reference"].mean()
                rows.append({
                    "dimension": dimension,
                    "reference": reference,
                    "metric": metric,
                    "paired_n": len(paired),
                    "mean_reference_minus_sc": np.mean(difference) if len(difference) else np.nan,
                    "mean_improvement_percent": 100 * np.mean(difference) / denominator if denominator else np.nan,
                    "bootstrap_ci95_low": low,
                    "bootstrap_ci95_high": high,
                    "wilcoxon_p_raw": signed_rank_p(difference),
                })
    result = pd.DataFrame(rows)
    result["wilcoxon_p_holm"] = np.nan
    for (_, metric), indexes in result.groupby(["dimension", "metric"]).groups.items():
        result.loc[indexes, "wilcoxon_p_holm"] = holm_adjust(result.loc[indexes, "wilcoxon_p_raw"].tolist())
    return result


def compact_table(summary: pd.DataFrame, algorithms: list[str]) -> str:
    lines = []
    for dimension in sorted(summary["dimension"].unique()):
        lines.extend([f"## {dimension}D", "", "| Algorithm | Success | First attempt | First time (s) | First cost (mm) | Fixed cost (mm) | Fixed time (s) | Nodes | Efficiency |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|"])
        block = summary[summary["dimension"] == dimension].set_index("algorithm")
        for algorithm in algorithms:
            if algorithm not in block.index:
                continue
            row = block.loc[algorithm]
            pm = " ± "
            lines.append(
                f"| {algorithm} | {100*row.success_rate:.1f}% | "
                f"{row.first_solution_attempt_mean:.1f}{pm}{row.first_solution_attempt_std:.1f} | "
                f"{row.first_solution_time_s_mean:.4f}{pm}{row.first_solution_time_s_std:.4f} | "
                f"{row.first_solution_cost_mm_mean:.2f}{pm}{row.first_solution_cost_mm_std:.2f} | "
                f"{row.fixed_budget_cost_mm_mean:.2f}{pm}{row.fixed_budget_cost_mm_std:.2f} | "
                f"{row.fixed_budget_time_s_mean:.4f}{pm}{row.fixed_budget_time_s_std:.4f} | "
                f"{row.total_nodes_mean:.1f}{pm}{row.total_nodes_std:.1f} | "
                f"{row.fixed_budget_efficiency_mean:.4f}{pm}{row.fixed_budget_efficiency_std:.4f} |"
            )
        lines.append("")
    return "\n".join(lines)


def plot_convergence(trace_path: Path, output: Path) -> None:
    data = pd.read_csv(trace_path)
    config = json.loads((ROOT / "config" / "protocol.json").read_text(encoding="utf-8"))
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 4.2), constrained_layout=True)
    colors = plt.cm.tab10(np.linspace(0, 1, len(ALGORITHM_ORDER)))
    for axis, dimension in zip(axes, (2, 3)):
        spec = config["dimensions"][str(dimension)]
        straight = np.linalg.norm(np.asarray(spec["goal_mm"]) - np.asarray(spec["start_mm"]))
        block = data[data["dimension"] == dimension].copy()
        block["efficiency"] = straight / block["best_cost_mm"]
        block["efficiency"] = block["efficiency"].fillna(0.0)
        for color, algorithm in zip(colors, ALGORITHM_ORDER):
            values = block[block["algorithm"] == algorithm]
            grouped = values.groupby("checkpoint")["efficiency"]
            x = np.asarray(sorted(values["checkpoint"].unique()))
            mean = grouped.mean().reindex(x).to_numpy()
            sem = grouped.sem().reindex(x).fillna(0.0).to_numpy()
            axis.plot(x, mean, label=algorithm, color=color, linewidth=1.7)
            axis.fill_between(x, mean - 1.96 * sem, mean + 1.96 * sem, color=color, alpha=0.10)
        axis.set_title(f"{dimension}D")
        axis.set_xlabel("Atomic extension attempts")
        axis.set_ylabel("Mean path efficiency (failure = 0)")
        axis.grid(alpha=0.25)
        axis.set_xlim(left=0)
        axis.set_ylim(0, 1.02)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="outside lower center", ncol=3, frameon=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--main", type=Path, required=True)
    parser.add_argument("--ablation", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    main_data = pd.read_csv(args.main / "runs.csv")
    summary = describe_runs(main_data)
    summary.to_csv(args.output / "main_summary.csv", index=False)
    comparisons = paired_comparisons(main_data, [name for name in ALGORITHM_ORDER if name != "SC-RRT"])
    comparisons.to_csv(args.output / "main_paired_comparisons.csv", index=False)
    (args.output / "main_table.md").write_text(compact_table(summary, ALGORITHM_ORDER), encoding="utf-8")
    plot_convergence(args.main / "traces.csv", args.output / "main_convergence.png")

    if args.ablation:
        ablation_data = pd.read_csv(args.ablation / "runs.csv")
        ablation_summary = describe_runs(ablation_data)
        ablation_summary.to_csv(args.output / "ablation_summary.csv", index=False)
        ablation_order = ["SC-RRT", "SC-RRT-no-ADCS", "SC-RRT-no-SSFOR"]
        comparisons = paired_comparisons(ablation_data, ablation_order[1:])
        comparisons.to_csv(args.output / "ablation_paired_comparisons.csv", index=False)
        (args.output / "ablation_table.md").write_text(compact_table(ablation_summary, ablation_order), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
