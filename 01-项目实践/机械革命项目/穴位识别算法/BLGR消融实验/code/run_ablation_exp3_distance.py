"""
run_ablation_exp3_distance.py — 消融实验三：拍摄距离鲁棒性验证

验证目标：证明 BLGR 骨度比例计算具有尺度不变性，
          误差不随相机到受试者的距离增大而显著增加。

实验设置：
    - 固定同一批受试者（建议中等体型，3~5人）
    - 三个拍摄距离：1.0 m / 1.5 m / 2.0 m
    - 计算各距离段的 Mean Error 和 SR

运行前准备：
    1. data/keypoints.json      — YOLO 提取的关键点
    2. data/gt.csv              — GT 标注（格式同 Exp1）
    3. data/distance_meta.csv   — 每帧对应的拍摄距离（image_id, distance_m）
    4. acupoints_config.json

执行：python run_ablation_exp3_distance.py
"""

import os
import sys
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from blgr_core import load_acupoints_config
from ablation_variants import compute_back_acupoints_full
from metrics import (
    compute_frame_errors,
    compute_batch_metrics,
    estimate_pixel_to_cm,
    load_gt_from_csv,
    load_keypoints_from_json,
)

# ==================== 路径配置 ====================
KEYPOINTS_JSON    = "./data/keypoints.json"
GT_CSV            = "./data/gt.csv"
DISTANCE_META_CSV = "./data/distance_meta.csv"
ACUCONFIG_JSON    = "./acupoints_config.json"
OUTPUT_DIR        = "./results"

TARGET_CODES = [
    "GV14", "GV12", "GV11", "GV8", "GV4",
    "BL13R", "BL15R", "BL17R", "BL23R",
]

SUCCESS_THRESHOLD_CM = 1.5
DISTANCE_LABELS = ["1.0m", "1.5m", "2.0m"]


def load_distance_meta(csv_path):
    """
    加载距离元数据。

    CSV 列：image_id, distance_m
    distance_m 为字符串，如 "1.0" / "1.5" / "2.0"，无日期字段。

    Returns:
        {image_id: "1.0m" / "1.5m" / "2.0m", ...}
    """
    meta = {}
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            dist_key = f"{float(row['distance_m']):.1f}m"
            meta[row["image_id"]] = dist_key
    return meta


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("[Exp3] 加载数据...")
    kp_data   = load_keypoints_from_json(KEYPOINTS_JSON)
    gt_data   = load_gt_from_csv(GT_CSV)
    dist_meta = load_distance_meta(DISTANCE_META_CSV)
    acuconfig = load_acupoints_config(ACUCONFIG_JSON)

    group_errors = {label: [] for label in DISTANCE_LABELS}
    all_errors   = []

    common_ids = sorted(set(kp_data.keys()) & set(gt_data.keys()) & set(dist_meta.keys()))
    print(f"[Exp3] 有效测试帧数：{len(common_ids)}")

    for img_id in common_ids:
        dist_label = dist_meta[img_id]
        if dist_label not in group_errors:
            print(f"[WARN] 未知距离标签 '{dist_label}'，跳过 {img_id}")
            continue

        keypoints = kp_data[img_id]
        gt_frame  = gt_data[img_id]

        neck  = keypoints.get("neck",      [0, 0, 0])
        lhip  = keypoints.get("left_hip",  [0, 0, 0])
        rhip  = keypoints.get("right_hip", [0, 0, 0])
        hip_mid_est = [(lhip[0]+rhip[0])/2, (lhip[1]+rhip[1])/2, 0.9]
        px_to_cm = estimate_pixel_to_cm(neck, hip_mid_est)

        preds  = compute_back_acupoints_full(keypoints, acuconfig)
        errors = compute_frame_errors(
            preds, gt_frame,
            pixel_to_cm=px_to_cm,
            target_codes=TARGET_CODES,
        )
        group_errors[dist_label].append(errors)
        all_errors.append(errors)

    _print_distance_table(group_errors, all_errors)

    out_csv = os.path.join(OUTPUT_DIR, "exp3_distance_table.csv")
    _save_distance_csv(group_errors, all_errors, out_csv)
    print(f"\n[Exp3] 结果已保存至 {out_csv}")


def _print_distance_table(group_errors, all_errors):
    col_w = 12
    header = (f"{'Distance':<{col_w}}"
              f"{'N':>6} {'Mean (cm)':>12} {'Std (cm)':>10} "
              f"{'SR@1.5cm':>10}")
    sep = "=" * len(header)
    print(f"\n{sep}")
    print("Table 3: Distance Robustness (AMP + BLGR, Ours)")
    print(sep)
    print(header)
    print("-" * len(header))

    for label in [*DISTANCE_LABELS, "Overall"]:
        errors_list = all_errors if label == "Overall" else group_errors.get(label, [])
        stats = compute_batch_metrics(errors_list, SUCCESS_THRESHOLD_CM)
        print(f"{label:<{col_w}}"
              f"{stats['n_samples']:>6}"
              f"{stats['mean_error_cm']:>12.3f}"
              f"{stats['std_cm']:>10.3f}"
              f"{stats['sr']*100:>9.1f}%")
    print(sep)


def _save_distance_csv(group_errors, all_errors, out_path):
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Distance", "N", "Mean Error (cm)", "Std (cm)",
                         f"SR@{SUCCESS_THRESHOLD_CM}cm"])
        for label in [*DISTANCE_LABELS, "Overall"]:
            errors_list = all_errors if label == "Overall" else group_errors.get(label, [])
            stats = compute_batch_metrics(errors_list, SUCCESS_THRESHOLD_CM)
            writer.writerow([
                label,
                stats["n_samples"],
                f"{stats['mean_error_cm']:.3f}",
                f"{stats['std_cm']:.3f}",
                f"{stats['sr']*100:.1f}%",
            ])


if __name__ == "__main__":
    main()
