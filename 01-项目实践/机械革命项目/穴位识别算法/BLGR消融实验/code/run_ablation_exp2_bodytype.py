"""
run_ablation_exp2_bodytype.py — 消融实验二：跨体型泛化验证

验证目标：证明 BLGR 双尺度骨度自适应对不同体型均保持稳定精度。

体型分组（见 docs/数据采集协议.md）：
    Thin:   BMI < 18.5
    Medium: 18.5 ≤ BMI < 25
    Large:  BMI ≥ 25

运行前准备：
    1. data/keypoints.json  — YOLO 提取的关键点
    2. data/gt.csv          — GT 标注（格式同 Exp1）
    3. data/subjects.csv    — 受试者元数据，含 image_id / body_type / bmi
    4. acupoints_config.json

执行：python run_ablation_exp2_bodytype.py
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
KEYPOINTS_JSON = "./data/keypoints.json"
GT_CSV         = "./data/gt.csv"
SUBJECTS_CSV   = "./data/subjects.csv"
ACUCONFIG_JSON = "./acupoints_config.json"
OUTPUT_DIR     = "./results"

TARGET_CODES = [
    "GV14", "GV12", "GV11", "GV8", "GV4",
    "BL13R", "BL15R", "BL17R", "BL23R",
]

SUCCESS_THRESHOLD_CM = 1.5
BODY_TYPES = ["Thin", "Medium", "Large"]


def load_subjects(csv_path):
    """
    加载受试者元数据。

    CSV 列：image_id, subject_id, body_type, bmi
    body_type 需在采集后人工填写（Thin/Medium/Large）。

    Returns:
        {image_id: {"body_type": str, "bmi": float}, ...}
    """
    subjects = {}
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            subjects[row["image_id"]] = {
                "body_type": row["body_type"],
                "bmi": float(row.get("bmi", 0.0)),
            }
    return subjects


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("[Exp2] 加载数据...")
    kp_data   = load_keypoints_from_json(KEYPOINTS_JSON)
    gt_data   = load_gt_from_csv(GT_CSV)
    subjects  = load_subjects(SUBJECTS_CSV)
    acuconfig = load_acupoints_config(ACUCONFIG_JSON)

    group_errors = {bt: [] for bt in BODY_TYPES}
    all_errors   = []

    common_ids = sorted(set(kp_data.keys()) & set(gt_data.keys()) & set(subjects.keys()))
    print(f"[Exp2] 有效测试帧数：{len(common_ids)}")

    for img_id in common_ids:
        body_type = subjects[img_id]["body_type"]
        if body_type not in group_errors:
            print(f"[WARN] 未知体型标签 '{body_type}'，跳过 {img_id}")
            continue

        keypoints = kp_data[img_id]
        gt_frame  = gt_data[img_id]

        neck  = keypoints.get("neck",      [0, 0, 0])
        lhip  = keypoints.get("left_hip",  [0, 0, 0])
        rhip  = keypoints.get("right_hip", [0, 0, 0])
        hip_mid_est = [(lhip[0]+rhip[0])/2, (lhip[1]+rhip[1])/2, 0.9]
        px_to_cm = estimate_pixel_to_cm(neck, hip_mid_est)

        # 使用完整 BLGR 系统（Ours）
        preds  = compute_back_acupoints_full(keypoints, acuconfig)
        errors = compute_frame_errors(
            preds, gt_frame,
            pixel_to_cm=px_to_cm,
            target_codes=TARGET_CODES,
        )
        group_errors[body_type].append(errors)
        all_errors.append(errors)

    _print_bodytype_table(group_errors, all_errors)

    out_csv = os.path.join(OUTPUT_DIR, "exp2_bodytype_table.csv")
    _save_bodytype_csv(group_errors, all_errors, out_csv)
    print(f"\n[Exp2] 结果已保存至 {out_csv}")


def _print_bodytype_table(group_errors, all_errors):
    col_w = 12
    header = (f"{'Body Type':<{col_w}}"
              f"{'N':>6} {'Mean (cm)':>12} {'Std (cm)':>10} "
              f"{'SR@1.5cm':>10}")
    sep = "=" * len(header)
    print(f"\n{sep}")
    print("Table 2: Cross-Body-Type Generalization (AMP + BLGR, Ours)")
    print(sep)
    print(header)
    print("-" * len(header))

    for bt in [*BODY_TYPES, "Overall"]:
        errors_list = all_errors if bt == "Overall" else group_errors.get(bt, [])
        stats = compute_batch_metrics(errors_list, SUCCESS_THRESHOLD_CM)
        print(f"{bt:<{col_w}}"
              f"{stats['n_samples']:>6}"
              f"{stats['mean_error_cm']:>12.3f}"
              f"{stats['std_cm']:>10.3f}"
              f"{stats['sr']*100:>9.1f}%")
    print(sep)


def _save_bodytype_csv(group_errors, all_errors, out_path):
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Body Type", "N", "Mean Error (cm)", "Std (cm)",
                         f"SR@{SUCCESS_THRESHOLD_CM}cm"])
        for bt in [*BODY_TYPES, "Overall"]:
            errors_list = all_errors if bt == "Overall" else group_errors.get(bt, [])
            stats = compute_batch_metrics(errors_list, SUCCESS_THRESHOLD_CM)
            writer.writerow([
                bt,
                stats["n_samples"],
                f"{stats['mean_error_cm']:.3f}",
                f"{stats['std_cm']:.3f}",
                f"{stats['sr']*100:.1f}%",
            ])


if __name__ == "__main__":
    main()
