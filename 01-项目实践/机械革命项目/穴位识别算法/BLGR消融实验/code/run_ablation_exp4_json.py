"""
run_ablation_exp4_json.py — 消融实验四：JSON 规则库扩展性验证

验证目标：证明系统具备零样本扩展能力（Scalability）。
          仅通过在 JSON 中新增 5 个穴位规则条目，不重新训练任何模型，
          直接运行现有几何引擎即可完成新穴位的定位。

新增的 5 个穴位（来自 acupoints_config_ext.json）：
    GV15     哑门    — 督脉，GV14 上方 0.5 寸
    GV16     风府    — 督脉，GV14 上方约 1 寸
    BL10_R   天柱右  — 膀胱经，GV16 同水平旁开 1.5 寸
    EX-B1_R  定喘右  — 经外奇穴，GV14 旁开 0.75 寸
    EX-B4_R  腰眼右  — 经外奇穴，第 4 腰椎棘突下旁 3 寸

运行前准备：
    1. data/keypoints.json        — YOLO 提取的关键点
    2. data/gt_ext.csv            — 新增 5 穴的 GT 标注
    3. acupoints_config_ext.json  — 已在本目录，无需修改

执行：python run_ablation_exp4_json.py
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
GT_EXT_CSV     = "./data/gt_ext.csv"
ACUCONFIG_EXT  = "./acupoints_config_ext.json"
OUTPUT_DIR     = "./results"

# 仅统计 5 个新增穴位
NEW_ACUPOINT_CODES = ["GV15", "GV16", "BL10_R", "EX-B1_R", "EX-B4_R"]

NEW_ACUPOINT_NAMES = {
    "GV15":    "哑门",
    "GV16":    "风府",
    "BL10_R":  "天柱（右）",
    "EX-B1_R": "定喘（右）",
    "EX-B4_R": "腰眼（右）",
}

SUCCESS_THRESHOLD_CM = 1.5


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("[Exp4] 加载数据（扩展规则库）...")
    kp_data       = load_keypoints_from_json(KEYPOINTS_JSON)
    gt_data       = load_gt_from_csv(GT_EXT_CSV)
    acuconfig_ext = load_acupoints_config(ACUCONFIG_EXT)

    print(f"[Exp4] 扩展规则库共 {len(acuconfig_ext['acupoints'])} 条规则")
    print(f"[Exp4] 待验证新穴位：{NEW_ACUPOINT_CODES}")

    # 按穴位收集各帧误差
    per_ap_errors = {code: [] for code in NEW_ACUPOINT_CODES}
    all_errors    = []

    common_ids = sorted(set(kp_data.keys()) & set(gt_data.keys()))
    print(f"[Exp4] 有效测试帧数：{len(common_ids)}")

    for img_id in common_ids:
        keypoints = kp_data[img_id]
        gt_frame  = gt_data[img_id]

        neck  = keypoints.get("neck",      [0, 0, 0])
        lhip  = keypoints.get("left_hip",  [0, 0, 0])
        rhip  = keypoints.get("right_hip", [0, 0, 0])
        hip_mid_est = [(lhip[0]+rhip[0])/2, (lhip[1]+rhip[1])/2, 0.9]
        px_to_cm = estimate_pixel_to_cm(neck, hip_mid_est)

        # 使用扩展规则库（只改了 JSON，模型不变）
        preds  = compute_back_acupoints_full(keypoints, acuconfig_ext)
        errors = compute_frame_errors(
            preds, gt_frame,
            pixel_to_cm=px_to_cm,
            target_codes=NEW_ACUPOINT_CODES,
        )

        for code, err in errors.items():
            if code in per_ap_errors:
                per_ap_errors[code].append({code: err})

        all_errors.append(errors)

    _print_ext_table(per_ap_errors, all_errors)

    out_csv = os.path.join(OUTPUT_DIR, "exp4_json_extensibility.csv")
    _save_ext_csv(per_ap_errors, all_errors, out_csv)
    print(f"\n[Exp4] 结果已保存至 {out_csv}")


def _print_ext_table(per_ap_errors, all_errors):
    col_w = 20
    header = (f"{'Acupoint (New)':<{col_w}}"
              f"{'Chinese':<12}"
              f"{'JSON Only':>10}"
              f"{'N':>6}"
              f"{'Mean (cm)':>12}"
              f"{'SR@1.5cm':>10}")
    sep = "=" * len(header)
    print(f"\n{sep}")
    print("Table 4: JSON Rule Library Extensibility (Zero-Shot, No Retraining)")
    print(sep)
    print(header)
    print("-" * len(header))

    for code in NEW_ACUPOINT_CODES:
        frame_errors = per_ap_errors[code]
        stats = compute_batch_metrics(frame_errors, SUCCESS_THRESHOLD_CM)
        print(f"{code:<{col_w}}"
              f"{NEW_ACUPOINT_NAMES.get(code, ''):<12}"
              f"{'√':>10}"
              f"{stats['n_samples']:>6}"
              f"{stats['mean_error_cm']:>12.3f}"
              f"{stats['sr']*100:>9.1f}%")

    overall = compute_batch_metrics(all_errors, SUCCESS_THRESHOLD_CM)
    print(f"{'Average':<{col_w}}"
          f"{'—':<12}"
          f"{'√':>10}"
          f"{overall['n_samples']:>6}"
          f"{overall['mean_error_cm']:>12.3f}"
          f"{overall['sr']*100:>9.1f}%")
    print(sep)


def _save_ext_csv(per_ap_errors, all_errors, out_path):
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Acupoint (New)", "Chinese", "JSON Only", "N",
                         "Mean Error (cm)", f"SR@{SUCCESS_THRESHOLD_CM}cm"])
        for code in NEW_ACUPOINT_CODES:
            frame_errors = per_ap_errors[code]
            stats = compute_batch_metrics(frame_errors, SUCCESS_THRESHOLD_CM)
            writer.writerow([
                code,
                NEW_ACUPOINT_NAMES.get(code, ""),
                "√",
                stats["n_samples"],
                f"{stats['mean_error_cm']:.3f}",
                f"{stats['sr']*100:.1f}%",
            ])
        overall = compute_batch_metrics(all_errors, SUCCESS_THRESHOLD_CM)
        writer.writerow([
            "Average", "—", "√",
            overall["n_samples"],
            f"{overall['mean_error_cm']:.3f}",
            f"{overall['sr']*100:.1f}%",
        ])


if __name__ == "__main__":
    main()
