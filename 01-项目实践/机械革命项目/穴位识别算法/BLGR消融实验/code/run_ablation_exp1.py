"""
run_ablation_exp1.py — 消融实验一：BLGR 贡献消融

验证目标：证明 BLGR 双尺度骨度几何推理对穴位定位精度的核心贡献。

运行前准备：
    1. 将 YOLO 推理后的关键点保存为 JSON（见 docs/数据采集协议.md）
    2. 将中医师标注 GT 保存为 CSV
    3. 将 acupoints_config.json 复制到 code/ 目录下
    4. 执行：python run_ablation_exp1.py

输出：
    - 终端打印 Table 1（三行对比）
    - 保存 results/exp1_ablation_table.csv

测试穴位集（背部代表性，覆盖中线+旁开、上背+下背）：
    GV14、GV12、GV11、GV8、GV4
    BL13R、BL15R、BL17R、BL23R
"""

import os
import sys
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from blgr_core import load_acupoints_config
from ablation_variants import run_all_variants, VARIANTS
from metrics import (
    compute_frame_errors,
    compare_variants,
    print_ablation_table,
    estimate_pixel_to_cm,
    load_gt_from_csv,
    load_keypoints_from_json,
)

# ==================== 路径配置（填入你的实际路径） ====================
KEYPOINTS_JSON   = "./data/keypoints.json"
GT_CSV           = "./data/gt.csv"
ACUCONFIG_JSON   = "./acupoints_config.json"
OUTPUT_DIR       = "./results"

# 评估穴位集（GV 中线 + BL 右侧旁开，代表上中下三段）
TARGET_CODES = [
    "GV14", "GV12", "GV11", "GV8", "GV4",
    "BL13R", "BL15R", "BL17R", "BL23R",
]

SUCCESS_THRESHOLD_CM = 1.5


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print("[Exp1] 加载关键点数据和 GT 标注...")
    kp_data   = load_keypoints_from_json(KEYPOINTS_JSON)
    gt_data   = load_gt_from_csv(GT_CSV)
    acuconfig = load_acupoints_config(ACUCONFIG_JSON)

    variant_errors = {name: [] for name in VARIANTS}
    common_ids = sorted(set(kp_data.keys()) & set(gt_data.keys()))
    print(f"[Exp1] 有效测试帧数：{len(common_ids)}")

    for img_id in common_ids:
        keypoints = kp_data[img_id]
        gt_frame  = gt_data[img_id]

        # 估算像素到 cm 的换算系数（利用颈椎到髋中点的物理距离）
        neck  = keypoints.get("neck",      [0, 0, 0])
        lhip  = keypoints.get("left_hip",  [0, 0, 0])
        rhip  = keypoints.get("right_hip", [0, 0, 0])
        hip_mid_est = [(lhip[0]+rhip[0])/2, (lhip[1]+rhip[1])/2, 0.9]
        px_to_cm = estimate_pixel_to_cm(neck, hip_mid_est)

        # 运行三个变体并统计误差
        variant_preds = run_all_variants(keypoints, acuconfig)
        for name, preds in variant_preds.items():
            errors = compute_frame_errors(
                preds, gt_frame,
                pixel_to_cm=px_to_cm,
                target_codes=TARGET_CODES,
            )
            variant_errors[name].append(errors)

    # 汇总并打印表格
    summary = compare_variants(variant_errors, threshold_cm=SUCCESS_THRESHOLD_CM)
    print_ablation_table(summary, threshold_cm=SUCCESS_THRESHOLD_CM)

    # 保存为 CSV
    out_csv = os.path.join(OUTPUT_DIR, "exp1_ablation_table.csv")
    _save_summary_csv(summary, out_csv)
    print(f"\n[Exp1] 结果已保存至 {out_csv}")


def _save_summary_csv(summary, out_path):
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Model", "Mean Error (cm)", "Std (cm)",
                         f"SR@{SUCCESS_THRESHOLD_CM}cm", "N"])
        for variant_name, stats in summary.items():
            writer.writerow([
                variant_name,
                f"{stats['mean_error_cm']:.3f}",
                f"{stats['std_cm']:.3f}",
                f"{stats['sr']*100:.1f}%",
                stats["n_samples"],
            ])


if __name__ == "__main__":
    main()
