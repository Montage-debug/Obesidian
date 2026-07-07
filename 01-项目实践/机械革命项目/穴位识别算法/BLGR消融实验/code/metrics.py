"""
metrics.py — 消融实验评估指标

提供 Mean Error (cm)、Std、SR@threshold 的计算函数。

GT 格式（与数据采集协议保持一致）：
    每帧 GT 为 dict：{acupointCode: [x_cm, y_cm]}
    坐标单位为物理距离（cm），由中医师或解剖学参考标注。

    当仅有像素坐标时，通过 pixel_to_cm() 换算后使用。
"""

import numpy as np
import csv
import json
from typing import List, Dict, Optional


# ==================== 单帧误差计算 ====================

def compute_frame_errors(
    pred_acpoints: List,
    gt_dict: Dict[str, List[float]],
    pixel_to_cm: float = 1.0,
    target_codes: Optional[List[str]] = None,
) -> Dict[str, float]:
    """
    计算单帧中各穴位的预测误差（cm）。

    Args:
        pred_acpoints: [(acupointCode, name_zh, [px, py]), ...]
        gt_dict:       {acupointCode: [x, y]}，坐标与预测坐标同单位
        pixel_to_cm:   像素到 cm 的换算系数（gt 若为像素坐标则传入此值）
        target_codes:  仅统计这些穴位编码，None 表示全部

    Returns:
        {acupointCode: error_cm, ...}
    """
    errors = {}
    pred_map = {code: coord for code, _, coord in pred_acpoints}

    for code, gt_coord in gt_dict.items():
        if target_codes is not None and code not in target_codes:
            continue
        if code not in pred_map:
            continue

        pred = np.array(pred_map[code][:2], dtype=np.float64)
        gt   = np.array(gt_coord[:2], dtype=np.float64)

        # 欧氏距离（像素），乘换算系数得到 cm 误差
        dist_px = np.linalg.norm(pred - gt)
        errors[code] = float(dist_px * pixel_to_cm)

    return errors


# ==================== 批量统计 ====================

def compute_batch_metrics(
    all_errors: List[Dict[str, float]],
    success_threshold_cm: float = 1.5,
) -> Dict[str, float]:
    """
    对多帧误差列表进行汇总统计。

    Args:
        all_errors: [frame1_errors_dict, frame2_errors_dict, ...]
        success_threshold_cm: SR 判定阈值（cm），默认 1.5 cm

    Returns:
        {
            "mean_error_cm": float,
            "std_cm":        float,
            "median_cm":     float,
            "sr":            float,   # Success Rate，0~1
            "n_samples":     int,
        }
    """
    flat_errors = []
    for frame_err in all_errors:
        flat_errors.extend(frame_err.values())

    if len(flat_errors) == 0:
        return {
            "mean_error_cm": float("nan"),
            "std_cm":        float("nan"),
            "median_cm":     float("nan"),
            "sr":            float("nan"),
            "n_samples":     0,
        }

    arr = np.array(flat_errors, dtype=np.float64)
    success_count = int(np.sum(arr <= success_threshold_cm))

    return {
        "mean_error_cm": float(np.mean(arr)),
        "std_cm":        float(np.std(arr)),
        "median_cm":     float(np.median(arr)),
        "sr":            float(success_count / len(arr)),
        "n_samples":     len(arr),
    }


# ==================== 多变体对比 ====================

def compare_variants(
    variant_errors: Dict[str, List[Dict[str, float]]],
    threshold_cm: float = 1.5,
) -> Dict[str, Dict[str, float]]:
    """
    对多个变体各自的批量误差汇总，生成消融对比数据。

    Args:
        variant_errors: {
            "Direct Projection": [frame1_errors, ...],
            "AMP":               [...],
            "AMP + BLGR":        [...],
        }

    Returns:
        {variant_name: {"mean_error_cm": ..., "std_cm": ..., "sr": ..., "n_samples": ...}, ...}
    """
    return {
        name: compute_batch_metrics(errors_list, threshold_cm)
        for name, errors_list in variant_errors.items()
    }


def print_ablation_table(summary: Dict[str, Dict[str, float]], threshold_cm: float = 1.5):
    """
    打印 SCI 论文格式的消融对比表格（Terminal 输出）。
    """
    col_w = 25
    header = (
        f"{'Model':<{col_w}}"
        f"{'Mean Error (cm)':>18}"
        f"{'Std (cm)':>12}"
        f"{'SR@{:.1f}cm'.format(threshold_cm):>12}"
        f"{'N':>8}"
    )
    sep = "=" * len(header)
    print(f"\n{sep}")
    print("Table 1: Ablation Study of BLGR Components")
    print(sep)
    print(header)
    print("-" * len(header))

    for variant_name, stats in summary.items():
        row = (
            f"{variant_name:<{col_w}}"
            f"{stats['mean_error_cm']:>18.3f}"
            f"{stats['std_cm']:>12.3f}"
            f"{stats['sr'] * 100:>11.1f}%"
            f"{stats['n_samples']:>8}"
        )
        print(row)

    print(sep)


# ==================== 像素→cm 换算 ====================

def estimate_pixel_to_cm(
    neck_kp: List[float],
    hip_kp: List[float],
    back_length_cm: float = 50.0,
) -> float:
    """
    利用颈椎到髋中点的像素距离估算像素/cm 换算系数。

    依据：成年人颈椎到髋中点约 50 cm（可根据受试者实测修正）。

    Args:
        neck_kp:        颈椎关键点 [x, y, conf]
        hip_kp:         髋中点 [x, y, conf]
        back_length_cm: 物理背长（cm），默认 50 cm

    Returns:
        cm_per_pixel: float
    """
    neck = np.array(neck_kp[:2], dtype=np.float64)
    hip  = np.array(hip_kp[:2], dtype=np.float64)
    dist_px = np.linalg.norm(hip - neck)
    if dist_px < 1.0:
        return 1.0
    return back_length_cm / dist_px


# ==================== GT 与关键点文件加载 ====================

def load_gt_from_csv(csv_path: str) -> Dict[str, Dict[str, List[float]]]:
    """
    从 CSV 加载 GT 标注数据。

    CSV 列：image_id, acupointCode, x_px, y_px, x_cm, y_cm

    Returns:
        {image_id: {acupointCode: [x_cm, y_cm], ...}, ...}
    """
    gt_data: Dict[str, Dict[str, List[float]]] = {}
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            img_id = row["image_id"]
            code   = row["acupointCode"]
            x_cm   = float(row["x_cm"])
            y_cm   = float(row["y_cm"])
            gt_data.setdefault(img_id, {})[code] = [x_cm, y_cm]
    return gt_data


def load_keypoints_from_json(json_path: str) -> Dict[str, Dict]:
    """
    从 JSON 加载 YOLO 预提取的关键点数据。

    JSON 结构：
    {
      "img_001": {
        "left_shoulder":  [x, y, conf],
        "right_shoulder": [x, y, conf],
        "left_hip":       [x, y, conf],
        "right_hip":      [x, y, conf],
        "neck":           [x, y, conf]
      },
      ...
    }
    """
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)
