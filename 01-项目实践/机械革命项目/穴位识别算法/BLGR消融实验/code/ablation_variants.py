"""
ablation_variants.py — 消融实验三行对比变体

实现论文 Table 1 中三行对应的穴位计算函数：
    第一行：Direct Projection Baseline（无 BLGR，固定尺度）
    第二行：AMP（完整 BLGR，无 Region Routing）
    第三行：AMP + BLGR (Ours)  ← 调用 blgr_core.compute_back_acupoints_blgr()

每个函数接受相同的 (keypoints, acupoints_config) 输入，返回相同格式：
    [(acupointCode, name_zh, [x, y]), ...]

消融设计原则：
    Baseline vs AMP：验证 BLGR 双尺度骨度几何的贡献
    AMP vs Ours：    验证 Region Routing（体区路由）的贡献
"""

import numpy as np

from blgr_core import (
    calculate_acupoints_along_baseline,
    mirror_right_acupoints_to_left,
    compute_back_acupoints_blgr,
    compute_dual_scale,
    build_back_baselines,
    keypoints_from_dict,
)


# ==================== 固定尺度常数 ====================
# 模拟"无 BLGR"场景：使用预先设定的固定像素/寸比率，
# 不依赖任何人体测量，代表直接在锚点附近固定偏移的 Baseline。
#
# 标定依据：标准成年人（中等体型），相机距离 1.5 m，
# 常见拍摄分辨率 640×480 时，颈椎到髋中点约 280 px，
# 折算 22 寸 → 约 12.7 px/cun；横向肩宽/2 约 120 px / 8.5 → 14 px/cun。
# 取保守公共值 13 px/cun 作为固定基准。
#
# 实际使用时请根据你的相机参数和平均拍摄距离重新标定此值。
FIXED_PIXELS_PER_CUN_L = 13.0   # 固定纵向尺度（px/cun）
FIXED_PIXELS_PER_CUN_W = 13.0   # 固定横向尺度（px/cun）


# ==================== 第一行：Direct Projection Baseline ====================

def compute_back_acupoints_baseline(keypoints, acupoints_config):
    """
    Baseline（无 BLGR）：使用固定 PIXELS_PER_CUN，以肩中点为纵向参考原点。

    不进行任何骨度自适应计算，直接将固定尺度应用于 JSON 规则的 offset_cun 字段。
    代表"直接在锚点附近按固定比例偏移预测穴位"的最简策略。

    Args:
        keypoints: dict，键：left_shoulder / right_shoulder / left_hip / right_hip / neck
        acupoints_config: 穴位规则库 dict

    Returns:
        [(acupointCode, name_zh, [x, y]), ...]（含左右两侧）
    """
    kp = keypoints_from_dict(keypoints)
    left_shoulder  = kp["left_shoulder"]
    right_shoulder = kp["right_shoulder"]
    left_hip       = kp["left_hip"]
    right_hip      = kp["right_hip"]

    # 肩中点和髋中点（仍需锚点作为几何参考）
    shoulder_mid = [
        (left_shoulder[0] + right_shoulder[0]) / 2,
        (left_shoulder[1] + right_shoulder[1]) / 2,
        min(left_shoulder[2], right_shoulder[2]),
    ]
    hip_mid = [
        (left_hip[0] + right_hip[0]) / 2,
        (left_hip[1] + right_hip[1]) / 2,
        min(left_hip[2], right_hip[2]),
    ]

    # Baseline 关键差异：使用 shoulder_mid 作为 P_s（无颈部 4 寸校准），
    # 使用固定尺度代替骨度自适应尺度
    back_start_point = shoulder_mid
    ppc_l = FIXED_PIXELS_PER_CUN_L
    ppc_w = FIXED_PIXELS_PER_CUN_W

    baselines = build_back_baselines(back_start_point, hip_mid, ppc_w)

    acpoints = []
    for ap in acupoints_config.get("acupoints", []):
        sp_key = ap.get("start_point", "back_start_point")
        ep_key = ap.get("end_point", "hip_mid")
        if sp_key not in baselines or ep_key not in baselines:
            continue
        coord = calculate_acupoints_along_baseline(
            baselines[sp_key], baselines[ep_key], ppc_l, ap["offset_cun"]
        )
        acpoints.append((ap["acupointCode"], ap.get("name_zh", ""), coord))

    acpoints = mirror_right_acupoints_to_left(acpoints, back_start_point, hip_mid)
    return acpoints


# ==================== 第二行：AMP（有 BLGR，无 Region Routing） ====================

def compute_back_acupoints_blgr_no_routing(keypoints, acupoints_config):
    """
    AMP 变体（有 BLGR 双尺度，无体区路由）：
    使用完整双尺度骨度计算，但不区分体区（背/腹/腿），
    仅运行背部算法（BP004），忽略 BP014/BP010 分支。

    与 Ours 的差异：去掉 chose_body_part() 区域路由，
    验证 Region Routing 对跨部位精度的额外贡献。

    Args / Returns: 格式与 compute_back_acupoints_baseline 相同
    """
    kp = keypoints_from_dict(keypoints)
    left_shoulder  = kp["left_shoulder"]
    right_shoulder = kp["right_shoulder"]
    left_hip       = kp["left_hip"]
    right_hip      = kp["right_hip"]
    neck           = kp["neck"]

    shoulder_mid = [
        (left_shoulder[0] + right_shoulder[0]) / 2,
        (left_shoulder[1] + right_shoulder[1]) / 2,
        min(left_shoulder[2], right_shoulder[2]),
    ]
    hip_mid = [
        (left_hip[0] + right_hip[0]) / 2,
        (left_hip[1] + right_hip[1]) / 2,
        min(left_hip[2], right_hip[2]),
    ]

    # 使用完整双尺度 BLGR，但不执行区域路由分发
    ppc_l, ppc_w, back_start_point = compute_dual_scale(
        neck, shoulder_mid, right_shoulder, hip_mid, right_hip
    )
    baselines = build_back_baselines(back_start_point, hip_mid, ppc_w)

    acpoints = []
    for ap in acupoints_config.get("acupoints", []):
        sp_key = ap.get("start_point", "back_start_point")
        ep_key = ap.get("end_point", "hip_mid")
        if sp_key not in baselines or ep_key not in baselines:
            continue
        coord = calculate_acupoints_along_baseline(
            baselines[sp_key], baselines[ep_key], ppc_l, ap["offset_cun"]
        )
        acpoints.append((ap["acupointCode"], ap.get("name_zh", ""), coord))

    acpoints = mirror_right_acupoints_to_left(acpoints, back_start_point, hip_mid)
    return acpoints


# ==================== 第三行：AMP + BLGR (Ours) ====================

def compute_back_acupoints_full(keypoints, acupoints_config):
    """
    完整系统（Ours）：AMP + BLGR 双尺度 + Region Routing + 镜像对称。

    直接调用 blgr_core 的完整实现，与生产系统等价（背部部分）。
    """
    return compute_back_acupoints_blgr(keypoints, acupoints_config)


# ==================== 统一调度接口 ====================

VARIANTS = {
    "Direct Projection": compute_back_acupoints_baseline,
    "AMP":               compute_back_acupoints_blgr_no_routing,
    "AMP + BLGR":        compute_back_acupoints_full,
}


def run_all_variants(keypoints, acupoints_config):
    """
    对同一组关键点运行全部三个变体，返回各变体的穴位预测结果。

    Args:
        keypoints: 单帧关键点 dict
        acupoints_config: 穴位规则库 dict

    Returns:
        {
            "Direct Projection": [(code, name, [x,y]), ...],
            "AMP":               [...],
            "AMP + BLGR":        [...],
        }
    """
    results = {}
    for name, fn in VARIANTS.items():
        try:
            results[name] = fn(keypoints, acupoints_config)
        except Exception as e:
            print(f"[WARN] 变体 '{name}' 计算失败: {e}")
            results[name] = []
    return results
