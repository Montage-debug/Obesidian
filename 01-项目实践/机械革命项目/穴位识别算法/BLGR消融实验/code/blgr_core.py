"""
blgr_core.py — BLGR 骨度引导局部几何路由 核心几何引擎（独立版）

从生产代码 draw_point.py 中提取的纯几何计算逻辑，去除 YOLO/OpenVINO/ROS 依赖，
供消融实验脚本直接调用。

输入：关键点字典（由 YOLO 推理结果预先序列化为 JSON）
输出：穴位坐标列表 [(acupointCode, name_zh, [x, y]), ...]

对应论文公式：
    式(2)  双尺度骨度  λ_l = |H-N|/22, λ_w = min(L_sh/8.5, L_hip/6)
    式(3)  纵向原点    P_s = N + 4·λ_l·d_0
    式(5)  统一映射    P = P_s + δ_l·λ_l·d + δ_w·λ_w·n
    式(8)  镜像对称    P' = 2F - P
"""

import numpy as np
import json
import os


# ==================== 骨度常数 ====================
CUN_OF_MIDLINE = 22           # 颈椎到尾椎骨度（寸）
CUN_SHOULDER_TO_MIDLINE = 8.5 # 肩端到肩中点横向骨度（寸）
CUN_HIP_TO_MIDLINE = 6        # 髋端到髋中点横向骨度（寸）

# 关键点检测失败时使用的默认模板
DEFAULT_BACK_KEYPOINTS = {
    "left_shoulder":  [110.0, 201.0, 0.9],
    "right_shoulder": [290.0, 201.0, 0.9],
    "left_hip":       [136.0, 473.0, 0.9],
    "right_hip":      [264.0, 473.0, 0.9],
    "neck":           [200.0, 156.0, 0.9],
}


# ==================== 基础几何原语（对应论文式1） ====================

def calculate_acupoints_along_baseline(
    start_point, end_point, pixels_per_cun,
    offset_cun=3.0, direction="vertical", base_point="start"
):
    """
    沿基线方向（或其法向）计算偏移后的穴位像素坐标。

    对应论文式(1)：D = S - δ·λ·u
    direction="right" 时为横向偏移，对应式(2)的 λ_w·n 分量。

    Args:
        start_point:    基线起点 [x, y, ...]
        end_point:      基线终点 [x, y, ...]
        pixels_per_cun: 每寸像素数（λ_l 或 λ_w）
        offset_cun:     偏移寸数，>0 为向头侧（反方向），<0 为向尾侧
        direction:      "vertical"=沿基线方向, "right"=沿右法向
        base_point:     "start"=以起点为基准, "end"=以终点为基准

    Returns:
        [x, y] 穴位像素坐标（float list）
    """
    S = np.array(start_point[:2], dtype=np.float64)
    H = np.array(end_point[:2], dtype=np.float64)

    midline_vector = H - S
    if direction == "right":
        # 右法向：将方向向量旋转 -90°（图像坐标系）
        midline_vector = np.array([midline_vector[1], -midline_vector[0]])

    norm = np.linalg.norm(midline_vector)
    if norm < 1e-6:
        raise ValueError("基线起点与终点距离过近，无法确定方向")

    unit_vector = midline_vector / norm
    offset_pixels = offset_cun * pixels_per_cun

    if direction == "vertical":
        base = S if base_point == "start" else H
        D = base - unit_vector * offset_pixels
    else:  # right
        base = S if base_point == "start" else H
        D = base + unit_vector * offset_pixels

    return D.tolist()


# ==================== 镜像对称（对应论文式8） ====================

def mirror_right_acupoints_to_left(acpoints_to_draw, axis_start, axis_end):
    """
    将右侧穴位沿体轴（axis_start → axis_end）对称到左侧。

    对应论文式(8)：P' = 2F - P，F 为 P 在对称轴上的投影点。

    Args:
        acpoints_to_draw: [(acupointCode, name_zh, [x, y]), ...]
        axis_start: 对称轴起点（P_s / back_start_point）
        axis_end:   对称轴终点（hip_mid）

    Returns:
        包含原右侧穴位和镜像左侧穴位的完整列表
    """
    mirrored = acpoints_to_draw.copy()
    S = np.array(axis_start[:2], dtype=np.float64)
    H = np.array(axis_end[:2], dtype=np.float64)
    midline_vec = H - S
    midline_len_sq = np.dot(midline_vec, midline_vec)

    if midline_len_sq < 1e-6:
        return mirrored

    for acupointCode, acupoint_name, coord in acpoints_to_draw:
        # 只镜像以 R/R1/R2/R3 结尾的右侧穴位
        is_right = any(acupointCode.endswith(s) for s in ('R1', 'R2', 'R3', 'R'))
        if not is_right:
            continue

        P = np.array(coord[:2], dtype=np.float64)
        SP = P - S
        t = np.dot(SP, midline_vec) / midline_len_sq
        foot = S + t * midline_vec
        P_mirrored = (2 * foot - P).tolist()

        # 将穴位编码末尾 R → L
        for suffix in ('R1', 'R2', 'R3', 'R'):
            if acupointCode.endswith(suffix):
                left_code = acupointCode[:-len(suffix)] + suffix.replace('R', 'L')
                break

        mirrored.append((left_code, acupoint_name, P_mirrored))

    return mirrored


# ==================== 双尺度骨度（对应论文式2、3） ====================

def compute_dual_scale(neck, shoulder_mid, right_shoulder, hip_mid, right_hip):
    """
    计算背部双尺度骨度参数，对应论文式(2)(3)。

        λ_l = |H - N| / 22
        λ_w = min(L_sh / 8.5, L_hip / 6)
        P_s = N + 4·λ_l·d_0   (纵向原点，对应 back_start_point)

    Args:
        neck:           颈椎关键点 [x, y, conf]
        shoulder_mid:   肩中点 [x, y, conf]
        right_shoulder: 右肩关键点 [x, y, conf]
        hip_mid:        髋中点 [x, y, conf]
        right_hip:      右髋关键点 [x, y, conf]

    Returns:
        ppc_l:              纵向尺度 λ_l（pixel/cun）
        ppc_w:              横向尺度 λ_w（pixel/cun）
        back_start_point:   纵向原点 P_s [x, y]
    """
    neck_arr  = np.array(neck[:2], dtype=np.float64)
    hip_arr   = np.array(hip_mid[:2], dtype=np.float64)
    sh_mid    = np.array(shoulder_mid[:2], dtype=np.float64)
    r_sh      = np.array(right_shoulder[:2], dtype=np.float64)
    r_hip     = np.array(right_hip[:2], dtype=np.float64)

    # 纵向尺度
    dist_neck_hip = np.linalg.norm(hip_arr - neck_arr)
    ppc_l = dist_neck_hip / CUN_OF_MIDLINE

    # 横向尺度（取肩宽与胯宽中的较小值，防体型差异失调）
    ppc_s = np.linalg.norm(r_sh - sh_mid) / CUN_SHOULDER_TO_MIDLINE
    ppc_h = np.linalg.norm(r_hip - hip_arr) / CUN_HIP_TO_MIDLINE
    ppc_w = min(ppc_s, ppc_h)

    # 纵向原点 P_s（颈椎向下 4 寸）
    back_start_point = calculate_acupoints_along_baseline(
        neck_arr, hip_arr, ppc_l, offset_cun=-4.0
    )

    # 尺度钳制：防裤腰遮挡导致 λ_l 偏小
    if ppc_l / ppc_s <= 1.28:
        ppc_l = ppc_s * 1.28

    return ppc_l, ppc_w, back_start_point


def build_back_baselines(back_start_point, hip_mid, ppc_w):
    """
    基于 P_s 和 hip_mid 计算所有旁开基线的起止点。

    旁开距离：1.5 / 0.75 / 3.0 / 4.5 / 7.0 / 6.0 寸，对应 right_start1~6。
    """
    offsets = {1: 1.5, 2: 0.75, 3: 3.0, 4: 4.5, 5: 7.0, 6: 6.0}
    baselines = {
        "back_start_point": back_start_point,
        "hip_mid": hip_mid,
    }
    for idx, cun in offsets.items():
        baselines[f"right_start{idx}"] = calculate_acupoints_along_baseline(
            back_start_point, hip_mid, ppc_w, offset_cun=cun, direction="right"
        )
        baselines[f"right_end{idx}"] = calculate_acupoints_along_baseline(
            back_start_point, hip_mid, ppc_w, offset_cun=cun,
            direction="right", base_point="end"
        )
    return baselines


# ==================== 完整 BLGR 背部穴位计算（对应论文式5） ====================

def compute_back_acupoints_blgr(keypoints, acupoints_config):
    """
    完整 BLGR 流水线：双尺度骨度 + 区域路由 + 镜像对称。

    这是消融实验"Ours（AMP + BLGR）"行的计算函数。

    Args:
        keypoints: dict，键包含 left_shoulder / right_shoulder /
                   left_hip / right_hip / neck，值为 [x, y, conf]
        acupoints_config: 从 acupoints_config.json 加载的规则库 dict

    Returns:
        acpoints: [(acupointCode, name_zh, [x, y]), ...]（含左右两侧）
    """
    kp = keypoints_from_dict(keypoints)
    left_shoulder  = kp["left_shoulder"]
    right_shoulder = kp["right_shoulder"]
    left_hip       = kp["left_hip"]
    right_hip      = kp["right_hip"]
    neck           = kp["neck"]

    # 计算中点
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

    # 双尺度计算（BLGR 核心）
    ppc_l, ppc_w, back_start_point = compute_dual_scale(
        neck, shoulder_mid, right_shoulder, hip_mid, right_hip
    )

    # 构建旁开基线
    baselines = build_back_baselines(back_start_point, hip_mid, ppc_w)

    acpoints = []

    # 常规穴位遍历
    for ap in acupoints_config.get("acupoints", []):
        sp_key = ap.get("start_point", "back_start_point")
        ep_key = ap.get("end_point", "hip_mid")
        if sp_key not in baselines or ep_key not in baselines:
            continue
        coord = calculate_acupoints_along_baseline(
            baselines[sp_key], baselines[ep_key], ppc_l, ap["offset_cun"]
        )
        acpoints.append((ap["acupointCode"], ap.get("name_zh", ""), coord))

    # 特殊肩背穴位
    _append_special_acupoints(acpoints, acupoints_config, baselines, ppc_l, ppc_w)

    # 镜像对称到左侧
    acpoints = mirror_right_acupoints_to_left(acpoints, back_start_point, hip_mid)

    return acpoints


def _append_special_acupoints(acpoints, config, baselines, ppc_l, ppc_w):
    """
    处理 acupoints_special 中需要跨穴基线的肩背特殊穴（天宗、肩贞等）。
    这些穴位使用"两穴之间的连线"作为基线，是式(1)的结构化扩展。
    """
    ap_special = config.get("acupoints_special", [])
    if len(acpoints) < 42 or len(ap_special) < 8:
        return

    # 肩贞：以 acpoints[17] 和 acpoints[41] 的连线为基线
    c = calculate_acupoints_along_baseline(
        acpoints[17][2], acpoints[41][2], ppc_w, ap_special[0]["offset_cun"]
    )
    acpoints.append((ap_special[0]["acupointCode"], ap_special[0].get("name_zh", ""), c))

    # 天宗、秉风（6 寸旁开线）
    tz_s = baselines.get("right_start6")
    tz_e = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=6.0, direction="right", base_point="end"
    )
    for i in (1, 2):
        c = calculate_acupoints_along_baseline(tz_s, tz_e, ppc_l, ap_special[i]["offset_cun"])
        acpoints.append((ap_special[i]["acupointCode"], ap_special[i].get("name_zh", ""), c))

    # 肩中俞（2 寸旁开线）
    jzy_s = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=2.0, direction="right"
    )
    jzy_e = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=2.0, direction="right", base_point="end"
    )
    c = calculate_acupoints_along_baseline(jzy_s, jzy_e, ppc_l, ap_special[3]["offset_cun"])
    acpoints.append((ap_special[3]["acupointCode"], ap_special[3].get("name_zh", ""), c))

    # 臑俞（9 寸旁开线）
    ny_s = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=9.0, direction="right"
    )
    ny_e = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=9.0, direction="right", base_point="end"
    )
    c = calculate_acupoints_along_baseline(ny_s, ny_e, ppc_l, ap_special[4]["offset_cun"])
    acpoints.append((ap_special[4]["acupointCode"], ap_special[4].get("name_zh", ""), c))

    # 肩外俞（acpoints[1] → acpoints[16] 连线）
    if len(acpoints) >= 17:
        c = calculate_acupoints_along_baseline(
            acpoints[1][2], acpoints[16][2], ppc_w, ap_special[5]["offset_cun"]
        )
        acpoints.append((ap_special[5]["acupointCode"], ap_special[5].get("name_zh", ""), c))

    # 曲垣（4.5 寸旁开线）
    qy_s = baselines.get("right_start4")
    qy_e = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=4.5, direction="right", base_point="end"
    )
    c = calculate_acupoints_along_baseline(qy_s, qy_e, ppc_l, ap_special[6]["offset_cun"])
    acpoints.append((ap_special[6]["acupointCode"], ap_special[6].get("name_zh", ""), c))

    # 天髎（5.5 寸旁开线）
    tl_s = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=5.5, direction="right"
    )
    tl_e = calculate_acupoints_along_baseline(
        baselines["back_start_point"], baselines["hip_mid"],
        ppc_w, offset_cun=5.5, direction="right", base_point="end"
    )
    c = calculate_acupoints_along_baseline(tl_s, tl_e, ppc_l, ap_special[7]["offset_cun"])
    acpoints.append((ap_special[7]["acupointCode"], ap_special[7].get("name_zh", ""), c))


# ==================== 辅助函数 ====================

def load_acupoints_config(json_path):
    """加载穴位规则库 JSON 文件。"""
    with open(json_path, "r", encoding="utf-8") as f:
        return json.load(f)


def keypoints_from_dict(kp_dict):
    """
    从字典中提取背部所需关键点，缺失或置信度不足时用默认模板填充。

    Args:
        kp_dict: {"left_shoulder": [x, y, conf], ...}

    Returns:
        包含所有必要关键点的 dict
    """
    required = ["left_shoulder", "right_shoulder", "left_hip", "right_hip", "neck"]
    result = {}
    for key in required:
        val = kp_dict.get(key)
        if val is not None and len(val) >= 3 and val[2] >= 0.3:
            result[key] = val
        else:
            result[key] = DEFAULT_BACK_KEYPOINTS.get(key, [0.0, 0.0, 0.0])
    return result
