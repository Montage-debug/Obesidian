from ultralytics import YOLO
import cv2
import numpy as np
import json
import time
import os
import shutil  # 添加导入
import logging
import tempfile
import atexit
import io
import tarfile
import platform
from cryptography.fernet import Fernet, InvalidToken
from contextlib import contextmanager

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)  # 添加日志基本配置
point_radius = 2  #图片中点的半径
FERNET_KEY = b"aHd1tFp3V0PcV0ov8Aiz2BAk68WBOEB0KOwNpoKwDPk="
# 缓存解密后的 PT 临时文件路径，避免同进程重复解密
DECRYPTED_MODEL_PATH_CACHE = {}
# 缓存解密后的 RKNN 临时目录路径，避免同进程重复解密和解包
DECRYPTED_RKNN_DIR_CACHE = {}
# 缓存 YOLO 模型实例，避免重复初始化后端
YOLO_MODEL_CACHE = {}
ENABLE_VISUALIZATION = os.getenv("YOLO_ENABLE_VIS", "1") not in {"0", "false", "False"}


def cleanup_decrypted_model_files():
    # 进程结束时删除临时解密模型，避免在磁盘上残留明文权重文件
    temp_paths = list(DECRYPTED_MODEL_PATH_CACHE.values()) + list(DECRYPTED_RKNN_DIR_CACHE.values())
    for temp_path in temp_paths:
        if not os.path.exists(temp_path):
            continue
        try:
            if os.path.isdir(temp_path):
                shutil.rmtree(temp_path)
            else:
                os.remove(temp_path)
        except OSError:
            pass


atexit.register(cleanup_decrypted_model_files)

#加载加密的厚的穴位配置文件，并解密返回
def load_encrypted_json(enc_filename):
    # 部署时直接使用内嵌密钥解密，避免运行时依赖外部 key 文件
    current_dir = os.path.dirname(os.path.abspath(__file__))
    enc_path = os.path.join(current_dir, enc_filename)

    if not os.path.exists(enc_path):
        raise FileNotFoundError(f"找不到加密配置文件: {enc_path}")

    # Fernet 自带完整性校验，密钥不匹配或密文被改动都会在解密时失败
    with open(enc_path, "rb") as enc_file:
        encrypted_content = enc_file.read()

    try:
        decrypted_content = Fernet(FERNET_KEY).decrypt(encrypted_content)
    except InvalidToken as exc:
        raise ValueError(f"解密失败，请检查密钥或加密文件是否匹配: {enc_filename}") from exc

    return json.loads(decrypted_content.decode("utf-8"))


#加载加密模型解密，返回解密后的模型路径
def decrypt_model_to_temp_file(enc_model_path, model_filename):
    # 缓存临时解密结果，避免同一模型在单次进程中重复解密
    if model_filename in DECRYPTED_MODEL_PATH_CACHE:
        cached_path = DECRYPTED_MODEL_PATH_CACHE[model_filename]
        if os.path.exists(cached_path):
            return cached_path

    with open(enc_model_path, "rb") as enc_file:
        encrypted_content = enc_file.read()

    try:
        decrypted_content = Fernet(FERNET_KEY).decrypt(encrypted_content)
    except InvalidToken as exc:
        raise ValueError(f"模型解密失败，请检查密钥或加密文件是否匹配: {enc_model_path}") from exc

    suffix = os.path.splitext(model_filename)[1] or ".pt"
    with tempfile.NamedTemporaryFile(prefix="dec_model_", suffix=suffix, delete=False) as temp_file:
        temp_file.write(decrypted_content)
        temp_model_path = temp_file.name

    DECRYPTED_MODEL_PATH_CACHE[model_filename] = temp_model_path
    return temp_model_path


def decrypt_rknn_model_dir_to_temp(enc_model_path, model_stem):
    # 同一个 .enc 文件在同一进程只解密一次
    cache_key = os.path.abspath(enc_model_path)
    if cache_key in DECRYPTED_RKNN_DIR_CACHE:
        cached_dir = DECRYPTED_RKNN_DIR_CACHE[cache_key]
        if os.path.isdir(cached_dir):
            return cached_dir

    # 读取加密包并用 Fernet 解密，解密结果应为 tar.gz 字节流
    with open(enc_model_path, "rb") as enc_file:
        encrypted_content = enc_file.read()

    try:
        decrypted_content = Fernet(FERNET_KEY).decrypt(encrypted_content)
    except InvalidToken as exc:
        raise ValueError(f"RKNN 模型目录解密失败: {enc_model_path}") from exc

    # 将解密后的归档解包到临时目录，模型生命周期绑定当前进程
    temp_root = tempfile.mkdtemp(prefix=f"dec_{model_stem}_rknn_")
    with tarfile.open(fileobj=io.BytesIO(decrypted_content), mode="r:gz") as archive:
        archive.extractall(temp_root)

    # 优先按约定目录名查找；兼容历史包结构（仅一个子目录）
    expected_dir = os.path.join(temp_root, f"{model_stem}_rknn_model")
    if os.path.isdir(expected_dir):
        model_dir = expected_dir
    else:
        sub_dirs = [
            os.path.join(temp_root, entry)
            for entry in os.listdir(temp_root)
            if os.path.isdir(os.path.join(temp_root, entry))
        ]
        if len(sub_dirs) == 1:
            model_dir = sub_dirs[0]
        else:
            raise ValueError(f"解密后的 RKNN 目录结构异常: {enc_model_path}")

    # RKNN 目录完整性校验：必须有 metadata.yaml 和至少一个 .rknn 文件
    metadata_path = os.path.join(model_dir, "metadata.yaml")
    has_rknn_file = any(name.endswith(".rknn") for name in os.listdir(model_dir))
    if not os.path.isfile(metadata_path) or not has_rknn_file:
        raise ValueError(f"RKNN 目录缺少 metadata.yaml 或 .rknn 文件: {model_dir}")

    # 写入缓存并返回可直接传给 YOLO(...) 的目录
    DECRYPTED_RKNN_DIR_CACHE[cache_key] = model_dir
    return model_dir


def is_arm_architecture():
    # OrangePi 等板端通常为 arm/aarch64
    machine = platform.machine().lower()
    return machine.startswith("arm") or machine.startswith("aarch64")

def get_model_path(model_filename="x-pose.pt"):
    """获取模型文件的绝对路径"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_stem = os.path.splitext(model_filename)[0]

    # ARM 架构：强制走加密 RKNN，解密后由 RKNN Runtime 在 NPU 上执行
    if is_arm_architecture():
        print("ARM 架构，强制走加密 RKNN")
        rknn_enc_filename = f"{model_stem}_rknn_model.enc"
        rknn_enc_path = os.path.join(current_dir, rknn_enc_filename)
        if os.path.isfile(rknn_enc_path):
            return decrypt_rknn_model_dir_to_temp(rknn_enc_path, model_stem)
        raise FileNotFoundError(f"ARM 架构下找不到加密 RKNN 模型文件: {rknn_enc_filename}")

    # 非 ARM 架构：走加密 PT，便于在 x86/开发机上保持兼容
    enc_filename = f"{model_stem}.enc"
    enc_model_path = os.path.join(current_dir, enc_filename)
    if os.path.isfile(enc_model_path):
        return decrypt_model_to_temp_file(enc_model_path, model_filename)

    # 兼容历史部署目录中的加密 PT 文件
    ws_root = '/home/cjk/myproject/wlzc_massage_robot/wlzc_massage_robot_ws'
    possible_enc_paths = [
        os.path.join(ws_root, 'install/visual_recognition/lib/python3.10/site-packages/visual_recognition', enc_filename),
        os.path.join(ws_root, 'src/visual_recognition/visual_recognition', enc_filename),
    ]
    for enc_path in possible_enc_paths:
        if os.path.isfile(enc_path):
            return decrypt_model_to_temp_file(enc_path, model_filename)

    raise FileNotFoundError(f"非 ARM 架构下找不到加密 PT 模型文件: {enc_filename}")


def load_yolo_model(model_filename="models/x-pose.pt", task="pose"):
    model_source = get_model_path(model_filename)
    cache_key = (model_source, task)
    if cache_key in YOLO_MODEL_CACHE:
        return YOLO_MODEL_CACHE[cache_key]

    kwargs = {}
    if task:
        kwargs["task"] = task
    model = YOLO(model_source, **kwargs) if kwargs else YOLO(model_source)
    YOLO_MODEL_CACHE[cache_key] = model
    return model


@contextmanager
def log_time(label, store=None):
    start = time.perf_counter()
    try:
        yield
    finally:
        duration = time.perf_counter() - start
        logger.info("%s耗时 %.3fs", label, duration)
        if store is not None:
            store.append((label, duration))

#直线方向的偏移
def calculate_acupoints_along_baseline(start_point, end_point, PIXELS_PER_CUN, offset_cun=3.0 ,direction="vertical" ,base_point="start"):
    """
    沿着 起始点 到 终点 的线方向，向反方向（或垂直方向）移动 offset_cun 寸，
    计算穴位或者参考点坐标。
    
    Args:
        start_point: 起始点坐标 [x, y]
        end_point: 终点坐标 [x, y]
        PIXELS_PER_CUN: 每寸像素值abdomen_acupoints_config
        offset_cun: 偏移的寸数,默认3寸
        direction: 用于选择偏移的方向是沿着开始点于结束点的直线方向，还是沿着两点直线的右垂直方向,可选择vertical和right,默认情况就是vertical.
        base_point: 用于选择最终点的座标是根据开始点位偏移还是结束点位偏移,可选start或end
    """
    # 将输入转换为NumPy数组方便计算
    S = np.array(start_point[:2], dtype=np.float32)  # 肩中点
    H = np.array(end_point[:2], dtype=np.float32)       # 髋中点
    
    # 1. 计算中线方向向量 (从 shoulder_mid 指向 hip_mid)r
    midline_vector = H - S  # 方向向下
    if direction == "right":
        midline_vector = np.array([midline_vector[1], -midline_vector[0]])
    # 2. 计算此方向向量的单位向量 (长度为1)
    # 防止两点重合导致除零
    norm = np.linalg.norm(midline_vector)
    if norm < 1e-6:  # 如果两点几乎重合
        raise ValueError("shoulder_mid 和 hip_mid 距离太近，无法确定方向")
    
    unit_vector = midline_vector / norm
    
    # 3. 计算向上（反方向）偏移的像素量
    offset_pixels = offset_cun * PIXELS_PER_CUN
    
    # 4. 从 shoulder_mid 点，沿单位向量的反方向移动 offset_pixels
    # 反方向 = 减去单位向量 * 偏移量
    if direction == "vertical":
        if base_point == "start":
            D = S - unit_vector * offset_pixels  # 穴位坐标
        elif base_point == "end":
            D = H - unit_vector * offset_pixels  
        else :
            print("请输入正确的base_point参数")
    
    elif direction == "right":
        if base_point == "start":
            D = S + unit_vector * offset_pixels  # 穴位坐标
        elif base_point == "end":
            D = H + unit_vector * offset_pixels  
        else :
            print("请输入正确的base_point参数")
    
    return D.tolist()  # 转换为整数像素坐标

    
#得到对称穴位的列表信息
def mirror_right_acupoints_to_left(acpoints_to_draw, shoulder_mid, hip_mid):
    """
    将右侧背部的穴位对称到左侧, 以shoulder_mid到hip_mid的连线为对称轴
    
    参数:
        acpoints_to_draw: 已有的穴位列表，每个元素为(name, coord, color)
        shoulder_mid: 肩中点坐标 (x, y)
        hip_mid: 髋中点坐标 (x, y)
    
    返回:
        更新后的穴位列表，包含左侧对称穴位
    """
    # 深拷贝原列表，避免修改原数据
    mirrored_acpoints = acpoints_to_draw.copy()
    
    # 将坐标转换为numpy数组便于计算
    S = np.array(shoulder_mid[:2])
    H = np.array(hip_mid[:2])
    
    # 计算对称轴向量
    midline_vector = H - S
    midline_length = np.linalg.norm(midline_vector)
    
    if midline_length < 1e-6:
        print("警告：肩中点和髋中点距离太近，无法确定对称轴")
        return mirrored_acpoints
    
    # 计算对称轴的单位法向量（垂直方向）
    # 先计算单位方向向量
    unit_direction = midline_vector / midline_length
    
    
    # 遍历现有穴位，找到右侧穴位并对称到左侧
    for acupointCode, acupoint_name, coord in acpoints_to_draw:
        # 检查是否是右侧穴位（name以right结尾）并且不是中线上的穴位
        if acupointCode.endswith('R') or acupointCode.endswith('R1') or acupointCode.endswith('R2') or acupointCode.endswith('R3'):
            
            # 获取穴位坐标
            P = np.array(coord)  # 右侧穴位坐标
            
            # 方法1：点到直线的对称（更准确）
            # 计算穴位到直线的垂足
            SP = P - S
            t = np.dot(SP, midline_vector) / (midline_length ** 2)
            
            # 垂足坐标
            foot = S + t * midline_vector
            
            # 计算对称点：P' = foot - (P - foot) = 2*foot - P
            P_mirrored = 2 * foot - P
            
            
            # 创建左侧穴位代码（将末尾 R/R1/R2/R3 改为 L/L1/L2/L3）
            if acupointCode.endswith('R1'):
                left_acupointCode = acupointCode[:-2] + 'L1'
            elif acupointCode.endswith('R2'):
                left_acupointCode = acupointCode[:-2] + 'L2'
            elif acupointCode.endswith('R3'):
                left_acupointCode = acupointCode[:-2] + 'L3'
            else:  # endswith('R')
                left_acupointCode = acupointCode[:-1] + 'L'
            # 创建左侧穴位名称（将"右"替换为"左"）
            if acupoint_name.endswith('R'):
                left_acupoint_name = acupoint_name[:-1] + 'L'
            else:
                left_acupoint_name = acupoint_name
            # 创建左侧穴位坐标
            left_coord = P_mirrored.tolist()
            
            
            # 添加到穴位列表
            mirrored_acpoints.append((left_acupointCode, left_acupoint_name, left_coord))
            
    return mirrored_acpoints


#人体姿势是否正确判断
def person_and_pose_judge(result, body_part_code):
    #1.判断是否有人，每个部位穴位前都要检测
    if result.boxes is None or len(result.boxes) == 0:
        message = "未检测到人体"
        status_code = -1
        return False, status_code, message
    
    #2. 检查是否有姿态关键点
    if result.keypoints is None or len(result.keypoints.data) == 0:
        message = "姿态关键点检测失败"
        status_code = -2
        return False, status_code, message
    
    kpts_data = result.keypoints.data[0].cpu().numpy() 

    #检测腿部关键点信息是否正常
    if body_part_code == "BP010":
        left_hip = kpts_data[11]
        right_hip = kpts_data[12]
        left_knee = kpts_data[13]
        right_knee = kpts_data[14]
        left_ankle = kpts_data[15]
        right_ankle = kpts_data[16]

        required_kpts = [
            ("left_hip", left_hip),
            ("right_hip", right_hip),
            ("left_knee", left_knee),
            ("right_knee", right_knee),
            ("left_ankle", left_ankle),
            ("right_ankle", right_ankle),
        ]
        failed_kpts = [name for name, kpt in required_kpts if kpt[2] < 0.6]
        if len(failed_kpts) > 0:
            message = f"腿部必要关键点检测失败: {','.join(failed_kpts)}"
            status_code = -10
            return False, status_code, message

        # 基于左右髋-膝方向向量，计算双大腿横向分开角度
        # 当双大腿分开角度超过40°时，判定姿态不符合要求
        def calc_thigh_spread_angle(left_hip_point, left_knee_point, right_hip_point, right_knee_point):
            left_thigh_vec = np.array([
                left_knee_point[0] - left_hip_point[0],
                left_knee_point[1] - left_hip_point[1],
            ], dtype=np.float64)
            right_thigh_vec = np.array([
                right_knee_point[0] - right_hip_point[0],
                right_knee_point[1] - right_hip_point[1],
            ], dtype=np.float64)

            left_thigh_norm = np.linalg.norm(left_thigh_vec)
            right_thigh_norm = np.linalg.norm(right_thigh_vec)
            if left_thigh_norm < 1e-6 or right_thigh_norm < 1e-6:
                return None

            cos_theta = float(np.dot(left_thigh_vec, right_thigh_vec) / (left_thigh_norm * right_thigh_norm))
            cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
            return float(np.degrees(np.arccos(cos_theta)))

        thigh_spread_angle = calc_thigh_spread_angle(left_hip, left_knee, right_hip, right_knee)

        MAX_THIGH_SPREAD_ANGLE = 30.0
        if thigh_spread_angle > MAX_THIGH_SPREAD_ANGLE or thigh_spread_angle is None:
            message = ("双大腿分开角度过大, 或大腿夹角计算失败")
            status_code = -11
            return False, status_code, message

        # 通过髋-膝与髋-踝距离比值约束小腿姿态，避免小腿回勾到臀部附近
        left_hip_knee_distance = float(np.sqrt((left_knee[0] - left_hip[0]) ** 2 + (left_knee[1] - left_hip[1]) ** 2))
        right_hip_knee_distance = float(np.sqrt((right_knee[0] - right_hip[0]) ** 2 + (right_knee[1] - right_hip[1]) ** 2))
        left_ankle_hip_distance = float(np.sqrt((left_ankle[0] - left_hip[0]) ** 2 + (left_ankle[1] - left_hip[1]) ** 2))
        right_ankle_hip_distance = float(np.sqrt((right_ankle[0] - right_hip[0]) ** 2 + (right_ankle[1] - right_hip[1]) ** 2))

        CALF_DISTANCE_RATIO_MIN = 1.5
        left_min_ankle_hip_distance = left_hip_knee_distance * CALF_DISTANCE_RATIO_MIN
        right_min_ankle_hip_distance = right_hip_knee_distance * CALF_DISTANCE_RATIO_MIN
        if left_ankle_hip_distance <= left_min_ankle_hip_distance or right_ankle_hip_distance <= right_min_ankle_hip_distance:
            message = (
                f"小腿回勾幅度过大,请保持小腿自然伸展"
            )
            status_code = -12
            return False, status_code, message
        

        return True, 0, "检测正常"

    #以下开始腹部或背部检测
    left_eye = kpts_data[1]
    right_eye = kpts_data[2]
    left_shoulder = kpts_data[5]
    right_shoulder = kpts_data[6]
    left_hip = kpts_data[11]
    right_hip = kpts_data[12]

    #背部和腹部检测必要关键点置信度是否正常
    if left_shoulder[2] < 0.6 or right_shoulder[2] < 0.6 or left_hip[2] < 0.6 or right_hip[2] < 0.6:
        message = "必要关键点检测失败"
        print(f"置信度：{left_shoulder[2]}, {right_shoulder[2]}, {left_hip[2]}, {right_hip[2]}")
        status_code = -3
        return False, status_code, message

    #背部识别的需求，检测人是否是背面朝上
    if body_part_code == "BP004":
        if left_eye[2] >= 0.8 and right_eye[2] >= 0.8:
            message = "你的面部没有完全朝下或者没有做到背部朝上，请保持面部朝下且背面朝上"
            status_code = -4
            return False, status_code, message

    # 肩中点 (颈部点)
    shoulder_mid_x = (left_shoulder[0] + right_shoulder[0]) / 2
    shoulder_mid_y = (left_shoulder[1] + right_shoulder[1]) / 2
    shoulder_mid = np.array([shoulder_mid_x, shoulder_mid_y,
                             min(left_shoulder[2], right_shoulder[2])])  # 取较低的置信度

    # 髋中点 (腰部中心点)
    hip_mid_x = (left_hip[0] + right_hip[0]) / 2
    hip_mid_y = (left_hip[1] + right_hip[1]) / 2
    hip_mid = np.array([hip_mid_x, hip_mid_y,
                        min(left_hip[2], right_hip[2])])
    distance_pixels = np.sqrt((hip_mid_x - shoulder_mid_x)**2 +
                              (hip_mid_y - shoulder_mid_y)**2)

    #shoulder_mid到hip_mid的距离除去left_shoulder到right_shoulder的距离得到比值
    two_shoulders_distance = np.sqrt((right_shoulder[0] - left_shoulder[0])**2 + (right_shoulder[1] - left_shoulder[1])**2)
    Mline_bi_Sline = distance_pixels / two_shoulders_distance
    two_hips_distance = np.sqrt((right_hip[0] - left_hip[0])**2 + (right_hip[1] - left_hip[1])**2)
    Mline_bi_Hline = distance_pixels / two_hips_distance

    # 1. 计算肩部连线角度
    shoulder_vector = np.array([right_shoulder[0] - left_shoulder[0],
                                right_shoulder[1] - left_shoulder[1]])
    shoulder_angle = np.degrees(np.arctan2(shoulder_vector[1], shoulder_vector[0]))

    # 2. 计算髋部连线角度
    hip_vector = np.array([right_hip[0] - left_hip[0],
                           right_hip[1] - left_hip[1]])
    hip_angle = np.degrees(np.arctan2(hip_vector[1], hip_vector[0]))

    # 3. 计算身体中线角度（肩中点到髋中点）
    body_vector = np.array([hip_mid[0] - shoulder_mid[0],
                            hip_mid[1] - shoulder_mid[1]])
    body_angle = np.degrees(np.arctan2(body_vector[1], body_vector[0]))

    #疑似侧躺没躺平
    if body_part_code == "BP004":
        if Mline_bi_Sline > 2.0 or Mline_bi_Hline > 3.0:
            message = "人没完全躺平，请保证人体平躺并放松"
            status_code = -5
            return False, status_code, message
    elif body_part_code == "BP014":
        if Mline_bi_Sline > 2.5 or Mline_bi_Hline > 3.2:
            message = "人没完全躺平，请保证人体平躺并放松"
            status_code = -5
            return False, status_code, message

    #疑似人躺斜了
    if body_part_code == "BP004":
        if abs(shoulder_angle) > 6.0 or abs(hip_angle) > 4.5 or (abs(body_angle) > 95.0 or abs(body_angle) < 85.0):
            message = "人体没有躺正，请保持身体正直，双肩自然放松"
            status_code = -6
            return False, status_code, message
    elif body_part_code == "BP014":
        if abs(shoulder_angle) < 174.0 or (abs(body_angle) > 95.0 or abs(body_angle) < 85.0):
            message = "人体没有躺正，请保持身体正直，双肩自然放松"
            print(f"{shoulder_angle},{body_angle}")
            status_code = -6
            return False, status_code, message

    return True, 0, "检测正常"      


def draw_back_acupoints_on_original(img, acupoints_config, body_parrt_code, output_path="output_with_points.jpg"):
    """
    1. 使用YOLO检测单人姿态
    2. 计算左右肩、左右髋、肩中点、髋中点坐标
    3. 在原始输入图像上绘制这6个点
    """
    model = load_yolo_model("models/x-pose.pt")
    with log_time("背部-模型推理"):
        results = model(img, max_det =1, conf=0.7)
    result = results[0]

    #检测是否有人，人体于必要关键点是否符合要求
    success = True
    message = "检测正常"
    logger.info("开始判断人体是否正常")
    success, status_code, message = person_and_pose_judge(result, body_parrt_code)
    if success == False:
        return success, status_code, message, None, 0.0, 0.0, None
    
    # 获取关键点数据
    kpts_data = result.keypoints.data[0].cpu().numpy()

    # 3. 提取原始图像并获取关键点索引 (COCO 17点格式)
    # 索引: 5-左肩, 6-右肩, 11-左髋, 12-右髋
    left_shoulder = kpts_data[5]
    right_shoulder = kpts_data[6]
    left_hip = kpts_data[11]
    right_hip = kpts_data[12]
    left_wirst = kpts_data[9]
    right_wirst = kpts_data[10]
    
    # 4. 计算中点坐标
    # 肩中点 (颈部点)
    shoulder_mid_x = (left_shoulder[0] + right_shoulder[0]) / 2
    shoulder_mid_y = (left_shoulder[1] + right_shoulder[1]) / 2
    shoulder_mid = np.array([shoulder_mid_x, shoulder_mid_y, 
                             min(left_shoulder[2], right_shoulder[2])])  # 取较低的置信度
    
    # 髋中点 (腰部中心点)
    hip_mid_x = (left_hip[0] + right_hip[0]) / 2
    hip_mid_y = (left_hip[1] + right_hip[1]) / 2
    hip_mid = np.array([hip_mid_x, hip_mid_y, 
                        min(left_hip[2], right_hip[2])])  # 取较低的置信度
    #寸的基准,分为两种，分为背部的骨度法和肩部分寸法。
    #第一种背部骨度法
    CUN_SHOULDER_TO_MIDLINE = 18
    distance_pixels = np.sqrt((hip_mid_x - shoulder_mid_x)**2 + 
                            (hip_mid_y - shoulder_mid_y)**2)
    PIXELS_PER_CUN = distance_pixels / CUN_SHOULDER_TO_MIDLINE
    #第二种肩部分寸法
    CUN_SHOULDER_TO_MIDLINE_2 = 8.5
    distance_pixels_1 = np.sqrt((right_shoulder[0] - shoulder_mid_x)**2 + 
                          (right_shoulder[1] - shoulder_mid_y)**2)
    PIXELS_PER_CUN_S = distance_pixels_1 / CUN_SHOULDER_TO_MIDLINE_2

    #第三种胯寸法
    CUN_HIP_TO_MIDLINE = 6
    distance_pixels_2 = np.sqrt((right_hip[0] - hip_mid_x)**2 + 
                            (right_hip[1] - hip_mid_y)**2)
    PIXELS_PER_CUN_H = distance_pixels_2 / CUN_HIP_TO_MIDLINE

    #在肩寸法和胯寸法中取小的那个作为横向偏移的寸单位
    PIXELS_PER_CUN_3 = min(PIXELS_PER_CUN_S,PIXELS_PER_CUN_H)

    shoulder_width = 0.0
    hip_width = 0.0
    shoulder_width = float(np.sqrt((right_shoulder[0] - left_shoulder[0])**2 + 
                                   (right_shoulder[1] - left_shoulder[1])**2))
    shoulder_width = 1.0 +  int((shoulder_width-175.0)/15)*0.1

    hip_width = float(np.sqrt((right_hip[0] - left_hip[0])**2 + 
                              (right_hip[1] - left_hip[1])**2))

    #防止裤子穿过高或者腰部有遮挡导致的背部寸长失调的问题
    if PIXELS_PER_CUN/PIXELS_PER_CUN_S <= 1.28:
        PIXELS_PER_CUN=PIXELS_PER_CUN_S*1.28        

    #这里判断人的姿态是抬手的还是放平的, 如果是抬手的那吗将肩中点和胯终点下移一寸，并重新设定PIXELS_PER_CUN的大小，让其变小
    if left_wirst[1] < (hip_mid_y+shoulder_mid_y)/2 or right_wirst[1] < (hip_mid_y+shoulder_mid_y)/2:
        adjust_shoulder_mid = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN, offset_cun=-1.0)
        adjust_hip_mid = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN, offset_cun=-1.0, base_point="end")
        shoulder_mid[0:2]=adjust_shoulder_mid[0:2]
        hip_mid[0:2]=adjust_hip_mid[0:2]
        PIXELS_PER_CUN = distance_pixels / 19

    
    #计算其他的基准点
      #常规穴位基准点
    right_start1 = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=1.5, direction="right")
    right_end1 = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=1.5, direction="right",base_point="end")

    right_start2 = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=0.75, direction="right")
    right_end2 = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=0.75, direction="right",base_point="end")

    right_start3 = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=3, direction="right")
    right_end3 = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=3, direction="right",base_point="end")
     #特殊穴位基准点
    Tianzong_start = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=6, direction="right")
    Tianzong_end = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=6, direction="right", base_point="end")

    Jianzhongyu_start = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=2.0, direction="right")
    Jianzhongyu_end = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=2.0, direction="right",base_point="end")

    Naoyu_start = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=7.5, direction="right")
    Naoyu_end = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=7.5, direction="right", base_point="end")

    Quyuan_start = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=4.5, direction="right")
    Quyuan_end = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=4.5, direction="right",base_point="end")

    Tianliao_start = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=5.5, direction="right")
    Tianliao_end = calculate_acupoints_along_baseline(shoulder_mid, hip_mid, PIXELS_PER_CUN_3, offset_cun=5.5, direction="right",base_point="end")

    acpoints_to_draw = []

    
    #计算中线及右背部的穴位座标
    for ap in acupoints_config["acupoints"]:
        if ap["start_point"] == "shoulder_mid" and ap["end_point"] == "hip_mid":
            start_point = shoulder_mid
            end_point = hip_mid
        elif ap["start_point"] == "right_start1" and ap["end_point"] == "right_end1":
            start_point = right_start1
            end_point = right_end1
        elif ap["start_point"] == "right_start2" and ap["end_point"] == "right_end2":
            start_point = right_start2
            end_point =right_end2
        elif ap["start_point"] == "right_start3" and ap["end_point"] == "right_end3":
            start_point = right_start3
            end_point = right_end3
        else :
            raise ValueError("基础起始点或终点点位信息错误")

        offset_cun = ap["offset_cun"]
        acupointCode = ap["acupointCode"]
        acupoint_name = ap.get("name_zh") or ""
        acupoint_coord = calculate_acupoints_along_baseline(start_point, end_point, PIXELS_PER_CUN, offset_cun)

        acpoints_to_draw.append((acupointCode, acupoint_name, acupoint_coord))

    #以下是特殊点位处理
    ap_special = acupoints_config["acupoints_special"]
    #右肩贞
    Jianzhen_right = calculate_acupoints_along_baseline(acpoints_to_draw[17][2], acpoints_to_draw[41][2], PIXELS_PER_CUN_3, ap_special[0]["offset_cun"])
    acpoints_to_draw.append((ap_special[0]["acupointCode"], ap_special[0].get("name_zh") or "", Jianzhen_right))
    #右天宗
    Tianzong_right = calculate_acupoints_along_baseline(Tianzong_start, Tianzong_end, PIXELS_PER_CUN, ap_special[1]["offset_cun"])
    acpoints_to_draw.append((ap_special[1]["acupointCode"], ap_special[1].get("name_zh") or "", Tianzong_right))
    #右秉风
    Bingfeng_right = calculate_acupoints_along_baseline(Tianzong_start, Tianzong_end, PIXELS_PER_CUN, ap_special[2]["offset_cun"])
    acpoints_to_draw.append((ap_special[2]["acupointCode"], ap_special[2].get("name_zh") or "", Bingfeng_right))
    #右肩中俞
    Jianzhongyu_right = calculate_acupoints_along_baseline(Jianzhongyu_start, Jianzhongyu_end, PIXELS_PER_CUN, ap_special[3]["offset_cun"])
    acpoints_to_draw.append((ap_special[3]["acupointCode"], ap_special[3].get("name_zh") or "", Jianzhongyu_right))
    #右臑俞
    Naoyu_right = calculate_acupoints_along_baseline(Naoyu_start,Naoyu_end, PIXELS_PER_CUN, ap_special[4]["offset_cun"])
    acpoints_to_draw.append((ap_special[4]["acupointCode"], ap_special[4].get("name_zh") or "", Naoyu_right))
    #右肩外俞
    Jianwaiyu_right = calculate_acupoints_along_baseline(acpoints_to_draw[1][2],acpoints_to_draw[16][2], PIXELS_PER_CUN_3, ap_special[5]["offset_cun"])
    acpoints_to_draw.append((ap_special[5]["acupointCode"], ap_special[5].get("name_zh") or "", Jianwaiyu_right))
    #右曲垣
    Quyuan_right = calculate_acupoints_along_baseline(Quyuan_start, Quyuan_end, PIXELS_PER_CUN, ap_special[6]["offset_cun"])
    acpoints_to_draw.append((ap_special[6]["acupointCode"], ap_special[6].get("name_zh") or "", Quyuan_right))
    #右肩井
    Jianjing_right = calculate_acupoints_along_baseline(Quyuan_start, Quyuan_end, PIXELS_PER_CUN, ap_special[7]["offset_cun"])
    acpoints_to_draw.append((ap_special[7]["acupointCode"], ap_special[7].get("name_zh") or "", Jianjing_right))
    #天髎elapsed_time = end_time - start_time
    Tianliao_right = calculate_acupoints_along_baseline(Tianliao_start,Tianliao_end,PIXELS_PER_CUN,ap_special[8]["offset_cun"])
    acpoints_to_draw.append((ap_special[8]["acupointCode"], ap_special[8].get("name_zh") or "", Tianliao_right))


    #得到左背部的穴位座标
    acpoints_to_draw = mirror_right_acupoints_to_left(acpoints_to_draw,shoulder_mid,hip_mid)

    # 6. 加载原始图像
    # 方法A：从results中获取原始图像 (已经读取过)
    orig_img = result.orig_img

    # 方法B：或者重新从文件读取 (确保是原始未修改的)
    # orig_img = cv2.imread(image_path)
    
    # 创建一份副本用于绘制，避免修改原始数据
    img_with_points = orig_img.copy()

    #绘制背部穴位
    color = (0,0,255)  
    for point_name, _, point_data in acpoints_to_draw:
        x, y = point_data
        x_int, y_int = int(round(x)), int(round(y))
        
        cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, 
                    color=color, thickness=-1)  # thickness=-1 表示实心
            
            # 在点旁边添加标签
        # cv2.putText(img_with_points, point_name, (x_int+10, y_int-10),
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


    # 7. 在原始图像上绘制关键点，可选
    draw_kpts = True
    points_to_draw = []
    if draw_kpts:
        # 准备要绘制的6个点关键点
        points_to_draw = [
            ("Left Shoulder", left_shoulder, (255, 0, 0)),    # 蓝色 - 左肩
            ("Right Shoulder", right_shoulder, (0, 255, 0)),  # 绿色 - 右肩
            ("Left Hip", left_hip, (255, 255, 0)),           # 青色 - 左髋
            ("Right Hip", right_hip, (0, 255, 255)),         # 黄色 - 右髋
            ("Shoulder Mid", shoulder_mid, (0, 0, 255)),     # 红色 - 肩中点 (颈部)
            ("Hip Mid", hip_mid, (255, 0, 255)),             # 紫色 - 髋中点
        ]
        for point_name, point_data, color in points_to_draw:
            x, y, conf = point_data
            # 只绘制置信度大于0.3的点
            if conf > 0.3:
                # 将坐标转换为整数 (cv2绘制需要整数坐标)
                x_int, y_int = int(round(x)), int(round(y))
                
                # 绘制一个实心圆
                cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, 
                        color=color, thickness=-1)  # thickness=-1 表示实心
                
                # 在点旁边添加标签
                # cv2.putText(img_with_points, point_name, (x_int+5, y_int-5),
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


        # 8. 可选：绘制背部中线 (连接肩中点和髋中点)
        if shoulder_mid[2] > 0.3 and hip_mid[2] > 0.3:
            start_point = (int(round(shoulder_mid[0])), int(round(shoulder_mid[1])))
            end_point = (int(round(hip_mid[0])), int(round(hip_mid[1])))
            cv2.line(img_with_points, start_point, end_point, 
                    color=(0, 255, 255), thickness=3)  # 黄色中线
            # cv2.putText(img_with_points, "Back Center Line", 
            #         (start_point[0], start_point[1]-20),
            #         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
         
    # 9. 保存结果图像
    if ENABLE_VISUALIZATION:
        cv2.imwrite(output_path, img_with_points)
        print(f"结果已保存至: {output_path}")
    
    # 10. 在终端输出坐标信息 (用于你的穴位计算)
    if draw_kpts and ENABLE_VISUALIZATION:
        print("\n=== 关键点坐标信息 ===")
        for point_name, point_data, _ in points_to_draw:
            x, y, conf = point_data
            print(f"{point_name}: ({x:.1f}, {y:.1f}), 置信度: {conf:.3f}")

    return success, status_code, message, acpoints_to_draw, shoulder_width, hip_width, img_with_points

def draw_abdomen_acupoints_on_original(img, abdomen_acupoints_config, body_part_code, output_path="output_bdomen_points.jpg"):
    pose_model = load_yolo_model("models/x-pose.pt")
    pose_results = pose_model(img, max_det=1, conf=0.7)
    pose_result = pose_results[0]

    logger.info("开始判断人体是否正常")
    success, status_code, message = person_and_pose_judge(pose_result, body_part_code)
    if success is False:
        return success, status_code, message, None, 0.0, 0.0, None

    pose_kpts_data = pose_result.keypoints.data[0].cpu().numpy()
    left_shoulder = pose_kpts_data[5]
    right_shoulder = pose_kpts_data[6]
    left_hip = pose_kpts_data[11]
    right_hip = pose_kpts_data[12]

    navel_model = load_yolo_model("models/n-abdomen_navel_only.pt")
    navel_results = navel_model(img, max_det=1, conf=0.7)
    navel_result = navel_results[0]

    if navel_result.boxes is None or len(navel_result.boxes) == 0:
        return False, -7, "未检测到肚脐区域", None, 0.0, 0.0, None

    if navel_result.keypoints is None or len(navel_result.keypoints.data) == 0:
        return False, -8, "肚脐关键点检测失败", None, 0.0, 0.0, None

    navel_kpts_data = navel_result.keypoints.data[0].cpu().numpy()
    navel = navel_kpts_data[0].copy()

    if navel[2] < 0.5:
        return False, -9, "肚脐关键点置信度过低", None, 0.0, 0.0, None

    hip_mid_x = (left_hip[0] + right_hip[0]) / 2
    hip_mid_y = (left_hip[1] + right_hip[1]) / 2
    hip_mid = np.array([hip_mid_x, hip_mid_y, min(left_hip[2], right_hip[2])])
    
    #计算左右髋关键点的距离
    hip_distance_pixels = np.sqrt((right_hip[0] - left_hip[0])**2 + 
                            (right_hip[1] - left_hip[1])**2)
    hip_width = float(hip_distance_pixels)
    
    #计算左髋到预测肚脐的横座标距离和右髋到肚脐的横座标的距离，选择小的那个作为腹部的横向4寸
    #得到肚脐管关键点数据
    # left_hip_to_navel_Horizontal_distance = navel[0]-left_hip[0]
    # right_hip_to_navel_Horizontal_distance = right_hip[0]-navel[0]
    # base_distance = min(left_hip_to_navel_Horizontal_distance, right_hip_to_navel_Horizontal_distance)  

    #计算肩宽
    shoulder_mid_x = (left_shoulder[0] + right_shoulder[0]) / 2
    shoulder_mid_y = (left_shoulder[1] + right_shoulder[1]) / 2
    shoulder_mid = np.array([shoulder_mid_x, shoulder_mid_y, 
                             min(left_shoulder[2], right_shoulder[2])])
    shoulder_width = float(np.sqrt((right_shoulder[0] - left_shoulder[0])**2 + 
                            (right_shoulder[1] - left_shoulder[1])**2))
    shoulder_width = 1.0 +  int((shoulder_width-175.0)/15)*0.1
    
    #定义上腹部的纵向一寸为左右髋距离的1/9
    UPPER_ABDOMEN_VERTICAL_CUN = 9.0 
    UPPER_VERTICAL_PIXELS_PER_CUN = hip_distance_pixels / UPPER_ABDOMEN_VERTICAL_CUN

    UNDER_ABDOMEN_VERTICAL_CUN = 11.0
    UNDER_VERTICAL_PIXELS_PER_CUN = hip_distance_pixels / UNDER_ABDOMEN_VERTICAL_CUN

    UPPER_ABDOMEN_HORIZONTAL_CUN = 8.0
    #UPPER_HORIZONTAL_PIXELS_PER_CUN = base_distance*2 / UPPER_ABDOMEN_HORIZONTAL_CUN
    UPPER_HORIZONTAL_PIXELS_PER_CUN = hip_distance_pixels / UPPER_ABDOMEN_HORIZONTAL_CUN

    #计算其他基准点位
    navel_up = np.array([navel[0], navel[1] - UPPER_VERTICAL_PIXELS_PER_CUN, navel[2]])
    right_start1 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 0.375, "right")
    right_end1 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 0.375, "right", "end")
    right_start2 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 1.5, "right")
    right_end2 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 1.5, "right", "end")
    right_start3 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 3.0, "right")
    right_end3 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 3.0, "right", "end")
    right_start4 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 2.625, "right")
    right_end4 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 2.625, "right", "end")
    right_start5 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 3.375, "right")
    right_end5 = calculate_acupoints_along_baseline(navel, navel_up, UPPER_HORIZONTAL_PIXELS_PER_CUN, 3.375, "right", "end")

    #开始通过穴位配置json算出腹部穴位信息
    acupoints_to_draw = [] #存储穴位信息
    for ap in abdomen_acupoints_config["acupoints"]:
        if ap["start_point"] == "navel" and ap["end_point"] == "navel_up":
            start_point = navel
            end_point = navel_up
        elif ap["start_point"] == "right_start1" and ap["end_point"] == "right_end1":
            start_point = right_start1
            end_point = right_end1
        elif ap["start_point"] == "right_start2" and ap["end_point"] == "right_end2":
            start_point = right_start2
            end_point = right_end2
        elif ap["start_point"] == "right_start3" and ap["end_point"] == "right_end3":
            start_point = right_start3
            end_point = right_end3
        elif ap["start_point"] == "right_start4" and ap["end_point"] == "right_end4":
            start_point = right_start4
            end_point = right_end4
        elif ap["start_point"] == "right_start5" and ap["end_point"] == "right_end5":
            start_point = right_start5
            end_point = right_end5
        else :
            raise ValueError("基础起始点或终点点位信息错误")

        if ap["cun"] == "cun1":
            PIXELS_PER_CUN = UPPER_VERTICAL_PIXELS_PER_CUN
        elif ap["cun"] == "cun2":
            PIXELS_PER_CUN = UNDER_VERTICAL_PIXELS_PER_CUN
        
        acupointCode = ap["acupointCode"]
        acupoint_name = ap.get("name_zh") or ""
        offset_cun = ap["offset_cun"]
        acupoint_coord = calculate_acupoints_along_baseline(start_point, end_point, PIXELS_PER_CUN, offset_cun)
        acupoints_to_draw.append((acupointCode, acupoint_name, acupoint_coord))
    
    #得到左腹部的穴位信息
    acupoints_to_draw = mirror_right_acupoints_to_left(acupoints_to_draw, navel_up, navel)


    #绘制点
    orig_img = pose_result.orig_img

    img_with_points = orig_img.copy()
    for _, _, point_data in acupoints_to_draw:
        x, y = point_data
        x_int, y_int = int(round(x)), int(round(y))
        
        cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, 
                    color=(0, 0, 255), thickness=-1)

    #画关键点，可选
    draw_kpts = True
    points_to_draw = []
    if draw_kpts:
        # 5. 准备要绘制的关键点
        points_to_draw = [
            ("Left Shoulder", left_shoulder, (255, 0, 0)),    # 蓝色 - 左肩
            ("Right Shoulder", right_shoulder, (0, 255, 0)),  # 绿色 - 右肩
            ("Left Hip", left_hip, (255, 255, 0)),           # 青色 - 左髋
            ("Right Hip", right_hip, (0, 255, 255)),
            ("Hip_mid", hip_mid, (255, 255, 255)),
            ("shoulder_mid", shoulder_mid, (255, 0, 255)),       # 黄色 - 右髋
            ("Navel", navel, (0, 0, 255)),
        ]
        for _, point_data, color in points_to_draw:
            x, y, conf = point_data
            # 只绘制置信度大于0.3的点
            if conf > 0.3:
                # 将坐标转换为整数 (cv2绘制需要整数坐标)
                x_int, y_int = int(round(x)), int(round(y))
                
                # 绘制一个实心圆
                cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, color=color, thickness=-1)
                
    # 保存图片        
    if ENABLE_VISUALIZATION:
        cv2.imwrite(output_path, img_with_points)
        print(f"结果已保存至: {output_path}")
    
    # 10. 在终端输出坐标信息 (用于你的穴位计算)
    if draw_kpts and ENABLE_VISUALIZATION:
        print("\n=== 关键点坐标信息 ===")
        for point_name, point_data, _ in points_to_draw:
            x, y, conf = point_data
            print(f"{point_name}: ({x:.1f}, {y:.1f}), 置信度: {conf:.3f}")

    return success, status_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points

# 绘制腿部穴位
def draw_leg_acupoints_on_original(img, leg_acupoints_config, body_part_code, output_path="output_leg_points.jpg"):
    pose_model = load_yolo_model("models/x-pose.pt")
    pose_results = pose_model(img, max_det=1, conf=0.7)
    pose_result = pose_results[0]

    logger.info("开始判断人体是否正常")
    success, status_code, message = person_and_pose_judge(pose_result, body_part_code)
    if success is False:
        return success, status_code, message, None, 0.0, 0.0, None

    pose_kpts_data = pose_result.keypoints.data[0].cpu().numpy()
    left_hip = pose_kpts_data[11]
    right_hip = pose_kpts_data[12]
    left_knee = pose_kpts_data[13]
    right_knee = pose_kpts_data[14]
    left_ankle = pose_kpts_data[15]
    right_ankle = pose_kpts_data[16]

    left_hip_to_knee_pixels = float(np.sqrt((left_knee[0] - left_hip[0]) ** 2 + (left_knee[1] - left_hip[1]) ** 2))
    left_knee_to_ankle_pixels = float(np.sqrt((left_ankle[0] - left_knee[0]) ** 2 + (left_ankle[1] - left_knee[1]) ** 2))
    right_hip_to_knee_pixels = float(np.sqrt((right_knee[0] - right_hip[0]) ** 2 + (right_knee[1] - right_hip[1]) ** 2))
    right_knee_to_ankle_pixels = float(np.sqrt((right_ankle[0] - right_knee[0]) ** 2 + (right_ankle[1] - right_knee[1]) ** 2))

    #定义腿部的寸分左右腿
    CUN_HIP_TO_KNEE_CUN = 19.0
    CUN_KNEE_TO_ANKLE_CUN = 16.0
    left_thigh_pixels_per_cun = left_hip_to_knee_pixels / CUN_HIP_TO_KNEE_CUN
    left_calf_pixels_per_cun = left_knee_to_ankle_pixels / CUN_KNEE_TO_ANKLE_CUN
    right_thigh_pixels_per_cun = right_hip_to_knee_pixels / CUN_HIP_TO_KNEE_CUN
    right_calf_pixels_per_cun = right_knee_to_ankle_pixels / CUN_KNEE_TO_ANKLE_CUN

    #为了统一返回信息我们这里将hip_width和shoulder_width返回
    hip_width = float(np.sqrt((right_hip[0] - left_hip[0]) ** 2 + (right_hip[1] - left_hip[1]) ** 2))
    shoulder_width = 1.0

    # 将配置中的起止点组合映射到具体基线和对应寸长比例
    baseline_map = {
        ("left_hip", "left_knee"): (left_hip, left_knee, left_thigh_pixels_per_cun),
        ("right_hip", "right_knee"): (right_hip, right_knee, right_thigh_pixels_per_cun),
        ("left_knee", "left_ankle"): (left_knee, left_ankle, left_calf_pixels_per_cun),
        ("right_knee", "right_ankle"): (right_knee, right_ankle, right_calf_pixels_per_cun),
    }

    # 按配置逐个计算腿部穴位坐标
    acupoints_to_draw = []
    for ap in leg_acupoints_config["acupoints"]:
        start_point_name = ap["start_point"]
        end_point_name = ap["end_point"]
        baseline_key = (start_point_name, end_point_name)

        if baseline_key not in baseline_map:
            raise ValueError("腿部基础起始点或终点点位信息错误")

        start_point, end_point, pixels_per_cun = baseline_map[baseline_key]
        offset_cun = ap["offset_cun"]
        acupoint_code = ap["acupointCode"]
        acupoint_name = ap.get("name_zh") or ""

        acupoint_coord = calculate_acupoints_along_baseline(
            start_point,
            end_point,
            pixels_per_cun,
            offset_cun,
        )
        acupoints_to_draw.append((acupoint_code, acupoint_name, acupoint_coord))

    orig_img = pose_result.orig_img
    img_with_points = orig_img.copy()

    #画关键点，可选
    draw_kpts = True
    points_to_draw = []
    if draw_kpts:
        points_to_draw = [
            ("Left Hip", left_hip, (255, 0, 0)), # 蓝色 - 左髋
            ("Right Hip", right_hip, (0, 255, 0)), # 绿色 - 右髋
            ("Left Knee", left_knee, (255, 255, 0)), # 青色 - 左膝盖
            ("Right Knee", right_knee, (0, 255, 255)), # 黄色 - 右膝盖
            ("Left Ankle", left_ankle, (255, 0, 255)), # 紫色 - 左脚踝
            ("Right Ankle", right_ankle, (0, 0, 255)), # 红色 - 右脚踝
        ]

        for _, point_data, color in points_to_draw:
            x, y, conf = point_data
            if conf > 0.3:
                x_int, y_int = int(round(x)), int(round(y))
                cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, color=color, thickness=-1)

    #绘制腿部穴位点
    for _, _, point_data in acupoints_to_draw:
        x, y = point_data
        x_int, y_int = int(round(x)), int(round(y))
        cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, color=(0, 0, 255), thickness=-1)

    if ENABLE_VISUALIZATION:
        cv2.imwrite(output_path, img_with_points)
        print(f"结果已保存至: {output_path}")

    if draw_kpts and ENABLE_VISUALIZATION:
        print("\n=== 腿部关键点坐标信息 ===")
        for point_name, point_data, _ in points_to_draw:
            x, y, conf = point_data
            print(f"{point_name}: ({x:.1f}, {y:.1f}), 置信度: {conf:.3f}")

    return success, status_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points

#各部位识别函数整合
def chose_body_part(img, body_part_code, output_path):
    # 获取当前脚本所在的目录
    if body_part_code == "BP004":
        # 背部流程改为读取加密后的配置文件，原始 json 仅作为维护源文件保留
        back_acupoints_config = load_encrypted_json("acu_config/acupoints_config.enc")
        success, status_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points = draw_back_acupoints_on_original(img, back_acupoints_config, body_part_code, output_path)
    
    elif body_part_code == "BP014":
        # 腹部流程读取对应加密配置文件
        abdomen_acupoints_config = load_encrypted_json("acu_config/abdomen_acupoints_config.enc")
        success, status_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points = draw_abdomen_acupoints_on_original(img, abdomen_acupoints_config, body_part_code, output_path)

    elif body_part_code == "BP010":
        # 腿部流程读取对应加密配置文件
        leg_acupoints_config = load_encrypted_json("acu_config/leg_acupoints_config.enc")
        success, status_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points = draw_leg_acupoints_on_original(img, leg_acupoints_config, body_part_code, output_path)

    else:
        success = False
        status_code = -11
        message = f"不支持的body_part_code: {body_part_code}"
        acupoints_to_draw = None
        shoulder_width = None
        hip_width = None
        img_with_points = None

    # 只有在成功的情况下才打印肩宽
    if success and shoulder_width is not None:
        logger.info(f"肩宽：{shoulder_width:.2f}")
    else:
        logger.info("肩宽：无法计算")
    

    # 在这里执行删除runs目录
    try:
        runs_dir = os.path.join(os.getcwd(), "runs")
        if os.path.exists(runs_dir):
            shutil.rmtree(runs_dir)
            print("成功删除runs目录")
        else:
            print("runs目录不存在")
    except Exception as e:
        print(f"删除runs目录时出错: {e}")
    
    return success, status_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points

# 使用示例
if __name__ == "__main__":
    # 指定你的背部图像路径
    input_image = "./product_test/bxp_1.jpg" #"./test_data/8.jpg"  # 请修改为实际路径
    

    start_time = time.time()
    success, status_code, message, acpoints_to_draw, shoulder_width, hip_width, img_with_points = chose_body_part(
        img=input_image,
        body_part_code="BP004",
        output_path="test_result/product_test_result/bxp_1.jpg"
    )

    end_time = time.time()
    elapsed_time = end_time - start_time

    if success:
        print(message)
        print("\n点位计算完成, 可用于后续穴位推算!")
        print(f"总耗时:{elapsed_time:.2}秒")
    else:
        print(message)
