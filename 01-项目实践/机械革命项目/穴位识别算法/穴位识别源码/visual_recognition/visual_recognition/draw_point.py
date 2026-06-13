from ultralytics import YOLO
import cv2
import numpy as np
import json
import time
import os
import ctypes
import shutil  # 添加导入
import logging
import tempfile
import atexit
import io
import tarfile
import platform
from cryptography.fernet import Fernet, InvalidToken
from contextlib import contextmanager
import concurrent.futures
import openvino as ov   

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)  # 添加日志基本配置
point_radius = 2  #图片中点的半径
FERNET_KEY = b"aHd1tFp3V0PcV0ov8Aiz2BAk68WBOEB0KOwNpoKwDPk="
# 缓存解密后的 RKNN 临时目录路径，避免同进程重复解密和解包
DECRYPTED_RKNN_DIR_CACHE = {}
# 缓存解密后的 OpenVINO 临时目录路径，避免同进程重复解密和解包
DECRYPTED_OPENVINO_DIR_CACHE = {}
# 缓存 YOLO 模型实例，避免重复初始化后端
YOLO_MODEL_CACHE = {}
ENABLE_VISUALIZATION = os.getenv("YOLO_ENABLE_VIS", "1") not in {"0", "false", "False"}
# 保存 ctypes 预加载句柄，避免被垃圾回收后动态库提前卸载
RKNN_RUNTIME_HANDLES = []
DEFAULT_BACK_KEYPOINTS = {
    "left_shoulder": [110.0, 226.0, 0.9],
    "right_shoulder": [290.0, 226.0, 0.9],
    "left_hip": [136.0, 498.0, 0.9],
    "right_hip": [264.0, 498.0, 0.9],
    "left_wrist": [90.0, 500.0, 0.9],
    "right_wrist": [310.0, 500.0, 0.9],
    "neck": [200.0, 181.0, 0.9],
}
DEFAULT_ABDOMEN_KEYPOINTS = {
    "left_shoulder": [290.0, 180.0, 0.9],
    "right_shoulder": [110.0, 180.0, 0.9],
    "left_hip": [260.0, 458.0, 0.9],
    "right_hip": [140.0, 458.0, 0.9],
    "navel": [200.0, 380.0, 0.9],
}
DEFAULT_LEG_KEYPOINTS = {
    "left_hip": [143.0, 148.0, 0.9],
    "right_hip": [257.0, 148.0, 0.9],
    "left_knee": [143.0, 380.0, 0.9],
    "right_knee": [257.0, 380.0, 0.9],
    "left_ankle": [143.0, 584.0, 0.9],
    "right_ankle": [257.0, 584.0, 0.9],
}

# ==================== OpenVINO 缓存优化 ====================
cache_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "openvino_cache")
os.makedirs(cache_dir, exist_ok=True)
# 同时设置两个环境变量，兼容不同 OpenVINO 版本
os.environ["OPENVINO_CACHE_DIR"] = cache_dir
os.environ["OV_CACHE_DIR"] = cache_dir

try:
    # Ultralytics 加载 OpenVINO 模型时会在内部 new 一个新的 ov.Core() 实例，
    # 直接对该实例 set_property 对 Ultralytics 内部的 Core 无效。
    # 通过 monkey-patch ov.Core.__init__ 确保进程内所有后续创建的
    # ov.Core 实例（包括 Ultralytics 内部的）都自动携带缓存配置。
    core = ov.Core()
    core.set_property({"CACHE_DIR": cache_dir})
    logger.info(f"OpenVINO 缓存目录已设置（全局 patch）: {cache_dir}")
except Exception as e:
    logger.warning(f"OpenVINO Core 缓存设置失败: {e}")
# ========================================================


def configure_local_rknn_runtime_libs():
    # 仅在 ARM 板端尝试注入本地 RKNN 运行库，避免对 x86 开发环境造成干扰
    machine = platform.machine().lower()
    if not (machine.startswith("arm") or machine.startswith("aarch64")):
        return False

    current_dir = os.path.dirname(os.path.abspath(__file__))
    third_tool_dir = os.path.join(current_dir, "third_tool")
    if not os.path.isdir(third_tool_dir):
        logger.warning("未找到 third_tool 目录，继续使用系统默认动态库搜索路径")
        return False

    # 1) 先把 third_tool 放到动态库搜索路径最前面，优先命中项目内 so
    current_ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_parts = [part for part in current_ld_path.split(":") if part]
    if third_tool_dir not in ld_parts:
        os.environ["LD_LIBRARY_PATH"] = ":".join([third_tool_dir] + ld_parts)

    # 2) 再显式预加载关键 so，降低运行期落到 /usr/lib 的概率
    mode = getattr(ctypes, "RTLD_GLOBAL", None)
    required_libs = ["librknn_api.so", "librknnrt.so"]
    runtime_lib_path = os.path.join(third_tool_dir, "librknnrt.so")
    loaded_any = False
    for lib_name in required_libs:
        lib_path = os.path.join(third_tool_dir, lib_name)
        if not os.path.isfile(lib_path):
            logger.warning("third_tool 中缺少 %s，跳过预加载", lib_name)
            continue
        try:
            handle = ctypes.CDLL(lib_path, mode=mode) if mode is not None else ctypes.CDLL(lib_path)
            RKNN_RUNTIME_HANDLES.append(handle)
            loaded_any = True
            logger.info("已从 third_tool 预加载 RKNN 运行库: %s", lib_path)
        except OSError as exc:
            logger.warning("预加载 RKNN 运行库失败: %s, error=%s", lib_path, exc)

    # 3) rknnlite2 在 RK3588 本机路径上会走私有方法 `_get_rknn_api_lib_path`，
    #    其内部对 /usr/lib 有硬编码检查。这里做一次受控猴子补丁，
    #    强制返回项目内 third_tool 的运行库路径，避免依赖 /usr/lib。
    if os.path.isfile(runtime_lib_path):
        try:
            from rknnlite.api.rknn_runtime import RKNNRuntime

            # 避免重复补丁导致闭包里 original 指向已补丁函数
            if not getattr(RKNNRuntime, "_third_tool_lib_patch_applied", False):
                original_get_lib_path = RKNNRuntime._get_rknn_api_lib_path

                def _patched_get_rknn_api_lib_path(self):
                    return runtime_lib_path

                RKNNRuntime._get_rknn_api_lib_path = _patched_get_rknn_api_lib_path
                RKNNRuntime._third_tool_lib_patch_applied = True
                RKNNRuntime._third_tool_original_get_lib_path = original_get_lib_path
                logger.info("已补丁 RKNNRuntime._get_rknn_api_lib_path -> %s", runtime_lib_path)
        except Exception as exc:
            logger.warning("注入 RKNNRuntime 路径补丁失败，继续尝试默认行为: %s", exc)

    return loaded_any


configure_local_rknn_runtime_libs()


def cleanup_decrypted_model_files():
    # OpenVINO 解密目录使用固定路径，需跨进程保留以命中编译缓存，不在此清理
    # 只清理 RKNN 临时目录（ARM 架构，随机 temp，生命周期绑定进程）
    for temp_path in list(DECRYPTED_RKNN_DIR_CACHE.values()):
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
    temp_root = tempfile.mkdtemp(prefix=f"dec_models_rknn_")
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


def decrypt_openvino_model_dir(enc_model_path, model_stem):
    """
    解密 OpenVINO 模型到固定路径（而非随机临时目录）。
    固定路径确保 OpenVINO 编译缓存 key 跨进程一致，首次编译后后续启动可直接复用缓存。
    解密结果持久保存在 openvino_models/ 目录中，不随进程退出删除。
    """
    base_stem = os.path.basename(model_stem)
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # 固定输出路径：与 openvino_cache/ 同级，确保跨进程路径不变
    fixed_models_root = os.path.join(current_dir, "openvino_models")
    os.makedirs(fixed_models_root, exist_ok=True)
    model_dir = os.path.join(fixed_models_root, f"{base_stem}_openvino_model")

    # 进程内缓存：同一进程内同一模型只走一次后续逻辑
    cache_key = os.path.abspath(enc_model_path)
    if cache_key in DECRYPTED_OPENVINO_DIR_CACHE:
        cached_dir = DECRYPTED_OPENVINO_DIR_CACHE[cache_key]
        if os.path.isdir(cached_dir):
            return cached_dir

    # 磁盘缓存：已解密目录完整则直接复用，无需重新解密
    if os.path.isdir(model_dir):
        dir_files = os.listdir(model_dir)
        has_metadata = os.path.isfile(os.path.join(model_dir, "metadata.yaml"))
        has_xml = f"{base_stem}.xml" in dir_files
        has_bin = f"{base_stem}.bin" in dir_files
        if has_metadata and has_xml and has_bin:
            logger.info("复用已解密的 OpenVINO 模型: %s", model_dir)
            DECRYPTED_OPENVINO_DIR_CACHE[cache_key] = model_dir
            return model_dir
        # 目录存在但不完整（上次解密中断），删除后重新解密
        logger.warning("OpenVINO 模型目录不完整，重新解密: %s", model_dir)
        shutil.rmtree(model_dir)

    # 读取加密包并用 Fernet 解密，解密结果应为 tar.gz 字节流
    with open(enc_model_path, "rb") as enc_file:
        encrypted_content = enc_file.read()

    try:
        decrypted_content = Fernet(FERNET_KEY).decrypt(encrypted_content)
    except InvalidToken as exc:
        raise ValueError(f"OpenVINO 模型目录解密失败: {enc_model_path}") from exc

    # 解包到固定目录
    with tarfile.open(fileobj=io.BytesIO(decrypted_content), mode="r:gz") as archive:
        archive.extractall(fixed_models_root)

    # 兼容历史包结构（解包后目录名不符合约定时，找唯一子目录并重命名）
    if not os.path.isdir(model_dir):
        sub_dirs = [
            os.path.join(fixed_models_root, entry)
            for entry in os.listdir(fixed_models_root)
            if os.path.isdir(os.path.join(fixed_models_root, entry))
               and entry != f"{base_stem}_openvino_model"
        ]
        if len(sub_dirs) == 1:
            os.rename(sub_dirs[0], model_dir)
        else:
            raise ValueError(f"解密后的 OpenVINO 目录结构异常: {enc_model_path}")

    # OpenVINO 目录完整性校验：必须有 metadata.yaml、{base_stem}.xml 和 {base_stem}.bin
    dir_files = os.listdir(model_dir)
    has_metadata = os.path.isfile(os.path.join(model_dir, "metadata.yaml"))
    has_xml = f"{base_stem}.xml" in dir_files
    has_bin = f"{base_stem}.bin" in dir_files
    if not has_metadata or not has_xml or not has_bin:
        raise ValueError(
            f"OpenVINO 目录缺少必要文件 (metadata.yaml / {base_stem}.xml / {base_stem}.bin): {model_dir}"
        )

    logger.info("OpenVINO 模型解密完成: %s", model_dir)
    DECRYPTED_OPENVINO_DIR_CACHE[cache_key] = model_dir
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

    # 非 ARM 架构：使用加密 OpenVINO 模型目录，利用 CPU 核显加速
    openvino_enc_filename = f"{model_stem}_openvino_model.enc"
    openvino_enc_path = os.path.join(current_dir, openvino_enc_filename)
    if os.path.isfile(openvino_enc_path):
        return decrypt_openvino_model_dir(openvino_enc_path, model_stem)

    raise FileNotFoundError(f"非 ARM 架构下找不到加密 OpenVINO 模型文件: {openvino_enc_filename}")


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


# 模块级全局模型实例，由 preload_all_models() 在节点启动时统一初始化
_g_pose_model = None
_g_neck_model = None
_g_navel_model = None


def preload_all_models():
    """预加载所有模型到内存，并并行执行预热推理触发 OpenVINO JIT 编译，在节点启动时调用"""
    s_time = time.time()
    global _g_pose_model, _g_neck_model, _g_navel_model
    logger.info("开始预加载所有模型...")
    _g_pose_model = load_yolo_model("models/x-pose.pt")
    _g_neck_model = load_yolo_model("models/neck_x.pt")
    _g_navel_model = load_yolo_model("models/n-abdomen_navel_only.pt")
    logger.info("所有模型加载完成，开始并行 OpenVINO 预热推理（首次 JIT 编译）...")

    # 每个线程独立持有一张哑图，避免多线程共享可写缓冲区
    # 尺寸必须与真实图片 letterbox 后的张量形状一致（640×416）
    def _warmup(name, model):
        dummy = np.zeros((640, 416, 3), dtype=np.uint8)
        if is_arm_architecture():
            model(dummy)
            logger.info("%s 预热完成", name)
        else:
            model(dummy, device="intel:gpu")
            logger.info("%s 预热完成", name)

    warmup_tasks = [
        ("x-pose",               _g_pose_model),
        ("neck_x",               _g_neck_model),
        ("n-abdomen_navel_only", _g_navel_model),
    ]

    with concurrent.futures.ThreadPoolExecutor(max_workers= 1) as executor: #这里本来是len(warmup_tasks)，但是多线程并行可能引起注册冲突，所以设置为1
        futures = {executor.submit(_warmup, name, model): name for name, model in warmup_tasks}
        for f in concurrent.futures.as_completed(futures):
            f.result()  # 有异常时在此处抛出，不会静默吞掉

    logger.info("所有模型预加载及预热完成")
    e_time = time.time()
    logger.info("预加载总耗时: %.2f 秒", e_time - s_time)



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
        failed_kpts = [name for name, kpt in required_kpts if kpt[2] < 0.4]
        if len(failed_kpts) > 0:
            message = f"腿部必要关键点检测失败"
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

        MAX_THIGH_SPREAD_ANGLE = 20.0
        if thigh_spread_angle > MAX_THIGH_SPREAD_ANGLE or thigh_spread_angle is None:
            message = "大腿分开角度过大, 或大腿夹角计算失败"
            status_code = -11
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
            print(Mline_bi_Sline, Mline_bi_Hline)
            status_code = -5
            return False, status_code, message
    elif body_part_code == "BP014":
        if Mline_bi_Sline > 2.5 or Mline_bi_Hline > 3.2:
            message = "人没完全躺平，请保证人体平躺并放松"
            print(Mline_bi_Sline, Mline_bi_Hline)
            status_code = -5
            return False, status_code, message

    #疑似人躺斜了
    if body_part_code == "BP004":
        if abs(shoulder_angle) > 10.0 or abs(hip_angle) > 6 or (abs(body_angle) > 105.0 or abs(body_angle) < 75.0):
            message = "人体没有躺正身体可能存在歪斜，请保持身体正直，双肩自然放松"
            print(abs(shoulder_angle), abs(hip_angle), abs(body_angle))
            status_code = -6
            return False, status_code, message
    elif body_part_code == "BP014":
        if (abs(body_angle) > 100.0 or abs(body_angle) < 80.0):
            print(abs(body_angle))
            message = "人体没有躺正身体可能存在歪斜，请保持身体正直，身体自然放松"
            status_code = -6
            return False, status_code, message

    return True, 0, "检测正常"      


def draw_back_acupoints_on_original(img, acupoints_config, body_parrt_code, output_path="output_with_points.jpg"):
    """
    1. 使用YOLO检测单人姿态
    2. 计算左右肩、左右髋、肩中点、髋中点坐标
    3. 在原始输入图像上绘制这6个点
    """
    success = True
    status_code = 0
    message = "检测正常"
    result = None
    use_default_template = False

    try:
        model = _g_pose_model
        neck_model = _g_neck_model

        with log_time("背部-模型推理"):
            if is_arm_architecture():
                results = model(img, max_det=1, conf=0.7)
                neck_results = neck_model(img, max_det=1, conf=0.4)
            else:
                results = model(img, max_det=1, conf=0.7, device = "intel:gpu")
                neck_results = neck_model(img, max_det=1, conf=0.4, device = "intel:gpu")
                
        if len(neck_results) == 0:
            raise ValueError("背部(颈部)模型未返回检测结果")
        result = results[0]
        neck_result = neck_results[0]

        logger.info("开始判断人体是否正常")
        success, status_code, message = person_and_pose_judge(result, body_parrt_code)

        #判断颈部关键点是否正常
        if success:
            if neck_result.boxes is None or len(neck_result.boxes.data) == 0:
                success = False
                message = "颈部检测失败"
                status_code = -12
            elif neck_result.keypoints is None or len(neck_result.keypoints.data) == 0:
                success = False
                message = "颈部关键点检测失败"
                status_code = -13
            elif neck_result.keypoints.data[0].cpu().numpy()[0][2] < 0.4:
                success = False
                message = "颈部关键点置信度过低"
                status_code = -14
        
        if success is False:
            use_default_template = True
            success = True
            message = f"{message}，已切换默认关键点模板"
            logger.warning("背部姿态判定未通过，使用默认关键点模板继续生成穴位")
               
    except Exception as exc:
        use_default_template = True
        success = True
        status_code = -15
        message = f"背部识别异常，已切换默认关键点模板: {exc}"
        logger.exception("背部模型推理或姿态判定异常，使用默认关键点模板继续生成穴位")

    if use_default_template:
        left_shoulder = np.array(DEFAULT_BACK_KEYPOINTS["left_shoulder"], dtype=np.float32)
        right_shoulder = np.array(DEFAULT_BACK_KEYPOINTS["right_shoulder"], dtype=np.float32)
        left_hip = np.array(DEFAULT_BACK_KEYPOINTS["left_hip"], dtype=np.float32)
        right_hip = np.array(DEFAULT_BACK_KEYPOINTS["right_hip"], dtype=np.float32)
        neck = np.array(DEFAULT_BACK_KEYPOINTS["neck"], dtype=np.float32)
    else:
        # 3. 提取原始图像并获取关键点索引 (COCO 17点格式)
        # 索引: 5-左肩, 6-右肩, 11-左髋, 12-右髋
        kpts_data = result.keypoints.data[0].cpu().numpy()
        left_shoulder = kpts_data[5]
        right_shoulder = kpts_data[6]
        left_hip = kpts_data[11]
        right_hip = kpts_data[12]
        neck = neck_result.keypoints.data[0].cpu().numpy()[0]
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
    CUN_OF_MIDLINE = 22
    distance_pixels = np.sqrt((hip_mid_x - neck[0])**2 + 
                            (hip_mid_y - neck[1])**2)
    PIXELS_PER_CUN = distance_pixels / CUN_OF_MIDLINE
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
    #shoulder_width_conference 用来做肩宽参考，这是一个像素值。目的是判断这个人的肩宽大小来判定当按摩对象抬手拍照是的整体穴位偏移。
    shoulder_width_conference = shoulder_width
    #这是一个肩宽因子用于给按摩手法确定打圈的半径
    shoulder_width = 1.0 +  int((shoulder_width-175.0)/15)*0.1

    hip_width = float(np.sqrt((right_hip[0] - left_hip[0])**2 + 
                              (right_hip[1] - left_hip[1])**2))   
    print(f"胯宽: {hip_width}")    


    back_start_point = calculate_acupoints_along_baseline(neck, hip_mid, PIXELS_PER_CUN, offset_cun=-4.0)

    # #防止裤子穿过高或者腰部有遮挡导致的背部寸长失调的问题
    if PIXELS_PER_CUN/PIXELS_PER_CUN_S <= 1.28:
        PIXELS_PER_CUN=PIXELS_PER_CUN_S*1.28 

    
    #计算其他的基准点
      #常规穴位基准点
    right_start1 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=1.5, direction="right")
    right_end1 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=1.5, direction="right",base_point="end")

    right_start2 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=0.75, direction="right")
    right_end2 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=0.75, direction="right",base_point="end")

    right_start3 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=3, direction="right")
    right_end3 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=3, direction="right",base_point="end")

    right_start4 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=4.5, direction="right")
    right_end4 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=4.5, direction="right",base_point="end")

    right_start5 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=7.0, direction="right")
    right_end5 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=7.0, direction="right",base_point="end")

    right_start6 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=6.0, direction="right")
    right_end6 = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=6.0, direction="right",base_point="end")
     #特殊穴位基准点
    Tianzong_start = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=6, direction="right")
    Tianzong_end = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=6, direction="right", base_point="end")

    Jianzhongyu_start = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=2.0, direction="right")
    Jianzhongyu_end = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=2.0, direction="right",base_point="end")

    Naoyu_start = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=9.0, direction="right")
    Naoyu_end = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=9.0, direction="right", base_point="end")

    Quyuan_start = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=4.5, direction="right")
    Quyuan_end = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=4.5, direction="right",base_point="end")

    Tianliao_start = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=5.5, direction="right")
    Tianliao_end = calculate_acupoints_along_baseline(back_start_point, hip_mid, PIXELS_PER_CUN_3, offset_cun=5.5, direction="right",base_point="end")

    acpoints_to_draw = []

    
    #计算中线及右背部的穴位座标
    for ap in acupoints_config["acupoints"]:
        if ap["start_point"] == "back_start_point" and ap["end_point"] == "hip_mid":
            start_point = back_start_point
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
    #天髎
    Tianliao_right = calculate_acupoints_along_baseline(Tianliao_start,Tianliao_end,PIXELS_PER_CUN,ap_special[7]["offset_cun"])
    acpoints_to_draw.append((ap_special[7]["acupointCode"], ap_special[7].get("name_zh") or "", Tianliao_right))

    #这里开始是虚拟点位    
    for ap in acupoints_config["acupoints_XN"]:
        if ap["start_point"] == "back_start_point" and ap["end_point"] == "hip_mid":
            start_point = back_start_point
            end_point = hip_mid
        elif ap["start_point"] == "right_start1" and ap["end_point"] == "right_end1":
            start_point = right_start1
            end_point = right_end1
        elif ap["start_point"] == "right_start3" and ap["end_point"] == "right_end3":
            start_point = right_start3
            end_point = right_end3
        elif ap["start_point"] == "right_start4" and ap["end_point"] == "right_end4":
            start_point = right_start4
            end_point = right_end4
        elif ap["start_point"] == "right_start5" and ap["end_point"] == "right_end5":
            start_point = right_start5
            end_point = right_end5
        elif ap["start_point"] == "right_start6" and ap["end_point"] == "right_end6":
            start_point = right_start6
            end_point = right_end6
            
        else :
            raise ValueError("基础起始点或终点点位信息错误")
        offset_cun = ap["offset_cun"]
        acupointCode = ap["acupointCode"]
        acupoint_name = ap.get("name_zh") or ""
        acupoint_coord = calculate_acupoints_along_baseline(start_point, end_point, PIXELS_PER_CUN, offset_cun)
        acpoints_to_draw.append((acupointCode, acupoint_name, acupoint_coord))

    #虚拟点位中的特殊点位
    ap_XN_special = acupoints_config["acupoints_XN_special"]
    #臑肩右
    Naojian_right = calculate_acupoints_along_baseline(acpoints_to_draw[60][2], acpoints_to_draw[56][2], PIXELS_PER_CUN, ap_XN_special[0]["offset_cun"])
    acpoints_to_draw.append((ap_XN_special[0]["acupointCode"], ap_XN_special[0].get("name_zh") or "", Naojian_right))

    #得到左背部的穴位座标
    acpoints_to_draw = mirror_right_acupoints_to_left(acpoints_to_draw, back_start_point,hip_mid)

    # 6. 加载原始图像
    # 方法A：从results中获取原始图像 (已经读取过)
    orig_img = result.orig_img  # 确保不修改输入图像

    # 方法B：或者重新从文件读取 (确保是原始未修改的)
    # orig_img = cv2.imread(image_path)
    
    # 创建一份副本用于绘制，避免修改原始数据
    img_with_points = orig_img.copy()

    #绘制背部穴位
    for point_name, _, point_data in acpoints_to_draw:
        x, y = point_data
        x_int, y_int = int(round(x)), int(round(y))
        
        if "XN" in point_name:
            cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, 
                    color=(255, 255, 0), thickness=-1)
        else:
            cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, 
                    color=(0, 0, 255), thickness=-1)  

    # 7. 在原始图像上绘制关键点，可选
    draw_kpts = True
    points_to_draw = []
    if draw_kpts:
        # 准备要绘制的6个点关键点
        points_to_draw = [
            ("Left Shoulder", left_shoulder, (255, 0, 0)),    # 蓝色 - 左肩
            ("Right Shoulder", right_shoulder, (0, 255, 0)),  # 绿色 - 右肩
            ("Left Hip", left_hip, (128, 0, 128)),           # 紫色 - 左髋
            ("Right Hip", right_hip, (0, 255, 255)),         # 黄色 - 右髋
            ("Shoulder Mid", shoulder_mid, (255, 0, 255)),     # 红色 - 肩中点 (颈部)
            ("Hip Mid", hip_mid, (255, 0, 255)),            # 紫色 - 髋中点
            ("Neck", neck, (255, 165, 0)),                 # 橙色 - 颈部点
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
        # if neck[2] > 0.3 and hip_mid[2] > 0.3:
        #     start_point = (int(round(neck[0])), int(round(neck[1])))
        #     end_point = (int(round(hip_mid[0])), int(round(hip_mid[1])))
        #     cv2.line(img_with_points, start_point, end_point, 
        #             color=(0, 255, 255), thickness=3)  # 黄色中线
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
    success = True
    status_code = 0
    message = "检测正常"
    pose_result = None
    use_default_template = False

    try:
        pose_model = _g_pose_model
        navel_model = _g_navel_model

        if is_arm_architecture():
            pose_results = pose_model(img, max_det=1, conf=0.7)
            navel_results = navel_model(img, max_det=1, conf=0.7)
        else:
            pose_results = pose_model(img, max_det=1, conf=0.7, device = "intel:gpu")
            navel_results = navel_model(img, max_det=1, conf=0.7, device = "intel:gpu")

        if len(pose_results) == 0:
            raise ValueError("腹部姿态模型未返回检测结果")
        pose_result = pose_results[0]

        logger.info("开始判断人体是否正常")
        success, status_code, message = person_and_pose_judge(pose_result, body_part_code)

        if success:
            pose_kpts_data = pose_result.keypoints.data[0].cpu().numpy()
            left_shoulder = pose_kpts_data[5]
            right_shoulder = pose_kpts_data[6]
            left_hip = pose_kpts_data[11]
            right_hip = pose_kpts_data[12]

            navel_result = navel_results[0]
            if navel_result.boxes is None or len(navel_result.boxes) == 0:
                status_code = -7
                message = "未检测到肚脐区域"

            elif navel_result.keypoints is None or len(navel_result.keypoints.data) == 0:
                status_code = -8
                message = "肚脐关键点检测失败"
            
            elif navel_result.keypoints.data[0][0][2] < 0.4:
                status_code = -9
                message = "肚脐关键点置信度过低"

            navel_kpts_data = navel_result.keypoints.data[0].cpu().numpy()
            navel = navel_kpts_data[0].copy()

        if success is False:
            use_default_template = True
            success = True
            message = f"{message}，已切换默认腹部关键点模板"
            logger.warning("腹部姿态判定未通过，使用默认腹部关键点模板继续生成穴位")
        
    except Exception as exc:
        use_default_template = True
        success = True
        status_code = -15
        message = f"腹部识别异常，已切换默认腹部关键点模板: {exc}"
        logger.exception("腹部识别流程异常，使用默认腹部关键点模板继续生成穴位")

    if use_default_template:
        left_shoulder = np.array(DEFAULT_ABDOMEN_KEYPOINTS["left_shoulder"], dtype=np.float32)
        right_shoulder = np.array(DEFAULT_ABDOMEN_KEYPOINTS["right_shoulder"], dtype=np.float32)
        left_hip = np.array(DEFAULT_ABDOMEN_KEYPOINTS["left_hip"], dtype=np.float32)
        right_hip = np.array(DEFAULT_ABDOMEN_KEYPOINTS["right_hip"], dtype=np.float32)
        navel = np.array(DEFAULT_ABDOMEN_KEYPOINTS["navel"], dtype=np.float32)

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
    navel_up = np.array([shoulder_mid_x - hip_mid_x + navel[0], shoulder_mid_y - hip_mid_y + navel[1], navel[2]])
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
    orig_img =  pose_result.orig_img  # 确保不修改输入图像
 # 确保不修改输入图像

    img_with_points = orig_img.copy()
    for acupointCode, _, point_data in acupoints_to_draw:
        x, y = point_data
        x_int, y_int = int(round(x)), int(round(y))
        if "XN" in acupointCode:
            cv2.circle(img_with_points, (x_int, y_int), radius=point_radius, 
                    color=(255, 255, 0), thickness=-1)
        else:
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
    success = True
    status_code = 0
    message = "检测正常"
    pose_result = None
    use_default_template = False

    try:
        pose_model = _g_pose_model
        if is_arm_architecture():
            pose_results = pose_model(img, max_det=1, conf=0.7)
        else:
            pose_results = pose_model(img, max_det=1, conf=0.7, device = "intel:gpu")

        pose_result = pose_results[0]

        logger.info("开始判断人体是否正常")
        success, status_code, message = person_and_pose_judge(pose_result, body_part_code)

        if success is False:
            use_default_template = True
            success = True
            message = f"{message}，已切换默认腿部关键点模板"
            logger.warning("腿部姿态判定未通过，使用默认腿部关键点模板继续生成穴位")

        else:
            pose_kpts_data = pose_result.keypoints.data[0].cpu().numpy()
            # 这里我们根据识别出来的左右脚的在图片中的x座标位置为确定真实的左腿和右腿。
            if pose_kpts_data[11][0] < pose_kpts_data[12][0]:
                #正常情况
                left_hip = pose_kpts_data[11]
                right_hip = pose_kpts_data[12]
                left_knee = pose_kpts_data[13]
                right_knee = pose_kpts_data[14]
                left_ankle = pose_kpts_data[15]
                right_ankle = pose_kpts_data[16]
                #很多时候出现左腿脚踝偏右一点的情况所以稍微吧左腿往左移动一点
                left_ankle[0] = left_ankle[0] - 10.0
            else:
                #识别反的情况
                left_hip = pose_kpts_data[12]
                right_hip = pose_kpts_data[11]
                left_knee = pose_kpts_data[14]
                right_knee = pose_kpts_data[13]
                left_ankle = pose_kpts_data[16]
                right_ankle = pose_kpts_data[15]
    except Exception as exc:
        success = True
        use_default_template = True
        status_code = -15
        message = f"腿部姿态识别异常,已切换默认腿部关键点模板: {exc}"
        logger.exception("腿部姿态识别流程异常")
        return success, status_code, message, None, 0.0, 0.0, None
    
    if use_default_template:
        left_hip = np.array(DEFAULT_LEG_KEYPOINTS["left_hip"], dtype=np.float32)
        right_hip = np.array(DEFAULT_LEG_KEYPOINTS["right_hip"], dtype=np.float32)
        left_knee = np.array(DEFAULT_LEG_KEYPOINTS["left_knee"], dtype=np.float32)
        right_knee = np.array(DEFAULT_LEG_KEYPOINTS["right_knee"], dtype=np.float32)
        left_ankle = np.array(DEFAULT_LEG_KEYPOINTS["left_ankle"], dtype=np.float32)
        right_ankle = np.array(DEFAULT_LEG_KEYPOINTS["right_ankle"], dtype=np.float32)
    

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

# 模块加载时自动执行
if __name__ != "__main__" or True:   # 确保被 import 时也执行
    if not is_arm_architecture():
        preload_all_models()   # 建议移到这里

# 使用示例
if __name__ == "__main__":
    input_image = f"./back_adjust_data/dhy1_orig.jpg" #"./test_data/8.jpg"  # 请修改为实际路径
        
    for i in range(3):
        start_time = time.time()
        success, status_code, message, acpoints_to_draw, shoulder_width, hip_width, img_with_points = chose_body_part(
            img=input_image,
            body_part_code="BP010",
            output_path=f"./leg_data/leg_1.jpg"
        )

        end_time = time.time()
        elapsed_time = end_time - start_time

        if success:
            print(message)
            print("\n点位计算完成, 可用于后续穴位推算!")
            print(f"总耗时:{elapsed_time:.2}秒")
        else:
            print(message)
