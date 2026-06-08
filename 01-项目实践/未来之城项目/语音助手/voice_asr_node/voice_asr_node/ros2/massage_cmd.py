from __future__ import annotations

import json
import re
from typing import Optional, Tuple

MASSAGE_POSITION_SERVICE = "/massage_position_control"
MASSAGE_DURATION_SERVICE = "/massage_duration_control"
MASSAGE_FORCE_SERVICE = "/massage_force_control"
MASSAGE_PROCESS_INFO_SERVICE = "/massage_process_info_get"
MASSAGE_HEAD_PARAM_SERVICE = "/massage_head_param_control"

DEFAULT_DURATION_DELTA = 5
DEFAULT_FORCE_DELTA = 5.0
MAX_SINGLE_FORCE_DELTA = 10.0

_POSITION_DIRECTION_MAP = {
    "left": "left",
    "right": "right",
    "upward": "forward",
    "up": "forward",
    "forward": "forward",
    "downward": "backward",
    "down": "backward",
    "backward": "backward",
    "clear": "clear",
}


def parse_colon_arg(arg: Optional[str]) -> Tuple[str, Optional[float]]:
    """解析标签参数，如 longer:5 -> (longer, 5.0)，set:30 -> (set, 30.0)。"""
    raw = str(arg or "").strip()
    if not raw:
        return "", None
    if ":" in raw:
        mode, _, rest = raw.partition(":")
        mode = mode.strip().lower()
        rest = rest.strip()
        if not rest:
            return mode, None
        try:
            return mode, float(rest)
        except ValueError:
            return mode, None
    return raw.lower(), None


def map_position_direction(direction: str) -> Optional[str]:
    return _POSITION_DIRECTION_MAP.get(str(direction or "").strip().lower())


_DURATION_LONGER_RE = re.compile(
    r"(?:再加|延长|多按).{0,8}?(\d+)\s*分钟[。！？]?",
)
_DURATION_SHORTER_RE = re.compile(
    r"(?:减|缩短|少按).{0,8}?(\d+)\s*分钟[。！？]?",
)
_DURATION_SET_RE = re.compile(
    r"(?:设为|调到|设置|设定)\s*(\d+)\s*分钟[。！？]?",
)
_FORCE_INCREASE_RE = re.compile(
    r"(?:加大|加|力度加)\s*(\d+)\s*牛?[。！？]?",
)
_FORCE_SET_RE = re.compile(
    r"(?:设为|调到|设置)\s*(\d+)\s*牛[。！？]?",
)


def is_valid_duration_arg(arg: Optional[str]) -> bool:
    command, _, _ = parse_duration_from_tag(arg)
    return command is not None


def is_valid_force_arg(arg: Optional[str]) -> bool:
    command, _, _ = parse_force_from_tag(arg)
    return command is not None


def infer_duration_arg_from_text(text: str) -> Optional[str]:
    """从口语或标签前 buffer 推断 duration 标签参数。"""
    query = str(text or "").strip()
    if not query:
        return None
    parts = [query] + [
        p.strip()
        for p in re.split(r"[。！？；;，,\s]+", query)
        if p.strip()
    ]
    for part in parts:
        m = _DURATION_SET_RE.search(part)
        if m:
            return f"set:{m.group(1)}"
        m = _DURATION_LONGER_RE.search(part)
        if m:
            return f"longer:{m.group(1)}"
        m = _DURATION_SHORTER_RE.search(part)
        if m:
            return f"shorter:{m.group(1)}"
        if any(k in part for k in ("延长", "多按一会儿", "再加", "时间延长")):
            return "longer"
        if any(k in part for k in ("缩短", "少按一会儿", "时间缩短")):
            return "shorter"
    return None


def infer_force_arg_from_text(text: str) -> Optional[str]:
    """从口语推断 force 标签参数。"""
    query = str(text or "").strip()
    if not query:
        return None
    parts = [query] + [
        p.strip()
        for p in re.split(r"[。！？；;，,\s]+", query)
        if p.strip()
    ]
    for part in parts:
        m = _FORCE_SET_RE.search(part)
        if m:
            return f"set:{m.group(1)}"
        m = _FORCE_INCREASE_RE.search(part)
        if m:
            return f"increase:{m.group(1)}"
        if any(k in part for k in ("重一点", "力度大", "加力", "用力", "力度加大")):
            return "increase"
        if any(k in part for k in ("轻一点", "力度小", "减小力度", "力度减小")):
            return "decrease"
    return None


def normalize_massage_param_arg(
    cmd_id: str,
    arg: Optional[str],
    *,
    utterance: str = "",
) -> Optional[str]:
    """补全或校验按摩参数指令 arg。"""
    token = str(arg or "").strip() or None
    if cmd_id == "massage_duration":
        if token and is_valid_duration_arg(token):
            return token
        return infer_duration_arg_from_text(utterance)
    if cmd_id == "massage_force":
        if token and is_valid_force_arg(token):
            return token
        return infer_force_arg_from_text(utterance)
    if cmd_id == "massage_position":
        if token and map_position_direction(token):
            return token
        return token
    if cmd_id == "head_param":
        if token:
            feat, action, _ = parse_head_param_tag(token)
            if feat and action:
                return token
        return token
    return token


def parse_duration_from_tag(arg: Optional[str]) -> Tuple[Optional[str], int, int]:
    """返回 (command, duration_value, duration_delta)；无效时 command=None。"""
    mode, value = parse_colon_arg(arg)
    if mode == "set":
        if value is None or value <= 0:
            return None, 0, 0
        return "set", int(value), 0
    if mode in ("longer", "shorter"):
        delta = int(value) if value is not None and value > 0 else DEFAULT_DURATION_DELTA
        return mode, 0, delta
    return None, 0, 0


def parse_force_from_tag(arg: Optional[str]) -> Tuple[Optional[str], float, float]:
    """返回 (command, force_value, force_delta)；command 为 harder/lighter/set。"""
    mode, value = parse_colon_arg(arg)
    if mode == "set":
        if value is None:
            return None, 0.0, 0.0
        return "set", max(1.0, min(160.0, abs(value))), 0.0
    if mode in ("increase", "harder"):
        delta = abs(value) if value is not None and value > 0 else DEFAULT_FORCE_DELTA
        delta = min(delta, MAX_SINGLE_FORCE_DELTA)
        return "harder", 0.0, delta
    if mode in ("decrease", "lighter"):
        delta = abs(value) if value is not None and value > 0 else DEFAULT_FORCE_DELTA
        delta = min(delta, MAX_SINGLE_FORCE_DELTA)
        return "lighter", 0.0, delta
    return None, 0.0, 0.0


def parse_head_param_tag(arg: Optional[str]) -> Tuple[Optional[str], Optional[str], Optional[float]]:
    """解析 head_param 标签参数 feature:action[:n]。"""
    raw = str(arg or "").strip()
    if not raw:
        return None, None, None
    parts = [p.strip().lower() for p in raw.split(":") if p.strip()]
    if len(parts) < 2:
        return None, None, None
    feature = parts[0]
    action = parts[1]
    value: Optional[float] = None
    if len(parts) >= 3:
        try:
            value = float(parts[2])
        except ValueError:
            value = None
    return feature, action, value


def build_head_param_json(
    feature: str,
    action: str,
    value: Optional[float] = None,
) -> Tuple[Optional[str], str]:
    """构建 MassageHeadParamControlReq.params JSON；失败返回 (None, error_msg)。"""
    feat = str(feature or "").strip().lower()
    act = str(action or "").strip().lower()

    if feat == "neg_pressure":
        if act == "on":
            return '{"negative_pressure": 1}', ""
        if act == "off":
            return '{"negative_pressure": 0}', ""
        if act == "set":
            if value is None:
                return None, "set 需要档位参数"
            level = max(1, min(16, int(value)))
            return f'{{"negative_pressure": {level}}}', ""
        if act in ("inc", "increase"):
            delta = max(1, int(value)) if value is not None else 1
            return f'{{"negative_pressure_delta": {delta}}}', ""
        if act in ("dec", "decrease"):
            delta = max(1, int(value)) if value is not None else 1
            return f'{{"negative_pressure_delta": {-delta}}}', ""

    if feat == "micro_electric":
        if act == "on":
            return '{"micro_electric": 5}', ""
        if act == "off":
            return '{"micro_electric": 0}', ""
        if act == "set":
            if value is None:
                return None, "set 需要档位参数"
            level = max(1, min(99, int(value)))
            return f'{{"micro_electric": {level}}}', ""
        if act in ("inc", "increase"):
            delta = max(1, int(value)) if value is not None else 5
            return f'{{"micro_electric_delta": {delta}}}', ""
        if act in ("dec", "decrease"):
            delta = max(1, int(value)) if value is not None else 5
            return f'{{"micro_electric_delta": {-delta}}}', ""

    if feat == "rf":
        if act == "on":
            return '{"ret": 15}', ""
        if act == "off":
            return '{"ret": 0}', ""
        if act == "set":
            if value is None:
                return None, "set 需要档位参数"
            level = max(10, min(99, int(value)))
            return f'{{"ret": {level}}}', ""
        if act in ("inc", "increase"):
            delta = max(1, int(value)) if value is not None else 5
            return f'{{"ret_delta": {delta}}}', ""
        if act in ("dec", "decrease"):
            delta = max(1, int(value)) if value is not None else 5
            return f'{{"ret_delta": {-delta}}}', ""

    if feat == "shock_wave":
        if act == "on":
            return '{"frequency": 1}', ""
        if act == "off":
            return '{"frequency": 0}', ""
        if act == "set":
            if value is None:
                return None, "set 需要档位参数"
            level = max(1, min(5, int(value)))
            return f'{{"frequency": {level}}}', ""
        if act in ("inc", "increase"):
            return '{"frequency_delta": 1}', ""
        if act in ("dec", "decrease"):
            return '{"frequency_delta": -1}', ""

    if feat == "temperature":
        if act == "set":
            if value is None:
                return None, "set 需要温度参数"
            deg = max(15, min(60, int(value)))
            return f'{{"temperature": {deg}}}', ""
        if act in ("inc", "increase"):
            delta = max(1, int(value)) if value is not None else 5
            return f'{{"temperature_delta": {delta}}}', ""
        if act in ("dec", "decrease"):
            delta = max(1, int(value)) if value is not None else 5
            return f'{{"temperature_delta": {-delta}}}', ""

    return None, f"未知按摩头功能: {feature}"


def format_massage_progress_summary(massage_process_data: str) -> str:
    """从 MassageProcessInfoGetReq 返回的 JSON 生成中文摘要。"""
    if not str(massage_process_data or "").strip():
        return "暂无按摩进度信息。"
    try:
        info = json.loads(massage_process_data)
    except json.JSONDecodeError:
        return "查询按摩进度失败。"
    if not isinstance(info, dict):
        return "暂无按摩进度信息。"
    process_info = info.get("process_info") or {}
    if process_info:
        total_minutes = int(process_info.get("massage_duration_minutes", 0))
        elapsed_seconds = float(process_info.get("elapsed_massage_seconds", 0.0))
        elapsed_minutes = elapsed_seconds / 60.0
        remaining_minutes = max(0.0, total_minutes - elapsed_minutes)
        return (
            f"按摩设定总时长 {total_minutes} 分钟，"
            f"已用约 {elapsed_minutes:.1f} 分钟，"
            f"预计剩余约 {remaining_minutes:.0f} 分钟。"
        )
    status_str = info.get("status", "未知")
    return f"当前状态：{status_str}，暂无按摩进度信息。"
