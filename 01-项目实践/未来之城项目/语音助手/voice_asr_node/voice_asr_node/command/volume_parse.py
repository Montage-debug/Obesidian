from __future__ import annotations

import re
from typing import Optional

from .cmd_result import CmdResult

_CN_ONES = {
    "零": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}

_VOLUME_SET_RE = re.compile(
    r"(?:声音|音量)"
    r".{0,8}?"
    r"(?:调到|调至|设为|设置为|调整到?|调为|已调到|已到|为)"
    r"\s*"
    r"(\d{1,3}|[一二两三四五六七八九十百]+|最大|最小|满|最高|最低)"
    r"\s*[%％]?",
)


def _cn_token_to_int(token: str) -> Optional[int]:
    s = str(token or "").strip()
    if not s:
        return None
    if s in ("最大", "满", "最高"):
        return 100
    if s in ("最小", "最低"):
        return 0
    if s.isdigit():
        return int(s)
    if s == "十":
        return 10
    m = re.fullmatch(r"([一二两三四五六七八九])十([一二三四五六七八九])?", s)
    if m:
        tens = _CN_ONES.get(m.group(1) or "一", 1)
        ones = _CN_ONES.get(m.group(2), 0) if m.group(2) else 0
        return tens * 10 + ones
    m = re.fullmatch(r"十([一二三四五六七八九])", s)
    if m:
        return 10 + _CN_ONES[m.group(1)]
    m = re.fullmatch(r"([一二两三四五六七八九])百", s)
    if m:
        return _CN_ONES[m.group(1)] * 100
    if s in _CN_ONES:
        return _CN_ONES[s]
    return None


def parse_volume_level_arg(text: str) -> Optional[str]:
    """从 ASR 文本解析绝对音量 0–100，失败返回 None。"""
    query = str(text or "").strip()
    if not query:
        return None
    for part in re.split(r"[。！？；;，,\s]+", query):
        part = part.strip()
        if not part:
            continue
        m = _VOLUME_SET_RE.search(part)
        if not m:
            continue
        level = _cn_token_to_int(m.group(1))
        if level is None:
            continue
        return str(max(0, min(100, int(level))))
    m = _VOLUME_SET_RE.search(query)
    if not m:
        return None
    level = _cn_token_to_int(m.group(1))
    if level is None:
        return None
    return str(max(0, min(100, int(level))))


def normalize_volume_arg(
    arg: Optional[str], *, utterance: str = ""
) -> Optional[str]:
    """将 regex/标签捕获的 arg 或整句 ASR 归一化为 0–100 字符串。"""
    token = str(arg or "").strip()
    if token.isdigit():
        return str(max(0, min(100, int(token))))
    if token:
        level = _cn_token_to_int(token)
        if level is not None:
            return str(max(0, min(100, int(level))))
    if utterance:
        return parse_volume_level_arg(utterance)
    return None


def try_match_volume_set_asr(text: str) -> Optional[CmdResult]:
    arg = parse_volume_level_arg(text)
    if arg is None:
        return None
    return CmdResult(
        cmd_id="volume_set",
        action="music_volume",
        reply="",
        arg=arg,
    )
