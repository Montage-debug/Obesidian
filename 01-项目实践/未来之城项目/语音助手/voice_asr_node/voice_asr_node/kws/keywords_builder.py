"""从配置中的唤醒词列表生成 Sherpa-ONNX KeywordSpotter 用的 keywords.txt。"""

from __future__ import annotations

import logging
import os
import re
import shutil
import subprocess
import sys
import tempfile
from typing import List, Optional

from ..utils.env_setup import chinese_to_kws_line

logger = logging.getLogger(__name__)


def _keyword_tag(display: str) -> str:
    return display.strip().replace(" ", "_")


def _expected_tags(keywords: List[str]) -> set[str]:
    return {_keyword_tag(k) for k in keywords if k.strip()}


def _tags_in_keywords_file(path: str) -> set[str]:
    tags: set[str] = set()
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or "@" not in line:
                continue
            tags.add(line.rsplit("@", 1)[-1].strip())
    return tags


def _english_lexicon_words(keyword: str) -> List[str]:
    """将 hey_robot / hey robot 等配置拆成 en.phone 可查的英文词。"""
    normalized = re.sub(r"[_\-]+", " ", keyword.strip())
    return [w for w in normalized.lower().split() if w]


def _keywords_raw_line(display: str) -> str:
    """zh-en KWS 原始行：原文 + @标签（标签不能含空格）。"""
    tag = _keyword_tag(display)
    if display.isascii():
        text = re.sub(r"[_\-]+", " ", display.strip()).upper()
        return f"{text} @{tag}"
    return f"{display} @{tag}"


def _find_sherpa_onnx_cli() -> Optional[str]:
    """在 PATH 或当前 Python 虚拟环境的 bin 目录查找 sherpa-onnx-cli。"""
    names = ("sherpa-onnx-cli", "sherpa-onnx-cli.exe")
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    if sys.executable:
        bin_dir = os.path.dirname(os.path.abspath(sys.executable))
        for name in names:
            candidate = os.path.join(bin_dir, name)
            if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
                return candidate
    return None


def build_keywords_file(
    *,
    keywords: List[str],
    tokens_path: str,
    lexicon_path: str = "",
    tokens_type: str = "auto",
    out_path: str = "",
) -> str:
    """
    生成或复用 keywords.txt。

    - 中英双语 KWS（含 en.phone）：sherpa-onnx-cli text2token (phone+ppinyin)
    - 中文 KWS：pypinyin 转带调拼音行
    """
    keywords = [str(k).strip() for k in keywords if str(k).strip()]
    if not keywords:
        raise ValueError("wakeword keywords list is empty")

    model_dir = os.path.dirname(tokens_path)
    if not out_path:
        out_path = os.path.join(model_dir, "keywords.txt")

    expected = _expected_tags(keywords)
    if os.path.isfile(out_path):
        existing = _tags_in_keywords_file(out_path)
        if existing == expected:
            logger.info("[keywords] use existing file: %s", out_path)
            return os.path.abspath(out_path)
        logger.info(
            "[keywords] stale %s (have %s, want %s), regenerating",
            out_path,
            sorted(existing),
            sorted(expected),
        )

    use_lexicon = lexicon_path and os.path.isfile(lexicon_path)
    if tokens_type == "auto":
        tokens_type = "phone+ppinyin" if use_lexicon else "ppinyin"

    # 逐条编码：bulk text2token 在跳过中文行时会错配 @ 标签，不可用于中英混合列表。
    lines: List[str] = []
    for kw in keywords:
        if use_lexicon and tokens_type == "phone+ppinyin" and kw.isascii():
            line = _encode_english_keyword(kw, tokens_path, lexicon_path)
            if line:
                lines.append(line)
                continue
        line = chinese_to_kws_line(kw)
        if line:
            lines.append(line)
        else:
            logger.warning("[keywords] skip keyword (cannot encode): %s", kw)

    if not lines:
        raise RuntimeError("failed to build any keyword line")

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    logger.info("[keywords] wrote %s (%d lines)", out_path, len(lines))
    return os.path.abspath(out_path)


def _encode_english_keyword(
    keyword: str,
    tokens_path: str,
    lexicon_path: str,
) -> Optional[str]:
    """单条英文唤醒词：优先 CLI，失败则用 en.phone 回退。"""
    line = _english_phone_fallback(keyword, lexicon_path)
    if line:
        return line

    cli = _find_sherpa_onnx_cli()
    if not cli:
        return None

    raw_fd, raw_path = tempfile.mkstemp(suffix="_keyword_raw.txt", text=True)
    out_fd, out_path = tempfile.mkstemp(suffix="_keyword_out.txt", text=True)
    os.close(raw_fd)
    os.close(out_fd)
    try:
        with open(raw_path, "w", encoding="utf-8") as f:
            f.write(_keywords_raw_line(keyword) + "\n")
        cmd = [
            cli,
            "text2token",
            "--tokens",
            tokens_path,
            "--tokens-type",
            "phone+ppinyin",
            "--lexicon",
            lexicon_path,
            raw_path,
            out_path,
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or "").strip()
            if err:
                logger.debug("[keywords] cli failed for %s: %s", keyword, err)
            return None
        with open(out_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line:
                    return line
        return None
    except FileNotFoundError:
        return None
    finally:
        for path in (raw_path, out_path):
            try:
                os.remove(path)
            except OSError:
                pass


def _english_phone_fallback(keyword: str, lexicon_path: str) -> Optional[str]:
    """
    从 en.phone 查词，拼 keywords.txt 行（无 CLI 时的简易回退）。
    """
    words = _english_lexicon_words(keyword)
    if not words:
        return None

    lex = _load_en_phone_lexicon(lexicon_path)
    phones: List[str] = []
    for w in words:
        entry = lex.get(w)
        if not entry:
            logger.warning("[keywords] word not in en.phone: %s", w)
            return None
        phones.extend(entry)

    tag = _keyword_tag(keyword)
    return " ".join(phones) + f" @{tag}"


def _load_en_phone_lexicon(path: str) -> dict[str, List[str]]:
    out: dict[str, List[str]] = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            word = parts[0].lower()
            out[word] = parts[1:]
    return out
