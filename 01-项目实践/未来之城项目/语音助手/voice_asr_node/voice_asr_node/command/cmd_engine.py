from __future__ import annotations

import re
from typing import AbstractSet, Dict, FrozenSet, List, Optional, Set

from .cmd_registry import CmdRegistry
from .cmd_result import CmdResult
from .volume_parse import normalize_volume_arg, try_match_volume_set_asr
from ..ros2.massage_cmd import normalize_massage_param_arg


CMD_TAG_PATTERN = re.compile(r"\[CMD:([a-zA-Z_]+)(?::([^\]]+))?\]")
_CMD_TAG_PARTIAL_RE = re.compile(r"\[CMD:?[a-zA-Z0-9_:]*", re.IGNORECASE)
_SPLIT_TAG_REPAIR_RE = re.compile(r"\[CMD:([a-zA-Z_]+)\](:[^\]]+\])")
_BARE_PARAM_TAG_RE = re.compile(r"\[CMD:([a-zA-Z_]+)\]\s*$")
_PARAMETRIC_CMD_IDS = frozenset(
    {
        "massage_duration",
        "massage_force",
        "massage_position",
        "head_param",
    }
)
_INTENT_BUFFER_MAX = 512
# 句号等后的 `[`（含 `。[CMD:`）；另匹配裸 `[CMD` 前缀，避免无标点时也朗读标签
_TAG_BOUNDARY_RE = re.compile(
    r"(?:[。！？,.!?；;]\s*\[)|(?:\[CMD\b)",
    re.IGNORECASE,
)
_TRAILING_PUNCT = "。！？,.!?；; "


def _normalize_utterance(text: str) -> str:
    return str(text or "").strip().strip(_TRAILING_PUNCT)


def _keyword_matches(keyword: str, query: str) -> bool:
    """关键词匹配：整句相等，或短句中包含关键词（避免长句子串误触发）。"""
    kw = str(keyword or "").strip()
    if not kw:
        return False
    q = _normalize_utterance(query)
    if not q:
        return False
    if q == kw:
        return True
    if kw not in q:
        return False
    idx = q.find(kw)
    if idx > 0 and q[idx - 1] in "不别勿没非":
        return False
    # 长句里出现「换首歌」等子串时不触发（ChatResponse 已走 match_intent_tags）
    return len(q) <= len(kw) + 2


def cmd_engine_from_command_raw(command_raw: Dict) -> "CmdEngine":
    """从 command.yaml 构造引擎（读取 settings.match_mode / asr_fallback_ids）。"""
    settings = (command_raw or {}).get("settings", {}) or {}
    mode = str(settings.get("match_mode", "llm_tag")).strip().lower()
    raw_fb = settings.get("asr_fallback_ids")
    if raw_fb is None:
        fallback: FrozenSet[str] = frozenset({"dialog_stop_speech"})
    else:
        fallback = frozenset(str(x).strip() for x in raw_fb if str(x).strip())
    return CmdEngine(CmdRegistry(command_raw), match_mode=mode, asr_fallback_ids=fallback)


class CmdEngine:
    def __init__(
        self,
        registry: CmdRegistry,
        *,
        match_mode: str = "llm_tag",
        asr_fallback_ids: Optional[AbstractSet[str]] = None,
    ):
        self._registry = registry
        self._match_mode = match_mode or "llm_tag"
        self._asr_fallback_ids: FrozenSet[str] = frozenset(
            asr_fallback_ids or ("dialog_stop_speech",)
        )
        self._intent_buffer = ""
        self._spoken_chars_before_tag = 0
        self._tag_tts_tail_pending = False
        self._tag_cut_urgency_snapshot = "none"

    def reset_intent_buffer(self, *, clear_tag_gate: bool = True) -> None:
        self._intent_buffer = ""
        if clear_tag_gate:
            self._spoken_chars_before_tag = 0
            self._tag_tts_tail_pending = False
            self._tag_cut_urgency_snapshot = "none"

    @property
    def spoken_chars_before_tag(self) -> int:
        return self._spoken_chars_before_tag

    def intent_buffer_text(self) -> str:
        return self._intent_buffer

    def spoken_text_before_tag(self) -> str:
        """标签前应对用户朗读的口语（不含 [CMD:...] 及之后内容）。"""
        buf = self._intent_buffer
        if not buf:
            return ""
        idx = self._tag_start_index(buf)
        if idx < 0:
            return buf.strip()
        if idx < len(buf) and buf[idx] in "。！？,.!?；;":
            return buf[: idx + 1].strip()
        return buf[:idx].strip()

    def _tag_start_index(self, buf: str) -> int:
        m = CMD_TAG_PATTERN.search(buf)
        if m:
            return m.start()
        pm = _CMD_TAG_PARTIAL_RE.search(buf)
        if pm:
            return pm.start()
        bm = _TAG_BOUNDARY_RE.search(buf)
        if bm:
            frag = bm.group(0)
            if frag.upper().startswith("[CMD"):
                return bm.start()
            return bm.end() - 1
        if buf.rstrip().endswith("["):
            return len(buf.rstrip()) - 1
        return -1

    def _refresh_spoken_chars_before_tag(self) -> None:
        buf = self._intent_buffer
        idx = self._tag_start_index(buf)
        if idx >= 0:
            if idx < len(buf) and buf[idx] in "。！？,.!?；;":
                spoken_len = len(buf[: idx + 1].strip())
            else:
                spoken_len = len(buf[:idx].strip())
            if spoken_len > 0:
                self._spoken_chars_before_tag = max(self._spoken_chars_before_tag, spoken_len)

    def _trim_intent_buffer_if_oversized(self) -> None:
        buf = self._intent_buffer
        if len(buf) <= _INTENT_BUFFER_MAX:
            return
        tag_m = CMD_TAG_PATTERN.search(buf)
        if tag_m is None and not _CMD_TAG_PARTIAL_RE.search(buf):
            self._intent_buffer = ""
            return
        keep_from = max(
            buf.rfind("[CMD", max(0, len(buf) - _INTENT_BUFFER_MAX)),
            len(buf) - _INTENT_BUFFER_MAX,
        )
        self._intent_buffer = buf[keep_from:] if keep_from > 0 else buf[-_INTENT_BUFFER_MAX:]

    def _repair_intent_buffer_text(self, buf: str) -> str:
        """修复 LLM 拆包标签：[CMD:xxx]:longer:5] → [CMD:xxx:longer:5]。"""
        return _SPLIT_TAG_REPAIR_RE.sub(r"[CMD:\1\2", str(buf or ""))

    def _intent_buffer_may_extend(self, buf: str) -> bool:
        """标签已闭合但参数可能还在后续 chunk（:longer:5]）。"""
        repaired = self._repair_intent_buffer_text(buf)
        if _BARE_PARAM_TAG_RE.search(repaired):
            return True
        return bool(re.search(r"\[CMD:[a-zA-Z_]+\](?:\:[^\]]*)?$", repaired))

    def _normalize_intent_buffer(self) -> None:
        self._intent_buffer = self._repair_intent_buffer_text(self._intent_buffer)

    def feed_intent_chunk(self, chunk: str) -> Optional[CmdResult]:
        """流式累积 ChatResponse 文本并扫描 [CMD:xxx]（Level 3）。"""
        self._intent_buffer += str(chunk or "")
        self._normalize_intent_buffer()
        self._trim_intent_buffer_if_oversized()
        if self.intent_tag_tts_cut_needed():
            self._refresh_spoken_chars_before_tag()
            self._tag_tts_tail_pending = True
            self._tag_cut_urgency_snapshot = self.tag_tts_cut_urgency()
        return self.match_intent_tags(self._intent_buffer, allow_bare_param=False)

    def flush_param_cmd_from_buffer(self) -> Optional[CmdResult]:
        """TTSEnded 时重试：修复拆包标签并从口语补 arg。"""
        self._normalize_intent_buffer()
        return self.match_intent_tags(self._intent_buffer, allow_bare_param=True)

    def clear_matched_cmd_tags_from_buffer(self) -> None:
        """处理完 L3 指令后剥离标签文本，避免重复触发。"""
        self._intent_buffer = CMD_TAG_PATTERN.sub("", self._intent_buffer)

    def tag_tts_tail_pending(self) -> bool:
        return self._tag_tts_tail_pending

    def tag_cut_urgency(self) -> str:
        u = self._tag_cut_urgency_snapshot
        return u if u != "none" else self.tag_tts_cut_urgency()

    def estimated_spoken_ms_before_tag_suppress(
        self, *, played_ms: int = 0, urgency: str = ""
    ) -> int:
        """根据标签前口语字数估算应开始丢弃标签 TTS 的播放进度。"""
        u = urgency or self.tag_cut_urgency()
        n = self._spoken_chars_before_tag
        if n <= 0:
            base = 1800
        else:
            base = max(1200, min(7500, int(n * 165) - 350))
        if u == "boundary" and played_ms > 0:
            return min(base, played_ms + 250)
        if u in ("boundary", "full") and played_ms == 0:
            return base
        return base

    def confirm_playback_target_ms(self) -> int:
        """标签前口语应播完的时长（毫秒）；无标签锚点时用保守默认。"""
        n = self._spoken_chars_before_tag
        if n > 0:
            return max(900, min(5500, int(n * 165) + 250))
        return self.estimated_spoken_ms_before_tag_suppress(
            played_ms=0, urgency="full"
        ) + 450

    def period_cut_target_ms(self) -> int:
        """句号处截止：只播到标签前句号（含），用于截断 [CMD 尾音）。"""
        n = self._spoken_chars_before_tag
        if n > 0:
            # 略收紧预算，减少句号后 [CMD:...] 对应尾音混入播放队列
            return max(450, min(3800, int(n * 148) + 60))
        return 650

    def period_pcm_bytes_before_tag(
        self, *, sample_rate: int = 24000, channels: int = 1
    ) -> int:
        """句号截止对应的 PCM 字节上限（s16le）。"""
        ms = self.period_cut_target_ms()
        return max(0, int(sample_rate * channels * 2 * ms / 1000))

    def tag_tail_pcm_cap(
        self, *, sample_rate: int = 24000, channels: int = 1
    ) -> int:
        """标签尾音硬上限：取句号预算与确认语预算的较小值，防止 [CMD:...] 混入播放。"""
        period_b = self.period_pcm_bytes_before_tag(
            sample_rate=sample_rate, channels=channels
        )
        confirm_b = self.estimated_confirm_pcm_bytes_before_tag(
            sample_rate=sample_rate, channels=channels
        )
        if period_b <= 0:
            return confirm_b
        if confirm_b <= 0:
            return period_b
        return min(period_b, confirm_b)

    def estimated_confirm_pcm_bytes(
        self, *, sample_rate: int = 24000, channels: int = 1, played_ms: int = 0
    ) -> int:
        """标签前确认语对应的 PCM 字节上限（s16le）。"""
        _ = played_ms
        ms = self.confirm_playback_target_ms()
        return max(0, int(sample_rate * channels * 2 * ms / 1000))

    def estimated_confirm_pcm_bytes_before_tag(
        self, *, sample_rate: int = 24000, channels: int = 1
    ) -> int:
        """仅按「。[」前已识别字数」预算，用于截断标签 TTS。"""
        return self.estimated_confirm_pcm_bytes(
            sample_rate=sample_rate, channels=channels
        )

    def tag_tts_cut_urgency(self) -> str:
        """full=完整标签；boundary=。[ 或不完整 [CMD:；none=无需切断。"""
        buf = self._intent_buffer
        if not buf:
            return "none"
        if CMD_TAG_PATTERN.search(buf):
            return "full"
        if _CMD_TAG_PARTIAL_RE.search(buf) or _TAG_BOUNDARY_RE.search(buf):
            return "boundary"
        if buf.rstrip().endswith("["):
            return "boundary"
        return "none"

    def intent_tag_tts_cut_needed(self) -> bool:
        """ChatResponse 流中出现指令标签（含未收齐的前缀）时应切断 TTS，避免朗读标签。"""
        return self.tag_tts_cut_urgency() != "none"

    def intent_buffer_has_tag_marker(self) -> bool:
        return self.intent_tag_tts_cut_needed()

    def should_log_chat_chunk(self, chunk: str) -> bool:
        """指令标签及其分片（如 。[、CMD、music_play]）不打印到对话日志。"""
        s = str(chunk or "")
        if not s:
            return False
        buf = self._intent_buffer
        if (
            _CMD_TAG_PARTIAL_RE.search(buf)
            or _TAG_BOUNDARY_RE.search(buf)
            or buf.rstrip().endswith("[")
        ):
            return False
        if "[" in s:
            return False
        stripped = s.strip()
        if stripped and re.fullmatch(r"[\[:\]_a-zA-Z0-9\]]+", stripped):
            return False
        return True

    def get_command(self, cmd_id: str) -> Optional[Dict]:
        return self._registry.find(cmd_id)

    @property
    def match_mode(self) -> str:
        return self._match_mode

    def _coerce_volume_set_hit(self, hit: CmdResult, text: str) -> CmdResult:
        if hit.cmd_id != "volume_set":
            return hit
        norm = normalize_volume_arg(hit.arg, utterance=text)
        if not norm:
            return hit
        return CmdResult(
            cmd_id=hit.cmd_id,
            action=hit.action,
            reply=hit.reply,
            arg=norm,
        )

    def _coerce_massage_param_hit(
        self, hit: CmdResult, *, utterance: str = ""
    ) -> CmdResult:
        if hit.cmd_id not in _PARAMETRIC_CMD_IDS:
            return hit
        norm = normalize_massage_param_arg(
            hit.cmd_id, hit.arg, utterance=utterance
        )
        if not norm:
            return hit
        return CmdResult(
            cmd_id=hit.cmd_id,
            action=hit.action,
            reply=hit.reply,
            arg=norm,
        )

    def _finalize_hit(
        self, hit: Optional[CmdResult], *, utterance: str = ""
    ) -> Optional[CmdResult]:
        if hit is None:
            return None
        hit = self._coerce_volume_set_hit(hit, utterance)
        return self._coerce_massage_param_hit(hit, utterance=utterance)

    def match_asr(self, text: str) -> Optional[CmdResult]:
        """ASR-final 匹配：llm_tag/hybrid 仅白名单；asr 为全量本地规则。"""
        if self._match_mode in ("llm_tag", "hybrid"):
            if not self._asr_fallback_ids:
                return None
            hit = self._match_with_clauses(
                text, command_ids=self._asr_fallback_ids, allow_intent_tag=False
            )
            if hit is not None:
                return self._finalize_hit(hit, utterance=text)
            if "volume_set" in self._asr_fallback_ids:
                return try_match_volume_set_asr(text)
            return None
        hit = self.match(text)
        if hit is not None:
            return self._finalize_hit(hit, utterance=text)
        return try_match_volume_set_asr(text)

    def match(self, text: str) -> Optional[CmdResult]:
        """全量本地匹配（match_mode=asr 时用于 ASR-final）。"""
        return self._match_with_clauses(text, command_ids=None, allow_intent_tag=True)

    def _match_with_clauses(
        self,
        text: str,
        *,
        command_ids: Optional[AbstractSet[str]],
        allow_intent_tag: bool,
    ) -> Optional[CmdResult]:
        query = str(text or "")
        if not query:
            return None
        hit = self._match_once(
            query, command_ids=command_ids, allow_intent_tag=allow_intent_tag
        )
        if hit is not None:
            return hit
        norm = _normalize_utterance(query)
        if len(norm) > 6:
            for clause in reversed(re.split(r"[。！？；;，,\s]+", norm)):
                clause = clause.strip()
                if len(clause) < 2:
                    continue
                hit = self._match_once(
                    clause,
                    command_ids=command_ids,
                    allow_intent_tag=allow_intent_tag,
                )
                if hit is not None:
                    return hit
        return None

    def _commands_for_match(
        self, command_ids: Optional[AbstractSet[str]]
    ) -> List[Dict]:
        commands = self._registry.commands_by_priority
        if not command_ids:
            return commands
        allowed: Set[str] = {str(x) for x in command_ids}
        return [c for c in commands if str(c.get("id", "")) in allowed]

    def _match_once(
        self,
        text: str,
        *,
        command_ids: Optional[AbstractSet[str]] = None,
        allow_intent_tag: bool = True,
    ) -> Optional[CmdResult]:
        query = str(text or "")
        if not query:
            return None

        commands = self._commands_for_match(command_ids)

        # Level 1: keyword（最长关键词优先，避免「继续播放音乐」被「播放音乐」抢先匹配）
        best_kw_len = -1
        best_priority = -1
        best_arg: Optional[str] = None
        best_result: Optional[CmdResult] = None
        for cmd in commands:
            pri = int(cmd.get("priority", 0))
            for pattern in cmd.get("patterns", []):
                if pattern.get("type") != "keyword":
                    continue
                pattern_arg = str(pattern.get("arg") or "").strip() or None
                for kw in pattern.get("words", []):
                    kw_s = str(kw)
                    if not _keyword_matches(kw_s, query):
                        continue
                    kw_len = len(kw_s.strip())
                    if kw_len > best_kw_len or (kw_len == best_kw_len and pri > best_priority):
                        best_kw_len = kw_len
                        best_priority = pri
                        best_arg = pattern_arg
                        best_result = CmdResult(
                            cmd_id=str(cmd.get("id", "")),
                            action=str(cmd.get("action", "")),
                            reply=str(cmd.get("tts_confirm", "")),
                            arg=best_arg,
                        )
        if best_result is not None:
            return best_result

        # Level 2: regex（可选 capture_group 写入 arg）
        norm_query = _normalize_utterance(query)
        for cmd in commands:
            for pattern in cmd.get("patterns", []):
                if pattern.get("type") != "regex":
                    continue
                expr = str(pattern.get("expr", "")).strip()
                if not expr:
                    continue
                m = re.search(expr, norm_query) or re.search(expr, query)
                if not m:
                    continue
                arg: Optional[str] = None
                cap = pattern.get("capture_group")
                if cap is not None:
                    try:
                        gi = int(cap)
                        if gi > 0 and m.lastindex and gi <= m.lastindex:
                            captured = (m.group(gi) or "").strip()
                            prefix = str(pattern.get("arg_prefix") or "")
                            suffix = str(pattern.get("arg_suffix") or "")
                            arg = f"{prefix}{captured}{suffix}".strip() or None
                    except (TypeError, ValueError, IndexError):
                        arg = None
                if arg is None:
                    fixed = str(pattern.get("arg") or "").strip()
                    arg = fixed or None
                return CmdResult(
                    cmd_id=str(cmd.get("id", "")),
                    action=str(cmd.get("action", "")),
                    reply=str(cmd.get("tts_confirm", "")),
                    arg=arg,
                )

        # Level 3: intent tag（ASR 路径默认关闭，仅 ChatResponse 流使用 match_intent_tags）
        if not allow_intent_tag:
            return None
        for m in CMD_TAG_PATTERN.finditer(query):
            tag = m.group(1)
            arg = m.group(2)
            for cmd in commands:
                for pattern in cmd.get("patterns", []):
                    if pattern.get("type") == "intent_tag" and str(pattern.get("tag_name", "")) == tag:
                        return CmdResult(
                            cmd_id=str(cmd.get("id", "")),
                            action=str(cmd.get("action", "")),
                            reply=str(cmd.get("tts_confirm", "")).replace("{arg}", arg or ""),
                            arg=arg,
                        )

        return None

    def match_intent_tags(
        self, text: str, *, allow_bare_param: bool = False
    ) -> Optional[CmdResult]:
        """仅匹配 [CMD:xxx] 标签（用于 ChatResponse，避免闲聊文本误触发关键词）。"""
        query = self._repair_intent_buffer_text(str(text or ""))
        if not query:
            return None
        commands = self._registry.commands_by_priority
        for m in CMD_TAG_PATTERN.finditer(query):
            tag = m.group(1)
            arg = m.group(2)
            for cmd in commands:
                for pattern in cmd.get("patterns", []):
                    if pattern.get("type") == "intent_tag" and str(pattern.get("tag_name", "")) == tag:
                        hit = CmdResult(
                            cmd_id=str(cmd.get("id", "")),
                            action=str(cmd.get("action", "")),
                            reply=str(cmd.get("tts_confirm", "")).replace("{arg}", arg or ""),
                            arg=arg,
                        )
                        utterance = query[: m.start()].strip()
                        hit = self._coerce_massage_param_hit(
                            hit, utterance=utterance
                        )
                        if hit.cmd_id in _PARAMETRIC_CMD_IDS and not (
                            hit.arg or ""
                        ).strip():
                            if not allow_bare_param and self._intent_buffer_may_extend(
                                query
                            ):
                                return None
                            if not allow_bare_param:
                                return None
                        return hit
        return None
