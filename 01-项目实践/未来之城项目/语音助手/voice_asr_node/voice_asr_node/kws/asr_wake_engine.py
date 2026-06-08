from __future__ import annotations

import logging
import queue
import threading
from dataclasses import dataclass
from typing import Callable, List, Optional

from ..config.app_config import AppConfig, WakewordConfig
from .keyword_spotter import KeywordSpotterConfig, KeywordSpotterRecognizer

_KWS_ENGINES = frozenset({"sherpa_onnx_kws", "kws", "keyword_spotter"})

logger = logging.getLogger(__name__)


@dataclass
class WakeEvent:
    keyword: str
    language: str


class ASRWakeEngine:
    """多语言本地唤醒：Sherpa-ONNX KeywordSpotter（见 config/doubao.yaml wakeword）。"""

    def __init__(
        self,
        cfg: WakewordConfig,
        *,
        models_root: str = "",
        on_wake: Optional[Callable[[str, str], None]] = None,
    ):
        self._cfg = cfg
        self._models_root = models_root
        self._on_wake = on_wake
        self._wake_queue: queue.Queue = queue.Queue(maxsize=32)
        self._recognizers: List[KeywordSpotterRecognizer] = []
        self._lock = threading.Lock()
        self._started = False

    @classmethod
    def from_app_config(cls, app_cfg: AppConfig, **kwargs) -> "ASRWakeEngine":
        return cls(app_cfg.wakeword, models_root=app_cfg.models_root, **kwargs)

    def init(self) -> bool:
        if not self._cfg.enabled:
            logger.info("[ASRWakeEngine] disabled in config")
            return False

        engine = self._cfg.engine.lower()
        if engine not in _KWS_ENGINES:
            logger.error(
                "[ASRWakeEngine] unsupported engine %r (supported: %s)",
                self._cfg.engine,
                sorted(_KWS_ENGINES),
            )
            return False

        self._recognizers.clear()
        for lang in self._cfg.active_languages:
            model_cfg = self._cfg.models.get(lang)
            if model_cfg is None:
                logger.warning("[ASRWakeEngine] no model config for language '%s', skip", lang)
                continue
            try:
                paths = model_cfg.resolve_paths(self._models_root)
                rec = KeywordSpotterRecognizer(
                    KeywordSpotterConfig(
                        language=lang,
                        encoder=paths["encoder"],
                        decoder=paths["decoder"],
                        joiner=paths["joiner"],
                        tokens=paths["tokens"],
                        keywords_file=paths.get("keywords_file", ""),
                        keywords=list(model_cfg.keywords),
                        lexicon=paths.get("lexicon", ""),
                        tokens_type=model_cfg.tokens_type,
                        num_threads=model_cfg.num_threads,
                        keywords_threshold=self._cfg.kws_threshold,
                        keywords_score=model_cfg.keywords_score,
                        max_active_paths=model_cfg.max_active_paths,
                        num_trailing_blanks=model_cfg.num_trailing_blanks,
                        input_gain=self._cfg.input_gain,
                    ),
                    self._wake_queue,
                    on_wake=self._on_wake,
                )
                self._recognizers.append(rec)
                logger.info(
                    "[ASRWakeEngine] loaded lang=%s encoder=%s keywords=%s",
                    lang,
                    paths["encoder"],
                    model_cfg.keywords,
                )
            except Exception as e:
                logger.error("[ASRWakeEngine] failed to load lang=%s: %s", lang, e)
        return len(self._recognizers) > 0

    @property
    def enabled(self) -> bool:
        return bool(self._recognizers)

    @property
    def wake_keywords(self) -> List[str]:
        words: List[str] = []
        for rec in self._recognizers:
            words.extend(rec._cfg.keywords)
        return sorted(set(words))

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            for rec in self._recognizers:
                rec.start()
            self._started = True

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            for rec in self._recognizers:
                rec.stop()
            self._started = False

    def feed_audio(self, pcm_int16: bytes) -> None:
        if not self._started:
            return
        for rec in self._recognizers:
            rec.feed_audio(pcm_int16)

    def poll_wake(self) -> Optional[WakeEvent]:
        try:
            keyword, lang = self._wake_queue.get_nowait()
            return WakeEvent(keyword=keyword, language=lang)
        except queue.Empty:
            return None
