"""Sherpa-ONNX KeywordSpotter 唤醒（与 zh_cn 目录模型同类，适合英文/定制唤醒词）。"""

from __future__ import annotations

import logging
import os
import queue
import threading
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np

try:
    import sherpa_onnx
except ImportError as e:  # pragma: no cover
    raise RuntimeError("missing dependency: sherpa-onnx") from e

from .keywords_builder import build_keywords_file

logger = logging.getLogger(__name__)


def _spotter_keyword(result) -> str:
    if isinstance(result, str):
        return result.strip()
    kw = getattr(result, "keyword", None)
    if kw is not None:
        return str(kw).strip()
    return str(result).strip()


@dataclass
class KeywordSpotterConfig:
    language: str
    encoder: str
    decoder: str
    joiner: str
    tokens: str
    keywords_file: str
    keywords: List[str] = field(default_factory=list)
    lexicon: str = ""
    tokens_type: str = "auto"
    num_threads: int = 2
    keywords_threshold: float = 0.25
    keywords_score: float = 1.0
    max_active_paths: int = 4
    num_trailing_blanks: int = 1
    sample_rate: int = 16000
    input_gain: float = 1.0
    stream_reset_sec: float = 30.0


class KeywordSpotterRecognizer:
    """单语言 KeywordSpotter 唤醒线程。"""

    def __init__(
        self,
        cfg: KeywordSpotterConfig,
        wake_queue: queue.Queue,
        *,
        on_wake: Optional[Callable[[str, str], None]] = None,
    ):
        self._cfg = cfg
        self._wake_queue = wake_queue
        self._on_wake = on_wake
        self._pcm_queue: queue.Queue = queue.Queue(maxsize=500)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_reset_ts = 0.0

        for path in (cfg.encoder, cfg.decoder, cfg.joiner, cfg.tokens):
            if not os.path.isfile(path):
                raise FileNotFoundError(
                    f"[KeywordSpotter:{cfg.language}] model file not found: {path}"
                )

        kw_file = build_keywords_file(
            keywords=list(cfg.keywords),
            tokens_path=cfg.tokens,
            lexicon_path=cfg.lexicon,
            tokens_type=cfg.tokens_type,
            out_path=cfg.keywords_file,
        )

        self._spotter = sherpa_onnx.KeywordSpotter(
            tokens=cfg.tokens,
            encoder=cfg.encoder,
            decoder=cfg.decoder,
            joiner=cfg.joiner,
            keywords_file=kw_file,
            keywords_threshold=cfg.keywords_threshold,
            keywords_score=cfg.keywords_score,
            num_trailing_blanks=cfg.num_trailing_blanks,
            max_active_paths=cfg.max_active_paths,
            num_threads=cfg.num_threads,
            sample_rate=cfg.sample_rate,
            feature_dim=80,
            provider="cpu",
        )
        self._stream = self._spotter.create_stream()
        self._keywords_file = kw_file
        logger.info(
            "[KeywordSpotter:%s] ready keywords_file=%s threshold=%.2f",
            cfg.language,
            kw_file,
            cfg.keywords_threshold,
        )

    @property
    def language(self) -> str:
        return self._cfg.language

    def feed_audio(self, pcm_int16: bytes) -> None:
        if not self._running:
            return
        try:
            self._pcm_queue.put_nowait(pcm_int16)
        except queue.Full:
            pass

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        import time

        self._last_reset_ts = time.time()
        self._thread = threading.Thread(
            target=self._run_loop,
            daemon=True,
            name=f"kws-spotter-{self._cfg.language}",
        )
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self._pcm_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._stream = self._spotter.create_stream()

    def _reset_stream(self) -> None:
        import time

        self._stream = self._spotter.create_stream()
        self._last_reset_ts = time.time()

    def _run_loop(self) -> None:
        import time

        while self._running:
            try:
                raw = self._pcm_queue.get()
                if raw is None:
                    break

                if (
                    self._cfg.stream_reset_sec > 0
                    and time.time() - self._last_reset_ts > self._cfg.stream_reset_sec
                ):
                    self._reset_stream()

                samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                if self._cfg.input_gain != 1.0:
                    samples = np.clip(samples * self._cfg.input_gain, -1.0, 1.0)

                self._stream.accept_waveform(self._cfg.sample_rate, samples)
                while self._spotter.is_ready(self._stream):
                    self._spotter.decode_stream(self._stream)

                keyword = _spotter_keyword(self._spotter.get_result(self._stream))
                if not keyword:
                    continue

                display = keyword
                if display.startswith("@"):
                    display = display[1:].replace("_", " ")

                logger.info(
                    "[KeywordSpotter:%s] wake hit keyword=%r",
                    self._cfg.language,
                    display,
                )
                self._reset_stream()
                try:
                    self._wake_queue.put_nowait((display, self._cfg.language))
                except queue.Full:
                    pass
                if self._on_wake is not None:
                    try:
                        self._on_wake(display, self._cfg.language)
                    except Exception:
                        logger.exception("[KeywordSpotter] on_wake callback failed")
            except Exception:
                if self._running:
                    logger.exception(
                        "[KeywordSpotter:%s] run loop error", self._cfg.language
                    )
