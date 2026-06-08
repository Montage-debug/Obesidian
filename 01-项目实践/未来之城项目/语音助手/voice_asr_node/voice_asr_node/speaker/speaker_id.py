from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from ..config.app_config import SpeakerIdConfig

logger = logging.getLogger(__name__)


def _cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=np.float32).reshape(-1)
    b = np.asarray(b, dtype=np.float32).reshape(-1)
    if a.size == 0 or b.size == 0:
        return 0.0
    denom = float(np.linalg.norm(a) * np.linalg.norm(b))
    if denom < 1e-8:
        return 0.0
    return float(np.dot(a, b) / denom)


class SpeakerIdentifier:
    """本地声纹识别（Sherpa-ONNX SpeakerEmbeddingExtractor，16kHz PCM）。"""

    def __init__(self, cfg: SpeakerIdConfig):
        self._cfg = cfg
        self._extractor = None
        self._profiles: Dict[str, np.ndarray] = {}
        self._ready = False
        self._warned_missing_model = False

    @property
    def ready(self) -> bool:
        return self._ready

    def init(self) -> bool:
        if not self._cfg.enabled:
            logger.info("[speaker_id] disabled in config")
            return False

        model_path = self._cfg.model_path
        if not model_path or not os.path.isfile(model_path):
            if not self._warned_missing_model:
                logger.warning(
                    "[speaker_id] model not found: %s "
                    "(run scripts/download_speaker_model.sh or set speaker_id.enabled: false)",
                    model_path,
                )
                self._warned_missing_model = True
            return False

        try:
            import sherpa_onnx
        except ImportError as e:
            logger.warning("[speaker_id] sherpa-onnx not installed: %s", e)
            return False

        try:
            config = sherpa_onnx.SpeakerEmbeddingExtractorConfig(
                model=model_path,
                num_threads=2,
            )
            self._extractor = sherpa_onnx.SpeakerEmbeddingExtractor(config)
        except Exception as e:
            logger.error("[speaker_id] failed to load model %s: %s", model_path, e)
            return False

        self._load_profiles()
        self._ready = True
        logger.info(
            "[speaker_id] ready profiles=%d model=%s",
            len(self._profiles),
            model_path,
        )
        return True

    def _load_profiles(self) -> None:
        self._profiles.clear()
        profiles_dir = self._cfg.profiles_dir
        if not profiles_dir or not os.path.isdir(profiles_dir):
            os.makedirs(profiles_dir, exist_ok=True)
            return

        for path in Path(profiles_dir).glob("*.npy"):
            name = path.stem
            try:
                emb = np.load(str(path))
                self._profiles[name] = np.asarray(emb, dtype=np.float32).reshape(-1)
            except Exception as e:
                logger.warning("[speaker_id] skip profile %s: %s", path, e)

    def extract_embedding(self, pcm_int16: np.ndarray) -> np.ndarray:
        if self._extractor is None:
            raise RuntimeError("speaker_id not initialized")

        samples = np.asarray(pcm_int16, dtype=np.float32).reshape(-1) / 32768.0
        stream = self._extractor.create_stream()
        stream.accept_waveform(16000, samples)
        stream.input_finished()
        emb = self._extractor.compute(stream)
        return np.asarray(emb, dtype=np.float32).reshape(-1)

    def identify(self, embedding: np.ndarray) -> Optional[str]:
        if not self._profiles:
            return None

        best_name = ""
        best_score = -1.0
        for name, ref in self._profiles.items():
            score = _cosine_similarity(embedding, ref)
            if score > best_score:
                best_score = score
                best_name = name

        if best_score < self._cfg.similarity_threshold:
            if self._cfg.reject_unknown:
                logger.info(
                    "[speaker_id] unknown speaker (best=%.3f < %.3f)",
                    best_score,
                    self._cfg.similarity_threshold,
                )
            return None

        logger.info("[speaker_id] identified %s (score=%.3f)", best_name, best_score)
        return best_name

    def identify_pcm(self, pcm_bytes: bytes, *, sample_rate: int = 16000) -> Optional[str]:
        if not self._ready or not pcm_bytes:
            return None
        if sample_rate != 16000:
            logger.warning("[speaker_id] expected 16kHz PCM, got %s", sample_rate)
        min_samples = int(16000 * self._cfg.min_audio_ms / 1000)
        arr = np.frombuffer(pcm_bytes, dtype=np.int16)
        if arr.size < min_samples:
            logger.debug(
                "[speaker_id] audio too short: %d samples < %d",
                arr.size,
                min_samples,
            )
            return None
        try:
            emb = self.extract_embedding(arr)
            return self.identify(emb)
        except Exception as e:
            logger.warning("[speaker_id] identify failed: %s", e)
            return None

    def register(self, name: str, pcm_int16: np.ndarray) -> bool:
        if not self._ready:
            if not self.init():
                return False

        min_samples = int(16000 * self._cfg.register_min_audio_ms / 1000)
        arr = np.asarray(pcm_int16, dtype=np.int16).reshape(-1)
        if arr.size < min_samples:
            raise ValueError(
                f"audio too short for register: {arr.size} samples < {min_samples}"
            )

        emb = self.extract_embedding(arr)
        profiles_dir = self._cfg.profiles_dir
        os.makedirs(profiles_dir, exist_ok=True)
        out_path = os.path.join(profiles_dir, f"{name}.npy")
        np.save(out_path, emb)
        self._profiles[name] = emb.reshape(-1)
        logger.info("[speaker_id] registered %s -> %s", name, out_path)
        return True
