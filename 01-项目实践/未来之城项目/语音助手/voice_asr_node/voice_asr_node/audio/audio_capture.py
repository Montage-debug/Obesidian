from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional


AudioCallback = Callable[[bytes], None]


@dataclass
class AudioCaptureConfig:
    sample_rate: int = 16000
    channels: int = 1
    chunk_ms: int = 20
    device: str = ""


class AudioCapture:
    """
    音频采集骨架实现。
    当前只保留接口和生命周期，后续将逐步替换为 pyaudio/sounddevice 回调桥接。
    """

    def __init__(self, cfg: AudioCaptureConfig):
        self._cfg = cfg
        self._callback: Optional[AudioCallback] = None
        self._running = False

    def set_callback(self, callback: AudioCallback) -> None:
        self._callback = callback

    def start(self) -> None:
        self._running = True

    def stop(self) -> None:
        self._running = False

    @property
    def running(self) -> bool:
        return self._running
