from __future__ import annotations

import asyncio
from dataclasses import dataclass


@dataclass
class AudioPlayerConfig:
    backend: str = "pulseaudio"
    device: str = ""
    sample_rate: int = 24000
    channels: int = 1


class AudioPlayer:
    def __init__(self, cfg: AudioPlayerConfig):
        self._cfg = cfg
        self._queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=256)
        self._running = False

    async def start(self) -> None:
        self._running = True

    async def stop(self) -> None:
        self._running = False
        while not self._queue.empty():
            _ = await self._queue.get()

    async def push(self, pcm_chunk: bytes) -> bool:
        if not self._running or not pcm_chunk:
            return False
        if self._queue.full():
            return False
        await self._queue.put(pcm_chunk)
        return True
