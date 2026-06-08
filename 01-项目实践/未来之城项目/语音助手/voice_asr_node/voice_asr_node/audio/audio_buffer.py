from __future__ import annotations

import asyncio
from queue import Queue, Empty
from typing import Optional


class AudioBuffer:
    def __init__(self, maxsize: int = 256):
        self._queue: Queue[bytes] = Queue(maxsize=maxsize)

    def push(self, chunk: bytes) -> bool:
        if not chunk:
            return False
        if self._queue.full():
            return False
        self._queue.put(chunk)
        return True

    def pop_nowait(self) -> Optional[bytes]:
        try:
            return self._queue.get_nowait()
        except Empty:
            return None

    async def pop_async(self, timeout_s: float = 0.2) -> Optional[bytes]:
        end_time = asyncio.get_running_loop().time() + max(timeout_s, 0.0)
        while asyncio.get_running_loop().time() < end_time:
            item = self.pop_nowait()
            if item is not None:
                return item
            await asyncio.sleep(0.01)
        return None
