from __future__ import annotations

import threading


class PcmRingBuffer:
    """固定容量的 PCM 字节环形缓冲（保留最近 max_bytes）。"""

    def __init__(self, max_bytes: int):
        self._max_bytes = max(1, int(max_bytes))
        self._buf = bytearray()
        self._lock = threading.Lock()

    def write(self, data: bytes) -> None:
        if not data:
            return
        with self._lock:
            self._buf.extend(data)
            if len(self._buf) > self._max_bytes:
                self._buf = self._buf[-self._max_bytes :]

    def snapshot(self) -> bytes:
        with self._lock:
            return bytes(self._buf)

    def clear(self) -> None:
        with self._lock:
            self._buf.clear()
