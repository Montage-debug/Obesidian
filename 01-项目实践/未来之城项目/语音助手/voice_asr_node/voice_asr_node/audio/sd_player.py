from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import sounddevice as sd
except ImportError as e:  # pragma: no cover
    raise RuntimeError("missing dependency: sounddevice") from e


@dataclass(frozen=True)
class PlayerConfig:
    playback_device: str = ""
    sample_rate: int = 24000
    channels: int = 1


class SoundDevicePlayer:
    """
    Audio playback using sounddevice.RawOutputStream (callback).

    Write int16 PCM bytes via `enqueue()`. Callback 用静音补齐欠载，避免 ALSA underrun。
    """

    def __init__(self, cfg: PlayerConfig, *, logger, on_first_chunk=None, queue_max: int = 512):
        self._cfg = cfg
        self._logger = logger
        self._q: "queue.Queue[bytes]" = queue.Queue(maxsize=queue_max)
        self._pending = bytearray()
        self._pending_lock = threading.Lock()
        self._stop = False
        self._flush_req = False
        self._on_first_chunk = on_first_chunk
        self._seen_first = False
        self._bytes_played = 0
        self._bytes_in_queue = 0
        self._block_frames = max(256, int(self._cfg.sample_rate * 0.04))

    def qsize(self) -> int:
        return self._q.qsize()

    def is_idle(self) -> bool:
        with self._pending_lock:
            return self._q.empty() and not self._pending

    def queued_bytes(self) -> int:
        with self._pending_lock:
            return max(0, self._bytes_in_queue) + len(self._pending)

    def clear_queue(self) -> int:
        cleared = 0
        while True:
            try:
                self._q.get_nowait()
                cleared += 1
            except queue.Empty:
                break
        with self._pending_lock:
            self._pending.clear()
            self._bytes_in_queue = 0
        return cleared

    def trim_queue_keep_bytes(self, keep_bytes: int) -> int:
        """保留队列头部 keep_bytes 字节的 PCM，丢弃其余（用于切掉标签尾音）。"""
        keep = max(0, int(keep_bytes))
        chunks: list[bytes] = []
        while True:
            try:
                chunks.append(self._q.get_nowait())
            except queue.Empty:
                break
        with self._pending_lock:
            if self._pending:
                chunks.insert(0, bytes(self._pending))
                self._pending.clear()
            if not chunks:
                self._bytes_in_queue = 0
                return 0
            merged = b"".join(chunks)
            if len(merged) <= keep:
                for c in chunks:
                    self._q.put_nowait(c)
                self._bytes_in_queue = len(merged)
                return 0
            kept = merged[:keep]
            dropped = len(merged) - keep
            if kept:
                self._q.put_nowait(kept)
            self._bytes_in_queue = len(kept)
            return dropped

    def flush(self) -> int:
        """清空待播队列并尽快停止当前正在输出的音频（用于用户打断）。"""
        cleared = self.clear_queue()
        self._flush_req = True
        if cleared:
            self._logger.debug(
                f"[playback] flush requested: cleared={cleared} "
                f"played_ms={self.played_ms()}"
            )
        return cleared

    @property
    def bytes_played(self) -> int:
        return self._bytes_played

    def played_ms(self) -> int:
        denom = self._cfg.sample_rate * self._cfg.channels * 2
        return int(self._bytes_played * 1000 / denom) if denom else 0

    def reset_playback_stats(self) -> None:
        self._bytes_played = 0

    def stop(self) -> None:
        self._stop = True
        self.flush()

    def enqueue(self, pcm: bytes) -> None:
        if not pcm:
            return
        self._bytes_in_queue += len(pcm)
        try:
            self._q.put_nowait(pcm)
        except queue.Full:
            try:
                _ = self._q.get_nowait()
            except Exception:
                pass
            try:
                self._q.put_nowait(pcm)
            except Exception:
                pass

    def _pull_pcm(self, nbytes: int) -> tuple[bytes, int]:
        if nbytes <= 0:
            return b"", 0
        out = bytearray()
        from_source = 0
        while len(out) < nbytes:
            with self._pending_lock:
                if self._pending:
                    take = min(nbytes - len(out), len(self._pending))
                    out.extend(self._pending[:take])
                    del self._pending[:take]
                    from_source += take
                    continue
            try:
                chunk = self._q.get_nowait()
            except queue.Empty:
                break
            self._bytes_in_queue = max(0, self._bytes_in_queue - len(chunk))
            with self._pending_lock:
                self._pending.extend(chunk)
        if len(out) < nbytes:
            out.extend(b"\x00" * (nbytes - len(out)))
        return bytes(out), from_source

    def _audio_callback(self, outdata, frames, _time_info, status) -> None:
        _ = (frames, status)
        if self._flush_req:
            outdata[:] = b"\x00" * len(outdata)
            with self._pending_lock:
                self._pending.clear()
            self._flush_req = False
            return
        pcm, played = self._pull_pcm(len(outdata))
        outdata[:] = pcm
        if played > 0:
            self._bytes_played += played
            if not self._seen_first:
                self._seen_first = True
                if self._on_first_chunk:
                    try:
                        self._on_first_chunk()
                    except Exception:
                        pass

    def _resolve_audio_device(self, requested_device, is_input: bool):
        if requested_device:
            return requested_device
        try:
            default_index = sd.default.device[0] if is_input else sd.default.device[1]
            info = sd.query_devices(default_index)
            self._logger.info(
                f"[audio] default {'input' if is_input else 'output'} device: {info['name']} (index={default_index})"
            )
            return default_index
        except Exception:
            return requested_device

    def run_forever(self) -> None:
        device = self._resolve_audio_device(self._cfg.playback_device, is_input=False)
        while not self._stop:
            try:
                with sd.RawOutputStream(
                    samplerate=self._cfg.sample_rate,
                    channels=self._cfg.channels,
                    dtype="int16",
                    blocksize=self._block_frames,
                    latency="high",
                    device=device,
                    callback=self._audio_callback,
                ):
                    while not self._stop:
                        time.sleep(0.1)
            except Exception as e:
                self._logger.error(f"[playback] stream error: {e}, retry in 1s")
                time.sleep(1.0)
