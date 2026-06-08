from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from math import gcd
from typing import Callable, Optional

import numpy as np

try:
    import sounddevice as sd
except ImportError as e:  # pragma: no cover
    raise RuntimeError("missing dependency: sounddevice") from e

try:
    from scipy.signal import resample_poly

    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


@dataclass(frozen=True)
class CaptureConfig:
    mic_device: str = ""
    target_sample_rate: int = 16000
    channels: int = 1
    frame_ms: int = 20


class SoundDeviceCapture:
    """
    Microphone capture using sounddevice.RawInputStream.

    Delivers 20ms frames at target_sample_rate as int16 PCM bytes.
    """

    def __init__(self, cfg: CaptureConfig, *, logger, on_frame: Callable[[bytes], None]):
        self._cfg = cfg
        self._logger = logger
        self._on_frame = on_frame

        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def _get_device_sample_rate(self, device) -> int:
        try:
            info = sd.query_devices(device)
            return int(info.get("default_samplerate", self._cfg.target_sample_rate))
        except Exception:
            return self._cfg.target_sample_rate

    def _detect_native_input_samplerate(self, device, fallback_sr: int) -> int:
        """
        For PulseAudio virtual devices, attempt to open with the native USB source sample rate
        to avoid double resampling (48k -> 44.1k -> 16k).
        """
        try:
            info = sd.query_devices(device)
            name = str(info.get("name", "")).lower()
            if "pulse" not in name and name != "default":
                return fallback_sr
            result = subprocess.run(["pactl", "get-default-source"], capture_output=True, text=True, timeout=2)
            if result.returncode != 0:
                return fallback_sr
            src_name = result.stdout.strip()
            if not src_name:
                return fallback_sr
            result2 = subprocess.run(["pactl", "list", "sources"], capture_output=True, text=True, timeout=5)
            in_target = False
            for line in result2.stdout.splitlines():
                stripped = line.strip()
                if f"Name: {src_name}" in stripped:
                    in_target = True
                if in_target and "Sample Specification" in stripped:
                    for token in stripped.split():
                        if token.endswith("Hz") and token[:-2].isdigit():
                            native_sr = int(token[:-2])
                            if native_sr != fallback_sr:
                                self._logger.info(
                                    f"[mic] USB 源原生采样率 {native_sr}Hz，替代 pulse 默认 {fallback_sr}Hz，避免 PulseAudio 内部重采样"
                                )
                            return native_sr
                    break
        except Exception:
            pass
        return fallback_sr

    def _resolve_audio_device(self, requested_device, is_input: bool):
        if requested_device:
            return requested_device

        try:
            default_index = sd.default.device[0] if is_input else sd.default.device[1]
            default_info = sd.query_devices(default_index)
            self._logger.info(
                f"[audio] default {'input' if is_input else 'output'} device: {default_info['name']} (index={default_index})"
            )
        except Exception:
            default_index = None

        try:
            devices = sd.query_devices()
            candidates = []
            for index, info in enumerate(devices):
                channels = info["max_input_channels"] if is_input else info["max_output_channels"]
                if channels <= 0:
                    continue
                name = str(info["name"]).lower()
                if "pulse" in name:
                    candidates.append((0, channels, index, info))
                elif "usb" in name:
                    candidates.append((1, channels, index, info))
                elif "default" in name:
                    candidates.append((2, channels, index, info))
            if candidates:
                candidates.sort(key=lambda x: (x[0], x[1]))
                _, _, index, info = candidates[0]
                self._logger.info(
                    f"[audio] auto-select {'input' if is_input else 'output'} device: {info['name']} (index={index})"
                )
                return index
        except Exception as e:
            self._logger.warn(f"[audio] failed to query sounddevice devices: {e}")

        return default_index

    def _resample_to_target(self, pcm_bytes: bytes, src_rate: int) -> bytes:
        dst_rate = self._cfg.target_sample_rate
        if src_rate == dst_rate:
            return pcm_bytes

        samples = np.frombuffer(pcm_bytes, dtype=np.int16).astype(np.float32)
        if _HAS_SCIPY:
            g = gcd(src_rate, dst_rate)
            resampled = resample_poly(samples, dst_rate // g, src_rate // g)
        else:
            src_len = len(samples)
            if src_len == 0:
                return b""
            target_len = int(round(src_len * dst_rate / src_rate))
            if target_len <= 0:
                return b""
            src_x = np.linspace(0.0, 1.0, src_len)
            dst_x = np.linspace(0.0, 1.0, target_len)
            resampled = np.interp(dst_x, src_x, samples)

        resampled = np.clip(resampled, -32768, 32767).astype(np.int16)
        return resampled.tobytes()

    def run_forever(self) -> None:
        device = self._resolve_audio_device(self._cfg.mic_device, is_input=True)
        device_sr = self._get_device_sample_rate(device)
        device_sr = self._detect_native_input_samplerate(device, device_sr)

        blocksize = int(device_sr * self._cfg.frame_ms / 1000)
        frame_bytes = int(self._cfg.target_sample_rate * self._cfg.frame_ms / 1000) * 2

        try:
            info = sd.query_devices(device)
            self._logger.info(
                f"[mic] open device index={device} name={info['name']} samplerate={device_sr} blocksize={blocksize}"
            )
        except Exception:
            self._logger.info(f"[mic] open device index={device} samplerate={device_sr} blocksize={blocksize}")

        last_status_log = 0.0

        def cb(indata, frames, _time_info, status):
            nonlocal last_status_log
            if status:
                now = time.time()
                if now - last_status_log > 1.0:
                    self._logger.warn(f"[mic] callback status: {status}")
                    last_status_log = now

            data = bytes(indata)
            if not data:
                return

            if device_sr != self._cfg.target_sample_rate:
                data = self._resample_to_target(data, device_sr)

            if len(data) % frame_bytes != 0:
                keep = (len(data) // frame_bytes) * frame_bytes
                data = data[:keep]
                if not data:
                    return

            for i in range(0, len(data), frame_bytes):
                if self._stop:
                    return
                self._on_frame(data[i : i + frame_bytes])

        while not self._stop:
            try:
                with sd.RawInputStream(
                    samplerate=device_sr,
                    channels=self._cfg.channels,
                    dtype="int16",
                    blocksize=blocksize,
                    callback=cb,
                    device=device,
                ):
                    while not self._stop:
                        time.sleep(0.2)
            except Exception as e:
                self._logger.error(f"[mic] stream error: {e}, retry in 1s")
                time.sleep(1.0)

