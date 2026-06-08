#!/usr/bin/env python3
"""注册声纹档案：从 16kHz mono WAV 提取 embedding 并保存到 profiles 目录。"""

from __future__ import annotations

import argparse
import sys
import wave

import numpy as np

from ..config.app_config import load_app_config
from ..speaker.speaker_id import SpeakerIdentifier


def _load_wav_pcm16(path: str) -> tuple[np.ndarray, int]:
    with wave.open(path, "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        rate = wf.getframerate()
        if sample_width != 2:
            raise ValueError(f"expected 16-bit WAV, got sample width {sample_width}")
        if channels != 1:
            raise ValueError(f"expected mono WAV, got {channels} channels")
        raw = wf.readframes(wf.getnframes())
    return np.frombuffer(raw, dtype=np.int16), rate


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Register speaker voice profile (ECAPA)")
    parser.add_argument("--name", required=True, help="Speaker display name / profile id")
    parser.add_argument("--wav", required=True, help="Path to 16kHz mono PCM WAV")
    parser.add_argument(
        "--doubao-config",
        default="",
        help="Path to doubao.yaml (default: MR_DOUBAO_CONFIG or package default)",
    )
    args = parser.parse_args(argv)

    app_cfg = load_app_config(doubao_path=args.doubao_config)
    sid_cfg = app_cfg.speaker_id
    sid_cfg.enabled = True

    pcm, rate = _load_wav_pcm16(args.wav)
    if rate != 16000:
        print(f"warning: resampling not implemented, WAV is {rate}Hz (expected 16000)", file=sys.stderr)

    identifier = SpeakerIdentifier(sid_cfg)
    if not identifier.init():
        print("failed to initialize speaker model", file=sys.stderr)
        return 1

    try:
        identifier.register(args.name, pcm)
    except Exception as e:
        print(f"register failed: {e}", file=sys.stderr)
        return 1

    print(f"registered speaker '{args.name}' in {sid_cfg.profiles_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
