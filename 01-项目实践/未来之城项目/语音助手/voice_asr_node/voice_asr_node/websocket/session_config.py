from __future__ import annotations

from typing import Any, Dict, List

from .payload_builder import build_start_session_payload as _legacy_build_start_payload


def build_start_session_payload(cfg: Dict[str, Any], command_hotwords: List[str]) -> Dict[str, Any]:
    return _legacy_build_start_payload(cfg, command_hotwords)
