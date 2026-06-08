"""豆包 Realtime API 鉴权（内置，勿写入 doubao.yaml）。"""
from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class DoubaoAuthConfig:
    api_key: str
    resource_id: str
    app_key: str
    ws_url: str


_API_KEY = "1b9e271a-0a90-427e-a1d1-63317ea5844c"
_RESOURCE_ID = "volc.speech.dialog"
_APP_KEY = "PlgvMymc7f3tQnJ6"
_WS_URL = "wss://openspeech.bytedance.com/api/v3/realtime/dialogue"


def get_doubao_auth() -> DoubaoAuthConfig:
    api_key = os.environ.get("DOUBAO_API_KEY", "").strip() or _API_KEY
    return DoubaoAuthConfig(
        api_key=api_key,
        resource_id=_RESOURCE_ID,
        app_key=_APP_KEY,
        ws_url=_WS_URL,
    )
