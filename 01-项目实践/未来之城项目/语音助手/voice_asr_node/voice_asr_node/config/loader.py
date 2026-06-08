from pathlib import Path
from typing import Any, Dict

import yaml

from .app_config import _resolve_config_file
from .schema import validate_doubao_config


def _discover_default_config_path() -> str:
    return _resolve_config_file("doubao.yaml")


def load_doubao_config(user_path: str = "") -> Dict[str, Any]:
    config_path = user_path.strip() if user_path else _discover_default_config_path()
    if not config_path or not Path(config_path).is_file():
        raise FileNotFoundError(f"doubao config not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    return validate_doubao_config(cfg)
