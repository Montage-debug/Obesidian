from typing import Any, Dict


_ALLOWED_MODEL_VERSIONS = {"1.2.1.1", "2.2.0.0"}
_ALLOWED_INPUT_MODS = {"microphone", "push_to_talk", "text", "audio_file", "keep_alive"}
_ALLOWED_WEBSEARCH_TYPES = {"web", "web_summary", "web_agent"}


class ConfigError(ValueError):
    pass


def validate_doubao_config(cfg: Dict[str, Any]) -> Dict[str, Any]:
    model = cfg.setdefault("model", {})
    version = str(model.get("version", "")).strip()
    if version not in _ALLOWED_MODEL_VERSIONS:
        raise ConfigError(
            f"model.version must be one of {sorted(_ALLOWED_MODEL_VERSIONS)}, got: {version!r}"
        )

    session_extra = cfg.setdefault("session", {}).setdefault("extra", {})
    input_mod = str(session_extra.get("input_mod", "keep_alive")).strip()
    if input_mod not in _ALLOWED_INPUT_MODS:
        raise ConfigError(
            f"session.extra.input_mod must be one of {sorted(_ALLOWED_INPUT_MODS)}, got: {input_mod!r}"
        )

    web = cfg.setdefault("websearch", {})
    web_type = str(web.get("type", "web")).strip()
    if web_type not in _ALLOWED_WEBSEARCH_TYPES:
        raise ConfigError(
            f"websearch.type must be one of {sorted(_ALLOWED_WEBSEARCH_TYPES)}, got: {web_type!r}"
        )

    if bool(session_extra.get("enable_music", False)) and version != "1.2.1.1":
        raise ConfigError("session.extra.enable_music=true only supports model.version=1.2.1.1")

    dialog = cfg.setdefault("dialog", {})
    if version == "2.2.0.0" and not str(dialog.get("character_manifest", "")).strip():
        raise ConfigError("model.version=2.2.0.0 requires non-empty dialog.character_manifest")

    ao = cfg.get("audio_output", {}) or {}
    for key in (
        "initial_volume_percent",
        "music_default_volume",
        "volume_step_percent",
        "min_volume_percent",
        "max_volume_percent",
        "ack_volume_percent",
    ):
        if key not in ao:
            continue
        val = float(ao[key])
        if key == "music_default_volume":
            if not (0.0 <= val <= 100.0):
                raise ConfigError(f"audio_output.{key} must be in [0, 100], got {val}")
        else:
            if not (0 <= val <= 100):
                raise ConfigError(f"audio_output.{key} must be in [0, 100], got {val}")

    return cfg
