from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List
import os

import yaml

from .schema import validate_doubao_config


def _resolve_config_dir() -> str:
    config_dir = os.environ.get("MR_CONFIG_DIR", "").strip()
    if not config_dir:
        raise RuntimeError("未设置环境变量 MR_CONFIG_DIR，无法加载语音配置文件")
    if not os.path.isdir(config_dir):
        raise RuntimeError(f"MR_CONFIG_DIR 指向目录不存在: {config_dir}")
    return os.path.abspath(config_dir)


def _resolve_config_file(name: str, explicit: str = "") -> str:
    config_dir = _resolve_config_dir()
    rel = explicit.strip() or name
    if os.path.isabs(rel):
        if not os.path.isfile(rel):
            raise FileNotFoundError(f"配置文件不存在: {rel}")
        return rel
    path = os.path.join(config_dir, os.path.basename(rel))
    if not os.path.isfile(path):
        raise FileNotFoundError(f"配置文件不存在: {path}")
    return path


def default_doubao_path() -> str:
    return _resolve_config_file("doubao.yaml")


def default_command_path() -> str:
    return _resolve_config_file("command.yaml")


def _read_yaml(path: str) -> Dict[str, Any]:
    file_path = Path(path)
    if not file_path.is_file():
        return {}
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _resolve_models_root() -> str:
    from ..utils.env_setup import get_models_root

    return get_models_root()


def _resource_root() -> str:
    mr = os.environ.get("MR_RESOURCE_DIR", "").strip()
    if mr and os.path.isdir(mr):
        return os.path.abspath(mr)
    try:
        return os.path.abspath(os.path.join(_resolve_models_root(), ".."))
    except Exception:
        return ""


def _strip_models_prefix(path: str, *, models_root: str) -> str:
    """models_root 已是 .../models 时，去掉配置里重复的 models/ 前缀。"""
    p = str(path or "").strip().replace("\\", "/")
    if not p:
        return p
    root = os.path.abspath(models_root).replace("\\", "/").rstrip("/")
    if root.endswith("/models") and p.startswith("models/"):
        return p[len("models/") :]
    return p


def _resolve_data_path(path: str, *, models_root: str) -> str:
    path = str(path or "").strip()
    if not path:
        return ""
    if os.path.isabs(path) and (os.path.isfile(path) or os.path.isdir(path)):
        return path
    if os.path.isfile(path) or os.path.isdir(path):
        return os.path.abspath(path)

    rel = _strip_models_prefix(path, models_root=models_root)
    root = _resource_root()
    candidates: list[str] = []

    # 与 KWS 相同包根（install/share/.../voice_asr_node 或源码树），不依赖 ament_index
    pkg_root = os.path.dirname(os.path.abspath(models_root))
    if pkg_root:
        candidates.append(os.path.join(pkg_root, rel))
        candidates.append(os.path.join(pkg_root, path))

    # colcon install（有 ament 时与 pkg_root 通常相同）
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory("voice_asr_node")
        candidates.append(os.path.join(share, rel))
        candidates.append(os.path.join(share, path))
        if rel.startswith("data/"):
            candidates.append(os.path.join(share, os.path.basename(rel)))
    except Exception:
        pass

    # 声纹模型/档案：优先包内路径，再 MR_RESOURCE_DIR（避免 resource/models 占位目录抢先）
    if rel.startswith("speaker/") or rel.startswith("data/"):
        pass
    if root and not (
        rel.startswith("speaker/") or rel.startswith("data/")
    ):
        candidates.append(os.path.join(root, path))
        candidates.append(os.path.join(root, rel))
    candidates.append(os.path.join(models_root, rel))
    candidates.append(os.path.join(models_root, path))
    candidates.append(os.path.join(models_root, os.path.basename(rel or path)))

    def _dir_has_profiles(directory: str) -> bool:
        try:
            for name in os.listdir(directory):
                if name.endswith(".npy"):
                    return True
        except OSError:
            return False
        return False

    for cand in candidates:
        if os.path.isfile(cand):
            return os.path.abspath(cand)
        if os.path.isdir(cand):
            # 空 data/speaker_profiles 不抢占 install 里已有 .npy 的目录
            if "speaker_profiles" in cand.replace("\\", "/") and not _dir_has_profiles(cand):
                continue
            return os.path.abspath(cand)
    return os.path.abspath(candidates[0]) if candidates else path


def _resolve_model_file(models_root: str, model_dir: str, filename: str) -> str:
    filename = str(filename or "").strip()
    if not filename:
        return ""
    if os.path.isabs(filename) and os.path.isfile(filename):
        return filename
    if os.path.isfile(filename):
        return os.path.abspath(filename)

    candidates = []
    model_dir = str(model_dir or "").strip()
    if model_dir:
        candidates.append(os.path.join(models_root, model_dir, filename))
        candidates.append(os.path.join(models_root, model_dir.lstrip("./"), filename))
    candidates.append(os.path.join(models_root, filename))

    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)
    return os.path.abspath(candidates[0]) if candidates else filename


@dataclass
class LangModelConfig:
    model_dir: str = ""
    encoder: str = ""
    decoder: str = ""
    joiner: str = ""
    tokens: str = "tokens.txt"
    keywords: List[str] = field(default_factory=list)
    keywords_file: str = ""
    lexicon: str = ""
    tokens_type: str = "auto"
    text_window_size: int = 20
    decoding_method: str = "greedy_search"
    num_threads: int = 2
    keywords_score: float = 1.0
    max_active_paths: int = 4
    num_trailing_blanks: int = 1

    def resolve_paths(self, models_root: str) -> Dict[str, str]:
        paths = {
            "encoder": _resolve_model_file(models_root, self.model_dir, self.encoder),
            "decoder": _resolve_model_file(models_root, self.model_dir, self.decoder),
            "joiner": _resolve_model_file(models_root, self.model_dir, self.joiner),
            "tokens": _resolve_model_file(models_root, self.model_dir, self.tokens),
        }
        if self.keywords_file:
            paths["keywords_file"] = _resolve_model_file(
                models_root, self.model_dir, self.keywords_file
            )
        if self.lexicon:
            paths["lexicon"] = _resolve_model_file(models_root, self.model_dir, self.lexicon)
        return paths


@dataclass
class CommandConfig:
    enabled: bool = True
    registry_path: str = field(default_factory=default_command_path)
    match_mode: str = "llm_tag"
    fuzzy_threshold: int = 88
    fuzzy_enabled: bool = True


@dataclass
class AudioOutputConfig:
    backend: str = "pulseaudio"
    pulse_sink: str = ""
    initial_volume_percent: int = 50
    music_default_volume: float = 50.0
    volume_step_percent: int = 10
    min_volume_percent: int = 0
    max_volume_percent: int = 100
    ack_volume_percent: int = 80


@dataclass
class SpeakerIdConfig:
    enabled: bool = False
    model_path: str = ""
    profiles_dir: str = ""
    similarity_threshold: float = 0.75
    reject_unknown: bool = False
    inject_username_to_prompt: bool = False
    min_audio_ms: int = 1500
    register_min_audio_ms: int = 2000


@dataclass
class SilenceConfig:
    wakeup_timeout_s: float = 12.0
    dialog_timeout_s: float = 15.0
    timeout_prompt: str = "好的，如需帮助请再次呼叫我。"


@dataclass
class WakewordConfig:
    enabled: bool = True
    engine: str = "sherpa_onnx_kws"
    active_languages: List[str] = field(default_factory=lambda: ["zh_en"])
    models: Dict[str, LangModelConfig] = field(default_factory=dict)
    kws_threshold: float = 0.15
    kws_input_gain_db: float = 6.0
    idle_timeout_sec: float = 30.0
    say_hello_direct_tts: bool = False
    say_hello_text: str = "我在，有什么可以帮您？"
    say_hello_by_speaker: Dict[str, str] = field(default_factory=dict)
    say_hello_prompt: str = ""
    say_hello_use_model: bool = True
    say_hello_model_query: str = "你好，小未"
    pre_generate_greeting: bool = False

    @property
    def input_gain(self) -> float:
        if self.kws_input_gain_db == 0:
            return 1.0
        return float(10.0 ** (self.kws_input_gain_db / 20.0))


@dataclass
class AudioConfig:
    mic_device: str = ""
    playback_device: str = ""
    send_frame_ms: int = 20


@dataclass
class ExitIntentConfig:
    enable_user_query_exit: bool = True
    local_exit_keywords: List[str] = field(
        default_factory=lambda: ["再见", "拜拜", "退出", "结束对话", "不聊了"]
    )
    exit_farewell_timeout_s: float = 12.0


@dataclass
class AppConfig:
    raw: Dict[str, Any]
    command_raw: Dict[str, Any] = field(default_factory=dict)
    models_root: str = field(default_factory=_resolve_models_root)

    @property
    def model_version(self) -> str:
        return str(self.raw.get("model", {}).get("version", "1.2.1.1"))

    @property
    def wakeword(self) -> WakewordConfig:
        wake = self.raw.get("wakeword") or {}
        models_raw = wake.get("models", {}) or {}
        models: Dict[str, LangModelConfig] = {}
        for lang, mc in models_raw.items():
            if not isinstance(mc, dict):
                continue
            models[str(lang)] = LangModelConfig(
                model_dir=str(mc.get("model_dir", f"kws/{lang}")),
                encoder=str(mc.get("encoder", "")),
                decoder=str(mc.get("decoder", "")),
                joiner=str(mc.get("joiner", "")),
                tokens=str(mc.get("tokens", "tokens.txt")),
                keywords=[str(x) for x in mc.get("keywords", []) if str(x).strip()],
                keywords_file=str(mc.get("keywords_file", "")),
                lexicon=str(mc.get("lexicon", "")),
                tokens_type=str(mc.get("tokens_type", "auto")),
                text_window_size=int(mc.get("text_window_size", 20)),
                decoding_method=str(mc.get("decoding_method", "greedy_search")),
                num_threads=int(mc.get("num_threads", 2)),
                keywords_score=float(mc.get("keywords_score", 1.0)),
                max_active_paths=int(mc.get("max_active_paths", 4)),
                num_trailing_blanks=int(mc.get("num_trailing_blanks", 1)),
            )
        active = wake.get("active_languages") or ["zh_en"]
        by_speaker_raw = wake.get("say_hello_by_speaker") or {}
        say_hello_by_speaker: Dict[str, str] = {}
        if isinstance(by_speaker_raw, dict):
            for k, v in by_speaker_raw.items():
                key = str(k).strip()
                val = str(v).strip()
                if key and val:
                    say_hello_by_speaker[key] = val
        return WakewordConfig(
            enabled=bool(wake.get("enabled", True)),
            engine=str(wake.get("engine", "sherpa_onnx_kws")),
            active_languages=[str(x) for x in active],
            models=models,
            kws_threshold=float(wake.get("kws_threshold", 0.15)),
            kws_input_gain_db=float(wake.get("kws_input_gain_db", 6.0)),
            idle_timeout_sec=float(wake.get("idle_timeout_sec", 30.0)),
            say_hello_direct_tts=bool(wake.get("say_hello_direct_tts", False)),
            say_hello_text=str(
                wake.get("say_hello_text", "我在，有什么可以帮您？")
            ).strip(),
            say_hello_by_speaker=say_hello_by_speaker,
            say_hello_prompt=str(wake.get("say_hello_prompt", "")).strip(),
            say_hello_use_model=bool(wake.get("say_hello_use_model", True)),
            say_hello_model_query=str(
                wake.get("say_hello_model_query", "你好，小未")
            ).strip(),
            pre_generate_greeting=bool(wake.get("pre_generate_greeting", False)),
        )

    @property
    def audio(self) -> AudioConfig:
        audio = self.raw.get("audio", {})
        return AudioConfig(
            mic_device=str(audio.get("mic_device", "")),
            playback_device=str(audio.get("playback_device", "")),
            send_frame_ms=int(audio.get("send_frame_ms", 20)),
        )

    @property
    def audio_output(self) -> AudioOutputConfig:
        ao = self.raw.get("audio_output", {}) or {}
        return AudioOutputConfig(
            backend=str(ao.get("backend", "pulseaudio")),
            pulse_sink=str(ao.get("pulse_sink", "")),
            initial_volume_percent=int(ao.get("initial_volume_percent", 50)),
            music_default_volume=float(ao.get("music_default_volume", 50)),
            volume_step_percent=int(ao.get("volume_step_percent", 10)),
            min_volume_percent=int(ao.get("min_volume_percent", 0)),
            max_volume_percent=int(ao.get("max_volume_percent", 100)),
            ack_volume_percent=int(ao.get("ack_volume_percent", 80)),
        )

    @property
    def speaker_id(self) -> SpeakerIdConfig:
        sid = self.raw.get("speaker_id", {}) or {}
        model_path = _resolve_data_path(
            str(
                sid.get(
                    "model_path",
                    "models/speaker/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx",
                )
            ),
            models_root=self.models_root,
        )
        profiles_dir = _resolve_data_path(
            str(sid.get("profiles_dir", "data/speaker_profiles")),
            models_root=self.models_root,
        )
        return SpeakerIdConfig(
            enabled=bool(sid.get("enabled", False)),
            model_path=model_path,
            profiles_dir=profiles_dir,
            similarity_threshold=float(sid.get("similarity_threshold", 0.75)),
            reject_unknown=bool(sid.get("reject_unknown", False)),
            inject_username_to_prompt=bool(sid.get("inject_username_to_prompt", False)),
            min_audio_ms=int(sid.get("min_audio_ms", 1500)),
            register_min_audio_ms=int(sid.get("register_min_audio_ms", 2000)),
        )

    @property
    def silence(self) -> SilenceConfig:
        sil = self.raw.get("silence", {}) or {}
        return SilenceConfig(
            wakeup_timeout_s=float(sil.get("wakeup_timeout_s", 12)),
            dialog_timeout_s=float(sil.get("dialog_timeout_s", 15)),
            timeout_prompt=str(sil.get("timeout_prompt", "好的，如需帮助请再次呼叫我。")),
        )

    @property
    def exit_intent(self) -> ExitIntentConfig:
        ei = self.raw.get("exit_intent", {}) or {}
        keywords_raw = ei.get("local_exit_keywords") or []
        keywords = [str(k).strip() for k in keywords_raw if str(k).strip()]
        return ExitIntentConfig(
            enable_user_query_exit=bool(ei.get("enable_user_query_exit", True)),
            local_exit_keywords=keywords,
            exit_farewell_timeout_s=float(ei.get("exit_farewell_timeout_s", 12.0)),
        )

    @property
    def command(self) -> CommandConfig:
        cmd = self.raw.get("command", {})
        settings = self.command_raw.get("settings", {})
        return CommandConfig(
            enabled=bool(cmd.get("enabled", True)),
            registry_path=_resolve_config_file(
                "command.yaml", str(cmd.get("registry_path", "")).strip()
            ),
            match_mode=str(settings.get("match_mode", "llm_tag")).strip().lower(),
            fuzzy_threshold=int(settings.get("fuzzy_threshold", 88)),
            fuzzy_enabled=bool(settings.get("fuzzy_enabled", True)),
        )

    @property
    def command_entries(self) -> List[Dict[str, Any]]:
        return list(self.command_raw.get("commands", []))


def resolve_wake_greeting_text(cfg: AppConfig) -> str:
    """按 tts.speaker / clone_speaker_id 选取固定问候文案（ChatTTSText 模式）。"""
    ww = cfg.wakeword
    tts = cfg.raw.get("tts", {}) or {}
    clone_id = str(tts.get("clone_speaker_id", "")).strip()
    speaker = clone_id or str(tts.get("speaker", "")).strip()
    if speaker and speaker in ww.say_hello_by_speaker:
        return ww.say_hello_by_speaker[speaker]
    if ww.say_hello_text:
        return ww.say_hello_text
    return "我在，有什么可以帮您？"


def resolve_wake_model_query(cfg: AppConfig) -> str:
    """ChatTextQuery：模拟用户短句，由模型以助手身份生成问候 TTS（勿传长指令）。"""
    ww = cfg.wakeword
    if ww.say_hello_prompt:
        return ww.say_hello_prompt
    if ww.say_hello_model_query:
        return ww.say_hello_model_query
    return "你好，小未"


def load_app_config(doubao_path: str = "", command_path: str = "") -> AppConfig:
    explicit_doubao = doubao_path.strip() or os.environ.get("MR_DOUBAO_CONFIG", "").strip()
    chosen_doubao = _resolve_config_file("doubao.yaml", explicit_doubao)
    raw = _read_yaml(chosen_doubao)
    if not raw:
        raise FileNotFoundError(f"doubao config not found: {chosen_doubao}")

    validated = validate_doubao_config(raw)
    cmd_section = validated.get("command", {}) or {}
    explicit_command = command_path.strip() or str(cmd_section.get("registry_path", "")).strip()
    chosen_command = _resolve_config_file("command.yaml", explicit_command)
    command_raw = _read_yaml(chosen_command)

    if command_raw and "commands" not in command_raw and isinstance(command_raw, dict):
        legacy_cmds = command_raw.get("commands", [])
        if legacy_cmds:
            command_raw = {"settings": command_raw.get("settings", {}), "commands": legacy_cmds}

    models_root = _resolve_models_root()
    return AppConfig(raw=validated, command_raw=command_raw, models_root=models_root)
