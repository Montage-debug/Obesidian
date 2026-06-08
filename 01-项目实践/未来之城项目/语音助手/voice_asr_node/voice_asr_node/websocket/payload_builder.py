from typing import Any, Dict, List


def _dedup_words(words: List[str]) -> List[str]:
    return sorted({w.strip() for w in words if str(w).strip()})


def build_start_session_payload(cfg: Dict[str, Any], command_hotwords: List[str]) -> Dict[str, Any]:
    model_cfg = cfg.get("model", {})
    dialog_cfg = cfg.get("dialog", {})
    tts_cfg = cfg.get("tts", {})
    asr_cfg = cfg.get("asr", {})
    session_cfg = cfg.get("session", {})
    web_cfg = cfg.get("websearch", {})
    context_cfg = cfg.get("context", {})
    aigc_cfg = cfg.get("aigc_metadata", {})

    tts_audio_cfg = tts_cfg.get("audio_config", {})
    tts_extra_cfg = tts_cfg.get("extra", {})
    asr_extra_cfg = asr_cfg.get("extra", {})
    asr_context_cfg = asr_extra_cfg.get("context", {})
    session_extra_cfg = session_cfg.get("extra", {})

    hotwords = list(asr_context_cfg.get("hotwords", [])) + list(command_hotwords)
    hotwords = _dedup_words([str(x) for x in hotwords])

    speaker = str(tts_cfg.get("clone_speaker_id", "")).strip() or str(tts_cfg.get("speaker", "")).strip()

    dialog_extra = {
        "strict_audit": bool(session_extra_cfg.get("strict_audit", True)),
        "audit_response": str(session_extra_cfg.get("audit_response", "")),
        "input_mod": str(session_extra_cfg.get("input_mod", "keep_alive")),
        "enable_music": bool(session_extra_cfg.get("enable_music", False)),
        "enable_loudness_norm": bool(tts_extra_cfg.get("enable_loudness_norm", False)),
        "enable_conversation_truncate": bool(
            context_cfg.get(
                "enable_conversation_truncate",
                session_extra_cfg.get("enable_conversation_truncate", False),
            )
        ),
        "enable_user_query_exit": bool(session_extra_cfg.get("enable_user_query_exit", True)),
        "enable_volc_websearch": bool(web_cfg.get("enable", False)),
        "model": str(model_cfg.get("version", "1.2.1.1")),
    }

    if web_cfg.get("enable", False):
        dialog_extra.update(
            {
                "volc_websearch_type": str(web_cfg.get("type", "web")),
                "volc_websearch_api_key": str(web_cfg.get("api_key", "")),
                "volc_websearch_bot_id": str(web_cfg.get("bot_id", "")),
                "volc_websearch_result_count": int(web_cfg.get("result_count", 5)),
                "volc_websearch_no_result_message": str(web_cfg.get("no_result_message", "")),
            }
        )

    payload = {
        "tts": {
            "speaker": speaker,
            "audio_config": {
                "channel": int(tts_audio_cfg.get("channel", 1)),
                "format": str(tts_audio_cfg.get("format", "pcm_s16le")),
                "sample_rate": int(tts_audio_cfg.get("sample_rate", 24000)),
                "speech_rate": int(tts_audio_cfg.get("speech_rate", 0)),
                "loudness_rate": int(tts_audio_cfg.get("loudness_rate", 0)),
            },
            "extra": {
                "explicit_dialect": str(tts_extra_cfg.get("explicit_dialect", "")),
                "aigc_metadata": {
                    "enable": bool(aigc_cfg.get("enable", False)),
                    "content_producer": str(aigc_cfg.get("content_producer", "")),
                    "produce_id": str(aigc_cfg.get("produce_id", "")),
                    "content_propagator": str(aigc_cfg.get("content_propagator", "")),
                    "propagate_id": str(aigc_cfg.get("propagate_id", "")),
                },
            },
        },
        "asr": {
            "audio_info": {
                "format": str(asr_cfg.get("audio_info", {}).get("format", "pcm")),
                "sample_rate": int(asr_cfg.get("audio_info", {}).get("sample_rate", 16000)),
                "channel": int(asr_cfg.get("audio_info", {}).get("channel", 1)),
            },
            "extra": {
                "end_smooth_window_ms": int(asr_extra_cfg.get("end_smooth_window_ms", 1200)),
                "enable_custom_vad": bool(asr_extra_cfg.get("enable_custom_vad", False)),
                "enable_asr_twopass": bool(asr_extra_cfg.get("enable_asr_twopass", True)),
                "boosting_table_id": str(asr_extra_cfg.get("boosting_table_id", "")),
                "boosting_table_name": str(asr_extra_cfg.get("boosting_table_name", "")),
                "regex_correct_table_id": str(asr_extra_cfg.get("regex_correct_table_id", "")),
                "regex_correct_table_name": str(asr_extra_cfg.get("regex_correct_table_name", "")),
                "context": {
                    "hotwords": [{"word": x} for x in hotwords],
                    "correct_words": dict(asr_context_cfg.get("correct_words", {})),
                },
            },
        },
        "dialog": {
            "bot_name": str(dialog_cfg.get("bot_name", "豆包")),
            "system_role": str(dialog_cfg.get("system_role", "")),
            "speaking_style": str(dialog_cfg.get("speaking_style", "")),
            "dialog_id": str(dialog_cfg.get("dialog_id", "")),
            "character_manifest": str(dialog_cfg.get("character_manifest", "")),
            "dialog_context": list(context_cfg.get("dialog_context", [])),
            "extra": dialog_extra,
        },
    }

    if web_cfg.get("enable", False):
        payload["dialog"]["location"] = dict(web_cfg.get("location", {}))

    return payload
