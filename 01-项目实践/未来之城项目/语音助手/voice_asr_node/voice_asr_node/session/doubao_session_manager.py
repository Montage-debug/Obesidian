from __future__ import annotations

import asyncio
import threading
import time
from typing import Any, Awaitable, Callable, Optional, Tuple

from ..command import cmd_engine_from_command_raw
from ..config.app_config import (
    AppConfig,
    resolve_wake_greeting_text,
    resolve_wake_model_query,
)
from ..config.doubao_auth import get_doubao_auth
from ..command.cmd_engine import (
    CMD_TAG_PATTERN,
    _CMD_TAG_PARTIAL_RE,
    _normalize_utterance,
)
from ..command.cmd_result import CmdResult
from ..command.volume_parse import normalize_volume_arg
from ..websocket import (
    DoubaoRealtimeClient,
    EVT_ASRInfo,
    EVT_ASREnded,
    EVT_ASRResponse,
    EVT_ChatResponse,
    EVT_DialogCommonError,
    EVT_TTSEnded,
    EVT_TTSSentenceEnd,
    EVT_TTSSentenceStart,
    EVT_SessionFinished,
    EVT_TTSResponse,
    EVT_UsageResponse,
    ServerFrame,
    build_start_session_payload,
)
from ..ros2.bridge import MUSIC_CONTROL_SERVICE, ROS2Bridge, is_massage_service
from ..speaker.speaker_id import SpeakerIdentifier
from .silence_detector import DialogPhase, SilenceConfig, SilenceDetector
from .states import SessionState

# 先听完确认 TTS 再执行（避免与 TTS 抢播）
_MUSIC_ACTIONS_DEFER_FOR_TTS = frozenset(
    {"play", "next", "previous", "replay", "user_resume"}
)
# 收到标签后立即执行，不等待 TTS 播完（关/停要立刻生效）
_MUSIC_ACTIONS_IMMEDIATE = frozenset({"stop", "user_pause"})
# 这些命令走 defer：先播豆包确认 TTS，再执行本地音乐；标签仍走字节门控，不朗读 [CMD:...]
_MUSIC_DEFER_CMD_IDS = frozenset(
    {
        "music_play",
        "music_play_song",
        "music_next",
        "music_previous",
        "music_replay",
        "music_resume",
        "music_pause",
        "music_stop",
    }
)
_MASSAGE_PARAM_CMD_IDS = frozenset(
    {
        "massage_position",
        "massage_duration",
        "massage_force",
        "head_param",
    }
)
# 按摩 ROS2：空请求 + 参数化服务；Chat 路径由模型 TTS 确认，本地不再 ChatTTSText
_MASSAGE_CMD_IDS = frozenset(
    {
        "massage_pause",
        "massage_resume",
        "massage_cancel",
        *_MASSAGE_PARAM_CMD_IDS,
        "massage_progress",
    }
)
# 切歌/上一首：模型常漏 [CMD] 且回歌词；确认语宜短、动作宜快
_MUSIC_QUICK_CMD_IDS = frozenset(
    {"music_next", "music_previous", "music_replay"}
)
# 模型常把点歌/唱歌理解成云端唱词：尽早切本地 MP3 并打断歌词 TTS
_MUSIC_LYRICS_EARLY_CMD_IDS = _MUSIC_QUICK_CMD_IDS | frozenset(
    {"music_play", "music_play_song", "music_resume"}
)
_CHAT_CONFIRM_HINTS = (
    "好的",
    "没问题",
    "马上",
    "这就",
    "继续",
    "为您",
    "播放",
    "下一首",
    "上一首",
    "切到",
)
_MUSIC_STOP_SPOKEN_HINTS = (
    "不想听",
    "不听了",
    "关闭音乐",
    "关掉音乐",
    "停止音乐",
    "音乐已关",
    "已关闭",
    "别放了",
    "不放了",
)
_MUSIC_PLAY_SPOKEN_HINTS = (
    "播放",
    "音乐",
    "响起",
    "来首",
    "放一首",
    "点播",
    "为你播",
    "开始播",
)
# 等待豆包确认 TTS 播完的最长时间（正常约 1~2s 内结束，不靠超时）
_DEFER_MUSIC_CONFIRM_MAX_S = 15.0
_MUSIC_DUCK_SUPPRESS_AFTER_LOCAL_S = 30.0
# ASR 点歌后等待豆包 ChatResponse 打出 [CMD:music_play_song:歌名] 再延后播
_SONG_TAG_WAIT_S = 5.0
# ASR 命中播歌后等待 Chat 标签；超时则用 ChatTTSText 兜底
_ASR_MUSIC_TAG_WAIT_S = 1.2
# ASR 误捕获的「歌名」碎片（如「不想听歌了」→「歌了」）
_PLAY_SONG_REJECT_NAMES = frozenset(
    {"歌了", "音乐了", "首歌", "一首", "一首歌", "轻音乐", "首歌了"}
)
_PAUSE_CLARIFY_TEXT = "您是要暂停按摩，还是暂停背景音乐？"
_PAUSE_CLARIFY_TIMEOUT_S = 10.0
_MUSIC_PLAY_INJECTED_CONFIRM = "好的，正在为您播放音乐。"
# 标签前已有自然口语确认时，用模型 TTS+门控，避免 isolate+ChatTTSText 打断闲聊
_MODEL_DEFER_PREFER_SPOKEN_CMD_IDS = frozenset(
    {"music_play", "music_play_song", "music_resume"}
)
_INJECTED_CONFIRM_FAIL_S = 1.25
_INJECTED_CONFIRM_FAIL_AFTER_RESEND_S = 1.55


class DoubaoSessionManager:
    """
    asyncio 驱动的豆包会话管理器（状态机中枢）。

    负责：
    - 维护 websocket 连接与 session 生命周期（CONNECTED/SERVICE_ACTIVE/...）
    - 接收服务端事件并分发（ASR/TTS/错误/退出意图）
    - 命令词匹配与触发 ROS2 动作（后台执行）
    - 支持外部注入 ChatTTSText 播报

    不负责：
    - 麦克风采集与播放（由上层 Node 维护线程/设备）
    - KWS 检测（上层在检测到唤醒词时调用 on_wake()）
    """

    def __init__(
        self,
        *,
        node,
        cfg: AppConfig,
        ros2: ROS2Bridge,
        on_tts_audio: Callable[[bytes], None],
        publish_playback_done: Callable[[], None],
        publish_exit_intent: Callable[[], None],
        speaker: Optional[SpeakerIdentifier] = None,
        cmd_exec: Any = None,
    ):
        self._node = node
        self._cfg = cfg
        self._ros2 = ros2
        self._speaker = speaker
        self._cmd_exec = cmd_exec
        self._cmd_engine = cmd_engine_from_command_raw(cfg.command_raw)

        self._on_tts_audio = on_tts_audio
        self._pub_done = publish_playback_done
        self._pub_exit = publish_exit_intent

        self._session_active = False
        self._session_state = SessionState.DISCONNECTED
        self._last_active_ts = time.time()
        self._idle_timeout_sec = cfg.wakeword.idle_timeout_sec
        sil = cfg.silence
        self._silence_detector = SilenceDetector(
            SilenceConfig(
                wakeup_timeout_s=sil.wakeup_timeout_s,
                dialog_timeout_s=sil.dialog_timeout_s,
            )
        )
        self._silence_phase = DialogPhase.WAKEUP
        self._silence_timeout_handled = False
        self._identified_user: Optional[str] = None
        self._pending_wake_pcm = b""
        self._last_final_text = ""
        self._last_final_ts = 0.0
        self._last_cmd_ts = 0.0
        self._last_cmd_id = ""
        self._last_cmd_arg = ""
        self._last_cmd_exec_ts = 0.0
        self._last_asr_param_ok: Optional[bool] = None
        self._last_asr_param_cmd_id = ""
        self._cmd_busy = False
        self._pending_question_id = ""
        self._active_question_id = ""

        self._audio_tx_start_ts: Optional[float] = None
        self._audio_playback_start_ts: Optional[float] = None
        audio_cfg = cfg.raw.get("audio", {}) or {}
        self._tts_music_independent = self._cfg_bool(
            audio_cfg.get("tts_music_independent", True), default=True
        )
        self._music_duck_enabled = (
            not self._tts_music_independent
            and self._cfg_bool(audio_cfg.get("music_duck_on_tts", True), default=True)
        )
        self._music_duck_suppress_until = 0.0
        self._user_music_paused = False
        self._tts_did_duck = False
        self._tts_playback_active = False
        self._is_tts_playback_idle: Callable[[], bool] = lambda: True
        self._deferred_music: Optional[Tuple[str, str]] = None
        self._deferred_music_task: Optional[asyncio.Task] = None
        self._defer_in_progress = False
        self._defer_tts_ended = False
        self._defer_last_tts_audio_ts = 0.0
        self._defer_confirm_target_ms = 0
        self._pending_song = ""
        self._song_tag_wait_task: Optional[asyncio.Task] = None
        self._defer_saw_tts = False
        self._defer_music_executed = False
        self._defer_tts_guard_until = 0.0
        self._parallel_confirm_guard_until = 0.0
        self._parallel_confirm_period_ms = 0
        self._defer_confirm_source = "model"
        self._awaiting_injected_tts = False
        self._pending_asr_music: Optional[Tuple[str, str, str]] = None
        self._pending_asr_music_utterance = ""
        self._asr_music_tag_wait_task: Optional[asyncio.Task] = None
        self._pending_chat_complete_task: Optional[asyncio.Task] = None
        self._last_music_current_file = ""
        self._injected_defer_key: Optional[Tuple[str, str]] = None
        self._injected_defer_started_mono = 0.0
        self._injected_confirm_text = ""
        self._injected_confirm_target_ms = 0
        self._injected_playback_started = False
        self._injected_chat_tts_sent_mono = 0.0
        self._injected_armed = False
        self._injected_tts_resend = False
        self._injected_tts_ended_pending = False
        self._awaiting_pause_clarification_until = 0.0
        self._server_asr_ended = False
        self._current_reply_id = ""
        self._greeting_sent = False
        self._pre_session_pending = False
        self._pre_session_ready = False
        self._pre_greeting_pcm = bytearray()
        self._pre_greeting_ready = False
        self._generating_greeting = False
        self._pre_session_task: Optional[asyncio.Task] = None
        self._greeting_uplink_hold = False
        self._awaiting_wake_greeting_tts = False
        self._wake_handler_busy = False
        self._pregen_wait_timeout_s = 4.0
        self._pending_session_exit = False
        self._pending_session_exit_reason = ""
        self._exit_uplink_muted = False
        self._session_exit_completing = False
        self._exit_timeout_task: Optional[asyncio.Task] = None
        self._exit_farewell_watch_task: Optional[asyncio.Task] = None
        self._exit_farewell_tts_started = False
        self._recent_client_interrupt_mono = 0.0
        self._wake_greeting_hold_task: Optional[asyncio.Task] = None
        context_cfg = cfg.raw.get("context", {})
        session_extra = cfg.raw.get("session", {}).get("extra", {})
        self._truncate_enabled = bool(
            context_cfg.get(
                "enable_conversation_truncate",
                session_extra.get("enable_conversation_truncate", False),
            )
        )
        self._drop_incoming_tts = False
        self._dropped_tts_chunks = 0
        self._dropped_tts_bytes = 0
        self._debug_interrupt = bool(cfg.raw.get("audio", {}).get("debug_interrupt", False))
        self._tag_tts_tail_suppressed = False
        self._tag_tts_gate_active = False
        self._tag_tts_received_bytes = 0
        self._tag_confirm_bytes_budget = 0
        self._tag_gate_prepare_logged = False
        self._tag_cloud_cut_sent = False
        self._reply_tts_sentence_idx = 0
        session_cfg = cfg.raw.get("session", {}) or {}
        self._biz_tts_auto_finish = self._cfg_bool(
            session_cfg.get("biz_tts_auto_finish", True), default=True
        )
        self._biz_tts_queue: Optional[asyncio.Queue] = None
        self._biz_tts_worker_task: Optional[asyncio.Task] = None
        self._biz_tts_playing = False
        self._awaiting_biz_tts = False
        self._biz_tts_done: Optional[asyncio.Event] = None
        self._biz_tts_hold_uplink = False
        self._biz_tts_hold_task: Optional[asyncio.Task] = None
        tts_cfg = cfg.raw.get("tts", {}).get("audio_config", {})
        self._tts_sample_rate = int(tts_cfg.get("sample_rate", 24000))
        self._tts_channels = int(tts_cfg.get("channel", 1))

        # WS asyncio thread
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True, name="doubao-ws-loop")
        self._client: Optional[DoubaoRealtimeClient] = None
        self._audio_async_q: Optional[asyncio.Queue] = None
        self._stop_event = threading.Event()
        self._shutting_down = False

    # ──────────────────────────────────────────────────────────────
    # lifecycle
    # ──────────────────────────────────────────────────────────────
    def bind_tts_playback_idle(self, is_idle: Callable[[], bool]) -> None:
        """由 DoubaoDialogNode 在创建 SoundDevicePlayer 后注入。"""
        self._is_tts_playback_idle = is_idle

    @property
    def tts_music_independent(self) -> bool:
        return self._tts_music_independent

    @staticmethod
    def _cfg_bool(value: object, *, default: bool) -> bool:
        if isinstance(value, bool):
            return value
        if value is None:
            return default
        return str(value).strip().lower() not in ("0", "false", "no", "off")

    def _log(self, level: str, msg: str) -> None:
        log_safe = getattr(self._node, "_log_safe", None)
        if callable(log_safe):
            log_safe(level, msg)
            return
        try:
            import rclpy

            if rclpy.ok() and not self._shutting_down:
                getattr(self._node.get_logger(), level)(msg)
                return
        except Exception:
            pass
        print(f"[doubao_session][{level}] {msg}", flush=True)

    def request_shutdown(self) -> None:
        """节点销毁前调用：取消延后任务，避免关闭阶段继续打 rosout。"""
        if self._shutting_down:
            return
        self._shutting_down = True
        self._stop_event.set()
        self._session_active = False
        self._cancel_deferred_music()
        self._cancel_song_tag_wait()
        self._cancel_pending_asr_defer()
        self._pending_song = ""

    def start(self) -> None:
        self._loop_thread.start()

    def stop(self) -> None:
        self.request_shutdown()
        try:
            if self._loop.is_running():

                async def _shutdown() -> None:
                    if self._client:
                        try:
                            await asyncio.wait_for(self._client.close(), timeout=1.5)
                        except Exception:
                            pass
                    current = asyncio.current_task(self._loop)
                    pending = [
                        t
                        for t in asyncio.all_tasks(self._loop)
                        if t is not current and not t.done()
                    ]
                    for task in pending:
                        task.cancel()
                    if pending:
                        await asyncio.gather(*pending, return_exceptions=True)
                    self._loop.stop()

                fut = asyncio.run_coroutine_threadsafe(_shutdown(), self._loop)
                try:
                    fut.result(timeout=2.0)
                except KeyboardInterrupt:
                    self._loop.call_soon_threadsafe(self._loop.stop)
                except Exception:
                    self._loop.call_soon_threadsafe(self._loop.stop)
        except KeyboardInterrupt:
            pass
        except Exception:
            pass
        if self._loop_thread.is_alive():
            self._loop_thread.join(timeout=2.0)

    # ──────────────────────────────────────────────────────────────
    # state helpers
    # ──────────────────────────────────────────────────────────────
    @property
    def session_active(self) -> bool:
        return self._session_active

    @property
    def is_in_session(self) -> bool:
        return bool(self._client and self._client.is_in_session)

    @property
    def defer_in_progress(self) -> bool:
        return self._defer_in_progress

    @property
    def pending_session_exit(self) -> bool:
        return self._pending_session_exit

    def _publish_playback_done_safe(self) -> None:
        if self._shutting_down:
            return
        try:
            self._pub_done()
        except Exception:
            pass

    def on_timer(self) -> None:
        if self._stop_event.is_set():
            return
        # 空闲超时自动结束 session（优先 silence.dialog_timeout，否则 wakeword.idle）
        if self._session_active:
            timeout_s = (
                self._cfg.silence.dialog_timeout_s
                if self._silence_phase == DialogPhase.DIALOG
                else self._cfg.silence.wakeup_timeout_s
            )
            if self._silence_detector.is_timeout(self._silence_phase):
                if not self._silence_timeout_handled:
                    self._silence_timeout_handled = True
                    self._node.get_logger().info(
                        f"[doubao] silence timeout ({timeout_s}s), finish session"
                    )
                    self._submit(self._on_silence_timeout())
                return
            if (time.time() - self._last_active_ts) > self._idle_timeout_sec:
                self._node.get_logger().info("[doubao] idle timeout, finish session")
                self._session_active = False
                if self._client:
                    self._submit(self._finish_session_and_cleanup())

    def _touch_activity(self) -> None:
        self._last_active_ts = time.time()
        self._silence_detector.on_activity()
        self._silence_timeout_handled = False

    async def _on_silence_timeout(self) -> None:
        prompt = self._cfg.silence.timeout_prompt.strip()
        self._session_active = False
        if self._client and self._client.is_in_session:
            if prompt:
                await self._send_chat_tts_text(prompt)
            await self._client.finish_session()
        self._silence_phase = DialogPhase.WAKEUP
        await self._on_session_finished_cleanup()

    def on_wake(self, *, source: str, keyword: str = "", wake_pcm: bytes = b"") -> None:
        _ = (source, keyword)
        self._pending_wake_pcm = wake_pcm or b""
        self._identified_user = None
        self._silence_phase = DialogPhase.WAKEUP
        self._touch_activity()
        self._cancel_deferred_music()
        self._cancel_song_tag_wait()
        self._cancel_pending_asr_defer()
        self._pending_song = ""
        self._session_active = True
        self._greeting_sent = False
        self._greeting_uplink_hold = True
        self._touch_activity()
        self._submit(self._handle_wake_once())

    def interrupt(self, *, reason: str = "manual") -> None:
        self._submit(self._interrupt_for_user(reason=reason))

    async def _interrupt_for_user(self, *, reason: str) -> None:
        """用户/KWS 打断：清陈旧 suppress，避免误伤下一轮云端 TTS。"""
        in_confirm_window = (
            self._defer_in_progress
            or time.monotonic() < self._defer_tts_guard_until
        )
        if reason == "kws" and in_confirm_window:
            await self._interrupt_tts_playback(
                reason=reason,
                suppress_followup=False,
            )
            return
        self._cancel_deferred_music()
        self._tag_tts_tail_suppressed = False
        self._drop_incoming_tts = False
        self._clear_tag_tts_gate()
        suppress_next = reason not in ("kws", "asr_info")
        await self._interrupt_tts_playback(
            reason=reason,
            suppress_followup=suppress_next,
        )

    def on_tts_text(self, text: str, *, urgent: bool = False) -> None:
        """业务播报（massage_biz_event → /voice_asr/tts_speak）：排队后 ChatTTSText。"""
        if not text.strip():
            return
        self._touch_activity()
        self._submit(self._enqueue_biz_tts(text.strip(), urgent=urgent))

    def offer_audio_frame(self, frame: bytes) -> None:
        if self._stop_event.is_set() or self._audio_async_q is None or not self._loop.is_running():
            return
        self._loop.call_soon_threadsafe(self._offer_audio_frame_async, frame)

    def on_playback_started(self) -> None:
        if self._injected_music_defer_active():
            self._injected_playback_started = True
        if self._audio_playback_start_ts is not None:
            return
        self._audio_playback_start_ts = time.perf_counter()
        if self._audio_tx_start_ts is not None:
            latency_ms = (self._audio_playback_start_ts - self._audio_tx_start_ts) * 1000.0
            self._node.get_logger().info(f"[dialog] 音频返回开始播放，音频上行到播放总延迟 {latency_ms:.1f} ms")
        else:
            self._node.get_logger().info("[dialog] 音频返回开始播放")

    # ──────────────────────────────────────────────────────────────
    # asyncio thread
    # ──────────────────────────────────────────────────────────────
    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._audio_async_q = asyncio.Queue(maxsize=200)
        self._biz_tts_queue = asyncio.Queue(maxsize=32)
        self._biz_tts_worker_task = self._loop.create_task(
            self._biz_tts_worker(), name="biz-tts-worker"
        )
        self._loop.create_task(self._connect_forever(), name="doubao-connect-forever")
        self._loop.create_task(self._audio_uplink_worker(), name="doubao-audio-uplink")
        self._loop.run_forever()

    def _submit(self, coro):
        if self._stop_event.is_set() or not self._loop.is_running():
            return None
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    async def _connect_forever(self):
        auth = get_doubao_auth()
        audio = self._cfg.raw.get("audio", {})
        while not self._stop_event.is_set():
            try:
                self._session_state = SessionState.CONNECTING
                self._client = DoubaoRealtimeClient(
                    ws_url=auth.ws_url,
                    api_key=auth.api_key,
                    resource_id=auth.resource_id,
                    app_key=auth.app_key,
                    reconnect_min=float(audio.get("reconnect_min", 1.0)),
                    reconnect_max=float(audio.get("reconnect_max", 30.0)),
                )
                self._client.set_handler(self._on_frame_async)
                await self._client.connect()
                self._node.get_logger().info("[doubao] connected")
                self._session_state = SessionState.CONNECTED

                # 若未启用 KWS，则默认常开会话
                if not self._cfg.wakeword.enabled:
                    self._session_active = True
                    await self._ensure_session_started()
                elif self._session_active and not self._client.is_in_session:
                    # KWS 早于 WS 连上：唤醒时已置 session_active，此处补建 session
                    self._node.get_logger().info(
                        "[doubao] wake before connect, starting session now"
                    )
                    await self._handle_wake_once()
                elif (
                    self._cfg.wakeword.enabled
                    and self._cfg.wakeword.pre_generate_greeting
                    and not self._session_active
                ):
                    self._schedule_pre_session()

                await self._client.wait_recv_done()
                if self._stop_event.is_set():
                    break
                self._node.get_logger().warn("[doubao] disconnected, will reconnect")
                self._session_active = False
                self._session_state = SessionState.DISCONNECTED
                self._reset_greeting_state()
                for _ in range(15):
                    if self._stop_event.is_set():
                        break
                    await asyncio.sleep(0.1)
            except Exception as e:
                self._node.get_logger().error(f"[doubao] connect loop error: {e}")
                self._session_active = False
                self._session_state = SessionState.DISCONNECTED
                await asyncio.sleep(2.0)

    async def _audio_uplink_worker(self):
        while not self._stop_event.is_set():
            try:
                frame = await self._audio_async_q.get()
                if self._stop_event.is_set():
                    continue
                if (
                    self._client
                    and self._session_active
                    and self._client.is_in_session
                    and not self._greeting_uplink_hold
                    and not self._exit_uplink_muted
                ):
                    if self._audio_tx_start_ts is None:
                        self._audio_tx_start_ts = time.perf_counter()
                        self._node.get_logger().info("[dialog] 开始传输音频")
                    await self._client.send_audio(frame)
            except Exception as e:
                self._node.get_logger().warn(f"[doubao] audio uplink error: {e}")
                await asyncio.sleep(0.05)

    def _offer_audio_frame_async(self, frm: bytes):
        try:
            self._audio_async_q.put_nowait(frm)
        except asyncio.QueueFull:
            try:
                _ = self._audio_async_q.get_nowait()
            except Exception:
                pass
            try:
                self._audio_async_q.put_nowait(frm)
            except Exception:
                pass

    async def _ensure_session_started(self):
        if not self._client:
            self._node.get_logger().warn(
                "[doubao] start session skipped: websocket not connected yet"
            )
            return
        if self._client.is_in_session:
            self._session_state = SessionState.SESSION_ACTIVE
            self._silence_phase = DialogPhase.DIALOG
            self._touch_activity()
            return

        self._session_state = SessionState.SESSION_STARTING
        self._pre_session_ready = False
        self._pre_session_pending = False
        await self._run_speaker_identify_if_needed()
        payload = self._build_start_session_payload()
        sid = await self._client.start_session(payload)
        self._node.get_logger().info(f"[doubao] session started: {sid}")
        self._last_cmd_id = ""
        self._last_cmd_arg = ""
        self._last_cmd_exec_ts = 0.0
        self._last_asr_param_ok = None
        self._last_asr_param_cmd_id = ""
        self._pending_question_id = ""
        self._active_question_id = ""
        self._session_state = SessionState.SESSION_ACTIVE
        self._silence_phase = DialogPhase.DIALOG
        self._touch_activity()

    async def _run_speaker_identify_if_needed(self) -> None:
        if self._speaker is None or not self._speaker.ready:
            return
        pcm = self._pending_wake_pcm
        self._pending_wake_pcm = b""
        if not pcm:
            return
        username = await asyncio.to_thread(self._speaker.identify_pcm, pcm)
        if username:
            self._identified_user = username
            self._node.get_logger().info(f"[speaker_id] session user: {username}")

    def _build_start_session_payload(self) -> dict:
        asr_extra = self._cfg.raw.get("asr", {}).get("extra", {})
        allow_cmd_hotword = bool(asr_extra.get("auto_inject_hotwords_from_voice_commands", True))
        cmd_hotwords = self._collect_command_hotwords() if allow_cmd_hotword else []
        payload = build_start_session_payload(self._cfg.raw, cmd_hotwords)
        if self._identified_user and self._cfg.speaker_id.inject_username_to_prompt:
            dialog = payload.setdefault("dialog", {})
            role = str(dialog.get("system_role", ""))
            dialog["system_role"] = (
                role
                + f"\n当前识别到的用户是「{self._identified_user}」，请用合适称呼与之对话。"
            )
        return payload

    def _collect_command_hotwords(self) -> list[str]:
        words: list[str] = []
        try:
            for cmd in self._cfg.command_entries:
                for pattern in cmd.get("patterns", []):
                    if pattern.get("type") != "keyword":
                        continue
                    for kw in pattern.get("words", []):
                        w = str(kw).strip()
                        if w:
                            words.append(w)
        except Exception:
            return []
        seen = set()
        out = []
        for w in words:
            if w not in seen:
                seen.add(w)
                out.append(w)
        return out

    # ──────────────────────────────────────────────────────────────
    # wake greeting / pre-session
    # ──────────────────────────────────────────────────────────────
    def _reset_greeting_state(self) -> None:
        self._cancel_wake_greeting_hold_task()
        self._greeting_sent = False
        self._generating_greeting = False
        self._pre_greeting_ready = False
        self._pre_greeting_pcm = bytearray()
        self._pre_session_pending = False
        self._pre_session_ready = False
        self._greeting_uplink_hold = False
        self._awaiting_wake_greeting_tts = False

    def _abort_pregen_on_wake(self) -> None:
        if self._pre_session_task and not self._pre_session_task.done():
            self._pre_session_task.cancel()
        self._generating_greeting = False
        self._pre_greeting_pcm = bytearray()
        self._pre_greeting_ready = False
        self._pre_session_pending = False
        self._pre_session_ready = False

    async def _send_keepalive_silence(self, *, frames: int = 5) -> None:
        """keep_alive 模式下先发少量静音，便于服务端返回 TTS。"""
        if not self._client or not self._client.is_in_session:
            return
        ms = max(10, int(self._cfg.audio.send_frame_ms))
        samples = int(16000 * ms / 1000)
        chunk = b"\x00" * (samples * 2)
        for _ in range(frames):
            await self._client.send_audio(chunk)
            await asyncio.sleep(ms / 1000.0)

    def _pregen_capture_active(self) -> bool:
        """仅空闲 pre-session 阶段把问候 TTS 写入内存，唤醒后必须走扬声器。"""
        return self._generating_greeting and not self._session_active

    def _schedule_pre_session(self) -> None:
        if self._stop_event.is_set() or self._shutting_down:
            return
        if not self._cfg.wakeword.enabled or not self._cfg.wakeword.pre_generate_greeting:
            return
        if self._session_active or self._pre_session_pending or self._pre_session_ready:
            return

        async def _delayed():
            await asyncio.sleep(0.3)
            if self._stop_event.is_set() or self._shutting_down:
                return
            if self._session_active or self._pre_session_pending or self._pre_session_ready:
                return
            await self._do_pre_session()

        if self._pre_session_task and not self._pre_session_task.done():
            return
        self._pre_session_task = asyncio.create_task(_delayed(), name="doubao-pre-session")

    async def _do_pre_session(self) -> None:
        if self._stop_event.is_set() or self._shutting_down:
            return
        if not self._cfg.wakeword.enabled or not self._cfg.wakeword.pre_generate_greeting:
            return
        if self._session_active or self._pre_session_pending or self._pre_session_ready:
            return
        if not self._client or self._client.is_in_session:
            return
        self._pre_session_pending = True
        self._pre_greeting_pcm = bytearray()
        self._pre_greeting_ready = False
        self._generating_greeting = False
        try:
            payload = self._build_start_session_payload()
            sid = await self._client.start_session(payload)
            self._pre_session_pending = False
            self._pre_session_ready = True
            self._log("info", f"[wake] pre-session ready: {sid}")
            await self._start_pregenerate_greeting()
        except Exception as e:
            self._pre_session_pending = False
            self._reset_greeting_state()
            self._log("warn", f"[wake] pre-session failed: {e!r}")

    async def _start_pregenerate_greeting(self) -> None:
        ww = self._cfg.wakeword
        if not ww.pre_generate_greeting or not ww.say_hello_direct_tts:
            return
        if not self._client or not self._client.is_in_session:
            return
        text = resolve_wake_greeting_text(self._cfg)
        if not text.strip():
            return
        self._pre_greeting_pcm = bytearray()
        self._pre_greeting_ready = False
        self._generating_greeting = True
        await self._send_keepalive_silence()
        await self._client.say_hello(text.strip())
        self._log("info", f"[wake] pre-generating SayHello greeting: {text}")

    async def _handle_wake_once(self) -> None:
        if self._wake_handler_busy:
            return
        self._wake_handler_busy = True
        try:
            await self._handle_wake()
        finally:
            self._wake_handler_busy = False

    async def _handle_wake(self) -> None:
        try:
            await self._ensure_session_started()
            self._silence_phase = DialogPhase.DIALOG

            ww = self._cfg.wakeword
            if ww.pre_generate_greeting:
                if self._consume_pre_greeting_playback():
                    self._abort_pregen_on_wake()
                    return
                pre_request_in_flight = (
                    self._generating_greeting
                    or self._pre_greeting_ready
                    or len(self._pre_greeting_pcm) > 0
                )
                if pre_request_in_flight:
                    deadline = time.monotonic() + self._pregen_wait_timeout_s
                    while time.monotonic() < deadline:
                        if self._consume_pre_greeting_playback():
                            self._abort_pregen_on_wake()
                            return
                        if not self._generating_greeting and self._pre_greeting_pcm:
                            self._pre_greeting_ready = True
                            if self._consume_pre_greeting_playback():
                                self._abort_pregen_on_wake()
                                return
                        await asyncio.sleep(0.05)
                if self._pre_greeting_pcm:
                    self._pre_greeting_ready = True
                    if self._consume_pre_greeting_playback():
                        self._abort_pregen_on_wake()
                        return

            self._abort_pregen_on_wake()

            if self._greeting_sent:
                return

            await self._send_keepalive_silence()
            await self._send_wake_greeting()
        finally:
            pass

    def _cancel_wake_greeting_hold_task(self) -> None:
        task = self._wake_greeting_hold_task
        self._wake_greeting_hold_task = None
        if task is not None and not task.done():
            task.cancel()

    def _schedule_wake_greeting_hold_watchdog(self, timeout_s: float = 6.0) -> None:
        self._cancel_wake_greeting_hold_task()
        if not self._loop.is_running():
            return
        self._wake_greeting_hold_task = self._loop.create_task(
            self._wake_greeting_hold_watchdog(timeout_s),
            name="wake-greeting-hold-watchdog",
        )

    async def _wake_greeting_hold_watchdog(self, timeout_s: float) -> None:
        try:
            await asyncio.sleep(timeout_s)
            if self._awaiting_wake_greeting_tts and self._greeting_uplink_hold:
                self._log(
                    "warn",
                    f"[wake] greeting TTS timeout ({timeout_s}s), release uplink hold",
                )
                self._greeting_uplink_hold = False
                self._awaiting_wake_greeting_tts = False
        except asyncio.CancelledError:
            pass

    def _consume_pre_greeting_playback(self) -> bool:
        if self._generating_greeting and self._pre_greeting_pcm:
            pcm = bytes(self._pre_greeting_pcm)
            self._pre_greeting_pcm = bytearray()
            self._generating_greeting = False
            self._pre_greeting_ready = False
            self._greeting_sent = True
            self._log(
                "info",
                f"[wake] playing in-flight pre-greeting ({len(pcm)} bytes)",
            )
            self._play_greeting_pcm(pcm)
            return True
        if not self._pre_greeting_ready or not self._pre_greeting_pcm:
            return False
        pcm = bytes(self._pre_greeting_pcm)
        self._pre_greeting_pcm = bytearray()
        self._pre_greeting_ready = False
        self._generating_greeting = False
        self._greeting_sent = True
        self._log("info", f"[wake] playing pre-generated greeting ({len(pcm)} bytes)")
        self._play_greeting_pcm(pcm)
        return True

    def _play_greeting_pcm(self, pcm: bytes) -> None:
        if not pcm:
            return
        reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_stats):
            reset_stats()
        chunk = 4800
        for i in range(0, len(pcm), chunk):
            self._on_tts_audio(pcm[i : i + chunk])
        self._touch_activity()

    async def _send_wake_greeting(self) -> None:
        if self._greeting_sent or not self._client or not self._client.is_in_session:
            return
        ww = self._cfg.wakeword
        self._awaiting_wake_greeting_tts = True
        self._drop_incoming_tts = False
        reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_stats):
            reset_stats()
        if ww.say_hello_direct_tts:
            text = resolve_wake_greeting_text(self._cfg).strip()
            if not text:
                self._awaiting_wake_greeting_tts = False
                self._greeting_uplink_hold = False
                return
            # SayHello 可在 Session 后立即播短句；ChatTTSText 需 ASREnded，易导致上行先开、问候迟播
            await self._client.say_hello(text)
            self._log("info", f"[wake] SayHello greeting: {text}")
            self._schedule_wake_greeting_hold_watchdog()
        elif ww.say_hello_use_model:
            query = resolve_wake_model_query(self._cfg).strip()
            if not query:
                self._awaiting_wake_greeting_tts = False
                return
            await self._client.send_chat_text_query(query)
            self._log("info", f"[wake] ChatTextQuery → model greeting, query={query!r}")
        else:
            spoken = resolve_wake_greeting_text(self._cfg).strip()
            if not spoken:
                self._awaiting_wake_greeting_tts = False
                return
            await self._client.say_hello(spoken)
            self._log("info", f"[wake] SayHello (spoken): {spoken}")
        self._greeting_sent = True
        self._touch_activity()

    def _matches_local_exit_keyword(self, text: str) -> bool:
        ei = self._cfg.exit_intent
        if not ei.enable_user_query_exit or not ei.local_exit_keywords:
            return False
        norm = _normalize_utterance(text)
        if not norm:
            return False
        for kw in ei.local_exit_keywords:
            k = _normalize_utterance(kw)
            if k and k in norm:
                return True
        return False

    def _cancel_exit_timeout_task(self) -> None:
        task = self._exit_timeout_task
        self._exit_timeout_task = None
        if task is not None and not task.done():
            task.cancel()

    def _cancel_exit_farewell_watch_task(self) -> None:
        task = self._exit_farewell_watch_task
        self._exit_farewell_watch_task = None
        if task is not None and not task.done():
            task.cancel()

    def _clear_session_exit_state(self) -> None:
        self._pending_session_exit = False
        self._pending_session_exit_reason = ""
        self._exit_uplink_muted = False
        self._exit_farewell_tts_started = False
        self._cancel_exit_timeout_task()
        self._cancel_exit_farewell_watch_task()

    def _schedule_exit_farewell_playback_watch(self) -> None:
        self._cancel_exit_farewell_watch_task()
        if not self._loop.is_running():
            return
        self._exit_farewell_watch_task = self._loop.create_task(
            self._watch_exit_farewell_playback(),
            name="exit-farewell-watch",
        )

    async def _watch_exit_farewell_playback(self) -> None:
        """告别 TTS 播完后结束会话（不依赖云端 TTSEnded，避免 KWS/二次 Interrupt 导致超时）。"""
        deadline = time.monotonic() + self._cfg.exit_intent.exit_farewell_timeout_s
        try:
            while time.monotonic() < deadline:
                if not self._pending_session_exit or self._session_exit_completing:
                    return
                if self._exit_farewell_tts_started and self._is_tts_playback_idle():
                    await asyncio.sleep(0.25)
                    if (
                        self._pending_session_exit
                        and not self._session_exit_completing
                        and self._is_tts_playback_idle()
                    ):
                        self._log(
                            "info",
                            "[exit] farewell playback idle, finish session",
                        )
                        await self._complete_session_exit(
                            reason=self._pending_session_exit_reason
                            or "farewell_playback_done"
                        )
                        return
                await asyncio.sleep(0.05)
        except asyncio.CancelledError:
            pass

    def _arm_session_exit(self, *, reason: str) -> None:
        if self._pending_session_exit or self._session_exit_completing:
            return
        self._pending_session_exit = True
        self._pending_session_exit_reason = reason
        self._exit_uplink_muted = True
        self._exit_farewell_tts_started = False
        self._log("info", f"[dialog] session exit armed: {reason}")
        self._cancel_exit_timeout_task()
        self._schedule_exit_farewell_playback_watch()
        timeout_s = self._cfg.exit_intent.exit_farewell_timeout_s
        if timeout_s > 0 and self._loop.is_running():
            self._exit_timeout_task = self._loop.create_task(
                self._exit_farewell_timeout(timeout_s),
                name="exit-farewell-timeout",
            )

    async def _exit_farewell_timeout(self, timeout_s: float) -> None:
        try:
            await asyncio.sleep(timeout_s)
            if self._pending_session_exit and not self._session_exit_completing:
                self._log(
                    "info",
                    f"[dialog] exit farewell timeout ({timeout_s}s), finish session",
                )
                await self._complete_session_exit(reason="exit_timeout", force=True)
        except asyncio.CancelledError:
            pass

    async def _complete_session_exit(self, *, reason: str, force: bool = False) -> None:
        if self._session_exit_completing:
            return
        if not force and not self._pending_session_exit:
            return
        self._session_exit_completing = True
        self._cancel_exit_timeout_task()
        try:
            self._cancel_deferred_music()
            self._cancel_song_tag_wait()
            self._cancel_pending_asr_defer()
            self._pending_song = ""
            self._session_active = False
            self._silence_phase = DialogPhase.WAKEUP
            self._audio_tx_start_ts = None
            self._silence_timeout_handled = False
            self._node.get_logger().info(f"[dialog] 对话结束: {reason}")
            try:
                self._pub_exit()
            except Exception:
                pass
            self._session_state = SessionState.CONNECTED
            if self._client and self._client.is_in_session:
                await self._client.finish_session()
            await self._on_session_finished_cleanup()
        finally:
            self._clear_session_exit_state()
            self._session_exit_completing = False

    async def _finish_session_and_cleanup(self) -> None:
        if self._client and self._client.is_in_session:
            await self._client.finish_session()
        await self._on_session_finished_cleanup()

    async def _on_session_finished_cleanup(self) -> None:
        self._clear_session_exit_state()
        self._reset_greeting_state()
        if (
            not self._shutting_down
            and self._cfg.wakeword.enabled
            and self._cfg.wakeword.pre_generate_greeting
            and not self._session_active
        ):
            self._schedule_pre_session()

    # ──────────────────────────────────────────────────────────────
    # event handling
    # ──────────────────────────────────────────────────────────────
    async def _on_frame_async(self, frame: ServerFrame):
        if frame.event == EVT_SessionFinished:
            self._session_active = False
            await self._on_session_finished_cleanup()
            return

        if frame.event == EVT_ASREnded:
            self._server_asr_ended = True
            return

        if frame.event == EVT_ASRInfo:
            self._server_asr_ended = False
            question_id = ""
            if frame.payload_json:
                question_id = str(frame.payload_json.get("question_id", "")).strip()
            if question_id and question_id != self._active_question_id:
                self._pending_question_id = question_id
                if self._active_question_id:
                    self._cmd_engine.reset_intent_buffer(clear_tag_gate=False)
            # 音乐/指令确认 TTS 窗口内忽略 VAD（多为回声），等 ASR-final
            if self._defer_in_progress or time.monotonic() < self._defer_tts_guard_until:
                return
            if self._pending_session_exit:
                return
            if self._awaiting_wake_greeting_tts or self._awaiting_biz_tts:
                return
            played_ms = self._tts_played_ms()
            period_ms = max(600, self._cmd_engine.period_cut_target_ms())
            queue_busy = not self._is_tts_playback_idle()
            if queue_busy or self._tts_playback_active:
                if played_ms < period_ms:
                    if self._debug_interrupt:
                        self._node.get_logger().debug(
                            f"[doubao][barge-in] ASRInfo deferred "
                            f"played_ms={played_ms}<{period_ms}"
                            + (
                                f" question_id={question_id}"
                                if question_id
                                else ""
                            )
                        )
                    return
                await self._interrupt_tts_playback(
                    reason="asr_info",
                    question_id=question_id,
                    suppress_followup=False,
                )
            elif self._debug_interrupt:
                self._node.get_logger().debug(
                    f"[doubao][barge-in] ASRInfo ignored (playback idle)"
                    + (f" question_id={question_id}" if question_id else "")
                )
            return

        if frame.event == EVT_TTSSentenceStart:
            if self._pregen_capture_active():
                return
            if self._awaiting_wake_greeting_tts:
                self._drop_incoming_tts = False
                self._tts_playback_active = True
                if frame.payload_json:
                    self._current_reply_id = str(
                        frame.payload_json.get("reply_id", "")
                    ).strip()
                reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
                if callable(reset_stats):
                    reset_stats()
                self._log("info", "[wake] greeting TTS started")
                return
            if self._awaiting_biz_tts:
                self._drop_incoming_tts = False
                self._tts_playback_active = True
                if frame.payload_json:
                    self._current_reply_id = str(
                        frame.payload_json.get("reply_id", "")
                    ).strip()
                reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
                if callable(reset_stats):
                    reset_stats()
                self._log("info", "[biz-tts] TTS started")
                return
            if self._pending_session_exit:
                self._exit_farewell_tts_started = True
                self._drop_incoming_tts = False
                self._tag_tts_tail_suppressed = False
                self._clear_tag_tts_gate()
                self._dropped_tts_chunks = 0
                self._dropped_tts_bytes = 0
                self._reply_tts_sentence_idx += 1
                self._tts_playback_active = True
                self._tts_did_duck = False
                if frame.payload_json:
                    self._current_reply_id = str(
                        frame.payload_json.get("reply_id", "")
                    ).strip()
                reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
                if callable(reset_stats):
                    reset_stats()
                self._log("info", "[exit] farewell TTS started")
                await self._music_duck_for_tts()
                return
            self._reply_tts_sentence_idx += 1
            if self._awaiting_injected_tts:
                self._drop_incoming_tts = False
                self._awaiting_injected_tts = False
                injected = self._injected_music_defer_active()
                if not injected:
                    self._tag_tts_tail_suppressed = False
                self._tag_tts_gate_active = False
                self._dropped_tts_chunks = 0
                self._dropped_tts_bytes = 0
                self._tts_playback_active = True
                self._tts_did_duck = False
                if frame.payload_json:
                    self._current_reply_id = str(
                        frame.payload_json.get("reply_id", "")
                    ).strip()
                if not injected:
                    reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
                    if callable(reset_stats):
                        reset_stats()
                if self._defer_in_progress:
                    self._defer_tts_ended = False
                    self._defer_saw_tts = True
                await self._music_duck_for_tts()
                return
            await self._sync_cmd_tag_tts_gate_async()
            if self._dropped_tts_chunks and self._debug_interrupt:
                self._node.get_logger().info(
                    f"[doubao][barge-in] new TTS sentence after suppress: "
                    f"dropped_chunks={self._dropped_tts_chunks} "
                    f"dropped_bytes={self._dropped_tts_bytes}"
                )
            self._dropped_tts_chunks = 0
            self._dropped_tts_bytes = 0
            tag_pending = (
                self._cmd_engine.tag_tts_tail_pending() or self._tag_tts_gate_active
            )
            tag_marker = self._cmd_engine.intent_buffer_has_tag_marker()
            if tag_marker or (
                self._reply_tts_sentence_idx > 1 and tag_pending
            ):
                await self._prepare_tag_tts_gate(reason="tag_tts_sentence")
            elif not self._cmd_engine.tag_tts_tail_pending():
                self._drop_incoming_tts = False
                if (
                    self._reply_tts_sentence_idx <= 1
                    and not self._injected_music_defer_active()
                ):
                    self._tag_tts_tail_suppressed = False
            self._tts_playback_active = True
            self._tts_did_duck = False
            if frame.payload_json:
                self._current_reply_id = str(frame.payload_json.get("reply_id", "")).strip()
            if self._reply_tts_sentence_idx <= 1:
                self._tag_tts_received_bytes = 0
                self._tag_gate_prepare_logged = False
                self._tag_cloud_cut_sent = False
            if not self._injected_music_defer_active():
                reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
                if callable(reset_stats) and not (
                    tag_pending and self._reply_tts_sentence_idx > 1
                ):
                    reset_stats()
            if self._defer_in_progress:
                self._defer_tts_ended = False
                self._defer_saw_tts = True
            await self._music_duck_for_tts()
            return

        if frame.event == EVT_TTSSentenceEnd:
            if self._defer_in_progress:
                self._defer_tts_ended = True
            return

        if frame.event == EVT_TTSResponse and frame.payload:
            if self._pregen_capture_active():
                self._pre_greeting_pcm.extend(frame.payload)
                return
            if self._awaiting_wake_greeting_tts:
                self._tts_playback_active = True
                self._on_tts_audio(frame.payload)
                self._touch_activity()
                return
            if self._awaiting_biz_tts:
                self._tts_playback_active = True
                self._on_tts_audio(frame.payload)
                self._touch_activity()
                return
            if self._pending_session_exit:
                self._exit_farewell_tts_started = True
                self._tts_playback_active = True
                self._on_tts_audio(frame.payload)
                self._touch_activity()
                return
            if (
                self._pending_asr_music is not None
                and not self._defer_in_progress
                and not self._awaiting_injected_tts
                and not self._tts_music_independent
            ):
                self._dropped_tts_chunks += 1
                self._dropped_tts_bytes += len(frame.payload)
                return
            if self._cmd_engine.tag_tts_tail_pending():
                tag_cmd = self._cmd_engine.match_intent_tags(
                    self._cmd_engine.intent_buffer_text()
                )
                if tag_cmd is not None:
                    self._try_arm_injected_music_confirm(tag_cmd.cmd_id)
            if (
                self._cmd_engine.tag_tts_tail_pending()
                and not self._tag_tts_gate_active
                and not self._tag_tts_tail_suppressed
                and not self._skip_tag_gate_for_injected_confirm()
            ):
                self._arm_tag_tts_gate(period_cap=True)
            if self._should_drop_cmd_tag_tts_chunk(frame.payload):
                self._dropped_tts_chunks += 1
                self._dropped_tts_bytes += len(frame.payload)
                if not self._tag_tts_tail_suppressed:
                    if not self._tag_tts_gate_active:
                        self._arm_tag_tts_gate(period_cap=True)
                    await self._mark_tag_tail_blocked(
                        reason="cmd_tag_tts_drop",
                        played_ms=self._tts_played_ms(),
                    )
                return
            if await self._gate_tag_tts_chunk(frame.payload):
                return
            await self._maybe_suppress_tag_tts_tail()
            if self._drop_incoming_tts:
                self._dropped_tts_chunks += 1
                self._dropped_tts_bytes += len(frame.payload)
                if self._debug_interrupt and self._dropped_tts_chunks <= 3:
                    self._node.get_logger().debug(
                        f"[doubao][barge-in] drop stale TTS chunk "
                        f"#{self._dropped_tts_chunks} bytes={len(frame.payload)}"
                    )
                return
            self._tts_playback_active = True
            if self._defer_in_progress:
                self._defer_last_tts_audio_ts = time.monotonic()
                self._defer_saw_tts = True
            if self._injected_music_defer_active():
                self._injected_playback_started = True
            self._on_tts_audio(frame.payload)
            self._touch_activity()
            return

        if frame.event == EVT_ASRResponse and frame.payload_json:
            if self._awaiting_wake_greeting_tts or self._awaiting_biz_tts:
                return
            results = frame.payload_json.get("results", [])
            for item in results:
                text = str(item.get("text", "")).strip()
                is_interim = bool(item.get("is_interim", False))
                if text and not is_interim:
                    await self._on_final_asr_text(text)
                    self._touch_activity()
            return

        if frame.event == EVT_ChatResponse and frame.payload_json:
            if self._awaiting_wake_greeting_tts or self._awaiting_biz_tts:
                return
            chat_qid = str(frame.payload_json.get("question_id", "")).strip()
            if chat_qid and not self._chat_question_id_accepted(chat_qid):
                return
            content = frame.payload_json.get("content", "")
            if content:
                cmd = self._cmd_engine.feed_intent_chunk(content)
                buf_after = self._cmd_engine.intent_buffer_text()
                if (
                    CMD_TAG_PATTERN.search(buf_after)
                    or _CMD_TAG_PARTIAL_RE.search(buf_after)
                ):
                    await self._halt_cmd_tag_tts_tail_async()
                elif self._cmd_engine.intent_tag_tts_cut_needed():
                    await self._sync_cmd_tag_tts_gate_async()
                    if CMD_TAG_PATTERN.search(buf_after):
                        if not self._should_defer_tag_cloud_cut():
                            await self._stop_tag_cloud_tts()
                if self._cmd_engine.should_log_chat_chunk(content):
                    self._node.get_logger().info(f"[doubao][chat] {content}")
                # 有 [CMD] 且未节流 → 标签路径；否则 pending 走短确认（勿用 elif 绑在 cmd 上）
                handled_chat_cmd = False
                if cmd and (time.time() - self._last_cmd_ts) > 0.5:
                    if self._pending_chat_complete_task is not None:
                        if not self._pending_chat_complete_task.done():
                            self._pending_chat_complete_task.cancel()
                        self._pending_chat_complete_task = None
                    self._last_cmd_ts = time.time()
                    self._log(
                        "info",
                        f"[cmd] L3 intent_tag {cmd.cmd_id} from chat buffer",
                    )
                    await self._halt_cmd_tag_tts_tail_async(force=True)
                    await self._handle_matched_command(cmd, source="chat")
                    self._cmd_engine.clear_matched_cmd_tags_from_buffer()
                    handled_chat_cmd = True
                if self._pending_asr_music is not None and not handled_chat_cmd:
                    buf = self._cmd_engine.intent_buffer_text()
                    has_cmd_marker = bool(
                        CMD_TAG_PATTERN.search(buf)
                        or _CMD_TAG_PARTIAL_RE.search(buf)
                    )
                    if self._chat_spoken_confirm_ready() and not has_cmd_marker:
                        await self._finish_pending_asr_chat_confirm()
                    elif (
                        not has_cmd_marker
                        and len(buf) >= 30
                        and not any(h in buf for h in _CHAT_CONFIRM_HINTS)
                        and self._pending_asr_music[1] in _MUSIC_LYRICS_EARLY_CMD_IDS
                        and not self._defer_in_progress
                    ):
                        # 歌词流早检测：点歌/切歌 pending 但 LLM 已生成 ≥30 字无 CMD
                        # 标签 → 判定为歌词，提前取消等待、直接执行
                        pending = self._pending_asr_music
                        self._pending_asr_music = None
                        self._cancel_asr_music_tag_wait_only()
                        music_action, cmd_id, _ = pending
                        self._log(
                            "info",
                            f"[cmd] lyrics stream detected ({len(buf)} chars): "
                            f"early execute {cmd_id}",
                        )
                        asyncio.create_task(
                            self._execute_quick_music_and_interrupt(
                                music_action, cmd_id
                            ),
                            name=f"lyrics-early-{cmd_id}",
                        )
                    else:
                        self._schedule_pending_chat_complete()
                if not self._pending_asr_music and (
                    CMD_TAG_PATTERN.search(self._cmd_engine.intent_buffer_text())
                    or _CMD_TAG_PARTIAL_RE.search(
                        self._cmd_engine.intent_buffer_text()
                    )
                ):
                    await self._sync_cmd_tag_tts_gate_async()
            self._touch_activity()
            return

        if frame.event == EVT_TTSEnded:
            if self._generating_greeting:
                self._generating_greeting = False
                if self._greeting_sent:
                    self._pre_greeting_pcm = bytearray()
                    self._pre_greeting_ready = False
                    self._log(
                        "info",
                        "[wake] discard late pre-greeting (greeting already played)",
                    )
                    return
                self._pre_greeting_ready = True
                self._log(
                    "info",
                    f"[wake] pre-greeting ready ({len(self._pre_greeting_pcm)} bytes)",
                )
                return
            if self._awaiting_wake_greeting_tts:
                self._cancel_wake_greeting_hold_task()
                self._awaiting_wake_greeting_tts = False
                self._greeting_uplink_hold = False
                self._tts_playback_active = False
                self._log("info", "[wake] greeting TTS ended")
                self._publish_playback_done_safe()
                self._touch_activity()
                return
            if self._awaiting_biz_tts:
                self._finish_biz_tts_playback(reason="tts_ended")
                return
            if self._dropped_tts_chunks:
                self._node.get_logger().info(
                    f"[doubao][barge-in] TTSEnded: suppressed "
                    f"{self._dropped_tts_chunks} chunk(s), "
                    f"{self._dropped_tts_bytes} bytes total"
                )
            self._dropped_tts_chunks = 0
            self._dropped_tts_bytes = 0
            if self._defer_in_progress:
                if (
                    self._defer_confirm_source == "injected"
                    and not self._defer_saw_tts
                ):
                    # 注入 TTS 的 TTSSentenceStart 还未收到，此 TTSEnded 来自被打断的
                    # 模型 TTS，不能标记为 defer 结束
                    pass
                elif (
                    self._defer_confirm_source == "injected"
                    and self._defer_saw_tts
                    and not self._injected_playback_started
                ):
                    # TTSEnded 早于音频实际写出 80ms：服务端已发完数据但本地播放器
                    # 队列还未消费。设置 pending 标志，wait loop 在 playback_started
                    # 为 True 后立即接收。
                    self._injected_tts_ended_pending = True
                else:
                    self._defer_tts_ended = True
            else:
                self._drop_incoming_tts = False
                self._tag_tts_tail_suppressed = False
                self._tag_gate_prepare_logged = False
                self._clear_tag_tts_gate()
                await self._flush_volume_cmd_from_chat_buffer()
                await self._flush_param_cmd_from_chat_buffer()
            await self._music_unduck_after_tts()
            self._publish_playback_done_safe()
            self._touch_activity()
            status_code = ""
            if frame.payload_json:
                status_code = str(frame.payload_json.get("status_code", ""))
            if status_code == "20000002":
                await self._complete_session_exit(
                    reason="cloud_exit_intent", force=True
                )
            elif self._pending_session_exit:
                exit_reason = self._pending_session_exit_reason or "farewell_done"
                self._log("info", "[exit] farewell TTS ended (cloud)")
                await self._complete_session_exit(reason=exit_reason)
            return

        if frame.event == EVT_UsageResponse and frame.payload_json:
            usage = frame.payload_json.get("usage", {})
            self._node.get_logger().debug(f"[doubao][usage] {usage}")
            return

        if frame.event == EVT_DialogCommonError:
            self._node.get_logger().warn(f"[doubao][error] {frame.payload_json}")
            return

    def _chat_question_id_accepted(self, question_id: str) -> bool:
        qid = str(question_id or "").strip()
        if not qid:
            return True
        if qid == self._active_question_id or qid == self._pending_question_id:
            return True
        return False

    def _cmd_dedup_key(self, matched: CmdResult) -> Tuple[str, str]:
        cid = str(matched.cmd_id or "")
        arg = str(matched.arg or "").strip()
        if cid.startswith("volume_") or cid in (
            "massage_position",
            "massage_duration",
            "massage_force",
            "head_param",
        ):
            return (cid, arg)
        return (cid, "")

    def _is_duplicate_cmd(self, matched: CmdResult, *, window_s: float) -> bool:
        key = self._cmd_dedup_key(matched)
        last_key = (self._last_cmd_id, self._last_cmd_arg)
        return key == last_key and (time.time() - self._last_cmd_exec_ts) < window_s

    async def _flush_volume_cmd_from_chat_buffer(self) -> None:
        """模型晚到 [CMD:volume_*] 时，在 TTSEnded 再执行一次（ASR 已执行则会被去重）。"""
        if self._cmd_busy:
            return
        cmd = self._cmd_engine.match_intent_tags(self._cmd_engine.intent_buffer_text())
        if cmd is None or not str(cmd.cmd_id).startswith("volume_"):
            return
        if self._is_duplicate_cmd(cmd, window_s=3.0):
            return
        self._log("info", f"[cmd] flush volume tag from chat buffer: {cmd.cmd_id}")
        await self._handle_matched_command(cmd, source="chat")

    async def _flush_param_cmd_from_chat_buffer(self) -> None:
        """模型晚到/拆包的按摩参数标签，在 TTSEnded 再执行（ASR 已成功则 dedup）。"""
        if self._cmd_busy:
            return
        cmd = self._cmd_engine.flush_param_cmd_from_buffer()
        if cmd is None or cmd.cmd_id not in _MASSAGE_PARAM_CMD_IDS:
            return
        if self._is_duplicate_cmd(cmd, window_s=3.0):
            return
        self._log(
            "info",
            f"[cmd] flush param tag from chat buffer: {cmd.cmd_id} arg={cmd.arg!r}",
        )
        await self._handle_matched_command(cmd, source="chat")

    def _mark_asr_param_scheduled(self, cmd_id: str) -> None:
        if cmd_id not in _MASSAGE_PARAM_CMD_IDS:
            return
        self._last_asr_param_cmd_id = cmd_id
        self._last_asr_param_ok = None

    def _record_param_cmd_result(self, cmd_id: str, *, source: str, ok: bool) -> None:
        if source != "asr" or cmd_id not in _MASSAGE_PARAM_CMD_IDS:
            return
        if self._last_asr_param_cmd_id == cmd_id:
            self._last_asr_param_ok = ok

    async def _on_final_asr_text(self, text: str):
        if self._awaiting_wake_greeting_tts or self._awaiting_biz_tts:
            return
        if self._pending_session_exit or self._session_exit_completing:
            return
        now = time.time()
        if text == self._last_final_text and (now - self._last_final_ts) < 1.0:
            return
        self._last_final_text = text
        self._last_final_ts = now
        # 新一句用户话：结束上一轮标签门控，避免误裁下一句 TTS
        if (
            self._tag_tts_gate_active
            or self._tag_tts_tail_suppressed
            or self._cmd_engine.tag_tts_tail_pending()
        ):
            self._clear_tag_tts_gate()
        self._reset_turn_tts_state()
        self._drop_incoming_tts = False
        self._cmd_engine.reset_intent_buffer(clear_tag_gate=True)
        self._last_asr_param_ok = None
        self._last_asr_param_cmd_id = ""
        self._reply_tts_sentence_idx = 0
        self._pending_asr_music_utterance = text.strip()
        if self._pending_question_id:
            self._active_question_id = self._pending_question_id

        self._node.get_logger().info(f"[ASR-final] {text}")
        # 部分链路 ASREnded 晚于 ASR-final；允许 ChatTTSText 注入
        self._server_asr_ended = True

        if self._is_bare_pause_utterance(text):
            await self._ask_pause_clarification()
            return

        matched = self._cmd_engine.match_asr(text)
        if matched is not None and matched.cmd_id == "music_stop":
            self._cancel_pending_asr_defer()
        if matched is not None and str(matched.cmd_id).startswith("volume_"):
            self._log(
                "info",
                f"[cmd] ASR matched {matched.cmd_id} arg={matched.arg!r} -> execute now",
            )
        if matched is not None and matched.cmd_id in (
            "music_pause",
            "massage_pause",
        ):
            self._awaiting_pause_clarification_until = 0.0
        if matched is not None and matched.cmd_id in _MASSAGE_CMD_IDS:
            self._log(
                "info",
                f"[cmd] ASR matched {matched.cmd_id} -> execute massage ROS2",
            )
        if (
            self._injected_music_defer_active()
            and not self._defer_music_executed
            and matched is not None
            and matched.cmd_id in ("music_stop", "dialog_stop_speech")
        ):
            self._log(
                "info",
                f"[cmd] cancel injected defer: user ASR {matched.cmd_id}",
            )
            self._cancel_deferred_music()
        # defer 等待确认 TTS 时：非命令的 ASR（寒暄/误识别）不打断，避免永远播不出音乐
        if matched is None and self._defer_in_progress:
            return

        if not matched:
            if (
                self._cfg.exit_intent.enable_user_query_exit
                and self._matches_local_exit_keyword(text)
            ):
                recent_interrupt = (
                    self._recent_client_interrupt_mono > 0
                    and (time.monotonic() - self._recent_client_interrupt_mono)
                    < 2.5
                )
                await self._interrupt_tts_playback(
                    reason="exit_keyword",
                    send_client_interrupt=not recent_interrupt,
                    truncate_context=not recent_interrupt,
                    suppress_followup=False,
                )
                self._drop_incoming_tts = False
                self._dropped_tts_chunks = 0
                self._dropped_tts_bytes = 0
                self._arm_session_exit(reason="exit_keyword")
            return

        if (
            self._defer_in_progress
            and matched.cmd_id in _MUSIC_DEFER_CMD_IDS
        ):
            self._log(
                "info",
                f"[cmd] new ASR {matched.cmd_id} supersedes in-flight defer",
            )
            self._cancel_deferred_music()

        await self._handle_matched_command(matched, source="asr")

    async def _run_massage_ros2_service(
        self, service_name: str, cmd_id: str, *, source: str
    ) -> None:
        """后台调用按摩服务；不阻塞 Chat TTS，也不在 ASR 路径上 ClientInterrupt。"""
        try:
            result = await self._ros2.call_massage_service(service_name)
            if result.ok:
                self._node.get_logger().info(
                    f"[cmd] {cmd_id} ({source}): {service_name} OK"
                )
            else:
                self._node.get_logger().warn(
                    f"[cmd] {cmd_id} ({source}): {service_name} failed: "
                    f"{result.message}"
                )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._node.get_logger().warn(
                f"[cmd] {cmd_id} ({source}): {service_name} error: {e}"
            )

    async def _run_massage_position(
        self, direction: str, cmd_id: str, *, source: str
    ) -> None:
        ok = False
        try:
            result = await self._ros2.call_massage_position(direction or "")
            ok = result.ok
            if result.ok:
                self._node.get_logger().info(f"[cmd] {cmd_id} ({source}): position OK")
            else:
                self._node.get_logger().warn(
                    f"[cmd] {cmd_id} ({source}): position failed: {result.message}"
                )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._node.get_logger().warn(
                f"[cmd] {cmd_id} ({source}): position error: {e}"
            )
        finally:
            self._record_param_cmd_result(cmd_id, source=source, ok=ok)

    async def _run_massage_duration(
        self, tag_arg: Optional[str], cmd_id: str, *, source: str
    ) -> None:
        ok = False
        try:
            result = await self._ros2.call_massage_duration_from_tag(tag_arg)
            ok = result.ok
            if result.ok:
                self._node.get_logger().info(f"[cmd] {cmd_id} ({source}): duration OK")
            else:
                self._node.get_logger().warn(
                    f"[cmd] {cmd_id} ({source}): duration failed: {result.message}"
                )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._node.get_logger().warn(
                f"[cmd] {cmd_id} ({source}): duration error: {e}"
            )
        finally:
            self._record_param_cmd_result(cmd_id, source=source, ok=ok)

    async def _run_massage_force(
        self, tag_arg: Optional[str], cmd_id: str, *, source: str
    ) -> None:
        ok = False
        try:
            result = await self._ros2.call_massage_force_from_tag(tag_arg)
            ok = result.ok
            if result.ok:
                self._node.get_logger().info(
                    f"[cmd] {cmd_id} ({source}): force OK "
                    f"(current={result.current_force:.1f}N)"
                )
            else:
                self._node.get_logger().warn(
                    f"[cmd] {cmd_id} ({source}): force failed: {result.message}"
                )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._node.get_logger().warn(
                f"[cmd] {cmd_id} ({source}): force error: {e}"
            )
        finally:
            self._record_param_cmd_result(cmd_id, source=source, ok=ok)

    async def _run_head_param(
        self, tag_arg: Optional[str], cmd_id: str, *, source: str
    ) -> None:
        ok = False
        try:
            result = await self._ros2.call_head_param_from_tag(tag_arg)
            ok = result.ok
            if result.ok:
                self._node.get_logger().info(f"[cmd] {cmd_id} ({source}): head_param OK")
            else:
                self._node.get_logger().warn(
                    f"[cmd] {cmd_id} ({source}): head_param failed: {result.message}"
                )
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self._node.get_logger().warn(
                f"[cmd] {cmd_id} ({source}): head_param error: {e}"
            )
        finally:
            self._record_param_cmd_result(cmd_id, source=source, ok=ok)

    async def _handle_matched_command(
        self, matched: CmdResult, *, source: str = "asr"
    ) -> None:
        now = time.time()
        if self._cmd_busy:
            self._node.get_logger().info(f"[cmd] skip {matched.cmd_id}: previous command still running")
            return
        pending_asr_same = (
            source == "chat"
            and self._pending_asr_music is not None
            and self._pending_asr_music[1] == matched.cmd_id
        )
        if (
            source == "chat"
            and matched.cmd_id == "music_pause"
            and self._last_cmd_id == "music_pause"
            and (now - self._last_cmd_exec_ts) < 2.5
            and (
                self._defer_music_executed
                or not self._defer_in_progress
            )
        ):
            self._cmd_engine.clear_matched_cmd_tags_from_buffer()
            self._log(
                "info",
                "[cmd] skip chat music_pause: pause already executed this turn",
            )
            return
        if source == "chat" and self._should_skip_duplicate_chat_music(matched):
            self._cmd_engine.clear_matched_cmd_tags_from_buffer()
            return
        if (
            source == "chat"
            and matched.cmd_id in _MASSAGE_PARAM_CMD_IDS
            and self._last_asr_param_cmd_id == matched.cmd_id
            and self._last_asr_param_ok is True
            and (now - self._last_cmd_exec_ts) < 2.5
        ):
            self._cmd_engine.clear_matched_cmd_tags_from_buffer()
            self._log(
                "info",
                f"[cmd] skip chat {matched.cmd_id}: ASR already scheduled ROS2",
            )
            return
        dedup_window = 3.0 if str(matched.cmd_id).startswith("volume_") else 1.5
        if (
            self._is_duplicate_cmd(matched, window_s=dedup_window)
            and not (
                matched.cmd_id == "music_play_song"
                and source == "chat"
                and (matched.arg or "").strip()
            )
            and not pending_asr_same
            and not (
                source == "chat"
                and self._defer_in_progress
                and matched.cmd_id in _MUSIC_DEFER_CMD_IDS
            )
        ):
            self._node.get_logger().info(f"[cmd] skip duplicate {matched.cmd_id}")
            self._cmd_engine.clear_matched_cmd_tags_from_buffer()
            return

        self._cmd_busy = True
        self._last_cmd_id = matched.cmd_id
        self._last_cmd_arg = str(matched.arg or "").strip()
        self._last_cmd_exec_ts = now

        try:
            entry = self._cmd_engine.get_command(matched.cmd_id) or {}
            reply = str(matched.reply or entry.get("tts_confirm", "")).strip()
            skip_tts_confirm = False

            if matched.action == "unsupported_ros2":
                svc = str(entry.get("ros2_service", "")).strip()
                self._node.get_logger().warn(
                    f"[cmd] {matched.cmd_id} ({source}): "
                    f"ros2 service not wired in voice_asr_node ({svc})"
                )
                if source == "chat":
                    skip_tts_confirm = True
                return

            if matched.action == "play_song_defer":
                song = self._sanitize_song_name(matched.arg or "")
                if not song or song in _PLAY_SONG_REJECT_NAMES:
                    self._node.get_logger().warn(
                        f"[cmd] music_play_song: invalid song name {song!r}, ignored"
                    )
                    return
                if source == "asr":
                    self._node.get_logger().info(
                        f"[cmd] music_play_song from ASR ignored "
                        f"(match_mode={self._cmd_engine.match_mode}), "
                        f"wait LLM [CMD:music_play_song:…]"
                    )
                    return
                self._cancel_song_tag_wait()
                if self._defer_in_progress:
                    self._cancel_deferred_music()
                self._pending_song = song
                self._log(
                    "info",
                    f"[cmd] chat tag play_song defer: {song!r}",
                )
                await self._begin_deferred_music_from_chat_tag(
                    f"play_named:{song}", matched.cmd_id, entry, matched
                )
                return

            if matched.action == "confirm_song_play":
                song = self._sanitize_song_name(self._pending_song)
                if song and not self._defer_in_progress:
                    self._schedule_deferred_music(
                        f"play_named:{song}", "music_confirm_play"
                    )
                return

            if matched.action == "music_volume":
                ao = self._cfg.audio_output
                min_v = float(ao.min_volume_percent)
                max_v = float(ao.max_volume_percent)
                mode = str(entry.get("volume_mode", "")).strip().lower()
                if mode == "absolute":
                    raw_arg = (matched.arg or "").strip()
                    if not raw_arg and entry.get("volume_level") is not None:
                        raw_arg = str(entry.get("volume_level"))
                    utterance = (
                        self._pending_asr_music_utterance if source == "asr" else ""
                    )
                    norm_arg = normalize_volume_arg(
                        raw_arg, utterance=utterance
                    )
                    if not norm_arg:
                        self._node.get_logger().info(
                            f"[cmd] skip {matched.cmd_id} ({source}): "
                            "empty volume level (avoid [CMD:volume_set] without :N)"
                        )
                        self._cmd_engine.clear_matched_cmd_tags_from_buffer()
                        return
                    target = float(norm_arg)
                    result = await self._ros2.set_music_volume(
                        target, timeout_s=5.0
                    )
                else:
                    delta = float(entry.get("volume_delta", ao.volume_step_percent))
                    result = await self._ros2.adjust_music_volume(
                        delta,
                        min_percent=min_v,
                        max_percent=max_v,
                        timeout_s=5.0,
                    )
                if result.ok:
                    mark = getattr(self._node, "mark_music_volume_initialized", None)
                    if callable(mark):
                        mark()
                    self._node.get_logger().info(
                        f"[cmd] {matched.cmd_id} ({source}): "
                        f"volume={result.current_volume:.0f}%"
                    )
                else:
                    self._node.get_logger().warn(
                        f"[cmd] {matched.cmd_id} volume failed: {result.message}"
                    )
                self._cmd_engine.clear_matched_cmd_tags_from_buffer()
                return

            if matched.action == "massage_position":
                skip_tts_confirm = True
                if source == "asr":
                    self._mark_asr_param_scheduled(matched.cmd_id)
                asyncio.create_task(
                    self._run_massage_position(
                        matched.arg or "", matched.cmd_id, source=source
                    ),
                    name=f"cmd:{matched.cmd_id}",
                )
                return

            if matched.action == "massage_duration":
                skip_tts_confirm = True
                if source == "asr":
                    self._mark_asr_param_scheduled(matched.cmd_id)
                asyncio.create_task(
                    self._run_massage_duration(
                        matched.arg, matched.cmd_id, source=source
                    ),
                    name=f"cmd:{matched.cmd_id}",
                )
                return

            if matched.action == "massage_force":
                skip_tts_confirm = True
                if source == "asr":
                    self._mark_asr_param_scheduled(matched.cmd_id)
                asyncio.create_task(
                    self._run_massage_force(
                        matched.arg, matched.cmd_id, source=source
                    ),
                    name=f"cmd:{matched.cmd_id}",
                )
                return

            if matched.action == "head_param":
                skip_tts_confirm = True
                if source == "asr":
                    self._mark_asr_param_scheduled(matched.cmd_id)
                asyncio.create_task(
                    self._run_head_param(
                        matched.arg, matched.cmd_id, source=source
                    ),
                    name=f"cmd:{matched.cmd_id}",
                )
                return

            if matched.action == "massage_query":
                skip_tts_confirm = True
                result = await self._ros2.get_massage_progress_summary()
                summary = (
                    result.message
                    if result.message
                    else ("查询成功。" if result.ok else "查询按摩进度失败。")
                )
                await self._ensure_session_started()
                if self._client:
                    await self._send_chat_tts_text(summary)
                self._cmd_engine.clear_matched_cmd_tags_from_buffer()
                return

            if matched.action == "stop_tts":
                # ASRInfo 常已 ClientInterrupt；再发会截断云端告别合成（与 _drop_incoming_tts 无关）
                recent_interrupt = (
                    self._recent_client_interrupt_mono > 0
                    and (time.monotonic() - self._recent_client_interrupt_mono)
                    < 2.5
                )
                await self._interrupt_tts_playback(
                    reason=f"cmd:{matched.cmd_id}",
                    send_client_interrupt=not recent_interrupt,
                    truncate_context=not recent_interrupt,
                    suppress_followup=False,
                )
                self._drop_incoming_tts = False
                self._dropped_tts_chunks = 0
                self._dropped_tts_bytes = 0
                self._node.get_logger().info(
                    f"[cmd] {matched.cmd_id}: ready for cloud farewell TTS"
                    f" (skip_second_interrupt={'yes' if recent_interrupt else 'no'})"
                )
                self._arm_session_exit(reason=f"cmd:{matched.cmd_id}")
                return

            if matched.action == "ros2_service":
                music_action = str(entry.get("music_action", "")).strip()
                service_name = str(entry.get("ros2_service", "")).strip()

                if music_action and service_name == MUSIC_CONTROL_SERVICE:
                    if (
                        music_action == "play"
                        and self._user_music_paused
                        and matched.cmd_id == "music_play"
                    ):
                        music_action = "user_resume"
                        matched = CmdResult(
                            cmd_id="music_resume",
                            action=matched.action,
                            reply=str(entry.get("tts_confirm", "")),
                        )
                        entry = self._cmd_engine.get_command("music_resume") or entry

                    if (
                        music_action in _MUSIC_ACTIONS_DEFER_FOR_TTS
                        or music_action in _MUSIC_ACTIONS_IMMEDIATE
                        or music_action.startswith("play_named:")
                    ):
                        skip_tts_confirm = True
                        reply = ""
                        if music_action == "stop":
                            self._cancel_deferred_music()
                            self._cancel_song_tag_wait()
                            self._cancel_pending_asr_defer()
                            self._pending_song = ""
                            self._tts_did_duck = False
                        elif music_action not in ("user_resume",):
                            self._user_music_paused = False
                        defer_play = (
                            music_action in _MUSIC_ACTIONS_DEFER_FOR_TTS
                            or music_action.startswith("play_named:")
                        )
                        if defer_play and source == "asr":
                            if (
                                self._tts_music_independent
                                and matched.cmd_id in _MUSIC_QUICK_CMD_IDS
                            ):
                                self._cancel_pending_asr_defer()
                                await self._execute_music_parallel_with_tts(
                                    music_action, matched.cmd_id
                                )
                                return
                            self._set_pending_asr_defer(
                                music_action, matched.cmd_id, entry, matched
                            )
                            return
                        if (
                            defer_play
                            and source == "chat"
                            and self._tts_music_independent
                        ):
                            await self._execute_music_parallel_with_tts(
                                music_action, matched.cmd_id
                            )
                            return
                        if (
                            defer_play
                            and source == "chat"
                            and (
                                self._cmd_engine.intent_tag_tts_cut_needed()
                                or self._cmd_engine.intent_buffer_has_tag_marker()
                            )
                            and self._use_injected_confirm(matched.cmd_id)
                        ):
                            if self._injected_music_defer_active():
                                self._cmd_engine.clear_matched_cmd_tags_from_buffer()
                                self._log(
                                    "info",
                                    f"[cmd] skip chat tag {matched.cmd_id}: "
                                    f"injected confirm already in flight",
                                )
                                return
                            if self._same_injected_defer_active(
                                music_action, matched.cmd_id
                            ):
                                self._log(
                                    "debug",
                                    f"[cmd] skip duplicate chat tag {matched.cmd_id} "
                                    f"(injected defer active)",
                                )
                                return
                            await self._begin_deferred_music_from_chat_tag(
                                music_action,
                                matched.cmd_id,
                                entry,
                                matched,
                            )
                            return
                        self._schedule_deferred_music(
                            music_action, matched.cmd_id
                        )
                elif service_name:
                    if is_massage_service(service_name) or matched.cmd_id in _MASSAGE_CMD_IDS:
                        # 与音乐 Chat 路径一致：确认语由豆包 TTS 播放，本地不 ChatTTSText、不 Interrupt
                        skip_tts_confirm = True
                        asyncio.create_task(
                            self._run_massage_ros2_service(
                                service_name,
                                matched.cmd_id,
                                source=source,
                            ),
                            name=f"cmd:{matched.cmd_id}",
                        )
                        return
                    else:
                        self._node.get_logger().warn(
                            f"[cmd] unsupported ros2_service {service_name!r} "
                            f"for {matched.cmd_id} (not in bridge)"
                        )

            if reply and self._client and not skip_tts_confirm:
                await self._ensure_session_started()
                await self._send_chat_tts_text(reply)
        finally:
            self._cmd_busy = False

    def _cancel_deferred_music(self) -> None:
        self._deferred_music = None
        self._defer_in_progress = False
        self._defer_tts_ended = False
        self._defer_confirm_source = "model"
        self._awaiting_injected_tts = False
        self._injected_defer_key = None
        self._injected_defer_started_mono = 0.0
        self._injected_confirm_text = ""
        self._injected_confirm_target_ms = 0
        self._injected_playback_started = False
        self._injected_chat_tts_sent_mono = 0.0
        self._injected_armed = False
        self._injected_tts_resend = False
        self._injected_tts_ended_pending = False
        if self._deferred_music_task is not None and not self._deferred_music_task.done():
            self._deferred_music_task.cancel()
        self._deferred_music_task = None

    def _cancel_pending_asr_defer(self) -> None:
        self._pending_asr_music = None
        self._pending_asr_music_utterance = ""
        self._cancel_asr_music_tag_wait_only()
        if (
            self._pending_chat_complete_task is not None
            and not self._pending_chat_complete_task.done()
        ):
            self._pending_chat_complete_task.cancel()
        self._pending_chat_complete_task = None

    def _cancel_asr_music_tag_wait_only(self) -> None:
        if (
            self._asr_music_tag_wait_task is not None
            and not self._asr_music_tag_wait_task.done()
        ):
            self._asr_music_tag_wait_task.cancel()
        self._asr_music_tag_wait_task = None

    @staticmethod
    def _is_bare_pause_utterance(text: str) -> bool:
        return _normalize_utterance(text) == "暂停"

    async def _ask_pause_clarification(self) -> None:
        """裸「暂停」：不执行按摩/音乐，追问用户明确意图。"""
        self._awaiting_pause_clarification_until = (
            time.monotonic() + _PAUSE_CLARIFY_TIMEOUT_S
        )
        self._log("info", "[cmd] bare pause: ask massage vs music (no action)")
        await self._ensure_session_started()
        if self._client and self._client.is_in_session:
            await self._send_chat_tts_text(_PAUSE_CLARIFY_TEXT)

    @staticmethod
    def _spoken_implies_music_stop(text: str) -> bool:
        s = str(text or "").strip()
        if not s:
            return False
        if CMD_TAG_PATTERN.search(s):
            for m in CMD_TAG_PATTERN.finditer(s):
                if m.group(1) in ("music_stop",):
                    return True
            return False
        if any(h in s for h in _MUSIC_STOP_SPOKEN_HINTS):
            return True
        if "不听" in s and ("歌" in s or "音乐" in s):
            return True
        return False

    def _chat_spoken_confirm_ready(self) -> bool:
        """Chat 已给出短句确认（句号结尾），但未必带 [CMD]（模型常漏打标签）。"""
        buf = self._cmd_engine.intent_buffer_text()
        if CMD_TAG_PATTERN.search(buf) or _CMD_TAG_PARTIAL_RE.search(buf):
            return False
        spoken = self._cmd_engine.spoken_text_before_tag()
        if self._spoken_implies_music_stop(spoken) or self._spoken_implies_music_stop(buf):
            return False
        if not any(h in spoken for h in _CHAT_CONFIRM_HINTS):
            return False
        tail = spoken.rstrip()
        if not tail.endswith(("。", "！", "？", ".", "!", "?")):
            return False
        pending = self._pending_asr_music
        if pending is not None:
            cmd_id = pending[1]
            # 切歌/上一首：模型常只回「好的。」，不必等长句或播放类关键词
            if cmd_id in _MUSIC_QUICK_CMD_IDS:
                return 2 <= len(spoken) <= 24
            # 点歌/恢复：单句「好的。」过短，易在标签 TTS 未门控前误播歌
            if len(spoken) < 8:
                return False
            if not any(h in spoken for h in _MUSIC_PLAY_SPOKEN_HINTS):
                return False
            return len(spoken) <= 32
        return 4 <= len(spoken) <= 32

    def _schedule_pending_chat_complete(self) -> None:
        if self._pending_asr_music is None or self._defer_in_progress:
            return
        if (
            self._pending_chat_complete_task is not None
            and not self._pending_chat_complete_task.done()
        ):
            self._pending_chat_complete_task.cancel()
        self._pending_chat_complete_task = asyncio.create_task(
            self._pending_chat_complete_worker(),
            name="pending-chat-complete",
        )

    async def _finish_pending_asr_chat_confirm(self) -> None:
        if self._pending_asr_music is None or self._defer_in_progress:
            return
        buf = self._cmd_engine.intent_buffer_text()
        if CMD_TAG_PATTERN.search(buf) or _CMD_TAG_PARTIAL_RE.search(buf):
            return
        if self._spoken_implies_music_stop(buf) or self._spoken_implies_music_stop(
            self._pending_asr_music_utterance
        ):
            self._log(
                "info",
                "[cmd] cancel pending music_play: chat/ASR implies stop",
            )
            self._cancel_pending_asr_defer()
            stop = self._cmd_engine.get_command("music_stop")
            if stop:
                await self._handle_matched_command(
                    CmdResult(
                        cmd_id="music_stop",
                        action=str(stop.get("action", "")),
                        reply=str(stop.get("tts_confirm", "")),
                    ),
                    source="chat_inferred_stop",
                )
            return
        if not self._chat_spoken_confirm_ready():
            return
        if (
            self._pending_chat_complete_task is not None
            and not self._pending_chat_complete_task.done()
        ):
            self._pending_chat_complete_task.cancel()
            self._pending_chat_complete_task = None
        self._cancel_asr_music_tag_wait_only()
        music_action, cmd_id, _fallback = self._pending_asr_music
        entry = self._cmd_engine.get_command(cmd_id) or {}
        matched = CmdResult(
            cmd_id=cmd_id,
            action="ros2_service",
            reply=str(entry.get("tts_confirm", "")),
        )
        spoken = self._cmd_engine.spoken_text_before_tag()
        if self._tts_music_independent:
            self._log(
                "info",
                f"[cmd] chat confirm without CMD tag -> parallel TTS "
                f"({cmd_id}) text={spoken!r}",
            )
            await self._execute_music_parallel_with_tts(music_action, cmd_id)
        else:
            self._log(
                "info",
                f"[cmd] chat confirm without CMD tag -> injected "
                f"({cmd_id}) text={spoken!r}",
            )
            await self._begin_deferred_music_from_chat_tag(
                music_action, cmd_id, entry, matched
            )
        self._pending_asr_music = None

    async def _pending_chat_complete_worker(self) -> None:
        try:
            await asyncio.sleep(0.28)
            await self._finish_pending_asr_chat_confirm()
        except asyncio.CancelledError:
            return

    def _use_injected_confirm(self, cmd_id: str) -> bool:
        if self._tts_music_independent:
            return False
        mode = str(self._cfg.raw.get("confirm_tts_mode", "injected")).strip().lower()
        if mode == "model":
            return False
        return cmd_id in _MUSIC_DEFER_CMD_IDS

    def _confirm_prefer_model_when_natural(self) -> bool:
        raw = self._cfg.raw.get("confirm_tts_prefer_model_when_natural", True)
        if isinstance(raw, bool):
            return raw
        return str(raw).strip().lower() not in ("0", "false", "no", "off")

    def _should_use_model_tts_instead_of_inject(self, cmd_id: str) -> bool:
        """标签前已有自然确认语时沿用模型 TTS，避免打断+注入无音频。"""
        if not self._use_injected_confirm(cmd_id):
            return False
        if not self._confirm_prefer_model_when_natural():
            return False
        if cmd_id not in _MODEL_DEFER_PREFER_SPOKEN_CMD_IDS:
            return False
        spoken = self._cmd_engine.spoken_text_before_tag().strip()
        if len(spoken) < 8 or len(spoken) > 56:
            return False
        tail = spoken.rstrip()
        has_end = tail.endswith(("。", "！", "？", ".", "!", "?"))
        if not has_end and len(spoken) < 12:
            return False
        templates = (
            _MUSIC_PLAY_INJECTED_CONFIRM,
            "好的，这就为您播放。",
            "好的，音乐已恢复",
            "好的，音乐已恢复。",
        )
        if spoken in templates:
            return False
        if CMD_TAG_PATTERN.search(spoken) or "[CMD" in spoken.upper():
            return False
        return True

    def _format_cmd_tts_confirm(self, entry: dict, matched: CmdResult) -> str:
        tpl = str(entry.get("tts_confirm", "") or matched.reply or "").strip()
        if not tpl:
            return ""
        return self._format_tts_confirm(
            tpl,
            current_file=self._last_music_current_file,
            arg=matched.arg or "",
        )

    def _set_pending_asr_defer(
        self,
        music_action: str,
        cmd_id: str,
        entry: dict,
        matched: CmdResult,
    ) -> None:
        if self._defer_in_progress and self._deferred_music is not None:
            cur_action, cur_cmd = self._deferred_music
            if cur_cmd == cmd_id and cur_action == music_action:
                self._log(
                    "debug",
                    f"[cmd] ASR {cmd_id} ignored: same defer already in flight",
                )
                return
            self._cancel_deferred_music()
        fallback = self._format_cmd_tts_confirm(entry, matched)
        self._pending_asr_music = (music_action, cmd_id, fallback)
        self._pending_asr_music_utterance = ""
        wait_s = (
            0.75 if cmd_id in _MUSIC_QUICK_CMD_IDS else _ASR_MUSIC_TAG_WAIT_S
        )
        self._log(
            "info",
            f"[cmd] ASR defer pending chat tag: {music_action} "
            f"(timeout={wait_s:.1f}s)",
        )
        self._cancel_asr_music_tag_wait_only()
        self._asr_music_tag_wait_task = asyncio.create_task(
            self._asr_music_tag_wait_worker(wait_s),
            name="asr-music-tag-wait",
        )

    async def _asr_music_tag_wait_worker(self, wait_s: float) -> None:
        try:
            await asyncio.sleep(wait_s)
            pending = self._pending_asr_music
            if pending is None or self._shutting_down:
                return
            _music_action, cmd_id, _fallback = pending
            if self._defer_in_progress:
                if cmd_id not in _MUSIC_QUICK_CMD_IDS:
                    self._pending_asr_music = None
                    return
                self._cancel_deferred_music()
            buf = self._cmd_engine.intent_buffer_text()
            if self._spoken_implies_music_stop(buf) or self._spoken_implies_music_stop(
                self._pending_asr_music_utterance
            ):
                self._log(
                    "info",
                    "[cmd] ASR tag timeout: cancel play, chat implies stop",
                )
                self._pending_asr_music = None
                return
            music_action, cmd_id, fallback = pending
            self._pending_asr_music = None
            if self._tts_music_independent:
                self._log(
                    "info",
                    f"[cmd] ASR tag timeout: parallel music+TTS cmd={cmd_id}",
                )
                await self._execute_music_parallel_with_tts(music_action, cmd_id)
                return
            if (
                cmd_id in _MUSIC_QUICK_CMD_IDS
                and not self._chat_spoken_confirm_ready()
            ):
                self._log(
                    "info",
                    f"[cmd] ASR tag timeout: direct {cmd_id} "
                    f"(no short LLM confirm, skip injected)",
                )
                await self._execute_quick_music_and_interrupt(music_action, cmd_id)
                return
            if self._chat_spoken_confirm_ready():
                text = self._cmd_engine.spoken_text_before_tag() or fallback
            else:
                text = fallback or "好的，这就为您播放。"
            self._log(
                "info",
                f"[cmd] ASR music tag wait timeout, ChatTTSText fallback "
                f"cmd={cmd_id}",
            )
            await self._start_injected_music_defer(
                music_action, cmd_id, text
            )
        except asyncio.CancelledError:
            return

    async def _start_injected_music_defer(
        self,
        music_action: str,
        cmd_id: str,
        confirm_text: str,
        *,
        fallback_phrase: str = "",
    ) -> None:
        """先进入 defer 再等注入 TTS（保证 TTSSentenceStart 时 defer 已激活）。"""
        key = (music_action, cmd_id)
        if self._same_injected_defer_active(music_action, cmd_id):
            self._log(
                "debug",
                f"[cmd] skip duplicate injected defer {cmd_id} "
                f"(already confirming)",
            )
            return
        if self._defer_in_progress:
            self._cancel_deferred_music()
        self._cancel_pending_asr_defer()
        confirm_text = self._injected_confirm_phrase(
            confirm_text,
            fallback_phrase or confirm_text,
            cmd_id=cmd_id,
        )
        self._injected_defer_key = key
        self._injected_confirm_text = confirm_text
        self._injected_confirm_target_ms = self._estimate_injected_confirm_play_ms(
            confirm_text, cmd_id=cmd_id
        )
        self._injected_playback_started = False
        self._injected_chat_tts_sent_mono = 0.0
        self._injected_tts_resend = False
        self._try_arm_injected_music_confirm(cmd_id)
        self._injected_defer_started_mono = time.monotonic()
        # 先 arm defer，发完 ChatTTSText 再启动 worker（避免 worker 空等 6s）
        self._schedule_deferred_music(
            music_action,
            cmd_id,
            confirm_source="injected",
            start_worker=False,
        )
        await self._isolate_cmd_tag_tts(confirm_text)
        self._start_deferred_music_worker()

    @staticmethod
    def _shorten_injected_confirm(text: str, *, max_chars: int = 16) -> str:
        s = str(text or "").strip()
        if len(s) <= max_chars:
            return s
        for cut in ("。", "！", "？", ".", "!", "?"):
            idx = s.find(cut)
            if 0 < idx <= max_chars:
                return s[: idx + 1]
        return s[:max_chars] + "。"

    @staticmethod
    def _estimate_injected_confirm_play_ms(text: str, *, cmd_id: str = "") -> int:
        """按字数估算确认语应播满的毫秒数（中文约 160~180ms/字）。"""
        n = len(str(text or "").strip())
        if n <= 0:
            base = 900
        else:
            base = max(900, min(5200, int(n * 175) + 350))
        if cmd_id in _MUSIC_QUICK_CMD_IDS or cmd_id == "music_play":
            return min(base, 1400)
        return base

    def _injected_confirm_phrase(
        self, spoken: str, fallback: str = "", *, cmd_id: str = ""
    ) -> str:
        """优先短句：模型确认过长时改用模板，避免播不完就被提前放行。"""
        if cmd_id == "music_play":
            return _MUSIC_PLAY_INJECTED_CONFIRM
        s = str(spoken or "").strip()
        fb = str(fallback or "").strip()
        if len(s) <= 16:
            return s
        short = self._shorten_injected_confirm(fb or "好的，这就为您播放。")
        if short:
            return short
        return self._shorten_injected_confirm(s)

    async def _execute_music_parallel_with_tts(
        self, music_action: str, cmd_id: str
    ) -> None:
        """豆包 TTS 与本地 MP3 并行：仅门控 [CMD] 尾音，不 duck/不等待/不 isolate。"""
        self._cancel_pending_asr_defer()
        if self._defer_in_progress:
            self._cancel_deferred_music()
        self._arm_parallel_music_confirm_guard()
        if self._cmd_engine.intent_tag_tts_cut_needed() and not self._tag_tts_gate_active:
            self._arm_tag_tts_gate(period_cap=True)
        self._log(
            "info",
            f"[cmd] music parallel with doubao TTS: {music_action} ({cmd_id})",
        )
        await self._execute_music_control(music_action, cmd_id)

    async def _begin_deferred_music_from_chat_tag(
        self,
        music_action: str,
        cmd_id: str,
        entry: dict,
        matched: CmdResult,
    ) -> None:
        self._cancel_pending_asr_defer()
        if self._defer_in_progress:
            self._cancel_deferred_music()
        if self._tts_music_independent:
            await self._execute_music_parallel_with_tts(music_action, cmd_id)
            return
        if self._should_use_model_tts_instead_of_inject(cmd_id):
            await self._begin_deferred_music_model_confirm(
                music_action, cmd_id
            )
            return
        spoken = self._cmd_engine.spoken_text_before_tag()
        fallback = self._format_cmd_tts_confirm(entry, matched)
        text = spoken or fallback
        if not text.strip():
            text = "好的，这就为您播放。"
        await self._start_injected_music_defer(
            music_action, cmd_id, text, fallback_phrase=fallback
        )

    async def _begin_deferred_music_model_confirm(
        self, music_action: str, cmd_id: str
    ) -> None:
        """标签前口语由模型 TTS 播出，字节门控丢弃 [CMD] 尾音。"""
        await self._sync_cmd_tag_tts_gate_async()
        spoken = self._cmd_engine.spoken_text_before_tag()
        n = max(8, len(spoken.strip()))
        preview = spoken if len(spoken) <= 48 else spoken[:45] + "…"
        self._log(
            "info",
            f"[cmd] defer model TTS before tag ({cmd_id}): {preview!r}",
        )
        self._schedule_deferred_music(
            music_action, cmd_id, confirm_source="model"
        )
        self._defer_confirm_target_ms = max(
            self._defer_confirm_target_ms,
            max(700, min(4500, int(n * 165) + 220)),
        )

    async def _wait_tts_reply_settled(self, timeout_s: float = 0.55) -> None:
        """ClientInterrupt 后等待本轮模型 TTS 收尾，再发 ChatTTSText。"""
        deadline = time.monotonic() + timeout_s
        saw_active = False
        while time.monotonic() < deadline and not self._shutting_down:
            if self._tts_playback_active or not self._is_tts_playback_idle():
                saw_active = True
            elif saw_active:
                await asyncio.sleep(0.06)
                return
            await asyncio.sleep(0.04)

    async def _isolate_cmd_tag_tts(self, spoken_text: str) -> None:
        """掐断本句模型 TTS，仅用 ChatTTSText 播标签前口语（不含 [CMD]）。"""
        spoken_text = str(spoken_text or "").strip()
        if not spoken_text:
            return
        played_ms = self._tts_played_ms()
        reply_id = self._current_reply_id
        self._clear_tag_tts_gate()
        period_ms = self._cmd_engine.period_cut_target_ms()
        flush_fn = getattr(self._node, "flush_tts_playback", None)
        if callable(flush_fn) and played_ms >= max(280, int(period_ms * 0.12)):
            flush_fn()
        reset_dialog = getattr(self._node, "_reset_dialog_timing", None)
        if callable(reset_dialog):
            reset_dialog()
        self._drop_incoming_tts = True
        self._awaiting_injected_tts = True
        self._tag_tts_tail_suppressed = True
        interrupted = bool(
            self._client
            and self._client.is_in_session
            and (played_ms > 120 or self._tts_playback_active)
        )
        if interrupted:
            try:
                await self._client.send_client_interrupt()
            except Exception as e:
                if self._debug_interrupt:
                    self._log(
                        "warn",
                        f"[cmd] isolate ClientInterrupt failed: {e}",
                    )
            if self._truncate_enabled and reply_id and played_ms > 0:
                try:
                    await self._client.send_conversation_truncate(
                        reply_id, played_ms
                    )
                except Exception as e:
                    if self._debug_interrupt:
                        self._log(
                            "warn",
                            f"[cmd] isolate truncate failed: {e}",
                        )
            await self._wait_tts_reply_settled()
        reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_stats) and interrupted:
            reset_stats()
        self._defer_tts_ended = False
        await self._ensure_session_started()
        if self._client and self._client.is_in_session:
            await asyncio.sleep(0.14 if interrupted else 0.06)
        await self._send_chat_tts_text(spoken_text)
        self._injected_chat_tts_sent_mono = time.monotonic()
        self._drop_incoming_tts = False
        # 保持 True 直至 ChatTTSText 的 TTSSentenceStart（走 awaiting 分支，勿 reset_stats）
        # 保持 suppressed 至 defer worker 结束，避免晚到 [CMD] 再开门控裁掉 ChatTTSText
        preview = spoken_text if len(spoken_text) <= 48 else spoken_text[:45] + "…"
        self._log("info", f"[cmd] isolate cmd tag tts: {preview!r}")

    @staticmethod
    def _sanitize_song_name(name: str) -> str:
        s = str(name or "").strip()
        for ch in "《》「」『』\"'":
            s = s.replace(ch, "")
        for prefix in ("一首", "一个", "首"):
            if s.startswith(prefix):
                s = s[len(prefix) :].strip()
        return s.strip()

    def _cancel_song_tag_wait(self) -> None:
        if self._song_tag_wait_task is not None and not self._song_tag_wait_task.done():
            self._song_tag_wait_task.cancel()
        self._song_tag_wait_task = None

    def _start_song_tag_wait(self, song: str) -> None:
        self._cancel_song_tag_wait()
        self._song_tag_wait_task = asyncio.create_task(
            self._song_tag_wait_worker(song),
            name="song-tag-wait",
        )

    async def _song_tag_wait_worker(self, song: str) -> None:
        try:
            await asyncio.sleep(_SONG_TAG_WAIT_S)
            if self._shutting_down or self._defer_in_progress:
                return
            pending = self._sanitize_song_name(self._pending_song or song)
            if not pending:
                return
            self._log(
                "info",
                f"[cmd] song tag wait timeout, defer with ASR name: {pending!r}",
            )
            self._schedule_deferred_music(f"play_named:{pending}", "music_play_song")
        except asyncio.CancelledError:
            return

    def _start_deferred_music_worker(self) -> None:
        if self._deferred_music is None or not self._defer_in_progress:
            return
        if self._deferred_music_task is not None and not self._deferred_music_task.done():
            self._deferred_music_task.cancel()
        self._deferred_music_task = asyncio.create_task(
            self._deferred_music_worker(),
            name="deferred-music",
        )

    def _schedule_deferred_music(
        self,
        music_action: str,
        cmd_id: str,
        *,
        confirm_source: str = "model",
        start_worker: bool = True,
    ) -> None:
        immediate_action = (
            music_action in _MUSIC_ACTIONS_IMMEDIATE
            and not music_action.startswith("play_named:")
        )
        if (
            immediate_action
            and self._defer_in_progress
            and self._defer_music_executed
            and self._deferred_music == (music_action, cmd_id)
            and (time.time() - self._last_cmd_exec_ts) < 2.5
        ):
            self._log(
                "info",
                f"[cmd] skip duplicate defer {cmd_id}: "
                f"{music_action} already executed via ASR",
            )
            return
        if self._tts_music_independent and (
            music_action in _MUSIC_ACTIONS_DEFER_FOR_TTS
            or music_action.startswith("play_named:")
        ):
            self._submit(
                self._execute_music_parallel_with_tts(music_action, cmd_id)
            )
            return
        if self._defer_in_progress:
            if immediate_action and self._injected_music_defer_active():
                self._cancel_deferred_music()
            elif not (
                confirm_source == "injected"
                and self._same_injected_defer_active(music_action, cmd_id)
            ):
                self._cancel_deferred_music()
        self._cancel_song_tag_wait()
        reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_stats) and confirm_source != "injected":
            reset_stats()
        self._defer_saw_tts = False
        self._defer_music_executed = False
        self._deferred_music = (music_action, cmd_id)
        self._defer_in_progress = True
        self._defer_confirm_source = confirm_source
        self._defer_tts_ended = False
        self._defer_last_tts_audio_ts = 0.0
        if confirm_source != "injected":
            self._drop_incoming_tts = False
        if confirm_source != "injected" and not self._cmd_engine.intent_tag_tts_cut_needed():
            self._tag_tts_tail_suppressed = False
            self._tag_tts_gate_active = False
            self._tag_tts_received_bytes = 0
            self._tag_confirm_bytes_budget = 0
        period_ms = self._cmd_engine.period_cut_target_ms()
        if immediate_action:
            self._defer_confirm_target_ms = max(
                1200, self._cmd_engine.confirm_playback_target_ms()
            )
            self._tag_tts_tail_suppressed = True
        elif confirm_source == "injected":
            self._defer_confirm_target_ms = max(400, period_ms)
        else:
            self._defer_confirm_target_ms = max(600, period_ms + 200)
        if immediate_action:
            mode = "execute now + drain TTS"
        elif confirm_source == "injected":
            mode = "wait injected ChatTTSText, then execute"
        else:
            mode = "wait doubao TTS end, then execute"
        self._log("info", f"[cmd] defer music {music_action}: {mode}")
        if cmd_id == "music_pause" and immediate_action:
            self._log(
                "info",
                f"[cmd] music_pause ASR/immediate: user_pause now (source={confirm_source})",
            )
        if confirm_source == "injected":
            self._tag_tts_tail_suppressed = True
            guard_s = 10.0
        else:
            guard_s = self._defer_confirm_target_ms / 1000.0 + 1.5
        self._defer_tts_guard_until = time.monotonic() + guard_s
        if immediate_action:
            self._submit(
                self._execute_deferred_music_immediate(music_action, cmd_id)
            )
        if start_worker:
            self._start_deferred_music_worker()

    async def _execute_deferred_music_immediate(
        self, music_action: str, cmd_id: str
    ) -> None:
        """仅 stop/user_pause：立即执行，worker 只等确认 TTS 收尾。"""
        if self._defer_music_executed or self._shutting_down:
            return
        self._defer_music_executed = True
        await self._execute_music_control(music_action, cmd_id)

    async def _deferred_music_worker(self) -> None:
        try:
            while self._deferred_music is not None and not self._shutting_down:
                music_action, cmd_id = self._deferred_music
                immediate = (
                    music_action in _MUSIC_ACTIONS_IMMEDIATE
                    and not music_action.startswith("play_named:")
                )
                try:
                    if immediate:
                        await self._wait_defer_after_immediate_action()
                    else:
                        await self._wait_doubao_tts_finish_for_defer()
                    if self._shutting_down:
                        return
                    if not self._defer_music_executed:
                        await self._execute_music_control(music_action, cmd_id)
                finally:
                    self._deferred_music = None
        except asyncio.CancelledError:
            return
        finally:
            self._defer_in_progress = False
            self._defer_tts_ended = False
            self._defer_tts_guard_until = 0.0
            self._defer_confirm_source = "model"
            self._awaiting_injected_tts = False
            self._drop_incoming_tts = False
            self._tag_tts_tail_suppressed = False
            self._tag_gate_prepare_logged = False
            self._injected_defer_key = None
            self._injected_defer_started_mono = 0.0
            self._injected_confirm_text = ""
            self._injected_confirm_target_ms = 0
            self._injected_playback_started = False
            self._injected_chat_tts_sent_mono = 0.0
            self._injected_armed = False
            self._injected_tts_resend = False
            self._clear_tag_tts_gate()

    def _same_injected_defer_active(
        self, music_action: str, cmd_id: str
    ) -> bool:
        if (
            not self._defer_in_progress
            or self._defer_confirm_source != "injected"
            or self._injected_defer_key is None
        ):
            return False
        return self._injected_defer_key == (music_action, cmd_id)

    def _injected_music_defer_active(self) -> bool:
        return (
            self._defer_in_progress
            and self._defer_confirm_source == "injected"
        )

    def _skip_tag_gate_for_injected_confirm(self) -> bool:
        return (
            self._injected_armed
            or self._awaiting_injected_tts
            or self._injected_music_defer_active()
        )

    def _try_arm_injected_music_confirm(self, cmd_id: str) -> None:
        cid = str(cmd_id or "").strip()
        if cid not in _MUSIC_DEFER_CMD_IDS or not self._use_injected_confirm(cid):
            return
        self._injected_armed = True
        self._clear_tag_tts_gate()

    def _tts_played_ms(self) -> int:
        played_fn = getattr(self._node, "tts_playback_played_ms", None)
        return int(played_fn() or 0) if callable(played_fn) else 0

    def _injected_defer_cmd_id(self) -> str:
        if self._injected_defer_key is None:
            return ""
        return str(self._injected_defer_key[1] or "")

    def _injected_tail_quiet(self) -> bool:
        last = self._defer_last_tts_audio_ts
        if last > 0.0 and time.monotonic() - last < 0.22:
            return False
        return self._is_tts_playback_idle()

    def _injected_heard_enough_ms(self, target_ms: int, *, quick: bool) -> int:
        n = len(self._injected_confirm_text or "")
        return max(
            350,
            int(n * 72),
            int(target_ms * (0.32 if quick else 0.4)),
        )

    async def _wait_defer_after_immediate_action(self) -> None:
        """stop/pause 已执行：短等确认 TTS 收尾，禁止 15s 超时 ClientInterrupt。"""
        start = time.monotonic()
        max_wait_s = 4.0
        min_confirm_ms = self._defer_confirm_target_ms or 1200
        soft_ms = max(900, min(1400, min_confirm_ms))
        self._log(
            "info",
            f"[cmd] wait stop/pause confirm tail: soft_ms={soft_ms}",
        )
        while time.monotonic() - start < max_wait_s and not self._shutting_down:
            played_ms = self._tts_played_ms()
            idle = self._is_tts_playback_idle()
            if self._defer_tts_ended and idle and played_ms >= soft_ms:
                self._log(
                    "info",
                    f"[cmd] immediate action confirm tail done played_ms={played_ms}",
                )
                return
            if (
                self._defer_tts_ended
                and idle
                and played_ms >= max(400, int(soft_ms * 0.45))
            ):
                return
            await asyncio.sleep(0.05)
        self._log(
            "info",
            "[cmd] immediate action confirm tail wait ended (no interrupt)",
        )

    async def _wait_injected_confirm_tts_for_defer(self) -> None:
        """等 ChatTTSText 确认语按字数播满且 TTSEnded + 队列排空后再播歌。"""
        start = time.monotonic()
        self._defer_tts_ended = False
        target_ms = self._injected_confirm_target_ms or 1200
        period_ms = self._cmd_engine.period_cut_target_ms()
        cmd_id = self._injected_defer_cmd_id()
        quick = cmd_id in _MUSIC_QUICK_CMD_IDS
        play_like = cmd_id in ("music_play", "music_resume")
        wallclock_ratio = 0.68 if play_like else 0.92
        wallclock_min_elapsed = (
            0.45 if play_like else (0.55 if quick else 0.75)
        )
        min_played_ms = max(
            500 if quick else 700,
            min(int(target_ms * 0.88), int(target_ms * 0.72) + period_ms // 4),
        )
        max_wait_s = (
            1.35
            if play_like
            else (1.8 if quick else min(3.5, target_ms / 1000.0 + 1.15))
        )
        heard_enough_ms = self._injected_heard_enough_ms(target_ms, quick=quick)
        first_audio_mono = 0.0
        self._log(
            "info",
            f"[cmd] wait injected confirm TTS: target_ms={target_ms} "
            f"heard_ok>={heard_enough_ms} max_wait={max_wait_s:.1f}s "
            f"text={self._injected_confirm_text!r}",
        )
        while time.monotonic() - start < max_wait_s and not self._shutting_down:
            played_ms = self._tts_played_ms()
            idle = self._is_tts_playback_idle()
            elapsed = time.monotonic() - start
            if played_ms >= 80:
                self._injected_playback_started = True
                if self._injected_tts_ended_pending:
                    # TTSEnded 到达时播放尚未满 80ms，现在补齐标记
                    self._injected_tts_ended_pending = False
                    self._defer_tts_ended = True
            if self._defer_last_tts_audio_ts > 0.0 and first_audio_mono <= 0.0:
                first_audio_mono = self._defer_last_tts_audio_ts
            if (
                self._defer_saw_tts
                and self._injected_tail_quiet()
                and played_ms >= heard_enough_ms
                and elapsed >= (0.55 if quick else 0.7)
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected, heard_ok) "
                    f"played_ms={played_ms}/{target_ms} "
                    f"tts_ended={self._defer_tts_ended}",
                )
                await self._wait_tts_playback_idle(timeout_s=0.6)
                self._tts_playback_active = False
                return
            sent_mono = self._injected_chat_tts_sent_mono
            if (
                sent_mono > 0.0
                and not self._defer_saw_tts
                and not self._injected_tts_resend
                and time.monotonic() - sent_mono >= 1.15
            ):
                self._injected_tts_resend = True
                self._awaiting_injected_tts = True
                self._log(
                    "warn",
                    "[cmd] injected ChatTTSText no audio; resend once",
                )
                await self._send_chat_tts_text(self._injected_confirm_text)
            if sent_mono > 0.0 and not self._defer_saw_tts:
                no_audio_s = time.monotonic() - sent_mono
                fail_s = (
                    _INJECTED_CONFIRM_FAIL_AFTER_RESEND_S
                    if self._injected_tts_resend
                    else _INJECTED_CONFIRM_FAIL_S
                )
                if no_audio_s >= fail_s:
                    self._log(
                        "warn",
                        "[cmd] injected ChatTTSText still no audio; "
                        "play music without confirm TTS",
                    )
                    return
            if not self._defer_saw_tts:
                await asyncio.sleep(0.04)
                continue
            if (
                self._defer_tts_ended
                and idle
                and played_ms >= min_played_ms
            ):
                last_chunk = self._defer_last_tts_audio_ts
                if last_chunk > 0.0 and time.monotonic() - last_chunk < 0.2:
                    await asyncio.sleep(0.04)
                    continue
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected) "
                    f"played_ms={played_ms}/{target_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=1.5)
                self._tts_playback_active = False
                return
            tail_idle = (
                idle
                or self._defer_last_tts_audio_ts <= 0.0
                or time.monotonic() - self._defer_last_tts_audio_ts >= 0.28
            )
            min_ratio_ms = max(500, int(target_ms * 0.72))
            if (
                self._defer_tts_ended
                and tail_idle
                and played_ms >= min_ratio_ms
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected, ratio_ok) "
                    f"played_ms={played_ms}/{target_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=1.0)
                self._tts_playback_active = False
                return
            # ChatTTSText 已结束但 played 统计偏低（门控/打断后常见）：按句号预算放行
            if (
                self._defer_tts_ended
                and tail_idle
                and played_ms >= max(500, period_ms - 120)
                and played_ms >= int(len(self._injected_confirm_text) * 120)
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected, period_ok) "
                    f"played_ms={played_ms}/{target_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=1.0)
                self._tts_playback_active = False
                return
            if (
                first_audio_mono > 0.0
                and time.monotonic() - first_audio_mono > 3.5
                and self._defer_tts_ended
                and idle
                and played_ms >= max(400, int(target_ms * 0.35))
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected, stall_ok) "
                    f"played_ms={played_ms}/{target_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=1.0)
                self._tts_playback_active = False
                return
            sent_mono = self._injected_chat_tts_sent_mono
            est_s = max(1.2, target_ms / 1000.0 + 0.35)
            if (
                self._injected_playback_started
                and sent_mono > 0.0
                and time.monotonic() - sent_mono >= est_s * 0.85
                and self._injected_tail_quiet()
                and played_ms >= max(320, int(len(self._injected_confirm_text) * 90))
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected, playback_ok) "
                    f"played_ms={played_ms}/{target_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=0.6)
                self._tts_playback_active = False
                return
            # 纯墙钟兜底：played_ms 统计偏低（ALSA underrun / reset 竞争）时仍能及时放行
            if (
                self._injected_playback_started
                and sent_mono > 0.0
                and time.monotonic() - sent_mono
                >= max(0.85, target_ms / 1000.0 * wallclock_ratio)
                and elapsed >= wallclock_min_elapsed
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (injected, wallclock_ok) "
                    f"played_ms={played_ms}/{target_ms} ratio={wallclock_ratio}",
                )
                await self._wait_tts_playback_idle(timeout_s=0.5)
                self._tts_playback_active = False
                return
            await asyncio.sleep(0.04)
        if not self._shutting_down:
            played_ms = self._tts_played_ms()
            self._log(
                "warn",
                f"[cmd] injected confirm wait timeout "
                f"({played_ms}/{target_ms}ms "
                f"playback_started={self._injected_playback_started} "
                f"saw_tts={self._defer_saw_tts} tts_ended={self._defer_tts_ended}), "
                f"proceed to music",
            )

    async def _wait_doubao_tts_finish_for_defer(self) -> None:
        """等确认语实际播够时长且队列排空后再执行本地音乐（标签封住后仍要播完）。"""
        if self._defer_confirm_source == "injected":
            await self._wait_injected_confirm_tts_for_defer()
            return
        start = time.monotonic()
        self._defer_tts_ended = False
        self._defer_last_tts_audio_ts = 0.0
        period_ms = self._cmd_engine.period_cut_target_ms()
        min_confirm_ms = max(period_ms, self._defer_confirm_target_ms or period_ms)
        self._log(
            "info",
            f"[cmd] wait confirm TTS: period_ms={period_ms} "
            f"target_ms={min_confirm_ms}",
        )

        while (
            time.monotonic() - start < _DEFER_MUSIC_CONFIRM_MAX_S
            and not self._shutting_down
        ):
            if not self._defer_saw_tts:
                await asyncio.sleep(0.05)
                continue

            played_ms = self._tts_played_ms()
            idle = self._is_tts_playback_idle()
            done_ms = max(period_ms, 400)
            if (
                self._tag_tts_tail_suppressed
                and played_ms >= done_ms
                and not self._drop_incoming_tts
            ):
                self._drop_incoming_tts = True
            if (
                self._tag_tts_tail_suppressed
                and played_ms >= done_ms
                and idle
                and self._defer_tts_ended
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (period+tag gate) "
                    f"played_ms={played_ms}/{done_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=2.0)
                self._tts_playback_active = False
                return
            if (
                played_ms >= done_ms
                and idle
                and self._defer_tts_ended
            ):
                last_chunk = self._defer_last_tts_audio_ts
                if last_chunk > 0.0:
                    silence_s = time.monotonic() - last_chunk
                    if silence_s < 0.25:
                        await asyncio.sleep(0.05)
                        continue
                self._log(
                    "info",
                    f"[cmd] doubao confirm done played_ms={played_ms}/"
                    f"{min_confirm_ms}, playback drained, tts_ended=yes",
                )
                await self._wait_tts_playback_idle(timeout_s=2.0)
                self._tts_playback_active = False
                return
            elapsed = time.monotonic() - start
            min_fallback_ms = int(min_confirm_ms * 0.82)
            tag_done_ms = max(period_ms, int(period_ms * 0.88))
            if (
                self._tag_tts_tail_suppressed
                and self._defer_tts_ended
                and idle
                and played_ms >= tag_done_ms
                and elapsed >= 1.5
            ):
                self._log(
                    "info",
                    f"[cmd] doubao confirm done (tag gate fallback) "
                    f"played_ms={played_ms}/{tag_done_ms}",
                )
                await self._wait_tts_playback_idle(timeout_s=2.0)
                self._tts_playback_active = False
                return
            if (
                elapsed >= 12.0
                and idle
                and self._defer_tts_ended
                and self._defer_saw_tts
                and played_ms >= min_fallback_ms
            ):
                self._drop_incoming_tts = True
                self._log(
                    "warn",
                    f"[cmd] confirm fallback after TTSEnded "
                    f"({played_ms}/{min_confirm_ms}ms)",
                )
                await self._wait_tts_playback_idle(timeout_s=2.0)
                self._tts_playback_active = False
                return
            await asyncio.sleep(0.05)

        if self._shutting_down:
            return
        if self._defer_music_executed:
            self._log(
                "info",
                "[cmd] defer wait ended (music action already executed)",
            )
            return
        self._log(
            "warn",
            "[cmd] doubao TTS wait timeout, force stop before local music",
        )
        await self._stop_doubao_tts_output()

    async def _stop_doubao_tts_output(self) -> None:
        await self._interrupt_tts_playback(
            reason="defer_timeout",
            suppress_followup=False,
        )
        await self._wait_tts_playback_idle(timeout_s=2.0)

    def _reset_turn_tts_state(self) -> None:
        """新一句用户话：清零播放统计，避免门控误用上一轮 played_ms。"""
        self._parallel_confirm_guard_until = 0.0
        self._parallel_confirm_period_ms = 0
        self._tag_cloud_cut_sent = False
        reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_stats):
            reset_stats()

    def _clear_tag_tts_gate(self) -> None:
        self._tag_tts_gate_active = False
        self._tag_tts_received_bytes = 0
        self._tag_confirm_bytes_budget = 0
        self._tag_gate_prepare_logged = False
        self._parallel_confirm_guard_until = 0.0
        self._parallel_confirm_period_ms = 0
        self._cmd_engine.reset_intent_buffer(clear_tag_gate=True)

    def _should_drop_cmd_tag_tts_chunk(self, payload: bytes) -> bool:
        """Chat 已出现 [CMD 且确认语已播够：丢弃标签对应 TTS（防朗读 CMD）。"""
        if not payload or self._skip_tag_gate_for_injected_confirm():
            return False
        buf = self._cmd_engine.intent_buffer_text()
        has_full = bool(CMD_TAG_PATTERN.search(buf))
        has_partial = bool(_CMD_TAG_PARTIAL_RE.search(buf))
        if not has_full and not has_partial:
            return False
        played_ms = self._tts_played_ms()
        period_ms = self._cmd_engine.period_cut_target_ms()
        if has_partial and not has_full:
            return played_ms >= period_ms
        if played_ms < self._tag_gate_min_block_ms():
            return False
        if played_ms < period_ms and not has_full:
            return False
        return True

    async def _halt_cmd_tag_tts_tail_async(self, *, force: bool = False) -> None:
        """Chat 已见 [ / [CMD:：立刻停云端标签合成，确认语播满后再丢尾音。"""
        buf = self._cmd_engine.intent_buffer_text()
        if not force and not (
            CMD_TAG_PATTERN.search(buf) or _CMD_TAG_PARTIAL_RE.search(buf)
        ):
            return
        if not self._tag_tts_gate_active:
            self._arm_tag_tts_gate(period_cap=True)
        await self._stop_tag_cloud_tts()
        played_ms = self._tts_played_ms()
        period_ms = self._cmd_engine.period_cut_target_ms()
        min_ms = max(period_ms, self._tag_gate_min_block_ms())
        if played_ms >= min_ms:
            self._trim_tag_tts_queue_to_budget()
            await self._mark_tag_tail_blocked(
                reason="halt_cmd_tag",
                played_ms=played_ms,
            )
        elif CMD_TAG_PATTERN.search(buf):
            self._drop_incoming_tts = True

    def _parallel_confirm_active(self) -> bool:
        return (
            self._tts_music_independent
            and time.monotonic() < self._parallel_confirm_guard_until
        )

    def _parallel_confirm_min_played_ms(self) -> int:
        period_ms = self._parallel_confirm_period_ms or self._cmd_engine.period_cut_target_ms()
        target_ms = self._cmd_engine.confirm_playback_target_ms()
        return max(450, min(period_ms, int(target_ms * 0.55)))

    def _arm_parallel_music_confirm_guard(self) -> None:
        """并行播歌：保护模型确认语播满，并抑制 ASRInfo 误打断。"""
        period_ms = self._cmd_engine.period_cut_target_ms()
        target_ms = max(
            period_ms,
            self._cmd_engine.confirm_playback_target_ms(),
        )
        self._parallel_confirm_period_ms = period_ms
        guard_s = max(target_ms, 900) / 1000.0 + 1.35
        until = time.monotonic() + guard_s
        self._parallel_confirm_guard_until = until
        self._defer_tts_guard_until = max(self._defer_tts_guard_until, until)

    def _should_defer_tag_cloud_cut(self) -> bool:
        if not self._cmd_engine.intent_tag_tts_cut_needed():
            return False
        played_ms = self._tts_played_ms()
        if self._parallel_confirm_active():
            return played_ms < self._parallel_confirm_min_played_ms()
        if self._defer_in_progress and self._deferred_music is not None:
            defer_action, _ = self._deferred_music
            if defer_action in _MUSIC_ACTIONS_IMMEDIATE:
                target_ms = max(
                    900,
                    self._defer_confirm_target_ms
                    or self._cmd_engine.confirm_playback_target_ms(),
                )
                return played_ms < int(target_ms * 0.55)
        return played_ms < 80

    def _tag_gate_min_block_ms(self) -> int:
        period_ms = self._cmd_engine.period_cut_target_ms()
        if self._parallel_confirm_active():
            return self._parallel_confirm_min_played_ms()
        if self._defer_in_progress and self._deferred_music is not None:
            defer_action, _ = self._deferred_music
            if defer_action in _MUSIC_ACTIONS_IMMEDIATE:
                target_ms = max(
                    900,
                    self._defer_confirm_target_ms
                    or self._cmd_engine.confirm_playback_target_ms(),
                )
                return int(target_ms * 0.55)
        return period_ms

    def _should_skip_duplicate_chat_music(self, matched: CmdResult) -> bool:
        if matched.cmd_id not in _MUSIC_DEFER_CMD_IDS:
            return False
        entry = self._cmd_engine.get_command(matched.cmd_id) or {}
        music_action = str(entry.get("music_action", "")).strip()
        if not music_action:
            return False
        now = time.time()
        if (
            music_action in _MUSIC_ACTIONS_IMMEDIATE
            and self._defer_music_executed
            and matched.cmd_id == self._last_cmd_id
            and (now - self._last_cmd_exec_ts) < 2.5
        ):
            self._log(
                "info",
                f"[cmd] skip chat {matched.cmd_id}: "
                f"{music_action} already executed this turn",
            )
            return True
        if (
            music_action in _MUSIC_ACTIONS_DEFER_FOR_TTS
            or music_action.startswith("play_named:")
        ):
            if (
                self._tts_music_independent
                and matched.cmd_id == self._last_cmd_id
                and (now - self._last_cmd_exec_ts) < 1.2
                and (
                    self._parallel_confirm_active()
                    or self._defer_music_executed
                )
            ):
                self._log(
                    "info",
                    f"[cmd] skip chat {matched.cmd_id}: "
                    f"parallel play already started",
                )
                return True
        if (
            music_action in _MUSIC_ACTIONS_IMMEDIATE
            and matched.cmd_id == self._last_cmd_id
            and (now - self._last_cmd_exec_ts) < 2.0
            and self._defer_music_executed
        ):
            self._log(
                "info",
                f"[cmd] skip chat {matched.cmd_id}: "
                f"{music_action} already executed via defer",
            )
            return True
        return False

    async def _wait_tts_confirm_floor_ms(
        self, min_ms: int, *, max_wait_s: float = 0.7
    ) -> None:
        """歌词早停等场景：尽量播满最短确认再 flush。"""
        if min_ms <= 0:
            return
        deadline = time.monotonic() + max_wait_s
        while time.monotonic() < deadline and not self._shutting_down:
            if self._tts_played_ms() >= min_ms:
                return
            if self._is_tts_playback_idle() and self._tts_played_ms() > 0:
                return
            await asyncio.sleep(0.04)

    def _pcm_bytes_for_ms(self, ms: int) -> int:
        denom = self._tts_sample_rate * self._tts_channels * 2
        return int(denom * max(0, ms) / 1000) if denom else 0

    def _tag_playback_totals(self) -> tuple[int, int, int]:
        """返回 (played_bytes, queued_bytes, played_ms)。"""
        played_ms_fn = getattr(self._node, "tts_playback_played_ms", None)
        played_ms = int(played_ms_fn() or 0) if callable(played_ms_fn) else 0
        played_b_fn = getattr(self._node, "tts_playback_played_bytes", None)
        if callable(played_b_fn):
            played_b = int(played_b_fn() or 0)
        else:
            played_b = self._pcm_bytes_for_ms(played_ms)
        queued_fn = getattr(self._node, "tts_playback_queued_bytes", None)
        queued_b = int(queued_fn() or 0) if callable(queued_fn) else 0
        return played_b, queued_b, played_ms

    def _trim_tag_tts_queue_to_budget(self) -> int:
        """裁掉播放队列里超出句号字节预算的 PCM（标签尾音常已先入队）。"""
        if not self._tag_tts_gate_active:
            return 0
        if (
            self._parallel_confirm_active()
            and self._tts_played_ms() < self._parallel_confirm_min_played_ms()
        ):
            return 0
        played_b, queued_b, _ = self._tag_playback_totals()
        budget = self._tag_confirm_bytes_budget
        if budget <= 0 or queued_b <= 0:
            return 0
        keep = max(0, budget - played_b)
        if queued_b <= keep:
            return 0
        trim_fn = getattr(self._node, "trim_tts_playback_queue", None)
        if not callable(trim_fn):
            return 0
        dropped = int(trim_fn(keep) or 0)
        if dropped > 0 and self._debug_interrupt:
            self._log(
                "debug",
                f"[doubao][cmd-tag] trim queued tag tail dropped_bytes={dropped} "
                f"keep={keep} budget={budget} played_b={played_b}",
            )
        return dropped

    async def _stop_tag_cloud_tts(self) -> None:
        """句号后通知云端停止合成，避免继续下发 [CMD:...] 对应音频。"""
        if self._tag_cloud_cut_sent or not self._client or not self._client.is_in_session:
            return
        try:
            await self._client.send_client_interrupt()
            self._tag_cloud_cut_sent = True
        except Exception as e:
            if self._debug_interrupt:
                self._log(
                    "warn",
                    f"[doubao][cmd-tag] ClientInterrupt for tag tail failed: {e}",
                )

    async def _sync_cmd_tag_tts_gate_async(self) -> None:
        """见到 。[ / [CMD: 时开启门控：先播完标签前确认语，再丢弃标签 TTS。"""
        if self._skip_tag_gate_for_injected_confirm():
            return
        if self._tag_tts_tail_suppressed:
            return
        if not self._cmd_engine.intent_tag_tts_cut_needed():
            return
        urgency = self._cmd_engine.tag_tts_cut_urgency()
        if urgency in ("boundary", "full"):
            await self._prepare_tag_tts_gate(reason=f"chat_{urgency}")
        elif not self._tag_tts_gate_active:
            self._arm_tag_tts_gate(period_cap=urgency in ("boundary", "full"))

    async def _prepare_tag_tts_gate(self, *, reason: str) -> None:
        """Chat 已见 。[ / [CMD:：按句号预算裁队列并停云端标签合成，避免朗读 [。"""
        if self._tag_tts_tail_suppressed:
            return
        # stop/pause / 并行播歌：须先播满短确认语，勿在 played_ms 过低时裁队列
        played_ms = self._tts_played_ms()
        if played_ms < self._tag_gate_min_block_ms():
            if not self._tag_tts_gate_active:
                self._arm_tag_tts_gate(period_cap=True)
            return
        if self._tag_gate_prepare_logged and self._tag_tts_gate_active:
            return
        if not self._tag_tts_gate_active:
            self._arm_tag_tts_gate(period_cap=True)
        if not self._should_defer_tag_cloud_cut():
            await self._stop_tag_cloud_tts()
        trimmed = await self._seal_tag_tts_gate()
        queue_trimmed = self._trim_tag_tts_queue_to_budget()
        trimmed += queue_trimmed
        # 不在此处 ClientInterrupt：会截断句号后确认语 PCM，导致 defer 永远等不满 target_ms
        played_b, queued_b, played_ms = self._tag_playback_totals()
        budget = self._tag_confirm_bytes_budget
        period_ms = self._cmd_engine.period_cut_target_ms()
        self._tag_gate_prepare_logged = True
        self._log(
            "info",
            f"[doubao][cmd-tag] gate at 。[/CMD ({reason}) "
            f"played_ms={played_ms}/{period_ms} "
            f"budget_bytes={budget} trimmed_bytes={trimmed} "
            f"queued_after={queued_b} "
            f"spoken_chars={self._cmd_engine.spoken_chars_before_tag} "
            f"(cut at period, tag tail dropped)",
        )

    def _arm_tag_tts_gate(self, *, period_cap: bool = False) -> None:
        """Chat 已见标签：按句号或确认语字节预算截断后续 TTS。"""
        if self._tag_tts_tail_suppressed:
            return
        played_b, queued_b, played_ms = self._tag_playback_totals()
        self._tag_tts_gate_active = True
        urgency = self._cmd_engine.tag_cut_urgency()
        use_period = period_cap or urgency in ("boundary", "full")
        if use_period:
            self._tag_confirm_bytes_budget = self._cmd_engine.tag_tail_pcm_cap(
                sample_rate=self._tts_sample_rate,
                channels=self._tts_channels,
            )
        else:
            budget = self._cmd_engine.estimated_confirm_pcm_bytes_before_tag(
                sample_rate=self._tts_sample_rate,
                channels=self._tts_channels,
            )
            self._tag_confirm_bytes_budget = max(self._tag_confirm_bytes_budget, budget)
        # 只计已播字节；勿把未播队列算进预算，否则确认语会被立刻掐断
        self._tag_tts_received_bytes = max(self._tag_tts_received_bytes, played_b)

    def _confirm_target_ms(self) -> int:
        if self._defer_confirm_target_ms > 0:
            return self._defer_confirm_target_ms
        return self._cmd_engine.confirm_playback_target_ms()

    async def _gate_tag_tts_chunk(self, payload: bytes) -> bool:
        """标签门控：句号字节预算内放行确认语，超出即丢 [CMD 尾音（不朗读 [）。"""
        if self._skip_tag_gate_for_injected_confirm():
            return False
        if not self._tag_tts_gate_active:
            return False
        period_ms = self._cmd_engine.period_cut_target_ms()
        played_ms = self._tts_played_ms()
        played_b, queued_b, _ = self._tag_playback_totals()
        budget = self._tag_confirm_bytes_budget
        parallel_protect = (
            self._parallel_confirm_active()
            and played_ms < period_ms
        )
        if parallel_protect:
            if payload:
                self._tts_playback_active = True
                if self._defer_in_progress:
                    self._defer_last_tts_audio_ts = time.monotonic()
                    self._defer_saw_tts = True
                self._on_tts_audio(payload)
                self._touch_activity()
            return True
        hard_cap = self._cmd_engine.tag_tail_pcm_cap(
            sample_rate=self._tts_sample_rate,
            channels=self._tts_channels,
        )
        if (
            hard_cap > 0
            and played_b >= hard_cap
            and played_ms >= period_ms
            and not parallel_protect
        ):
            self._dropped_tts_chunks += 1
            self._dropped_tts_bytes += len(payload)
            if not self._tag_tts_tail_suppressed:
                await self._mark_tag_tail_blocked(
                    reason="gate_byte_cap",
                    played_ms=played_ms,
                )
            return True
        if budget <= 0:
            await self._apply_tag_tts_suppress(
                reason="byte_gate_zero",
                played_ms=played_ms,
                send_client_interrupt=False,
            )
            self._dropped_tts_chunks += 1
            self._dropped_tts_bytes += len(payload)
            return True

        data = payload
        if parallel_protect:
            total_b = played_b
        else:
            total_b = played_b + queued_b

        if total_b > budget and not parallel_protect:
            self._trim_tag_tts_queue_to_budget()
            played_b, queued_b, played_ms = self._tag_playback_totals()
            total_b = played_b + queued_b

        if played_ms >= period_ms:
            self._dropped_tts_chunks += 1
            self._dropped_tts_bytes += len(data)
            if not self._tag_tts_tail_suppressed:
                await self._mark_tag_tail_blocked(
                    reason="gate_period_played",
                    played_ms=played_ms,
                )
            return True

        room = max(0, budget - total_b)
        if len(data) > room:
            tail_drop = len(data) - room
            self._dropped_tts_bytes += tail_drop
            self._dropped_tts_chunks += 1
            if room <= 0:
                self._trim_tag_tts_queue_to_budget()
                if not self._tag_tts_tail_suppressed:
                    await self._mark_tag_tail_blocked(
                        reason="gate_period_cap",
                        played_ms=played_ms,
                    )
                return True
            data = data[:room]

        if data:
            self._tts_playback_active = True
            if self._defer_in_progress:
                self._defer_last_tts_audio_ts = time.monotonic()
                self._defer_saw_tts = True
            self._on_tts_audio(data)
            self._touch_activity()
        return True

    async def _finalize_tag_tail_after_confirm(
        self,
        *,
        reason: str,
        played_ms: int = 0,
        trimmed_bytes: int = 0,
    ) -> None:
        """确认语已播够：丢弃标签尾音并通知云端停止合成。"""
        if self._tag_tts_tail_suppressed:
            return
        target_ms = self._confirm_target_ms()
        self._tag_tts_tail_suppressed = True
        self._drop_incoming_tts = True
        if played_ms >= target_ms and self._client and self._client.is_in_session:
            try:
                await self._client.send_client_interrupt()
            except Exception as e:
                if self._debug_interrupt:
                    self._log(
                        "warn",
                        f"[doubao][cmd-tag] ClientInterrupt after confirm failed: {e}",
                    )
        self._log(
            "info",
            f"[doubao][cmd-tag] tag tail off ({reason}) "
            f"played_ms={played_ms}/{target_ms} "
            f"gate={self._tag_tts_received_bytes}/{self._tag_confirm_bytes_budget} "
            f"trimmed_bytes={trimmed_bytes}",
        )

    async def _mark_tag_tail_blocked(
        self,
        *,
        reason: str,
        played_ms: int = 0,
        trimmed_bytes: int = 0,
    ) -> None:
        """仅阻断标签尾音分片，不 flush 已排队确认语、不 ClientInterrupt。"""
        if self._tag_tts_tail_suppressed:
            return
        if played_ms < self._tag_gate_min_block_ms():
            return
        self._tag_tts_tail_suppressed = True
        self._drop_incoming_tts = True
        queue_trimmed = self._trim_tag_tts_queue_to_budget()
        trimmed_bytes = trimmed_bytes + queue_trimmed
        await self._stop_tag_cloud_tts()
        target_ms = self._confirm_target_ms()
        _, queued_b, _ = self._tag_playback_totals()
        self._log(
            "info",
            f"[doubao][cmd-tag] block tag tail ({reason}) "
            f"played_ms={played_ms} confirm_target_ms={target_ms} "
            f"trimmed_bytes={trimmed_bytes} queued_after={queued_b} "
            f"gate={self._tag_tts_received_bytes}/{self._tag_confirm_bytes_budget} "
            f"(drop_after_played_ok)",
        )

    async def _apply_tag_tts_suppress(
        self,
        *,
        reason: str,
        played_ms: int = 0,
        send_client_interrupt: bool = True,
        flush_playback: bool = True,
    ) -> None:
        """丢弃标签 TTS：可选清空本地队列并通知云端（打断场景用）。"""
        if self._tag_tts_tail_suppressed:
            return
        self._tag_tts_tail_suppressed = True
        self._drop_incoming_tts = True
        cleared = 0
        if flush_playback:
            flush_fn = getattr(self._node, "flush_tts_playback", None)
            if callable(flush_fn):
                cleared = int(flush_fn() or 0)
        if send_client_interrupt and self._client and self._client.is_in_session:
            try:
                await self._client.send_client_interrupt()
            except Exception as e:
                if self._debug_interrupt:
                    self._log(
                        "warn",
                        f"[doubao][cmd-tag] ClientInterrupt failed: {e}",
                    )
        if (
            self._truncate_enabled
            and self._client
            and self._current_reply_id
            and played_ms > 0
        ):
            try:
                await self._client.send_conversation_truncate(
                    self._current_reply_id, played_ms
                )
            except Exception as e:
                if self._debug_interrupt:
                    self._log(
                        "warn",
                        f"[doubao][cmd-tag] ConversationTruncate failed: {e}",
                    )
        urgency = self._cmd_engine.tag_cut_urgency()
        need_ms = self._cmd_engine.estimated_spoken_ms_before_tag_suppress(
            played_ms=played_ms, urgency=urgency
        )
        self._log(
            "info",
            f"[doubao][cmd-tag] suppress tag TTS ({reason}) "
            f"played_ms={played_ms} est_confirm={need_ms} "
            f"urgency={urgency} gate_bytes={self._tag_tts_received_bytes}/"
            f"{self._tag_confirm_bytes_budget} cleared_chunks={cleared}",
        )

    async def _seal_tag_tts_gate(self) -> int:
        """Chat 已见 。[：裁掉队列中句号之后的 PCM（含 [ 对应尾音）。"""
        if self._shutting_down or self._tag_tts_tail_suppressed:
            return 0
        if not self._tag_tts_gate_active:
            self._arm_tag_tts_gate(period_cap=True)
        played_b, queued_b, played_ms = self._tag_playback_totals()
        budget = self._tag_confirm_bytes_budget
        trimmed = 0
        keep_queued = max(0, budget - played_b)
        if queued_b > keep_queued:
            trim_fn = getattr(self._node, "trim_tts_playback_queue", None)
            if callable(trim_fn):
                trimmed = int(trim_fn(keep_queued) or 0)
            played_b, queued_b, played_ms = self._tag_playback_totals()
            self._tag_tts_received_bytes = played_b + queued_b
        trimmed += self._trim_tag_tts_queue_to_budget()
        period_ms = self._cmd_engine.period_cut_target_ms()
        min_block_ms = self._tag_gate_min_block_ms()
        if played_ms >= min_block_ms and (
            played_ms >= period_ms or trimmed > 0
        ):
            await self._mark_tag_tail_blocked(
                reason="chat_seal",
                played_ms=played_ms,
                trimmed_bytes=trimmed,
            )
        return trimmed

    async def _maybe_suppress_tag_tts_tail(self) -> None:
        """确认语播到标签前文本后再丢弃 [CMD:...] 对应 TTS（门控未启用时的兜底）。"""
        if self._injected_music_defer_active():
            return
        if self._cmd_engine.intent_tag_tts_cut_needed() and not self._tag_tts_gate_active:
            self._arm_tag_tts_gate()
        if self._tag_tts_gate_active:
            return
        if self._tag_tts_tail_suppressed or not self._cmd_engine.tag_tts_tail_pending():
            return
        played_fn = getattr(self._node, "tts_playback_played_ms", None)
        played_ms = int(played_fn() or 0) if callable(played_fn) else 0
        urgency = self._cmd_engine.tag_cut_urgency()
        need_ms = self._cmd_engine.estimated_spoken_ms_before_tag_suppress(
            played_ms=played_ms, urgency=urgency
        )
        if played_ms < need_ms:
            return
        await self._apply_tag_tts_suppress(
            reason="played_ms",
            played_ms=played_ms,
            send_client_interrupt=True,
        )

    async def _interrupt_tts_playback(
        self,
        *,
        reason: str = "unknown",
        question_id: str = "",
        send_client_interrupt: bool = True,
        truncate_context: bool = True,
        suppress_followup: bool = True,
    ) -> None:
        """用户开口或命令打断：截断上下文、通知服务端、清空本地播放队列。"""
        queue_before = 0
        qsize_fn = getattr(self._node, "tts_playback_queue_size", None)
        if callable(qsize_fn):
            queue_before = int(qsize_fn() or 0)

        played_ms = 0
        played_fn = getattr(self._node, "tts_playback_played_ms", None)
        if callable(played_fn):
            played_ms = int(played_fn() or 0)

        truncated = False
        if (
            truncate_context
            and self._truncate_enabled
            and self._client
            and self._current_reply_id
            and played_ms > 0
        ):
            try:
                await self._client.send_conversation_truncate(
                    self._current_reply_id, played_ms
                )
                truncated = True
            except Exception as e:
                if self._debug_interrupt:
                    self._node.get_logger().warn(
                        f"[doubao][barge-in] ConversationTruncate failed: {e}"
                    )

        sent_interrupt = False
        if send_client_interrupt and self._client and self._client.is_in_session:
            try:
                await self._client.send_client_interrupt()
                sent_interrupt = True
                self._recent_client_interrupt_mono = time.monotonic()
            except Exception as e:
                if self._debug_interrupt:
                    self._node.get_logger().warn(
                        f"[doubao][barge-in] ClientInterrupt failed: {e}"
                    )
        elif not send_client_interrupt and self._debug_interrupt:
            self._node.get_logger().debug(
                f"[doubao][barge-in] skip ClientInterrupt (reason={reason})"
            )

        cleared = 0
        flush_fn = getattr(self._node, "flush_tts_playback", None)
        if callable(flush_fn):
            cleared = int(flush_fn() or 0)
        else:
            clear_fn = getattr(self._node, "clear_tts_playback_queue", None)
            if callable(clear_fn):
                cleared = int(clear_fn() or 0)

        self._tts_playback_active = False
        was_suppressing = self._drop_incoming_tts
        if suppress_followup:
            self._drop_incoming_tts = True

        msg = (
            f"[doubao][barge-in] reason={reason}"
            f" queue_before={queue_before}"
            f" cleared={cleared}"
            f" played_ms={played_ms}"
            f" reply_id={self._current_reply_id or '-'}"
            f" truncate={'yes' if truncated else 'no'}"
            f" client_interrupt={'yes' if sent_interrupt else 'skip'}"
            f" suppress_after={'on' if suppress_followup else 'off'}"
            f" suppress_before={'on' if was_suppressing else 'off'}"
        )
        if question_id:
            msg += f" question_id={question_id}"
        self._node.get_logger().info(msg)

    async def _wait_tts_playback_idle(self, *, timeout_s: float = 60.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._is_tts_playback_idle():
                await asyncio.sleep(0.2)
                if self._is_tts_playback_idle():
                    return
            await asyncio.sleep(0.05)

    async def _execute_music_control(self, music_action: str, cmd_id: str) -> None:
        try:
            result = await self._ros2.call_music_control(music_action, timeout_s=8.0)
        except Exception as e:
            self._node.get_logger().warn(
                f"[cmd] music {music_action} ({cmd_id}): ros2 error: {e}"
            )
            return
        if (
            not result.ok
            and music_action == "user_resume"
            and "未在播放" in (result.message or "")
        ):
            self._node.get_logger().info(
                "[cmd] user_resume: nothing playing, fallback to play"
            )
            music_action = "play"
            result = await self._ros2.call_music_control("play", timeout_s=8.0)
        if (
            not result.ok
            and music_action.startswith("play_named:")
            and "未找到" in (result.message or "")
        ):
            self._node.get_logger().warn(
                f"[cmd] {music_action}: not in library, fallback to random local play"
            )
            result = await self._ros2.call_music_control("play", timeout_s=8.0)
        if result.ok:
            if result.current_file:
                self._last_music_current_file = str(result.current_file)
            self._node.get_logger().info(
                f"[cmd] music {music_action}: ok file={result.current_file!r}"
            )
            if music_action == "user_pause":
                self._user_music_paused = True
                if not self._tts_music_independent:
                    self._music_duck_suppress_until = time.time() + 25.0
            elif music_action == "stop":
                self._user_music_paused = False
                if not self._tts_music_independent:
                    self._music_duck_suppress_until = time.time() + 12.0
            elif music_action in _MUSIC_ACTIONS_DEFER_FOR_TTS or music_action.startswith(
                "play_named:"
            ):
                if music_action == "user_resume":
                    self._user_music_paused = False
                if not self._tts_music_independent:
                    self._music_duck_suppress_until = (
                        time.time() + _MUSIC_DUCK_SUPPRESS_AFTER_LOCAL_S
                    )
        else:
            self._node.get_logger().warn(
                f"[cmd] music {music_action}: failed: {result.message or 'unknown'}"
            )

    async def _execute_quick_music_and_interrupt(
        self, music_action: str, cmd_id: str
    ) -> None:
        """Quick music 命令直接执行并打断歌词 TTS，清空播放队列，防止与本地 MP3 叠播。

        适用于 ASR 超时直接执行 / 歌词流早检测 两条路径。
        """
        self._log(
            "info",
            f"[cmd] quick music + interrupt lyrics: {cmd_id} action={music_action}",
        )
        spoken = self._cmd_engine.spoken_text_before_tag()
        has_short_confirm = bool(spoken) and any(
            h in spoken for h in _CHAT_CONFIRM_HINTS
        )
        if has_short_confirm and len(spoken) <= 24:
            floor_ms = max(380, self._cmd_engine.period_cut_target_ms() // 3)
            await self._wait_tts_confirm_floor_ms(floor_ms)
        await self._execute_music_control(music_action, cmd_id)
        # 屏蔽在途的歌词 TTS 块，清空队列
        self._drop_incoming_tts = True
        flush_fn = getattr(self._node, "flush_tts_playback", None)
        if callable(flush_fn):
            flush_fn()
        reset_fn = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_fn):
            reset_fn()
        self._tts_playback_active = False
        self._dropped_tts_chunks = 0
        self._dropped_tts_bytes = 0
        if self._client and self._client.is_in_session:
            try:
                await self._client.send_client_interrupt()
            except Exception:
                pass
        # ClientInterrupt 已发出；解除屏蔽，让后续回复的 TTS 正常播放
        # （若 EVT_TTSEnded 先到则其 handler 也会 reset，此处为 no-op）
        self._drop_incoming_tts = False

    @staticmethod
    def _format_tts_confirm(template: str, *, current_file: str = "", arg: str = "") -> str:
        label = current_file.strip() or "音乐"
        try:
            return template.format(current_file=label, arg=arg)
        except Exception:
            return template

    async def _music_duck_for_tts(self) -> None:
        if not self._music_duck_enabled or self._cmd_busy:
            return
        if (
            self._defer_in_progress
            or self._deferred_music is not None
            or time.monotonic() < self._defer_tts_guard_until
        ):
            return
        if self._cmd_engine.tag_tts_tail_pending() or self._tag_tts_gate_active:
            return
        if self._user_music_paused:
            return
        if time.time() < self._music_duck_suppress_until:
            return
        result = await self._ros2.call_music_control("pause", timeout_s=2.0)
        if result.ok:
            self._tts_did_duck = True
        elif result.message:
            self._node.get_logger().debug(f"[music] TTS duck pause: {result.message}")

    async def _music_unduck_after_tts(self) -> None:
        if not self._music_duck_enabled or not self._tts_did_duck:
            return
        self._tts_did_duck = False
        if self._user_music_paused or self._defer_in_progress or self._deferred_music is not None:
            return
        if time.time() < self._music_duck_suppress_until:
            return
        result = await self._ros2.call_music_control("resume", timeout_s=3.0)
        if not result.ok and result.message:
            self._node.get_logger().debug(f"[music] TTS duck resume: {result.message}")

    async def _enqueue_biz_tts(self, text: str, *, urgent: bool = False) -> None:
        if self._biz_tts_queue is None or self._shutting_down:
            return
        if urgent:
            while not self._biz_tts_queue.empty():
                try:
                    self._biz_tts_queue.get_nowait()
                    self._biz_tts_queue.task_done()
                except asyncio.QueueEmpty:
                    break
        try:
            self._biz_tts_queue.put_nowait(text)
        except asyncio.QueueFull:
            self._log("warn", f"[biz-tts] queue full, drop oldest for {text!r}")
            try:
                self._biz_tts_queue.get_nowait()
                self._biz_tts_queue.task_done()
            except asyncio.QueueEmpty:
                pass
            await self._biz_tts_queue.put(text)

    async def _biz_tts_worker(self) -> None:
        while not self._shutting_down:
            try:
                text = await self._biz_tts_queue.get()
            except asyncio.CancelledError:
                return
            try:
                await self._play_biz_tts(text)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._log("warn", f"[biz-tts] play error: {e}")
            finally:
                self._biz_tts_queue.task_done()

    def _cancel_biz_tts_hold_task(self) -> None:
        task = self._biz_tts_hold_task
        self._biz_tts_hold_task = None
        if task is not None and not task.done():
            task.cancel()

    def _schedule_biz_tts_hold_watchdog(self, timeout_s: float = 15.0) -> None:
        self._cancel_biz_tts_hold_task()
        if not self._loop.is_running():
            return
        self._biz_tts_hold_task = self._loop.create_task(
            self._biz_tts_hold_watchdog(timeout_s),
            name="biz-tts-hold-watchdog",
        )

    async def _biz_tts_hold_watchdog(self, timeout_s: float) -> None:
        try:
            await asyncio.sleep(timeout_s)
            if self._awaiting_biz_tts:
                self._log(
                    "warn",
                    f"[biz-tts] playback timeout ({timeout_s}s), force release",
                )
                self._finish_biz_tts_playback(reason="timeout")
        except asyncio.CancelledError:
            return

    def _finish_biz_tts_playback(self, *, reason: str) -> None:
        self._cancel_biz_tts_hold_task()
        self._awaiting_biz_tts = False
        if self._biz_tts_hold_uplink:
            self._biz_tts_hold_uplink = False
            self._greeting_uplink_hold = False
        self._tts_playback_active = False
        if self._biz_tts_done is not None and not self._biz_tts_done.is_set():
            self._biz_tts_done.set()
        self._log("info", f"[biz-tts] TTS ended ({reason})")
        self._publish_playback_done_safe()
        self._touch_activity()

    async def _wait_biz_tts_audio_start(self, *, timeout_s: float = 6.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._shutting_down:
            if self._tts_played_ms() > 50:
                return True
            if self._biz_tts_done is not None and self._biz_tts_done.is_set():
                return self._tts_played_ms() > 0
            await asyncio.sleep(0.04)
        return self._tts_played_ms() > 0

    async def _play_biz_tts(self, text: str) -> None:
        text = str(text or "").strip()
        if not text or self._shutting_down:
            return
        self._session_active = True
        await self._ensure_session_started()
        if not self._client or not self._client.is_in_session:
            self._log("warn", f"[biz-tts] skip (no session): {text!r}")
            return
        if self._tts_playback_active or not self._is_tts_playback_idle():
            await self._interrupt_tts_playback(
                reason="biz_tts",
                send_client_interrupt=True,
                truncate_context=False,
                suppress_followup=True,
            )
        self._drop_incoming_tts = False
        self._dropped_tts_chunks = 0
        self._dropped_tts_bytes = 0
        reset_stats = getattr(self._node, "reset_tts_playback_stats", None)
        if callable(reset_stats):
            reset_stats()
        self._biz_tts_done = asyncio.Event()
        self._awaiting_biz_tts = True
        self._biz_tts_hold_uplink = True
        self._greeting_uplink_hold = True
        self._biz_tts_playing = True
        self._schedule_biz_tts_hold_watchdog()
        try:
            # 无用户 query 时 ChatTTSText 常无音频；SayHello 可在 Session 后直接 TTS
            self._log("info", f"[biz-tts] SayHello: {text!r}")
            await self._client.say_hello(text)
            got_audio = await self._wait_biz_tts_audio_start(timeout_s=6.0)
            if not got_audio:
                self._log(
                    "warn",
                    f"[biz-tts] SayHello no audio, fallback ChatTTSText: {text!r}",
                )
                self._server_asr_ended = True
                await self._client.send_chat_tts_text(text, start=True, end=True)
                got_audio = await self._wait_biz_tts_audio_start(timeout_s=5.0)
            if not got_audio:
                self._log("warn", f"[biz-tts] no TTS audio received: {text!r}")
                return
            try:
                await asyncio.wait_for(self._biz_tts_done.wait(), timeout=30.0)
            except asyncio.TimeoutError:
                self._log("warn", f"[biz-tts] wait TTSEnded timeout: {text!r}")
            await self._wait_tts_playback_idle(timeout_s=20.0)
            self._log(
                "info",
                f"[biz-tts] played: {text!r} ({self._tts_played_ms()}ms)",
            )
        finally:
            self._biz_tts_playing = False
            if self._awaiting_biz_tts:
                self._finish_biz_tts_playback(reason="cleanup")
        await self._music_unduck_after_tts()
        await self._maybe_finish_session_after_biz_tts()

    async def _maybe_finish_session_after_biz_tts(self) -> None:
        if self._shutting_down or not self._biz_tts_auto_finish:
            return
        if self._pending_session_exit or self._defer_in_progress:
            return
        if self._biz_tts_queue is not None and not self._biz_tts_queue.empty():
            return
        if (time.time() - self._last_final_ts) < 30.0:
            return
        if not self._client or not self._client.is_in_session:
            return
        self._log("info", "[biz-tts] auto finish_session (no recent user speech)")
        self._session_active = False
        await self._finish_session_and_cleanup()

    async def _wait_server_asr_ended(self, *, timeout_s: float = 3.0) -> bool:
        """豆包要求 ChatTTSText 在 ASREnded 之后发送，否则常无 TTS 音频。"""
        if self._server_asr_ended:
            return True
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._shutting_down:
            if self._server_asr_ended:
                return True
            await asyncio.sleep(0.04)
        self._log(
            "warn",
            f"[cmd] ChatTTSText: ASREnded not seen within {timeout_s:.1f}s, send anyway",
        )
        return False

    async def _send_chat_tts_text(self, text: str):
        if not self._client or not text.strip():
            return
        await self._wait_server_asr_ended(timeout_s=3.0)
        text = text.strip()
        max_len = 80
        if len(text) <= max_len:
            await self._client.send_chat_tts_text(text, start=True, end=True)
            return
        chunks = [text[i : i + max_len] for i in range(0, len(text), max_len)]
        for i, c in enumerate(chunks):
            await self._client.send_chat_tts_text(c, start=(i == 0), end=(i == len(chunks) - 1))

