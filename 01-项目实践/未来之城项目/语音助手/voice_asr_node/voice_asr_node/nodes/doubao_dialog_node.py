#!/usr/bin/env python3
# ================================================================
# doubao_dialog_node.py
#
# ROS2 语音对话节点（豆包端到端实时语音）
#
# 功能：
#   1) 本地 KWS 唤醒（sherpa-onnx）
#   2) 麦克风 16k int16 20ms 流式上传到豆包 TaskRequest
#   3) 接收豆包 TTSResponse(PCM s16le 24k) 并本地播放
#   4) 接收 ASRResponse(final) 后优先走 CmdEngine 命令词执行
#   5) 订阅 /voice_asr/tts_speak，将文本通过 ChatTTSText 走豆包音色播报
#
# 说明：
#   - 默认使用 server_vad（不启用 push_to_talk）
#   - 联网搜索默认关闭（config/doubao.yaml 中 websearch.enable=false）
# ================================================================

import asyncio
import concurrent.futures
import logging
import os
import queue
import subprocess
import threading
import time
from typing import Any, Callable, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String

from ..command import CmdEngine, CmdRegistry
from ..config.app_config import load_app_config
from ..websocket import (
    DoubaoRealtimeClient,
    EVT_ASRResponse,
    EVT_ChatResponse,
    EVT_DialogCommonError,
    EVT_TTSEnded,
    EVT_TTSResponse,
    EVT_UsageResponse,
    ServerFrame,
)
from ..audio.sd_capture import CaptureConfig, SoundDeviceCapture
from ..audio.sd_player import PlayerConfig, SoundDevicePlayer
from ..utils.env_setup import setup_audio
from ..kws.asr_wake_engine import ASRWakeEngine
from ..ros2.bridge import ROS2Bridge
from ..session.doubao_session_manager import DoubaoSessionManager
from ..audio.pcm_ring_buffer import PcmRingBuffer
from ..speaker.speaker_id import SpeakerIdentifier

logger = logging.getLogger("doubao_dialog_node")
logging.basicConfig(level=logging.INFO)

MODEL_SR = 16000
MODEL_CH = 1
MODEL_FRAME_MS = 20
MODEL_FRAME_SAMPLES = int(MODEL_SR * MODEL_FRAME_MS / 1000)
MODEL_FRAME_BYTES = MODEL_FRAME_SAMPLES * 2

PLAYBACK_SR = 24000
PLAYBACK_CH = 1


class DoubaoDialogNode(Node):
    def __init__(self):
        super().__init__("doubao_dialog_node")

        # ── 参数 ────────────────────────────────────────────────
        self.declare_parameter("doubao_config", "")
        self.declare_parameter("command_config", "")
        self.declare_parameter("tts_speak_topic", "/voice_asr/tts_speak")
        self.declare_parameter("exit_intent_topic", "/voice_asr/exit_intent")
        self.declare_parameter("initial_volume", 50)
        self.declare_parameter("mic_device", "")
        self.declare_parameter("playback_device", "")
        self.declare_parameter("kws_threshold", 0.15)
        self.declare_parameter("kws_input_gain_db", 6.0)
        self.declare_parameter("kws_debug_audio_level", False)

        self._tts_speak_topic = self.get_parameter("tts_speak_topic").value
        self._exit_intent_topic = self.get_parameter("exit_intent_topic").value

        # 统一配置入口：固定读取 config/doubao.yaml + config/command.yaml
        self._app_cfg = load_app_config(
            doubao_path=str(self.get_parameter("doubao_config").value),
            command_path=str(self.get_parameter("command_config").value),
        )
        self._cfg = self._app_cfg.raw
        from ..command import cmd_engine_from_command_raw

        self._cmd_engine = cmd_engine_from_command_raw(self._app_cfg.command_raw)
        self._ros2 = ROS2Bridge(self)

        # launch 参数覆盖配置文件
        mic_device = self.get_parameter("mic_device").value
        if mic_device:
            self._cfg.setdefault("audio", {})["mic_device"] = mic_device
        pb_device = self.get_parameter("playback_device").value
        if pb_device:
            self._cfg.setdefault("audio", {})["playback_device"] = pb_device
        kws_thr = float(self.get_parameter("kws_threshold").value)
        kws_gain_db = float(self.get_parameter("kws_input_gain_db").value)
        ww_cfg = self._cfg.setdefault("wakeword", {})
        ww_cfg["kws_threshold"] = kws_thr
        ww_cfg["kws_input_gain_db"] = kws_gain_db
        # 兼容 ROS2 launch 传入字符串 'true'/'false' 和原生 bool 两种情况
        _kws_dbg_raw = self.get_parameter("kws_debug_audio_level").value
        self._kws_debug_audio_level = str(_kws_dbg_raw).lower() not in ('false', '0', 'no', '')

        # ── PulseAudio 预热（优先 doubao.yaml audio_output）────────
        ao = self._app_cfg.audio_output
        initial_volume = int(ao.initial_volume_percent)
        launch_vol = int(self.get_parameter("initial_volume").value)
        if launch_vol != 50:
            initial_volume = launch_vol
        setup_audio(
            initial_volume_pct=initial_volume,
            pulse_sink=ao.pulse_sink,
        )

        # 声纹识别（可选）
        self._speaker = SpeakerIdentifier(self._app_cfg.speaker_id)
        self._speaker.init()

        min_ms = max(500, self._app_cfg.speaker_id.min_audio_ms)
        self._wake_pcm_buffer = PcmRingBuffer(
            max_bytes=int(MODEL_SR * 2 * min_ms / 1000)
        )

        # 命令词引擎已由 config/command.yaml 加载（self._cmd_engine）

        # ── 会话状态（交由 DoubaoSessionManager 维护）──────────────
        self._destroying = False
        self._dialog_wakeup_logged = False
        self._audio_tx_start_ts = None
        self._audio_playback_start_ts = None
        self._last_kws_interrupt_ts = 0.0
        self._ros_invoke_q: queue.Queue = queue.Queue()
        self._ros_invoke_stop = threading.Event()
        self._ros_invoke_thread = threading.Thread(
            target=self._ros_invoke_worker, daemon=True, name="doubao-ros-invoke"
        )

        # ── ROS 通信（先创建，供 session 回调使用）────────────────
        self._sub_tts = self.create_subscription(String, self._tts_speak_topic, self._on_tts_text, 10)
        self._pub_done = self.create_publisher(Empty, "/voice/speech_playback_done", 10)
        self._pub_exit = self.create_publisher(Empty, self._exit_intent_topic, 10)
        self.create_timer(1.0, self._on_timer)
        self._music_volume_applied = False
        self._music_vol_sync_scheduled = False
        self._music_vol_timer = self.create_timer(2.0, self._apply_config_music_volume_once)

        # ── 会话管理器（必须先于 audio/kws 模块，供回调引用）────
        self._session = DoubaoSessionManager(
            node=self,
            cfg=self._app_cfg,
            ros2=self._ros2,
            on_tts_audio=self._on_tts_audio_chunk,
            publish_playback_done=self._publish_playback_done_safe,
            publish_exit_intent=self._publish_exit_intent_safe,
            speaker=self._speaker,
        )

        # ── 音频线程资源（采集/播放模块化）───────────────────────
        self._stop_event = threading.Event()
        self._player = SoundDevicePlayer(
            PlayerConfig(
                playback_device=str(self._cfg.get("audio", {}).get("playback_device", "")),
                sample_rate=PLAYBACK_SR,
                channels=PLAYBACK_CH,
            ),
            logger=self.get_logger(),
            on_first_chunk=self._session.on_playback_started,
        )
        self._session.bind_tts_playback_idle(self._player.is_idle)
        self._capture = SoundDeviceCapture(
            CaptureConfig(
                mic_device=str(self._cfg.get("audio", {}).get("mic_device", "")),
                target_sample_rate=MODEL_SR,
                channels=MODEL_CH,
                frame_ms=MODEL_FRAME_MS,
            ),
            logger=self.get_logger(),
            on_frame=self._on_mic_frame,
        )
        self._mic_thread = threading.Thread(
            target=self._capture.run_forever, daemon=True, name="doubao-mic"
        )
        self._playback_thread = threading.Thread(
            target=self._player.run_forever, daemon=True, name="doubao-playback"
        )

        # ── 本地 KWS 唤醒（Sherpa KeywordSpotter，见 config wakeword）──
        self._wake_engine: Optional[ASRWakeEngine] = None
        try:
            ww = self._app_cfg.wakeword
            self._wake_engine = ASRWakeEngine.from_app_config(self._app_cfg)
            if self._wake_engine.init():
                if self._kws_debug_audio_level:
                    logging.getLogger("voice_asr_node.kws.keyword_spotter").setLevel(
                        logging.DEBUG
                    )
                    self.get_logger().info(
                        "[ASRWake] debug: KeywordSpotter 详细日志已开启"
                    )
                self._wake_engine.start()
                self.get_logger().info(
                    f"[ASRWake] ready langs={ww.active_languages} keywords={self._wake_engine.wake_keywords}"
                )
            else:
                self._wake_engine = None
                self.get_logger().warn("[ASRWake] no recognizers loaded")
        except Exception as e:
            self._wake_engine = None
            self.get_logger().warn(f"[ASRWake] disabled: {e}")

        # ── 启动线程 ───────────────────────────────────────────
        self._ros_invoke_thread.start()
        self._session.start()
        self._mic_thread.start()
        self._playback_thread.start()

        self.get_logger().info("[doubao_dialog_node] started")
        model_version = self._cfg.get("model", {}).get("version", "")
        speaker = self._cfg.get("tts", {}).get("speaker", "")
        self.get_logger().info(
            f"[doubao_dialog_node] config: model={model_version} speaker={speaker}"
        )
        if self._session.tts_music_independent:
            pb = str(self._cfg.get("audio", {}).get("playback_device", "") or "default")
            self.get_logger().info(
                "[audio] TTS 与背景音乐独立：线程 doubao-playback (sounddevice) "
                f"+ music_player (paplay)，互不 duck；[CMD:...] 仅门控不朗读"
                f" playback_device={pb!r}"
            )

    def _log_safe(self, level: str, msg: str) -> None:
        """关闭阶段 ROS context 可能已失效，避免 rosout 报错。"""
        try:
            if rclpy.ok() and not self._destroying:
                getattr(self.get_logger(), level)(msg)
                return
        except Exception:
            pass
        print(f"[doubao_dialog_node][{level}] {msg}", flush=True)

    def _cancel_music_vol_timer(self) -> None:
        timer = getattr(self, "_music_vol_timer", None)
        if timer is not None:
            try:
                timer.cancel()
            except Exception:
                pass
            self._music_vol_timer = None

    def mark_music_volume_initialized(self) -> None:
        """用户或启动流程已设定音量后，禁止默认定时器再次写回默认音量。"""
        self._music_volume_applied = True
        self._music_vol_sync_scheduled = True
        self._cancel_music_vol_timer()

    def _apply_config_music_volume_once(self) -> None:
        if self._music_volume_applied or self._destroying:
            self._cancel_music_vol_timer()
            return
        if self._music_vol_sync_scheduled:
            return
        self._music_vol_sync_scheduled = True
        self._cancel_music_vol_timer()

        vol = float(self._app_cfg.audio_output.music_default_volume)

        def _sync():
            return self._ros2.set_music_volume_sync(vol, timeout_s=3.0)

        fut = self.invoke_on_spin_thread(_sync)

        def _done(f: concurrent.futures.Future) -> None:
            try:
                result = f.result(timeout=5.0)
                if result.ok:
                    self.get_logger().info(
                        f"[audio] music default volume -> {result.current_volume:.0f}%"
                    )
                    self.mark_music_volume_initialized()
                else:
                    self._music_vol_sync_scheduled = False
                    self.get_logger().debug(
                        f"[audio] music volume sync pending: {result.message}"
                    )
            except Exception:
                self._music_vol_sync_scheduled = False

        fut.add_done_callback(_done)

    def _on_mic_frame(self, frm: bytes) -> None:
        if self._destroying or self._stop_event.is_set():
            return
        self._wake_pcm_buffer.write(frm)
        # KWS 全程运行：唤醒词可在对话中持续打断当前响应
        if self._wake_engine is not None:
            self._wake_engine.feed_audio(frm)
            hit_evt = self._wake_engine.poll_wake()
            hit = hit_evt.keyword if hit_evt else None
            if hit:
                self._on_kws_detect(hit)
        # 会话态推送到异步上行队列（由 DoubaoSessionManager 管理）
        self._session.offer_audio_frame(frm)

    def _reset_dialog_timing(self) -> None:
        self._audio_tx_start_ts = None
        self._audio_playback_start_ts = None

    def _log_dialog_started(self, source: str, keyword: str = "") -> None:
        if self._dialog_wakeup_logged:
            return
        self._dialog_wakeup_logged = True
        self._reset_dialog_timing()
        if keyword:
            self.get_logger().info(f"[dialog] 已唤醒 ({source}): {keyword}")
        else:
            self.get_logger().info(f"[dialog] 已唤醒 ({source})")

    def _log_dialog_ended(self, reason: str) -> None:
        if self._audio_tx_start_ts is not None and self._audio_playback_start_ts is not None:
            latency_ms = (self._audio_playback_start_ts - self._audio_tx_start_ts) * 1000.0
            self.get_logger().info(f"[dialog] 对话结束: {reason} | 音频上行到开始播放延迟 {latency_ms:.1f} ms")
        else:
            self.get_logger().info(f"[dialog] 对话结束: {reason}")
        self._dialog_wakeup_logged = False
        self._reset_dialog_timing()

    def _on_tts_audio_chunk(self, payload: bytes) -> None:
        """由 DoubaoSessionManager 调用：把 TTS 音频推入播放器。"""
        self._player.enqueue(payload)

    def _publish_playback_done_safe(self) -> None:
        if self._destroying:
            return
        try:
            if rclpy.ok():
                self._pub_done.publish(Empty())
        except Exception:
            pass

    def _publish_exit_intent_safe(self) -> None:
        if self._destroying:
            return
        try:
            if rclpy.ok():
                self._pub_exit.publish(Empty())
        except Exception:
            pass

    def _clear_playback_queue(self) -> int:
        return self._player.clear_queue()

    def clear_tts_playback_queue(self) -> int:
        """供 DoubaoSessionManager 在切本地音乐前清空豆包 TTS 播放队列。"""
        return self._player.clear_queue()

    def tts_playback_queue_size(self) -> int:
        return self._player.qsize()

    def flush_tts_playback(self) -> int:
        """立即停止本地 TTS 输出并清空待播队列（用户打断）。"""
        q_before = self._player.qsize()
        cleared = self._player.flush()
        debug = bool(self._cfg.get("audio", {}).get("debug_interrupt", False))
        if debug and (cleared or q_before):
            self.get_logger().info(
                f"[playback][barge-in] flush queue_before={q_before} "
                f"cleared_chunks={cleared} played_ms={self._player.played_ms()}"
            )
        return cleared

    def reset_tts_playback_stats(self) -> None:
        self._player.reset_playback_stats()

    def tts_playback_played_ms(self) -> int:
        return self._player.played_ms()

    def tts_playback_played_bytes(self) -> int:
        return self._player.bytes_played

    def tts_playback_queued_bytes(self) -> int:
        return self._player.queued_bytes()

    def trim_tts_playback_queue(self, keep_bytes: int) -> int:
        return self._player.trim_queue_keep_bytes(keep_bytes)

    # ============================================================
    # Doubao 会话逻辑已下沉至 `DoubaoSessionManager`
    # ============================================================

    # ============================================================
    # ROS 回调 / 定时
    # ============================================================
    def _on_tts_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        urgent = False
        if text.startswith("[URGENT]"):
            urgent = True
            text = text[len("[URGENT]"):].strip()
        if not text:
            return
        self.get_logger().info(f"[biz-tts] recv: {text!r} urgent={urgent}")
        self._session.on_tts_text(text, urgent=urgent)

    def _on_timer(self):
        self._session.on_timer()

    def invoke_on_spin_thread(self, fn: Callable[[], Any]) -> concurrent.futures.Future:
        """将阻塞型 rclpy 服务调用投递到 spin 线程执行（供 ROS2Bridge 使用）。"""
        done: concurrent.futures.Future = concurrent.futures.Future()
        self._ros_invoke_q.put((fn, done))
        return done

    def _ros_invoke_worker(self):
        while not self._ros_invoke_stop.is_set():
            try:
                fn, done = self._ros_invoke_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if done.done():
                continue
            try:
                done.set_result(fn())
            except Exception as exc:
                done.set_exception(exc)

    # ============================================================
    # KWS
    # ============================================================
    def _on_kws_detect(self, keyword: str):
        now = time.time()
        if now - self._last_kws_interrupt_ts < 1.2:
            return
        self._last_kws_interrupt_ts = now

        self.get_logger().info(f"[KWS] wake word detected: {keyword}")

        if self._session.pending_session_exit:
            self.get_logger().info(
                "[KWS] ignored during session exit (wait for farewell to finish)"
            )
            return

        if self._session.session_active and self._session.is_in_session:
            if self._session.defer_in_progress:
                cleared = self.flush_tts_playback()
                if cleared:
                    self.get_logger().info(
                        f"[KWS] flushed {cleared} queued playback chunks (defer confirm)"
                    )
                self.get_logger().info(
                    "[KWS] interrupt TTS during music confirm window (keep defer)"
                )
                self._session.interrupt(reason="kws")
                return
            cleared = self.flush_tts_playback()
            if cleared:
                self.get_logger().info(f"[KWS] flushed {cleared} queued playback chunks")
            self.get_logger().info("[KWS] interrupting active dialog (keep session)")
            self._session.interrupt(reason="kws")
            return

        if self._session.session_active and not self._session.is_in_session:
            self.get_logger().info(
                "[KWS] wake pending (ws not ready), retry start session"
            )
            self._dialog_wakeup_logged = False

        wake_pcm = self._wake_pcm_buffer.snapshot()
        self._session.on_wake(source="KWS", keyword=keyword, wake_pcm=wake_pcm)
        self._log_dialog_started("KWS", keyword)

    # 麦克风采集与播放逻辑已迁移到 audio/sd_capture.py 与 audio/sd_player.py
    # ============================================================
    # 退出
    # ============================================================
    def destroy_node(self):
        if self._destroying:
            super().destroy_node()
            return
        self._destroying = True
        self._log_safe("info", "正在关闭 doubao_dialog_node...")

        try:
            self._session.request_shutdown()
        except Exception:
            pass

        self._ros_invoke_stop.set()
        if self._ros_invoke_thread.is_alive():
            self._ros_invoke_thread.join(timeout=2.0)

        # 1. 先停采集/唤醒，避免关闭 WS 后仍上行音频
        self._stop_event.set()
        try:
            if self._wake_engine is not None:
                self._wake_engine.stop()
        except Exception:
            pass
        try:
            self._capture.stop()
        except Exception:
            pass
        try:
            self._player.stop()
        except Exception:
            pass

        if self._mic_thread.is_alive():
            self._mic_thread.join(timeout=2.0)

        if self._playback_thread.is_alive():
            self._playback_thread.join(timeout=2.0)

        # 2. 再停 WS 会话
        try:
            self._session.stop()
        except Exception as e:
            self._log_safe("warn", f"关闭 session manager 失败: {e}")

        self._log_safe("info", "doubao_dialog_node 已关闭")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DoubaoDialogNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
