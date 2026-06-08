#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：音乐播放节点 - 智能按摩机器人背景音乐管理
支持：播放、暂停、恢复、停止、切换曲目、音量调节
与 TTS 协同：默认由 config/doubao.yaml `audio.tts_music_independent=true` 关闭
TTS duck（豆包 TTS 与 paplay 背景音乐并行混音）；设为 false 时 TTS 会 SIGSTOP 音乐。

播放方式：ffmpeg 解码 → paplay 输出（PulseAudio 软件混音，独立线程 monitor）
"""

# 导出 main 函数和关键模块，防止 PyArmor RFT 模式重命名
__all__ = ['main', 'glob', 'os', 'threading', 'random']

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import MusicControlAction, MusicControlVolume, MusicGetStatus
from robot_interfaces.msg import ModuleStates
from std_msgs.msg import Header
# std_msgs 订阅已移除（音乐由 /music/* 服务控制）
import os
import glob
import json
import signal
import subprocess
import threading
import random
import time
from typing import List


class MusicPlayerNode(Node):
    """音乐播放管理节点（PulseAudio paplay 方式）"""

    def __init__(self):
        super().__init__('music_player_node')

        # 参数配置
        self._setup_parameters()

        # 检查 ffmpeg 和 paplay 可用性
        self.audio_available = self._check_audio_tools()

        # 音乐库管理
        self.playlist: List[str] = []
        self.current_index: int = -1

        # 播放状态
        self.is_playing: bool = False
        self.is_paused: bool = False
        # TTS 暂停计数器：多条 TTS 重叠时只有全部结束才恢复播放
        self._tts_pause_count: int = 0
        self._tts_lock = threading.Lock()
        self._was_playing_before_tts: bool = False  # TTS 前是否正在播放，仅此为 True 才恢复

        # 音量控制（通过 PulseAudio pactl 调节）
        self.current_volume: float = self.default_volume

        # 播放子进程管理
        self._play_lock = threading.Lock()
        self._ffmpeg_proc: subprocess.Popen = None
        self._paplay_proc: subprocess.Popen = None
        self._stop_event = threading.Event()

        # 加载音乐库
        self._load_music_library()

        # ROS 服务和话题
        self._setup_communication()

        # 设置初始 PulseAudio 音量
        self._set_hw_volume(self.current_volume)

        # 播放监控线程
        self._monitor_thread = threading.Thread(target=self._monitor_playback, daemon=True)
        self._monitor_thread.start()

        self.get_logger().info('音乐播放节点已启动（PulseAudio 模式）')
        self.get_logger().info(f'音乐根目录: {self.music_root}')
        self.get_logger().info(f'已加载: {len(self.playlist)} 首音乐')

    # ── 参数配置 ────────────────────────────────────────────────

    def _setup_parameters(self):
        """参数配置"""
        self.declare_parameter('music_root', '')
        self.declare_parameter('default_volume', 0.5)
        self.declare_parameter('fade_duration_ms', 2000)
        self.declare_parameter('pulse_sink', '')  # PulseAudio sink 名称，空=默认

        music_root_param = self.get_parameter('music_root').value
        default_volume_param = self.get_parameter('default_volume').value
        fade_duration_param = self.get_parameter('fade_duration_ms').value
        pulse_sink_param = self.get_parameter('pulse_sink').value

        self.music_root = music_root_param
        # 统一为 0–100 百分数（launch 可传 0.5 表示 50%）
        dv = float(default_volume_param)
        if 0.0 < dv <= 1.0:
            dv *= 100.0
        self.default_volume = dv
        self.fade_duration = fade_duration_param
        self.pulse_sink = pulse_sink_param if pulse_sink_param else None

        if not self.music_root:
            self.get_logger().error('music_root 参数未设置！请检查 launch 文件配置')
            raise ValueError('music_root parameter is required')

    def _check_audio_tools(self) -> bool:
        """检查 ffmpeg 和 paplay 是否可用"""
        try:
            subprocess.run(['ffmpeg', '-version'], capture_output=True, timeout=5)
            subprocess.run(['paplay', '--version'], capture_output=True, timeout=5)
            self.get_logger().info('ffmpeg + paplay (PulseAudio) 可用')
            return True
        except Exception as e:
            self.get_logger().error(f'音频工具不可用: {e}')
            return False

    # ── 音乐库管理 ──────────────────────────────────────────────

    def _load_music_library(self):
        """加载音乐库"""
        if not os.path.exists(self.music_root):
            self.get_logger().warn(f'音乐目录不存在: {self.music_root}')
            return

        audio_extensions = ['*.mp3', '*.wav', '*.ogg', '*.flac']

        for ext in audio_extensions:
            self.playlist.extend(glob.glob(os.path.join(self.music_root, ext)))
            self.playlist.extend(glob.glob(os.path.join(self.music_root, '**', ext), recursive=True))

        self.playlist = list(set(self.playlist))
        self.playlist.sort()

        if self.playlist:
            self.get_logger().info(f'已加载 {len(self.playlist)} 首音乐')
            for f in self.playlist:
                self.get_logger().info(f'  - {os.path.basename(f)}')
        else:
            self.get_logger().warn('未找到任何音乐文件')

    # ── ROS 通信 ────────────────────────────────────────────────

    def _setup_communication(self):
        """设置 ROS 通信"""
        self.music_action_service = self.create_service(
            MusicControlAction, '/music/control_action', self.handle_music_action)

        self.music_volume_service = self.create_service(
            MusicControlVolume, '/music/control_volume', self.handle_music_volume)

        self.status_service = self.create_service(
            MusicGetStatus, '/music/get_status', self.handle_get_status)

        # 注意：不订阅 /ai_response 或 /voice/speech_playback_done
        # 音乐暂停/恢复由 /music/control_action 服务触发；
        # 系统事件（报错、进度播报）默认不打断音乐。

        # ── 发布者：节点状态上报（1 Hz）──────────────────────────
        # 上层通过订阅此话题确认 music_player_node 存活；
        # 超过 3s 未收到消息可视为节点已崩溃。
        # state_json 暂为空 JSON，后续可按需扩展字段。
        self._states_pub = self.create_publisher(ModuleStates, '/music_player/states', 10)

        # 1 Hz 定时器，周期性发布节点心跳状态
        self._states_timer = self.create_timer(1.0, self._publish_states)

        self.get_logger().info('ROS 通信已初始化，状态话题: /music_player/states')

    # ── 播放引擎（ffmpeg → pipe → paplay，PulseAudio 软件混音）────

    def _start_playback(self, music_file: str) -> bool:
        """启动 ffmpeg | paplay 管道播放指定音乐文件。

        通过 PulseAudio 软件混音播放，避免与麦克风录音产生 ALSA 设备抢占。
        ffmpeg 提前解码填满 OS pipe 缓冲，paplay 从 stdin 读取 PCM 播放。
        """
        self._kill_playback()
        self._stop_event.clear()

        try:
            # ffmpeg 尽快解码填满 pipe，给 paplay 提供充足缓冲
            self._ffmpeg_proc = subprocess.Popen(
                [
                    'ffmpeg',
                    '-i', music_file,
                    '-f', 's16le', '-ar', '48000', '-ac', '2',
                    '-loglevel', 'error',
                    'pipe:1'
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=1024 * 512,  # 512 KB
            )

            # paplay 从 stdin 读取原始 PCM，通过 PulseAudio 播放
            paplay_cmd = [
                'paplay', '--raw',
                '--format=s16le', '--rate=48000', '--channels=2',
                '--latency=200'
            ]
            if self.pulse_sink:
                paplay_cmd.extend(['--device', self.pulse_sink])

            self._paplay_proc = subprocess.Popen(
                paplay_cmd,
                stdin=self._ffmpeg_proc.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )

            # 释放父进程的 stdout 引用，让 paplay 独占管道
            self._ffmpeg_proc.stdout.close()
            self._ffmpeg_proc.stdout = None

            return True
        except Exception as e:
            self.get_logger().error(f'启动播放失败: {e}')
            self._kill_playback()
            return False

    def _kill_playback(self):
        """终止当前播放进程"""
        self._stop_event.set()
        for name, proc in [('paplay', self._paplay_proc), ('ffmpeg', self._ffmpeg_proc)]:
            if proc is not None:
                try:
                    # 先尝试 SIGCONT（如果进程被暂停了需要先恢复才能正常终止）
                    try:
                        os.kill(proc.pid, signal.SIGCONT)
                    except Exception:
                        pass
                    proc.terminate()
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    try:
                        proc.wait(timeout=1)
                    except Exception:
                        pass
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
        self._paplay_proc = None
        self._ffmpeg_proc = None

    def _is_playback_active(self) -> bool:
        """检查播放进程是否仍在运行"""
        if self._paplay_proc is None:
            return False
        return self._paplay_proc.poll() is None

    def _pause_playback(self):
        """暂停播放（发送 SIGSTOP 给 paplay 和 ffmpeg）"""
        for proc in [self._paplay_proc, self._ffmpeg_proc]:
            if proc is not None and proc.poll() is None:
                try:
                    os.kill(proc.pid, signal.SIGSTOP)
                except Exception:
                    pass

    def _resume_playback(self):
        """恢复播放（发送 SIGCONT 给 ffmpeg 和 paplay）"""
        for proc in [self._ffmpeg_proc, self._paplay_proc]:
            if proc is not None and proc.poll() is None:
                try:
                    os.kill(proc.pid, signal.SIGCONT)
                except Exception:
                    pass

    # ── PulseAudio 音量控制 ──────────────────────────────────────────

    def _volume_to_percent(self, volume: float) -> int:
        """default_volume 为 0.0~1.0；大于 1 时按 0~100 百分数处理。"""
        v = float(volume)
        if 0.0 <= v <= 1.0:
            pct = int(round(v * 100.0))
        else:
            pct = int(round(v))
        return max(0, min(100, pct))

    def _set_hw_volume(self, volume: float):
        """通过 pactl 设置 PulseAudio 音量"""
        pct = self._volume_to_percent(volume)
        try:
            #先解除可能存在的静音再调节应音量，确保音量调整生效。
            if pct > 0:
                subprocess.run(
                    ['pactl', 'set-sink-mute', '@DEFAULT_SINK@', '0'],
                    capture_output=True, timeout=2)
            subprocess.run(
                ['pactl', 'set-sink-volume', '@DEFAULT_SINK@', f'{pct}%'],
                capture_output=True, timeout=2)
            self.get_logger().info(f'PulseAudio 音量设置为 {pct}%')
        except Exception as e:
            self.get_logger().warn(f'pactl 音量调节失败: {e}')

    # ── ROS 服务处理器 ──────────────────────────────────────────

    def handle_music_action(self, request, response):
        """处理音乐操作控制请求"""
        action = request.action.lower()
        self.get_logger().info(f'收到音乐操作请求: action={action}')

        if not self.audio_available:
            response.success = False
            response.message = '音频系统不可用'
            response.current_file = ''
            return response

        try:
            if action.startswith('play_named:'):
                song = action.split(':', 1)[1].strip()
                response.success, response.message = self._play_music_by_name(song)
            elif action == 'play':
                response.success, response.message = self._play_music()
            elif action == 'pause':
                response.success, response.message = self._pause_music()
            elif action == 'resume':
                response.success, response.message = self._resume_music()
            elif action == 'stop':
                response.success, response.message = self._stop_music()
            elif action == 'next':
                response.success, response.message = self._next_track()
            elif action == 'previous':
                response.success, response.message = self._previous_track()
            elif action == 'user_pause':
                # 用户主动暂停：直接 SIGSTOP，完全不经过 TTS 计数器，与 TTS 自动暂停隔离
                response.success, response.message = self._user_pause_music()
            elif action == 'user_resume':
                # 用户主动恢复：直接 SIGCONT，完全不经过 TTS 计数器，与 TTS 自动恢复隔离
                response.success, response.message = self._user_resume_music()
            elif action == 'replay':
                response.success, response.message = self._replay_music()
            else:
                response.success = False
                response.message = f'未知操作: {action}'

            if self.current_index >= 0 and self.playlist:
                full_filename = os.path.basename(self.playlist[self.current_index])
                response.current_file = os.path.splitext(full_filename)[0]
            else:
                response.current_file = ''

        except Exception as e:
            self.get_logger().error(f'音乐操作失败: {e}')
            response.success = False
            response.message = f'执行失败: {str(e)}'

        return response

    def handle_music_volume(self, request, response):
        """处理音乐音量控制请求"""
        volume = request.volume
        self.get_logger().info(f'收到音量调整请求: volume={volume}')

        if not self.audio_available:
            response.success = False
            response.message = '音频系统不可用'
            response.current_volume = 0.0
            return response

        try:
            vol = float(volume)
            if 0.0 < vol <= 1.0:
                vol *= 100.0
            vol = max(0.0, min(100.0, vol))
            self.current_volume = vol
            self._set_hw_volume(vol)

            response.success = True
            response.message = f'音量已调整为: {self.current_volume:.2f}'
            response.current_volume = self.current_volume

        except Exception as e:
            self.get_logger().error(f'音量调整失败: {e}')
            response.success = False
            response.message = f'音量调整失败: {str(e)}'
            response.current_volume = self.current_volume

        return response

    def handle_get_status(self, request, response):
        """处理获取音乐状态请求"""
        try:
            if self.is_playing:
                response.status = "playing"
            elif self.is_paused:
                response.status = "paused"
            else:
                response.status = "stopped"

            response.volume = self.current_volume

            if self.current_index >= 0 and self.playlist:
                full_filename = os.path.basename(self.playlist[self.current_index])
                response.filename = os.path.splitext(full_filename)[0]
            else:
                response.filename = ""

            response.success = True
            response.message = f'状态: {response.status}, 音量: {response.volume:.2f}, 文件: {response.filename}'

        except Exception as e:
            self.get_logger().error(f'获取状态失败: {e}')
            response.status = "stopped"
            response.volume = 0.0
            response.filename = ""
            response.success = False
            response.message = f'获取状态失败: {str(e)}'

        return response

    # ── 播放操作 ────────────────────────────────────────────────

    def _find_track_index(self, name: str) -> int:
        """按歌名或文件名片段在播放列表中查找曲目（支持「歌手 歌名」「歌手的歌名」）。"""
        raw = str(name or '').strip()
        if not raw or not self.playlist:
            return -1
        needles: list[str] = []
        s = raw.lower()
        if '的' in s:
            tail = s.split('的')[-1].strip()
            if len(tail) >= 2:
                needles.append(tail)
        needles.append(s)
        if '的' in s:
            parts = [p.strip() for p in s.replace('的', ' ').split() if p.strip()]
            needles.extend(parts)
        for token in s.replace('的', ' ').split():
            if len(token) >= 2:
                needles.append(token)
        seen: set[str] = set()
        best_idx = -1
        best_score = 0
        for needle in needles:
            if needle in seen:
                continue
            seen.add(needle)
            for i, path in enumerate(self.playlist):
                base = os.path.splitext(os.path.basename(path))[0].lower()
                if needle in base or base in needle:
                    score = len(needle) if needle in base else len(base)
                    if score > best_score:
                        best_score = score
                        best_idx = i
        return best_idx

    def _play_music_by_name(self, name: str) -> tuple:
        """语音点歌：按歌名片段匹配本地文件后播放。"""
        if not self.playlist:
            return False, '播放列表为空，请添加音乐文件'
        idx = self._find_track_index(name)
        if idx < 0:
            return False, f'本地曲库未找到：{name.strip()}'
        return self._play_music(target_index=idx)

    def _play_music(self, target_index: int = -1) -> tuple:
        """播放音乐（随机或指定索引）。"""
        if not self.playlist:
            return False, '播放列表为空，请添加音乐文件'

        # 新一轮播放开始，重置 TTS 暂停计数器，清除上次对话会话遗留的计数（防止泄漏积累后
        # 导致下次 _pause_music 的 count==1 条件永远无法触发从而无法 SIGSTOP）
        with self._tts_lock:
            self._tts_pause_count = 0
            self._was_playing_before_tts = False

        with self._play_lock:
            if target_index < 0:
                self.current_index = random.randint(0, len(self.playlist) - 1)
            else:
                self.current_index = target_index % len(self.playlist)

            music_file = self.playlist[self.current_index]
            filename = os.path.basename(music_file)
            filename_without_ext = os.path.splitext(filename)[0]

            if self._start_playback(music_file):
                self.is_playing = True
                self.is_paused = False
                self.get_logger().info(f'正在播放: {filename}')
                return True, f'正在播放音乐：{filename_without_ext}'
            else:
                return False, f'播放失败: {filename_without_ext}'

    def _pause_music(self) -> tuple:
        """暂停音乐 - 发送 SIGSTOP 冻结 ffmpeg/paplay，保留播放进度，TTS 结束后 SIGCONT 续播"""
        do_pause = False
        with self._tts_lock:
            self._tts_pause_count += 1
            if self._tts_pause_count == 1:
                # 记录 TTS 开始前是否正在正常播放（排除已暂停状态，防止重复 SIGSTOP）
                self._was_playing_before_tts = self.is_playing and not self.is_paused
                if self._was_playing_before_tts:
                    # 同步清 is_playing，防止 is_playing=True 与 is_paused=True 并存，
                    # 否则 handle_get_status 会因 is_playing 优先而永远返回 'playing' 而非 'paused'
                    self.is_playing = False
                    self.is_paused = True
                    do_pause = True

        if do_pause:
            # SIGSTOP：进程冻结，进度保留；TTS 结束后 SIGCONT 从原位续播
            self._pause_playback()
            self.get_logger().info('音乐已暂停 (SIGSTOP，进度保留)')
        return True, '音乐已暂停'

    def _resume_music(self) -> tuple:
        """恢复音乐 - 只有所有 TTS 结束（计数器=0）且 TTS 前确实在播放才 SIGCONT 续播"""
        with self._tts_lock:
            if self._tts_pause_count > 0:
                self._tts_pause_count -= 1
            remaining = self._tts_pause_count

        self.get_logger().info(f'TTS 暂停计数器剩余: {remaining}')

        if remaining > 0:
            return False, f'还有 {remaining} 条 TTS 待结束，暂不恢复音乐'

        # TTS 前音乐未在播放，不启动播放，也不修改 is_paused。
        # 注意：不能清 is_paused，因为可能是用户主动暂停（user_pause）造成的 paused 状态，
        # 若此处强制清零，用户后续的 user_resume 调用会因 is_paused=False 而失效。
        if not self._was_playing_before_tts:
            return False, '音乐播放前未在播放，无需恢复'

        if not self.is_paused:
            return False, '音乐未处于暂停状态'

        # SIGCONT：从 SIGSTOP 冻结的位置续播，无需重启进程也无需从头开始
        # 安全检查：若进程在 SIGSTOP 期间意外死亡，则降级为重新播放
        if self._is_playback_active():
            self._resume_playback()
            self.is_playing = True   # 与 _pause_music 里 is_playing=False 配对，恢复正确状态
            self.is_paused = False
            self._was_playing_before_tts = False
        else:
            # 进程意外死亡（罕见），降级为重启播放
            self.get_logger().warn('恢复时发现播放进程已死亡，降级为重新播放')
            self.is_paused = False
            self._was_playing_before_tts = False
            if not self.playlist or self.current_index < 0:
                self.is_playing = False
                return False, '播放列表为空，无法恢复'
            music_file = self.playlist[self.current_index]
            if self._start_playback(music_file):
                self.is_playing = True
            else:
                self.is_playing = False
                return False, '恢复播放失败'

        if self.current_index >= 0 and self.playlist:
            filename = os.path.basename(self.playlist[self.current_index])
            filename_without_ext = os.path.splitext(filename)[0]
            self.get_logger().info(f'音乐已恢复: {filename}')
            return True, f'音乐已恢复：{filename_without_ext}'
        else:
            return True, '音乐已恢复'

    def _stop_music(self) -> tuple:
        """停止音乐"""
        self._kill_playback()
        self.is_playing = False
        self.is_paused = False
        self.current_index = -1
        # 重置 TTS 计数器，防止旧计数携带到下次播放会话导致 SIGSTOP 失效
        with self._tts_lock:
            self._tts_pause_count = 0
            self._was_playing_before_tts = False
        self.get_logger().info('音乐已停止')
        return True, '音乐已停止'

    def _user_pause_music(self) -> tuple:
        """用户主动暂停音乐 - 直接 SIGSTOP，完全不触碰 TTS 计数器。

        与 _pause_music（TTS 自动暂停路径）严格隔离：
          - _pause_music  专用于 TTS 播报时的自动静音，使用计数器管理并发
          - _user_pause_music 专用于用户语音命令，绕过计数器，避免产生无法配对的 +1

        特殊情况处理：TTS 播报可能已提前 SIGSTOP 音乐，
        因此 user_pause 到达时 is_paused 可能已经为 True。
        此时需要把 _was_playing_before_tts 清零，阻止 TTS 结束后自动 SIGCONT 恢复音乐，
        否则用户的暂停意图会被 TTS 的 resume 路径覆盖掉。
        """
        if self.is_paused:
            # 音乐已被 TTS 提前 SIGSTOP（_early_pause_proc 争跑先到）。
            # 清除 _was_playing_before_tts，使 _resume_music 的 SIGCONT 分支不再触发，
            # 保证 TTS 结束后音乐不会自动恢复，从而遵从用户的明确暂停意图。
            with self._tts_lock:
                self._was_playing_before_tts = False
            self.get_logger().info('用户主动暂停：覆盖 TTS 恢复标记，TTS 结束后不自动恢复音乐')
            return True, '音乐已暂停'
        if not self.is_playing:
            return False, '音乐当前未在播放，无需暂停'
        self._pause_playback()
        self.is_playing = False
        self.is_paused = True
        self.get_logger().info('用户主动暂停音乐 (SIGSTOP，进度保留)')
        return True, '音乐已暂停'

    def _replay_music(self) -> tuple:
        """从当前曲目开头重新播放（用户说「重新播放音乐」）。"""
        if not self.playlist:
            return False, '播放列表为空，请添加音乐文件'
        if self.current_index < 0:
            return self._play_music()
        self._kill_playback()
        with self._tts_lock:
            self._tts_pause_count = 0
            self._was_playing_before_tts = False
        self.is_paused = False
        return self._play_music(self.current_index)

    def _user_resume_music(self) -> tuple:
        """用户主动恢复音乐 - 直接 SIGCONT，完全不触碰 TTS 计数器。

        与 _resume_music（TTS 自动恢复路径）严格隔离，
        不依赖 _tts_pause_count 归零，不依赖 _was_playing_before_tts 标记。
        """
        if not self.is_paused:
            return False, '音乐未处于暂停状态'
        if self._is_playback_active():
            # 进程仍在（SIGSTOP 冻结中），SIGCONT 解冻续播
            self._resume_playback()
            self.is_playing = True
            self.is_paused = False
            self.get_logger().info('用户主动恢复音乐 (SIGCONT)')
        else:
            # 进程意外死亡（罕见），降级为重新播放当前曲目
            self.get_logger().warn('用户恢复时发现播放进程已死亡，降级为重新播放')
            self.is_paused = False
            if not self.playlist or self.current_index < 0:
                self.is_playing = False
                return False, '播放列表为空，无法恢复'
            music_file = self.playlist[self.current_index]
            if self._start_playback(music_file):
                self.is_playing = True
            else:
                self.is_playing = False
                return False, '恢复播放失败'
        if self.current_index >= 0 and self.playlist:
            filename_without_ext = os.path.splitext(
                os.path.basename(self.playlist[self.current_index]))[0]
            return True, f'音乐已恢复：{filename_without_ext}'
        return True, '音乐已恢复'

    def _next_track(self) -> tuple:
        """下一曲"""
        if not self.playlist:
            return False, '播放列表为空'
        next_idx = (self.current_index + 1) % len(self.playlist)
        return self._play_music(next_idx)

    def _previous_track(self) -> tuple:
        """上一曲"""
        if not self.playlist:
            return False, '播放列表为空'
        prev_idx = (self.current_index - 1) % len(self.playlist)
        return self._play_music(prev_idx)

    # ── 监控与 TTS 协同 ────────────────────────────────────────

    def _monitor_playback(self):
        """监控播放状态，自动切換下一曲；异常退出时退避重试"""
        consecutive_errors = 0
        while rclpy.ok():
            try:
                if self.is_playing and not self._is_playback_active():
                    # TTS 正在进行（计数器 > 0 或 is_paused），monitor 不介入
                    with self._tts_lock:
                        tts_active = self._tts_pause_count > 0
                    if tts_active or self.is_paused:
                        self.is_playing = False
                        time.sleep(1.0)
                        continue

                    # 获取 paplay 退出码判断是正常结束还是错误退出
                    aplay_rc = self._paplay_proc.returncode if self._paplay_proc is not None else 0

                    # 负数返回码 = 被信号杀死（SIGTERM/SIGKILL），视为正常停止
                    normal_exit = (aplay_rc == 0 or aplay_rc is None or aplay_rc < 0)

                    if not normal_exit:
                        # 设备忙或其他错误，指数退避后重试当前曲目
                        consecutive_errors += 1
                        wait_sec = min(consecutive_errors * 2, 10)
                        self.get_logger().warn(
                            f'播放异常退出 rc={aplay_rc}，{wait_sec}s 后重试 (第{consecutive_errors}次)')
                        self.is_playing = False
                        # 退避等待期间如发生 TTS，以 is_paused 为准，不再重试
                        _deadline = time.time() + wait_sec
                        while time.time() < _deadline:
                            time.sleep(0.5)
                            if self.is_paused:
                                self.get_logger().info('退避等待中检测到 TTS 暂停，取消本次重试')
                                consecutive_errors = 0
                                break
                        else:
                            if consecutive_errors <= 5 and self.current_index >= 0:
                                # 再次确认 TTS 未在进行
                                with self._tts_lock:
                                    tts_active = self._tts_pause_count > 0
                                if not tts_active and not self.is_paused:
                                    self._play_music(self.current_index)
                                else:
                                    self.get_logger().info('重试前检测到 TTS，放弃本次重试')
                                    consecutive_errors = 0
                            else:
                                self.get_logger().error('连续播放失败超过 5 次，音乐已停止')
                                self.is_paused = False
                                consecutive_errors = 0
                    else:
                        consecutive_errors = 0
                        self.get_logger().info('当前曲目播放完毕，切换下一曲')
                        next_idx = (self.current_index + 1) % len(self.playlist) if self.playlist else -1
                        if next_idx >= 0:
                            self._play_music(next_idx)
                        else:
                            self.is_playing = False
                else:
                    if self._is_playback_active():
                        consecutive_errors = 0  # 正在正常播放，重置错误计数
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'播放监控错误: {e}')
                time.sleep(5.0)

    # ── 状态上报 ────────────────────────────────────────────────

    def _publish_states(self):
        """每秒发布一次节点心跳状态到 /music_player/states。

        Header.stamp 由 ROS 时钟填充，供上层计算心跳超时；
        state_json 包含实时音乐数据：播放状态、当前文件名、音量。
        """
        msg = ModuleStates()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # 确定播放状态字符串
        if self.is_playing:
            status = 'playing'
        elif self.is_paused:
            status = 'paused'
        else:
            status = 'stopped'

        # 确定当前文件名（不含扩展名）
        if self.current_index >= 0 and self.playlist:
            full_filename = os.path.basename(self.playlist[self.current_index])
            filename = os.path.splitext(full_filename)[0]
        else:
            filename = ''

        msg.state_json = json.dumps({
            'status': status,
            'filename': filename,
            'volume': round(self.current_volume, 4)
        })
        self._states_pub.publish(msg)

    # ── 清理 ────────────────────────────────────────────────────

    def destroy_node(self):
        """节点清理"""
        print('正在关闭音乐播放节点...')
        
        # 1. 停止播放并设置停止标志
        self._stop_event.set()
        self._kill_playback()
        
        # 2. 等待监控线程退出
        if hasattr(self, '_monitor_thread') and self._monitor_thread.is_alive():
            print('等待播放监控线程退出...')
            self._monitor_thread.join(timeout=2.0)
            if self._monitor_thread.is_alive():
                print('⚠ 播放监控线程未能在超时时间内退出')
            else:
                print('播放监控线程已退出')
        
        super().destroy_node()


def main(args=None):
    # # 授权验证（宽松模式：模块不存在时允许运行）
    # try:
    #     from license_validator_py import check_system_license
    #     license_result = check_system_license()
    #     if not license_result.is_valid:
    #         print(f'[MusicPlayerNode] 授权验证失败: {license_result.message}')
    #         return
    # except ImportError:
    #     pass
    # except Exception as e:
    #     print(f'[MusicPlayerNode] 授权验证异常（已跳过）: {e}')

    rclpy.init(args=args)
    node = None

    try:
        node = MusicPlayerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭节点...")
    except Exception as e:
        print(f'节点运行错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception as e:
                print(f'节点销毁时出错: {e}')
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        print('音乐播放节点已关闭')


if __name__ == '__main__':
    main()
