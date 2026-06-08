"""
voice_asr.launch.py

启动语音完整节点栈（豆包端到端实时语音）：
    1) voice_asr_node      —— 业务事件 -> /voice_asr/tts_speak
    2) doubao_dialog_node  —— KWS + 豆包 Realtime WS + 音频上下行
    3) music_player_node   —— 背景音乐管理

默认工作模式：
    - 本地 KWS 唤醒
    - server_vad 自动判停（不使用 push_to_talk）
    - 联网搜索默认关闭（可在 config/doubao.yaml 后续开启）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PKG_SHARE = get_package_share_directory('voice_asr_node')
_WS_ROOT = os.path.abspath(os.path.join(_PKG_SHARE, '..', '..', '..', '..'))

# PulseAudio 环境变量默认值（确保子进程能连接 PulseAudio）
_UID = str(os.getuid())
_XDG_RUNTIME_DIR = os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{_UID}')
_DBUS_BUS_ADDR = os.environ.get(
    'DBUS_SESSION_BUS_ADDRESS', f'unix:path=/run/user/{_UID}/bus')


def _resolve_default_config_paths():
    config_dir = os.environ.get('MR_CONFIG_DIR')
    if not config_dir:
        raise RuntimeError('未设置环境变量 MR_CONFIG_DIR，无法加载语音配置文件')
    if not os.path.exists(config_dir):
        raise RuntimeError(f'MR_CONFIG_DIR 指向目录不存在: {config_dir}')
    doubao = os.path.join(config_dir, 'doubao.yaml')
    command = os.path.join(config_dir, 'command.yaml')
    print(f'使用环境变量配置目录: {config_dir}')
    for label, path in (('doubao.yaml', doubao), ('command.yaml', command)):
        if not os.path.exists(path):
            raise RuntimeError(f'未找到 {label}: {path}')
        print(f'✓ {label} 已找到: {path}')
    return doubao, command


def generate_launch_description():

    _doubao_default, _command_default = _resolve_default_config_paths()

    # ── 可覆盖参数声明 ────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'doubao_config',
            default_value=_doubao_default,
            description='豆包配置文件路径'),
        DeclareLaunchArgument(
            'command_config',
            default_value=_command_default,
            description='命令配置文件路径，默认读取 MR_CONFIG_DIR/command.yaml'),
        DeclareLaunchArgument(
            'use_refactor_node',
            default_value='false',
            description='是否启用重构版 voicebot_session_node（灰度开关）'),
        DeclareLaunchArgument(
            'mic_device', default_value='',
            description='麦克风设备（空=系统默认）'),
        DeclareLaunchArgument(
            'playback_device', default_value='',
            description='播放设备（空=系统默认）'),
        DeclareLaunchArgument(
            'pulse_sink', default_value='',
            description='PulseAudio 输出设备名称（空=使用系统默认 sink）'),
        DeclareLaunchArgument(
            'kws_threshold', default_value='0.15',
            description='KWS 唤醒灵敏度（越低越敏感）'),
        DeclareLaunchArgument(
            'kws_input_gain_db', default_value='6.0',
            description='KWS 输入增益(dB)，用于提升弱语音唤醒概率'),
        DeclareLaunchArgument(
            'music_root', default_value='',
            description='音乐文件根目录，默认使用 MR_RESOURCE_DIR/music'),
        DeclareLaunchArgument(
            'music_default_volume', default_value='0.5',
            description='音乐默认音量 0.0~1.0'),
        DeclareLaunchArgument(
            'music_fade_duration_ms', default_value='2000',
            description='音乐淡入淡出时长（毫秒）'),
        DeclareLaunchArgument(
            'initial_volume', default_value='50',
            description='初始 PulseAudio 音量百分比 (0-100)'),
        DeclareLaunchArgument(
            'kws_debug_audio_level', default_value='false',
            description='KWS 调试模式：每 2s 打印麦克风输入电平 rms/dBFS'),
    ]

    # ── 环境变量（确保所有子进程能连接 PulseAudio）──────────────
    env_actions = [
        SetEnvironmentVariable('XDG_RUNTIME_DIR', _XDG_RUNTIME_DIR),
        SetEnvironmentVariable('DBUS_SESSION_BUS_ADDRESS', _DBUS_BUS_ADDR),
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
    ]

    # ── Python 主节点：业务事件订阅 + 机械臂服务调用 ───────────────
    voice_asr_node = Node(
        package='voice_asr_node',
        executable='voice_asr_node',
        name='voice_asr_node',
        output='both',
        parameters=[{
            'tts_speak_topic':          '/voice_asr/tts_speak',
            'asr_command_topic':        '/voice_asr/asr_command',
            'massage_biz_topic':        'massage_biz_event',
            'massage_err_topic':        'massage_error_biz_event',
            'voice_announcement_topic': 'voice_announcement',
        }],
        # 设置 PYTHONUNBUFFERED=1，强制 Python 对 stdout/stderr 完全不缓冲
        # 每条 print/log 立即写入，即使程序意外死掉也不丢日志
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    # ── 豆包实时对话节点：KWS + WS + 音频上下行 ─────────────────
    doubao_dialog_node = Node(
        package='voice_asr_node',
        executable='doubao_dialog_node',
        name='doubao_dialog_node',
        output='both',
        condition=UnlessCondition(LaunchConfiguration('use_refactor_node')),
        parameters=[{
            'doubao_config':        LaunchConfiguration('doubao_config'),
            'command_config':       LaunchConfiguration('command_config'),
            'mic_device':           LaunchConfiguration('mic_device'),
            'playback_device':      LaunchConfiguration('playback_device'),
            'kws_threshold':        LaunchConfiguration('kws_threshold'),
            'kws_input_gain_db':    LaunchConfiguration('kws_input_gain_db'),
            'initial_volume':       LaunchConfiguration('initial_volume'),
            'kws_debug_audio_level': LaunchConfiguration('kws_debug_audio_level'),
        }],
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    # ── 重构版会话节点（灰度）────────────────────────────────────
    voicebot_session_node = Node(
        package='voice_asr_node',
        executable='voicebot_session_node',
        name='voicebot_session_node',
        output='both',
        condition=IfCondition(LaunchConfiguration('use_refactor_node')),
        parameters=[{
            'doubao_config': LaunchConfiguration('doubao_config'),
            'command_config': LaunchConfiguration('command_config'),
            'debug_input_topic': '/voice_asr/debug_text_input',
        }],
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    # ── 音乐播放节点：背景音乐管理 + TTS 协同 ─────────────────────
    _mr_resource_dir = os.environ.get('MR_RESOURCE_DIR', '')
    if _mr_resource_dir:
        _music_root_env = os.path.join(_mr_resource_dir, 'music')
    else:
        _music_root_env = os.path.join(_WS_ROOT, 'resource', 'music')

    music_player_node = Node(
        package='voice_asr_node',
        executable='music_player_node',
        name='music_player_node',
        output='both',
        parameters=[{
            'music_root':             _music_root_env,
            'default_volume':         LaunchConfiguration('music_default_volume'),
            'fade_duration_ms':       LaunchConfiguration('music_fade_duration_ms'),
            'pulse_sink':             LaunchConfiguration('pulse_sink'),
        }],
        additional_env={'PYTHONUNBUFFERED': '1'},
    )

    return LaunchDescription([
        *args,
        *env_actions,
        LogInfo(msg='[voice_asr] 启动 doubao_dialog_node + voice_asr_node + music_player_node ...'),
        doubao_dialog_node,
        voicebot_session_node,
        voice_asr_node,
        music_player_node,
    ])
