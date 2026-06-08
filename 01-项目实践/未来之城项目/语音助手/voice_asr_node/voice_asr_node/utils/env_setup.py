#!/usr/bin/env python3
import functools
import os
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

# ═══════════════════════════════════════════════════════════════════
# 1. 模型路径自动解析（KWS）
# ═══════════════════════════════════════════════════════════════════

_KWS_ACTIVE_DIR = "kws/zh_en_kws"  # doubao_dialog_node KeywordSpotter（中英双语 KWS）


def _has_wake_models(base: str) -> bool:
    return os.path.isdir(os.path.join(base, _KWS_ACTIVE_DIR))


@functools.lru_cache(maxsize=1)
def _get_model_candidates() -> tuple:
    """返回模型搜索候选目录列表。结果缓存，仅计算一次。"""
    scripts_dir = os.path.dirname(os.path.realpath(__file__))
    home = os.path.expanduser('~')
    seen: set[str] = set()
    ordered: list[str] = []

    def _add(path: str) -> None:
        if not path:
            return
        abspath = os.path.abspath(path)
        if abspath not in seen:
            seen.add(abspath)
            ordered.append(abspath)

    _add(os.environ.get('VOICE_ASR_MODELS_DIR', ''))

    # ROS2 安装路径（colcon install 后最可靠）
    try:
        from ament_index_python.packages import get_package_share_directory

        _add(os.path.join(get_package_share_directory('voice_asr_node'), 'models'))
    except Exception:
        pass

    # 从当前模块向上遍历：匹配 share/.../models 或源码 src/voice_asr_node/models
    walk = scripts_dir
    for _ in range(10):
        _add(os.path.join(walk, 'share', 'voice_asr_node', 'models'))
        _add(os.path.join(walk, 'src', 'voice_asr_node', 'models'))
        _add(os.path.join(walk, 'models'))
        parent = os.path.dirname(walk)
        if parent == walk:
            break
        walk = parent

    # 开发时包内 models（pip install -e 或 PYTHONPATH 指向源码）
    _add(os.path.join(scripts_dir, '..', 'models'))

    _add(os.path.join(home, 'voice_asr', 'models'))
    _add(os.path.join(home, 'Documents', 'voice_asr', 'models'))

    return tuple(ordered)


def get_models_root() -> str:
    """返回 voice_asr_node models 根目录（含 kws/zh_en_kws 等）。"""
    for base in _get_model_candidates():
        if _has_wake_models(base):
            return os.path.abspath(base)
    for base in _get_model_candidates():
        if os.path.isdir(base):
            return os.path.abspath(base)
    raise FileNotFoundError(f"未找到 models 根目录。已搜索: {_get_model_candidates()}")


# ═══════════════════════════════════════════════════════════════════
# 2. PulseAudio 音频环境初始化（doubao_dialog_node 启动时调用）
# ═══════════════════════════════════════════════════════════════════

def _run_cmd(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout or '', r.stderr or ''
    except subprocess.TimeoutExpired:
        return -1, '', 'timeout'
    except FileNotFoundError:
        return -1, '', f'command not found: {cmd[0]}'
    except Exception as e:
        return -1, '', str(e)


def _setup_pulse_env():
    """设置 PulseAudio 客户端所需的环境变量。"""
    uid = os.getuid()
    os.environ.setdefault('XDG_RUNTIME_DIR', f'/run/user/{uid}')
    os.environ.setdefault('DBUS_SESSION_BUS_ADDRESS', f'unix:path=/run/user/{uid}/bus')


def _check_and_restart_pulse() -> bool:
    """检查 PulseAudio 连通性，不可达时尝试重启。返回最终是否可达。"""
    rc, _, _ = _run_cmd(['pactl', 'info'], timeout=2)
    if rc == 0:
        print('🔊 [PulseAudio] 服务正常')
        return True

    print('🔄 [PulseAudio] 不可达，尝试重启...')
    _run_cmd(['pulseaudio', '--kill'], timeout=2)
    time.sleep(0.2)  # 缩短等待：kill 后 0.2s 即可重启
    try:
        subprocess.Popen(
            ['pulseaudio', '--start', '--exit-idle-time=-1'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f'⚠️  [PulseAudio] 启动失败: {e}')
        return False
    time.sleep(0.5)  # 缩短等待：0.5s 足够 PulseAudio 就绪

    rc, _, _ = _run_cmd(['pactl', 'info'], timeout=2)
    if rc == 0:
        print('✅ [PulseAudio] 重启成功')
        return True
    print('⚠️  [PulseAudio] 仍不可达，录音将使用 ALSA 直连模式')
    return False


def _setup_usb_audio(initial_volume_pct: int = 50):
    """检测 USB 声卡并激活 Profile，设置默认源/接收器和音量。

    优化：source/sink 查询并行执行，减少 ~200ms。
    """
    rc, stdout, _ = _run_cmd(['pactl', 'list', 'cards', 'short'])
    if rc != 0:
        _set_volume(initial_volume_pct)
        return

    usb_card = ''
    for line in stdout.splitlines():
        if 'usb' in line.lower():
            parts = line.split()
            if len(parts) >= 2:
                usb_card = parts[1]
                break

    if not usb_card:
        print('⚠️  [Audio] 未找到 USB 声卡，使用系统默认设备')
    else:
        print(f'🔌 [Audio] USB 声卡: {usb_card}')
        # 设置全双工 Profile
        for profile in ['output:analog-stereo+input:analog-stereo', 'input:analog-stereo']:
            rc, _, _ = _run_cmd(['pactl', 'set-card-profile', usb_card, profile])
            if rc == 0:
                print(f'  ✅ profile → {profile}')
                break
        else:
            print('  ⚠️  profile 设置失败')

        # 并行查询 source 和 sink（省去串行等待 + 移除多余 sleep）
        def _detect_source():
            _rc, _out, _ = _run_cmd(['pactl', 'list', 'sources', 'short'])
            if _rc != 0:
                return
            for _line in _out.splitlines():
                if 'monitor' not in _line.lower() and 'usb' in _line.lower():
                    _src = _line.split()[1] if len(_line.split()) >= 2 else ''
                    if _src:
                        _run_cmd(['pactl', 'set-default-source', _src])
                        print(f'✅ [Audio] USB 录音源: {_src}')
                    break

        def _detect_sink():
            _rc, _out, _ = _run_cmd(['pactl', 'list', 'sinks', 'short'])
            if _rc != 0:
                return
            for _line in _out.splitlines():
                if 'monitor' not in _line.lower() and 'usb' in _line.lower():
                    _sink = _line.split()[1] if len(_line.split()) >= 2 else ''
                    if _sink:
                        _run_cmd(['pactl', 'set-default-sink', _sink])
                        print(f'✅ [Audio] USB 播放器: {_sink}')
                    break

        with ThreadPoolExecutor(max_workers=2) as _pa_pool:
            _src_f = _pa_pool.submit(_detect_source)
            _snk_f = _pa_pool.submit(_detect_sink)
            _src_f.result()
            _snk_f.result()

    _set_volume(initial_volume_pct)


def _set_volume(pct: int):
    pct = max(0, min(100, pct))
    #先解除可能存在的静音再调节应音量，确保音量调整生效。
    if pct > 0:
        _run_cmd(['pactl', 'set-sink-mute', '@DEFAULT_SINK@', '0'])
    _run_cmd(['pactl', 'set-sink-volume', '@DEFAULT_SINK@', f'{pct}%'])


def set_default_sink(sink_name: str) -> bool:
    """将指定 PulseAudio sink 设为默认输出。"""
    sink = str(sink_name or "").strip()
    if not sink:
        return False
    rc, _, err = _run_cmd(['pactl', 'set-default-sink', sink])
    if rc == 0:
        print(f'✅ [Audio] 默认播放器: {sink}')
        return True
    print(f'⚠️  [Audio] set-default-sink 失败: {err.strip() or sink}')
    return False


def setup_audio(initial_volume_pct: int = 50, *, pulse_sink: str = "") -> bool:
    """一键初始化音频环境。返回 PulseAudio 是否可达。"""
    print('\n── 音频环境初始化 ──────────────────────────────────')
    _setup_pulse_env()
    pa_ok = _check_and_restart_pulse()
    if pa_ok:
        _setup_usb_audio(initial_volume_pct)
        if pulse_sink:
            set_default_sink(pulse_sink)
    print('── 音频初始化完成 ──────────────────────────────────\n')
    return pa_ok


# ═══════════════════════════════════════════════════════════════════
# 3. KWS 中文唤醒词拼音转换
# ═══════════════════════════════════════════════════════════════════

# ── pypinyin 模块缓存 ─────────────────────────────────────────────
_pypinyin_pinyin = None
_pypinyin_Style = None
_pypinyin_checked = False


def _ensure_pypinyin():
    """懒加载并缓存 pypinyin，避免每次 chinese_to_kws_line 重复 import。"""
    global _pypinyin_pinyin, _pypinyin_Style, _pypinyin_checked
    if _pypinyin_checked:
        return _pypinyin_pinyin is not None
    try:
        from pypinyin import pinyin, Style
        _pypinyin_pinyin = pinyin
        _pypinyin_Style = Style
    except ImportError:
        print('⚠️  [KWS] pypinyin 未安装，无法自动转换中文唤醒词')
        print('        请运行: pip install pypinyin')
    _pypinyin_checked = True
    return _pypinyin_pinyin is not None


def chinese_to_kws_line(text: str) -> str | None:
    """
    将纯中文唤醒词自动转换为 KWS 模型所需的带声调拼音格式。

    转换规则（与 tokens.txt phone+ppinyin 格式精确匹配）：
      - 每个汉字拆为 声母(INITIALS) + 韵母(FINALS_TONE)
      - 零声母字（如 "安" "二"）只输出韵母
      - 所有 token 以空格分隔，末尾追加 @原文标签

    示例：
      "你好小未" → "n ǐ h ǎo x iǎo w èi @你好小未"
      "安全"     → "ān q üán @安全"

    Args:
        text: 纯中文字符串（2~8个汉字）

    Returns:
        KWS 格式行字符串，转换失败返回 None
    """
    if not _ensure_pypinyin():
        return None

    text = text.strip()
    if not text:
        return None

    pinyin = _pypinyin_pinyin
    Style = _pypinyin_Style
    tokens = []
    for char in text:
        # 跳过非中文字符（空格、标点等）
        if not ('\u4e00' <= char <= '\u9fff'):
            continue
        ini = pinyin(char, style=Style.INITIALS, strict=False)[0][0]
        fin = pinyin(char, style=Style.FINALS_TONE, strict=False)[0][0]
        if ini:
            tokens.append(ini)
        if fin:
            tokens.append(fin)

    if not tokens:
        return None

    return ' '.join(tokens) + f' @{text}'
