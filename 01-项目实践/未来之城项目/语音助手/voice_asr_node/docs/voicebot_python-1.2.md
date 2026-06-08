# VoiceBot 整体框架设计文档（Python 版）

> 基于豆包端到端实时语音大模型 RealtimeAPI（O2.0 / SC2.0），**Python 工程框架**
> 面向具身智能 / 机器人场景，支持唤醒词触发、多语言、声纹识别、命令词控制、ROS2 接口

---

## 目录

1. [产品定位与核心能力](#1-产品定位与核心能力)
2. [整体架构图](#2-整体架构图)
3. [模块划分与文件结构](#3-模块划分与文件结构)
4. [配置文件设计（config.yaml）](#4-配置文件设计)
5. [核心流程设计](#5-核心流程设计)
6. [命令词控制体系](#6-命令词控制体系)
7. [各模块详细说明](#7-各模块详细说明)
8. [豆包 API 关键机制挖掘与建议](#8-豆包-api-关键机制挖掘与建议)
9. [依赖与环境说明](#9-依赖与环境说明)
10. [后续迭代路线图](#10-后续迭代路线图)

---

## 1. 产品定位与核心能力

### 1.1 定位

VoiceBot 是一个运行在机器人嵌入式主机（ARM/x86 Ubuntu）上的 **Python 语音助手中间件**，连接：

```
物理世界（麦克风/扬声器）
        ↕
VoiceBot 中间件（Python 3.10+）
        ↕
豆包 RealtimeAPI（WebSocket 流式）
        ↕
ROS2 服务 / 机械臂控制层
```

**Python 方案优势**：
- 生态丰富：KWS 小模型（Porcupine、OpenWakeWord、Sherpa-ONNX KWS）均有原生 Python 绑定，`pyaudio`、`websockets`、`onnxruntime` 同样完备
- 异步友好：`asyncio` + `websockets` 天然适配 WebSocket 流式场景
- ROS2 集成：`rclpy` 是 ROS2 官方 Python 客户端，接口与 C++ 对等
- 快速迭代：原型验证到生产部署比 C++ 更短路径

### 1.2 核心能力矩阵

| 能力模块 | 实现方案 | Python 库 | 备注 |
|--------|--------|---------|------|
| 唤醒词检测 | 多 KWS 小模型插件化选配（Porcupine / OpenWakeWord / Sherpa-ONNX KWS / 自定义） | `pvporcupine` / `openwakeword` / `sherpa_onnx` | yaml 配置选型，运行时并联 |
| 多语言 ASR | 豆包端到端模型（流式实时） | `websockets` + 二进制协议 | 中文、方言、英/日/韩 |
| 语音合成 TTS | 豆包端到端模型（流式输出） | `asyncio` 事件驱动 | PCM S16LE / OGG Opus |
| 声纹识别 | 本地 ECAPA-TDNN Embedding | `sherpa-onnx`（SpeakerEmbeddingExtractor） | 识别说话人身份；详见 [speaker_id.md](./speaker_id.md)（与豆包克隆音色 `S_xxx` 不同） |
| 角色扮演 | SC2.0 character_manifest | WebSocket JSON payload | 可运行时切换 |
| 声音复刻 | SC2.0 saturn_ 系列克隆音色 | REST API + `httpx` | 需提前注册 |
| 联网搜索 | enable_volc_websearch + location | StartSession payload | 精准位置感知 |
| 唱歌 | enable_music（O2.0 1.2.1.1） | StartSession payload | 版本约束 |
| 命令词控制 | ChatResponse 文本流 + 规则引擎 | `re`, `asyncio.Queue` | 触发 ROS2 服务 |
| 静默超时 | asyncio.wait_for + 事件标志 | `asyncio.Event` | 可配置秒数 |
| 上下文管理 | ConversationCreate/Update/Delete | WebSocket 事件 | 最多 20 轮 QA |

---

## 2. 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                    VoiceBot 进程（Python asyncio）                 │
│                                                                  │
│  ┌──────────────┐   ┌────────────────┐   ┌──────────────────┐   │
│  │ 音频采集      │──▶│  唤醒词检测     │──▶│  声纹识别模块     │   │
│  │ AudioCapture │   │  KWSManager    │   │  SpeakerID       │   │
│  │ (pyaudio)    │   │  (插件化多模型) │   │  (onnxruntime)   │   │
│  └──────────────┘   └────────────────┘   └────────┬─────────┘   │
│        ▲                                           │             │
│        │            ┌──────────────────────────────▼──────────┐  │
│        │            │       会话管理器 SessionManager           │  │
│        │            │   - asyncio 状态机                        │  │
│        │            │   (IDLE/WAKEUP/DIALOG/CMD_EXEC/SLEEPING) │  │
│        │            └──────────────┬────────────┬─────────────┘  │
│        │                           │            │                │
│  ┌─────┴───────┐  ┌────────────────▼──┐  ┌─────▼────────────┐   │
│  │ 音频播放     │◀─│  WebSocket 客户端  │  │  命令词引擎       │   │
│  │ AudioPlayer │  │  DoubaoWSClient   │  │  CmdEngine       │   │
│  │ (subprocess │  │  (websockets lib) │  │  (re + asyncio)  │   │
│  │  + paplay)  │  └────────┬──────────┘  └──────┬───────────┘   │
│  └─────────────┘           │                    │               │
│                        豆包 API             ┌───▼──────────┐    │
│                       RealtimeAPI           │  ROS2 桥接    │    │
│                      (wss://...)            │  ROS2Bridge  │    │
│                                             │  (rclpy)     │    │
│                                             └──────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### 异步并发模型

```
主事件循环（asyncio.get_event_loop）
    ├── Task: SessionManager.run()           ← 状态机主控
    ├── Task: DoubaoWSClient._recv_loop()    ← WebSocket 接收
    ├── Task: audio_capture_loop()           ← 麦克风采集（线程池）
    ├── Task: KWSManager.run_all()           ← 多 KWS 模型并联（线程池）
    └── Task: SilenceDetector.run()         ← 超时检测

线程池（asyncio.to_thread / ThreadPoolExecutor）
    ├── pyaudio 采集回调（非 async，需 bridge 到 asyncio）
    ├── KWS 模型推理（CPU 密集，各模型独立线程，避免阻塞事件循环）
    └── ONNX 声纹推理
```

### 状态机

```
         唤醒词命中
IDLE ─────────────────▶ WAKEUP
  ▲                        │ 播放"我在" + 建立 WS Session
  │                        ▼
  │  静默超时/退出意图   DIALOG ◀──────────────┐
  │◀────────────────────   │  流式音频上传/接收  │
  │                        │                   │
  │                        ▼ 命令词命中          │
  │                     CMD_EXEC               │
  │                        │ ROS2 执行中         │
  │                        │ 执行完毕            │
  │◀────────────────────────────────────────────┘
```

---

## 3. 模块划分与文件结构

```
voicebot/
├── requirements.txt              # Python 依赖列表
├── setup.py                      # 可选：打包安装
├── config/
│   ├── config.yaml               # 全局配置文件（核心）
│   └── commands.yaml             # 命令词注册表
├── voicebot/
│   ├── __init__.py
│   ├── main.py                   # 程序入口：加载配置、构建依赖、启动 asyncio 事件循环
│   │
│   ├── config/
│   │   ├── __init__.py
│   │   └── app_config.py         # 配置数据类（dataclass）+ YAML 解析（PyYAML/ruamel）
│   │
│   ├── audio/
│   │   ├── __init__.py
│   │   ├── audio_capture.py      # 麦克风采集（pyaudio，回调桥接到 asyncio.Queue）
│   │   ├── audio_player.py       # 扬声器播放（subprocess paplay + stdin 管道）
│   │   └── audio_buffer.py       # asyncio.Queue 封装的线程安全音频缓冲区
│   │
│   ├── kws/
│   │   ├── __init__.py
│   │   ├── kws_manager.py        # KWS 管理器：读取配置、实例化插件、并联运行、统一唤醒出口
│   │   ├── base_kws.py           # 抽象基类 BaseKWS，定义插件接口规范
│   │   ├── porcupine_kws.py      # 插件：Picovoice Porcupine（商业，极低功耗，内置词）
│   │   ├── openwakeword_kws.py   # 插件：OpenWakeWord（开源，自定义模型，ONNX 推理）
│   │   ├── sherpa_kws.py         # 插件：Sherpa-ONNX KWS（开源，关键词 CTC 解码）
│   │   └── custom_onnx_kws.py   # 插件：通用自定义 ONNX KWS（用户自训练模型）
│   │
│   ├── speaker/
│   │   ├── __init__.py
│   │   └── speaker_id.py         # 声纹识别（onnxruntime ECAPA-TDNN）
│   │
│   ├── websocket/
│   │   ├── __init__.py
│   │   ├── doubao_ws_client.py   # 豆包 WebSocket 客户端（websockets 库）
│   │   ├── binary_protocol.py    # 二进制帧组装 / 解析（struct + bytes）
│   │   ├── event_dispatcher.py   # 服务端事件分发（事件码 → 回调）
│   │   └── session_config.py     # StartSession JSON 构建（dataclass + json）
│   │
│   ├── session/
│   │   ├── __init__.py
│   │   ├── session_manager.py    # 会话状态机（asyncio 驱动，协调所有模块）
│   │   ├── silence_detector.py   # 静默超时（asyncio.wait_for + Event）
│   │   └── dialog_context.py     # 上下文管理（ConversationCRUD）
│   │
│   ├── command/
│   │   ├── __init__.py
│   │   ├── cmd_engine.py         # 命令词匹配引擎（re 正则 + 关键词 + 意图标签）
│   │   ├── cmd_registry.py       # 命令注册表（YAML 驱动）
│   │   └── cmd_result.py         # 命令执行结果数据类
│   │
│   ├── ros2/
│   │   ├── __init__.py
│   │   ├── ros2_bridge.py        # ROS2 服务调用桥接（rclpy）
│   │   ├── arm_controller.py     # 机械臂控制（自定义 srv）
│   │   └── navigation_controller.py  # 导航控制
│   │
│   └── utils/
│       ├── __init__.py
│       ├── logger.py             # 日志模块（loguru 或 logging）
│       └── uuid_gen.py           # UUID 生成（uuid 标准库）
│
├── tests/
│   ├── test_protocol.py          # 二进制协议单元测试（pytest）
│   ├── test_cmd_engine.py        # 命令词引擎测试
│   ├── test_kws_manager.py       # KWS 管理器与插件加载测试
│   └── test_ws_client.py         # WS 连接集成测试（pytest-asyncio）
│
└── scripts/
    ├── register_clone_voice.py   # 克隆音色注册脚本（httpx）
    └── test_audio.py             # 音频链路测试
```

---

## 4. 配置文件设计

`config/config.yaml` 是系统唯一配置入口，结构与 C++ 版完全一致，由 `AppConfig` 数据类解析。

```yaml
# =============================================================
# VoiceBot 全局配置文件
# 场景预设：robot_assistant / home_companion / demo / debug
# =============================================================

scene: robot_assistant

auth:
  api_key: "1b9e271a-0a90-427e-a1d1-63317ea5844c"
  resource_id: "volc.speech.dialog"
  app_key: "PlgvMymc7f3tQnJ6"
  ws_url: "wss://openspeech.bytedance.com/api/v3/realtime/dialogue"
  connect_timeout_s: 5
  reconnect_max_retry: 3
  reconnect_delay_s: 1.0

model:
  version: "1.2.1.1"           # 1.2.1.1(O2.0) | 2.2.0.0(SC2.0)

persona:
  bot_name: "小未"
  system_role: |
    你是一个搭载在协作机械臂上的智能语音助手，名叫小未。
    你帮助操作员理解和执行操作指令，同时能与用户自然对话。
    当用户发出机械臂相关指令时，你需要简洁确认并执行。

    【指令标签协议 - 必须严格遵守】
    当用户意图匹配以下动作时，在回复文字末尾（句号之后）追加对应标签（仅追加一个，勿朗读标签）：
    - 用户想抓取/夹取/拿起物品     → 回复末尾追加 [CMD:arm_grab]
    - 用户想放置/放下/释放物品     → 回复末尾追加 [CMD:arm_release]
    - 用户想让机械臂归零/回初始位  → 回复末尾追加 [CMD:arm_home]
    - 用户说停止/急停/暂停/stop    → 回复末尾追加 [CMD:emergency_stop]
    - 用户想导航到某个位置（目标X）→ 回复末尾追加 [CMD:navigate_to:X]
    - 用户想听歌/放音乐/来首歌     → 回复末尾追加 [CMD:music_play]
    - 下一首/换一首/切歌           → 回复末尾追加 [CMD:music_next]
    - 上一首                       → 回复末尾追加 [CMD:music_previous]
    - 暂停背景音乐（非暂停按摩）   → 回复末尾追加 [CMD:music_pause]
    - 继续播放音乐                 → 回复末尾追加 [CMD:music_resume]
    - 停止/关闭音乐                → 回复末尾追加 [CMD:music_stop]
    - 重新播放当前曲               → 回复末尾追加 [CMD:music_replay]
    示例："好的，这就为你放一首轻音乐。[CMD:music_play]"

  speaking_style: |
    你说话简洁、专业、友好，像一个老练的工程师助手。
    偶尔使用"好的"、"收到"、"正在执行"这类回应。
    回复控制在2句话以内，不啰嗦。

  # 占位符：{time} {workstation} {arm_status}
  say_hello_prompt: "用户唤醒了我，现在是{time}，机械臂当前位于{workstation}，状态为{arm_status}。"
  character_manifest: ""

tts:
  speaker: "zh_male_yunzhou_jupiter_bigtts"
  clone_speaker_id: ""
  audio_format: "pcm_s16le"    # ogg_opus | pcm | pcm_s16le
  sample_rate: 24000
  channel: 1
  speech_rate: 0
  loudness_rate: 0
  enable_loudness_norm: false
  explicit_dialect: ""

asr:
  audio_format: "pcm"
  sample_rate: 16000
  channel: 1
  end_smooth_window_ms: 1200
  enable_custom_vad: true
  hotwords:
    - "机械臂"
    - "夹爪"
    - "导航"
    - "抓取"
    - "放置"
    - "回零"
  correct_words:
    "ros\\s*2": "ROS2"
    "机器臂": "机械臂"
  boosting_table_id: ""

input:
  mode: "microphone"            # microphone | push_to_talk | text | audio_file | keep_alive
  chunk_ms: 20

wakeword:
  enabled: true
  # ack_audio_path: 任意 KWS 模型命中后统一播放的唤醒确认音
  ack_audio_path: "assets/wakeup_ack.wav"
  ack_text: "我在，请说。"

  # ══════════════════════════════════════════════════════════════
  # KWS 插件列表（active_models）
  # 程序初始化时按顺序读取，enabled=true 的模型全部实例化并并联运行，
  # 任意一个模型命中配置的唤醒词即触发唤醒，其余立即暂停等待下轮。
  #
  # 支持的 type 值：
  #   porcupine      Picovoice Porcupine  商业 SDK，极低 CPU，内置/自定义唤醒词
  #   openwakeword   OpenWakeWord         开源，社区 ONNX 模型库，可自训练
  #   sherpa_kws     Sherpa-ONNX KWS     开源，CTC 解码，支持任意词表
  #   custom_onnx    用户自训练 ONNX 模型  通用接口，需指定输入/输出 tensor 名
  #
  # 选型建议：
  #   生产嵌入式（Jetson Nano / RPi 4B）  →  porcupine（最省 CPU）
  #   快速原型 / 中文场景                 →  openwakeword + sherpa_kws 并联
  #   需要高度定制词汇                    →  custom_onnx 或 sherpa_kws
  # ══════════════════════════════════════════════════════════════

  active_models:

    # ── Porcupine 插件 ────────────────────────────────────────────
    - name: "porcupine_zh"            # 实例别名（日志/调试用）
      type: "porcupine"
      enabled: true
      # Picovoice 控制台获取：https://console.picovoice.ai/
      access_key: "YOUR_PICOVOICE_ACCESS_KEY"
      # 内置唤醒词列表（英文）：alexa / hey_google / hey_siri / jarvis / ok_google / porcupine ...
      # 自定义中文唤醒词：在 Picovoice 控制台训练后下载 .ppn 文件，填入 keyword_paths
      keyword_paths:
        - "models/kws/porcupine/xiao_wei_zh.ppn"   # 自定义"小未"中文唤醒词
      # keyword_paths 为空时使用 built_in_keywords（英文内置词）
      built_in_keywords: []                           # e.g. ["porcupine"]
      # 灵敏度 [0.0, 1.0]，值越大越灵敏（误唤醒率也越高），每个 keyword 对应一个值
      sensitivities: [0.6]
      # 推理设备：cpu（默认）
      device: "cpu"
      # 内存参考：< 2MB，CPU 占用 < 1%（ARM Cortex-A）

    # ── OpenWakeWord 插件 ─────────────────────────────────────────
    - name: "oww_xiao_wei"
      type: "openwakeword"
      enabled: true
      # ONNX 模型文件路径（社区模型库：https://github.com/dscripka/openWakeWord）
      # 自训练流程：openWakeWord 提供 Colab 笔记本，约 5 分钟生成自定义模型
      model_path: "models/kws/oww/xiao_wei.onnx"
      # 触发阈值 [0.0, 1.0]，超过此值视为命中，建议 0.5 起步按实测调整
      threshold: 0.5
      # 推理帧大小（ms），OpenWakeWord 内部以 80ms 帧处理
      frame_ms: 80
      # 内存参考：约 20MB（含特征提取器 embedding 模型）

    - name: "oww_hey_robot"
      type: "openwakeword"
      enabled: false                  # 按需开启
      model_path: "models/kws/oww/hey_robot_en.onnx"
      threshold: 0.55
      frame_ms: 80

    # ── Sherpa-ONNX KWS 插件 ──────────────────────────────────────
    - name: "sherpa_kws_zh"
      type: "sherpa_kws"
      enabled: false                  # 与 porcupine/oww 任选其一或并联
      # Sherpa-ONNX KWS 专用模型（非流式 Zipformer，而是关键词 CTC 小模型）
      # 下载：https://github.com/k2-fsa/sherpa-onnx/releases  关键词 kws 分类
      model_path: "models/kws/sherpa/sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01"
      # 唤醒词列表（支持中文 / 英文 / 拼音，逐行一个词）
      keywords_file: "models/kws/sherpa/keywords.txt"
      # keywords.txt 格式示例（每行一个唤醒词 + 拼音）：
      #   小未 xiao3 wei4
      #   你好小未 ni3 hao3 xiao3 wei4
      num_threads: 2
      # 关键词触发阈值（越高误触越低，建议 0.25～0.5）
      keywords_score: 0.25
      # 连续触发抑制帧数（命中后静默 N 帧再重新监听，防止重复触发）
      num_trailing_blanks: 1
      # 内存参考：约 15MB（3.3M 参数量 int8 量化模型）

    # ── 自定义 ONNX KWS 插件 ──────────────────────────────────────
    - name: "custom_kws_arm"
      type: "custom_onnx"
      enabled: false
      # 用户自训练的任意 ONNX KWS 模型
      model_path: "models/kws/custom/my_kws_model.onnx"
      # 输入：16kHz int16 PCM，帧大小（ms）
      frame_ms: 30
      # ONNX 模型输入 / 输出 tensor 名（用 Netron 查看模型结构）
      input_name: "input"
      output_name: "output"
      # 输出正例（命中）的 class index
      positive_class_index: 1
      # 触发阈值
      threshold: 0.7
      # 唤醒词标签（仅用于日志记录）
      label: "小未"

speaker_id:
  enabled: false
  model_path: "models/speaker/ecapa_tdnn.onnx"
  profiles_dir: "data/speaker_profiles/"
  similarity_threshold: 0.75
  reject_unknown: false
  inject_username_to_prompt: true

silence:
  wakeup_timeout_s: 5
  dialog_timeout_s: 10
  timeout_prompt: "好的，如需帮助请再次呼叫我。"

web_search:
  enabled: false
  type: "web_summary"
  api_key: "YOUR_WEBSEARCH_API_KEY"
  bot_id: ""
  result_count: 5
  location:
    enabled: true
    country: "中国"
    country_code: "CN"
    province: "上海"
    city: "上海"
    longitude: 121.473701
    latitude: 31.230416

music:
  enable_music: false

audit:
  strict_audit: true
  audit_response: "这个问题我暂时无法回答，请换个话题吧。"

command:
  enabled: true
  registry_path: "config/commands.yaml"
  match_strategy: "regex"
  exec_timeout_s: 10
  exec_prompt: "好的，正在执行。"
  exec_done_prompt: "执行完成。"
  exec_fail_prompt: "执行失败，请重试。"

ros2:
  enabled: true
  node_name: "voicebot_node"
  arm_service: "/arm_controller/execute"
  nav_service: "/navigation/move_to"
  gripper_service: "/gripper/control"
  service_timeout_s: 5.0

audio_output:
  backend: "pulseaudio"
  pulse_sink: ""
  paplay_args: "--rate=24000 --channels=1 --format=s16le"
  buffer_ms: 80
  interruptible: true
  ack_volume_percent: 80

audio_input:
  device: "default"
  gain_db: 0
  enable_noise_suppress: true

context:
  dialog_id: ""
  enable_conversation_truncate: true
  persist_path: "data/dialog_id.txt"

exit_intent:
  enable_user_query_exit: true
  local_exit_keywords:
    - "再见"
    - "拜拜"
    - "退出"
    - "结束对话"

logging:
  level: "INFO"
  file: "logs/voicebot.log"
  max_size_mb: 50
  console_output: true

aigc_metadata:
  enable: false
  content_producer: "VoiceBot"
```

### 配置解析（`voicebot/config/app_config.py`）

```python
# voicebot/config/app_config.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import yaml


@dataclass
class AuthConfig:
    api_key: str
    resource_id: str
    app_key: str
    ws_url: str
    connect_timeout_s: float = 5.0
    reconnect_max_retry: int = 3
    reconnect_delay_s: float = 1.0


@dataclass
class PorcupineKWSConfig:
    """Picovoice Porcupine 插件配置"""
    name: str
    enabled: bool
    access_key: str
    keyword_paths: List[str] = field(default_factory=list)
    built_in_keywords: List[str] = field(default_factory=list)
    sensitivities: List[float] = field(default_factory=lambda: [0.6])
    device: str = "cpu"


@dataclass
class OpenWakeWordKWSConfig:
    """OpenWakeWord 插件配置"""
    name: str
    enabled: bool
    model_path: str
    threshold: float = 0.5
    frame_ms: int = 80


@dataclass
class SherpaKWSConfig:
    """Sherpa-ONNX KWS 插件配置"""
    name: str
    enabled: bool
    model_path: str
    keywords_file: str
    num_threads: int = 2
    keywords_score: float = 0.25
    num_trailing_blanks: int = 1


@dataclass
class CustomOnnxKWSConfig:
    """通用自定义 ONNX KWS 插件配置"""
    name: str
    enabled: bool
    model_path: str
    frame_ms: int = 30
    input_name: str = "input"
    output_name: str = "output"
    positive_class_index: int = 1
    threshold: float = 0.7
    label: str = "wakeword"


# 所有插件配置的联合类型
KWSModelConfig = PorcupineKWSConfig | OpenWakeWordKWSConfig | SherpaKWSConfig | CustomOnnxKWSConfig


@dataclass
class WakewordConfig:
    enabled: bool = True
    ack_audio_path: str = "assets/wakeup_ack.wav"
    ack_text: str = "我在，请说。"
    # 按 yaml active_models 顺序解析得到的插件配置列表（enabled=true 的才进列表）
    active_models: List[KWSModelConfig] = field(default_factory=list)


@dataclass
class TTSConfig:
    speaker: str = "zh_male_yunzhou_jupiter_bigtts"
    clone_speaker_id: str = ""
    audio_format: str = "pcm_s16le"
    sample_rate: int = 24000
    channel: int = 1
    speech_rate: int = 0
    loudness_rate: int = 0
    enable_loudness_norm: bool = False
    explicit_dialect: str = ""


@dataclass
class ASRConfig:
    audio_format: str = "pcm"
    sample_rate: int = 16000
    channel: int = 1
    end_smooth_window_ms: int = 1200
    enable_custom_vad: bool = True
    hotwords: List[str] = field(default_factory=list)
    correct_words: Dict[str, str] = field(default_factory=dict)
    boosting_table_id: str = ""


@dataclass
class PersonaConfig:
    bot_name: str = "小未"
    system_role: str = ""
    speaking_style: str = ""
    say_hello_prompt: str = ""
    character_manifest: str = ""


@dataclass
class SilenceConfig:
    wakeup_timeout_s: int = 5
    dialog_timeout_s: int = 10
    timeout_prompt: str = "好的，如需帮助请再次呼叫我。"


@dataclass
class ContextConfig:
    dialog_id: str = ""
    enable_conversation_truncate: bool = True
    persist_path: str = "data/dialog_id.txt"


@dataclass
class AudioOutputConfig:
    backend: str = "pulseaudio"
    pulse_sink: str = ""
    paplay_args: str = "--rate=24000 --channels=1 --format=s16le"
    buffer_ms: int = 80
    interruptible: bool = True
    ack_volume_percent: int = 80


@dataclass
class AppConfig:
    scene: str = "robot_assistant"
    auth: AuthConfig = field(default_factory=AuthConfig)
    model_version: str = "1.2.1.1"
    persona: PersonaConfig = field(default_factory=PersonaConfig)
    tts: TTSConfig = field(default_factory=TTSConfig)
    asr: ASRConfig = field(default_factory=ASRConfig)
    wakeword: WakewordConfig = field(default_factory=WakewordConfig)
    silence: SilenceConfig = field(default_factory=SilenceConfig)
    context: ContextConfig = field(default_factory=ContextConfig)
    audio_output: AudioOutputConfig = field(default_factory=AudioOutputConfig)

    @classmethod
    def from_yaml(cls, path: str) -> "AppConfig":
        """从 YAML 文件加载完整配置"""
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)

        cfg = cls()
        cfg.scene = raw.get("scene", "robot_assistant")
        cfg.model_version = raw.get("model", {}).get("version", "1.2.1.1")

        # 解析 auth
        a = raw.get("auth", {})
        cfg.auth = AuthConfig(
            api_key=a["api_key"],
            resource_id=a.get("resource_id", "volc.speech.dialog"),
            app_key=a["app_key"],
            ws_url=a.get("ws_url", "wss://openspeech.bytedance.com/api/v3/realtime/dialogue"),
            connect_timeout_s=a.get("connect_timeout_s", 5),
            reconnect_max_retry=a.get("reconnect_max_retry", 3),
            reconnect_delay_s=a.get("reconnect_delay_s", 1.0),
        )

        # 解析 wakeword 插件列表
        ww = raw.get("wakeword", {})
        active_models: list = []
        for m in ww.get("active_models", []):
            if not m.get("enabled", False):
                continue   # 跳过 enabled=false 的插件
            t = m["type"]
            if t == "porcupine":
                active_models.append(PorcupineKWSConfig(
                    name=m["name"],
                    enabled=True,
                    access_key=m.get("access_key", ""),
                    keyword_paths=m.get("keyword_paths", []),
                    built_in_keywords=m.get("built_in_keywords", []),
                    sensitivities=m.get("sensitivities", [0.6]),
                    device=m.get("device", "cpu"),
                ))
            elif t == "openwakeword":
                active_models.append(OpenWakeWordKWSConfig(
                    name=m["name"],
                    enabled=True,
                    model_path=m["model_path"],
                    threshold=m.get("threshold", 0.5),
                    frame_ms=m.get("frame_ms", 80),
                ))
            elif t == "sherpa_kws":
                active_models.append(SherpaKWSConfig(
                    name=m["name"],
                    enabled=True,
                    model_path=m["model_path"],
                    keywords_file=m["keywords_file"],
                    num_threads=m.get("num_threads", 2),
                    keywords_score=m.get("keywords_score", 0.25),
                    num_trailing_blanks=m.get("num_trailing_blanks", 1),
                ))
            elif t == "custom_onnx":
                active_models.append(CustomOnnxKWSConfig(
                    name=m["name"],
                    enabled=True,
                    model_path=m["model_path"],
                    frame_ms=m.get("frame_ms", 30),
                    input_name=m.get("input_name", "input"),
                    output_name=m.get("output_name", "output"),
                    positive_class_index=m.get("positive_class_index", 1),
                    threshold=m.get("threshold", 0.7),
                    label=m.get("label", "wakeword"),
                ))
            else:
                import logging
                logging.warning(f"[AppConfig] unknown kws type '{t}' for model '{m.get('name')}', skip")
        cfg.wakeword = WakewordConfig(
            enabled=ww.get("enabled", True),
            ack_audio_path=ww.get("ack_audio_path", "assets/wakeup_ack.wav"),
            ack_text=ww.get("ack_text", "我在，请说。"),
            active_models=active_models,
        )
        return cfg
```

---

## 5. 核心流程设计

### 5.1 唤醒 → 会话建立流程

```
[asyncio 事件循环]

[音频采集协程] pyaudio 回调 → asyncio.Queue（pcm_queue）
    │
    ▼
[KWSManager] 广播 PCM → 各 KWS 插件独立线程（各自 queue.Queue）
    │ 任意插件命中（keyword, model_name）→ loop.call_soon_threadsafe → wake_queue
    ▼
[SessionManager] 状态切换 IDLE → WAKEUP
    │
    ├─ 1. kws_manager.stop()                # 停止所有 KWS 插件线程
    ├─ 2. await audio_player.play_file(ack_audio)  # 播放"我在，请说。"
    ├─ 3. await speaker_id.identify(buf)    # 声纹识别（可选）
    ├─ 4. await doubao_ws.connect()         # 建立 WebSocket
    ├─ 5. await doubao_ws.send_start_connection()
    ├─ 6. await doubao_ws.wait_connection_started()
    ├─ 7. payload = build_start_session(cfg)
    ├─ 8. await doubao_ws.send_start_session(payload)
    ├─ 9. dialog_id = await doubao_ws.wait_session_started()
    ├─ 10. state = DIALOG; asyncio.create_task(audio_upload_loop())
    └─ 11. asyncio.create_task(send_say_hello())  # 异步，不阻塞采集
           # 读取时间 + 查 ROS2 状态 → 填充 prompt 模板 → send_say_hello
```

### 5.2 对话流程（DIALOG 状态）

```
[音频上传 Task]              [WS 接收 Task]                    [音频播放 Task]
      │                           │                                   │
      │ pcm_queue.get()           │                                   │
      │ → build_task_request()    │                                   │
      │ → ws.send(binary_frame)   │                                   │
      │                           │                                   │
      │                           │ ① ASRInfo（用户开始说话）            │
      │                           │ → audio_player.stop()             │ 立即停止播放
      │                           │ → 若 enable_conversation_truncate: │
      │                           │     reply_id = player.reply_id    │
      │                           │     played_ms = player.played_ms  │
      │                           │     await ws.send_conv_truncate(  │
      │                           │       reply_id, played_ms)        │
      │                           │ → cmd_engine.reset()              │
      │                           │ → silence_detector.on_activity()  │
      │                           │                                   │
      │                           │ ② ASRResponse（流式识别文本）        │
      │                           │ → 更新 UI / 日志                   │
      │                           │                                   │
      │                           │ ③ ASREnded（用户说完）              │
      │                           │ → cmd_engine.on_asr_ended(text)  │
      │                           │   Level1/2 本地匹配（< 5ms）        │
      │                           │   命中 → await ws.send_chat_tts() │ 豆包直接 TTS
      │                           │         → ros2.call_service()     │ 并行执行
      │                           │                                   │
      │                           │ ④ ChatResponse（模型文本流）         │
      │                           │ → cmd_engine.feed(chunk)          │
      │                           │   扫描 [CMD:xxx] 标签              │
      │                           │   命中 → send_chat_tts + ros2     │
      │                           │                                   │
      │                           │ ⑤ TTSSentenceStart / TTSResponse  │
      │                           │ → audio_player.push(reply_id,     │ 推入播放队列
      │                           │     pcm_chunk)                    │
      │                           │                                   │
      │                           │ ⑥ TTSEnded                        │
      │                           │ → 若 status_code=="20000002"       │
      │                           │     → trigger_exit_intent()       │
```

#### 5.2.1 SayHello 情境感知（Python 实现）

```python
# voicebot/session/session_manager.py（片段）
import asyncio
from datetime import datetime


async def _send_say_hello(self) -> None:
    """
    SessionStarted 后异步发送 SayHello，不阻塞主流程。
    读取当前时间 + ROS2 状态，填充 config.yaml 中的 say_hello_prompt 模板。
    """
    # 1. 获取当前时间段
    hour = datetime.now().hour
    if 6 <= hour < 12:
        time_str = f"上午{hour}点"
    elif 12 <= hour < 18:
        time_str = f"下午{hour - 12}点"
    else:
        time_str = f"晚上{hour if hour < 24 else hour - 24}点"

    # 2. 查询 ROS2 机械臂状态（设置 500ms 超时，超时使用默认值）
    try:
        arm_status, workstation = await asyncio.wait_for(
            self.ros2_bridge.query_arm_status(),  # 返回 (status, workstation) 元组
            timeout=0.5
        )
    except asyncio.TimeoutError:
        arm_status, workstation = "待机", "默认工位"

    # 3. 填充模板占位符
    prompt = self.cfg.persona.say_hello_prompt
    prompt = prompt.replace("{time}", time_str)
    prompt = prompt.replace("{workstation}", workstation)
    prompt = prompt.replace("{arm_status}", arm_status)
    # 最终示例: "用户唤醒了我，现在是下午3点，机械臂当前位于工位A，状态为待机。"

    # 4. 发送 SayHello 事件
    await self.ws_client.send_say_hello(prompt)
```

#### 5.2.2 流式上下文截断（ConversationTruncate）

```python
# voicebot/session/session_manager.py（片段）
async def _on_asr_info(self, question_id: str) -> None:
    """
    用户开始说话（首字命中）回调。
    立即停止 TTS 播放，并发送 ConversationTruncate 告知服务端截断点。
    这样服务端只把已播放部分计入模型上下文，避免模型幻觉。
    """
    # 1. 立即停止当前 TTS 播放
    self.audio_player.stop()

    # 2. 发送 ConversationTruncate（若开启且正在播放）
    if self.cfg.context.enable_conversation_truncate and self.audio_player.is_playing:
        reply_id = self.audio_player.current_reply_id
        played_ms = self.audio_player.played_ms
        # Payload: {"item_id": "reply_id_xxx", "audio_end_ms": 1234}
        await self.ws_client.send_conversation_truncate(reply_id, played_ms)

    # 3. 重置命令词引擎缓冲区
    self.cmd_engine.reset()

    # 4. 重置静默检测计时器
    self.silence_detector.on_activity()
```

### 5.3 静默超时流程

```python
# voicebot/session/silence_detector.py
import asyncio
from enum import Enum


class DialogPhase(Enum):
    WAKEUP = "wakeup"
    DIALOG = "dialog"


class SilenceDetector:
    def __init__(self, wakeup_timeout_s: int, dialog_timeout_s: int):
        self._wakeup_timeout = wakeup_timeout_s
        self._dialog_timeout = dialog_timeout_s
        self._activity_event = asyncio.Event()
        self._running = False
        self._on_timeout_cb = None

    def on_activity(self) -> None:
        """有语音活动时调用，重置计时器"""
        self._activity_event.set()

    def set_timeout_callback(self, cb) -> None:
        self._on_timeout_cb = cb

    async def run(self, phase: DialogPhase) -> None:
        """异步运行静默检测"""
        self._running = True
        timeout = (
            self._wakeup_timeout if phase == DialogPhase.WAKEUP
            else self._dialog_timeout
        )
        while self._running:
            self._activity_event.clear()
            try:
                # 等待语音活动，超时则触发回调
                await asyncio.wait_for(
                    self._activity_event.wait(), timeout=timeout
                )
            except asyncio.TimeoutError:
                if self._on_timeout_cb:
                    await self._on_timeout_cb(phase)
                break

    def stop(self) -> None:
        self._running = False
        self._activity_event.set()  # 解除 wait 阻塞
```

### 5.4 命令词控制流程

```
[ChatResponse 流式文本] asyncio.Queue 实时分发
    │
    ▼
[CmdEngine.feed(chunk)]  ← 每个文本片段调用一次
    │
    ├── 正则 / 关键词实时扫描（流式匹配）
    │
    │ 命中命令词
    ▼
[CmdEngine._dispatch(cmd_result)]
    ├─ 1. await ws.send_chat_tts_text(cmd.tts_confirm)  # 注入确认语音
    ├─ 2. state → CMD_EXEC
    ├─ 3. asyncio.create_task(ros2.call_service(...))    # 并行执行
    ├─ 4. await asyncio.wait_for(ros2_task, timeout=exec_timeout_s)
    ├─ 5. 播放执行结果语音
    └─ 6. state → DIALOG（继续对话）
```

---

## 6. 命令词控制体系

### 6.1 三级匹配策略

```
优先级 高 → 低：

Level 1: 本地关键词精确匹配（极低延迟 < 1ms）
    └── "急停"、"回零" 等安全关键词（Python set 查找，O(1)）

Level 2: 本地正则表达式匹配（低延迟 < 5ms）
    └── re.search("(抓取|夹取).*(物品|工件)", text)

Level 3: LLM 意图识别（高召回，与模型响应同步）
    └── ChatResponse 文本流中扫描 [CMD:xxx] 标签
        system_role 已约定标签协议，CmdEngine 流式检测
```

### 6.2 意图标签注入（推荐机制）

在 `system_role` 中加入约定后，CmdEngine 通过以下正则扫描：

```python
# voicebot/command/cmd_engine.py（片段）
import re

# 意图标签正则：匹配 [CMD:arm_grab] 或 [CMD:navigate_to:工位A]
CMD_TAG_PATTERN = re.compile(r'\[CMD:([a-zA-Z_]+)(?::([^\]]+))?\]')

class CmdEngine:
    def feed(self, chunk: str) -> None:
        """流式喂入文本，扫描并剥离意图标签"""
        self._buffer += chunk
        for m in CMD_TAG_PATTERN.finditer(self._buffer):
            cmd_name = m.group(1)   # e.g. "arm_grab"
            cmd_arg  = m.group(2)   # e.g. "工位A" （可选）
            if cmd_def := self._registry.get(cmd_name):
                asyncio.create_task(
                    self._dispatch(cmd_def, arg=cmd_arg)
                )
        # 剥离已处理的标签，保证用户不可见
        self._buffer = CMD_TAG_PATTERN.sub("", self._buffer)
```

---

## 7. 各模块详细说明

### 7.1 DoubaoWSClient（websocket/doubao_ws_client.py）

**职责**：管理 WebSocket 连接生命周期，处理二进制协议帧的组装与解析。基于 `websockets` 异步库实现。

```python
# voicebot/websocket/doubao_ws_client.py
import asyncio
import struct
import uuid
import json
from typing import Callable, Optional
import websockets


class DoubaoWSClient:
    """
    豆包 RealtimeAPI WebSocket 客户端。
    使用 websockets 异步库，全程 asyncio 驱动。

    二进制帧格式（Header 4 字节 + Optional + Payload）：
      Byte 0: 0x11  Protocol v1, Header Size=1
      Byte 1: 0x10  Message Type=Full-client, flags=with_event(0b0100)
      Byte 2: 0x10  Serialization=JSON, Compression=None
      Byte 3: 0x00  Reserved
      [4 bytes] event_id (big-endian uint32)
      [4 bytes] session_id_size + session_id (UTF-8 UUID 字符串)
      [4 bytes] payload_size + payload bytes
    """

    # 豆包客户端事件 ID 定义（发送方向）
    EVENT_START_CONNECTION = 1
    EVENT_FINISH_CONNECTION = 2
    EVENT_START_SESSION = 100
    EVENT_FINISH_SESSION = 102
    EVENT_TASK_REQUEST = 200      # 音频上传
    EVENT_SAY_HELLO = 201         # 情境感知问候（需在 SessionStarted 后）
    EVENT_CHAT_TTS_TEXT = 202     # 注入 TTS 文本（绕过模型生成，需在 ASREnded 后）
    EVENT_CHAT_TEXT_QUERY = 203   # 纯文本 query
    EVENT_CHAT_RAG_TEXT = 204     # 外部 RAG 注入
    EVENT_UPDATE_CONFIG = 205     # 运行时热更新（音色/人设/位置）
    EVENT_CONV_TRUNCATE = 301     # 流式上下文截断（需在 ASRInfo 后，开启 truncate 时使用）
    EVENT_CONV_UPDATE = 302       # 上下文更新
    EVENT_CONV_DELETE = 303       # 上下文删除

    # 服务端事件 ID 定义（接收方向）
    EVENT_CONNECTION_STARTED = 50
    EVENT_SESSION_STARTED = 150
    EVENT_SESSION_FINISHED = 152
    EVENT_ASR_INFO = 350
    EVENT_ASR_RESPONSE = 351
    EVENT_ASR_ENDED = 352
    EVENT_CHAT_RESPONSE = 360
    EVENT_TTS_SENTENCE_START = 400
    EVENT_TTS_RESPONSE = 401
    EVENT_TTS_ENDED = 402
    EVENT_DIALOG_COMMON_ERROR = 599

    def __init__(self, cfg):
        self.cfg = cfg
        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._session_id: str = str(uuid.uuid4())
        self._callbacks: dict[int, Callable] = {}
        # 用于 wait_session_started() 等待响应的 Future
        self._session_started_fut: Optional[asyncio.Future] = None
        self._connection_started_fut: Optional[asyncio.Future] = None

    async def connect(self) -> None:
        """建立 WebSocket 连接，携带鉴权 Header"""
        headers = {
            "X-Api-App-ID": self.cfg.auth.app_key,  # 注意：App-ID 字段填 app_key
            "X-Api-Access-Key": self.cfg.auth.api_key,
            "X-Api-Resource-Id": self.cfg.auth.resource_id,
            "X-Api-App-Key": self.cfg.auth.app_key,
            "X-Api-Connect-Id": str(uuid.uuid4()),
        }
        self._ws = await websockets.connect(
            self.cfg.auth.ws_url,
            extra_headers=headers,
            open_timeout=self.cfg.auth.connect_timeout_s,
        )
        # 启动接收循环
        asyncio.create_task(self._recv_loop())

    async def _recv_loop(self) -> None:
        """持续接收 WebSocket 消息并分发到对应回调"""
        async for raw in self._ws:
            try:
                event_id, payload_bytes = self._parse_frame(raw)
                await self._dispatch(event_id, payload_bytes)
            except Exception as e:
                import logging
                logging.error(f"[DoubaoWSClient] recv error: {e}")

    def _build_frame(self, event_id: int, payload: bytes) -> bytes:
        """
        组装豆包二进制帧。
        Header(4B) + event_id(4B) + session_id_size(4B) + session_id + payload_size(4B) + payload
        """
        header = bytes([0x11, 0x10, 0x10, 0x00])
        ev = struct.pack(">I", event_id)
        sid = self._session_id.encode("utf-8")
        sid_size = struct.pack(">I", len(sid))
        pl_size = struct.pack(">I", len(payload))
        return header + ev + sid_size + sid + pl_size + payload

    def _parse_frame(self, data: bytes) -> tuple[int, bytes]:
        """
        解析豆包服务端二进制帧，返回 (event_id, payload_bytes)。
        服务端帧格式同客户端，header 4B 后依次为 event_id、session_id、payload。
        """
        offset = 4  # 跳过 header
        event_id = struct.unpack(">I", data[offset:offset + 4])[0]
        offset += 4
        sid_size = struct.unpack(">I", data[offset:offset + 4])[0]
        offset += 4 + sid_size
        pl_size = struct.unpack(">I", data[offset:offset + 4])[0]
        offset += 4
        payload = data[offset:offset + pl_size]
        return event_id, payload

    async def _dispatch(self, event_id: int, payload: bytes) -> None:
        """将接收到的服务端事件分发给注册的回调"""
        cb = self._callbacks.get(event_id)

        if event_id == self.EVENT_SESSION_STARTED:
            data = json.loads(payload)
            dialog_id = data.get("dialog", {}).get("dialog_id", "")
            if self._session_started_fut and not self._session_started_fut.done():
                self._session_started_fut.set_result(dialog_id)

        elif event_id == self.EVENT_CONNECTION_STARTED:
            if self._connection_started_fut and not self._connection_started_fut.done():
                self._connection_started_fut.set_result(True)

        elif event_id == self.EVENT_TTS_RESPONSE:
            # TTS 音频帧：payload 为原始 PCM/OGG 二进制，非 JSON
            if cb:
                await cb(payload)
            return

        # 其余事件 payload 为 JSON
        if cb:
            try:
                data = json.loads(payload) if payload else {}
                await cb(data)
            except Exception as e:
                import logging
                logging.error(f"[DoubaoWSClient] callback error event={event_id}: {e}")

    def on(self, event_id: int, callback: Callable) -> None:
        """注册事件回调"""
        self._callbacks[event_id] = callback

    async def wait_session_started(self) -> str:
        """等待 SessionStarted 事件，返回 dialog_id"""
        loop = asyncio.get_event_loop()
        self._session_started_fut = loop.create_future()
        return await self._session_started_fut

    async def send_start_connection(self) -> None:
        payload = json.dumps({}).encode()
        await self._ws.send(self._build_frame(self.EVENT_START_CONNECTION, payload))

    async def send_start_session(self, session_payload: dict) -> None:
        payload = json.dumps(session_payload).encode()
        await self._ws.send(self._build_frame(self.EVENT_START_SESSION, payload))

    async def send_audio_chunk(self, pcm: bytes) -> None:
        """
        发送 20ms PCM 音频帧（TaskRequest 事件）。
        PCM 格式：16k Hz, int16, 单声道，640 字节/包。
        """
        await self._ws.send(self._build_frame(self.EVENT_TASK_REQUEST, pcm))

    async def send_chat_tts_text(self, text: str, start: bool = True, end: bool = True) -> None:
        """
        注入 TTS 文本（ChatTTSText 事件）。
        必须在收到 ASREnded 之后才能调用，否则服务端拒绝。
        服务端直接合成音频，不经过 LLM 生成，适合快速确认语音（< 100ms 响应）。
        """
        payload = json.dumps({
            "text": text,
            "start": start,
            "end": end,
        }).encode()
        await self._ws.send(self._build_frame(self.EVENT_CHAT_TTS_TEXT, payload))

    async def send_say_hello(self, content: str) -> None:
        """
        发送 SayHello 事件（UpdateConfig 事件 ID 201）。
        必须在 SessionStarted 之后发送，模型据此生成情境感知开场白。
        """
        payload = json.dumps({"content": content}).encode()
        await self._ws.send(self._build_frame(self.EVENT_SAY_HELLO, payload))

    async def send_conversation_truncate(self, item_id: str, audio_end_ms: int) -> None:
        """
        发送 ConversationTruncate 事件。
        在用户打断（ASRInfo）后调用，告知服务端只把 audio_end_ms 以前的内容
        计入模型上下文，避免模型误以为说完了未播放的内容。
        item_id: 当前播放的 TTSResponse reply_id
        audio_end_ms: 已播放的毫秒数
        """
        payload = json.dumps({
            "item_id": item_id,
            "audio_end_ms": audio_end_ms,
        }).encode()
        await self._ws.send(self._build_frame(self.EVENT_CONV_TRUNCATE, payload))

    async def send_finish_session(self) -> None:
        payload = json.dumps({}).encode()
        await self._ws.send(self._build_frame(self.EVENT_FINISH_SESSION, payload))

    async def send_finish_connection(self) -> None:
        payload = json.dumps({}).encode()
        await self._ws.send(self._build_frame(self.EVENT_FINISH_CONNECTION, payload))

    async def close(self) -> None:
        if self._ws:
            await self._ws.close()
```

### 7.2 SessionConfig 构建（websocket/session_config.py）

```python
# voicebot/websocket/session_config.py
import json
from dataclasses import dataclass, asdict, field
from typing import Optional, List, Dict


def build_start_session_payload(cfg) -> dict:
    """
    根据 AppConfig 构建 StartSession 事件的 JSON payload。
    涵盖 TTS / ASR / Dialog / Location / Extra 等全部参数。
    """
    # TTS 配置
    speaker = cfg.tts.clone_speaker_id or cfg.tts.speaker
    tts_cfg: dict = {
        "speaker": speaker,
        "audio_config": {
            "format": cfg.tts.audio_format,   # "pcm_s16le" | "ogg_opus"
            "sample_rate": cfg.tts.sample_rate,
            "channel": cfg.tts.channel,
        },
        "extra": {
            "speech_rate": cfg.tts.speech_rate,
            "loudness_rate": cfg.tts.loudness_rate,
            "enable_loudness_norm": cfg.tts.enable_loudness_norm,
        },
    }
    # 方言仅 O2.0 vv 音色生效
    if cfg.tts.explicit_dialect:
        tts_cfg["extra"]["explicit_dialect"] = cfg.tts.explicit_dialect

    # ASR 配置
    asr_cfg: dict = {
        "audio_info": {
            "format": cfg.asr.audio_format,   # "pcm" | "speech_opus"
            "sample_rate": cfg.asr.sample_rate,
            "channel": cfg.asr.channel,
        },
        "extra": {
            "end_smooth_window_ms": cfg.asr.end_smooth_window_ms,
            "enable_custom_vad": cfg.asr.enable_custom_vad,
        },
    }
    # 热词
    if cfg.asr.hotwords:
        asr_cfg["extra"]["hotwords"] = cfg.asr.hotwords
    # 文字替换规则
    if cfg.asr.correct_words:
        asr_cfg["extra"]["correct_words"] = [
            {"before": k, "after": v} for k, v in cfg.asr.correct_words.items()
        ]

    # Dialog 配置（人设 + 上下文 + 模型版本）
    dialog_cfg: dict = {
        "extra": {
            "model": cfg.model_version,
            "input_mod": "microphone",          # 按 config.input.mode 映射
            "enable_conversation_truncate": cfg.context.enable_conversation_truncate,
            "enable_user_query_exit": True,     # 退出意图识别
        }
    }
    # O2.0 版本人设字段
    if cfg.model_version.startswith("1."):
        dialog_cfg["extra"]["bot_name"] = cfg.persona.bot_name
        dialog_cfg["extra"]["system_role"] = cfg.persona.system_role
        dialog_cfg["extra"]["speaking_style"] = cfg.persona.speaking_style
    # SC2.0 版本人设字段
    else:
        if cfg.persona.character_manifest:
            dialog_cfg["extra"]["character_manifest"] = cfg.persona.character_manifest

    # 续接历史上下文
    if cfg.context.dialog_id:
        dialog_cfg["extra"]["dialog_id"] = cfg.context.dialog_id

    # 安全审核
    dialog_cfg["extra"]["strict_audit"] = True
    dialog_cfg["extra"]["audit_response"] = "这个问题我暂时无法回答，请换个话题吧。"

    # 唱歌能力（O2.0 1.2.1.1 专属）
    dialog_cfg["extra"]["enable_music"] = False

    return {
        "tts": tts_cfg,
        "asr": asr_cfg,
        "dialog": dialog_cfg,
    }
```

### 7.3 KWS 唤醒模块（kws/）

#### 设计思路

```
麦克风 PCM（16kHz, int16, 20ms/包）
        │
        ▼
  KWSManager.feed_audio(pcm)
        │  广播给所有 enabled 插件
        ├──▶ PorcupineKWS._pcm_queue  → 推理线程 → 命中 → wake_queue
        ├──▶ OpenWakeWordKWS._pcm_queue → 推理线程 → 命中 → wake_queue
        ├──▶ SherpaKWS._pcm_queue    → 推理线程 → 命中 → wake_queue
        └──▶ CustomOnnxKWS._pcm_queue → 推理线程 → 命中 → wake_queue
                                                        │
                               loop.call_soon_threadsafe(wake_queue.put)
                                                        │
                                          asyncio 事件循环
                                                        │
                                   SessionManager.wait_for_wake()
                                    返回 (keyword, model_name)
```

**核心规则**：
- 每个 KWS 插件独占一个 `threading.Thread`，推理完全在线程内，不阻塞 asyncio 事件循环
- 所有插件共享同一个 `asyncio.Queue`（`wake_queue`），谁先命中谁先通知
- `KWSManager` 是唯一对外接口，`SessionManager` 只与它交互，不感知具体插件类型
- 插件通过 `BaseKWS` 抽象基类统一接口，新增引擎只需继承并实现三个方法

---

#### 7.3.1 抽象基类（kws/base_kws.py）

```python
# voicebot/kws/base_kws.py
"""
KWS 插件抽象基类。
所有唤醒引擎插件必须继承此类并实现 _run_loop()。
"""
import asyncio
import queue
import threading
import logging
from abc import ABC, abstractmethod
from typing import Optional


class BaseKWS(ABC):
    """
    KWS 插件基类。
    子类只需实现 _init_model() 和 _run_loop()，
    线程管理、PCM 队列、asyncio 桥接由基类统一处理。
    """

    def __init__(self, name: str, wake_queue: asyncio.Queue,
                 loop: asyncio.AbstractEventLoop):
        self.name = name                                  # 实例别名（来自 yaml name 字段）
        self._wake_queue = wake_queue                     # 共享唤醒队列（asyncio.Queue）
        self._loop = loop                                 # asyncio 事件循环引用
        self._pcm_queue: queue.Queue = queue.Queue(maxsize=200)  # 线程内 PCM 缓冲
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── 子类必须实现 ────────────────────────────────────────────────

    @abstractmethod
    def _init_model(self) -> None:
        """
        加载模型资源（在 start() 之前调用）。
        阻塞操作，只执行一次，模型加载完成后线程才启动。
        """

    @abstractmethod
    def _run_loop(self) -> None:
        """
        推理线程主循环。
        从 self._pcm_queue 中取 PCM bytes，执行推理，命中时调用 self._notify(keyword)。
        循环需检查 self._running，收到 None 哨兵时退出。
        """

    # ── 基类提供的公共接口 ──────────────────────────────────────────

    def init(self) -> None:
        """加载模型，程序启动时由 KWSManager 调用一次"""
        logging.info(f"[KWS:{self.name}] loading model...")
        self._init_model()
        logging.info(f"[KWS:{self.name}] model loaded")

    def start(self) -> None:
        """启动推理线程，进入 IDLE 监听状态"""
        self._running = True
        self._thread = threading.Thread(
            target=self._run_loop, name=f"kws-{self.name}", daemon=True
        )
        self._thread.start()
        logging.info(f"[KWS:{self.name}] started")

    def stop(self) -> None:
        """停止推理线程（唤醒命中后调用，防止重复触发）"""
        self._running = False
        self._pcm_queue.put(None)   # 哨兵：解除 _run_loop 的 queue.get() 阻塞
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        # 清空 PCM 缓冲，防止残留数据干扰下一轮
        while not self._pcm_queue.empty():
            try:
                self._pcm_queue.get_nowait()
            except queue.Empty:
                break
        logging.info(f"[KWS:{self.name}] stopped")

    def feed_audio(self, pcm: bytes) -> None:
        """
        由 KWSManager 调用，将 PCM 推入插件的推理队列。
        非阻塞：队列满时丢弃（避免主线程被推理延迟拖慢）。
        """
        try:
            self._pcm_queue.put_nowait(pcm)
        except queue.Full:
            pass  # 丢包优于阻塞采集

    def _notify(self, keyword: str) -> None:
        """
        子类在命中时调用，将 (keyword, model_name) 线程安全地提交给 asyncio 事件循环。
        call_soon_threadsafe 是唯一安全的跨线程 asyncio 操作方式。
        """
        self._loop.call_soon_threadsafe(
            self._wake_queue.put_nowait, (keyword, self.name)
        )
        logging.info(f"[KWS:{self.name}] WAKE → keyword='{keyword}'")
```

---

#### 7.3.2 Porcupine 插件（kws/porcupine_kws.py）

```python
# voicebot/kws/porcupine_kws.py
"""
Picovoice Porcupine KWS 插件。

特点：
  - 极低 CPU 占用（< 1%，专为嵌入式优化的声学模型）
  - 支持内置英文唤醒词 + 控制台自训练的自定义词（.ppn 文件）
  - 商业 SDK，免费额度：单设备无限制，多设备需授权

安装：
  pip install pvporcupine

自定义中文唤醒词训练：
  1. 前往 https://console.picovoice.ai/  →  Wake Word  →  Train
  2. 输入中文词（如"小未"），录音 / 上传样本，约 5 分钟出 .ppn 文件
  3. 将 .ppn 放入 models/kws/porcupine/ 目录，填入 yaml keyword_paths
"""
import queue
import numpy as np
import pvporcupine
import logging

from voicebot.kws.base_kws import BaseKWS
from voicebot.config.app_config import PorcupineKWSConfig


class PorcupineKWS(BaseKWS):

    # Porcupine 要求 PCM 帧大小固定 = frame_length 个 int16 样本
    # frame_length 由 SDK 决定（通常 512 samples @ 16kHz ≈ 32ms）
    SAMPLE_RATE = 16000

    def __init__(self, cfg: PorcupineKWSConfig, wake_queue, loop):
        super().__init__(cfg.name, wake_queue, loop)
        self._cfg = cfg
        self._porcupine = None
        self._frame_length = 0
        self._pcm_buf = b""    # 用于将 20ms 包对齐到 Porcupine 所需帧长

    def _init_model(self) -> None:
        """
        初始化 Porcupine 实例。
        keyword_paths 不为空时使用自定义词，否则使用内置词。
        """
        kw_paths = self._cfg.keyword_paths or None
        built_in = [
            pvporcupine.KEYWORD_PATHS[kw]
            for kw in self._cfg.built_in_keywords
            if kw in pvporcupine.KEYWORD_PATHS
        ] if self._cfg.built_in_keywords else []

        # 合并自定义和内置词路径
        final_paths = (kw_paths or []) + built_in
        sensitivities = self._cfg.sensitivities

        # 确保 sensitivities 长度与词数匹配
        if len(sensitivities) < len(final_paths):
            sensitivities = sensitivities + [0.6] * (len(final_paths) - len(sensitivities))

        self._porcupine = pvporcupine.create(
            access_key=self._cfg.access_key,
            keyword_paths=final_paths if final_paths else None,
            keywords=self._cfg.built_in_keywords if not final_paths else None,
            sensitivities=sensitivities[:len(final_paths)] if final_paths else sensitivities,
        )
        self._frame_length = self._porcupine.frame_length
        logging.info(
            f"[KWS:{self.name}] Porcupine ready, "
            f"frame_length={self._frame_length}, keywords={final_paths or self._cfg.built_in_keywords}"
        )

    def _run_loop(self) -> None:
        """
        推理线程：将输入 PCM 拼接到 frame_length 边界后交给 Porcupine 处理。
        Porcupine 要求每次传入恰好 frame_length 个 int16 样本（不能多也不能少）。
        """
        while self._running:
            chunk = self._pcm_queue.get()
            if chunk is None:
                break

            self._pcm_buf += chunk

            # 一次处理尽可能多的完整帧
            frame_bytes = self._frame_length * 2   # int16 = 2 bytes/sample
            while len(self._pcm_buf) >= frame_bytes:
                frame_bytes_data = self._pcm_buf[:frame_bytes]
                self._pcm_buf = self._pcm_buf[frame_bytes:]

                samples = np.frombuffer(frame_bytes_data, dtype=np.int16)
                keyword_index = self._porcupine.process(samples)

                if keyword_index >= 0:
                    # keyword_index 对应 keyword_paths 列表中的位置
                    label = (
                        self._cfg.keyword_paths[keyword_index]
                        if self._cfg.keyword_paths and keyword_index < len(self._cfg.keyword_paths)
                        else self._cfg.built_in_keywords[keyword_index] if self._cfg.built_in_keywords
                        else f"keyword_{keyword_index}"
                    )
                    self._notify(label)
                    self._pcm_buf = b""  # 清空缓冲防止重复触发
                    break

    def stop(self) -> None:
        super().stop()
        if self._porcupine:
            self._porcupine.delete()   # 释放 Porcupine native 资源
            self._porcupine = None
```

---

#### 7.3.3 OpenWakeWord 插件（kws/openwakeword_kws.py）

```python
# voicebot/kws/openwakeword_kws.py
"""
OpenWakeWord KWS 插件。

特点：
  - 完全开源（Apache-2.0），社区提供丰富预训练模型
  - 自训练极其简单：官方提供 Colab 笔记本，上传正负样本，约 5 分钟导出 ONNX
  - 内部使用 Google Speech Embedding（melspectrogram → embedding → 分类器）
  - 支持多唤醒词并行（一个模型文件包含多个分类头）

安装：
  pip install openwakeword

社区模型库：
  https://github.com/dscripka/openWakeWord/tree/main/openwakeword/resources/models

自训练工具：
  https://github.com/dscripka/openWakeWord/blob/main/notebooks/automatic_model_training.ipynb
"""
import queue
import numpy as np
import logging

from voicebot.kws.base_kws import BaseKWS
from voicebot.config.app_config import OpenWakeWordKWSConfig


class OpenWakeWordKWS(BaseKWS):

    SAMPLE_RATE = 16000

    def __init__(self, cfg: OpenWakeWordKWSConfig, wake_queue, loop):
        super().__init__(cfg.name, wake_queue, loop)
        self._cfg = cfg
        self._oww = None
        # OWW 以 80ms 帧为单位处理（1280 samples @ 16kHz）
        self._frame_samples = int(self.SAMPLE_RATE * cfg.frame_ms / 1000)
        self._pcm_buf = b""

    def _init_model(self) -> None:
        """
        加载 OpenWakeWord 模型。
        model_path 指向 .onnx 文件；OWW 内部自动加载配套的特征提取器。
        """
        from openwakeword.model import Model
        self._oww = Model(
            wakeword_models=[self._cfg.model_path],
            inference_framework="onnx",
        )
        logging.info(
            f"[KWS:{self.name}] OpenWakeWord ready, "
            f"model={self._cfg.model_path} threshold={self._cfg.threshold}"
        )

    def _run_loop(self) -> None:
        """
        推理线程：以 frame_ms 为单位喂入 OWW，检查输出置信度是否超过阈值。
        OWW 的 predict() 返回 {model_name: score} 字典。
        """
        while self._running:
            chunk = self._pcm_queue.get()
            if chunk is None:
                break

            self._pcm_buf += chunk
            frame_bytes = self._frame_samples * 2

            while len(self._pcm_buf) >= frame_bytes:
                frame_data = self._pcm_buf[:frame_bytes]
                self._pcm_buf = self._pcm_buf[frame_bytes:]

                samples = np.frombuffer(frame_data, dtype=np.int16)
                # OWW predict 返回 {model_label: confidence_score}
                scores: dict = self._oww.predict(samples)

                for label, score in scores.items():
                    if score >= self._cfg.threshold:
                        self._notify(label)
                        self._pcm_buf = b""   # 清空缓冲防止连续触发
                        # 重置 OWW 内部滑动窗口（避免同一声音重复报告）
                        self._oww.reset()
                        break
```

---

#### 7.3.4 Sherpa-ONNX KWS 插件（kws/sherpa_kws.py）

```python
# voicebot/kws/sherpa_kws.py
"""
Sherpa-ONNX KWS 插件（关键词 CTC 解码，非流式 Zipformer ASR）。

特点：
  - 开源，支持任意词表（keywords_file），无需重新训练模型
  - 使用专用 KWS 小模型（3.3M 参数），比流式 ASR Zipformer 轻量得多
  - 支持中文 / 英文 / 拼音，关键词以字符 + 拼音格式写入 keywords.txt
  - keywords.txt 修改后重启生效，无需重新训练

安装：
  pip install sherpa-onnx
  # ARM 预编译轮子：https://github.com/k2-fsa/sherpa-onnx/releases

模型下载：
  # 在 sherpa-onnx releases 页搜索 kws（keyword spotting）分类
  # 推荐：sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01（英文）
  #        sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01（中文）

keywords.txt 格式（每行一个唤醒词 + 对应拼音，拼音带声调数字）：
  小未 xiao3 wei4
  你好小未 ni3 hao3 xiao3 wei4
  hey robot
"""
import os
import queue
import numpy as np
import logging

from voicebot.kws.base_kws import BaseKWS
from voicebot.config.app_config import SherpaKWSConfig


class SherpaKWS(BaseKWS):

    SAMPLE_RATE = 16000

    def __init__(self, cfg: SherpaKWSConfig, wake_queue, loop):
        super().__init__(cfg.name, wake_queue, loop)
        self._cfg = cfg
        self._kws = None         # sherpa_onnx.KeywordSpotter
        self._stream = None      # sherpa_onnx.OnlineStream
        self._pcm_buf = b""
        # Sherpa KWS 以 chunk 方式处理，每次至少喂 10ms (160 samples)
        self._chunk_samples = int(self.SAMPLE_RATE * 0.01)  # 160

    def _init_model(self) -> None:
        """
        初始化 Sherpa-ONNX KeywordSpotter。
        model_path 是包含 encoder.onnx / decoder.onnx / joiner.onnx / tokens.txt 的目录。
        """
        import sherpa_onnx
        encoder = os.path.join(self._cfg.model_path, "encoder.onnx")
        decoder = os.path.join(self._cfg.model_path, "decoder.onnx")
        joiner  = os.path.join(self._cfg.model_path, "joiner.onnx")
        tokens  = os.path.join(self._cfg.model_path, "tokens.txt")

        self._kws = sherpa_onnx.KeywordSpotter(
            encoder=encoder,
            decoder=decoder,
            joiner=joiner,
            tokens=tokens,
            num_threads=self._cfg.num_threads,
            # keywords_file 每行一个词条，格式见模块文档
            keywords_file=self._cfg.keywords_file,
            keywords_score=self._cfg.keywords_score,
            keywords_threshold=self._cfg.keywords_score,  # 同 score，二者一致
            num_trailing_blanks=self._cfg.num_trailing_blanks,
        )
        self._stream = self._kws.create_stream()
        logging.info(
            f"[KWS:{self.name}] SherpaKWS ready, "
            f"model={self._cfg.model_path} keywords_file={self._cfg.keywords_file}"
        )

    def _run_loop(self) -> None:
        """
        推理线程：将 PCM 逐块喂入 KeywordSpotter，检查是否命中。
        Sherpa KWS 内部维护解码状态，命中后自动重置。
        """
        while self._running:
            chunk = self._pcm_queue.get()
            if chunk is None:
                break

            # int16 bytes → float32
            samples = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
            self._stream.accept_waveform(self.SAMPLE_RATE, samples)

            while self._kws.is_ready(self._stream):
                self._kws.decode(self._stream)

            result = self._kws.get_result(self._stream)
            if result.keyword.strip():
                # result.keyword 是命中的关键词文本
                self._notify(result.keyword.strip())
                # 重置 stream 防止同一词连续触发
                self._stream = self._kws.create_stream()

    def stop(self) -> None:
        super().stop()
        # 重置解码状态，为下一轮监听准备干净的 stream
        if self._kws:
            self._stream = self._kws.create_stream()
```

---

#### 7.3.5 自定义 ONNX KWS 插件（kws/custom_onnx_kws.py）

```python
# voicebot/kws/custom_onnx_kws.py
"""
通用自定义 ONNX KWS 插件。

适用场景：
  - 用户使用 PyTorch / TensorFlow 自训练的二分类 KWS 模型，导出为 ONNX
  - 模型输入：(1, frame_samples) float32 归一化 PCM
  - 模型输出：(1, num_classes) 各类别 softmax 概率

yaml 配置项：
  input_name: ONNX 输入张量名（用 Netron 打开模型查看）
  output_name: ONNX 输出张量名
  positive_class_index: 输出中"唤醒词命中"的类别 index（通常为 1）
  threshold: 置信度阈值，超过即触发
  frame_ms: 输入帧大小（ms），等于模型训练时的窗口长度

安装：
  pip install onnxruntime          # CPU 推理
  pip install onnxruntime-gpu      # GPU 推理（Jetson 使用 onnxruntime-gpu-jetson）
"""
import queue
import numpy as np
import logging

from voicebot.kws.base_kws import BaseKWS
from voicebot.config.app_config import CustomOnnxKWSConfig


class CustomOnnxKWS(BaseKWS):

    SAMPLE_RATE = 16000

    def __init__(self, cfg: CustomOnnxKWSConfig, wake_queue, loop):
        super().__init__(cfg.name, wake_queue, loop)
        self._cfg = cfg
        self._session = None
        self._frame_samples = int(self.SAMPLE_RATE * cfg.frame_ms / 1000)
        self._pcm_buf = b""

    def _init_model(self) -> None:
        """加载 ONNX 模型，创建推理 Session"""
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 1   # 轻量 KWS 单线程足够
        opts.inter_op_num_threads = 1
        self._session = ort.InferenceSession(
            self._cfg.model_path,
            sess_options=opts,
            providers=["CPUExecutionProvider"],
        )
        logging.info(
            f"[KWS:{self.name}] CustomOnnxKWS ready, "
            f"model={self._cfg.model_path} frame_ms={self._cfg.frame_ms} "
            f"threshold={self._cfg.threshold} label='{self._cfg.label}'"
        )

    def _run_loop(self) -> None:
        """
        推理线程：拼接 PCM 到 frame_samples 边界，逐帧推理并检查置信度。
        连续帧中只在第一次命中时触发 _notify，清空缓冲防止重复。
        """
        while self._running:
            chunk = self._pcm_queue.get()
            if chunk is None:
                break

            self._pcm_buf += chunk
            frame_bytes = self._frame_samples * 2   # int16

            while len(self._pcm_buf) >= frame_bytes:
                frame_data = self._pcm_buf[:frame_bytes]
                self._pcm_buf = self._pcm_buf[frame_bytes:]

                # 归一化为 float32，shape: (1, frame_samples)
                samples = (
                    np.frombuffer(frame_data, dtype=np.int16)
                    .astype(np.float32) / 32768.0
                ).reshape(1, -1)

                outputs = self._session.run(
                    [self._cfg.output_name],
                    {self._cfg.input_name: samples},
                )
                # outputs[0] shape: (1, num_classes)
                prob = float(outputs[0][0][self._cfg.positive_class_index])

                if prob >= self._cfg.threshold:
                    self._notify(self._cfg.label)
                    self._pcm_buf = b""   # 清空缓冲防止连续触发
                    break
```

---

#### 7.3.6 KWSManager（kws/kws_manager.py）

```python
# voicebot/kws/kws_manager.py
"""
KWS 管理器。
程序初始化时读取 config.yaml → wakeword.active_models，
为每个 enabled=true 的插件配置实例化对应的 KWS 子类，
统一管理生命周期（init / start / stop / feed_audio）。
SessionManager 只与 KWSManager 交互，不感知具体插件类型。

初始化流程：
  1. from_config(cfg) 读取 active_models 列表
  2. 按 type 实例化对应插件（工厂函数 _build_plugin）
  3. 调用每个插件的 init()（加载模型，阻塞，只执行一次）
  4. start() 启动所有插件线程，进入 IDLE 监听
  5. feed_audio() 广播 PCM 给所有插件
  6. wait_for_wake() 等待任意插件命中，返回 (keyword, model_name)
  7. stop() 在唤醒后调用，停止所有推理线程
"""
import asyncio
import logging
from typing import List

from voicebot.kws.base_kws import BaseKWS
from voicebot.kws.porcupine_kws import PorcupineKWS
from voicebot.kws.openwakeword_kws import OpenWakeWordKWS
from voicebot.kws.sherpa_kws import SherpaKWS
from voicebot.kws.custom_onnx_kws import CustomOnnxKWS
from voicebot.config.app_config import (
    AppConfig, WakewordConfig,
    PorcupineKWSConfig, OpenWakeWordKWSConfig,
    SherpaKWSConfig, CustomOnnxKWSConfig,
)


class KWSManager:

    def __init__(self):
        self._plugins: List[BaseKWS] = []
        # 所有插件共享同一 asyncio.Queue，谁先命中谁先通知
        self._wake_queue: asyncio.Queue = asyncio.Queue()
        self._loop: asyncio.AbstractEventLoop = None

    @classmethod
    def from_config(cls, cfg: AppConfig) -> "KWSManager":
        """
        工厂方法：读取配置，实例化并初始化所有 enabled 插件。
        在 asyncio 事件循环启动后调用（需要获取当前 loop）。
        """
        mgr = cls()
        mgr._loop = asyncio.get_event_loop()
        ww: WakewordConfig = cfg.wakeword

        if not ww.enabled:
            logging.warning("[KWSManager] wakeword disabled in config")
            return mgr

        if not ww.active_models:
            logging.warning("[KWSManager] no enabled KWS models in active_models")
            return mgr

        for model_cfg in ww.active_models:
            plugin = mgr._build_plugin(model_cfg)
            if plugin is None:
                continue
            try:
                plugin.init()           # 加载模型（阻塞，仅一次）
                mgr._plugins.append(plugin)
            except Exception as e:
                logging.error(f"[KWSManager] failed to init plugin '{model_cfg.name}': {e}")

        logging.info(
            f"[KWSManager] initialized {len(mgr._plugins)} plugin(s): "
            f"{[p.name for p in mgr._plugins]}"
        )
        return mgr

    def _build_plugin(self, model_cfg) -> BaseKWS | None:
        """
        插件工厂：根据配置类型实例化对应的 KWS 子类。
        新增插件类型时只需在此处添加一个 elif 分支。
        """
        q = self._wake_queue
        lp = self._loop
        if isinstance(model_cfg, PorcupineKWSConfig):
            return PorcupineKWS(model_cfg, q, lp)
        elif isinstance(model_cfg, OpenWakeWordKWSConfig):
            return OpenWakeWordKWS(model_cfg, q, lp)
        elif isinstance(model_cfg, SherpaKWSConfig):
            return SherpaKWS(model_cfg, q, lp)
        elif isinstance(model_cfg, CustomOnnxKWSConfig):
            return CustomOnnxKWS(model_cfg, q, lp)
        else:
            logging.error(f"[KWSManager] unknown config type: {type(model_cfg)}")
            return None

    def start(self) -> None:
        """启动所有插件的推理线程，进入唤醒监听状态"""
        for plugin in self._plugins:
            plugin.start()
        logging.info(f"[KWSManager] all {len(self._plugins)} plugin(s) started")

    def stop(self) -> None:
        """
        停止所有插件推理线程（唤醒命中后调用）。
        会话结束后调用 start() 重新进入监听。
        """
        for plugin in self._plugins:
            plugin.stop()
        # 清空 wake_queue，防止上一轮残留信号干扰下一轮
        while not self._wake_queue.empty():
            try:
                self._wake_queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        logging.info("[KWSManager] all plugins stopped")

    def feed_audio(self, pcm: bytes) -> None:
        """
        将 20ms PCM 广播给所有运行中的插件。
        由 AudioCapture 回调（非 async 线程）调用，各插件各自缓冲。
        """
        for plugin in self._plugins:
            plugin.feed_audio(pcm)

    async def wait_for_wake(self) -> tuple[str, str]:
        """
        异步等待任意插件命中唤醒词。
        返回 (keyword, model_name)，由 SessionManager 在 IDLE 状态下 await。
        阻塞直到有插件命中，不占用 CPU。
        """
        return await self._wake_queue.get()

    @property
    def plugin_count(self) -> int:
        return len(self._plugins)
```

### 7.4 AudioPlayer（audio/audio_player.py）— PulseAudio + paplay 实现

```python
# voicebot/audio/audio_player.py
"""
音频播放模块。
使用 subprocess paplay + stdin 管道方式播放流式 PCM S16LE 音频。

架构：
  豆包 TTS PCM → AudioPlayer.push() → paplay stdin → PulseAudio → 扬声器

选用 paplay 而非直接 ALSA 的原因：
  1. PulseAudio 是混音层，麦克风采集和 TTS 播放可以共存，不抢占设备
  2. paplay stdin 管道方案代码极简，paplay 崩溃不影响主进程
  3. 支持 --device 指定输出 sink，便于多声卡路由

安装：
  sudo apt install pulseaudio pulseaudio-utils
  pulseaudio --start
"""
import asyncio
import subprocess
import signal
import shutil
import time
import logging
from typing import Optional


class AudioPlayer:

    # PCM S16LE 参数（与豆包 TTS 输出对齐）
    SAMPLE_RATE = 24000
    CHANNELS = 1
    BYTES_PER_SAMPLE = 2  # int16

    def __init__(self, cfg):
        self.cfg = cfg
        self._proc: Optional[subprocess.Popen] = None
        self._current_reply_id: str = ""
        self._bytes_written: int = 0
        self._playing: bool = False

    def check_pulseaudio(self) -> bool:
        """程序启动时检查 PulseAudio 环境"""
        if not shutil.which("paplay"):
            logging.error("paplay not found. Install: sudo apt install pulseaudio-utils")
            return False
        result = subprocess.run(["pactl", "info"], capture_output=True)
        if result.returncode != 0:
            logging.warning("PulseAudio not running, trying to start...")
            subprocess.run(["pulseaudio", "--start", "--log-target=syslog"])
            time.sleep(0.5)
            result = subprocess.run(["pactl", "info"], capture_output=True)
            if result.returncode != 0:
                logging.error("Failed to start PulseAudio")
                return False
        logging.info("PulseAudio OK")
        return True

    def _spawn_paplay(self) -> bool:
        """
        启动 paplay 子进程，建立 stdin 管道。
        paplay --raw：直接接收原始 PCM，无需 WAV 文件头。
        """
        cmd = ["paplay", "--raw"] + self.cfg.audio_output.paplay_args.split()
        # 若指定了 PulseAudio sink，追加 --device
        if self.cfg.audio_output.pulse_sink:
            cmd += [f"--device={self.cfg.audio_output.pulse_sink}"]

        self._proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self._bytes_written = 0
        self._playing = True
        return self._proc.poll() is None

    def push(self, reply_id: str, pcm: bytes) -> None:
        """
        推入一帧 TTS 音频。
        reply_id 变化时（新一轮对话）自动重启 paplay 子进程。
        pcm: 原始 PCM S16LE 字节（来自 TTSResponse 事件）。
        """
        if reply_id != self._current_reply_id:
            self._kill_paplay()
            self._current_reply_id = reply_id
            self._spawn_paplay()

        if self._proc and self._proc.stdin:
            try:
                self._proc.stdin.write(pcm)
                self._proc.stdin.flush()
                self._bytes_written += len(pcm)
            except BrokenPipeError:
                # paplay 进程已结束（正常播放完毕）
                self._playing = False

    def stop(self) -> None:
        """
        立即停止播放（用户打断时调用）。
        向 paplay 子进程发送 SIGTERM，关闭 stdin 管道。
        """
        self._kill_paplay()

    def _kill_paplay(self) -> None:
        if self._proc:
            try:
                self._proc.send_signal(signal.SIGTERM)
                self._proc.wait(timeout=1)
            except Exception:
                pass
            self._proc = None
        self._playing = False
        self._bytes_written = 0

    @property
    def is_playing(self) -> bool:
        return self._playing and self._proc is not None and self._proc.poll() is None

    @property
    def current_reply_id(self) -> str:
        return self._current_reply_id

    @property
    def played_ms(self) -> int:
        """
        已播放毫秒数。
        计算方式：已写入 bytes / (采样率 × 声道数 × 字节深度) × 1000
        用于 ConversationTruncate 的 audio_end_ms 参数。
        """
        denom = self.SAMPLE_RATE * self.CHANNELS * self.BYTES_PER_SAMPLE
        return int(self._bytes_written * 1000 / denom) if denom else 0
```

### 7.5 AudioCapture（audio/audio_capture.py）

```python
# voicebot/audio/audio_capture.py
"""
麦克风采集模块。
使用 pyaudio 回调方式采集 PCM，通过 asyncio.Queue 桥接到事件循环。

采集参数（与豆包 ASR 上行格式一致）：
  格式：int16（PCM）
  采样率：16000 Hz
  声道：1（单声道）
  包大小：20ms = 320 samples = 640 bytes

安装：
  pip install pyaudio
  sudo apt install portaudio19-dev  # Ubuntu/Debian
"""
import asyncio
import pyaudio
import logging
from typing import Optional


class AudioCapture:

    SAMPLE_RATE = 16000
    CHANNELS = 1
    FORMAT = pyaudio.paInt16
    CHUNK_MS = 20
    CHUNK_SAMPLES = int(SAMPLE_RATE * CHUNK_MS / 1000)  # 320
    CHUNK_BYTES = CHUNK_SAMPLES * 2                       # 640 bytes / 包

    def __init__(self, cfg, loop: asyncio.AbstractEventLoop):
        self.cfg = cfg
        self._loop = loop
        self._pa = pyaudio.PyAudio()
        self._stream: Optional[pyaudio.Stream] = None
        # 线程安全桥接：pyaudio 回调（非 async）→ asyncio.Queue
        self._pcm_queue: asyncio.Queue = asyncio.Queue(maxsize=500)
        self._capturing = False

    def start(self) -> None:
        """开始采集"""
        self._capturing = True
        self._stream = self._pa.open(
            rate=self.SAMPLE_RATE,
            channels=self.CHANNELS,
            format=self.FORMAT,
            input=True,
            frames_per_buffer=self.CHUNK_SAMPLES,
            stream_callback=self._callback,
        )
        logging.info("[AudioCapture] started")

    def stop(self) -> None:
        """停止采集"""
        self._capturing = False
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None

    def _callback(self, in_data, frame_count, time_info, status):
        """pyaudio 回调（运行在独立线程），将 PCM 数据推入 asyncio.Queue"""
        if self._capturing:
            # call_soon_threadsafe 安全地从非 async 线程提交到事件循环
            self._loop.call_soon_threadsafe(
                self._pcm_queue.put_nowait, in_data
            )
        return (None, pyaudio.paContinue)

    async def read(self) -> bytes:
        """从队列读取一帧 PCM（640 bytes / 20ms），供上层 await"""
        return await self._pcm_queue.get()

    def __del__(self):
        self._pa.terminate()
```

### 7.6 CmdEngine（command/cmd_engine.py）

```python
# voicebot/command/cmd_engine.py
"""
命令词匹配引擎。
支持三级匹配：
  Level 1: keyword  - Python set O(1) 精确匹配
  Level 2: regex    - re.search 正则匹配
  Level 3: intent   - ChatResponse 流中的 [CMD:xxx] 意图标签扫描
"""
import re
import asyncio
import logging
from dataclasses import dataclass
from typing import Callable, Optional

# 意图标签正则：[CMD:arm_grab] 或 [CMD:navigate_to:工位A]
CMD_TAG_PATTERN = re.compile(r'\[CMD:([a-zA-Z_]+)(?::([^\]]+))?\]')


@dataclass
class CmdResult:
    cmd_id: str
    cmd_name: str
    arg: Optional[str]
    tts_confirm: str
    action: str              # "ros2_service" | "local_function" | "tts_only"
    ros2_service: str = ""
    ros2_request: dict = None
    function_name: str = ""


class CmdEngine:

    def __init__(self, cfg):
        self.cfg = cfg
        self._registry: dict[str, dict] = {}   # cmd_id → CommandDef
        self._keyword_index: dict[str, str] = {}  # keyword → cmd_id（Level 1）
        self._regex_rules: list[tuple] = []     # [(pattern, cmd_id), ...]
        self._buffer: str = ""
        self._on_cmd_cb: Optional[Callable] = None

    def load_registry(self, path: str) -> None:
        """从 commands.yaml 加载命令定义"""
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        for cmd in raw.get("commands", []):
            cmd_id = cmd["id"]
            self._registry[cmd_id] = cmd
            for pat in cmd.get("patterns", []):
                if pat["type"] == "keyword":
                    for w in pat.get("words", []):
                        self._keyword_index[w] = cmd_id
                elif pat["type"] == "regex":
                    self._regex_rules.append(
                        (re.compile(pat["expr"]), cmd_id)
                    )
        # 按 priority 排序（priority 越大越先匹配）
        self._regex_rules.sort(key=lambda x: -self._registry[x[1]].get("priority", 0))
        logging.info(f"[CmdEngine] loaded {len(self._registry)} commands")

    def set_callback(self, cb: Callable) -> None:
        self._on_cmd_cb = cb

    def on_asr_ended(self, full_text: str) -> None:
        """
        ASREnded 后调用（Level 1 & 2 匹配）。
        full_text: 用户本轮完整语音识别文本。
        Level 1/2 命中时，通过 asyncio.create_task 异步触发命令执行。
        """
        # Level 1: 关键词精确匹配
        for kw, cmd_id in self._keyword_index.items():
            if kw in full_text:
                cmd_def = self._registry[cmd_id]
                logging.info(f"[CmdEngine] Level1 match: {cmd_id} (keyword={kw})")
                asyncio.create_task(self._dispatch(cmd_def))
                return

        # Level 2: 正则匹配
        for pattern, cmd_id in self._regex_rules:
            if pattern.search(full_text):
                cmd_def = self._registry[cmd_id]
                logging.info(f"[CmdEngine] Level2 match: {cmd_id} (regex)")
                asyncio.create_task(self._dispatch(cmd_def))
                return

    def feed(self, chunk: str) -> None:
        """
        流式喂入 ChatResponse 文本（Level 3：意图标签扫描）。
        每收到一个文本 chunk 调用一次，扫描并剥离 [CMD:xxx] 标签。
        """
        self._buffer += chunk
        for m in CMD_TAG_PATTERN.finditer(self._buffer):
            cmd_name = m.group(1)
            cmd_arg = m.group(2)
            if cmd_name in self._registry:
                cmd_def = self._registry[cmd_name]
                logging.info(f"[CmdEngine] Level3 match: {cmd_name} (tag, arg={cmd_arg})")
                asyncio.create_task(self._dispatch(cmd_def, arg=cmd_arg))
        # 剥离已处理的标签（保证用户不可见）
        self._buffer = CMD_TAG_PATTERN.sub("", self._buffer)

    async def _dispatch(self, cmd_def: dict, arg: Optional[str] = None) -> None:
        if self._on_cmd_cb:
            result = CmdResult(
                cmd_id=cmd_def["id"],
                cmd_name=cmd_def["name"],
                arg=arg,
                tts_confirm=cmd_def.get("tts_confirm", ""),
                action=cmd_def.get("action", "tts_only"),
                ros2_service=cmd_def.get("ros2_service", ""),
                ros2_request=cmd_def.get("ros2_request", {}),
                function_name=cmd_def.get("function", ""),
            )
            await self._on_cmd_cb(result)

    def reset(self) -> None:
        """每轮对话开始时清空缓冲区"""
        self._buffer = ""
```

### 7.7 ROS2Bridge（ros2/ros2_bridge.py）

```python
# voicebot/ros2/ros2_bridge.py
"""
ROS2 服务调用桥接模块。
使用 rclpy（ROS2 官方 Python 客户端），在独立线程中运行 ROS2 节点，
通过 asyncio.to_thread 暴露异步接口给 SessionManager。

安装：
  ROS2 Humble/Iron 标准安装自带 rclpy
  source /opt/ros/humble/setup.bash
"""
import asyncio
import logging
from typing import Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("[ROS2Bridge] rclpy not found, ROS2 functions disabled")


class ROS2Bridge:

    def __init__(self, cfg):
        self.cfg = cfg
        self._node: Optional[object] = None
        self._enabled = cfg.ros2.enabled and ROS2_AVAILABLE

    def init(self) -> bool:
        """初始化 ROS2 节点（在启动时调用一次）"""
        if not self._enabled:
            return False
        rclpy.init()
        self._node = rclpy.create_node(self.cfg.ros2.node_name)
        logging.info(f"[ROS2Bridge] node '{self.cfg.ros2.node_name}' initialized")
        return True

    async def query_arm_status(self) -> Tuple[str, str]:
        """
        异步查询机械臂状态和工位。
        返回 (arm_status, workstation)，用于 SayHello 情境填充。
        在独立线程中执行（避免阻塞 asyncio 事件循环）。
        """
        if not self._enabled:
            return "待机", "默认工位"
        return await asyncio.to_thread(self._query_arm_status_sync)

    def _query_arm_status_sync(self) -> Tuple[str, str]:
        """同步版本，在 to_thread 线程中执行"""
        # 此处调用实际 ROS2 服务或 topic 获取状态
        # 示例：向 /arm_controller/status 服务发送请求
        try:
            # from your_msgs.srv import ArmStatus
            # client = self._node.create_client(ArmStatus, "/arm_controller/status")
            # ... 调用逻辑 ...
            return "待机", "工位A"
        except Exception as e:
            logging.error(f"[ROS2Bridge] query_arm_status failed: {e}")
            return "未知", "未知"

    async def call_arm_service(self, command: str, arg: Optional[str] = None) -> bool:
        """
        异步调用机械臂控制服务。
        command: "grab" | "release" | "home" | "emergency_stop"
        返回 True 表示成功。
        """
        if not self._enabled:
            logging.info(f"[ROS2Bridge] [MOCK] arm command: {command}")
            return True
        return await asyncio.to_thread(self._call_arm_sync, command, arg)

    def _call_arm_sync(self, command: str, arg: Optional[str]) -> bool:
        """同步 ROS2 服务调用，在 to_thread 线程中执行"""
        try:
            timeout = self.cfg.ros2.service_timeout_s
            # from your_msgs.srv import ArmControl
            # client = self._node.create_client(ArmControl, self.cfg.ros2.arm_service)
            # if not client.wait_for_service(timeout_sec=timeout):
            #     return False
            # req = ArmControl.Request()
            # req.command = command
            # future = client.call_async(req)
            # rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            # return future.result().success
            logging.info(f"[ROS2Bridge] arm command '{command}' sent (mock)")
            return True
        except Exception as e:
            logging.error(f"[ROS2Bridge] arm service error: {e}")
            return False

    async def call_nav_service(self, target: str) -> bool:
        """异步调用导航服务"""
        if not self._enabled:
            logging.info(f"[ROS2Bridge] [MOCK] navigate to: {target}")
            return True
        return await asyncio.to_thread(self._call_nav_sync, target)

    def _call_nav_sync(self, target: str) -> bool:
        try:
            logging.info(f"[ROS2Bridge] navigate to '{target}' (mock)")
            return True
        except Exception as e:
            logging.error(f"[ROS2Bridge] nav service error: {e}")
            return False

    def shutdown(self) -> None:
        if self._enabled and rclpy.ok():
            rclpy.shutdown()
```

### 7.8 SessionManager（session/session_manager.py）

```python
# voicebot/session/session_manager.py
"""
会话状态机。
使用 asyncio 协程协调所有模块的生命周期，驱动完整对话流程。
"""
import asyncio
import logging
from enum import Enum, auto
from voicebot.audio.audio_capture import AudioCapture
from voicebot.audio.audio_player import AudioPlayer
from voicebot.kws.kws_manager import KWSManager
from voicebot.websocket.doubao_ws_client import DoubaoWSClient
from voicebot.websocket.session_config import build_start_session_payload
from voicebot.command.cmd_engine import CmdEngine, CmdResult
from voicebot.session.silence_detector import SilenceDetector, DialogPhase
from voicebot.ros2.ros2_bridge import ROS2Bridge


class State(Enum):
    IDLE = auto()
    WAKEUP = auto()
    DIALOG = auto()
    CMD_EXEC = auto()


class SessionManager:

    def __init__(self, cfg):
        self.cfg = cfg
        self.state = State.IDLE
        self._loop = asyncio.get_event_loop()

        # 初始化各模块
        self.audio_capture = AudioCapture(cfg, self._loop)
        self.audio_player = AudioPlayer(cfg)
        # KWSManager 在事件循环启动后通过工厂方法构建，初始化时读取 active_models 并加载所有插件
        self.kws_manager = KWSManager.from_config(cfg)
        self.ws_client = DoubaoWSClient(cfg)
        self.cmd_engine = CmdEngine(cfg)
        self.silence_detector = SilenceDetector(
            cfg.silence.wakeup_timeout_s,
            cfg.silence.dialog_timeout_s,
        )
        self.ros2_bridge = ROS2Bridge(cfg)
        self._dialog_id: str = cfg.context.dialog_id

    async def run(self) -> None:
        """主循环：启动采集、KWS 插件，等待唤醒词后建立对话"""
        self.audio_player.check_pulseaudio()
        self.ros2_bridge.init()
        # KWSManager 已在 __init__ 中通过 from_config 完成模型加载
        # 此处直接启动所有插件推理线程
        self.audio_capture.start()
        self.kws_manager.start()
        logging.info(
            f"[SessionManager] IDLE — {self.kws_manager.plugin_count} KWS plugin(s) listening..."
        )

        # 启动 PCM 广播给 KWS 管理器
        asyncio.create_task(self._broadcast_audio_to_kws())

        while True:
            keyword, model_name = await self.kws_manager.wait_for_wake()
            logging.info(f"[SessionManager] Wake! keyword='{keyword}' from model='{model_name}'")
            await self._handle_wakeup()

    async def _broadcast_audio_to_kws(self) -> None:
        """将麦克风 PCM 广播给 KWSManager（IDLE 状态下持续运行）"""
        while True:
            pcm = await self.audio_capture.read()
            if self.state == State.IDLE:
                self.kws_manager.feed_audio(pcm)

    async def _handle_wakeup(self) -> None:
        """唤醒处理：建立 WebSocket 会话，进入 DIALOG 状态"""
        self.state = State.WAKEUP
        self.kws_manager.stop()   # 停止所有插件推理线程，防止唤醒期间重复触发

        # 播放唤醒确认音
        await self.audio_player.play_file(self.cfg.wakeword.ack_audio_path)

        # 建立 WebSocket
        await self.ws_client.connect()
        await self.ws_client.send_start_connection()
        # 注册服务端事件回调
        self._register_ws_callbacks()

        # 发送 StartSession
        payload = build_start_session_payload(self.cfg)
        if self._dialog_id:
            payload["dialog"]["extra"]["dialog_id"] = self._dialog_id
        await self.ws_client.send_start_session(payload)

        # 等待 SessionStarted
        self._dialog_id = await self.ws_client.wait_session_started()
        logging.info(f"[SessionManager] Session started, dialog_id={self._dialog_id}")

        # 切换到 DIALOG 状态
        self.state = State.DIALOG
        asyncio.create_task(self._audio_upload_loop())
        asyncio.create_task(self._send_say_hello())
        asyncio.create_task(
            self.silence_detector.run(DialogPhase.WAKEUP)
        )

    def _register_ws_callbacks(self) -> None:
        """注册所有服务端事件回调"""
        ws = self.ws_client
        ws.on(DoubaoWSClient.EVENT_ASR_INFO, self._on_asr_info)
        ws.on(DoubaoWSClient.EVENT_ASR_RESPONSE, self._on_asr_response)
        ws.on(DoubaoWSClient.EVENT_ASR_ENDED, self._on_asr_ended)
        ws.on(DoubaoWSClient.EVENT_CHAT_RESPONSE, self._on_chat_response)
        ws.on(DoubaoWSClient.EVENT_TTS_RESPONSE, self._on_tts_response)
        ws.on(DoubaoWSClient.EVENT_TTS_ENDED, self._on_tts_ended)
        ws.on(DoubaoWSClient.EVENT_DIALOG_COMMON_ERROR, self._on_error)

    async def _audio_upload_loop(self) -> None:
        """持续读取麦克风 PCM，发送给豆包（20ms/包）"""
        while self.state in (State.DIALOG, State.CMD_EXEC):
            pcm = await self.audio_capture.read()
            await self.ws_client.send_audio_chunk(pcm)

    async def _send_say_hello(self) -> None:
        """SessionStarted 后异步发送 SayHello（见 §5.2.1）"""
        from datetime import datetime
        hour = datetime.now().hour
        if 6 <= hour < 12:
            time_str = f"上午{hour}点"
        elif 12 <= hour < 18:
            time_str = f"下午{hour - 12}点"
        else:
            time_str = f"晚上{hour}点"

        try:
            arm_status, workstation = await asyncio.wait_for(
                self.ros2_bridge.query_arm_status(), timeout=0.5
            )
        except asyncio.TimeoutError:
            arm_status, workstation = "待机", "默认工位"

        prompt = self.cfg.persona.say_hello_prompt
        prompt = prompt.replace("{time}", time_str)
        prompt = prompt.replace("{workstation}", workstation)
        prompt = prompt.replace("{arm_status}", arm_status)
        await self.ws_client.send_say_hello(prompt)

    async def _on_asr_info(self, data: dict) -> None:
        """用户开始说话 → 停止 TTS 播放 + 发送 ConversationTruncate"""
        self.audio_player.stop()
        if (self.cfg.context.enable_conversation_truncate
                and self.audio_player.is_playing):
            await self.ws_client.send_conversation_truncate(
                self.audio_player.current_reply_id,
                self.audio_player.played_ms,
            )
        self.cmd_engine.reset()
        self.silence_detector.on_activity()

    async def _on_asr_response(self, data: dict) -> None:
        text = data.get("asr", {}).get("text", "")
        logging.debug(f"[ASR] {text}")
        self.silence_detector.on_activity()

    async def _on_asr_ended(self, data: dict) -> None:
        full_text = data.get("asr", {}).get("text", "")
        logging.info(f"[ASR Ended] '{full_text}'")
        # Level 1 & 2 命令词匹配
        self.cmd_engine.on_asr_ended(full_text)

    async def _on_chat_response(self, data: dict) -> None:
        chunk = data.get("chat", {}).get("text", "")
        # Level 3 意图标签扫描
        self.cmd_engine.feed(chunk)

    async def _on_tts_response(self, pcm: bytes) -> None:
        """收到 TTS 音频帧，推入播放器"""
        reply_id = getattr(self, "_current_reply_id", "default")
        self.audio_player.push(reply_id, pcm)

    async def _on_tts_ended(self, data: dict) -> None:
        status_code = str(data.get("status_code", ""))
        # 退出意图信号（豆包 26.03.07 新特性）
        if status_code == "20000002":
            logging.info("[SessionManager] exit intent detected")
            await self._finish_session()

    async def _on_error(self, data: dict) -> None:
        code = data.get("status_code")
        msg = data.get("message", "")
        logging.error(f"[DoubaoError] code={code} msg={msg}")
        # 5xx 错误统一触发重连
        if str(code).startswith("5"):
            await self._finish_session()

    async def _handle_cmd(self, result: CmdResult) -> None:
        """命令词执行（由 CmdEngine 回调触发）"""
        self.state = State.CMD_EXEC
        # 注入确认语音（绕过模型，直接 TTS）
        if result.tts_confirm:
            await self.ws_client.send_chat_tts_text(result.tts_confirm)

        # 并行执行 ROS2 服务
        if result.action == "ros2_service":
            try:
                success = await asyncio.wait_for(
                    self._call_ros2(result),
                    timeout=self.cfg.command.exec_timeout_s,
                )
                done_text = self.cfg.command.exec_done_prompt if success \
                    else self.cfg.command.exec_fail_prompt
                await self.ws_client.send_chat_tts_text(done_text)
            except asyncio.TimeoutError:
                logging.error(f"[CMD] {result.cmd_id} timeout")
                await self.ws_client.send_chat_tts_text(self.cfg.command.exec_fail_prompt)

        self.state = State.DIALOG

    async def _call_ros2(self, result: CmdResult) -> bool:
        if "arm" in result.ros2_service:
            return await self.ros2_bridge.call_arm_service(
                result.ros2_request.get("command", ""),
                result.arg,
            )
        elif "navigation" in result.ros2_service:
            return await self.ros2_bridge.call_nav_service(result.arg or "")
        return False

    async def _finish_session(self) -> None:
        """结束会话，回到 IDLE"""
        self.state = State.IDLE
        await self.ws_client.send_finish_session()
        await self.ws_client.send_finish_connection()
        await self.ws_client.close()
        self.silence_detector.stop()
        # 重新启动所有 KWS 插件线程，进入下一轮 IDLE 监听
        self.kws_manager.start()
        logging.info("[SessionManager] back to IDLE")
```

### 7.9 程序入口（main.py）

```python
# voicebot/main.py
"""
VoiceBot Python 版入口。
加载配置，构建 SessionManager，启动 asyncio 事件循环。

运行：
  python -m voicebot.main --config config/config.yaml

关闭：
  Ctrl+C → asyncio 捕获 KeyboardInterrupt → 正常发送 FinishSession/FinishConnection
"""
import asyncio
import argparse
import signal
import logging
from voicebot.config.app_config import AppConfig
from voicebot.session.session_manager import SessionManager
from voicebot.utils.logger import setup_logger


async def main_async(cfg: AppConfig) -> None:
    session_mgr = SessionManager(cfg)
    # 注册命令词回调
    session_mgr.cmd_engine.load_registry(cfg.command.registry_path)
    session_mgr.cmd_engine.set_callback(session_mgr._handle_cmd)

    # 注册 SIGINT/SIGTERM 退出处理
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(
            sig,
            lambda: asyncio.create_task(session_mgr._finish_session())
        )

    await session_mgr.run()


def main():
    parser = argparse.ArgumentParser(description="VoiceBot Python Edition")
    parser.add_argument("--config", default="config/config.yaml", help="配置文件路径")
    args = parser.parse_args()

    cfg = AppConfig.from_yaml(args.config)
    setup_logger(cfg)
    logging.info(f"[VoiceBot] starting, scene={cfg.scene} model={cfg.model_version}")

    asyncio.run(main_async(cfg))


if __name__ == "__main__":
    main()
```

---

## 8. 豆包 API 关键机制挖掘与建议

### 8.1 🌟 流式上下文对齐（26.02.26 新特性）

**机制**：`ConversationTruncate` 事件支持基于客户端实际播报进度截断上下文，只把已播放内容告诉模型，未播完的部分不算进历史。

**Python 实现要点**：
```python
# 在 _on_asr_info() 中，停止播放后立即发送
await self.ws_client.send_conversation_truncate(
    item_id=self.audio_player.current_reply_id,
    audio_end_ms=self.audio_player.played_ms,  # 已播字节数换算的毫秒数
)
```
**需开启**：`config.yaml → context.enable_conversation_truncate: true`

### 8.2 🌟 退出意图识别信号（26.03.07 新特性）

**机制**：开启 `enable_user_query_exit` 后，豆包感知用户说"再见/拜拜/结束"，在 `TTSEnded` 的 `status_code` 中返回 `"20000002"`。

**Python 实现**：
```python
async def _on_tts_ended(self, data: dict) -> None:
    if str(data.get("status_code", "")) == "20000002":
        await self._finish_session()
```

### 8.3 🌟 音频级热修复（O2.0/SC2.0）

使用 2.0 版本模型自动享受，无需额外配置。TN 转写修正（如数字读音）、发音问题在线修复，适合机器人场景中涉及数字/专业术语的语音输出。

### 8.4 🌟 ChatTTSText 主动注入（绕过模型生成）

**核心价值**：命令词命中后跳过 LLM，直接注入确认语音，实现 < 100ms 的极速响应。

**使用约束**（来自豆包 API 文档）：
- 必须在 `ASREnded` 之后才能发送（否则服务端拒绝）
- `start=True` 标记文本开始，`end=True` 标记文本结束
- 适合固定短句确认；长回复仍走 ChatResponse 流

```python
# ASREnded 后命中命令词时：
await self.ws_client.send_chat_tts_text("好的，正在抓取", start=True, end=True)
# 同时并行触发 ROS2（语音 + 执行真正并行，< 100ms 响应）
asyncio.create_task(self.ros2_bridge.call_arm_service("grab"))
```

### 8.5 🌟 SayHello 情境感知（SessionStarted 后）

**使用约束**：必须在收到 `SessionStarted` 之后才能调用，模型据此生成自然开场白，而非生硬的"你好有什么可以帮你"。完整实现见 §5.2.1。

### 8.6 🌟 UpdateConfig 运行时热更新

机器人从工位 A 移动到工位 B 时，无需重启会话即可动态更新位置、音色、人设：

```python
async def update_location(self, longitude: float, latitude: float) -> None:
    """机器人移动后实时更新位置信息"""
    payload = json.dumps({
        "dialog": {
            "extra": {
                "location": {
                    "longitude": longitude,
                    "latitude": latitude,
                }
            }
        }
    }).encode()
    await self.ws_client._ws.send(
        self.ws_client._build_frame(DoubaoWSClient.EVENT_UPDATE_CONFIG, payload)
    )
```

### 8.7 🌟 外部 RAG 注入（ChatRAGText）

用户问"这个工件的规格是多少" → 本地数据库查询 → `ChatRAGText` 注入规格参数 → 模型自然语言输出：

```python
async def inject_rag(self, context_text: str) -> None:
    payload = json.dumps({"text": context_text}).encode()
    await self.ws_client._ws.send(
        self.ws_client._build_frame(DoubaoWSClient.EVENT_CHAT_RAG_TEXT, payload)
    )
```

### 8.8 🌟 KWS 插件化选型指南

#### 四种插件对比

| 维度 | Porcupine | OpenWakeWord | Sherpa-ONNX KWS | Custom ONNX |
|------|-----------|-------------|----------------|-------------|
| 开源 | ❌ 商业 | ✅ Apache-2.0 | ✅ Apache-2.0 | ✅ 用户自有 |
| 模型大小 | < 1MB / 词 | ~20MB（含特征提取器） | ~15MB（3.3M int8） | 用户决定 |
| CPU 占用（ARM） | < 1%（最低） | ~3-5% | ~3-5% | 视模型大小 |
| 中文唤醒词 | ✅ 需控制台训练 .ppn | ✅ 需训练 | ✅ 内置中文模型 | ✅ 用户决定 |
| 英文唤醒词 | ✅ 内置多词 | ✅ 社区模型库 | ✅ 内置英文模型 | ✅ 用户决定 |
| 词库修改 | 重新训练（5分钟） | 重新训练（5分钟） | 改 keywords.txt（立即） | 重新训练 |
| 误唤醒率 | 低（商业调优） | 中（依赖样本质量） | 低（CTC 解码精确匹配） | 用户调优 |
| 并联支持 | ✅ 多词同时监听 | ✅ 多模型并联 | ✅ 关键词文件多词 | ✅ 独立线程 |

#### 推荐选型策略

```
嵌入式 ARM（Jetson Nano / RPi 4B）需长时待机：
  → porcupine（< 1% CPU，商业调优，.ppn 自定义中文词）

快速原型 / 开源优先 / 中文场景：
  → openwakeword（社区模型 + Colab 5分钟训练）
  → 或 sherpa_kws（keywords.txt 改完即生效，无需训练）

多唤醒词并联（如中文"小未" + 英文"hey robot"）：
  → porcupine_zh + oww_hey_robot 两个插件同时 enabled: true
  → 任一命中均触发唤醒

用户自研算法团队 / 定制声学特征：
  → custom_onnx（自训练 PyTorch 模型 → 导出 ONNX → yaml 配置路径）
```

#### yaml 快速切换示例

```yaml
# 场景 A：嵌入式生产环境，仅 Porcupine
active_models:
  - name: "porcupine_main"
    type: "porcupine"
    enabled: true
    access_key: "YOUR_KEY"
    keyword_paths: ["models/kws/porcupine/xiao_wei_zh.ppn"]
    sensitivities: [0.6]

# 场景 B：开发调试，OpenWakeWord + Sherpa 双保险
active_models:
  - name: "oww_xiao_wei"
    type: "openwakeword"
    enabled: true
    model_path: "models/kws/oww/xiao_wei.onnx"
    threshold: 0.5
    frame_ms: 80
  - name: "sherpa_kws_zh"
    type: "sherpa_kws"
    enabled: true
    model_path: "models/kws/sherpa/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01"
    keywords_file: "models/kws/sherpa/keywords_zh.txt"
    keywords_score: 0.3

# 场景 C：国际化，中英双语唤醒
active_models:
  - name: "porcupine_zh"
    type: "porcupine"
    enabled: true
    access_key: "YOUR_KEY"
    keyword_paths: ["models/kws/porcupine/xiao_wei_zh.ppn"]
    sensitivities: [0.6]
  - name: "oww_hey_robot_en"
    type: "openwakeword"
    enabled: true
    model_path: "models/kws/oww/hey_robot_en.onnx"
    threshold: 0.55
    frame_ms: 80
```

### 8.9 ⚠️ 重要工程注意事项

| 注意点 | Python 实现说明 |
|--------|---------------|
| FinishSession 必须等 ACK | `await ws.send_finish_session()` 后等 `SessionFinished` 回调，再 `close()` |
| 音频包大小 | 严格 20ms/包，16k int16 = 640 bytes；`asyncio.sleep(0.02)` 精确控速 |
| QPM 限流 | StartSession 每分钟 ≤ 60 次，注意唤醒频率 |
| keep_alive 模式 | 麦克风有静音键时务必在 StartSession payload 中设置 `input_mod: keep_alive` |
| ChatTTSText 时机 | 必须在 `ASREnded` 回调触发后才能调用，否则服务端返回错误 |
| pyaudio 回调线程安全 | pyaudio 回调在非 async 线程，必须用 `call_soon_threadsafe` 提交到事件循环 |
| asyncio.create_task 时机 | 必须在事件循环运行中调用；模块初始化阶段不能用 `create_task` |
| ROS2 + asyncio 共存 | rclpy spin 不能在 asyncio 事件循环线程中调用，使用 `asyncio.to_thread` 包裹同步调用 |

---

## 9. 依赖与环境说明

### 9.1 Python 版本

Python 3.10+（`asyncio` TaskGroup、`match` 语句等特性）

### 9.2 requirements.txt

```
# WebSocket 通信
websockets>=12.0

# 配置文件解析
PyYAML>=6.0

# 音频采集
pyaudio>=0.2.13

# ── KWS 唤醒引擎（按需安装，不需要的插件可不装）────────────────────────
# Porcupine（商业，极低 CPU）
pvporcupine>=3.0.2

# OpenWakeWord（开源，社区模型）
openwakeword>=0.6.0

# Sherpa-ONNX KWS（开源，关键词文件驱动）
# ARM 设备用预编译轮子：
#   pip install https://github.com/k2-fsa/sherpa-onnx/releases/download/.../sherpa_onnx-*.whl
sherpa-onnx>=1.10.0

# Custom ONNX（ONNX 推理引擎，自定义模型也需要）
onnxruntime>=1.17.0          # CPU 版
# onnxruntime-gpu>=1.17.0    # GPU 版（Jetson 用 onnxruntime-gpu-jetson）
# ─────────────────────────────────────────────────────────────────────────

# 数值计算（所有 KWS 插件共用）
numpy>=1.24.0

# 日志
loguru>=0.7.0

# HTTP 客户端（克隆音色注册、RAG 接口）
httpx>=0.27.0

# ROS2（通过系统安装，不通过 pip）
# source /opt/ros/humble/setup.bash
# rclpy 随 ROS2 安装自带

# 测试
pytest>=8.0
pytest-asyncio>=0.23
```

### 9.3 安装步骤

```bash
# 1. 系统依赖（Ubuntu 22.04）
sudo apt update
sudo apt install -y \
  portaudio19-dev \          # pyaudio 依赖
  pulseaudio pulseaudio-utils \  # 音频播放
  libpulse-dev

# 2. Python 虚拟环境
python3.10 -m venv .venv
source .venv/bin/activate

# 3. Python 依赖
pip install -r requirements.txt

# 4. KWS 插件（按 yaml 中启用的插件选择安装）
# Porcupine（商业，极低 CPU）：
pip install pvporcupine
# OpenWakeWord（开源）：
pip install openwakeword
# Sherpa-ONNX KWS（开源，x86_64）：
pip install sherpa-onnx
# Sherpa-ONNX KWS（ARM aarch64，从 releases 页下载对应轮子）：
# pip install sherpa_onnx-1.x.y-cp310-cp310-linux_aarch64.whl
# Custom ONNX（通用推理引擎，以上插件也依赖）：
pip install onnxruntime

# 5. ROS2（已安装则跳过）
source /opt/ros/humble/setup.bash

# 6. 确认 PulseAudio 运行
pulseaudio --start
pactl list sinks short   # 查看可用输出设备

# 7. 运行
python -m voicebot.main --config config/config.yaml
```

### 9.4 测试

```bash
# 单元测试（pytest-asyncio）
pytest tests/ -v

# 二进制协议测试
pytest tests/test_protocol.py -v

# 命令词引擎测试
pytest tests/test_cmd_engine.py -v

# WS 集成测试（需要真实 API Key）
pytest tests/test_ws_client.py -v -m integration
```

---

## 10. 后续迭代路线图

### Phase 1 — MVP（第 1-2 周）
- [ ] WebSocket 连接 + 二进制协议帧组装（`binary_protocol.py`）
- [ ] StartSession / FinishSession 基础流程
- [ ] PCM 音频上传（`AudioCapture` → `send_audio_chunk`）
- [ ] OGG Opus / PCM S16LE 接收 + paplay 播放
- [ ] 配置文件驱动的 `build_start_session_payload()`
- [ ] 基础命令词关键词匹配（Level 1）

### Phase 2 — 完整对话（第 3-4 周）
- [ ] Sherpa-ONNX 流式唤醒引擎（`ASRWakeEngine` 多语言并行）
- [ ] asyncio 静默超时检测（`SilenceDetector`）
- [ ] `ChatTTSText` 快速响应注入（ASREnded 后立即调用）
- [ ] `ConversationTruncate` 流式上下文截断（打断时发送）
- [ ] 退出意图信号处理（`TTSEnded` status_code 20000002）
- [ ] 正则命令词匹配（Level 2）和意图标签扫描（Level 3）

### Phase 3 — 智能增强（第 5-6 周）
- [ ] 声纹识别集成（ECAPA-TDNN ONNX 推理）
- [ ] ROS2 服务桥接（机械臂 + 导航，`asyncio.to_thread` 包裹）
- [ ] `SayHello` 情境感知问候（时间 + ROS2 状态填充）
- [ ] 外部 RAG 知识库对接（`ChatRAGText` 注入）
- [ ] 多方言 / 多语言 ASR 测试

### Phase 4 — 产品化（第 7-8 周）
- [ ] 克隆音色注册自动化脚本（`scripts/register_clone_voice.py`）
- [ ] 联网搜索 + `UpdateConfig` 位置动态更新
- [ ] 多场景配置快速切换（robot / home / demo）
- [ ] 热词表控制台配置联动
- [ ] 压力测试（QPM / TPM 限流验证）
- [ ] 完整错误处理与自动重连（指数退避）
- [ ] SIGUSR1 热重载配置（`active_languages` 无缝切换）

---

## 附录：案例实现——完整机械臂抓取对话链路

以下展示从用户说"帮我抓取工件"到机械臂执行的完整 Python 异步链路：

```python
# 完整链路时序（Python asyncio）

# 1. [IDLE] AudioCapture 持续采集，KWSManager 广播给所有插件线程
pcm = await audio_capture.read()           # 640 bytes / 20ms
kws_manager.feed_audio(pcm)                # 广播给 PorcupineKWS / OpenWakeWordKWS / 等

# 2. 某插件线程推理命中（以 PorcupineKWS 为例）
# → _notify("xiao_wei_zh.ppn")
# → loop.call_soon_threadsafe(wake_queue.put_nowait, ("xiao_wei_zh.ppn", "porcupine_zh"))

# 3. [SessionManager] 等到唤醒信号
keyword, model_name = await kws_manager.wait_for_wake()
# keyword="xiao_wei_zh.ppn", model_name="porcupine_zh"
kws_manager.stop()   # 停止所有插件线程
# 4. 建立 WebSocket 会话
await ws.connect()
await ws.send_start_session(build_start_session_payload(cfg))
dialog_id = await ws.wait_session_started()

# 5. 异步发送 SayHello（不阻塞采集）
asyncio.create_task(_send_say_hello())
# → "用户唤醒了我，现在是下午3点，机械臂当前位于工位A，状态为待机。"
# → 模型回复："下午好！工位A一切就绪，请说指令。"

# 6. 用户说"帮我抓取工件"
# [DoubaoWSClient] 接收 ASREnded 事件
async def _on_asr_ended(data):
    full_text = "帮我抓取工件"
    cmd_engine.on_asr_ended(full_text)    # Level 1/2 匹配

# 7. CmdEngine Level 2 正则命中 (抓取|夹取).*(物品|工件)
async def _dispatch(cmd_def, arg=None):
    # Level 1/2 命中后，在 ASREnded 回调中立即执行：

    # 7a. 注入确认语音（绕过 LLM，< 50ms 合成）
    await ws.send_chat_tts_text("好的，开始抓取目标。", start=True, end=True)

    # 7b. 并行触发 ROS2（语音 + 动作同时发生）
    asyncio.create_task(ros2_bridge.call_arm_service("grab"))

# 8. [AudioPlayer] 收到 TTSResponse，推入 paplay stdin
async def _on_tts_response(pcm: bytes):
    audio_player.push(reply_id="tts_001", pcm=pcm)
    # paplay 子进程：PCM → PulseAudio → 扬声器输出"好的，开始抓取目标。"

# 9. 同时 ROS2 服务返回成功
success = await ros2_bridge.call_arm_service("grab")   # True
await ws.send_chat_tts_text("执行完成。", start=True, end=True)

# 10. [SessionManager] 继续 DIALOG，等待下一轮用户输入
# 若用户说"再见" → TTSEnded status_code=20000002
# → await _finish_session() → IDLE → 重启 ASRWakeEngine
```

---

*文档版本：v2.1-Python | 2026-05（唤醒模块重构：多 KWS 小模型插件化选配，移除流式 ASR 唤醒方案）*
*v2.0 基础：基于 C++ v1.4 全量重构为 Python asyncio 版*
*参考豆包 RealtimeAPI 文档版本：26.03.07*

**v2.1 唤醒模块变更摘要**：
- 移除 Sherpa-ONNX 流式 Zipformer 大模型 ASR 唤醒方案
- 新增 `BaseKWS` 抽象基类，统一插件接口
- 新增四类 KWS 插件：`PorcupineKWS` / `OpenWakeWordKWS` / `SherpaKWS`（专用 KWS 小模型）/ `CustomOnnxKWS`
- 新增 `KWSManager`：工厂模式实例化、并联运行、统一唤醒出口
- yaml `wakeword.active_models` 列表驱动选型，`enabled: true/false` 热切换
- `AppConfig` 新增四个插件数据类，`from_yaml` 按 `type` 字段分发解析