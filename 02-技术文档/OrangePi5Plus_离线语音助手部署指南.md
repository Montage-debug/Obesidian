# Orange Pi 5 Plus 离线语音助手部署指南

> **目标平台**: Orange Pi 5 Plus (RK3588)  
> **核心框架**: [sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx) (v1.12.27+)  
> **完全离线**: KWS 语音唤醒 → VAD 静音检测 → ASR 语音转文字 → TTS 文字转语音  
> **文档版本**: 2025-01  

---

## 目录

1. [项目概述](#1-项目概述)
2. [硬件平台](#2-硬件平台)
3. [系统架构与流程](#3-系统架构与流程)
4. [为什么选择 sherpa-onnx](#4-为什么选择-sherpa-onnx)
5. [系统环境准备](#5-系统环境准备)
6. [安装 sherpa-onnx](#6-安装-sherpa-onnx)
7. [模型下载与配置](#7-模型下载与配置)
   - [7.1 KWS 语音唤醒模型](#71-kws-语音唤醒模型)
   - [7.2 VAD 静音检测模型](#72-vad-静音检测模型)
   - [7.3 ASR 语音识别模型](#73-asr-语音识别模型)
   - [7.4 TTS 语音合成模型](#74-tts-语音合成模型)
8. [模型对比与选型建议](#8-模型对比与选型建议)
9. [自定义唤醒词配置](#9-自定义唤醒词配置)
10. [集成代码示例](#10-集成代码示例)
    - [10.1 KWS 唤醒检测独立测试](#101-kws-唤醒检测独立测试)
    - [10.2 VAD + ASR 语音识别测试](#102-vad--asr-语音识别测试)
    - [10.3 TTS 语音合成测试](#103-tts-语音合成测试)
    - [10.4 完整流水线 (KWS → VAD → ASR → TTS)](#104-完整流水线-kws--vad--asr--tts)
11. [验收标准](#11-验收标准)
12. [诊断命令与故障排查](#12-诊断命令与故障排查)
13. [参考仓库与文档链接](#13-参考仓库与文档链接)

---

## 1. 项目概述

本文档描述如何在 **Orange Pi 5 Plus** (RK3588 SoC) 上搭建一套 **完全离线** 的语音交互系统，用于按摩机器人人机交互。整个流水线不依赖任何云端服务，所有推理均在本地 CPU 完成。

**核心功能**：
| 模块 | 功能 | 离线支持 |
|------|------|----------|
| **KWS** (Keyword Spotting) | 语音唤醒，检测自定义唤醒词 | ✅ 完全离线 |
| **VAD** (Voice Activity Detection) | 静音检测，切割有效语音段 | ✅ 完全离线 |
| **ASR** (Automatic Speech Recognition) | 语音转文字 | ✅ 完全离线 |
| **TTS** (Text-to-Speech) | 文字转语音 | ✅ 完全离线 |

---

## 2. 硬件平台

### Orange Pi 5 Plus 规格

| 项目 | 参数 |
|------|------|
| **SoC** | Rockchip RK3588 |
| **CPU** | 4x Cortex-A76 @ 2.4GHz + 4x Cortex-A55 @ 1.8GHz |
| **NPU** | 6 TOPS (RKNN, 可选加速) |
| **RAM** | 8GB / 16GB / 32GB LPDDR4x |
| **架构** | ARM64 / aarch64 |
| **OS** | Ubuntu 22.04 / Debian 12 (推荐 Ubuntu 22.04) |

**购买链接**: [Orange Pi 官网](http://www.orangepi.cn/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5-Plus.html)

### 推荐外设

- **USB 麦克风**: ReSpeaker USB Mic Array / 普通 USB 声卡 + 3.5mm 麦克风
- **音箱**: 3.5mm 有源音箱 或 USB 音箱
- **存储**: 至少 32GB eMMC 或 TF 卡（模型总计约 500MB-1GB）

---

## 3. 系统架构与流程

```
┌──────────────────────────────────────────────────────────┐
│                     Orange Pi 5 Plus                      │
│                                                          │
│  ┌─────────┐   ┌─────┐   ┌─────┐   ┌─────┐   ┌──────┐  │
│  │ 麦克风   │──→│ KWS │──→│ VAD │──→│ ASR │──→│ 业务  │  │
│  │ (16kHz) │   │唤醒  │   │切割  │   │识别  │   │ 逻辑  │  │
│  └─────────┘   └─────┘   └─────┘   └─────┘   └──┬───┘  │
│                                                   │      │
│                                              ┌────▼───┐  │
│  ┌─────────┐                                 │  TTS   │  │
│  │  音箱   │◀────────────────────────────────│ 合成   │  │
│  └─────────┘                                 └────────┘  │
└──────────────────────────────────────────────────────────┘
```

**工作流程**：
1. **KWS 常驻监听**: 低功耗持续监听麦克风，检测到唤醒词（如 "你好小艾"）后激活
2. **VAD 端点检测**: 自动检测语音起始和结束，切割出有效音频段
3. **ASR 语音识别**: 将音频段转为文本
4. **业务逻辑处理**: 根据识别文本生成响应（可接入本地 LLM 或规则引擎）
5. **TTS 语音播报**: 将文本响应合成语音并通过音箱播放

---

## 4. 为什么选择 sherpa-onnx

| 优势 | 说明 |
|------|------|
| **全栈覆盖** | 一个框架同时提供 KWS/VAD/ASR/TTS/说话人识别等功能 |
| **ARM64 原生支持** | 官方 wheel 直接支持 aarch64，RK3588 明确验证 |
| **ONNX Runtime** | 高性能推理引擎，支持多线程，无需 GPU |
| **模型丰富** | 100+ 预训练模型，覆盖中文、英文、多语言 |
| **API 完善** | Python/C++/C/Java/Go 等多语言 API |
| **活跃维护** | 10.5k+ GitHub Stars，持续更新，v1.12.27 (2025-01) |
| **Apache-2.0** | 商用友好许可证 |
| **RKNN 可选** | 官方已支持 RKNN NPU 加速（可选，CPU 已足够） |

**官方文档**: https://k2-fsa.github.io/sherpa/onnx/index.html

---

## 5. 系统环境准备

### 5.1 更新系统

```bash
sudo apt update && sudo apt upgrade -y
```

### 5.2 安装系统依赖

```bash
# 音频相关
sudo apt install -y alsa-utils pulseaudio portaudio19-dev

# 编译工具（如需从源码编译）
sudo apt install -y build-essential cmake git wget curl

# Python 相关
sudo apt install -y python3 python3-pip python3-venv python3-dev
```

### 5.3 创建 Python 虚拟环境

```bash
# 创建项目目录
mkdir -p ~/voice_assistant && cd ~/voice_assistant

# 创建虚拟环境
python3 -m venv .venv
source .venv/bin/activate

# 升级 pip
pip install --upgrade pip setuptools wheel
```

### 5.4 验证音频设备

```bash
# 列出录音设备
arecord -l

# 列出播放设备
aplay -l

# 测试录音（5秒）
arecord -d 5 -f S16_LE -r 16000 -c 1 test_record.wav

# 测试播放
aplay test_record.wav
```

> **重要**: 确保麦克风和音箱工作正常后再进行后续步骤。

---

## 6. 安装 sherpa-onnx

### 方式一：pip 安装（推荐）

```bash
# 激活虚拟环境
source ~/voice_assistant/.venv/bin/activate

# 安装 sherpa-onnx（自动匹配 aarch64 wheel）
pip install sherpa-onnx

# 安装音频处理依赖
pip install soundfile numpy sounddevice
```

### 方式二：从源码编译（需要定制功能时）

```bash
git clone https://github.com/k2-fsa/sherpa-onnx.git
cd sherpa-onnx
mkdir build && cd build

cmake \
  -DSHERPA_ONNX_ENABLE_PYTHON=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DSHERPA_ONNX_ENABLE_CHECK=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  ..

make -j$(nproc)
```

### 验证安装

```bash
python3 -c "import sherpa_onnx; print(sherpa_onnx.__version__)"
# 预期输出: 1.12.27 或更高版本
```

---

## 7. 模型下载与配置

### 模型存储目录结构

```bash
mkdir -p ~/voice_assistant/models/{kws,vad,asr,tts}
cd ~/voice_assistant/models
```

最终目录结构：
```
~/voice_assistant/models/
├── kws/
│   └── sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/
├── vad/
│   └── silero_vad.onnx
├── asr/
│   └── sherpa-onnx-streaming-paraformer-bilingual-zh-en/
└── tts/
    ├── vits-melo-tts-zh_en/        # 或选择其他 TTS 模型
    └── matcha-icefall-zh-baker/     # 备选
```

---

### 7.1 KWS 语音唤醒模型

#### 推荐模型：sherpa-onnx-kws-zipformer-zh-en-3M (2025-12-20)

支持 **中文 + 英文** 唤醒词，是最新版本。

| 项目 | 参数 |
|------|------|
| **模型名** | sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20 |
| **语言** | 中文 + 英文 |
| **模型大小** | encoder ~4.4MB (int8) / ~12MB (fp32) |
| **延迟** | chunk-8: 160ms / chunk-16: 320ms |
| **打包大小** | 31.4MB (含所有变体) |

#### 下载命令

```bash
cd ~/voice_assistant/models/kws

wget https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
tar xvf sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
rm sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
```

#### 验证文件结构

```bash
ls -lh sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/
# 应包含:
# encoder-epoch-99-avg-1-chunk-16-left-128.onnx   (~12MB fp32)
# encoder-epoch-99-avg-1-chunk-16-left-128.int8.onnx (~4.4MB int8, 推荐)
# decoder-epoch-99-avg-1-chunk-16-left-128.onnx
# joiner-epoch-99-avg-1-chunk-16-left-128.onnx
# tokens.txt
# keywords.txt          (预置唤醒词列表)
```

#### 备选 KWS 模型

| 模型 | 语言 | 备注 |
|------|------|------|
| `sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01` | 仅中文 | 使用 `ppinyin` Token 类型 |
| `sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01` | 仅英文 | 英文场景使用 |

---

### 7.2 VAD 静音检测模型

#### 推荐模型：Silero VAD

| 项目 | 参数 |
|------|------|
| **模型名** | silero_vad.onnx |
| **模型大小** | ~2.2MB |
| **延迟** | <10ms |
| **准确率** | >99% 端点检测 |

#### 下载命令

```bash
cd ~/voice_assistant/models/vad

wget https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx
```

#### 验证

```bash
ls -lh silero_vad.onnx
# 预期大小: 约 2.2MB
```

---

### 7.3 ASR 语音识别模型

#### 推荐模型：Paraformer 流式双语 (中+英)

| 项目 | 参数 |
|------|------|
| **模型名** | sherpa-onnx-streaming-paraformer-bilingual-zh-en |
| **语言** | 中文 + 英文 |
| **模型大小** | ~220MB |
| **类型** | 流式 (Streaming) |
| **RTF (RK3588)** | < 0.3 (实时率) |

#### 下载命令

```bash
cd ~/voice_assistant/models/asr

wget https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-paraformer-bilingual-zh-en.tar.bz2
tar xvf sherpa-onnx-streaming-paraformer-bilingual-zh-en.tar.bz2
rm sherpa-onnx-streaming-paraformer-bilingual-zh-en.tar.bz2
```

#### 验证文件结构

```bash
ls -lh sherpa-onnx-streaming-paraformer-bilingual-zh-en/
# 应包含:
# encoder.int8.onnx
# decoder.int8.onnx
# tokens.txt
```

#### 备选 ASR 模型

| 模型 | 语言 | 大小 | 特点 |
|------|------|------|------|
| `sherpa-onnx-streaming-zipformer-bilingual-zh-en-2023-02-20` | 中+英 | ~100MB | Zipformer 架构，可 RKNN 加速 |
| `sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17` | 中英日韩粤 | ~230MB | 非流式，FunASR SenseVoice |
| `sherpa-onnx-paraformer-zh-2023-09-14` | 中文 | ~230MB | 非流式，高准确率 |

---

### 7.4 TTS 语音合成模型

#### 推荐方案 A：vits-melo-tts-zh_en (中英文, 体积适中)

| 项目 | 参数 |
|------|------|
| **模型名** | vits-melo-tts-zh_en |
| **语言** | 中文 + 英文 |
| **说话人数** | 1 |
| **模型大小** | 163MB |
| **采样率** | 44100 Hz |
| **RTF (RPi4, 2线程)** | 3.877 |

> **注意**: RPi4 的 RTF 较高（3.877），但 RK3588 的 A76 大核性能约为 RPi4 Cortex-A72 的 2-3 倍，预计 RK3588 RTF ≈ 1.3-1.9，满足实时要求。

#### 下载命令

```bash
cd ~/voice_assistant/models/tts

wget https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-melo-tts-zh_en.tar.bz2
tar xvf vits-melo-tts-zh_en.tar.bz2
rm vits-melo-tts-zh_en.tar.bz2
```

#### 验证文件结构

```bash
ls -lh vits-melo-tts-zh_en/
# 应包含:
# model.onnx       (~163MB)
# lexicon.txt      (~6.5MB)
# tokens.txt
# date.fst
# number.fst
# phone.fst
# dict/
```

---

#### 推荐方案 B：matcha-icefall-zh-baker (中文, 高自然度, 需 Vocoder)

| 项目 | 参数 |
|------|------|
| **模型名** | matcha-icefall-zh-baker |
| **语言** | 中文 |
| **说话人数** | 1 (女声) |
| **模型大小** | 72MB (声学) + 51MB (Vocoder) |
| **采样率** | 22050 Hz |
| **RTF (RPi4, 2线程)** | 0.536 |

> Matcha TTS + Vocos Vocoder 架构更先进, RTF 更低。但只支持中文，不支持英文。

#### 下载命令

```bash
cd ~/voice_assistant/models/tts

# 声学模型
wget https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/matcha-icefall-zh-baker.tar.bz2
tar xvf matcha-icefall-zh-baker.tar.bz2
rm matcha-icefall-zh-baker.tar.bz2

# Vocoder (必须)
wget https://github.com/k2-fsa/sherpa-onnx/releases/download/vocoder-models/vocos-22khz-univ.onnx
```

---

#### 推荐方案 C：aishell3 VITS (中文, 多说话人, 最轻量)

| 项目 | 参数 |
|------|------|
| **模型名** | vits-icefall-zh-aishell3 |
| **语言** | 中文 |
| **说话人数** | 174 |
| **模型大小** | 29MB |
| **采样率** | 8000 Hz |
| **RTF (RPi4, 2线程)** | 0.220 |

> 模型最小 (仅 29MB)，RTF 最低。但采样率仅 8kHz，语音质量略低。

#### 下载命令

```bash
cd ~/voice_assistant/models/tts

wget https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-icefall-zh-aishell3.tar.bz2
tar xvf vits-icefall-zh-aishell3.tar.bz2
rm vits-icefall-zh-aishell3.tar.bz2
```

---

## 8. 模型对比与选型建议

### KWS 模型对比

| 模型 | 语言 | 大小 (int8) | 延迟 | 推荐场景 |
|------|------|-------------|------|----------|
| **zh-en-3M-2025-12-20** ⭐ | 中+英 | ~4.4MB | 160ms | **通用推荐** |
| wenetspeech-3.3M | 中文 | ~3.3MB | 160ms | 纯中文场景 |
| gigaspeech-3.3M | 英文 | ~3.3MB | 160ms | 纯英文场景 |

### TTS 模型对比

| 模型 | 语言 | 大小 | RTF (RPi4×2) | 采样率 | 推荐场景 |
|------|------|------|-------------|--------|----------|
| **matcha-zh-baker** ⭐ | 中文 | 123MB | **0.536** | 22kHz | **中文最佳质量** |
| **vits-melo-tts-zh_en** ⭐ | 中+英 | 163MB | 3.877 | 44kHz | **中英文混合** |
| vits-icefall-zh-aishell3 | 中文 | 29MB | **0.220** | 8kHz | 极致轻量 |
| sherpa-onnx-vits-zh-ll | 中文 | 115MB | 2.494 | 16kHz | 5 说话人可选 |
| vits-zh-hf-fanchen-C | 中文 | 116MB | 2.451 | 16kHz | 187 说话人 |

### 推荐组合

**按摩机器人推荐配置**:
```
KWS:  sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20 (int8, chunk-16)
VAD:  silero_vad.onnx
ASR:  sherpa-onnx-streaming-paraformer-bilingual-zh-en
TTS:  matcha-icefall-zh-baker + vocos-22khz-univ.onnx （纯中文）
      或 vits-melo-tts-zh_en （中英文混合）
```

**磁盘占用估算**:
| 模块 | 模型大小 |
|------|----------|
| KWS (int8) | ~10MB |
| VAD | ~2MB |
| ASR | ~220MB |
| TTS (Matcha) | ~123MB |
| **总计** | **~355MB** |

---

## 9. 自定义唤醒词配置

### 9.1 唤醒词格式

KWS 模型使用 `keywords.txt` 文件定义唤醒词。每行格式为：

```
拼音序列 @显示文本
```

示例:
```
nǐ hǎo xiǎo ài @你好小艾
kāi shǐ àn mó @开始按摩
tíng zhǐ àn mó @停止按摩
```

### 9.2 使用 text2token 工具生成拼音

```bash
# 安装 sherpa-onnx CLI 工具（pip 已安装）
# 使用 text2token 将中文文本转为拼音 Token

# 对于 zh-en-3M 模型，使用 phone+ppinyin Token 类型
sherpa-onnx-cli text2token \
  --tokens=~/voice_assistant/models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/tokens.txt \
  --tokens-type=phone+ppinyin \
  --text="你好小艾"
```

### 9.3 创建自定义唤醒词文件

```bash
cat > ~/voice_assistant/models/kws/my_keywords.txt << 'EOF'
nǐ hǎo xiǎo ài @你好小艾
kāi shǐ àn mó @开始按摩
tíng zhǐ àn mó @停止按摩
jiā dà lì dù @加大力度
jiǎn xiǎo lì dù @减小力度
EOF
```

### 9.4 关于唤醒词的建议

- **字数**: 推荐 4-6 个字，太短容易误唤醒，太长用户不便
- **声调**: 确保拼音包含声调标记（如 nǐ 而不是 ni）
- **测试**: 每个新唤醒词都要经过充分的唤醒率和误唤醒率测试

---

## 10. 集成代码示例

### 10.1 KWS 唤醒检测独立测试

```python
#!/usr/bin/env python3
"""KWS 唤醒词检测独立测试"""

import sherpa_onnx
import sounddevice as sd
import numpy as np

# ============ 配置 ============
MODEL_DIR = "models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"

config = sherpa_onnx.KeywordSpotterConfig(
    feat_config=sherpa_onnx.FeatureExtractorConfig(sample_rate=16000),
    model_config=sherpa_onnx.OnlineModelConfig(
        transducer=sherpa_onnx.OnlineTransducerModelConfig(
            encoder=f"{MODEL_DIR}/encoder-epoch-99-avg-1-chunk-16-left-128.int8.onnx",
            decoder=f"{MODEL_DIR}/decoder-epoch-99-avg-1-chunk-16-left-128.onnx",
            joiner=f"{MODEL_DIR}/joiner-epoch-99-avg-1-chunk-16-left-128.onnx",
        ),
        tokens=f"{MODEL_DIR}/tokens.txt",
        num_threads=2,
        provider="cpu",
    ),
    keywords_file=f"{MODEL_DIR}/keywords.txt",  # 或使用自定义 my_keywords.txt
    keywords_threshold=0.25,  # 唤醒阈值，越低越灵敏，越高越严格
    keywords_score=1.0,
    num_trailing_blanks=1,
    max_active_paths=4,
)

kws = sherpa_onnx.KeywordSpotter(config)
stream = kws.create_stream()

# ============ 实时麦克风监听 ============
SAMPLE_RATE = 16000
CHUNK_DURATION = 0.1  # 100ms per chunk

print("🎤 KWS 唤醒检测已启动，请说唤醒词...")

def audio_callback(indata, frames, time, status):
    if status:
        print(f"Audio status: {status}")

    # 转换音频数据
    samples = indata[:, 0].astype(np.float32)
    stream.accept_waveform(SAMPLE_RATE, samples)

    while kws.is_ready(stream):
        kws.decode_stream(stream)

    result = kws.get_result(stream)
    if result:
        print(f"✅ 检测到唤醒词: {result}")

try:
    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        blocksize=int(SAMPLE_RATE * CHUNK_DURATION),
        callback=audio_callback,
    ):
        print("按 Ctrl+C 退出")
        import time
        while True:
            time.sleep(0.1)
except KeyboardInterrupt:
    print("\n已退出 KWS 检测")
```

---

### 10.2 VAD + ASR 语音识别测试

```python
#!/usr/bin/env python3
"""VAD + ASR 语音识别测试"""

import sherpa_onnx
import sounddevice as sd
import numpy as np
from collections import deque

# ============ VAD 配置 ============
vad_config = sherpa_onnx.VadModelConfig()
vad_config.silero_vad.model = "models/vad/silero_vad.onnx"
vad_config.silero_vad.threshold = 0.5
vad_config.silero_vad.min_silence_duration = 0.5   # 静音 0.5s 认为说完
vad_config.silero_vad.min_speech_duration = 0.25
vad_config.silero_vad.max_speech_duration = 30.0    # 最长 30s
vad_config.sample_rate = 16000

vad = sherpa_onnx.VoiceActivityDetector(vad_config, buffer_size_in_seconds=60)

# ============ ASR 配置 (流式 Paraformer) ============
ASR_DIR = "models/asr/sherpa-onnx-streaming-paraformer-bilingual-zh-en"

recognizer = sherpa_onnx.OnlineRecognizer.from_paraformer(
    paraformer=f"{ASR_DIR}/encoder.int8.onnx",
    tokens=f"{ASR_DIR}/tokens.txt",
    num_threads=2,
    sample_rate=16000,
    feature_dim=80,
    decoding_method="greedy_search",
)

# ============ 实时识别 ============
SAMPLE_RATE = 16000

print("🎤 语音识别已启动，请开始说话...")

def audio_callback(indata, frames, time, status):
    samples = indata[:, 0].astype(np.float32)
    vad.accept_waveform(samples)

    while not vad.empty():
        speech = vad.front
        vad.pop()

        # 创建 ASR stream 并识别
        asr_stream = recognizer.create_stream()
        asr_stream.accept_waveform(SAMPLE_RATE, speech.samples)

        # 添加尾部静音以触发 endpoint
        tail_paddings = np.zeros(int(0.3 * SAMPLE_RATE), dtype=np.float32)
        asr_stream.accept_waveform(SAMPLE_RATE, tail_paddings)
        asr_stream.input_finished()

        while recognizer.is_ready(asr_stream):
            recognizer.decode_stream(asr_stream)

        result = recognizer.get_result(asr_stream)
        text = result.text.strip()
        if text:
            print(f"📝 识别结果: {text}")

try:
    with sd.InputStream(
        samplerate=SAMPLE_RATE,
        channels=1,
        dtype="float32",
        blocksize=int(SAMPLE_RATE * 0.1),
        callback=audio_callback,
    ):
        print("按 Ctrl+C 退出")
        import time
        while True:
            time.sleep(0.1)
except KeyboardInterrupt:
    print("\n已退出语音识别")
```

---

### 10.3 TTS 语音合成测试

#### 方案 A：使用 vits-melo-tts-zh_en

```python
#!/usr/bin/env python3
"""TTS 语音合成测试 - VITS MeloTTS"""

import sherpa_onnx
import soundfile as sf

TTS_DIR = "models/tts/vits-melo-tts-zh_en"

tts_config = sherpa_onnx.OfflineTtsConfig(
    model=sherpa_onnx.OfflineTtsModelConfig(
        vits=sherpa_onnx.OfflineTtsVitsModelConfig(
            model=f"{TTS_DIR}/model.onnx",
            lexicon=f"{TTS_DIR}/lexicon.txt",
            tokens=f"{TTS_DIR}/tokens.txt",
        ),
        num_threads=2,
        provider="cpu",
    ),
    rule_fsts=f"{TTS_DIR}/date.fst,{TTS_DIR}/number.fst,{TTS_DIR}/phone.fst",
)

tts = sherpa_onnx.OfflineTts(tts_config)

# 合成测试
text = "你好，欢迎使用按摩机器人。请问需要什么服务？"
audio = tts.generate(text, sid=0, speed=1.0)

# 保存为 WAV
sf.write("test_tts_output.wav", audio.samples, audio.sample_rate)
print(f"✅ TTS 合成完成: test_tts_output.wav")
print(f"   采样率: {audio.sample_rate} Hz")
print(f"   时长: {len(audio.samples) / audio.sample_rate:.2f} 秒")
```

#### 方案 B：使用 matcha-icefall-zh-baker

```python
#!/usr/bin/env python3
"""TTS 语音合成测试 - Matcha TTS"""

import sherpa_onnx
import soundfile as sf

TTS_DIR = "models/tts/matcha-icefall-zh-baker"
VOCODER = "models/tts/vocos-22khz-univ.onnx"

tts_config = sherpa_onnx.OfflineTtsConfig(
    model=sherpa_onnx.OfflineTtsModelConfig(
        matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
            acoustic_model=f"{TTS_DIR}/model-steps-3.onnx",
            vocoder=VOCODER,
            lexicon=f"{TTS_DIR}/lexicon.txt",
            tokens=f"{TTS_DIR}/tokens.txt",
        ),
        num_threads=2,
        provider="cpu",
    ),
    rule_fsts=f"{TTS_DIR}/phone.fst,{TTS_DIR}/date.fst,{TTS_DIR}/number.fst",
)

tts = sherpa_onnx.OfflineTts(tts_config)

text = "你好，欢迎使用按摩机器人。请问需要什么服务？"
audio = tts.generate(text, sid=0, speed=1.0)

sf.write("test_tts_matcha.wav", audio.samples, audio.sample_rate)
print(f"✅ TTS 合成完成: test_tts_matcha.wav")
print(f"   采样率: {audio.sample_rate} Hz")
print(f"   时长: {len(audio.samples) / audio.sample_rate:.2f} 秒")
```

---

### 10.4 完整流水线 (KWS → VAD → ASR → TTS)

```python
#!/usr/bin/env python3
"""
完整离线语音助手流水线
KWS 唤醒 → VAD 端点检测 → ASR 语音识别 → TTS 语音播报

适用平台: Orange Pi 5 Plus (RK3588)
"""

import sherpa_onnx
import sounddevice as sd
import soundfile as sf
import numpy as np
import time
import threading
import queue

# =============================================
#               配置区域
# =============================================
SAMPLE_RATE = 16000
MODELS_DIR = "models"

# ----- KWS 配置 -----
KWS_DIR = f"{MODELS_DIR}/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"

kws_config = sherpa_onnx.KeywordSpotterConfig(
    feat_config=sherpa_onnx.FeatureExtractorConfig(sample_rate=SAMPLE_RATE),
    model_config=sherpa_onnx.OnlineModelConfig(
        transducer=sherpa_onnx.OnlineTransducerModelConfig(
            encoder=f"{KWS_DIR}/encoder-epoch-99-avg-1-chunk-16-left-128.int8.onnx",
            decoder=f"{KWS_DIR}/decoder-epoch-99-avg-1-chunk-16-left-128.onnx",
            joiner=f"{KWS_DIR}/joiner-epoch-99-avg-1-chunk-16-left-128.onnx",
        ),
        tokens=f"{KWS_DIR}/tokens.txt",
        num_threads=2,
        provider="cpu",
    ),
    keywords_file=f"{KWS_DIR}/keywords.txt",
    keywords_threshold=0.25,
    keywords_score=1.0,
    num_trailing_blanks=1,
    max_active_paths=4,
)

# ----- VAD 配置 -----
vad_config = sherpa_onnx.VadModelConfig()
vad_config.silero_vad.model = f"{MODELS_DIR}/vad/silero_vad.onnx"
vad_config.silero_vad.threshold = 0.5
vad_config.silero_vad.min_silence_duration = 0.5
vad_config.silero_vad.min_speech_duration = 0.25
vad_config.silero_vad.max_speech_duration = 30.0
vad_config.sample_rate = SAMPLE_RATE

# ----- ASR 配置 -----
ASR_DIR = f"{MODELS_DIR}/asr/sherpa-onnx-streaming-paraformer-bilingual-zh-en"

# ----- TTS 配置 (使用 VITS MeloTTS) -----
TTS_DIR = f"{MODELS_DIR}/tts/vits-melo-tts-zh_en"

tts_config = sherpa_onnx.OfflineTtsConfig(
    model=sherpa_onnx.OfflineTtsModelConfig(
        vits=sherpa_onnx.OfflineTtsVitsModelConfig(
            model=f"{TTS_DIR}/model.onnx",
            lexicon=f"{TTS_DIR}/lexicon.txt",
            tokens=f"{TTS_DIR}/tokens.txt",
        ),
        num_threads=2,
        provider="cpu",
    ),
    rule_fsts=f"{TTS_DIR}/date.fst,{TTS_DIR}/number.fst,{TTS_DIR}/phone.fst",
)

# =============================================
#              初始化模块
# =============================================
print("⏳ 初始化语音模块...")
t0 = time.time()

kws = sherpa_onnx.KeywordSpotter(kws_config)
vad = sherpa_onnx.VoiceActivityDetector(vad_config, buffer_size_in_seconds=60)

recognizer = sherpa_onnx.OnlineRecognizer.from_paraformer(
    paraformer=f"{ASR_DIR}/encoder.int8.onnx",
    tokens=f"{ASR_DIR}/tokens.txt",
    num_threads=2,
    sample_rate=SAMPLE_RATE,
    feature_dim=80,
    decoding_method="greedy_search",
)

tts = sherpa_onnx.OfflineTts(tts_config)

print(f"✅ 所有模块初始化完成 ({time.time() - t0:.1f}s)")

# =============================================
#              状态机
# =============================================
class VoiceAssistant:
    STATE_IDLE = "idle"           # 待机，仅运行 KWS
    STATE_LISTENING = "listening"  # 已唤醒，正在录音
    STATE_PROCESSING = "processing"  # 处理中
    STATE_SPEAKING = "speaking"    # 播报中

    def __init__(self):
        self.state = self.STATE_IDLE
        self.kws_stream = kws.create_stream()
        self.audio_queue = queue.Queue()

    def process_command(self, text: str) -> str:
        """
        业务逻辑：根据识别文本生成回复。
        这里是简单的规则引擎示例，可替换为本地 LLM。
        """
        text = text.strip()
        if not text:
            return "抱歉，我没有听清楚，请再说一遍。"

        # 简单命令匹配
        if "开始" in text and "按摩" in text:
            return "好的，开始按摩。请放松身体。"
        elif "停止" in text or "暂停" in text:
            return "已停止按摩。"
        elif "加大" in text and "力度" in text:
            return "好的，正在加大力度。"
        elif "减小" in text or "轻一点" in text:
            return "好的，正在减小力度。"
        elif "什么" in text or "哪" in text:
            return f"你说的是：{text}。请问还需要什么帮助？"
        else:
            return f"收到指令：{text}"

    def speak(self, text: str):
        """TTS 播报"""
        self.state = self.STATE_SPEAKING
        print(f"🔊 播报: {text}")

        audio = tts.generate(text, sid=0, speed=1.0)
        sd.play(audio.samples, samplerate=audio.sample_rate)
        sd.wait()  # 等待播放完成

        self.state = self.STATE_IDLE
        print("🎤 回到待机状态，等待唤醒...")

    def run(self):
        """主循环"""
        print("=" * 50)
        print("🤖 离线语音助手已启动")
        print(f"   唤醒词: 参见 {KWS_DIR}/keywords.txt")
        print("   按 Ctrl+C 退出")
        print("=" * 50)
        print("🎤 待机中，请说唤醒词...")

        def audio_callback(indata, frames, time_info, status):
            if status:
                print(f"⚠️  Audio: {status}")
            samples = indata[:, 0].astype(np.float32)

            if self.state == self.STATE_IDLE:
                # KWS 唤醒检测
                self.kws_stream.accept_waveform(SAMPLE_RATE, samples)
                while kws.is_ready(self.kws_stream):
                    kws.decode_stream(self.kws_stream)

                result = kws.get_result(self.kws_stream)
                if result:
                    print(f"\n✅ 唤醒词命中: {result}")
                    self.state = self.STATE_LISTENING
                    vad.reset()
                    print("🎤 请说指令...")

            elif self.state == self.STATE_LISTENING:
                # VAD 端点检测
                vad.accept_waveform(samples)

                while not vad.empty():
                    speech = vad.front
                    vad.pop()
                    self.audio_queue.put(speech.samples)

        try:
            with sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype="float32",
                blocksize=int(SAMPLE_RATE * 0.1),
                callback=audio_callback,
            ):
                while True:
                    if not self.audio_queue.empty():
                        speech_samples = self.audio_queue.get()
                        self.state = self.STATE_PROCESSING
                        print("⏳ 正在识别...")

                        # ASR 识别
                        t_asr = time.time()
                        asr_stream = recognizer.create_stream()
                        asr_stream.accept_waveform(SAMPLE_RATE, speech_samples)

                        tail = np.zeros(int(0.3 * SAMPLE_RATE), dtype=np.float32)
                        asr_stream.accept_waveform(SAMPLE_RATE, tail)
                        asr_stream.input_finished()

                        while recognizer.is_ready(asr_stream):
                            recognizer.decode_stream(asr_stream)

                        result = recognizer.get_result(asr_stream)
                        text = result.text.strip()
                        asr_time = time.time() - t_asr

                        print(f"📝 识别结果: \"{text}\" ({asr_time:.2f}s)")

                        # 生成回复并播报
                        reply = self.process_command(text)
                        self.speak(reply)

                    time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n\n👋 语音助手已退出")


if __name__ == "__main__":
    assistant = VoiceAssistant()
    assistant.run()
```

---

## 11. 验收标准

### 11.1 性能指标

| 指标 | 达标标准 | 测试方法 |
|------|----------|----------|
| **KWS CPU 占用** | < 5% (空闲监听) | `htop` 持续观察 30s |
| **KWS 唤醒延迟** | < 500ms | 说完唤醒词到系统响应的时间 |
| **KWS 唤醒率** | > 95% (安静环境) | 测试 20 次唤醒，统计成功率 |
| **KWS 误唤醒率** | < 1次/小时 | 播放 1 小时普通对话/新闻 |
| **VAD 端点检测延迟** | < 600ms | 停止说话到端点触发的时间 |
| **ASR RTF** | < 0.5 (RK3588) | 计算 (识别时间 / 音频时长) |
| **ASR 中文字符错误率** | < 15% (安静环境) | 朗读 20 个标准句子 |
| **TTS RTF** | < 1.0 (RK3588) | 计算 (合成时间 / 音频时长) |
| **TTS 可懂度** | 清晰可理解 | 人工听测 |
| **端到端响应时间** | < 3s | 说完指令到开始播报的时间 |
| **总内存占用** | < 500MB | 全模块加载后 `free -h` |

### 11.2 批量 ASR 准确率测试

```python
#!/usr/bin/env python3
"""ASR 准确率批量测试脚本"""

import sherpa_onnx
import soundfile as sf
import time

# 准备测试数据: 录制 20 个标准句子对应的 WAV 文件
# 格式: [(wav_path, expected_text), ...]
TEST_CASES = [
    ("test_data/01.wav", "今天天气怎么样"),
    ("test_data/02.wav", "请开始按摩"),
    ("test_data/03.wav", "加大力度"),
    # ... 添加更多测试句对
]

ASR_DIR = "models/asr/sherpa-onnx-streaming-paraformer-bilingual-zh-en"
recognizer = sherpa_onnx.OnlineRecognizer.from_paraformer(
    paraformer=f"{ASR_DIR}/encoder.int8.onnx",
    tokens=f"{ASR_DIR}/tokens.txt",
    num_threads=2,
    sample_rate=16000,
    feature_dim=80,
    decoding_method="greedy_search",
)

correct = 0
total = len(TEST_CASES)
total_rtf = 0

for wav_path, expected in TEST_CASES:
    audio, sr = sf.read(wav_path, dtype="float32")

    stream = recognizer.create_stream()
    t0 = time.time()
    stream.accept_waveform(sr, audio)
    stream.input_finished()
    while recognizer.is_ready(stream):
        recognizer.decode_stream(stream)
    elapsed = time.time() - t0

    result = recognizer.get_result(stream).text.strip()
    duration = len(audio) / sr
    rtf = elapsed / duration
    total_rtf += rtf

    match = result == expected
    if match:
        correct += 1

    status = "✅" if match else "❌"
    print(f"{status} 期望: \"{expected}\" → 识别: \"{result}\" (RTF={rtf:.3f})")

print(f"\n准确率: {correct}/{total} = {correct/total*100:.1f}%")
print(f"平均 RTF: {total_rtf/total:.3f}")
```

### 11.3 TTS RTF 测试

```bash
# 使用 sherpa-onnx 命令行测试 TTS RTF
# matcha 版本
for t in 1 2 3 4; do
  sherpa-onnx-offline-tts \
    --num-threads=$t \
    --matcha-acoustic-model=models/tts/matcha-icefall-zh-baker/model-steps-3.onnx \
    --matcha-vocoder=models/tts/vocos-22khz-univ.onnx \
    --matcha-lexicon=models/tts/matcha-icefall-zh-baker/lexicon.txt \
    --matcha-tokens=models/tts/matcha-icefall-zh-baker/tokens.txt \
    --output-filename=./tts_rtf_test.wav \
    "当夜幕降临，星光点点，伴随着微风拂面，我在静谧中感受着时光的流转。" 2>&1 | grep -i "rtf\|elapsed"
done
```

---

## 12. 诊断命令与故障排查

### 12.1 系统信息

```bash
# 查看 CPU 信息
cat /proc/cpuinfo | grep "model name" | head -1
lscpu | grep -E "Architecture|CPU|Thread|Core|Socket"

# 查看内存
free -h

# 查看系统版本
cat /etc/os-release
uname -a

# 查看架构（应为 aarch64）
arch
```

### 12.2 音频设备诊断

```bash
# 列出所有音频设备
arecord -l
aplay -l

# 查看 ALSA 混音器
amixer

# PulseAudio 状态
pactl info
pactl list sources short   # 输入设备
pactl list sinks short     # 输出设备

# 录音测试 (5秒, 16kHz, 单声道)
arecord -d 5 -f S16_LE -r 16000 -c 1 /tmp/test_mic.wav
aplay /tmp/test_mic.wav

# 检查设备是否被占用
fuser -v /dev/snd/*
```

### 12.3 进程与资源监控

```bash
# 实时监控 CPU/内存
htop

# 观察 Python 进程 CPU 占用
top -p $(pgrep -f "python.*voice_assistant") -d 1

# 内存详细占用
ps aux --sort=-%mem | head -20

# 模型加载后内存占用
smem -rt -k | head -20
```

### 12.4 sherpa-onnx 诊断

```bash
# 验证安装
python3 -c "import sherpa_onnx; print(f'Version: {sherpa_onnx.__version__}')"

# 验证 ONNX Runtime
python3 -c "
import sherpa_onnx
print('sherpa-onnx OK')
print(f'Version: {sherpa_onnx.__version__}')
"

# 检查模型文件完整性
ls -lhR models/

# KWS 命令行快速测试 (使用预录音频)
sherpa-onnx-keyword-spotter \
  --encoder=models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/encoder-epoch-99-avg-1-chunk-16-left-128.int8.onnx \
  --decoder=models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/decoder-epoch-99-avg-1-chunk-16-left-128.onnx \
  --joiner=models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/joiner-epoch-99-avg-1-chunk-16-left-128.onnx \
  --tokens=models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/tokens.txt \
  --keywords-file=models/kws/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/keywords.txt \
  /tmp/test_mic.wav

# TTS 快速测试
sherpa-onnx-offline-tts \
  --vits-model=models/tts/vits-melo-tts-zh_en/model.onnx \
  --vits-lexicon=models/tts/vits-melo-tts-zh_en/lexicon.txt \
  --vits-tokens=models/tts/vits-melo-tts-zh_en/tokens.txt \
  --output-filename=/tmp/test_tts.wav \
  "测试语音合成功能正常。"
aplay /tmp/test_tts.wav
```

### 12.5 常见问题排查

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| `import sherpa_onnx` 失败 | 未安装或版本不兼容 | `pip install --upgrade sherpa-onnx` |
| 录音无声 | 麦克风未连接/权限 | `arecord -l` 确认设备, `sudo usermod -aG audio $USER` |
| TTS 播放无声 | 音箱未连接/音量为 0 | `amixer set Master 80%`, `aplay -l` 确认设备 |
| KWS 频繁误唤醒 | 阈值太低 | 调高 `keywords_threshold` (如 0.3 → 0.5) |
| KWS 唤醒困难 | 阈值太高 / 唤醒词不在模型词汇中 | 调低阈值, 检查 `keywords.txt` 拼音格式 |
| ASR 识别率低 | 环境噪音大 / 麦克风质量差 | 使用阵列麦克风, 调整 VAD 参数 |
| TTS RTF > 1.0 | 线程数不够 | 增加 `num_threads` (建议 2-4) |
| 内存不足 | 模型全部加载 | 使用 int8 模型, 减少缓冲区大小 |
| `sounddevice` 报错 | 未安装 portaudio | `sudo apt install portaudio19-dev` |

---

## 13. 参考仓库与文档链接

### 核心框架

| 项目 | 链接 | 说明 |
|------|------|------|
| **sherpa-onnx** | https://github.com/k2-fsa/sherpa-onnx | 核心推理框架 (⭐ 10.5k+) |
| sherpa-onnx 官方文档 | https://k2-fsa.github.io/sherpa/onnx/index.html | 安装/配置/API 文档 |
| KWS 预训练模型 | https://k2-fsa.github.io/sherpa/onnx/kws/pretrained_models/index.html | 唤醒词模型下载 |
| TTS 预训练模型 | https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/index.html | 语音合成模型下载 |
| ASR 预训练模型 | https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html | 语音识别模型下载 |
| VAD 文档 | https://k2-fsa.github.io/sherpa/onnx/vad/index.html | 语音活动检测 |
| RKNN 加速指南 | https://k2-fsa.github.io/sherpa/onnx/rknn/index.html | RK3588 NPU 加速 |

### 模型发布页

| 分类 | GitHub Release 链接 |
|------|---------------------|
| KWS 模型 | https://github.com/k2-fsa/sherpa-onnx/releases/tag/kws-models |
| ASR 模型 | https://github.com/k2-fsa/sherpa-onnx/releases/tag/asr-models |
| TTS 模型 | https://github.com/k2-fsa/sherpa-onnx/releases/tag/tts-models |
| Vocoder 模型 | https://github.com/k2-fsa/sherpa-onnx/releases/tag/vocoder-models |

### 参考项目

| 项目 | 链接 | 说明 |
|------|------|------|
| **AiVoiceAssistant** | https://github.com/Duangi/AiVoiceAssistant | OrangePi 5B (RK3588) 语音助手，使用 RKNN NPU |
| **QSmartAssistant** | https://github.com/xinhecuican/QSmartAssistant | C++/Qt 智能语音助手，⭐ 135 |
| **chinese-voice-assistant** | https://github.com/ioldyao/chinese-voice-assistant | sherpa-onnx 中文离线语音助手 |
| **Liu-Curiousity/voice_assistant** | https://github.com/Liu-Curiousity/voice_assistant | OrangePi 5 + sherpa-onnx 语音助手 |

### Python 示例代码 (sherpa-onnx 官方)

```
https://github.com/k2-fsa/sherpa-onnx/tree/master/python-api-examples
```

关键文件:
- `keyword-spotter-from-microphone.py` — KWS 麦克风实时唤醒
- `speech-recognition-from-microphone-with-vad.py` — VAD + ASR 实时识别
- `offline-tts.py` — 离线 TTS 合成
- `offline-tts-play.py` — TTS 合成并播放

### HuggingFace 在线试用

- TTS 试听: https://huggingface.co/spaces/k2-fsa/text-to-speech
- ASR 试用: https://huggingface.co/spaces/k2-fsa/automatic-speech-recognition

---

## 附录 A：一键下载全部模型脚本

```bash
#!/bin/bash
# download_all_models.sh
# 一键下载 KWS + VAD + ASR + TTS 全部模型

set -e
BASE_DIR="${HOME}/voice_assistant/models"

echo "=============================="
echo "  离线语音助手模型下载脚本"
echo "=============================="

# --- KWS ---
echo ""
echo "[1/4] 下载 KWS 模型..."
mkdir -p "${BASE_DIR}/kws" && cd "${BASE_DIR}/kws"
if [ ! -d "sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20" ]; then
    wget -q --show-progress https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
    tar xf sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
    rm sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
    echo "  ✅ KWS 模型下载完成"
else
    echo "  ⏭️  KWS 模型已存在，跳过"
fi

# --- VAD ---
echo ""
echo "[2/4] 下载 VAD 模型..."
mkdir -p "${BASE_DIR}/vad" && cd "${BASE_DIR}/vad"
if [ ! -f "silero_vad.onnx" ]; then
    wget -q --show-progress https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx
    echo "  ✅ VAD 模型下载完成"
else
    echo "  ⏭️  VAD 模型已存在，跳过"
fi

# --- ASR ---
echo ""
echo "[3/4] 下载 ASR 模型..."
mkdir -p "${BASE_DIR}/asr" && cd "${BASE_DIR}/asr"
if [ ! -d "sherpa-onnx-streaming-paraformer-bilingual-zh-en" ]; then
    wget -q --show-progress https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-paraformer-bilingual-zh-en.tar.bz2
    tar xf sherpa-onnx-streaming-paraformer-bilingual-zh-en.tar.bz2
    rm sherpa-onnx-streaming-paraformer-bilingual-zh-en.tar.bz2
    echo "  ✅ ASR 模型下载完成"
else
    echo "  ⏭️  ASR 模型已存在，跳过"
fi

# --- TTS (默认 vits-melo-tts-zh_en) ---
echo ""
echo "[4/4] 下载 TTS 模型..."
mkdir -p "${BASE_DIR}/tts" && cd "${BASE_DIR}/tts"
if [ ! -d "vits-melo-tts-zh_en" ]; then
    wget -q --show-progress https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-melo-tts-zh_en.tar.bz2
    tar xf vits-melo-tts-zh_en.tar.bz2
    rm vits-melo-tts-zh_en.tar.bz2
    echo "  ✅ TTS 模型下载完成"
else
    echo "  ⏭️  TTS 模型已存在，跳过"
fi

echo ""
echo "=============================="
echo "  所有模型下载完成！"
echo "=============================="
echo ""
echo "模型目录结构:"
du -sh "${BASE_DIR}"/*/* 2>/dev/null || true
echo ""
echo "总大小:"
du -sh "${BASE_DIR}"
```

使用方法：
```bash
chmod +x download_all_models.sh
./download_all_models.sh
```

---

## 附录 B：开机自启动配置

```bash
# 创建 systemd 服务
sudo tee /etc/systemd/system/voice-assistant.service << EOF
[Unit]
Description=Offline Voice Assistant
After=network.target sound.target pulseaudio.service
Wants=pulseaudio.service

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER/voice_assistant
ExecStart=/home/$USER/voice_assistant/.venv/bin/python3 /home/$USER/voice_assistant/main.py
Restart=always
RestartSec=5
Environment=PULSE_RUNTIME_PATH=/run/user/$(id -u)/pulse

[Install]
WantedBy=multi-user.target
EOF

# 启用并启动
sudo systemctl daemon-reload
sudo systemctl enable voice-assistant
sudo systemctl start voice-assistant

# 查看日志
sudo journalctl -u voice-assistant -f
```

---

## 附录 C：RKNN NPU 加速（可选进阶）

sherpa-onnx 从 v1.11+ 起官方支持 RK3588 RKNN NPU 加速，可将 ASR 推理从 CPU 卸载到 NPU。

**参考文档**: https://k2-fsa.github.io/sherpa/onnx/rknn/index.html

> 注意：对于按摩机器人场景，CPU-only 方案 RK3588 已足够满足实时性要求。RKNN 加速可作为后续性能优化选项。

---

> **文档维护**: 本文档基于 sherpa-onnx v1.12.27 编写。模型链接和 API 可能随版本更新变化，请参考官方发布页获取最新信息。
