# 本地声纹识别（Sherpa-ONNX）与豆包「克隆音色」的区别

| 能力 | 配置项 | 说明 |
|------|--------|------|
| **本地声纹** | `config/doubao.yaml` → `speaker_id` | Sherpa `SpeakerEmbeddingExtractor` + 本地 `.npy` 档案，唤醒后识别说话人 |
| **豆包克隆音色** | `tts.clone_speaker_id` / 控制台 `S_xxx` | 见 [doubao_assistant.md](./doubao_assistant.md) 上传接口，用于 TTS 发音人，**不是**声纹验证 |

## 推荐模型（项目默认）

已选型并放入包内路径（需执行下载脚本或自行 curl）：

- **文件**：`models/speaker/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx`
- **来源**：[k2-fsa/sherpa-onnx speaker-recongition-models](https://github.com/k2-fsa/sherpa-onnx/releases/tag/speaker-recongition-models)
- **说明**：3D-Speaker **CAM++**，中文 16kHz，embedding 维度 **192**，与 KWS 共用 `sherpa-onnx` 依赖

下载：

```bash
cd /home/wlzckj/WorkSpace/wlzc_massage_robot_ws/src/voice_asr_node
chmod +x scripts/download_speaker_model.sh
./scripts/download_speaker_model.sh
```

## 启用步骤

1. 确认模型文件存在（见上）。
2. 编辑 `config/doubao.yaml`：

```yaml
speaker_id:
  enabled: true
  model_path: "models/speaker/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx"
  profiles_dir: "data/speaker_profiles"
  similarity_threshold: 0.75
  inject_username_to_prompt: true
```

3. 注册声纹（16kHz、单声道、16bit WAV，建议 2s 以上清晰语音）：

```bash
source /home/wlzckj/WorkSpace/wlzc_massage_robot_ws/.venv/bin/activate
register_speaker --name 张三 --wav /path/to/sample.wav
```

4. 重启 `voice_asr` launch，唤醒后日志应出现 `[speaker_id] identified 张三`。

## 文件位置与验证

| 内容 | 典型路径 |
|------|----------|
| 声纹模型 | `install/voice_asr_node/share/voice_asr_node/models/speaker/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx`（或源码 `src/voice_asr_node/models/speaker/`） |
| 已注册档案 | `install/voice_asr_node/share/voice_asr_node/data/speaker_profiles/<姓名>.npy` |

启动时应看到 **`[speaker_id] ready profiles=N`**（不是 `model not found`）。唤醒对话开始后应看到 **`[speaker_id] identified 天才`** 或 **`session user: 天才`**（`inject_username_to_prompt: true` 时）。

排查：

```bash
# 模型是否存在
ls -la install/voice_asr_node/share/voice_asr_node/models/speaker/*.onnx
# 档案是否存在
ls -la install/voice_asr_node/share/voice_asr_node/data/speaker_profiles/
```

若日志仍写 `resource/models/speaker/... not found`，说明路径解析未命中 install；`colcon build --packages-select voice_asr_node` 后重试。

档案目录解析顺序：`MR_RESOURCE_DIR/data/...` → `share/voice_asr_node/data/...` → 相对 models 根。

## 调参建议

| 参数 | 建议 |
|------|------|
| `similarity_threshold` | 0.70–0.80；误识多则调高，拒识多则调低 |
| `min_audio_ms` | 1500–2500；唤醒后环缓长度 |
| `reject_unknown` | `true` 时低于阈值不注入用户名 |

## 媒体音量

系统 Pulse 音量由 `audio_output` 与 `[CMD:volume_*]` 控制，见 `config/command.yaml`。
