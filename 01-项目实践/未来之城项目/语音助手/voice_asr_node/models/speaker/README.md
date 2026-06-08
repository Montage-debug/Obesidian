# 声纹识别模型（Sherpa-ONNX）

本目录存放 **说话人 embedding** ONNX，供 `SpeakerIdentifier` 与 `register_speaker` 使用。  
与豆包控制台「克隆音色」`S_xxx` **不是同一类模型**。

## 推荐模型（已选型）

| 项目 | 说明 |
|------|------|
| 名称 | `3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx` |
| 来源 | [sherpa-onnx speaker-recongition-models](https://github.com/k2-fsa/sherpa-onnx/releases/tag/speaker-recongition-models) |
| 架构 | 3D-Speaker **CAM++**，中文通用，16kHz |
| 体积 | 约 27 MB |
| 向量维度 | 192 |

选用理由：中文按摩机器人场景下载量高、体积适中、与现有 KWS 相同依赖 `sherpa-onnx`，无需单独 ECAPA 导出链。

## 下载

```bash
cd /home/wlzckj/WorkSpace/wlzc_massage_robot_ws/src/voice_asr_node
./scripts/download_speaker_model.sh
```

或手动：

```bash
curl -fL -o models/speaker/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx \
  https://github.com/k2-fsa/sherpa-onnx/releases/download/speaker-recongition-models/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx
```

## 配置

在 [config/doubao.yaml](../../../config/doubao.yaml)：

```yaml
speaker_id:
  enabled: true
  model_path: "models/speaker/3dspeaker_speech_campplus_sv_zh-cn_16k-common.onnx"
  profiles_dir: "data/speaker_profiles"
  similarity_threshold: 0.75
  inject_username_to_prompt: true
```

## 注册与使用

```bash
# 使用工作空间 venv（需已安装 sherpa-onnx，与 KWS 相同）
source /home/wlzckj/WorkSpace/wlzc_massage_robot_ws/.venv/bin/activate
register_speaker --name 张三 --wav /path/to/16k_mono.wav
```

唤醒后节点会从麦克风环缓取约 1.5s 音频做识别。详见 [docs/speaker_id.md](../../docs/speaker_id.md)。

## 其他可选模型

同发布页还可选（需改 `model_path` 并重新注册档案）：

- `3dspeaker_speech_eres2net_base_sv_zh-cn_3dspeaker_16k.onnx` — 精度略高、体积更大  
- `wespeaker_zh_cnceleb_resnet34.onnx` — WeSpeaker 中文 ResNet34  
- `3dspeaker_speech_campplus_sv_zh_en_16k-common_advanced.onnx` — 中英混合场景  

不建议使用文件名含 `ecapa_tdnn` 的第三方导出，除非确认与 `SpeakerEmbeddingExtractor` API 兼容。
