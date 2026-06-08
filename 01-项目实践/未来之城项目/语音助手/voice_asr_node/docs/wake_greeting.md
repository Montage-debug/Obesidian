# 唤醒应答语

## 三种模式（`config/doubao.yaml` → `wakeword`）

| 配置 | 事件 | 说明 |
|------|------|------|
| `say_hello_direct_tts: true` | SayHello (300) | 固定短句直读（`tts.speaker` 音色），Session 后即可播 |
| `say_hello_use_model: true`（默认） | ChatTextQuery (501) | 发**短句**如 `你好，小未`，模型以助手身份回问候，每次可不同 |
| 两者均为 `false` | SayHello (300) | `content` 为**要播的短句**（同官方 demo），不是长指令 |

**常见错误**：把一长段「请用一句话打招呼…」放进 SayHello 的 `content`，服务端会**原样朗读**这段话。模型生成应走 **ChatTextQuery**，人设约束在 `dialog.system_role` 里。

## 默认推荐（短句「我在」，用 yaml 里 tts.speaker 音色）

```yaml
wakeword:
  say_hello_direct_tts: true
  say_hello_use_model: false
  say_hello_text: "我在"
  pre_generate_greeting: false
```

日志应为：`[wake] SayHello greeting: 我在` → `greeting TTS started` → 播完 `greeting TTS ended` 后再 `开始传输音频`（问候期间不上行，避免误识别）。

若需模型每次换说法，再改 `say_hello_use_model: true`（勿与「只要我在」同时使用）。

## 验证

1. 唤醒后应听到一句短问候，**不会**念配置里的长说明文字。
2. 多次唤醒，回复措辞可略有变化。
3. 需要固定文案时设 `say_hello_direct_tts: true` 并配置 `say_hello_by_speaker`。
