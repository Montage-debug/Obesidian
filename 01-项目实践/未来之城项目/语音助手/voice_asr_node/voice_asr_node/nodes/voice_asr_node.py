#!/usr/bin/env python3
# ================================================================
# voice_asr_node.py  (Python 等价替换 voice_asr_node.cpp)
#
# 话题接口：
#   订阅  massage_biz_event              (robot_interfaces/MassageBizEvent)
#         → event_msg 字段直接入 TTS 队列
#
#   订阅  massage_error_biz_event        (robot_interfaces/MassageErrorBizEvent)
#         → 拼装 module_type / error_type / error_code / detail_message → TTS
#
#   订阅  voice_announcement             (robot_interfaces/VoiceAnnouncement)
#         → text_content 按 priority 决定是否插队播报
#
#   订阅  /voice_asr/asr_command         (std_msgs/String)
#         → 解析意图码或自然语言关键词（暂无处理逻辑）
#
#   发布  /voice_asr/tts_speak           (std_msgs/String)
#         → doubao_dialog_node 订阅后经豆包 TTS 播报
#
# ================================================================

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from robot_interfaces.msg import MassageBizEvent, MassageErrorBizEvent, VoiceAnnouncement


class VoiceAsrNode(Node):
    """
    语音 ASR 主节点（Python 版）。

    负责：
    - 订阅业务事件 / 错误事件 / 语音播报，统一转发到 TTS 话题
    - 订阅 ASR 命令词，匹配预设点位后异步调用机械臂服务
    """

    def __init__(self):
        super().__init__('voice_asr_node')

        # ── 参数声明 ────────────────────────────────────────────
        self.declare_parameter('tts_speak_topic',          '/voice_asr/tts_speak')
        self.declare_parameter('asr_command_topic',        '/voice_asr/asr_command')
        self.declare_parameter('massage_biz_topic',        'massage_biz_event')
        self.declare_parameter('massage_err_topic',        'massage_error_biz_event')
        self.declare_parameter('voice_announcement_topic', 'voice_announcement')

        tts_topic = self.get_parameter('tts_speak_topic').value
        asr_topic = self.get_parameter('asr_command_topic').value
        biz_topic = self.get_parameter('massage_biz_topic').value
        err_topic = self.get_parameter('massage_err_topic').value
        ann_topic = self.get_parameter('voice_announcement_topic').value

        # ── 发布者：TTS 播报文本 ────────────────────────────────
        self._tts_pub = self.create_publisher(String, tts_topic, 10)
        self.get_logger().info(f'TTS 发布话题: {tts_topic}')

        # ── 订阅：ASR 命令词 ────────────────────────────────────
        self.create_subscription(String, asr_topic, self._on_asr_command, 10)
        self.get_logger().info(f'ASR 命令词订阅: {asr_topic}')

        # ── 订阅：按摩业务事件
        self.create_subscription(
            MassageBizEvent, biz_topic, self._on_massage_biz_event, 10)
        self.get_logger().info(f'按摩业务事件订阅: {biz_topic}')

        # ── 订阅：按摩错误事件 ──────────────────────────────────
        self.create_subscription(
            MassageErrorBizEvent, err_topic, self._on_massage_error_biz_event, 10)
        self.get_logger().info(f'错误事件订阅: {err_topic}')

        # ── 订阅：语音播报统一入口 ──────────────────────────────
        self.create_subscription(
            VoiceAnnouncement, ann_topic, self._on_voice_announcement, 10)
        self.get_logger().info(f'语音播报订阅: {ann_topic}')

        self.get_logger().info(
            f'[voice_asr_node] 初始化完成。')

    # ================================================================
    # 回调：ASR 命令词
    # ================================================================
    def _on_asr_command(self, msg: String):
        text = msg.data
        self.get_logger().info(f'[ASR命令] 收到: "{text}"')

    # ================================================================
    # 回调：按摩业务进度事件
    # ================================================================
    def _on_massage_biz_event(self, msg: MassageBizEvent):
        if not msg.event_msg:
            self.get_logger().debug('[MassageBizEvent] event_msg 为空，跳过')
            return
        self.get_logger().info(
            f'[MassageBizEvent] type={msg.event_type} status={msg.massage_status} '
            f'msg={msg.event_msg}')
        self._publish_tts(msg.event_msg)

    # ================================================================
    # 回调：按摩系统错误事件
    # ================================================================
    def _on_massage_error_biz_event(self, msg: MassageErrorBizEvent):
        parts = ['系统提示：']
        if msg.module_type:
            parts.append(f'{msg.module_type}模块')
        if msg.error_type:
            parts.append(f'发生{msg.error_type}')
        parts.append('错误')
        if msg.error_code:
            parts.append(f'，错误码{msg.error_code}')
        if msg.detail_message:
            parts.append(f'，{msg.detail_message}')
        parts.append('，请检查设备状态。')

        tts_text = ''.join(parts)
        self.get_logger().warn(
            f'[MassageErrorBizEvent] module={msg.module_type} type={msg.error_type} '
            f'code={msg.error_code} | {msg.detail_message}')
        self._publish_tts(tts_text)

    # ================================================================
    # 回调：统一语音播报入口（VoiceAnnouncement）
    # ================================================================
    def _on_voice_announcement(self, msg: VoiceAnnouncement):
        if not msg.text_content:
            return
        self.get_logger().info(
            f'[VoiceAnnouncement] type={msg.announcement_type} '
            f'priority={msg.priority} text={msg.text_content}')
        if msg.priority >= 8:
            self._publish_tts('[URGENT]' + msg.text_content)
        else:
            self._publish_tts(msg.text_content)

    # ================================================================
    # 内部：发布 TTS 文本
    # ================================================================
    def _publish_tts(self, text: str):
        if not text:
            return
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)
        self.get_logger().info(f'  [TTS→] {text}')


# ================================================================
# main
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VoiceAsrNode()
    node.get_logger().info('voice_asr_node 已启动')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
