#!/usr/bin/env python3
# ================================================================
# 豆包端到端实时语音大模型 —— 二进制协议封包/解包
#
# 协议布局：
#   [4B header] [optional...] [4B payload_size] [payload]
#
#   header[0] = (version<<4) | header_size       version=1, header_size=1 (4B)
#   header[1] = (msg_type<<4) | msg_type_flags
#   header[2] = (serial_method<<4) | compression
#   header[3] = 0x00
#
# optional 字段顺序（如启用对应 flag 才出现）：
#   - code         (msg_type_flags == 0b1111)            4B int (大端)
#   - sequence     (msg_type_flags 0b0001/0b0010/0b0011) 4B int
#   - event        (msg_type_flags & 0b0100)             4B int  ← 我们一直启用
#   - connect_id_size + connect_id                       仅 Connect 类事件
#   - session_id_size + session_id                       仅 Session 类事件
# ================================================================

import json
import struct
import uuid
from dataclasses import dataclass
from typing import Optional, Tuple

# ── 协议常量 ────────────────────────────────────────────────────
PROTOCOL_VERSION = 0b0001
HEADER_SIZE_4B = 0b0001

# Message Type
MSG_TYPE_FULL_CLIENT_REQUEST = 0b0001
MSG_TYPE_AUDIO_ONLY_REQUEST = 0b0010
MSG_TYPE_FULL_SERVER_RESPONSE = 0b1001
MSG_TYPE_AUDIO_ONLY_RESPONSE = 0b1011
MSG_TYPE_ERROR = 0b1111

# Message Type Specific Flags
MSG_FLAGS_NO_SEQ = 0b0000
MSG_FLAGS_POS_SEQ = 0b0001
MSG_FLAGS_NEG_SEQ_TERM = 0b0010
MSG_FLAGS_NEG_SEQ = 0b0011
MSG_FLAGS_WITH_EVENT = 0b0100
MSG_FLAGS_ERROR = 0b1111

# Serialization
SERIAL_RAW = 0b0000
SERIAL_JSON = 0b0001

# Compression
COMPRESS_NONE = 0b0000
COMPRESS_GZIP = 0b0001

# ── 客户端事件 ID ───────────────────────────────────────────────
EVT_StartConnection = 1
EVT_FinishConnection = 2
EVT_StartSession = 100
EVT_FinishSession = 102
EVT_TaskRequest = 200
EVT_UpdateConfig = 201
EVT_SayHello = 300
EVT_EndASR = 400
EVT_ChatTTSText = 500
EVT_ChatTextQuery = 501
EVT_ChatRAGText = 502
EVT_ConversationCreate = 510
EVT_ConversationUpdate = 511
EVT_ConversationRetrieve = 512
EVT_ConversationTruncate = 513
EVT_ConversationDelete = 514
EVT_ClientInterrupt = 515

# ── 服务端事件 ID ───────────────────────────────────────────────
EVT_ConnectionStarted = 50
EVT_ConnectionFailed = 51
EVT_ConnectionFinished = 52
EVT_SessionStarted = 150
EVT_SessionFinished = 152
EVT_SessionFailed = 153
EVT_UsageResponse = 154
EVT_ConfigUpdated = 251
EVT_TTSSentenceStart = 350
EVT_TTSSentenceEnd = 351
EVT_TTSResponse = 352
EVT_TTSEnded = 359
EVT_ASRInfo = 450
EVT_ASRResponse = 451
EVT_ASREnded = 459
EVT_ChatResponse = 550
EVT_ChatTextQueryConfirmed = 553
EVT_ChatEnded = 559
EVT_ConversationCreated = 567
EVT_ConversationUpdated = 568
EVT_ConversationRetrieved = 569
EVT_ConversationTruncated = 570
EVT_ConversationDeleted = 571
EVT_DialogCommonError = 599


# ── 工具函数：连接 ID ───────────────────────────────────────────
def new_connect_id() -> str:
    return str(uuid.uuid4())


def new_session_id() -> str:
    return str(uuid.uuid4())


# ── 通用 4 字节头 ───────────────────────────────────────────────
def _header(msg_type: int, flags: int, serial: int, compress: int = COMPRESS_NONE) -> bytes:
    return bytes([
        (PROTOCOL_VERSION << 4) | HEADER_SIZE_4B,
        (msg_type << 4) | flags,
        (serial << 4) | compress,
        0x00,
    ])


def _pack_optional(
    event: Optional[int] = None,
    connect_id: Optional[str] = None,
    session_id: Optional[str] = None,
    sequence: Optional[int] = None,
    code: Optional[int] = None,
) -> bytes:
    """按文档要求的顺序打包 optional 字段：code → sequence → event → connect_id → session_id"""
    parts = []
    if code is not None:
        parts.append(struct.pack(">i", code))
    if sequence is not None:
        parts.append(struct.pack(">i", sequence))
    if event is not None:
        parts.append(struct.pack(">i", event))
    if connect_id is not None:
        cid = connect_id.encode("utf-8")
        parts.append(struct.pack(">I", len(cid)) + cid)
    if session_id is not None:
        sid = session_id.encode("utf-8")
        parts.append(struct.pack(">I", len(sid)) + sid)
    return b"".join(parts)


def build_client_event(
    event: int,
    payload: bytes,
    *,
    session_id: Optional[str] = None,
    connect_id: Optional[str] = None,
    serial: int = SERIAL_JSON,
    msg_type: int = MSG_TYPE_FULL_CLIENT_REQUEST,
    flags: int = MSG_FLAGS_WITH_EVENT,
) -> bytes:
    """组装一帧客户端事件二进制数据。"""
    h = _header(msg_type, flags, serial)
    opt = _pack_optional(event=event, connect_id=connect_id, session_id=session_id)
    return h + opt + struct.pack(">I", len(payload)) + payload


def build_json_event(event: int, obj, **kwargs) -> bytes:
    payload = json.dumps(obj, ensure_ascii=False).encode("utf-8")
    return build_client_event(event, payload, serial=SERIAL_JSON, **kwargs)


def build_empty_event(event: int, **kwargs) -> bytes:
    return build_json_event(event, {}, **kwargs)


def build_audio_event(
    event: int,
    audio_bytes: bytes,
    *,
    session_id: Optional[str] = None,
) -> bytes:
    """音频帧（TaskRequest 200）：msg_type=AUDIO_ONLY，serial=RAW。"""
    return build_client_event(
        event,
        audio_bytes,
        session_id=session_id,
        serial=SERIAL_RAW,
        msg_type=MSG_TYPE_AUDIO_ONLY_REQUEST,
    )


# ── 解包 ────────────────────────────────────────────────────────
@dataclass
class ServerFrame:
    msg_type: int
    flags: int
    serial: int
    compression: int
    event: Optional[int] = None
    session_id: Optional[str] = None
    connect_id: Optional[str] = None
    sequence: Optional[int] = None
    code: Optional[int] = None
    payload: bytes = b""
    # 解出来的 json/text（若 serial=JSON）
    payload_json: Optional[dict] = None

    def is_audio(self) -> bool:
        return self.msg_type == MSG_TYPE_AUDIO_ONLY_RESPONSE

    def is_error(self) -> bool:
        return self.msg_type == MSG_TYPE_ERROR


def parse_server_frame(data: bytes) -> ServerFrame:
    if len(data) < 4:
        raise ValueError(f"frame too short: {len(data)} bytes")

    b0, b1, b2, _b3 = data[0], data[1], data[2], data[3]
    # version = b0 >> 4
    header_size_4b = b0 & 0x0F
    msg_type = b1 >> 4
    flags = b1 & 0x0F
    serial = b2 >> 4
    compression = b2 & 0x0F

    offset = header_size_4b * 4  # 头部 4 字节

    frame = ServerFrame(
        msg_type=msg_type, flags=flags, serial=serial, compression=compression
    )

    # optional 字段
    if flags == MSG_FLAGS_ERROR or msg_type == MSG_TYPE_ERROR:
        frame.code = struct.unpack(">i", data[offset:offset + 4])[0]
        offset += 4
    elif flags in (MSG_FLAGS_POS_SEQ, MSG_FLAGS_NEG_SEQ_TERM, MSG_FLAGS_NEG_SEQ):
        frame.sequence = struct.unpack(">i", data[offset:offset + 4])[0]
        offset += 4

    if flags & MSG_FLAGS_WITH_EVENT:
        frame.event = struct.unpack(">i", data[offset:offset + 4])[0]
        offset += 4

        # Connect 类事件携带 connect_id；Session 类携带 session_id
        if frame.event in (EVT_ConnectionStarted, EVT_ConnectionFailed, EVT_ConnectionFinished):
            cid_size = struct.unpack(">I", data[offset:offset + 4])[0]
            offset += 4
            frame.connect_id = data[offset:offset + cid_size].decode("utf-8", errors="replace")
            offset += cid_size
        elif frame.event is not None and frame.event >= 100:
            # Session 级事件（>=100）携带 session_id
            sid_size = struct.unpack(">I", data[offset:offset + 4])[0]
            offset += 4
            frame.session_id = data[offset:offset + sid_size].decode("utf-8", errors="replace")
            offset += sid_size

    # payload
    if offset + 4 <= len(data):
        payload_size = struct.unpack(">I", data[offset:offset + 4])[0]
        offset += 4
        frame.payload = data[offset:offset + payload_size]

    # 解 JSON
    if frame.serial == SERIAL_JSON and frame.payload:
        try:
            frame.payload_json = json.loads(frame.payload.decode("utf-8"))
        except Exception:
            frame.payload_json = None

    return frame


# ── 自检 ────────────────────────────────────────────────────────
def _self_test():
    """根据文档示例校验字节序列。"""
    # ① StartConnection: [17 20 16 0  0 0 0 1  0 0 0 2  123 125]
    frame = build_empty_event(EVT_StartConnection)
    expected = bytes([17, 20, 16, 0, 0, 0, 0, 1, 0, 0, 0, 2, 123, 125])
    assert frame == expected, f"StartConnection mismatch:\n got: {list(frame)}\nwant: {list(expected)}"
    print("✅ StartConnection 字节序列匹配文档示例")

    # ② StartSession (session_id=75a6...db3, payload={"dialog":{"bot_name":"豆包","dialog_id":"","extra":null}})
    sid = "75a6126e-427f-49a1-a2c1-621143cb9db3"
    payload_obj = {"dialog": {"bot_name": "豆包", "dialog_id": "", "extra": None}}
    payload = json.dumps(payload_obj, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    frame = build_client_event(EVT_StartSession, payload, session_id=sid, serial=SERIAL_JSON)
    expected_prefix = bytes([17, 20, 16, 0, 0, 0, 0, 100, 0, 0, 0, 36])
    assert frame[:12] == expected_prefix, f"StartSession header mismatch: {list(frame[:12])}"
    assert frame[12:12 + 36].decode() == sid, "session_id mismatch"
    assert struct.unpack(">I", frame[12 + 36:12 + 36 + 4])[0] == len(payload), "payload size mismatch"
    print("✅ StartSession 头/SessionID/PayloadSize 与文档示例一致")

    # ③ 解包回环
    parsed = parse_server_frame(frame)
    assert parsed.event == EVT_StartSession
    assert parsed.session_id == sid
    print("✅ parse_server_frame 回环正常")


if __name__ == "__main__":
    _self_test()
