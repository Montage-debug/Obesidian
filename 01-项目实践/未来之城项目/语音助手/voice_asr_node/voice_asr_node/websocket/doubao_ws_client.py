#!/usr/bin/env python3
# ================================================================
# 豆包端到端实时语音 —— asyncio websockets 客户端
#
# 提供高层 API：
#   connect()              -> 建立 ws + StartConnection + 等待 ConnectionStarted
#   start_session(cfg)     -> StartSession + 等待 SessionStarted
#   send_audio(pcm_bytes)  -> 发送一帧 16k mono int16 PCM
#   send_chat_tts_text(t)  -> 让服务端把 t 当作 LLM 回复直接 TTS 播报
#   send_client_interrupt()-> 打断当前 TTS
#   finish_session()
#   finish_connection()
#   set_handler(cb)        -> cb(frame: ServerFrame) 在事件循环线程被调用
# ================================================================

import asyncio
import logging
import random
from typing import Awaitable, Callable, Optional

import websockets
from websockets.exceptions import ConnectionClosed

from .binary_protocol import (
    EVT_ChatTextQuery,
    EVT_ChatTTSText,
    EVT_ClientInterrupt,
    EVT_ConversationTruncate,
    EVT_ConnectionFailed,
    EVT_ConnectionFinished,
    EVT_ConnectionStarted,
    EVT_FinishConnection,
    EVT_FinishSession,
    EVT_SayHello,
    EVT_SessionFailed,
    EVT_SessionFinished,
    EVT_SessionStarted,
    EVT_StartConnection,
    EVT_StartSession,
    EVT_TaskRequest,
    ServerFrame,
    build_audio_event,
    build_empty_event,
    build_json_event,
    new_connect_id,
    new_session_id,
    parse_server_frame,
)

logger = logging.getLogger("doubao_client")

FrameHandler = Callable[[ServerFrame], Awaitable[None]]


class DoubaoRealtimeClient:
    def __init__(
        self,
        ws_url: str,
        api_key: str,
        resource_id: str = "volc.speech.dialog",
        app_key: str = "PlgvMymc7f3tQnJ6",
        reconnect_min: float = 1.0,
        reconnect_max: float = 30.0,
    ):
        self.ws_url = ws_url
        self.api_key = api_key
        self.resource_id = resource_id
        self.app_key = app_key
        self.reconnect_min = reconnect_min
        self.reconnect_max = reconnect_max

        self._ws: Optional[websockets.WebSocketClientProtocol] = None
        self._connect_id: Optional[str] = None
        self._session_id: Optional[str] = None
        self._handler: Optional[FrameHandler] = None
        self._recv_task: Optional[asyncio.Task] = None
        self._recv_done = asyncio.Event()
        self._connection_ready = asyncio.Event()
        self._session_ready = asyncio.Event()
        self._closed = False

    # ── 公共属性 ─────────────────────────────────────────────────
    @property
    def session_id(self) -> Optional[str]:
        return self._session_id

    @property
    def is_in_session(self) -> bool:
        return self._session_id is not None and self._session_ready.is_set()

    def set_handler(self, handler: FrameHandler) -> None:
        self._handler = handler

    # ── 建立连接 ────────────────────────────────────────────────
    async def connect(self) -> None:
        self._connect_id = new_connect_id()
        headers = {
            "x-api-key": self.api_key,
            "X-Api-Resource-Id": self.resource_id,
            "X-Api-App-Key": self.app_key,
            "X-Api-Connect-Id": self._connect_id,
        }

        backoff = self.reconnect_min
        while not self._closed:
            try:
                # websockets 库 12.x+：使用 additional_headers；早期版本：extra_headers
                try:
                    self._ws = await websockets.connect(
                        self.ws_url,
                        additional_headers=headers,
                        max_size=None,
                        ping_interval=20,
                        ping_timeout=20,
                    )
                except TypeError:
                    self._ws = await websockets.connect(
                        self.ws_url,
                        extra_headers=headers,
                        max_size=None,
                        ping_interval=20,
                        ping_timeout=20,
                    )
                logger.info(f"[doubao] ws connected (connect_id={self._connect_id})")
                break
            except Exception as e:
                logger.warning(f"[doubao] ws connect failed: {e!r}, retry in {backoff:.1f}s")
                await asyncio.sleep(backoff + random.random() * 0.5)
                backoff = min(self.reconnect_max, backoff * 2)

        if self._closed:
            return

        # 启动 recv loop
        self._connection_ready.clear()
        self._session_ready.clear()
        self._recv_done.clear()
        self._recv_task = asyncio.create_task(self._recv_loop(), name="doubao-recv")

        # 发 StartConnection 并等待 ConnectionStarted
        await self._ws.send(build_empty_event(EVT_StartConnection))
        try:
            await asyncio.wait_for(self._connection_ready.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            logger.error("[doubao] timeout waiting ConnectionStarted")
            raise

    # ── Session ─────────────────────────────────────────────────
    async def start_session(self, session_payload: dict) -> str:
        self._session_id = new_session_id()
        self._session_ready.clear()
        await self._ws.send(build_json_event(
            EVT_StartSession,
            session_payload,
            session_id=self._session_id,
        ))
        try:
            await asyncio.wait_for(self._session_ready.wait(), timeout=10.0)
        except asyncio.TimeoutError:
            logger.error("[doubao] timeout waiting SessionStarted")
            raise
        return self._session_id

    async def say_hello(self, content: str) -> None:
        if not self._session_id:
            return
        await self._ws.send(build_json_event(
            EVT_SayHello, {"content": content}, session_id=self._session_id,
        ))

    async def send_audio(self, audio: bytes) -> None:
        if self._closed or not self._ws or not self._session_id:
            return
        try:
            await self._ws.send(build_audio_event(
                EVT_TaskRequest, audio, session_id=self._session_id,
            ))
        except ConnectionClosed:
            if not self._closed:
                logger.warning("[doubao] send_audio on closed ws")

    async def send_chat_text_query(self, content: str) -> None:
        """文本 query，由模型生成闲聊回复并 TTS（用于唤醒问候等）。"""
        if not self._session_id or not content.strip():
            return
        await self._ws.send(build_json_event(
            EVT_ChatTextQuery,
            {"content": content.strip()},
            session_id=self._session_id,
        ))

    async def send_chat_tts_text(self, text: str, *, start: bool = True, end: bool = True) -> None:
        """让服务端直接将 text 当作大模型回复进行 TTS（语音风格保持一致）。"""
        if not self._session_id:
            return
        await self._ws.send(build_json_event(
            EVT_ChatTTSText,
            {"start": start, "content": text, "end": end},
            session_id=self._session_id,
        ))

    async def send_client_interrupt(self) -> None:
        if not self._session_id:
            return
        try:
            await self._ws.send(build_empty_event(
                EVT_ClientInterrupt, session_id=self._session_id,
            ))
        except ConnectionClosed:
            pass

    async def send_conversation_truncate(self, item_id: str, audio_end_ms: int) -> None:
        """按客户端实际播报进度截断上下文（需 StartSession 开启 enable_conversation_truncate）。"""
        if not self._session_id or not item_id or audio_end_ms <= 0:
            return
        try:
            await self._ws.send(build_json_event(
                EVT_ConversationTruncate,
                {"item_id": item_id, "audio_end_ms": int(audio_end_ms)},
                session_id=self._session_id,
            ))
        except ConnectionClosed:
            pass

    async def finish_session(self) -> None:
        if not self._ws or not self._session_id:
            return
        try:
            await self._ws.send(build_empty_event(
                EVT_FinishSession, session_id=self._session_id,
            ))
        except ConnectionClosed:
            pass

    async def finish_connection(self) -> None:
        if not self._ws:
            return
        try:
            await self._ws.send(build_empty_event(EVT_FinishConnection))
        except ConnectionClosed:
            pass

    async def close(self) -> None:
        self._closed = True
        try:
            await self.finish_session()
            await self.finish_connection()
        except Exception:
            pass
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
        if self._recv_task is not None:
            self._recv_task.cancel()

    async def wait_recv_done(self) -> None:
        await self._recv_done.wait()

    # ── recv ────────────────────────────────────────────────────
    async def _recv_loop(self) -> None:
        try:
            async for msg in self._ws:
                if isinstance(msg, str):
                    msg = msg.encode("utf-8")
                try:
                    frame = parse_server_frame(msg)
                except Exception as e:
                    logger.warning(f"[doubao] parse frame failed: {e!r}")
                    continue

                # 内部状态机
                if frame.event == EVT_ConnectionStarted:
                    self._connection_ready.set()
                elif frame.event == EVT_ConnectionFailed:
                    logger.error(f"[doubao] ConnectionFailed: {frame.payload_json}")
                    self._connection_ready.set()
                elif frame.event == EVT_SessionStarted:
                    self._session_ready.set()
                elif frame.event == EVT_SessionFailed:
                    logger.error(f"[doubao] SessionFailed: {frame.payload_json}")
                    self._session_ready.set()
                    self._session_id = None
                elif frame.event in (EVT_SessionFinished, EVT_ConnectionFinished):
                    self._session_id = None

                if self._handler is not None:
                    try:
                        await self._handler(frame)
                    except Exception as e:
                        logger.exception(f"[doubao] handler raised: {e!r}")
        except ConnectionClosed as e:
            logger.warning(f"[doubao] ws closed: {e!r}")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.exception(f"[doubao] recv loop crash: {e!r}")
        finally:
            self._recv_done.set()
