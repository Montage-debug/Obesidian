from __future__ import annotations

import asyncio
import concurrent.futures
import logging
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Type

from .massage_cmd import (
    MASSAGE_DURATION_SERVICE,
    MASSAGE_FORCE_SERVICE,
    MASSAGE_HEAD_PARAM_SERVICE,
    MASSAGE_POSITION_SERVICE,
    MASSAGE_PROCESS_INFO_SERVICE,
    format_massage_progress_summary,
    map_position_direction,
    parse_duration_from_tag,
    parse_force_from_tag,
    parse_head_param_tag,
    build_head_param_json,
)

logger = logging.getLogger(__name__)

MUSIC_CONTROL_SERVICE = "/music/control_action"
MUSIC_VOLUME_SERVICE = "/music/control_volume"
MUSIC_STATUS_SERVICE = "/music/get_status"

# command.yaml 中 ros2_service -> robot_interfaces.srv 类型（空请求服务）
_MASSAGE_SERVICE_TYPES: dict[str, str] = {
    "/massage_pause": "MassagePauseReq",
    "/massage_resume": "MassageResumeReq",
    "/massage_cancel": "MassageCancelReq",
}

_MASSAGE_PARAM_SERVICE_TYPES: dict[str, str] = {
    MASSAGE_POSITION_SERVICE: "MassagePositionControlReq",
    MASSAGE_DURATION_SERVICE: "MassageDurationControlReq",
    MASSAGE_FORCE_SERVICE: "MassageForceControlReq",
    MASSAGE_PROCESS_INFO_SERVICE: "MassageProcessInfoGetReq",
    MASSAGE_HEAD_PARAM_SERVICE: "MassageHeadParamControlReq",
}

MASSAGE_SERVICE_NAMES = frozenset(_MASSAGE_SERVICE_TYPES.keys())
MASSAGE_PARAM_SERVICE_NAMES = frozenset(_MASSAGE_PARAM_SERVICE_TYPES.keys())


def is_massage_service(service_name: str) -> bool:
    return str(service_name or "").strip() in MASSAGE_SERVICE_NAMES


def is_massage_param_service(service_name: str) -> bool:
    return str(service_name or "").strip() in MASSAGE_PARAM_SERVICE_NAMES


@dataclass
class ROS2ActionResult:
    ok: bool
    message: str = ""


@dataclass
class MusicControlResult:
    ok: bool
    message: str = ""
    current_file: str = ""


@dataclass
class MusicVolumeResult:
    ok: bool
    message: str = ""
    current_volume: float = 0.0


@dataclass
class MassageParamResult:
    ok: bool
    message: str = ""
    current_force: float = 0.0
    remaining_duration: int = 0


class ROS2Bridge:
    """
    ROS2 调用桥接（Python / rclpy）。

    服务调用通过 node.invoke_on_spin_thread() 投递到 rclpy spin 线程执行。
    """

    def __init__(self, node: Any):
        self._node = node
        self._clients: dict[str, Any] = {}
        self._srv_types: dict[str, Type] = {}

        self._rclpy = None
        self._MusicControlAction = None
        self._MusicControlVolume = None
        self._MusicGetStatus = None
        self._import_errors: list[str] = []

        try:
            import rclpy as _rclpy  # type: ignore

            self._rclpy = _rclpy
        except Exception as e:
            self._import_errors.append(f"rclpy: {e}")
            return

        try:
            from robot_interfaces.srv import MusicControlAction as _MusicControlAction  # type: ignore

            self._MusicControlAction = _MusicControlAction
            self._srv_types["MusicControlAction"] = _MusicControlAction
        except Exception as e:
            self._import_errors.append(f"MusicControlAction: {e}")

        try:
            from robot_interfaces.srv import MusicControlVolume as _MusicControlVolume  # type: ignore
            from robot_interfaces.srv import MusicGetStatus as _MusicGetStatus  # type: ignore

            self._MusicControlVolume = _MusicControlVolume
            self._MusicGetStatus = _MusicGetStatus
            self._srv_types["MusicControlVolume"] = _MusicControlVolume
            self._srv_types["MusicGetStatus"] = _MusicGetStatus
        except Exception as e:
            self._import_errors.append(f"MusicVolume/GetStatus: {e}")

        for srv_name in (
            "MassagePauseReq",
            "MassageResumeReq",
            "MassageCancelReq",
            "MassagePositionControlReq",
            "MassageDurationControlReq",
            "MassageForceControlReq",
            "MassageProcessInfoGetReq",
            "MassageHeadParamControlReq",
        ):
            try:
                mod = __import__("robot_interfaces.srv", fromlist=[srv_name])
                srv_type = getattr(mod, srv_name)
                self._srv_types[srv_name] = srv_type
            except Exception as e:
                self._import_errors.append(f"{srv_name}: {e}")

        if self._import_errors:
            logger.warning("ROS2Bridge partial import: %s", "; ".join(self._import_errors))
        elif self._MusicControlAction is not None:
            logger.info("ROS2Bridge ready (MusicControlAction + massage srv)")

    def _invoke_on_spin_thread(self, fn: Callable[[], Any], *, timeout_s: float) -> Any:
        invoke = getattr(self._node, "invoke_on_spin_thread", None)
        if invoke is None:
            return fn()
        fut: concurrent.futures.Future = invoke(fn)
        return fut.result(timeout=timeout_s + 1.0)

    def _get_or_create_client(self, service_name: str, service_type: Type):
        if service_type is None:
            return None
        key = f"{service_type.__name__}:{service_name}"
        if key not in self._clients:
            self._clients[key] = self._node.create_client(service_type, service_name)
        return self._clients[key]

    def _call_sync(self, client, req, *, timeout_s: float, service_name: str):
        """在 doubao-ros-invoke 工作线程执行；由主线程 MultiThreadedExecutor 处理响应回调。"""
        if not client.wait_for_service(timeout_sec=min(timeout_s, 5.0)):
            raise TimeoutError(f"service unavailable: {service_name}")
        fut = client.call_async(req)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if fut.done():
                return fut.result()
            time.sleep(0.01)
        raise TimeoutError(f"timeout calling {service_name}")

    def _resolve_massage_srv_type(self, service_name: str) -> Optional[Type]:
        type_name = _MASSAGE_SERVICE_TYPES.get(service_name.strip())
        if not type_name:
            return None
        return self._srv_types.get(type_name)

    def _call_empty_request_srv_sync(
        self, service_name: str, srv_type: Type, *, timeout_s: float = 3.0
    ) -> ROS2ActionResult:
        if self._rclpy is None:
            return ROS2ActionResult(ok=False, message="rclpy not available")

        client = self._get_or_create_client(service_name, srv_type)
        if client is None:
            return ROS2ActionResult(ok=False, message="client init failed")

        req = srv_type.Request()
        try:
            resp = self._call_sync(client, req, timeout_s=timeout_s, service_name=service_name)
        except Exception as e:
            return ROS2ActionResult(ok=False, message=str(e))
        return ROS2ActionResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
        )

    def _call_music_control_sync(
        self,
        action: str,
        *,
        service_name: str = MUSIC_CONTROL_SERVICE,
        timeout_s: float = 10.0,
    ) -> MusicControlResult:
        if self._MusicControlAction is None or self._rclpy is None:
            detail = "; ".join(self._import_errors) if self._import_errors else "MusicControlAction import failed"
            return MusicControlResult(ok=False, message=f"robot_interfaces not available ({detail})")

        client = self._get_or_create_client(service_name, self._MusicControlAction)
        if client is None:
            return MusicControlResult(ok=False, message="client init failed")

        req = self._MusicControlAction.Request()
        req.action = str(action).strip().lower()
        try:
            resp = self._call_sync(client, req, timeout_s=timeout_s, service_name=service_name)
        except Exception as e:
            return MusicControlResult(ok=False, message=str(e))
        return MusicControlResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
            current_file=str(getattr(resp, "current_file", "")),
        )

    async def call_massage_service(self, service_name: str, *, timeout_s: float = 3.0) -> ROS2ActionResult:
        srv_type = self._resolve_massage_srv_type(service_name)
        if srv_type is None:
            return ROS2ActionResult(
                ok=False,
                message=f"unknown or unsupported massage service: {service_name}",
            )
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_empty_request_srv_sync(service_name, srv_type, timeout_s=timeout_s),
                timeout_s=timeout_s,
            )
        )

    async def call_music_control(
        self,
        action: str,
        *,
        service_name: str = MUSIC_CONTROL_SERVICE,
        timeout_s: float = 10.0,
    ) -> MusicControlResult:
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_music_control_sync(
                    action, service_name=service_name, timeout_s=timeout_s
                ),
                timeout_s=timeout_s,
            )
        )

    def _normalize_volume_percent(self, volume: float) -> float:
        v = float(volume)
        if 0.0 <= v <= 1.0:
            v *= 100.0
        return max(0.0, min(100.0, v))

    def _call_music_volume_sync(
        self,
        volume_percent: float,
        *,
        timeout_s: float = 5.0,
    ) -> MusicVolumeResult:
        if self._MusicControlVolume is None or self._rclpy is None:
            detail = "; ".join(self._import_errors) if self._import_errors else "MusicControlVolume import failed"
            return MusicVolumeResult(ok=False, message=f"robot_interfaces not available ({detail})")

        client = self._get_or_create_client(MUSIC_VOLUME_SERVICE, self._MusicControlVolume)
        if client is None:
            return MusicVolumeResult(ok=False, message="client init failed")

        req = self._MusicControlVolume.Request()
        req.volume = float(self._normalize_volume_percent(volume_percent))
        try:
            resp = self._call_sync(client, req, timeout_s=timeout_s, service_name=MUSIC_VOLUME_SERVICE)
        except Exception as e:
            return MusicVolumeResult(ok=False, message=str(e))
        return MusicVolumeResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
            current_volume=self._normalize_volume_percent(
                float(getattr(resp, "current_volume", 0.0))
            ),
        )

    def _call_music_get_status_sync(self, *, timeout_s: float = 3.0) -> MusicVolumeResult:
        if self._MusicGetStatus is None or self._rclpy is None:
            return MusicVolumeResult(ok=False, message="MusicGetStatus not available")

        client = self._get_or_create_client(MUSIC_STATUS_SERVICE, self._MusicGetStatus)
        if client is None:
            return MusicVolumeResult(ok=False, message="client init failed")

        req = self._MusicGetStatus.Request()
        try:
            resp = self._call_sync(client, req, timeout_s=timeout_s, service_name=MUSIC_STATUS_SERVICE)
        except Exception as e:
            return MusicVolumeResult(ok=False, message=str(e))
        if not bool(getattr(resp, "success", False)):
            return MusicVolumeResult(ok=False, message=str(getattr(resp, "message", "")))
        return MusicVolumeResult(
            ok=True,
            message=str(getattr(resp, "message", "")),
            current_volume=self._normalize_volume_percent(float(getattr(resp, "volume", 50.0))),
        )

    async def get_music_volume(self, *, timeout_s: float = 3.0) -> MusicVolumeResult:
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_music_get_status_sync(timeout_s=timeout_s),
                timeout_s=timeout_s,
            )
        )

    async def set_music_volume(
        self,
        volume_percent: float,
        *,
        timeout_s: float = 5.0,
    ) -> MusicVolumeResult:
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_music_volume_sync(volume_percent, timeout_s=timeout_s),
                timeout_s=timeout_s,
            )
        )

    async def adjust_music_volume(
        self,
        delta_percent: float,
        *,
        min_percent: float = 0.0,
        max_percent: float = 100.0,
        timeout_s: float = 5.0,
    ) -> MusicVolumeResult:
        cur = await self.get_music_volume(timeout_s=timeout_s)
        base = cur.current_volume if cur.ok else 50.0
        target = max(min_percent, min(max_percent, base + float(delta_percent)))
        return await self.set_music_volume(target, timeout_s=timeout_s)

    def set_music_volume_sync(
        self,
        volume_percent: float,
        *,
        timeout_s: float = 5.0,
    ) -> MusicVolumeResult:
        return self._invoke_on_spin_thread(
            lambda: self._call_music_volume_sync(volume_percent, timeout_s=timeout_s),
            timeout_s=timeout_s,
        )

    def _resolve_param_srv_type(self, service_name: str) -> Optional[Type]:
        type_name = _MASSAGE_PARAM_SERVICE_TYPES.get(service_name.strip())
        if not type_name:
            return None
        return self._srv_types.get(type_name)

    def _call_massage_position_sync(
        self, direction: str, *, timeout_s: float = 5.0
    ) -> MassageParamResult:
        srv_type = self._resolve_param_srv_type(MASSAGE_POSITION_SERVICE)
        if srv_type is None or self._rclpy is None:
            return MassageParamResult(ok=False, message="MassagePositionControlReq not available")
        command = map_position_direction(direction)
        if not command:
            return MassageParamResult(ok=False, message=f"无效方向: {direction}")
        client = self._get_or_create_client(MASSAGE_POSITION_SERVICE, srv_type)
        if client is None:
            return MassageParamResult(ok=False, message="client init failed")
        req = srv_type.Request()
        req.command = command
        try:
            resp = self._call_sync(
                client, req, timeout_s=timeout_s, service_name=MASSAGE_POSITION_SERVICE
            )
        except Exception as e:
            return MassageParamResult(ok=False, message=str(e))
        return MassageParamResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
        )

    def _call_massage_duration_sync(
        self,
        command: str,
        *,
        duration_value: int = 0,
        duration_delta: int = 0,
        timeout_s: float = 5.0,
    ) -> MassageParamResult:
        srv_type = self._resolve_param_srv_type(MASSAGE_DURATION_SERVICE)
        if srv_type is None or self._rclpy is None:
            return MassageParamResult(ok=False, message="MassageDurationControlReq not available")
        client = self._get_or_create_client(MASSAGE_DURATION_SERVICE, srv_type)
        if client is None:
            return MassageParamResult(ok=False, message="client init failed")
        req = srv_type.Request()
        req.command = command
        req.duration_value = int(duration_value)
        req.duration_delta = int(duration_delta)
        try:
            resp = self._call_sync(
                client, req, timeout_s=timeout_s, service_name=MASSAGE_DURATION_SERVICE
            )
        except Exception as e:
            return MassageParamResult(ok=False, message=str(e))
        return MassageParamResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
            remaining_duration=int(getattr(resp, "remaining_duration", 0)),
        )

    def _call_massage_force_sync(
        self,
        command: str,
        *,
        force_value: float = 0.0,
        force_delta: float = 0.0,
        timeout_s: float = 5.0,
    ) -> MassageParamResult:
        srv_type = self._resolve_param_srv_type(MASSAGE_FORCE_SERVICE)
        if srv_type is None or self._rclpy is None:
            return MassageParamResult(ok=False, message="MassageForceControlReq not available")
        client = self._get_or_create_client(MASSAGE_FORCE_SERVICE, srv_type)
        if client is None:
            return MassageParamResult(ok=False, message="client init failed")
        req = srv_type.Request()
        req.command = command
        req.force_value = float(force_value)
        req.force_delta = float(force_delta)
        try:
            resp = self._call_sync(
                client, req, timeout_s=timeout_s, service_name=MASSAGE_FORCE_SERVICE
            )
        except Exception as e:
            return MassageParamResult(ok=False, message=str(e))
        return MassageParamResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
            current_force=float(getattr(resp, "current_force", 0.0)),
        )

    def _call_massage_process_info_sync(self, *, timeout_s: float = 5.0) -> MassageParamResult:
        srv_type = self._resolve_param_srv_type(MASSAGE_PROCESS_INFO_SERVICE)
        if srv_type is None or self._rclpy is None:
            return MassageParamResult(ok=False, message="MassageProcessInfoGetReq not available")
        client = self._get_or_create_client(MASSAGE_PROCESS_INFO_SERVICE, srv_type)
        if client is None:
            return MassageParamResult(ok=False, message="client init failed")
        req = srv_type.Request()
        req.attributes = ["process_info"]
        try:
            resp = self._call_sync(
                client, req, timeout_s=timeout_s, service_name=MASSAGE_PROCESS_INFO_SERVICE
            )
        except Exception as e:
            return MassageParamResult(ok=False, message=str(e))
        if not bool(getattr(resp, "success", False)):
            return MassageParamResult(ok=False, message=str(getattr(resp, "message", "查询失败")))
        summary = format_massage_progress_summary(str(getattr(resp, "massage_process_data", "")))
        return MassageParamResult(ok=True, message=summary)

    def _call_head_param_sync(self, params_json: str, *, timeout_s: float = 5.0) -> MassageParamResult:
        srv_type = self._resolve_param_srv_type(MASSAGE_HEAD_PARAM_SERVICE)
        if srv_type is None or self._rclpy is None:
            return MassageParamResult(ok=False, message="MassageHeadParamControlReq not available")
        client = self._get_or_create_client(MASSAGE_HEAD_PARAM_SERVICE, srv_type)
        if client is None:
            return MassageParamResult(ok=False, message="client init failed")
        req = srv_type.Request()
        req.params = str(params_json)
        try:
            resp = self._call_sync(
                client, req, timeout_s=timeout_s, service_name=MASSAGE_HEAD_PARAM_SERVICE
            )
        except Exception as e:
            return MassageParamResult(ok=False, message=str(e))
        return MassageParamResult(
            ok=bool(getattr(resp, "success", False)),
            message=str(getattr(resp, "message", "")),
        )

    async def call_massage_position(
        self, direction: str, *, timeout_s: float = 5.0
    ) -> MassageParamResult:
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_massage_position_sync(direction, timeout_s=timeout_s),
                timeout_s=timeout_s,
            )
        )

    async def call_massage_duration_from_tag(
        self, tag_arg: Optional[str], *, timeout_s: float = 5.0
    ) -> MassageParamResult:
        command, duration_value, duration_delta = parse_duration_from_tag(tag_arg)
        if not command:
            return MassageParamResult(ok=False, message="请指定时长调整方式（longer/shorter/set）")
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_massage_duration_sync(
                    command,
                    duration_value=duration_value,
                    duration_delta=duration_delta,
                    timeout_s=timeout_s,
                ),
                timeout_s=timeout_s,
            )
        )

    async def call_massage_force_from_tag(
        self, tag_arg: Optional[str], *, timeout_s: float = 5.0
    ) -> MassageParamResult:
        command, force_value, force_delta = parse_force_from_tag(tag_arg)
        if not command:
            return MassageParamResult(ok=False, message="请指定力度调整方式（increase/decrease/set）")
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_massage_force_sync(
                    command,
                    force_value=force_value,
                    force_delta=force_delta,
                    timeout_s=timeout_s,
                ),
                timeout_s=timeout_s,
            )
        )

    async def get_massage_progress_summary(self, *, timeout_s: float = 5.0) -> MassageParamResult:
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_massage_process_info_sync(timeout_s=timeout_s),
                timeout_s=timeout_s,
            )
        )

    async def call_head_param_from_tag(
        self, tag_arg: Optional[str], *, timeout_s: float = 5.0
    ) -> MassageParamResult:
        feature, action, value = parse_head_param_tag(tag_arg)
        if not feature or not action:
            return MassageParamResult(ok=False, message="按摩头参数标签格式无效")
        params_json, err = build_head_param_json(feature, action, value)
        if not params_json:
            return MassageParamResult(ok=False, message=err or "参数构建失败")
        return await asyncio.to_thread(
            lambda: self._invoke_on_spin_thread(
                lambda: self._call_head_param_sync(params_json, timeout_s=timeout_s),
                timeout_s=timeout_s,
            )
        )
