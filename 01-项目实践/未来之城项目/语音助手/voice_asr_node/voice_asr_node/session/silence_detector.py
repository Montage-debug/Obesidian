from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import time


class DialogPhase(Enum):
    WAKEUP = "wakeup"
    DIALOG = "dialog"


@dataclass
class SilenceConfig:
    wakeup_timeout_s: float = 5.0
    dialog_timeout_s: float = 10.0


class SilenceDetector:
    def __init__(self, cfg: SilenceConfig):
        self._cfg = cfg
        self._last_activity_ts = time.time()

    def on_activity(self) -> None:
        self._last_activity_ts = time.time()

    def is_timeout(self, phase: DialogPhase) -> bool:
        elapsed = time.time() - self._last_activity_ts
        if phase == DialogPhase.WAKEUP:
            return elapsed >= self._cfg.wakeup_timeout_s
        return elapsed >= self._cfg.dialog_timeout_s
