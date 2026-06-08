from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class CmdResult:
    cmd_id: str
    action: str
    reply: str = ""
    arg: Optional[str] = None
