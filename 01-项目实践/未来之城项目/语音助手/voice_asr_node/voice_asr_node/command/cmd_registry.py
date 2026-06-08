from __future__ import annotations

from typing import Any, Dict, List, Optional


class CmdRegistry:
    def __init__(self, raw: Dict[str, Any]):
        self._raw = raw or {}

    @property
    def settings(self) -> Dict[str, Any]:
        return dict(self._raw.get("settings", {}))

    @property
    def commands(self) -> List[Dict[str, Any]]:
        return list(self._raw.get("commands", []))

    @property
    def commands_by_priority(self) -> List[Dict[str, Any]]:
        return sorted(self.commands, key=lambda c: -int(c.get("priority", 0)))

    def find(self, cmd_id: str) -> Optional[Dict[str, Any]]:
        for cmd in self.commands:
            if str(cmd.get("id", "")) == cmd_id:
                return cmd
        return None
