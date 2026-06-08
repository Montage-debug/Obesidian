from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Dict, Any


@dataclass
class DialogContext:
    dialog_id: str = ""
    messages: List[Dict[str, Any]] = field(default_factory=list)

    def append(self, role: str, text: str) -> None:
        self.messages.append({"role": role, "text": text})
