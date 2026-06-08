from __future__ import annotations

from collections import defaultdict
from typing import Callable, Dict, List, Any


EventHandler = Callable[[Any], None]


class EventDispatcher:
    def __init__(self):
        self._handlers: Dict[str, List[EventHandler]] = defaultdict(list)

    def on(self, event_name: str, handler: EventHandler) -> None:
        self._handlers[event_name].append(handler)

    def emit(self, event_name: str, payload: Any) -> None:
        for handler in self._handlers.get(event_name, []):
            handler(payload)
