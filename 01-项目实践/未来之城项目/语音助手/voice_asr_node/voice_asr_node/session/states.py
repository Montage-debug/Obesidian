from enum import Enum


class SessionState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    SESSION_STARTING = "session_starting"
    SESSION_ACTIVE = "session_active"
