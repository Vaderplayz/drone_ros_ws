from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LandingPhase(str, Enum):
    IDLE = "IDLE"
    ACTIVATING = "ACTIVATING"
    SEARCHING = "SEARCHING"
    TARGET_DETECTED = "TARGET_DETECTED"
    ALIGNING = "ALIGNING"
    CENTERED = "CENTERED"
    DESCENDING = "DESCENDING"
    LANDING = "LANDING"
    LANDED = "LANDED"
    TARGET_LOST = "TARGET_LOST"
    ABORTED = "ABORTED"
    ERROR = "ERROR"


@dataclass
class LandingState:
    phase: LandingPhase = LandingPhase.IDLE
    detected: bool = False
    tag_id: int = -1
    confidence: float = 0.0
    center_x: float = 0.0
    center_y: float = 0.0
    image_width: int = 0
    image_height: int = 0
    error_x: float = 0.0
    error_y: float = 0.0
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_stamp: float = 0.0
    corners: tuple[tuple[float, float], ...] = ()
    detail: str = ""


class LandingStateMachine:
    _allowed = {
        LandingPhase.IDLE: {LandingPhase.ACTIVATING, LandingPhase.ABORTED, LandingPhase.ERROR},
        LandingPhase.ACTIVATING: {LandingPhase.SEARCHING, LandingPhase.ERROR, LandingPhase.ABORTED},
        LandingPhase.SEARCHING: {
            LandingPhase.TARGET_DETECTED,
            LandingPhase.TARGET_LOST,
            LandingPhase.ABORTED,
            LandingPhase.ERROR,
        },
        LandingPhase.TARGET_DETECTED: {
            LandingPhase.ALIGNING,
            LandingPhase.CENTERED,
            LandingPhase.TARGET_LOST,
            LandingPhase.ABORTED,
            LandingPhase.ERROR,
        },
        LandingPhase.ALIGNING: {
            LandingPhase.CENTERED,
            LandingPhase.TARGET_LOST,
            LandingPhase.ABORTED,
            LandingPhase.ERROR,
        },
        LandingPhase.CENTERED: {
            LandingPhase.ALIGNING,
            LandingPhase.DESCENDING,
            LandingPhase.TARGET_LOST,
            LandingPhase.ABORTED,
            LandingPhase.ERROR,
        },
        LandingPhase.DESCENDING: {
            LandingPhase.ALIGNING,
            LandingPhase.LANDING,
            LandingPhase.TARGET_LOST,
            LandingPhase.ABORTED,
            LandingPhase.ERROR,
        },
        LandingPhase.LANDING: {LandingPhase.LANDED, LandingPhase.ABORTED, LandingPhase.ERROR},
        LandingPhase.LANDED: {LandingPhase.IDLE},
        LandingPhase.TARGET_LOST: {
            LandingPhase.SEARCHING,
            LandingPhase.TARGET_DETECTED,
            LandingPhase.ABORTED,
            LandingPhase.ERROR,
        },
        LandingPhase.ABORTED: {LandingPhase.IDLE, LandingPhase.ACTIVATING},
        LandingPhase.ERROR: {LandingPhase.IDLE, LandingPhase.ABORTED},
    }

    def __init__(self) -> None:
        self.state = LandingState()

    def transition(self, target: LandingPhase, detail: str = "") -> LandingState:
        if target == self.state.phase:
            self.state.detail = detail or self.state.detail
            return self.state
        if target not in self._allowed[self.state.phase]:
            raise ValueError(f"invalid landing transition {self.state.phase.value} -> {target.value}")
        self.state.phase = target
        self.state.detail = detail
        return self.state

    @staticmethod
    def from_status_text(text: str) -> LandingPhase:
        upper = text.upper()
        aliases = {
            "WAIT_INPUT": LandingPhase.SEARCHING,
            "WAIT_OFFBOARD": LandingPhase.ACTIVATING,
            "APPROACH": LandingPhase.ALIGNING,
            "DESCEND": LandingPhase.DESCENDING,
            "AUTO_LAND": LandingPhase.LANDING,
        }
        for token, phase in aliases.items():
            if token in upper:
                return phase
        for phase in LandingPhase:
            if phase.value in upper:
                return phase
        return LandingPhase.ERROR
