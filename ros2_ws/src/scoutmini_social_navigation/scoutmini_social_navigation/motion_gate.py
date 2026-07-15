"""Pure state machine for bounded, supervised velocity forwarding."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class GateDecision:
    """One bounded output decision from the motion gate."""

    linear: float
    angular: float
    state: str
    armed: bool
    input_age_sec: float
    remaining_sec: float


class MotionGate:
    """Require explicit arming and stop on stale input or time limit."""

    def __init__(self, max_linear: float, max_angular: float,
                 input_timeout: float, max_duration: float) -> None:
        if min(max_linear, max_angular, input_timeout, max_duration) <= 0.0:
            raise ValueError('motion gate limits must be positive')
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.input_timeout = input_timeout
        self.max_duration = max_duration
        self.armed = False
        self._armed_at: Optional[float] = None
        self._input_at: Optional[float] = None
        self._linear = 0.0
        self._angular = 0.0

    def arm(self, now_sec: float) -> None:
        """Arm a new bounded interval, initially commanding zero."""
        self.armed = True
        self._armed_at = now_sec
        self._input_at = None
        self._linear = 0.0
        self._angular = 0.0

    def disarm(self) -> None:
        """Latch the gate stopped until explicitly rearmed."""
        self.armed = False
        self._armed_at = None
        self._input_at = None
        self._linear = 0.0
        self._angular = 0.0

    def observe(self, linear: float, angular: float, now_sec: float) -> bool:
        """Store one finite shadow command for the next timer decision."""
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._input_at = None
            self._linear = 0.0
            self._angular = 0.0
            return False
        self._input_at = now_sec
        self._linear = linear
        self._angular = angular
        return True

    def evaluate(self, now_sec: float) -> GateDecision:
        """Return a safe command and update an expired gate to disarmed."""
        if not self.armed or self._armed_at is None:
            return self._stopped('disarmed')
        elapsed = max(0.0, now_sec - self._armed_at)
        remaining = max(0.0, self.max_duration - elapsed)
        if elapsed >= self.max_duration:
            self.disarm()
            return self._stopped('duration expired')
        if self._input_at is None:
            return self._stopped('waiting for fresh input', True, remaining)
        input_age = max(0.0, now_sec - self._input_at)
        if input_age > self.input_timeout:
            return self._stopped('input stale', True, remaining, input_age)
        return GateDecision(
            linear=max(-self.max_linear, min(self.max_linear, self._linear)),
            angular=max(-self.max_angular, min(self.max_angular, self._angular)),
            state='forwarding bounded command',
            armed=True,
            input_age_sec=input_age,
            remaining_sec=remaining,
        )

    @staticmethod
    def _stopped(state: str, armed: bool = False, remaining: float = 0.0,
                 input_age: float = float('inf')) -> GateDecision:
        return GateDecision(0.0, 0.0, state, armed, input_age, remaining)
