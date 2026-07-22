"""Freshness state for the AdaSCoRe shadow controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class ShadowHealth:
    """Track whether controller output keeps pace with live people input."""

    last_people_sec: Optional[float] = None
    last_output_sec: Optional[float] = None
    people_messages: int = 0
    output_messages: int = 0
    stale_transitions: int = 0
    controller_active: bool = False
    action_started_sec: Optional[float] = None
    _was_stale: bool = False

    def observe_people(self, now_sec: float) -> None:
        """Record one people input message."""
        self.last_people_sec = now_sec
        self.people_messages += 1

    def observe_output(self, now_sec: float) -> None:
        """Record one controller output message."""
        self.last_output_sec = now_sec
        self.output_messages += 1

    def observe_action(self, active: bool, now_sec: Optional[float] = None) -> None:
        """Record whether FollowPath currently has an executing goal."""
        if active and not self.controller_active:
            self.action_started_sec = now_sec
        elif not active:
            self.action_started_sec = None
            self._was_stale = False
        self.controller_active = active

    def evaluate(
        self,
        now_sec: float,
        people_timeout_sec: float,
        output_timeout_sec: float,
    ) -> Tuple[int, str, float, float]:
        """Return diagnostic level, reason, input age, and output age."""
        people_age = self._age(now_sec, self.last_people_sec)
        output_for_action = (
            self.last_output_sec is not None
            and (
                self.action_started_sec is None
                or self.last_output_sec >= self.action_started_sec
            )
        )
        output_reference = (
            self.last_output_sec
            if output_for_action else self.action_started_sec
        )
        output_age = self._age(now_sec, output_reference)
        input_fresh = people_age <= people_timeout_sec
        output_stale = output_age > output_timeout_sec
        stale = (
            self.controller_active
            and input_fresh
            and output_stale
        )
        if stale and not self._was_stale:
            self.stale_transitions += 1
        self._was_stale = stale

        if stale:
            return 2, 'controller output stale while people input is fresh', people_age, output_age
        if not self.controller_active:
            return 0, 'controller action is inactive', people_age, output_age
        if self.people_messages == 0 or not output_for_action:
            return 1, 'waiting for people and controller output', people_age, output_age
        if not input_fresh:
            return 1, 'people input is stale', people_age, output_age
        return 0, 'controller output is fresh', people_age, output_age

    @staticmethod
    def _age(now_sec: float, timestamp: Optional[float]) -> float:
        if timestamp is None:
            return float('inf')
        return max(0.0, now_sec - timestamp)
