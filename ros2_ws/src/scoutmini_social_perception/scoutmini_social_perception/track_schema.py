"""Panorama-aware public track ID reconciliation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Set


@dataclass(frozen=True)
class TrackObservation:
    """Minimal detector output used for public ID assignment."""

    upstream_id: Optional[str]
    center_x: float
    center_y: float
    image_width: int
    image_height: int


@dataclass
class _TrackState:
    public_id: str
    upstream_id: Optional[str]
    center_x: float
    center_y: float
    image_width: int
    image_height: int
    last_seen_sec: float


def circular_pixel_distance(x1: float, x2: float, width: int) -> float:
    """Return the shortest horizontal distance on an equirectangular image."""
    if width <= 0:
        return abs(x1 - x2)
    direct = abs(x1 - x2) % float(width)
    return min(direct, float(width) - direct)


class TrackReconciler:
    """Map tracker IDs to stable public IDs across seams and short gaps."""

    def __init__(
        self,
        timeout_sec: float = 1.0,
        max_match_distance_norm: float = 0.12,
    ) -> None:
        self.timeout_sec = max(0.05, float(timeout_sec))
        self.max_match_distance_norm = max(0.01, float(max_match_distance_norm))
        self._next_id = 1
        self._tracks: Dict[str, _TrackState] = {}
        self._upstream_to_public: Dict[str, str] = {}
        self._last_stamp_sec: Optional[float] = None

    def reconcile(
        self,
        observations: Iterable[TrackObservation],
        stamp_sec: float,
    ) -> List[str]:
        """Assign one public ID to each observation in input order."""
        if (
            self._last_stamp_sec is not None
            and stamp_sec < self._last_stamp_sec
        ):
            self.reset()
        self._last_stamp_sec = stamp_sec
        self._expire(stamp_sec)
        assigned: Set[str] = set()
        public_ids: List[str] = []

        for observation in observations:
            public_id = self._mapped_id(observation, assigned)
            if public_id is None:
                public_id = self._nearest_id(observation, assigned)
            if public_id is None:
                public_id = f'person_{self._next_id:06d}'
                self._next_id += 1

            assigned.add(public_id)
            state = _TrackState(
                public_id=public_id,
                upstream_id=observation.upstream_id,
                center_x=observation.center_x,
                center_y=observation.center_y,
                image_width=observation.image_width,
                image_height=observation.image_height,
                last_seen_sec=stamp_sec,
            )
            self._tracks[public_id] = state
            if observation.upstream_id:
                self._upstream_to_public[observation.upstream_id] = public_id
            public_ids.append(public_id)

        return public_ids

    def reset(self) -> None:
        """Drop active state while keeping public IDs monotonic."""
        self._tracks.clear()
        self._upstream_to_public.clear()
        self._last_stamp_sec = None

    def _mapped_id(
        self,
        observation: TrackObservation,
        assigned: Set[str],
    ) -> Optional[str]:
        if not observation.upstream_id:
            return None
        public_id = self._upstream_to_public.get(observation.upstream_id)
        if public_id is None or public_id in assigned:
            return None
        state = self._tracks.get(public_id)
        if state is None:
            return None
        if self._distance_norm(observation, state) > self.max_match_distance_norm * 2.0:
            return None
        return public_id

    def _nearest_id(
        self,
        observation: TrackObservation,
        assigned: Set[str],
    ) -> Optional[str]:
        best_id = None
        best_distance = self.max_match_distance_norm
        for public_id, state in self._tracks.items():
            if public_id in assigned:
                continue
            distance = self._distance_norm(observation, state)
            if distance < best_distance:
                best_distance = distance
                best_id = public_id
        return best_id

    @staticmethod
    def _distance_norm(observation: TrackObservation, state: _TrackState) -> float:
        width = max(1, observation.image_width, state.image_width)
        height = max(1, observation.image_height, state.image_height)
        dx = circular_pixel_distance(observation.center_x, state.center_x, width) / width
        dy = abs(observation.center_y - state.center_y) / height
        return math.hypot(dx, dy)

    def _expire(self, stamp_sec: float) -> None:
        expired = {
            public_id
            for public_id, state in self._tracks.items()
            if stamp_sec - state.last_seen_sec > self.timeout_sec
        }
        for public_id in expired:
            self._tracks.pop(public_id, None)
        self._upstream_to_public = {
            upstream_id: public_id
            for upstream_id, public_id in self._upstream_to_public.items()
            if public_id not in expired
        }
