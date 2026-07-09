"""Shared JSON schema helpers for people perception topics."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
import math
from typing import Any, Dict, Iterable, List, Optional

from builtin_interfaces.msg import Time
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


SCHEMA_VERSION = 'scoutmini.people.v1'


@dataclass
class Detection2D:
    label: str
    confidence: float
    bbox_xyxy: List[float]
    image_width: int
    image_height: int
    track_id: Optional[int] = None
    source: str = 'detector'

    @property
    def center(self) -> List[float]:
        x1, y1, x2, y2 = self.bbox_xyxy
        return [(x1 + x2) * 0.5, (y1 + y2) * 0.5]


@dataclass
class Track2D:
    track_id: int
    label: str
    confidence: float
    bbox_xyxy: List[float]
    image_width: int
    image_height: int
    bearing_rad: float
    elevation_rad: float = 0.0
    age_sec: float = 0.0
    source: str = 'tracker'


@dataclass
class ProjectedPerson:
    track_id: int
    label: str
    confidence: float
    x: float
    y: float
    z: float
    range_m: float
    bearing_rad: float
    vx: float = 0.0
    vy: float = 0.0
    range_source: str = 'unknown'
    source: str = 'projection'


@dataclass
class PeopleFrame:
    stamp_sec: float
    frame_id: str
    people: List[Dict[str, Any]] = field(default_factory=list)
    schema: str = SCHEMA_VERSION

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(',', ':'), sort_keys=True)


def stamp_to_float(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def make_people_frame(stamp: Time, frame_id: str, people: Iterable[Any]) -> PeopleFrame:
    normalized = []
    for person in people:
        if hasattr(person, '__dataclass_fields__'):
            normalized.append(asdict(person))
        else:
            normalized.append(dict(person))
    return PeopleFrame(stamp_sec=stamp_to_float(stamp), frame_id=frame_id, people=normalized)


def parse_people_frame(data: str) -> PeopleFrame:
    raw = json.loads(data)
    return PeopleFrame(
        stamp_sec=float(raw.get('stamp_sec', 0.0)),
        frame_id=str(raw.get('frame_id', '')),
        people=list(raw.get('people', [])),
        schema=str(raw.get('schema', SCHEMA_VERSION)),
    )


def bearing_from_equirectangular(
    x_px: float,
    image_width: int,
    center_x_norm: float = 0.5,
    bearing_offset_rad: float = 0.0,
) -> float:
    if image_width <= 0:
        return 0.0
    bearing = ((x_px / float(image_width)) - center_x_norm) * 2.0 * math.pi
    return normalize_angle(bearing + bearing_offset_rad)


def elevation_from_equirectangular(y_px: float, image_height: int) -> float:
    if image_height <= 0:
        return 0.0
    return -(((y_px / float(image_height)) - 0.5) * math.pi)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    msg = ColorRGBA()
    msg.r = r
    msg.g = g
    msg.b = b
    msg.a = a
    return msg


def make_projected_markers(
    stamp: Time,
    frame_id: str,
    people: Iterable[Dict[str, Any]],
    namespace: str = 'projected_people',
) -> MarkerArray:
    markers = MarkerArray()
    active_ids = set()
    for person in people:
        track_id = int(person.get('track_id', 0))
        active_ids.add(track_id)

        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        body = Marker()
        body.header = header
        body.ns = namespace
        body.id = track_id
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = float(person.get('x', 0.0))
        body.pose.position.y = float(person.get('y', 0.0))
        body.pose.position.z = 0.85
        body.pose.orientation.w = 1.0
        body.scale.x = 0.45
        body.scale.y = 0.45
        body.scale.z = 1.7
        body.color = _color(0.0, 0.8, 1.0, 0.75)
        body.lifetime.sec = 1
        markers.markers.append(body)

        text = Marker()
        text.header = header
        text.ns = f'{namespace}_labels'
        text.id = track_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = body.pose.position.x
        text.pose.position.y = body.pose.position.y
        text.pose.position.z = 1.9
        text.pose.orientation.w = 1.0
        text.scale.z = 0.28
        text.color = _color(1.0, 1.0, 1.0, 0.95)
        text.text = f"id {track_id} {float(person.get('range_m', 0.0)):.1f}m"
        text.lifetime.sec = 1
        markers.markers.append(text)

    return markers


def make_image_markers(
    stamp: Time,
    frame_id: str,
    people: Iterable[Dict[str, Any]],
    namespace: str,
) -> MarkerArray:
    markers = MarkerArray()
    for idx, person in enumerate(people):
        raw_track_id = person.get('track_id', idx)
        marker_id = idx if raw_track_id is None else int(raw_track_id)
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(idx) * 0.25
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color = _color(1.0, 0.8, 0.0, 0.95)
        marker.text = f"{person.get('label', 'person')} {person.get('confidence', 0.0):.2f}"
        marker.lifetime.sec = 1
        markers.markers.append(marker)
    return markers
