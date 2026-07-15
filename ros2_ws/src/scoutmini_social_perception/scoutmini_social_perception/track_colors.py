"""Deterministic, high-contrast colors for visualizing track identities."""

from __future__ import annotations

import colorsys
import hashlib
import re
from typing import Tuple


_GOLDEN_RATIO_CONJUGATE = 0.618033988749895


def track_color(track_id: str) -> Tuple[int, int, int]:
    """Return a stable OpenCV BGR color for a track ID."""
    identity = str(track_id)
    numeric_suffix = re.search(r'(\d+)$', identity)
    if numeric_suffix:
        seed = int(numeric_suffix.group(1))
        hue = (seed * _GOLDEN_RATIO_CONJUGATE) % 1.0
    else:
        digest = hashlib.sha256(identity.encode('utf-8')).digest()
        hue = int.from_bytes(digest[:8], 'big') / float(1 << 64)
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
    return (
        int(round(blue * 255)),
        int(round(green * 255)),
        int(round(red * 255)),
    )
