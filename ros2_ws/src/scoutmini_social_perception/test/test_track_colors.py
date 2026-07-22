"""Tests for deterministic track visualization colors."""

from scoutmini_social_perception.track_colors import track_color


def test_track_color_is_stable_for_numeric_and_text_ids():
    assert track_color('person_7') == track_color('person_7')
    assert track_color('baseline') == track_color('baseline')


def test_consecutive_ids_get_distinct_high_contrast_colors():
    colors = [track_color(f'person_{index}') for index in range(12)]

    assert len(set(colors)) == len(colors)
    assert all(min(color) <= 40 and max(color) >= 240 for color in colors)
