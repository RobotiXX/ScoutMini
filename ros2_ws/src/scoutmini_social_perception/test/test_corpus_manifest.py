"""Tests for deterministic grouped corpus splits."""

from scoutmini_social_perception.corpus_manifest import (
    assign_splits,
    sample_segments,
)


def test_assign_splits_keeps_groups_together_and_nonempty():
    records = [
        {'group': 'session-a'},
        {'group': 'session-a'},
        {'group': 'session-b'},
        {'group': 'session-c'},
    ]
    assign_splits(records, 'seed', 0.2)
    assert {record['split'] for record in records} == {'tune', 'holdout'}
    assert records[0]['split'] == records[1]['split']
    first = [record['split'] for record in records]
    assign_splits(records, 'seed', 0.2)
    assert [record['split'] for record in records] == first


def test_sample_segments_uses_full_short_bag_and_windows_long_bag():
    assert sample_segments(10.0, 180.0, 3) == [{
        'id': 'full',
        'start_offset_sec': 0.0,
        'duration_sec': 10.0,
    }]
    segments = sample_segments(600.0, 180.0, 3)
    assert len(segments) == 3
    assert all(segment['duration_sec'] == 60.0 for segment in segments)
    assert segments[0]['start_offset_sec'] < segments[1]['start_offset_sec']
