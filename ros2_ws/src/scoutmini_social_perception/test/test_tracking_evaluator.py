"""Tests for intrinsic and baseline-assisted tracking evaluation."""

from scoutmini_social_perception.tracking_evaluator import (
    BoxObservation,
    TrackFrame,
    evaluate_tracks,
    greedy_matches,
)


def observation(track_id, center_x, center_y=50.0):
    """Create a stable 20-by-40 test box."""
    return BoxObservation(
        track_id,
        (center_x - 10.0, center_y - 20.0,
         center_x + 10.0, center_y + 20.0),
        0.9,
    )


def test_greedy_matches_returns_ids_and_iou():
    first = observation('ours', 50.0)
    second = observation('baseline', 50.0)
    assert greedy_matches([first], [second]) == [(first, second, 1.0)]


def test_evaluator_finds_fragmentation_and_baseline_association_change():
    current = [
        TrackFrame(1_000_000_000, (observation('one', 50.0),)),
        TrackFrame(1_125_000_000, (observation('one', 52.0),)),
        TrackFrame(1_250_000_000, (observation('two', 54.0),)),
    ]
    reference = [
        TrackFrame(1_000_000_000, (observation('baseline', 50.0),)),
        TrackFrame(1_125_000_000, (observation('baseline', 52.0),)),
        TrackFrame(1_250_000_000, (observation('baseline', 54.0),)),
    ]
    metrics, events = evaluate_tracks(
        current,
        reference,
        image_size=(100, 100),
    )
    assert metrics['suspected_fragmentations'] == 1
    assert metrics['baseline_association_changes'] == 1
    assert metrics['baseline_ids_split_across_multiple_current_ids'] == 1
    assert {event['event_type'] for event in events} == {
        'suspected_id_fragmentation',
        'baseline_association_change',
    }
