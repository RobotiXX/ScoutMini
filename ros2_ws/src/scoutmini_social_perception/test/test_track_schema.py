from scoutmini_social_perception.track_schema import (
    TrackObservation,
    TrackReconciler,
    circular_pixel_distance,
)


def observation(upstream_id, x, width=1920, y=320):
    return TrackObservation(upstream_id, x, y, width, 640)


def test_circular_pixel_distance_wraps_at_panorama_seam():
    assert circular_pixel_distance(1910, 10, 1920) == 20


def test_upstream_id_remains_stable():
    reconciler = TrackReconciler()
    first = reconciler.reconcile([observation('7', 500)], 1.0)
    second = reconciler.reconcile([observation('7', 510)], 1.1)
    assert second == first


def test_upstream_id_switch_is_reconciled_spatially():
    reconciler = TrackReconciler()
    first = reconciler.reconcile([observation('7', 500)], 1.0)
    second = reconciler.reconcile([observation('22', 510)], 1.1)
    assert second == first


def test_seam_crossing_preserves_public_id():
    reconciler = TrackReconciler(max_match_distance_norm=0.05)
    first = reconciler.reconcile([observation('7', 1905)], 1.0)
    second = reconciler.reconcile([observation('22', 12)], 1.1)
    assert second == first


def test_two_people_cannot_receive_same_public_id():
    reconciler = TrackReconciler(max_match_distance_norm=0.20)
    public_ids = reconciler.reconcile(
        [observation(None, 500), observation(None, 505)],
        1.0,
    )
    assert len(set(public_ids)) == 2


def test_expired_track_is_not_reused():
    reconciler = TrackReconciler(timeout_sec=0.5)
    first = reconciler.reconcile([observation('7', 500)], 1.0)
    second = reconciler.reconcile([observation('8', 505)], 2.0)
    assert second != first


def test_reset_drops_associations():
    reconciler = TrackReconciler()
    first = reconciler.reconcile([observation('7', 500)], 1.0)
    reconciler.reset()
    second = reconciler.reconcile([observation('7', 500)], 1.1)
    assert second != first


def test_timestamp_regression_resets_associations():
    reconciler = TrackReconciler()
    first = reconciler.reconcile([observation('7', 500)], 10.0)
    second = reconciler.reconcile([observation('7', 500)], 2.0)
    assert second != first
