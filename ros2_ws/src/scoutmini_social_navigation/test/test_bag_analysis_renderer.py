"""Tests for deterministic bag-analysis rendering helpers."""

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection2D, Detection2DArray

from scoutmini_social_navigation.bag_analysis_renderer import (
    TimedMessages,
    align_receive_time,
    detection_box,
    greedy_match_iou,
    header_stamp_ns,
    scale_detections,
    world_to_panel,
)


def make_detection(center_x=10.0, center_y=20.0, width=8.0, height=12.0):
    """Create a minimal Detection2D for geometry tests."""
    detection = Detection2D()
    detection.id = 'person_7'
    detection.bbox.center.position.x = center_x
    detection.bbox.center.position.y = center_y
    detection.bbox.size_x = width
    detection.bbox.size_y = height
    return detection


def test_timed_messages_uses_nearest_with_tolerance():
    records = TimedMessages([(1_000_000_000, 'first'), (2_000_000_000, 'second')])
    assert records.nearest(1_900_000_000, 0.2) == 'second'
    assert records.nearest(1_500_000_000, 0.1) is None


def test_detection_box_and_iou_matching():
    detection = make_detection()
    assert detection_box(detection) == (6.0, 14.0, 14.0, 26.0)
    assert greedy_match_iou([detection], [make_detection()]) == [1.0]
    assert greedy_match_iou([detection], [make_detection(center_x=100.0)]) == []


def test_scale_detections_does_not_modify_source():
    message = Detection2DArray()
    message.detections.append(make_detection())
    scaled = scale_detections(message, 0.5, 0.25)
    assert detection_box(message.detections[0]) == (6.0, 14.0, 14.0, 26.0)
    assert detection_box(scaled.detections[0]) == (3.0, 3.5, 7.0, 6.5)


def test_world_to_panel_places_forward_point_above_robot():
    odom = Odometry()
    odom.pose.pose.orientation.w = 1.0
    point = Point(x=2.0)
    assert world_to_panel(point, odom) == (256, 346)


def test_marker_array_uses_nested_header_stamp():
    marker = Marker()
    marker.header.stamp.sec = 12
    marker.header.stamp.nanosec = 34
    message = MarkerArray(markers=[marker])
    assert header_stamp_ns(message) == 12_000_000_034


def test_align_receive_time_interpolates_replay_clock():
    anchors = [(100, 1_000), (200, 1_050)]
    assert align_receive_time(150, anchors) == 1_025
    assert align_receive_time(50, anchors) == 950
    assert align_receive_time(250, anchors) == 1_100
