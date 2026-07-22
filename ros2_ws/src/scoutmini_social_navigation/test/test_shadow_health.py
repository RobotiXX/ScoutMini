"""Tests for AdaSCoRe shadow output freshness."""

from scoutmini_social_navigation.shadow_health import ShadowHealth


def test_reports_stale_output_while_people_remain_fresh():
    health = ShadowHealth()
    health.observe_action(True)
    health.observe_output(1.0)
    health.observe_people(2.0)

    level, reason, _, output_age = health.evaluate(2.2, 0.75, 1.0)

    assert level == 2
    assert 'output stale' in reason
    assert abs(output_age - 1.2) < 1e-9
    assert health.stale_transitions == 1


def test_counts_only_new_stale_transitions():
    health = ShadowHealth()
    health.observe_action(True)
    health.observe_output(1.0)
    health.observe_people(2.0)
    health.evaluate(2.2, 0.75, 1.0)
    health.evaluate(2.3, 0.75, 1.0)
    health.observe_output(2.4)
    health.evaluate(2.4, 0.75, 1.0)
    health.observe_people(4.0)
    health.evaluate(4.1, 0.75, 1.0)

    assert health.stale_transitions == 2


def test_stale_people_does_not_blame_controller():
    health = ShadowHealth()
    health.observe_action(True)
    health.observe_output(1.0)
    health.observe_people(1.0)

    level, reason, _, _ = health.evaluate(3.0, 0.75, 1.0)

    assert level == 1
    assert reason == 'people input is stale'


def test_inactive_action_does_not_report_stale_output():
    health = ShadowHealth()
    health.observe_output(1.0)
    health.observe_people(2.0)

    level, reason, _, _ = health.evaluate(2.2, 0.75, 1.0)

    assert level == 0
    assert reason == 'controller action is inactive'


def test_new_action_gets_output_grace_after_old_command():
    health = ShadowHealth()
    health.observe_output(1.0)
    health.observe_people(10.0)
    health.observe_action(True, 10.0)

    level, reason, _, output_age = health.evaluate(10.2, 0.75, 1.0)

    assert level == 1
    assert reason == 'waiting for people and controller output'
    assert abs(output_age - 0.2) < 1e-9

    health.observe_people(11.2)
    level, reason, _, output_age = health.evaluate(11.2, 0.75, 1.0)

    assert level == 2
    assert 'output stale' in reason
    assert abs(output_age - 1.2) < 1e-9
