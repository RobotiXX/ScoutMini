"""Tests for the supervised velocity gate state machine."""

from scoutmini_social_navigation.motion_gate import MotionGate


def make_gate():
    return MotionGate(0.05, 0.15, 0.25, 2.0)


def test_gate_requires_explicit_arm_and_fresh_input():
    gate = make_gate()
    assert gate.evaluate(1.0).state == 'disarmed'

    gate.arm(1.0)
    decision = gate.evaluate(1.1)
    assert decision.armed
    assert decision.linear == 0.0
    assert decision.state == 'waiting for fresh input'


def test_gate_clamps_fresh_command_and_stops_when_stale():
    gate = make_gate()
    gate.arm(1.0)
    assert gate.observe(0.30, -0.60, 1.1)

    decision = gate.evaluate(1.2)
    assert decision.linear == 0.05
    assert decision.angular == -0.15

    decision = gate.evaluate(1.4)
    assert decision.linear == 0.0
    assert decision.state == 'input stale'


def test_gate_auto_disarms_at_duration_limit():
    gate = make_gate()
    gate.arm(1.0)
    gate.observe(0.03, 0.0, 2.9)

    decision = gate.evaluate(3.0)
    assert decision.state == 'duration expired'
    assert not decision.armed
    assert gate.evaluate(3.1).state == 'disarmed'


def test_gate_rejects_non_finite_command():
    gate = make_gate()
    gate.arm(1.0)

    assert not gate.observe(float('nan'), 0.0, 1.1)
    assert gate.evaluate(1.2).state == 'waiting for fresh input'
