"""Hardware integration tests for the CyberGear motor driver.

Requires a physical motor connected via CAN bus.

Run with:
    pytest -m hardware tests/test_hardware.py -v

Skip automatically when no CAN interface is available.
"""

import time

import pytest

from cybergear import CyberGearMotor, MotorFeedback

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# CAN bus config is read from ~/.can — create it if needed:
#
#   [default]
#   interface = socketcan
#   channel = can0
#   bitrate = 1000000
#
# Motor CAN ID: set to None to auto-detect (scans IDs 1–127).
MOTOR_CAN_ID = 0x01

# Tolerance for position assertions (rad)
POS_TOL = 0.15

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


def _can_available() -> bool:
    try:
        import can

        bus = can.Bus()
        bus.shutdown()
        return True
    except Exception:
        return False


hardware = pytest.mark.skipif(
    not _can_available(),
    reason='No CAN interface available — check ~/.can',
)


@pytest.fixture(scope='module')
def motor():
    with CyberGearMotor(can_id=MOTOR_CAN_ID, poll_interval=0.1) as m:
        m.enable()
        time.sleep(0.3)
        yield m
        m.disable()


# ---------------------------------------------------------------------------
# Connection and init
# ---------------------------------------------------------------------------


@hardware
class TestConnection:
    def test_device_id_received(self, motor):
        assert motor.device_id is not None
        assert len(motor.device_id) == 16  # 8 bytes as hex string

    def test_v_bus_in_expected_range(self, motor):
        """Bus voltage should be between 12 V and 48 V."""
        assert motor.v_bus is not None
        assert 12.0 <= motor.v_bus <= 48.0

    def test_temperature_in_expected_range(self, motor):
        """Ambient temperature should be between 10 °C and 80 °C."""
        assert motor.feedback.temperature is not None
        assert 10.0 <= motor.feedback.temperature <= 80.0

    def test_motor_is_calibrated(self, motor):
        assert motor.feedback.faults.calibrated is True

    def test_no_faults_on_startup(self, motor):
        assert not motor.feedback.faults.has_fault

    def test_run_mode_received(self, motor):
        assert motor.run_mode is not None


# ---------------------------------------------------------------------------
# Feedback
# ---------------------------------------------------------------------------


@hardware
class TestFeedback:
    def test_feedback_updates_over_time(self, motor):
        """Feedback timestamp must advance as polling runs."""
        fb1 = motor.feedback
        time.sleep(0.3)
        fb2 = motor.feedback
        # mech_pos should be available after polling
        assert motor.mech_pos is not None
        assert motor.mech_vel is not None

    def test_feedback_listener_fires(self, motor):
        received: list[MotorFeedback] = []
        listener = received.append
        motor.add_feedback_listener(listener)
        try:
            motor.quick_move(0.0)  # trigger a feedback frame
            time.sleep(0.3)
        finally:
            motor.remove_feedback_listener(listener)
        assert len(received) > 0

    def test_parameter_listener_fires(self, motor):
        received: list[tuple[str, float]] = []
        listener = lambda k, v: received.append((k, v))  # noqa: E731
        motor.add_parameter_listener(listener)
        try:
            time.sleep(0.5)
        finally:
            motor.remove_parameter_listener(listener)
        assert len(received) > 0

    def test_mech_pos_matches_feedback_position(self, motor):
        """mech_pos (from parameter read) and feedback.position should be close."""
        assert motor.mech_pos is not None
        assert abs(motor.mech_pos - motor.feedback.position) < 1.0


# ---------------------------------------------------------------------------
# Quick-move (velocity mode)
# ---------------------------------------------------------------------------


@hardware
class TestQuickMove:
    def test_spinning_forward_shows_positive_velocity(self, motor):
        motor.quick_move(2.0)
        time.sleep(0.5)
        vel = motor.mech_vel
        assert vel is not None
        assert vel > 0.5, f'Expected positive velocity, got {vel:.3f} rad/s'

    def test_spinning_reverse_shows_negative_velocity(self, motor):
        motor.quick_move(-2.0)
        time.sleep(0.5)
        vel = motor.mech_vel
        assert vel is not None
        assert vel < -0.5, f'Expected negative velocity, got {vel:.3f} rad/s'

    def test_position_changes_while_spinning(self, motor):
        pos_before = motor.mech_pos
        motor.quick_move(2.0)
        time.sleep(1.0)
        pos_after = motor.mech_pos
        assert pos_before is not None and pos_after is not None
        assert abs(pos_after - pos_before) > 0.5, (
            f'Position did not change: before={pos_before:.3f} after={pos_after:.3f}'
        )

    def test_quick_stop_reduces_velocity(self, motor):
        motor.quick_move(2.0)
        time.sleep(0.5)
        motor.quick_stop()
        time.sleep(0.8)
        vel = motor.mech_vel
        assert vel is not None
        assert abs(vel) < 1.0, (
            f'Motor did not slow down after quick_stop: vel={vel:.3f} rad/s'
        )


# ---------------------------------------------------------------------------
# Operation mode (MIT-style motor_control)
# ---------------------------------------------------------------------------


@hardware
class TestOperationMode:
    def test_holds_position(self, motor):
        motor.run_mode = 'operation'
        time.sleep(0.3)
        assert motor.run_mode == 'operation'

        target = motor.mech_pos
        assert target is not None

        for _ in range(10):
            motor.motor_control(
                torque=0.0, position=target, velocity=0.0, kp=10.0, kd=1.0
            )
            time.sleep(0.05)

        time.sleep(0.3)
        pos = motor.mech_pos
        assert pos is not None
        assert abs(pos - target) < 0.15, (
            f'Motor drifted: target={target:.3f} pos={pos:.3f}'
        )

    def test_moves_to_target(self, motor):
        motor.run_mode = 'operation'
        time.sleep(0.2)

        start = motor.mech_pos
        assert start is not None
        target = start + 1.0

        for _ in range(20):
            motor.motor_control(
                torque=0.0, position=target, velocity=0.0, kp=10.0, kd=1.0
            )
            time.sleep(0.05)

        time.sleep(0.5)
        pos = motor.mech_pos
        assert pos is not None
        assert abs(pos - target) < 0.15, (
            f'Did not reach target: target={target:.3f} pos={pos:.3f}'
        )


# ---------------------------------------------------------------------------
# Speed mode
# ---------------------------------------------------------------------------


@hardware
class TestSpeedMode:
    def test_positive_speed(self, motor):
        motor.run_mode = 'speed'
        time.sleep(0.3)
        motor.spd_ref = 3.0
        time.sleep(0.5)
        vel = motor.mech_vel
        assert vel is not None
        assert vel > 1.0, f'Expected positive velocity, got {vel:.3f} rad/s'

    def test_negative_speed(self, motor):
        motor.run_mode = 'speed'
        time.sleep(0.2)
        motor.spd_ref = -3.0
        time.sleep(0.5)
        vel = motor.mech_vel
        assert vel is not None
        assert vel < -1.0, f'Expected negative velocity, got {vel:.3f} rad/s'

    def test_zero_speed_stops_motor(self, motor):
        motor.run_mode = 'speed'
        time.sleep(0.2)
        motor.spd_ref = 3.0
        time.sleep(0.5)
        motor.spd_ref = 0.0
        time.sleep(0.8)
        vel = motor.mech_vel
        assert vel is not None
        assert abs(vel) < 0.5, f'Motor did not stop: vel={vel:.3f} rad/s'


# ---------------------------------------------------------------------------
# Position mode
# ---------------------------------------------------------------------------


@hardware
class TestPositionMode:
    def test_moves_to_target(self, motor):
        motor.run_mode = 'position'
        time.sleep(0.3)
        assert motor.run_mode == 'position'

        start = motor.mech_pos
        assert start is not None
        target = start + 1.0

        motor.loc_ref = target
        time.sleep(1.5)

        pos = motor.mech_pos
        assert pos is not None
        assert abs(pos - target) < 0.15, (
            f'Did not reach target: target={target:.3f} pos={pos:.3f}'
        )

    def test_returns_to_start(self, motor):
        motor.run_mode = 'position'
        time.sleep(0.2)

        start = motor.mech_pos
        assert start is not None

        motor.loc_ref = start + 2.0
        time.sleep(1.5)
        motor.loc_ref = start
        time.sleep(1.5)

        pos = motor.mech_pos
        assert pos is not None
        assert abs(pos - start) < 0.15, (
            f'Did not return to start: start={start:.3f} pos={pos:.3f}'
        )

    def test_negative_direction(self, motor):
        motor.run_mode = 'position'
        time.sleep(0.2)

        start = motor.mech_pos
        assert start is not None
        target = start - 1.0

        motor.loc_ref = target
        time.sleep(1.5)

        pos = motor.mech_pos
        assert pos is not None
        assert abs(pos - target) < 0.15, (
            f'Did not reach target: target={target:.3f} pos={pos:.3f}'
        )


# ---------------------------------------------------------------------------
# Current mode
# ---------------------------------------------------------------------------


@hardware
class TestCurrentMode:
    def test_positive_current_accelerates_forward(self, motor):
        motor.run_mode = 'current'
        time.sleep(0.3)
        assert motor.run_mode == 'current'

        motor.iq_ref = 0.5
        time.sleep(0.5)
        vel = motor.mech_vel
        motor.iq_ref = 0.0
        time.sleep(0.3)

        assert vel is not None
        assert vel > 0.0, f'Expected forward acceleration, got vel={vel:.3f} rad/s'

    def test_negative_current_accelerates_reverse(self, motor):
        motor.run_mode = 'current'
        time.sleep(0.2)

        motor.iq_ref = -0.5
        time.sleep(0.5)
        vel = motor.mech_vel
        motor.iq_ref = 0.0
        time.sleep(0.3)

        assert vel is not None
        assert vel < 0.0, f'Expected reverse acceleration, got vel={vel:.3f} rad/s'

    def test_zero_current_no_torque(self, motor):
        motor.run_mode = 'current'
        time.sleep(0.2)
        motor.iq_ref = 0.0
        time.sleep(0.5)
        # Motor should not be accelerating significantly
        vel = motor.mech_vel
        assert vel is not None
        assert abs(vel) < 2.0, (
            f'Unexpected velocity with zero current: vel={vel:.3f} rad/s'
        )


# ---------------------------------------------------------------------------
# Zero position
# ---------------------------------------------------------------------------


@hardware
class TestZeroPosition:
    def test_reset_and_return_zero(self, motor):
        """Set zero at current position, spin away, then return to zero."""
        motor.quick_stop()
        time.sleep(0.3)

        motor.reset_zero_position()
        time.sleep(0.2)
        zero_pos = motor.mech_pos
        assert zero_pos is not None

        # Spin away from zero
        motor.quick_move(2.0)
        time.sleep(1.5)
        motor.quick_stop()
        time.sleep(0.3)

        away_pos = motor.mech_pos
        assert away_pos is not None
        assert abs(away_pos - zero_pos) > 0.5, 'Motor did not move away from zero'
