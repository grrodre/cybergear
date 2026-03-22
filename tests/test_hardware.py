"""Hardware integration tests for the CyberGear motor driver.

Requires a physical motor connected via CAN bus.

Run with:
    pytest -m hardware tests/test_hardware.py -v

Skip automatically when no CAN interface is available.
"""

import os
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
# Motor CAN ID: override via CYBERGEAR_CAN_ID env var; factory default is 0x7F.
MOTOR_CAN_ID = int(os.environ.get('CYBERGEAR_CAN_ID', '0x7F'), 16)

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
        """Motor temperature should be between 10 °C and 80 °C."""
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
        """Feedback is updated by the notifier as the motor sends frames."""
        time.sleep(0.3)
        fb = motor.feedback
        # Real feedback frames set temperature > 0 (default MotorFeedback has 0.0)
        assert fb.temperature > 0, f'No feedback received from motor: {fb}'
        # Polling thread should have populated parameter values by now
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
        """mech_pos tracks true cumulative rotation; feedback.position saturates at ±12.5 rad.

        They only agree when the motor is within the saturation range. After many
        rotations mech_pos diverges from feedback.position — this is expected.
        """
        mech_pos = motor.mech_pos
        feedback_pos = motor.feedback.position
        assert mech_pos is not None
        assert feedback_pos is not None
        assert abs(feedback_pos) <= CyberGearMotor.FEEDBACK_P_MAX + 0.1
        if abs(mech_pos) < CyberGearMotor.FEEDBACK_P_MAX:
            assert abs(mech_pos - feedback_pos) < 1.0, (
                f'mech_pos={mech_pos:.3f} feedback={feedback_pos:.3f}'
            )


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
        target_feedback = motor.feedback.position
        target_mech = motor.mech_pos
        assert target_mech is not None

        time.sleep(5)
        pos_feedback = motor.feedback.position
        pos_mech = motor.mech_pos
        print(f'[hold] pos_feedback={pos_feedback:.3f} pos_mech={pos_mech:.3f}')
        assert pos_feedback is not None
        assert abs(pos_feedback - target_feedback) < 0.15, (
            f'Motor drifted: target_feedback={target_feedback:.3f} pos_feedback={pos_feedback:.3f} '
            f'target_mech={target_mech:.3f} pos_mech={pos_mech:.3f}'
        )

    def test_moves_to_target(self, motor):
        # Reset zero to bring mech_pos near 0 and avoid the 2π wrap-around issue.
        motor.quick_stop()
        time.sleep(0.3)
        motor.reset_zero_position()
        time.sleep(0.5)  # allow motor and poller to settle after reset

        motor.run_mode = 'operation'
        motor.enable()
        time.sleep(0.3)  # wait for fresh feedback after enable

        start_mech = motor.mech_pos
        assert start_mech is not None

        # feedback.position is in the motor's internal frame (reset by motor_stopped
        # inside enable()). Use it as the baseline for motor_control encoding.
        start_feedback = motor.feedback.position
        target = start_feedback + 0.5  # small delta, well within ±12.5 rad range

        # Send frames for 2 s to let the motor settle.  MIT mode has a ~25 ms
        # watchdog; a long wait with no frames causes the motor to go passive.
        for _ in range(40):
            motor.motor_control(
                torque=0.0, position=target, velocity=0.0, kp=5.0, kd=2.0
            )
            time.sleep(0.05)

        # mech_pos is updated by the polling thread (every 100 ms, float32).
        # Wait one poll cycle so the final position is reflected.
        time.sleep(0.15)
        end_mech = motor.mech_pos
        assert end_mech is not None
        moved = end_mech - start_mech
        assert abs(moved - 0.5) < 0.15, (
            f'Did not move 0.5 rad: start={start_mech:.3f} end={end_mech:.3f} moved={moved:.3f} '
            f'start_feedback={start_feedback:.3f} target={target:.3f}'
        )


# ---------------------------------------------------------------------------
# Speed mode
# ---------------------------------------------------------------------------


@hardware
class TestSpeedMode:
    def test_positive_speed(self, motor):
        motor.run_mode = 'speed'
        motor.enable()
        time.sleep(0.3)
        motor.spd_ref = 3.0
        time.sleep(0.5)
        vel = motor.mech_vel
        assert vel is not None
        assert vel > 1.0, f'Expected positive velocity, got {vel:.3f} rad/s'

    def test_negative_speed(self, motor):
        motor.run_mode = 'speed'
        motor.enable()
        motor.spd_ref = 0.0
        time.sleep(0.3)
        motor.spd_ref = -3.0
        time.sleep(0.5)
        vel = motor.mech_vel
        assert vel is not None
        assert vel < -1.0, f'Expected negative velocity, got {vel:.3f} rad/s'

    def test_zero_speed_stops_motor(self, motor):
        motor.run_mode = 'speed'
        motor.enable()
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
        motor.enable()
        # Set limit_spd and wait for the polling thread to read the updated value
        # back from the motor into the _parameters cache. loc_ref sends the cached
        # limit_spd value before the position command — if the cache still holds 0
        # (factory default) the motor will not move.
        motor.limit_spd = 10.0
        time.sleep(0.5)
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
        motor.enable()
        motor.limit_spd = 10.0
        time.sleep(0.5)  # wait for poller to cache limit_spd

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
        motor.enable()
        motor.limit_spd = 10.0
        time.sleep(0.5)  # wait for poller to cache limit_spd

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
        motor.enable()
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
        motor.enable()
        motor.iq_ref = 0.0
        time.sleep(0.5)  # let residual velocity from previous test settle

        motor.iq_ref = -0.5
        time.sleep(0.5)
        vel = motor.mech_vel
        motor.iq_ref = 0.0
        time.sleep(0.3)

        assert vel is not None
        assert vel < 0.0, f'Expected reverse acceleration, got vel={vel:.3f} rad/s'

    def test_zero_current_no_torque(self, motor):
        motor.run_mode = 'current'
        motor.enable()
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
        time.sleep(2)
        motor.quick_stop()
        time.sleep(0.3)

        away_pos = motor.mech_pos
        assert away_pos is not None
        assert abs(away_pos - zero_pos) > 0.5, 'Motor did not move away from zero'

        # Return to zero and verify
        motor.return_zero_position()

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            pos = motor.mech_pos
            if pos is not None and abs(pos) < 0.1:
                break
            time.sleep(0.1)
        else:
            pytest.fail(
                f'Motor did not return to zero within 30s: pos={motor.mech_pos}'
            )

        motor.quick_stop()
