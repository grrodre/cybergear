"""Unit tests for the CyberGear motor driver.

Run with: pytest tests/test_unit.py
"""

import importlib
import io
import struct
import sys
import time
import unittest
import unittest.mock
from unittest.mock import MagicMock, patch

import can

from cybergear.can_definitions import (
    CommunicationTypeCan,
    ParameterIndex,
    ParameterTable,
    RunMode,
)
from cybergear.cybergear import CyberGearMotor, FaultState, MotorFeedback
from cybergear.exceptions import CybergearMotorInitException

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_motor(poll_interval: float | None = None) -> tuple[CyberGearMotor, MagicMock]:
    """Return a motor instance with a mocked CAN bus. Init is bypassed."""
    mock_bus = MagicMock(spec=can.BusABC)
    mock_bus.set_filters = MagicMock()
    mock_bus.send = MagicMock()

    with (
        patch('cybergear.cybergear.can.interface.Bus', return_value=mock_bus),
        patch('cybergear.cybergear.can.Notifier'),
        patch.object(CyberGearMotor, '_init_motor', lambda self: None),
    ):
        motor = CyberGearMotor(
            bus_config={'interface': 'socketcan', 'channel': 'can0'},
            can_id=0x7F,
            poll_interval=poll_interval,
        )

    motor._device_id = b'\x01\x02\x03\x04\x05\x06\x07\x08'
    motor._bus = mock_bus
    return motor, mock_bus


def _feedback_message(
    pos_raw: int = 0x7FFF,
    vel_raw: int = 0x7FFF,
    torq_raw: int = 0x7FFF,
    temp_raw: int = 333,  # 33.3 °C × 10
    fault_bits: int = 0,  # bits 21–16 of arbitration_id
    mode_bits: int = 2,  # bits 23–22 (2 = Run)
    motor_can_id: int = 0x7F,
    host_can_id: int = 0xFD,
) -> can.Message:
    """Build a comm-type-2 feedback CAN message."""
    arb = (
        int(CommunicationTypeCan.motor_feedback)
        | (mode_bits << 22)
        | (fault_bits << 16)
        | (motor_can_id << 8)
        | host_can_id
    )
    data = (
        struct.pack('>H', pos_raw)
        + struct.pack('>H', vel_raw)
        + struct.pack('>H', torq_raw)
        + struct.pack('>H', temp_raw)
    )
    return can.Message(arbitration_id=arb, data=data, is_extended_id=True)


def _parameter_message(
    index: ParameterIndex,
    value: float | int,
    motor_can_id: int = 0x7F,
    host_can_id: int = 0xFD,
) -> can.Message:
    """Build a comm-type-17 parameter read response CAN message."""
    arb = (
        int(CommunicationTypeCan.parameter_reading) | (motor_can_id << 8) | host_can_id
    )
    if index == ParameterIndex.run_mode:
        payload = struct.pack('B', int(value))
    elif index == ParameterIndex.rotation:
        payload = struct.pack('h', int(value))
    else:
        payload = struct.pack('f', float(value))

    data = struct.pack('2H', index.value, 0) + payload.ljust(4, b'\x00')
    return can.Message(arbitration_id=arb, data=data, is_extended_id=True)


# ---------------------------------------------------------------------------
# Conversion helpers
# ---------------------------------------------------------------------------


class TestConversionHelpers(unittest.TestCase):
    def test_float_to_uint_midpoint(self):
        result = CyberGearMotor._float_to_uint(0.0, -12.5, 12.5, 16)
        self.assertEqual(result, 32767)  # midpoint of 0–65535

    def test_float_to_uint_min(self):
        result = CyberGearMotor._float_to_uint(-12.5, -12.5, 12.5, 16)
        self.assertEqual(result, 0)

    def test_float_to_uint_max(self):
        result = CyberGearMotor._float_to_uint(12.5, -12.5, 12.5, 16)
        self.assertEqual(result, 65535)

    def test_float_to_uint_clamps_below_min(self):
        result = CyberGearMotor._float_to_uint(-999.0, -12.5, 12.5, 16)
        self.assertEqual(result, 0)

    def test_float_to_uint_clamps_above_max(self):
        result = CyberGearMotor._float_to_uint(999.0, -12.5, 12.5, 16)
        self.assertEqual(result, 65535)

    def test_uint_to_float_min(self):
        result = CyberGearMotor._uint_to_float(0, -12.5, 12.5, 16)
        self.assertAlmostEqual(result, -12.5, places=3)

    def test_uint_to_float_max(self):
        result = CyberGearMotor._uint_to_float(65535, -12.5, 12.5, 16)
        self.assertAlmostEqual(result, 12.5, places=3)

    def test_round_trip(self):
        for value in (-10.0, -5.0, 0.0, 5.0, 10.0):
            packed = CyberGearMotor._float_to_uint(value, -12.5, 12.5, 16)
            recovered = CyberGearMotor._uint_to_float(packed, -12.5, 12.5, 16)
            self.assertAlmostEqual(value, recovered, places=2)


# ---------------------------------------------------------------------------
# Feedback message parsing
# ---------------------------------------------------------------------------


class TestFeedbackParsing(unittest.TestCase):
    def setUp(self):
        self.motor, _ = _make_motor()

    def test_temperature_divided_by_ten(self):
        """Raw temp value is Celsius × 10; driver must divide by 10."""
        msg = _feedback_message(temp_raw=333)  # 33.3 °C
        self.motor._parse_motor_feedback_message(msg)
        self.assertAlmostEqual(self.motor.feedback.temperature, 33.3, places=1)

    def test_temperature_not_multiplied(self):
        """Guard against the old × 10 bug returning."""
        msg = _feedback_message(temp_raw=333)
        self.motor._parse_motor_feedback_message(msg)
        self.assertLess(self.motor.feedback.temperature, 100.0)

    def test_position_midpoint_is_zero(self):
        """0x7FFF (midpoint of uint16) should map to ≈ 0 rad."""
        msg = _feedback_message(pos_raw=0x7FFF)
        self.motor._parse_motor_feedback_message(msg)
        self.assertAlmostEqual(self.motor.feedback.position, 0.0, places=1)

    def test_position_min(self):
        msg = _feedback_message(pos_raw=0)
        self.motor._parse_motor_feedback_message(msg)
        self.assertAlmostEqual(
            self.motor.feedback.position, CyberGearMotor.FEEDBACK_P_MIN, places=2
        )

    def test_position_max(self):
        msg = _feedback_message(pos_raw=65535)
        self.motor._parse_motor_feedback_message(msg)
        self.assertAlmostEqual(
            self.motor.feedback.position, CyberGearMotor.FEEDBACK_P_MAX, places=2
        )

    def test_feedback_position_range_matches_control_range(self):
        """Feedback and control position ranges must be equal (both ±4π ≈ ±12.5).

        Hardware verified: feedback pos saturates at exactly ±12.5 rad while
        mech_pos (parameter read) continues tracking the true position.
        """
        self.assertEqual(CyberGearMotor.FEEDBACK_P_MIN, CyberGearMotor.P_MIN)
        self.assertEqual(CyberGearMotor.FEEDBACK_P_MAX, CyberGearMotor.P_MAX)

    def test_mode_bits_decoded(self):
        msg = _feedback_message(mode_bits=2)  # Run mode
        self.motor._parse_motor_feedback_message(msg)
        self.assertEqual(self.motor.feedback.mode, 2)

    def test_no_fault_when_all_bits_clear(self):
        msg = _feedback_message(fault_bits=0)
        self.motor._parse_motor_feedback_message(msg)
        self.assertFalse(self.motor.feedback.faults.has_fault)

    def test_over_temperature_fault_bit(self):
        # Bit 18 of arbitration_id = over_temperature
        msg = _feedback_message(
            fault_bits=0b001000
        )  # bit 3 relative to bit 16 = bit 19
        self.motor._parse_motor_feedback_message(msg)
        # fault_bits shifts to bits 21-16; bit 18 is fault_bits bit 2
        msg2 = _feedback_message(fault_bits=0b000100)  # bit 18 = over_temperature
        self.motor._parse_motor_feedback_message(msg2)
        self.assertTrue(self.motor.feedback.faults.over_temperature)

    def test_undervoltage_fault_bit(self):
        # Bit 16 = undervoltage → fault_bits bit 0
        msg = _feedback_message(fault_bits=0b000001)
        # calibrated bit is bit 21 (fault_bits bit 5); bit 16 is undervoltage
        # fault_bits=1 means bit 16 is set = undervoltage
        self.motor._parse_motor_feedback_message(msg)
        self.assertTrue(self.motor.feedback.faults.undervoltage)

    def test_feedback_listener_called(self):
        listener = MagicMock()
        self.motor.add_feedback_listener(listener)
        msg = _feedback_message()
        self.motor._parse_motor_feedback_message(msg)
        listener.assert_called_once()
        self.assertIsInstance(listener.call_args[0][0], MotorFeedback)

    def test_feedback_listener_receives_snapshot(self):
        received = []
        self.motor.add_feedback_listener(lambda fb: received.append(fb))
        msg = _feedback_message(temp_raw=250)
        self.motor._parse_motor_feedback_message(msg)
        self.assertEqual(len(received), 1)
        self.assertAlmostEqual(received[0].temperature, 25.0, places=1)

    def test_multiple_feedback_listeners_all_called(self):
        a, b = MagicMock(), MagicMock()
        self.motor.add_feedback_listener(a)
        self.motor.add_feedback_listener(b)
        self.motor._parse_motor_feedback_message(_feedback_message())
        a.assert_called_once()
        b.assert_called_once()

    def test_remove_feedback_listener(self):
        listener = MagicMock()
        self.motor.add_feedback_listener(listener)
        self.motor.remove_feedback_listener(listener)
        self.motor._parse_motor_feedback_message(_feedback_message())
        listener.assert_not_called()

    def test_listener_exception_does_not_crash_driver(self):
        def bad_listener(fb):
            raise RuntimeError('boom')

        self.motor.add_feedback_listener(bad_listener)
        # Should not raise
        self.motor._parse_motor_feedback_message(_feedback_message())


# ---------------------------------------------------------------------------
# Fault listener
# ---------------------------------------------------------------------------


class TestFaultListener(unittest.TestCase):
    def setUp(self):
        self.motor, _ = _make_motor()

    def test_fault_listener_fires_on_fault_appearing(self):
        listener = MagicMock()
        self.motor.add_fault_listener(listener)

        # First message: no fault
        self.motor._parse_motor_feedback_message(_feedback_message(fault_bits=0))
        listener.assert_not_called()

        # Second message: undervoltage fault appears
        self.motor._parse_motor_feedback_message(_feedback_message(fault_bits=0b000001))
        listener.assert_called_once()

    def test_fault_listener_fires_on_fault_clearing(self):
        listener = MagicMock()

        # Establish faulted state without listener
        self.motor._parse_motor_feedback_message(_feedback_message(fault_bits=0b000001))

        self.motor.add_fault_listener(listener)

        # Fault clears
        self.motor._parse_motor_feedback_message(_feedback_message(fault_bits=0))
        listener.assert_called_once()
        self.assertFalse(listener.call_args[0][0].has_fault)

    def test_fault_listener_does_not_fire_when_state_unchanged(self):
        listener = MagicMock()
        self.motor.add_fault_listener(listener)

        self.motor._parse_motor_feedback_message(_feedback_message(fault_bits=0))
        self.motor._parse_motor_feedback_message(_feedback_message(fault_bits=0))
        listener.assert_not_called()


# ---------------------------------------------------------------------------
# Parameter reading
# ---------------------------------------------------------------------------


class TestParameterReading(unittest.TestCase):
    def setUp(self):
        self.motor, _ = _make_motor()

    def test_float_parameter_stored(self):
        msg = _parameter_message(ParameterIndex.spd_ref, 5.0)
        self.motor._process_parameter_reading(msg)
        self.assertAlmostEqual(self.motor.spd_ref, 5.0, places=4)  # ty: ignore[no-matching-overload]

    def test_run_mode_stored_as_int(self):
        msg = _parameter_message(ParameterIndex.run_mode, RunMode.speed.value)
        self.motor._process_parameter_reading(msg)
        self.assertEqual(self.motor.run_mode, 'speed')

    def test_rotation_stored_as_int(self):
        msg = _parameter_message(ParameterIndex.rotation, 3)
        self.motor._process_parameter_reading(msg)
        self.assertEqual(self.motor.rotation, 3)

    def test_parameter_listener_called(self):
        listener = MagicMock()
        self.motor.add_parameter_listener(listener)
        msg = _parameter_message(ParameterIndex.v_bus, 24.0)
        self.motor._process_parameter_reading(msg)
        listener.assert_called_once_with('v_bus', unittest.mock.ANY)

    def test_unknown_parameter_index_ignored(self):
        arb = int(CommunicationTypeCan.parameter_reading) | (0x7F << 8) | 0xFD
        data = struct.pack('2H', 0xDEAD, 0) + struct.pack('f', 1.0)
        msg = can.Message(arbitration_id=arb, data=data, is_extended_id=True)
        # Should not raise
        self.motor._process_parameter_reading(msg)


# ---------------------------------------------------------------------------
# Parameter table parsing
# ---------------------------------------------------------------------------


class TestParameterTableParsing(unittest.TestCase):
    def setUp(self):
        self.motor, _ = _make_motor()

    def test_parse_parameter_table_populates_parameters(self):
        """Injecting a fragment-4 (numeric value) frame populates _parameters_table."""
        # gear_ratio: ParameterTable index 0x200F, type 'f'
        expected = 6.0
        arb = (
            int(CommunicationTypeCan.parameter_table)
            | (4 << 16)  # fragment index 4 = numeric value
            | (0x7F << 8)
            | 0xFD
        )
        # data[0:2] = raw index, data[2:4] = padding, data[4:8] = float value
        data = (
            struct.pack('H', ParameterTable.gear_ratio.value)
            + b'\x00\x00'
            + struct.pack('f', expected)
        )
        msg = can.Message(arbitration_id=arb, data=data, is_extended_id=True)

        self.motor._parse_parameter_table_message(msg)

        self.assertAlmostEqual(
            self.motor._parameters_table['gear_ratio'], expected, places=4
        )

    def test_parse_parameter_table_unknown_index_ignored(self):
        """An unknown parameter index in a table frame is silently ignored."""
        arb = int(CommunicationTypeCan.parameter_table) | (4 << 16) | (0x7F << 8) | 0xFD
        data = struct.pack('H', 0xBEEF) + b'\x00\x00' + struct.pack('f', 1.0)
        msg = can.Message(arbitration_id=arb, data=data, is_extended_id=True)

        self.motor._parse_parameter_table_message(msg)  # must not raise
        self.assertEqual(len(self.motor._parameters_table), 0)


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------


class TestValidation(unittest.TestCase):
    def setUp(self):
        self.motor, _ = _make_motor()

    def test_limit_torque_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            self.motor.limit_torque = 13.0

    def test_limit_torque_rejects_negative(self):
        with self.assertRaises(ValueError):
            self.motor.limit_torque = -1.0

    def test_limit_spd_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            self.motor.limit_spd = 31.0

    def test_limit_cur_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            self.motor.limit_cur = 24.0

    def test_cur_filt_gain_rejects_out_of_range(self):
        with self.assertRaises(ValueError):
            self.motor.cur_filt_gain = 1.1

    def test_watchdog_timeout_rejects_negative(self):
        with self.assertRaises(ValueError):
            self.motor.set_watchdog_timeout(-1)

    def test_watchdog_timeout_rejects_over_max(self):
        with self.assertRaises(ValueError):
            self.motor.set_watchdog_timeout(100_001)

    def test_watchdog_timeout_zero_is_valid(self):
        # Should not raise
        self.motor.set_watchdog_timeout(0)

    def test_watchdog_timeout_sends_can_message(self):
        # The frame is sent but hardware testing shows the motor firmware ignores it.
        # ParameterTable writes (0x200x indices) via comm_type 0x12 are not supported.
        self.motor.set_watchdog_timeout(500)
        self.motor._bus.send.assert_called()  # ty: ignore[unresolved-attribute]

    def test_run_mode_invalid_logs_warning(self):
        """Setting an unknown run_mode string logs a warning and does not send a frame."""
        motor, mock_bus = _make_motor()
        with self.assertLogs('cybergear', level='WARNING'):
            motor.run_mode = 'invalid'
        mock_bus.send.assert_not_called()

    def test_spd_ref_in_wrong_mode_logs_warning(self):
        """Setting spd_ref while not in speed mode logs a warning."""
        motor, _ = _make_motor()
        motor._parameters[ParameterIndex.run_mode.name] = RunMode.position.value
        with self.assertLogs('cybergear', level='WARNING'):
            motor.spd_ref = 5.0

    def test_loc_ref_in_wrong_mode_logs_warning(self):
        """Setting loc_ref while not in position mode logs a warning."""
        motor, _ = _make_motor()
        motor._parameters[ParameterIndex.run_mode.name] = RunMode.speed.value
        with self.assertLogs('cybergear', level='WARNING'):
            motor.loc_ref = 1.0

    def test_iq_ref_in_wrong_mode_logs_warning(self):
        """Setting iq_ref while not in current mode logs a warning."""
        motor, _ = _make_motor()
        motor._parameters[ParameterIndex.run_mode.name] = RunMode.speed.value
        with self.assertLogs('cybergear', level='WARNING'):
            motor.iq_ref = 1.0


# ---------------------------------------------------------------------------
# Init and context manager
# ---------------------------------------------------------------------------


class TestInit(unittest.TestCase):
    def test_init_timeout_raises(self):
        mock_bus = MagicMock(spec=can.BusABC)
        mock_bus.set_filters = MagicMock()
        mock_bus.send = MagicMock()

        with (
            patch('cybergear.cybergear.can.interface.Bus', return_value=mock_bus),
            patch('cybergear.cybergear.can.Notifier'),
            self.assertRaises(CybergearMotorInitException),
        ):
            CyberGearMotor(
                bus_config={'interface': 'socketcan', 'channel': 'can0'},
                poll_interval=None,
            )

    def test_context_manager_calls_close(self):
        motor, mock_bus = _make_motor()
        motor._notifier = MagicMock()
        with motor:
            pass
        motor._notifier.stop.assert_called_once()
        mock_bus.shutdown.assert_called_once()

    def test_device_id_hex_string(self):
        motor, _ = _make_motor()
        self.assertEqual(motor.device_id, '0102030405060708')

    def test_device_id_none_before_init(self):
        motor, _ = _make_motor()
        motor._device_id = None
        self.assertIsNone(motor.device_id)

    def test_context_manager_cleanup_on_init_failure(self):
        """close() shuts down the bus and notifier even when called after a partial init."""
        motor, mock_bus = _make_motor()
        motor._notifier = MagicMock()

        motor.close()

        motor._notifier.stop.assert_called_once()
        mock_bus.shutdown.assert_called_once()


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------


class TestErrorHandling(unittest.TestCase):
    def test_send_message_can_error_does_not_raise(self):
        """A CanError on bus.send is caught and does not propagate to the caller."""
        motor, mock_bus = _make_motor()
        mock_bus.send.side_effect = can.CanError('simulated send failure')

        motor.enable()  # must not raise

    def test_send_message_can_error_state_unchanged(self):
        """Motor state is not corrupted when a CAN send fails."""
        motor, mock_bus = _make_motor()
        mock_bus.send.side_effect = can.CanError('fail')

        motor.enable()
        motor.disable()  # motor is still usable; also must not raise

    def test_parameter_listener_exception_does_not_propagate(self):
        """An exception raised inside a parameter listener does not crash the driver."""
        motor, _ = _make_motor()

        def bad_listener(name, value):
            raise RuntimeError('listener crash')

        motor.add_parameter_listener(bad_listener)
        motor._process_parameter_reading(_parameter_message(ParameterIndex.v_bus, 24.0))

    def test_remove_feedback_listener_not_registered(self):
        """Removing a listener that was never added does not raise."""
        motor, _ = _make_motor()

        def cb(fb):
            pass

        motor.remove_feedback_listener(cb)

    def test_remove_parameter_listener_not_registered(self):
        """Removing a parameter listener that was never added does not raise."""
        motor, _ = _make_motor()

        def cb(name, value):
            pass

        motor.remove_parameter_listener(cb)

    def test_remove_fault_listener_not_registered(self):
        """Removing a fault listener that was never added does not raise."""
        motor, _ = _make_motor()

        def cb(faults):
            pass

        motor.remove_fault_listener(cb)


# ---------------------------------------------------------------------------
# Polling thread
# ---------------------------------------------------------------------------


class TestPollingThread(unittest.TestCase):
    def test_polling_thread_survives_failed_poll(self):
        """Polling thread keeps running if a poll raises an exception."""
        motor, mock_bus = _make_motor()
        mock_bus.send.side_effect = can.CanError('poll fail')
        motor.start_polling(0.05)
        time.sleep(0.2)  # allow several poll cycles with failures
        self.assertIsNotNone(motor._poller)
        self.assertTrue(motor._poller.is_alive())
        motor.stop_polling()

    def test_stop_polling_thread_exits(self):
        """stop_polling() causes the polling thread to exit cleanly."""
        motor, _ = _make_motor()
        motor.start_polling(0.05)
        self.assertTrue(motor._poller.is_alive())
        motor.stop_polling()
        self.assertIsNone(motor._poller)


# ---------------------------------------------------------------------------
# FaultState
# ---------------------------------------------------------------------------


class TestFaultState(unittest.TestCase):
    def test_no_fault_by_default(self):
        self.assertFalse(FaultState().has_fault)

    def test_uncalibrated_is_fault(self):
        self.assertTrue(FaultState(calibrated=False).has_fault)

    def test_over_temperature_is_fault(self):
        self.assertTrue(FaultState(over_temperature=True).has_fault)

    def test_all_clear_is_no_fault(self):
        fs = FaultState(
            calibrated=True,
            hall_encoding_failure=False,
            magnetic_encoding_failure=False,
            over_temperature=False,
            over_current=False,
            undervoltage=False,
        )
        self.assertFalse(fs.has_fault)

    def test_frozen(self):
        fs = FaultState()
        with self.assertRaises(Exception):
            fs.over_current = True  # type: ignore[misc]


# ---------------------------------------------------------------------------
# MotorFeedback
# ---------------------------------------------------------------------------


class TestMotorFeedback(unittest.TestCase):
    def test_frozen(self):
        fb = MotorFeedback()
        with self.assertRaises(Exception):
            fb.position = 1.0  # type: ignore[misc]

    def test_default_values(self):
        fb = MotorFeedback()
        self.assertEqual(fb.position, 0.0)
        self.assertEqual(fb.velocity, 0.0)
        self.assertEqual(fb.torque, 0.0)
        self.assertEqual(fb.temperature, 0.0)
        self.assertEqual(fb.mode, 0)
        self.assertFalse(fb.faults.has_fault)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


class TestCLI(unittest.TestCase):
    def test_cli_scan_prints_motor(self):
        """cybergear-scan prints a table row when a motor is found."""
        from cybergear.cli import scan

        with (
            patch.object(CyberGearMotor, 'scan', return_value=[(1, b'\x01' * 8)]),
            patch('sys.argv', ['cybergear-scan']),
            patch('sys.stdout', new_callable=io.StringIO) as fake_out,
        ):
            scan.main()

        output = fake_out.getvalue()
        self.assertIn('1', output)
        self.assertIn('01' * 8, output)

    def test_cli_scan_no_motors(self):
        """cybergear-scan exits with code 0 and prints 'No motors found' when none respond."""
        from cybergear.cli import scan

        with (
            patch.object(CyberGearMotor, 'scan', return_value=[]),
            patch('sys.argv', ['cybergear-scan']),
            patch('sys.stdout', new_callable=io.StringIO) as fake_out,
        ):
            with self.assertRaises(SystemExit) as ctx:
                scan.main()

        self.assertEqual(ctx.exception.code, 0)
        self.assertIn('No motors found', fake_out.getvalue())

    def test_dashboard_missing_dependency_exits(self):
        """Importing dashboard without textual installed calls sys.exit(1)."""
        # Remove cached module and textual so the import runs fresh.
        to_remove = [
            k
            for k in sys.modules
            if k == 'cybergear.cli.dashboard' or k.startswith('textual')
        ]
        backup = {k: sys.modules.pop(k) for k in to_remove}

        try:
            with (
                patch.dict(
                    'sys.modules',
                    {
                        'textual': None,
                        'textual.app': None,
                        'textual.widgets': None,
                        'textual.containers': None,
                    },
                ),
                self.assertRaises(SystemExit) as ctx,
            ):
                importlib.import_module('cybergear.cli.dashboard')

            self.assertEqual(ctx.exception.code, 1)
        finally:
            for k in [k for k in sys.modules if k == 'cybergear.cli.dashboard']:
                sys.modules.pop(k, None)
            sys.modules.update(backup)


if __name__ == '__main__':
    unittest.main()
