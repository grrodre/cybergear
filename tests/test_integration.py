"""Integration tests using python-can's virtual bus.

Each test creates the motor on a virtual channel, calls a command, then reads
the frame off a sniffer bus and compares it to the expected bytes.

Run with: pytest tests/test_integration.py
"""

import struct
import threading
import time
from unittest.mock import MagicMock, patch

import can
import pytest

from cybergear.can_definitions import ParameterIndex, RunMode
from cybergear.cybergear import CyberGearMotor
from cybergear.exceptions import CybergearMotorInitException

MOTOR_ID = 0x01
HOST_ID = 0xFD
CHANNEL = 'cybergear_test'
BUS_CFG = {'interface': 'virtual', 'channel': CHANNEL}


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def motor():
    with patch.object(CyberGearMotor, '_init_motor'):
        m = CyberGearMotor(
            bus_config=BUS_CFG, can_id=MOTOR_ID, host_can_id=HOST_ID, poll_interval=None
        )
    yield m
    m.close()


@pytest.fixture
def sniffer():
    bus = can.interface.Bus(**BUS_CFG)  # ty: ignore[invalid-argument-type]
    yield bus
    bus.shutdown()


def recv(sniffer) -> can.Message:
    msg = sniffer.recv(timeout=0.5)
    assert msg is not None, 'No CAN frame received within timeout'
    return msg


# ---------------------------------------------------------------------------
# Enable / disable
# ---------------------------------------------------------------------------


def test_enable(motor, sniffer):
    motor.enable()

    # enable() sends motor_stopped first, then motor_enable
    msg1 = recv(sniffer)
    assert msg1.arbitration_id == 0x0400FD01
    assert bytes(msg1.data) == b'\x00' * 8

    msg2 = recv(sniffer)
    assert msg2.arbitration_id == 0x0300FD01
    assert bytes(msg2.data) == b'\x00' * 8


def test_disable(motor, sniffer):
    motor.disable()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x0400FD01
    assert bytes(msg.data) == b'\x00' * 8


def test_emergency_brake(motor, sniffer):
    motor.emergency_brake()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1400FD01
    assert bytes(msg.data) == b'\x00' * 8


def test_reset_zero_position(motor, sniffer):
    motor.reset_zero_position()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x0600FD01
    assert bytes(msg.data) == bytes([1, 0, 0, 0, 0, 0, 0, 0])


# ---------------------------------------------------------------------------
# MIT motor control
# ---------------------------------------------------------------------------


def test_motor_control(motor, sniffer):
    torque, position, velocity, kp, kd = 1.5, 2.0, 3.0, 10.0, 0.5

    motor.motor_control(
        torque=torque, position=position, velocity=velocity, kp=kp, kd=kd
    )

    msg = recv(sniffer)

    # Torque is encoded in arb_id bits[23:8]
    torque_raw = CyberGearMotor._float_to_uint(
        torque, CyberGearMotor.T_MIN, CyberGearMotor.T_MAX, 16
    )
    assert msg.arbitration_id == 0x01000000 | (torque_raw << 8) | MOTOR_ID

    # Data: position | velocity | kp | kd — all big-endian uint16
    expected = struct.pack(
        '>4H',
        CyberGearMotor._float_to_uint(
            position, CyberGearMotor.P_MIN, CyberGearMotor.P_MAX, 16
        ),
        CyberGearMotor._float_to_uint(
            velocity, CyberGearMotor.V_MIN, CyberGearMotor.V_MAX, 16
        ),
        CyberGearMotor._float_to_uint(
            kp, CyberGearMotor.KP_MIN, CyberGearMotor.KP_MAX, 16
        ),
        CyberGearMotor._float_to_uint(
            kd, CyberGearMotor.KD_MIN, CyberGearMotor.KD_MAX, 16
        ),
    )
    assert bytes(msg.data) == expected


# ---------------------------------------------------------------------------
# Run mode
# ---------------------------------------------------------------------------


def test_run_mode_speed(motor, sniffer):
    motor.run_mode = 'speed'

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    assert bytes(msg.data) == struct.pack(
        '2H4B', ParameterIndex.run_mode, 0, RunMode.speed, 0, 0, 0
    )


def test_run_mode_position(motor, sniffer):
    motor.run_mode = 'position'

    msg = recv(sniffer)
    assert bytes(msg.data) == struct.pack(
        '2H4B', ParameterIndex.run_mode, 0, RunMode.position, 0, 0, 0
    )


def test_run_mode_current(motor, sniffer):
    motor.run_mode = 'current'

    msg = recv(sniffer)
    assert bytes(msg.data) == struct.pack(
        '2H4B', ParameterIndex.run_mode, 0, RunMode.current, 0, 0, 0
    )


# ---------------------------------------------------------------------------
# Setpoint writes
# ---------------------------------------------------------------------------


def test_spd_ref(motor, sniffer):
    motor.spd_ref = 5.0

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.spd_ref, 0, 5.0)


def test_iq_ref(motor, sniffer):
    motor.iq_ref = 1.5

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.iq_ref, 0, 1.5)


def test_loc_ref_no_cached_limit_spd(motor, sniffer):
    motor._parameters.clear()
    motor.loc_ref = 1.0

    msg = recv(sniffer)
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.loc_ref, 0, 1.0)
    # No second frame
    assert sniffer.recv(timeout=0.05) is None


def test_loc_ref_with_cached_limit_spd(motor, sniffer):
    motor._parameters[ParameterIndex.limit_spd.name] = 3.0
    motor.loc_ref = 2.0

    msg1 = recv(sniffer)
    msg2 = recv(sniffer)
    assert bytes(msg1.data) == struct.pack('2Hf', ParameterIndex.limit_spd, 0, 3.0)
    assert bytes(msg2.data) == struct.pack('2Hf', ParameterIndex.loc_ref, 0, 2.0)


def test_limit_spd(motor, sniffer):
    motor.limit_spd = 10.0

    msg = recv(sniffer)
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.limit_spd, 0, 10.0)


def test_limit_cur(motor, sniffer):
    motor.limit_cur = 5.0

    msg = recv(sniffer)
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.limit_cur, 0, 5.0)


def test_limit_torque(motor, sniffer):
    motor.limit_torque = 6.0

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.limit_torque, 0, 6.0)


def test_cur_filt_gain(motor, sniffer):
    motor.cur_filt_gain = 0.5

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    assert bytes(msg.data) == struct.pack('2Hf', ParameterIndex.cur_filt_gain, 0, 0.5)


# ---------------------------------------------------------------------------
# quick_move / quick_stop
# ---------------------------------------------------------------------------


def test_quick_move_positive(motor, sniffer):
    motor.quick_move(5.0)

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    expected = (
        struct.pack('2H', ParameterIndex.run_mode, 0)
        + struct.pack('2B', RunMode.quick_move, 1)
        + struct.pack(
            '>H',
            CyberGearMotor._float_to_uint(
                5.0, CyberGearMotor.V_MIN, CyberGearMotor.V_MAX, 16
            ),
        )
    )
    assert bytes(msg.data) == expected


def test_quick_move_negative(motor, sniffer):
    motor.quick_move(-5.0)

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    expected = (
        struct.pack('2H', ParameterIndex.run_mode, 0)
        + struct.pack('2B', RunMode.quick_move, 1)
        + struct.pack(
            '>H',
            CyberGearMotor._float_to_uint(
                -5.0, CyberGearMotor.V_MIN, CyberGearMotor.V_MAX, 16
            ),
        )
    )
    assert bytes(msg.data) == expected


def test_quick_stop(motor, sniffer):
    motor.quick_stop()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1200FD01
    expected = (
        struct.pack('2H', ParameterIndex.run_mode, 0)
        + struct.pack('2B', RunMode.quick_move, 0)
        + struct.pack('>H', 0x7FFF)
    )
    assert bytes(msg.data) == expected
    assert sniffer.recv(timeout=0.05) is None  # single frame only


# ---------------------------------------------------------------------------
# return_zero_position
# ---------------------------------------------------------------------------


def test_return_zero_position(motor, sniffer):
    motor.return_zero_position()

    # Frame 1: parameter_writing to set run_mode=zero_position
    msg1 = recv(sniffer)
    assert msg1.arbitration_id == 0x1200FD01
    expected = (
        struct.pack('2H', ParameterIndex.run_mode, 0)
        + struct.pack('2B', RunMode.zero_position, 0)
        + struct.pack('>H', 0x0000)
    )
    assert bytes(msg1.data) == expected

    # Frame 2+3: enable() sends motor_stopped then motor_enable
    msg2 = recv(sniffer)
    assert msg2.arbitration_id == 0x0400FD01
    assert bytes(msg2.data) == b'\x00' * 8

    msg3 = recv(sniffer)
    assert msg3.arbitration_id == 0x0300FD01
    assert bytes(msg3.data) == b'\x00' * 8


# ---------------------------------------------------------------------------
# set_can_id
# ---------------------------------------------------------------------------


def test_set_can_id(motor, sniffer):
    device_id = b'\x01\x02\x03\x04\x05\x06\x07\x08'
    motor._device_id = device_id
    new_id = 0x05

    motor.set_can_id(new_id)

    msg = recv(sniffer)
    data2 = (new_id << 8) | HOST_ID
    assert msg.arbitration_id == 0x07000000 | (data2 << 8) | MOTOR_ID
    assert bytes(msg.data) == device_id


# ---------------------------------------------------------------------------
# fetch_parameter_table
# ---------------------------------------------------------------------------


def test_fetch_parameter_table(motor, sniffer):
    device_id = b'\x01\x02\x03\x04\x05\x06\x07\x08'
    motor._device_id = device_id

    motor.fetch_parameter_table()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x1300FD01
    assert bytes(msg.data) == device_id


# ---------------------------------------------------------------------------
# Encoder calibration
# ---------------------------------------------------------------------------


def test_encoder_calibration_trigger(motor, sniffer):
    threading.Thread(
        target=lambda: motor.encoder_calibration(timeout=0.2), daemon=True
    ).start()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x0500FD01
    assert bytes(msg.data) == b'\x00' * 8


def test_encoder_calibration_response_fires_callback(motor):
    expected_offset = -4.4347
    result: list[float] = []
    done = threading.Event()

    def run():
        value = motor.encoder_calibration(timeout=2.0)
        if value is not None:
            result.append(value)
        done.set()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    time.sleep(0.1)  # let the thread register the callback

    fake_msg = can.Message(
        arbitration_id=0x0500FD01,
        data=struct.pack('<f', expected_offset) + b'\x00\x00\x00\x00',
        is_extended_id=True,
    )
    motor._parse_encoder_calibration_message(fake_msg)

    assert done.wait(timeout=2.0), 'encoder_calibration did not return'
    assert len(result) == 1
    assert abs(result[0] - expected_offset) < 1e-4


# ---------------------------------------------------------------------------
# Bus scanning — edge cases
# ---------------------------------------------------------------------------


def test_scan_no_motors_found():
    """__init__ raises CybergearMotorInitException when scan finds no motors."""
    with patch.object(CyberGearMotor, 'scan', return_value=[]):
        with pytest.raises(CybergearMotorInitException, match='No CyberGear motors'):
            CyberGearMotor(bus_config=BUS_CFG, can_id=None)


def test_scan_multiple_motors_raises():
    """__init__ raises CybergearMotorInitException when scan finds more than one motor."""
    two_motors = [(0x01, b'\x00' * 8), (0x02, b'\x00' * 8)]
    with patch.object(CyberGearMotor, 'scan', return_value=two_motors):
        with pytest.raises(CybergearMotorInitException, match='Multiple'):
            CyberGearMotor(bus_config=BUS_CFG, can_id=None)


def test_scan_returns_single_motor():
    """scan() returns [(can_id, device_id)] when exactly one motor responds."""
    fake_device_id = b'\x01\x02\x03\x04\x05\x06\x07\x08'
    # device_id response frame: comm_type=0x00000000, motor_id in bits[15:8]
    fake_msg = can.Message(
        arbitration_id=(MOTOR_ID << 8),
        data=fake_device_id,
        is_extended_id=True,
    )

    def fake_notifier(bus, listeners, **kwargs):
        for fn in listeners:
            fn(fake_msg)
        return MagicMock()

    with (
        patch('can.interface.Bus', return_value=MagicMock()),
        patch('can.Notifier', side_effect=fake_notifier),
    ):
        result = CyberGearMotor.scan(
            bus_config=BUS_CFG, candidates=[MOTOR_ID], timeout=0.0
        )

    assert result == [(MOTOR_ID, fake_device_id)]


def test_scan_timeout():
    """scan() returns [] promptly when no motors respond within the timeout."""
    with (
        patch('can.interface.Bus', return_value=MagicMock()),
        patch('can.Notifier', return_value=MagicMock()),
    ):
        result = CyberGearMotor.scan(
            bus_config=BUS_CFG, candidates=[MOTOR_ID], timeout=0.0
        )

    assert result == []
