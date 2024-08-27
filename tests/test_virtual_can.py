"""Integration tests using python-can's virtual bus.

Each test creates the motor on a virtual channel, calls a command, then reads
the frame off a sniffer bus and compares it to the expected bytes.

Run with: pytest tests/test_virtual_can.py
"""

import struct
from unittest.mock import patch

import can
import pytest

from cybergear.can_definitions import ParameterIndex, RunMode
from cybergear.cybergear import CyberGearMotor

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
    assert bytes(msg.data) == bytes([1, 1, 0, 0, 0, 0, 0, 0])


# ---------------------------------------------------------------------------
# MIT motor control
# ---------------------------------------------------------------------------


def test_motor_control(motor, sniffer):
    torque, position, velocity, kp, kd = 1.5, 2.0, 3.0, 10.0, 0.5

    motor.motor_control(
        torque=torque, position=position, velocity=velocity, kp=kp, kd=kd
    )

    msg = recv(sniffer)

    # Position is encoded in arb_id bits[23:8]
    pos_raw = CyberGearMotor._float_to_uint(
        position, CyberGearMotor.P_MIN, CyberGearMotor.P_MAX, 16
    )
    assert msg.arbitration_id == 0x01000000 | (pos_raw << 8) | MOTOR_ID

    # Data: velocity | kp | kd | torque — all big-endian uint16
    expected = struct.pack(
        '>4H',
        CyberGearMotor._float_to_uint(
            velocity, CyberGearMotor.V_MIN, CyberGearMotor.V_MAX, 16
        ),
        CyberGearMotor._float_to_uint(
            kp, CyberGearMotor.KP_MIN, CyberGearMotor.KP_MAX, 16
        ),
        CyberGearMotor._float_to_uint(
            kd, CyberGearMotor.KD_MIN, CyberGearMotor.KD_MAX, 16
        ),
        CyberGearMotor._float_to_uint(
            torque, CyberGearMotor.T_MIN, CyberGearMotor.T_MAX, 16
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


# ---------------------------------------------------------------------------
# Encoder calibration
# ---------------------------------------------------------------------------


def test_encoder_calibration_trigger(motor, sniffer):
    import threading

    threading.Thread(
        target=lambda: motor.encoder_calibration(timeout=0.2), daemon=True
    ).start()

    msg = recv(sniffer)
    assert msg.arbitration_id == 0x0500FD01
    assert bytes(msg.data) == b'\x00' * 8
