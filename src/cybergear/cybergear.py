import logging
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import can
from can import CanError

from cybergear.can_definitions import (
    CanMask,
    CommunicationTypeCan,
    ParameterIndex,
    ParameterTable,
    RunMode,
)
from cybergear.exceptions import CybergearMotorInitException

logger = logging.getLogger(__name__)

FeedbackListener = Callable[['MotorFeedback'], None]
ParameterListener = Callable[[str, float | int], None]
FaultListener = Callable[['FaultState'], None]


@dataclass(frozen=True)
class FaultState:
    """Immutable snapshot of motor fault bits decoded from a feedback frame."""

    calibrated: bool = True
    hall_encoding_failure: bool = False
    magnetic_encoding_failure: bool = False
    over_temperature: bool = False
    over_current: bool = False
    undervoltage: bool = False

    @property
    def has_fault(self) -> bool:
        """True if any fault bit is active."""
        return (
            not self.calibrated
            or self.hall_encoding_failure
            or self.magnetic_encoding_failure
            or self.over_temperature
            or self.over_current
            or self.undervoltage
        )


@dataclass(frozen=True)
class MotorFeedback:
    """Immutable snapshot of motor state from a single feedback frame (comm type 2).

    Delivered to feedback listeners on every incoming feedback frame.
    For full-precision parameter values (float), use the motor's parameter
    properties (e.g. ``motor.mech_pos``), which are updated by parameter reads.
    """

    position: float = 0.0  # rad — load-side mechanical position, ±12.5 rad
    velocity: float = 0.0  # rad/s — load-side mechanical velocity, ±30 rad/s
    torque: float = 0.0  # Nm — ±12 Nm
    temperature: float = 0.0  # °C
    mode: int = 0  # 0: Reset, 1: Calibration, 2: Run
    faults: FaultState = field(default_factory=FaultState)


class _PollingThread(threading.Thread):
    """Background thread that periodically sends parameter read requests.

    Replaces the SocketCAN BCM approach with a portable threading solution
    that works on any python-can interface.
    """

    def __init__(self, poll_fn: Callable[[], None], interval: float) -> None:
        super().__init__(daemon=True, name='cybergear-poller')
        self._poll_fn = poll_fn
        self._interval = interval
        self._stop_event = threading.Event()

    def run(self) -> None:
        while not self._stop_event.wait(self._interval):
            try:
                self._poll_fn()
            except Exception:
                logger.exception('Error in polling thread')

    def stop(self) -> None:
        self._stop_event.set()


class CyberGearMotor:
    """Python driver for the Xiaomi CyberGear brushless motor over CAN bus.

    Supports five control modes:

    - **Operation mode** (MIT-style): ``motor.motor_control(torque, position, velocity, kp, kd)``
    - **Position mode**: ``motor.run_mode = 'position'``, then ``motor.loc_ref = <rad>``
    - **Speed mode**: ``motor.run_mode = 'speed'``, then ``motor.spd_ref = <rad/s>``
    - **Current mode**: ``motor.run_mode = 'current'``, then ``motor.iq_ref = <A>``
    - **Quick-move**: ``motor.quick_move(speed)`` / ``motor.quick_stop()``

    Incoming frames are handled asynchronously by a ``can.Notifier`` listener.
    Optional background polling (portable, works on any python-can interface)
    keeps parameter properties up to date.

    Use as a context manager to ensure clean resource release::

        bus_cfg = {'interface': 'socketcan', 'channel': 'can0', 'bitrate': 1_000_000}
        with CyberGearMotor(bus_config=bus_cfg) as motor:
            motor.run_mode = 'speed'
            motor.enable()
            motor.spd_ref = 5.0

    Subscribe to motor events::

        def on_feedback(fb: MotorFeedback) -> None:
            print(fb.position, fb.velocity, fb.temperature)

        motor.add_feedback_listener(on_feedback)
    """

    # Control ranges — from datasheet section 4.2.1 and 4.1.2.
    # Position: ±4π rad; the C reference uses 12.5 as the approximation of 4π.
    P_MIN: float = -12.5
    P_MAX: float = 12.5
    V_MIN: float = -30.0  # rad/s
    V_MAX: float = 30.0
    KP_MIN: float = 0.0
    KP_MAX: float = 500.0
    KD_MIN: float = 0.0
    KD_MAX: float = 5.0
    T_MIN: float = -12.0  # Nm
    T_MAX: float = 12.0

    # Feedback frame position uses the same ±4π range as the control frame
    # (datasheet 4.1.3). Previously ±4.0 — corrected; verify on hardware.
    FEEDBACK_P_MIN: float = -12.5
    FEEDBACK_P_MAX: float = 12.5

    def __init__(
        self,
        bus_config: dict | None = None,
        can_id: int | None = None,
        host_can_id: int = 0xFD,
        poll_interval: float | None = 1.0,
        send_timeout: float = 0.1,
    ) -> None:
        """
        :param bus_config: python-can ``Bus`` kwargs (e.g. ``interface``, ``channel``,
                           ``bitrate``). Uses python-can's default bus when ``None``.
        :param can_id: CAN node ID of the motor. If ``None``, the bus is scanned
                       automatically; raises if zero or multiple motors are found.
        :param host_can_id: CAN node ID of this host (default ``0xFD``).
        :param poll_interval: Seconds between background parameter polls.
                              ``None`` disables polling entirely.
        :param send_timeout: Seconds to wait for the TX buffer to accept each frame
                             (default ``0.1``). Increase if you see transmit buffer errors
                             on slow USB CAN adapters.
        :raises CybergearMotorInitException: If the motor does not respond within 1 s,
                                             or if auto-scan finds zero or multiple motors.

        **Internal state dicts**

        ``_parameters`` — values reported *by the motor* via poll responses.
        Updated only when the motor echoes a value back. Represents confirmed
        motor state; may lag behind a write by up to one poll interval.

        ``_commanded`` — values *last sent* by the driver via write commands.
        Updated immediately on every ``_write_parameter`` call, without waiting
        for the motor to confirm. Used where a command depends on the most recent
        value the user set (e.g. ``loc_ref`` prepending ``limit_spd``), so that
        the commanded value is used instead of a potentially stale polled value.
        """
        self._send_timeout = send_timeout
        if can_id is None:
            found = CyberGearMotor.scan(
                bus_config=bus_config,
                host_can_id=host_can_id,
                send_timeout=send_timeout,
            )
            if not found:
                raise CybergearMotorInitException(
                    'No CyberGear motors found on the CAN bus.'
                )
            if len(found) > 1:
                ids = [cid for cid, _ in found]
                raise CybergearMotorInitException(
                    f'Multiple motors found {ids}. Specify can_id explicitly.'
                )
            can_id = found[0][0]
            logger.info('Auto-detected motor can_id=%d', can_id)
        self._bus_config = bus_config
        self._bus = self._get_bus()
        self.can_id = can_id
        self.host_can_id = host_can_id

        self._bus.set_filters(
            [
                {
                    'can_id': self.can_id << 8,
                    'can_mask': 0x0000FF00,
                    'extended': True,
                }
            ]
        )

        self._feedback = MotorFeedback()
        self._parameters: dict[str, float | int] = {}
        self._commanded: dict[str, float | int] = {}
        self._parameters_table: dict[str, float | int | str] = {}
        self._device_id: bytes | None = None

        self._feedback_listeners: list[FeedbackListener] = []
        self._parameter_listeners: list[ParameterListener] = []
        self._fault_listeners: list[FaultListener] = []
        self._calibration_callback: Callable[[float], None] | None = None

        self._device_id_event = threading.Event()
        self._notifier = can.Notifier(self._bus, [self._on_message_received])
        self._poller: _PollingThread | None = None

        self._init_motor()

        if poll_interval is not None:
            self.start_polling(poll_interval)

    # -------------------------------------------------------------------------
    # Context manager
    # -------------------------------------------------------------------------

    def __enter__(self) -> 'CyberGearMotor':
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def close(self) -> None:
        """Release all resources: stop polling, notifier, and CAN bus.

        The motor is NOT disabled before closing. If you want the motor to
        stop holding position/torque on exit, call disable() first.
        """
        self.stop_polling()
        self._notifier.stop()
        self._bus.shutdown()

    # -------------------------------------------------------------------------
    # Bus scanning
    # -------------------------------------------------------------------------

    @classmethod
    def scan(
        cls,
        bus_config: dict | None = None,
        host_can_id: int = 0xFD,
        timeout: float = 0.5,
        candidates: list[int] | None = None,
        send_timeout: float = 0.1,
    ) -> list[tuple[int, bytes]]:
        """Scan the CAN bus for responding CyberGear motors.

        Sends a ``device_id`` query to each candidate CAN ID and collects
        responses within *timeout* seconds.

        :param bus_config: python-can ``Bus`` kwargs.
        :param host_can_id: CAN node ID of this host (default ``0xFD``).
        :param timeout: Seconds to wait for responses after sending all queries.
        :param candidates: Motor CAN IDs to probe. Defaults to 1–127.
        :param send_timeout: Seconds to wait for the TX buffer to accept each frame
                             (default ``0.1``). Increase if you see transmit buffer errors
                             on slow USB CAN adapters.
        :returns: Sorted list of ``(can_id, device_id_bytes)`` for each motor found.
        """
        if candidates is None:
            candidates = list(range(1, 128))

        found: dict[int, bytes] = {}
        bus = can.interface.Bus(**bus_config) if bus_config else can.Bus()

        def _on_message(msg: can.Message) -> None:
            comm = msg.arbitration_id & CanMask.communication_type
            if comm == CommunicationTypeCan.device_id:
                motor_id = (msg.arbitration_id >> 8) & 0xFF
                if motor_id and motor_id not in found:
                    found[motor_id] = bytes(msg.data)

        notifier = can.Notifier(bus, [_on_message])
        try:
            for can_id in candidates:
                arb_id = (
                    int(CommunicationTypeCan.device_id) | (host_can_id << 8) | can_id
                )
                bus.send(
                    can.Message(
                        arbitration_id=arb_id,
                        data=bytearray(1),
                        is_extended_id=True,
                    ),
                    timeout=send_timeout,
                )
            time.sleep(timeout)
        except CanError as e:
            raise CanError(f'CAN bus unavailable: {e}') from e
        finally:
            notifier.stop()
            bus.shutdown()

        logger.info('Scan found %d motor(s): %s', len(found), sorted(found))
        return sorted(found.items())

    # -------------------------------------------------------------------------
    # Listener management
    # -------------------------------------------------------------------------

    def add_feedback_listener(self, fn: FeedbackListener) -> None:
        """Register a callback invoked on every motor feedback frame (comm type 2).

        The callback receives a :class:`MotorFeedback` snapshot and runs in the
        ``can.Notifier`` thread. Keep it fast; offload heavy work to a queue.
        """
        self._feedback_listeners.append(fn)

    def remove_feedback_listener(self, fn: FeedbackListener) -> None:
        """Unregister a previously added feedback listener. No-op if not registered."""
        try:
            self._feedback_listeners.remove(fn)
        except ValueError:
            pass

    def add_parameter_listener(self, fn: ParameterListener) -> None:
        """Register a callback invoked when a parameter read response arrives.

        Receives ``(name: str, value: float | int)``.
        """
        self._parameter_listeners.append(fn)

    def remove_parameter_listener(self, fn: ParameterListener) -> None:
        """Unregister a previously added parameter listener. No-op if not registered."""
        try:
            self._parameter_listeners.remove(fn)
        except ValueError:
            pass

    def add_fault_listener(self, fn: FaultListener) -> None:
        """Register a callback invoked whenever the fault state changes.

        Fires on any transition (fault appears *or* clears). Receives a
        :class:`FaultState` snapshot.
        """
        self._fault_listeners.append(fn)

    def remove_fault_listener(self, fn: FaultListener) -> None:
        """Unregister a previously added fault listener. No-op if not registered."""
        try:
            self._fault_listeners.remove(fn)
        except ValueError:
            pass

    # -------------------------------------------------------------------------
    # Polling
    # -------------------------------------------------------------------------

    def start_polling(self, interval: float = 1.0) -> None:
        """Start periodic background parameter polling.

        Sends a read request for every :class:`~cybergear.can_definitions.ParameterIndex`
        every *interval* seconds. Works on any python-can interface (no SocketCAN BCM needed).

        :param interval: Poll interval in seconds.
        """
        self.stop_polling()
        self._poller = _PollingThread(self._fetch_parameter_list, interval)
        self._poller.start()

    def stop_polling(self) -> None:
        """Stop background parameter polling."""
        if self._poller is not None:
            self._poller.stop()
            self._poller.join()
            self._poller = None

    # -------------------------------------------------------------------------
    # Motor control
    # -------------------------------------------------------------------------

    def enable(self) -> None:
        """Arm the motor. Clears any latched fault state before enabling."""
        self._send_message(CommunicationTypeCan.motor_stopped, bytearray(8))
        self._send_message(CommunicationTypeCan.motor_enable, bytearray(8))

    def disable(self) -> None:
        """Disarm the motor gracefully."""
        self._send_message(CommunicationTypeCan.motor_stopped, bytearray(8))

    def emergency_brake(self) -> None:
        """Immediately cut motor torque (reverse-engineered, comm type 0x14).

        Call ``enable()`` to recover. Behaviour confirmed by traffic capture;
        not documented in the official datasheet.
        """
        self._send_message(CommunicationTypeCan.emergency_stop, bytearray(8))

    def motor_control(
        self,
        torque: float,
        position: float,
        velocity: float,
        kp: float,
        kd: float,
    ) -> None:
        """MIT-style operation control (comm type 1).

        All five parameters act simultaneously as a PD controller with torque
        feed-forward::

            τ_out = kp * (position − pos) + kd * (velocity − vel) + torque

        The motor must be enabled and in operation mode before calling this.

        :param torque: Feed-forward torque (Nm), clamped to ±12 Nm.
        :param position: Target position (rad), clamped to ±12.5 rad.
        :param velocity: Target velocity (rad/s), clamped to ±30 rad/s.
        :param kp: Position gain, 0–500.
        :param kd: Velocity gain, 0–5.
        """
        torque_raw = self._float_to_uint(torque, self.T_MIN, self.T_MAX, 16)
        data = (
            struct.pack('>H', self._float_to_uint(position, self.P_MIN, self.P_MAX, 16))
            + struct.pack(
                '>H', self._float_to_uint(velocity, self.V_MIN, self.V_MAX, 16)
            )
            + struct.pack('>H', self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16))
            + struct.pack('>H', self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16))
        )
        self._send_message(
            CommunicationTypeCan.control_instructions, data, data2=torque_raw
        )

    def quick_move(self, speed: float) -> None:
        """Command a target velocity in quick-move mode (rad/s)."""
        data = (
            struct.pack('2H', ParameterIndex.run_mode.value, 0)
            + struct.pack('2B', RunMode.quick_move.value, 1)
            + struct.pack('>H', self._float_to_uint(speed, self.V_MIN, self.V_MAX, 16))
        )
        self._send_message(CommunicationTypeCan.parameter_writing, data)

    def quick_stop(self) -> None:
        """Halt the motor in quick-move mode."""
        data = (
            struct.pack('2H', ParameterIndex.run_mode.value, 0)
            + struct.pack('2B', RunMode.quick_move.value, 0)
            + struct.pack('>H', 0x7FFF)
        )
        self._send_message(CommunicationTypeCan.parameter_writing, data)

    def reset_zero_position(self) -> None:
        """Set the current mechanical position as the new zero reference (lost on power-off)."""
        self._send_message(CommunicationTypeCan.zero_position, [1, 0, 0, 0, 0, 0, 0, 0])

    def return_zero_position(self) -> None:
        """Command the motor to return to the stored zero reference position."""
        data = (
            struct.pack('2H', ParameterIndex.run_mode.value, 0)
            + struct.pack('2B', RunMode.zero_position.value, 0)
            + struct.pack('>H', 0x0000)
        )
        self._send_message(CommunicationTypeCan.parameter_writing, data)
        self.enable()

    def set_can_id(self, can_id: int) -> None:
        """Persistently change the motor's CAN node ID (survives power cycles)."""
        logger.warning('Changing motor CAN ID to %d', can_id)
        data2 = (can_id & 0xFF) << 8 | self.host_can_id
        self._send_message(
            CommunicationTypeCan.set_can_id, data1=self._device_id, data2=data2
        )
        self.can_id = can_id

    def fetch_parameter_table(self) -> None:
        """Request the full parameter table from the motor (response is async)."""
        self._send_message(CommunicationTypeCan.parameter_table, data1=self._device_id)

    def print_parameter_table(self) -> None:
        """Print all parameter table values received so far."""
        for name, value in self._parameters_table.items():
            print(f'{name}: {value}')

    def encoder_calibration(self, timeout: float = 30.0) -> float | None:
        """Trigger encoder electrical offset calibration (comm type 0x05).

        The motor sweeps the encoder and computes the electrical offset. This
        takes several seconds. The result is returned as a float (rad) and also
        stored in the parameter table as ``elec_offset``.

        .. warning::
            The motor shaft must be free to rotate during calibration.

        :param timeout: Maximum seconds to wait for the calibration result.
        :returns: Electrical offset in radians, or ``None`` if timed out.
        """
        event = threading.Event()
        result: list[float] = []

        def _on_calib(value: float) -> None:
            result.append(value)
            event.set()

        self._calibration_callback = _on_calib
        self._send_message(CommunicationTypeCan.encoder_calibration, bytearray(8))
        if not event.wait(timeout=timeout):
            logger.warning('Encoder calibration timed out after %.1f s', timeout)
            self._calibration_callback = None
            return None
        self._calibration_callback = None
        elec_offset = result[0]
        self._parameters_table[ParameterTable.elec_offset.name] = elec_offset
        logger.info('Encoder calibration complete: elec_offset=%.6f rad', elec_offset)
        return elec_offset

    def set_watchdog_timeout(self, ms: int) -> None:
        """Configure the motor's CAN timeout watchdog.

        When non-zero, the motor stops if no CAN frame is received within *ms*
        milliseconds. Set to 0 to disable (factory default).

        .. warning::
            Hardware testing shows this command has no effect. The motor firmware
            does not appear to accept ``parameter_writing`` (comm type 0x12) for
            ``ParameterTable`` indices (0x200x range). The watchdog may only be
            configurable via the official Xiaomi debugger application.

        :param ms: Timeout in milliseconds (0 = disabled, max 100 000).
        :raises ValueError: If *ms* is outside the valid range.
        """
        if not 0 <= ms <= 100_000:
            raise ValueError(f'Watchdog timeout must be 0–100 000 ms, got {ms}')
        self._write_parameter(ParameterTable.can_timeout, 'I', ms)

    # -------------------------------------------------------------------------
    # Properties — feedback snapshot (read-only, updated from comm type 2)
    # -------------------------------------------------------------------------

    @property
    def feedback(self) -> MotorFeedback:
        """Latest motor feedback snapshot (updated on every feedback frame)."""
        return self._feedback

    @property
    def device_id(self) -> str | None:
        """64-bit MCU unique identifier as a hex string, or ``None`` before init."""
        return self._device_id.hex() if self._device_id else None

    # -------------------------------------------------------------------------
    # Properties — parameters (read: last polled value; write: sends CAN frame)
    # -------------------------------------------------------------------------

    @property
    def run_mode(self) -> str | None:
        """Current run mode name, or ``None`` if not yet received."""
        raw = self._parameters.get(ParameterIndex.run_mode.name)
        return RunMode(raw).name if raw is not None else None

    @run_mode.setter
    def run_mode(self, value: str) -> None:
        """Set run mode by name. Valid values: ``'operation'``, ``'position'``,
        ``'speed'``, ``'current'``."""
        try:
            mode = RunMode[value].value
        except KeyError:
            logger.warning(
                'Run mode %r not supported. Valid modes: %s',
                value,
                [e.name for e in RunMode],
            )
            return
        self._write_parameter(ParameterIndex.run_mode, '4B', mode, 0, 0, 0)

    @property
    def iq_ref(self) -> float | None:
        """Current mode Iq command (A), last polled value."""
        return self._parameters.get(ParameterIndex.iq_ref.name)

    @iq_ref.setter
    def iq_ref(self, value: float) -> None:
        """Set Iq current command (A). Motor should be in ``'current'`` mode."""
        self._warn_run_mode(RunMode.current)
        self._write_parameter(ParameterIndex.iq_ref, 'f', value)

    @property
    def spd_ref(self) -> float | None:
        """Speed mode command (rad/s), last polled value."""
        return self._parameters.get(ParameterIndex.spd_ref.name)

    @spd_ref.setter
    def spd_ref(self, value: float) -> None:
        """Set speed command (rad/s). Motor should be in ``'speed'`` mode."""
        self._warn_run_mode(RunMode.speed)
        self._write_parameter(ParameterIndex.spd_ref, 'f', value)

    @property
    def limit_torque(self) -> float | None:
        """Torque limit (Nm), last polled value."""
        return self._parameters.get(ParameterIndex.limit_torque.name)

    @limit_torque.setter
    def limit_torque(self, value: float) -> None:
        """Set torque limit (Nm). Valid range: 0–12 Nm."""
        if not 0.0 <= value <= 12.0:
            raise ValueError(f'limit_torque must be 0–12 Nm, got {value}')
        self._write_parameter(ParameterIndex.limit_torque, 'f', value)

    @property
    def cur_kp(self) -> float | None:
        """Current loop proportional gain (0–200), last polled value."""
        return self._parameters.get(ParameterIndex.cur_kp.name)

    @cur_kp.setter
    def cur_kp(self, value: float) -> None:
        """Set current loop proportional gain. Valid range: 0–200."""
        self._write_parameter(ParameterIndex.cur_kp, 'f', value)

    @property
    def cur_ki(self) -> float | None:
        """Current loop integral gain (0–200), last polled value."""
        return self._parameters.get(ParameterIndex.cur_ki.name)

    @cur_ki.setter
    def cur_ki(self, value: float) -> None:
        """Set current loop integral gain. Valid range: 0–200."""
        self._write_parameter(ParameterIndex.cur_ki, 'f', value)

    @property
    def cur_filt_gain(self) -> float | None:
        """Current filter coefficient (0–1), last polled value."""
        return self._parameters.get(ParameterIndex.cur_filt_gain.name)

    @cur_filt_gain.setter
    def cur_filt_gain(self, value: float) -> None:
        """Set current filter coefficient. Valid range: 0–1."""
        if not 0.0 <= value <= 1.0:
            raise ValueError(f'cur_filt_gain must be 0–1, got {value}')
        self._write_parameter(ParameterIndex.cur_filt_gain, 'f', value)

    @property
    def loc_ref(self) -> float | None:
        """Position mode angle command (rad), last polled value."""
        return self._parameters.get(ParameterIndex.loc_ref.name)

    @loc_ref.setter
    def loc_ref(self, value: float) -> None:
        """Set position command (rad). Motor should be in ``'position'`` mode.

        Also sends the current ``limit_spd`` to the motor before the position
        command — required by the firmware for position mode to work (motor will
        not move if ``limit_spd`` is 0). Uses the last polled value if available;
        if ``limit_spd`` has not been polled yet, set it explicitly first.
        """
        self._warn_run_mode(RunMode.position)
        limit = self._commanded.get(
            ParameterIndex.limit_spd.name,
            self._parameters.get(ParameterIndex.limit_spd.name),
        )
        if limit is not None:
            self._write_parameter(ParameterIndex.limit_spd, 'f', limit)
        self._write_parameter(ParameterIndex.loc_ref, 'f', value)

    @property
    def limit_spd(self) -> float | None:
        """Position mode speed limit (rad/s), last polled value."""
        return self._parameters.get(ParameterIndex.limit_spd.name)

    @limit_spd.setter
    def limit_spd(self, value: float) -> None:
        """Set position mode speed limit (rad/s). Valid range: 0–30 rad/s."""
        if not 0.0 <= value <= 30.0:
            raise ValueError(f'limit_spd must be 0–30 rad/s, got {value}')
        self._warn_run_mode(RunMode.position)
        self._write_parameter(ParameterIndex.limit_spd, 'f', value)

    @property
    def limit_cur(self) -> float | None:
        """Speed/position mode current limit (A), last polled value."""
        return self._parameters.get(ParameterIndex.limit_cur.name)

    @limit_cur.setter
    def limit_cur(self, value: float) -> None:
        """Set current limit (A). Valid range: 0–23 A."""
        if not 0.0 <= value <= 23.0:
            raise ValueError(f'limit_cur must be 0–23 A, got {value}')
        self._warn_run_mode(RunMode.speed)
        self._write_parameter(ParameterIndex.limit_cur, 'f', value)

    @property
    def mech_pos(self) -> float | None:
        """Load-side lap-counting mechanical angle (rad), last polled value.

        Full float precision. For a fast low-precision reading, use
        ``motor.feedback.position`` instead.
        """
        return self._parameters.get(ParameterIndex.mech_pos.name)

    @property
    def iqf(self) -> float | None:
        """Filtered Iq current feedback (A), last polled value."""
        return self._parameters.get(ParameterIndex.iqf.name)

    @property
    def mech_vel(self) -> float | None:
        """Load-side mechanical velocity (rad/s), last polled value."""
        return self._parameters.get(ParameterIndex.mech_vel.name)

    @property
    def v_bus(self) -> float | None:
        """Bus voltage (V), last polled value."""
        return self._parameters.get(ParameterIndex.v_bus.name)

    @property
    def rotation(self) -> int | float | None:
        """Turn counter (number of full rotations), last polled value."""
        return self._parameters.get(ParameterIndex.rotation.name)

    @property
    def loc_kp(self) -> float | None:
        """Position loop proportional gain (0–200), last polled value."""
        return self._parameters.get(ParameterIndex.loc_kp.name)

    @loc_kp.setter
    def loc_kp(self, value: float) -> None:
        """Set position loop proportional gain. Valid range: 0–200."""
        self._write_parameter(ParameterIndex.loc_kp, 'f', value)

    @property
    def spd_kp(self) -> float | None:
        """Speed loop proportional gain (0–200), last polled value."""
        return self._parameters.get(ParameterIndex.spd_kp.name)

    @spd_kp.setter
    def spd_kp(self, value: float) -> None:
        """Set speed loop proportional gain. Valid range: 0–200."""
        self._write_parameter(ParameterIndex.spd_kp, 'f', value)

    @property
    def spd_ki(self) -> float | None:
        """Speed loop integral gain (0–200), last polled value."""
        return self._parameters.get(ParameterIndex.spd_ki.name)

    @spd_ki.setter
    def spd_ki(self, value: float) -> None:
        """Set speed loop integral gain. Valid range: 0–200."""
        self._write_parameter(ParameterIndex.spd_ki, 'f', value)

    # -------------------------------------------------------------------------
    # Private helpers
    # -------------------------------------------------------------------------

    def _get_bus(self) -> can.BusABC:
        if not self._bus_config:
            return can.Bus()
        return can.interface.Bus(**self._bus_config)

    def _init_motor(self) -> None:
        self._send_message(CommunicationTypeCan.device_id, bytearray(1))
        if not self._device_id_event.wait(timeout=1):
            raise CybergearMotorInitException(
                'Motor did not respond within 1 s. Check CAN connection and motor power.'
            )

    def _fetch_parameter_list(self) -> None:
        for element in ParameterIndex:
            self._send_message(
                CommunicationTypeCan.parameter_reading,
                struct.pack('4H', element.value, 0, 0, 0),
            )

    def _send_message(
        self,
        communication_type: CommunicationTypeCan,
        data1: bytes | list | bytearray | None,
        data2: int | None = None,
    ) -> None:
        arbitration_id = int(communication_type) | self.can_id
        arbitration_id |= (data2 << 8) if data2 is not None else (self.host_can_id << 8)
        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data1,
            is_extended_id=True,
        )
        try:
            self._bus.send(msg, timeout=self._send_timeout)
        except CanError as e:
            logger.error(
                'Failed to send CAN message (type=%s) - %s', communication_type.name, e
            )

    def _write_parameter(
        self,
        index: ParameterIndex | ParameterTable,
        fmt: str,
        *values: float | int,
    ) -> None:
        data = struct.pack('2H', index.value, 0) + struct.pack(fmt, *values)
        self._send_message(CommunicationTypeCan.parameter_writing, data)
        if isinstance(index, ParameterIndex) and values:
            self._commanded[index.name] = values[0]

    def _warn_run_mode(self, expected: RunMode) -> None:
        raw = self._parameters.get(ParameterIndex.run_mode.name)
        if raw is not None and raw != expected.value:
            logger.warning(
                'Motor is in %r mode; this command requires %r mode.',
                RunMode(raw).name,
                expected.name,
            )

    def _fire_listeners(self, listeners: list, *args: object) -> None:
        for fn in listeners:
            try:
                fn(*args)
            except Exception:
                logger.exception('Listener %r raised an exception', fn)

    # -------------------------------------------------------------------------
    # Message dispatch — runs in the can.Notifier thread
    # -------------------------------------------------------------------------

    def _on_message_received(self, message: can.Message) -> None:
        comm_type = message.arbitration_id & CanMask.communication_type
        if comm_type == CommunicationTypeCan.parameter_reading:
            self._process_parameter_reading(message)
        elif comm_type == CommunicationTypeCan.device_id:
            self._device_id = bytes(message.data)
            self._device_id_event.set()
        elif comm_type == CommunicationTypeCan.motor_feedback:
            self._parse_motor_feedback_message(message)
        elif comm_type == CommunicationTypeCan.parameter_table:
            self._parse_parameter_table_message(message)
        elif comm_type == CommunicationTypeCan.encoder_calibration:
            self._parse_encoder_calibration_message(message)
        else:
            logger.debug('Unhandled CAN message: %s', message)

    def _process_parameter_reading(self, message: can.Message) -> None:
        index = struct.unpack('H', message.data[0:2])[0]
        try:
            param = ParameterIndex(index)
        except ValueError:
            logger.debug('Unknown parameter index in read response: 0x%04X', index)
            return

        if param == ParameterIndex.run_mode:
            fmt, size = 'B', 1
        elif param == ParameterIndex.rotation:
            fmt, size = 'h', 2
        else:
            fmt, size = 'f', 4

        value: float | int = struct.unpack(fmt, message.data[4 : 4 + size])[0]
        self._parameters[param.name] = value
        self._fire_listeners(self._parameter_listeners, param.name, value)

    def _parse_motor_feedback_message(self, message: can.Message) -> None:
        arb = message.arbitration_id
        faults = FaultState(
            calibrated=not bool((arb >> 21) & 0x1),
            hall_encoding_failure=bool((arb >> 20) & 0x1),
            magnetic_encoding_failure=bool((arb >> 19) & 0x1),
            over_temperature=bool((arb >> 18) & 0x1),
            over_current=bool((arb >> 17) & 0x1),
            undervoltage=bool((arb >> 16) & 0x1),
        )
        feedback = MotorFeedback(
            position=self._uint_to_float(
                struct.unpack('>H', message.data[0:2])[0],
                self.FEEDBACK_P_MIN,
                self.FEEDBACK_P_MAX,
                16,
            ),
            velocity=self._uint_to_float(
                struct.unpack('>H', message.data[2:4])[0],
                self.V_MIN,
                self.V_MAX,
                16,
            ),
            torque=self._uint_to_float(
                struct.unpack('>H', message.data[4:6])[0],
                self.T_MIN,
                self.T_MAX,
                16,
            ),
            # Raw value is Temp(°C) × 10 per datasheet table (motorTemp remark)
            temperature=struct.unpack('>H', message.data[6:8])[0] / 10.0,
            mode=(arb >> 22) & 0x3,
            faults=faults,
        )

        prev_faults = self._feedback.faults
        self._feedback = feedback
        self._fire_listeners(self._feedback_listeners, feedback)

        if faults != prev_faults:
            self._fire_listeners(self._fault_listeners, faults)

    def _parse_parameter_table_message(self, message: can.Message) -> None:
        raw_index = struct.unpack('H', message.data[0:2])[0]
        try:
            parameter = ParameterTable(raw_index)
        except ValueError:
            logger.debug('Unknown parameter table index: 0x%04X', raw_index)
            return
        index = (message.arbitration_id & CanMask.index_parameter_table) >> 16

        if parameter.parameter_type == '6s':
            data = (
                struct.unpack('6s', message.data[2:8])[0]
                .strip(b'\x00')
                .decode(errors='ignore')
            )
            if index == 0:
                self._parameters_table[parameter.name] = ''
            elif index in (6, 7, 8):
                self._parameters_table[parameter.name] += data  # ty: ignore[unsupported-operator]
        elif index == 4:
            fmt = parameter.parameter_type
            data = struct.unpack(fmt, message.data[4 : 4 + struct.calcsize(fmt)])[0]
            self._parameters_table[parameter.name] = data

    def _parse_encoder_calibration_message(self, message: can.Message) -> None:
        # Motor sends back the computed electrical offset as a LE float32 in bytes[0:4].
        elec_offset = struct.unpack('<f', message.data[0:4])[0]
        logger.debug('Encoder calibration result frame: elec_offset=%.6f', elec_offset)
        if self._calibration_callback is not None:
            self._calibration_callback(elec_offset)

    @staticmethod
    def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        x = max(min(x, x_max), x_min)
        return int((x - x_min) * (2**bits - 1) / (x_max - x_min))

    @staticmethod
    def _uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
        return x * (x_max - x_min) / (2**bits - 1) + x_min
