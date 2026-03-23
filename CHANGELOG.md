# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [0.1.1] - 2026-03-22

### Fixed
- `motor_control()` frame layout: torque field belongs in `arb_id[23:8]`, position/velocity/kp/kd in data bytes (big-endian uint16). Previous layout had torque and position swapped, causing motors to go to wrong positions.
- `reset_to_zero()` now works correctly.
- `loc_ref` setter now auto-sends `limit_spd` first (mirrors official app behaviour — motor won't move if speed limit is zero).
- `_parse_parameter_table_message` no longer crashes the notifier thread on unknown parameter index.

### Added
- Full unit test suite (mock-based, no hardware required).
- Virtual CAN integration tests using `vcan0`.
- Hardware tests gated with `-m hardware` pytest marker.
- `ruff` formatting and linting.

### Changed
- `can_id` default changed from `0x7F` to `None`; passing `None` triggers `scan()` to auto-discover the motor.
- `ParameterTable.v_ref` renamed to `elec_offset` (confirmed via encoder calibration dump).

---

## [0.1.0] - 2026-03-21

Initial release of the Python driver for the Xiaomi CyberGear brushless motor.

### Added
- `CyberGearMotor` class for controlling the motor over CAN bus via `python-can`.
- Motion modes: motion control, position, velocity, current (torque).
- `motor_control(position, velocity, torque, kp, kd)` for closed-loop control.
- `set_position_ref()`, `set_speed_ref()`, `set_current_ref()` setters.
- `enable()` / `disable()` / `emergency_brake()` motor state commands.
- `reset_to_zero()` — sets the current position as the new zero reference.
- `encoder_calibration()` — triggers calibration and blocks up to 30 s for the result.
- `set_can_id()` — changes the motor's CAN ID.
- `scan()` classmethod — auto-discovers a motor by querying CAN IDs 1–127.
- `CommunicationTypeCan`, `ParameterIndex`, `ParameterTable`, `RunMode`, `CanMask` enums.
- `CybergearException` / `CybergearMotorInitException` for structured error handling.
- `cybergear-scan` CLI entry point.
- `cybergear-dashboard` CLI entry point (requires `textual` / `rich`).
