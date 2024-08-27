# Control modes

The motor supports five control modes. The mode cannot be switched while the
motor is running — always call `motor.disable()` or `motor.quick_stop()` before
switching.

---

## Quick-move (velocity)

The simplest way to spin the motor. No mode switch needed — the motor starts in
this mode by default after `enable()`.

```python
motor.enable()

motor.quick_move(2.0)   # 2 rad/s forward
motor.quick_move(-2.0)  # 2 rad/s reverse
motor.quick_stop()      # decelerate to zero
```

`quick_move` uses a special run-mode encoding that does not require switching
`run_mode` first. It is the recommended mode for jogging and simple velocity
control.

---

## Speed mode

Set `limit_cur` before commanding motion to protect the motor:

```python
motor.run_mode = 'speed'
motor.enable()

motor.limit_cur = 5.0    # A — current limit (0–23 A)
motor.spd_ref = 5.0      # rad/s — range: ±30 rad/s

# Stop
motor.spd_ref = 0.0
```

---

## Position mode

Set `limit_spd` and `limit_cur` before sending the first position command.
The motor will not move if `limit_spd` is zero.

```python
motor.run_mode = 'position'
motor.enable()

motor.limit_spd = 10.0   # rad/s — position mode speed limit (0–30 rad/s)
motor.limit_cur = 5.0    # A — current limit (0–23 A)
motor.loc_ref = 3.14     # move to π rad

# Move to another position
motor.loc_ref = 0.0      # return to zero
```

!!! note
    `loc_ref` automatically re-sends the cached `limit_spd` value before each
    position command, as required by the firmware. If polling is disabled and
    you have not set `limit_spd` explicitly, set it before the first `loc_ref`.

---

## Operation mode (MIT-style)

Full PD control with torque feed-forward. Suitable for compliant or
force-controlled applications. Must be called in a loop at your desired
control rate.

```python
motor.run_mode = 'operation'
motor.enable()

import time

target = 1.0  # rad

for _ in range(100):
    # τ_out = kp * (target − pos) + kd * (0 − vel) + torque
    motor.motor_control(
        torque=0.0,
        position=target,
        velocity=0.0,
        kp=10.0,
        kd=1.0,
    )
    time.sleep(0.01)  # 100 Hz
```

Parameter ranges:

| Parameter  | Range         |
|------------|---------------|
| `position` | ±12.5 rad     |
| `velocity` | ±30 rad/s     |
| `kp`       | 0–500         |
| `kd`       | 0–5           |
| `torque`   | ±12 Nm        |

---

## Current mode

Direct torque control via Iq current command. The motor accelerates freely —
use with care on unloaded shafts.

```python
motor.run_mode = 'current'
motor.enable()

motor.iq_ref = 0.5    # A — forward torque
time.sleep(0.5)
motor.iq_ref = -0.5   # A — reverse torque
time.sleep(0.5)
motor.iq_ref = 0.0    # zero torque
```

Range: ±23 A. Rated current is 6.5 A; peak is 23 A (short bursts only).

---

## Zero position

```python
# Set current position as the new zero (lost on power-off)
motor.reset_zero_position()

# Command the motor to drive back to the stored zero
motor.return_zero_position()
```
