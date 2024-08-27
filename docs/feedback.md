# Feedback, polling & listeners

## Feedback snapshot

Every command sent to the motor triggers a feedback frame in response.
`motor.feedback` returns the latest immutable snapshot:

```python
fb = motor.feedback

print(fb.position)     # rad   — load-side position, saturates at ±12.5
print(fb.velocity)     # rad/s — load-side velocity, ±30
print(fb.torque)       # Nm    — ±12
print(fb.temperature)  # °C
print(fb.mode)         # 0=Reset, 1=Calibration, 2=Run
print(fb.faults)       # FaultState dataclass
```

!!! warning
    `fb.position` saturates at ±12.5 rad. Use `motor.mech_pos` for unbounded
    multi-turn position. `fb.velocity` and `fb.torque` only update when a
    command is sent — use `motor.mech_vel` for live velocity.

---

## Background polling

Start polling to keep parameter properties continuously up to date:

```python
motor.start_polling(interval=0.1)   # every 100 ms

# These now update automatically
print(motor.mech_pos)   # rad   — unbounded multi-turn position
print(motor.mech_vel)   # rad/s
print(motor.v_bus)      # V
print(motor.run_mode)   # 'speed', 'position', etc.
print(motor.rotation)   # turn counter
```

Polling is enabled by default at 1 s when the motor is created. Pass
`poll_interval=None` to disable it:

```python
motor = CyberGearMotor(can_id=0x01, poll_interval=None)
```

Stop and restart at any time:

```python
motor.stop_polling()
motor.start_polling(interval=0.05)  # 20 Hz
```

---

## Parameter properties

All readable parameters (updated by polling or by a write response):

| Property        | Unit   | Range        | Description                        |
|-----------------|--------|--------------|------------------------------------|
| `mech_pos`      | rad    | unbounded    | Load-side lap-counting position    |
| `mech_vel`      | rad/s  | ±30          | Load-side velocity                 |
| `v_bus`         | V      | 16–28        | Bus voltage                        |
| `run_mode`      | str    | —            | Current mode name                  |
| `rotation`      | int    | —            | Full-turn counter                  |
| `iq_ref`        | A      | ±23          | Last Iq command sent               |
| `spd_ref`       | rad/s  | ±30          | Last speed command sent            |
| `loc_ref`       | rad    | unbounded    | Last position command sent         |
| `limit_spd`     | rad/s  | 0–30         | Position mode speed limit          |
| `limit_cur`     | A      | 0–23         | Speed/position mode current limit  |
| `limit_torque`  | Nm     | 0–12         | Torque limit                       |
| `iqf`           | A      | ±23          | Filtered Iq feedback               |
| `cur_kp`        | —      | 0–200        | Current loop P gain                |
| `cur_ki`        | —      | 0–200        | Current loop I gain                |
| `cur_filt_gain` | —      | 0–1          | Current filter coefficient         |
| `loc_kp`        | —      | 0–200        | Position loop P gain               |
| `spd_kp`        | —      | 0–200        | Speed loop P gain                  |
| `spd_ki`        | —      | 0–200        | Speed loop I gain                  |

---

## Listeners / callbacks

Subscribe to motor events from any thread. Callbacks run in the `can.Notifier`
thread — keep them fast, offload heavy work to a queue.

```python
from cybergear import CyberGearMotor, MotorFeedback, FaultState

def on_feedback(fb: MotorFeedback) -> None:
    print(f'pos={fb.position:.4f} rad  temp={fb.temperature:.1f} °C')

def on_parameter(name: str, value: float) -> None:
    print(f'{name} = {value}')

def on_fault(state: FaultState) -> None:
    if state.has_fault:
        print(f'FAULT: {state}')

motor.add_feedback_listener(on_feedback)
motor.add_parameter_listener(on_parameter)
motor.add_fault_listener(on_fault)

# Remove when no longer needed
motor.remove_feedback_listener(on_feedback)
```

Fault listeners fire on any **transition** — when a fault appears and when it
clears.

---

## Fault state

```python
fb = motor.feedback
fs = fb.faults

print(fs.has_fault)                  # True if any fault is active
print(fs.calibrated)                 # False = not calibrated (counts as fault)
print(fs.over_temperature)
print(fs.over_current)
print(fs.undervoltage)
print(fs.hall_encoding_failure)
print(fs.magnetic_encoding_failure)
```
