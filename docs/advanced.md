# Advanced usage

## Emergency stop

Immediately cuts motor torque (reverse-engineered comm type 0x14, not in the
official datasheet). Call `motor.enable()` to recover:

```python
motor.emergency_brake()

# Recover
motor.enable()
```

---

## Encoder calibration

Computes the electrical offset between the magnetic encoder's zero and the
rotor's magnetic zero. Required for efficient FOC. The factory runs it once —
only repeat it after disassembly or encoder replacement.

!!! warning
    The motor shaft must be free to rotate during calibration.

```python
offset = motor.encoder_calibration(timeout=30.0)
print(f'Electrical offset: {offset:.6f} rad')
```

The method blocks until the motor finishes the calibration sweep (typically a
few seconds) and returns the computed electrical offset in radians. Returns
`None` if the motor does not respond within `timeout` seconds.

---

## Changing the motor CAN ID

Persistently changes the motor's CAN node ID. Survives power cycles.

```python
motor.set_can_id(0x02)   # motor will now respond on ID 2
```

!!! warning
    This writes to non-volatile memory. If you set a conflicting or unknown ID,
    use `CyberGearMotor.scan()` to find the motor again.

---

## Scanning for motors

```python
found = CyberGearMotor.scan()
# [(1, b'\x01\x02...'), (3, b'\x03\x04...')]

for can_id, device_id in found:
    print(f'Motor CAN ID={can_id}  device_id={device_id.hex()}')
```

Scan a specific set of IDs to speed things up:

```python
found = CyberGearMotor.scan(candidates=[1, 2, 3, 4])
```

---

## Controller gains

All gains are lost on power-off unless saved via the official Xiaomi debugger.

```python
# Current loop
motor.cur_kp = 0.025      # default
motor.cur_ki = 0.0258
motor.cur_filt_gain = 0.9  # 0–1

# Speed loop
motor.spd_kp = 2.0
motor.spd_ki = 0.021

# Position loop
motor.loc_kp = 30.0
```

---

## Torque limit

```python
motor.limit_torque = 6.0   # Nm — range: 0–12 Nm
```

!!! warning
    Do not set `limit_torque` below the torque required to hold the load. The
    Xiaomi manual also warns against raising the protection temperature or
    over-temperature time.

---

## Watchdog timeout

!!! note "Known limitation"
    Hardware testing shows the motor firmware ignores writes to the
    `CAN_TIMEOUT` register via comm type 0x12. This method has no effect on
    current firmware. The watchdog can only be configured via the official
    Xiaomi debugger application.

```python
motor.set_watchdog_timeout(500)   # 500 ms — motor stops if no frame received
motor.set_watchdog_timeout(0)     # disable (factory default)
```

---

## Logging

The library uses Python's standard `logging` module under the
`cybergear.cybergear` logger. Enable debug output to see every CAN frame:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

Or target just the cybergear logger:

```python
logging.getLogger('cybergear.cybergear').setLevel(logging.DEBUG)
```
