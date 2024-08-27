# Examples

## Spin and stop

```python
from cybergear import CyberGearMotor

with CyberGearMotor(can_id=0x01) as motor:
    motor.enable()

    motor.quick_move(3.0)   # 3 rad/s

    import time
    time.sleep(2.0)

    motor.quick_stop()
    motor.disable()
```

---

## Move to absolute position

```python
from cybergear import CyberGearMotor
import time

with CyberGearMotor(can_id=0x01) as motor:
    motor.run_mode = 'position'
    motor.enable()

    motor.limit_spd = 10.0
    motor.limit_cur = 5.0

    for target in [1.57, 3.14, 0.0]:   # 90°, 180°, back to 0
        motor.loc_ref = target
        time.sleep(1.5)

    motor.disable()
```

---

## Velocity control with live feedback

```python
from cybergear import CyberGearMotor, MotorFeedback
import time

def on_feedback(fb: MotorFeedback) -> None:
    print(f'vel={fb.velocity:+.3f} rad/s  torque={fb.torque:+.3f} Nm  temp={fb.temperature:.1f} °C')

with CyberGearMotor(can_id=0x01, poll_interval=0.1) as motor:
    motor.add_feedback_listener(on_feedback)
    motor.run_mode = 'speed'
    motor.enable()

    motor.limit_cur = 5.0
    motor.spd_ref = 5.0
    time.sleep(3.0)

    motor.spd_ref = 0.0
    time.sleep(1.0)
    motor.disable()
```

---

## MIT operation mode — impedance control

```python
from cybergear import CyberGearMotor
import time

with CyberGearMotor(can_id=0x01) as motor:
    motor.run_mode = 'operation'
    motor.enable()
    time.sleep(0.3)

    start = motor.mech_pos or 0.0
    target = start + 1.0

    # Drive to target at 100 Hz
    for _ in range(200):
        motor.motor_control(
            torque=0.0,
            position=target,
            velocity=0.0,
            kp=10.0,
            kd=1.0,
        )
        time.sleep(0.01)

    motor.disable()
```

---

## Fault monitoring

```python
from cybergear import CyberGearMotor, FaultState

def on_fault(state: FaultState) -> None:
    if state.has_fault:
        print('--- FAULT ---')
        if not state.calibrated:
            print('  Motor not calibrated')
        if state.over_temperature:
            print('  Over temperature')
        if state.over_current:
            print('  Over current')
        if state.undervoltage:
            print('  Under voltage')
    else:
        print('Fault cleared')

with CyberGearMotor(can_id=0x01) as motor:
    motor.add_fault_listener(on_fault)
    motor.enable()

    # ... do work ...

    motor.disable()
```

---

## Live parameter table (Rich terminal UI)

The `examples/table.py` script displays all motor parameters in a live Rich
table in the terminal:

```bash
python examples/table.py
```

![Motor values table](imgs/motor_values_table.png)
