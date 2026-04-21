# Getting started

## Installation

```bash
pip install python-cybergear
```

Or with [uv](https://github.com/astral-sh/uv):

```bash
uv add python-cybergear
```

## CAN interface setup (Linux)

Bring up the CAN interface at 1 Mbit/s before using the library:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

To make it persistent across reboots, add a systemd-networkd config or an
`/etc/network/interfaces` entry for `can0`.

## CAN configuration file

The driver reads CAN interface settings from `~/.can` automatically.
Create it once and you never need to pass `bus_config=` in your code:

```ini title="~/.can"
[default]
interface = socketcan
channel = can0
bitrate = 1000000
```

A template is included in the repo at `.can.example`.

You can still pass `bus_config=` explicitly when you need to override it
(e.g. multiple buses, tests):

```python
motor = CyberGearMotor(bus_config={'interface': 'socketcan', 'channel': 'can1'})
```

## Connecting to a motor

```python
from cybergear import CyberGearMotor

# CAN config from ~/.can, motor CAN ID = 0x01
with CyberGearMotor(can_id=0x01) as motor:
    motor.enable()

    fb = motor.feedback
    print(f'temp={fb.temperature:.1f} °C  pos={fb.position:.4f} rad')

    motor.disable()
```

`can_id` must match the motor's configured CAN node ID (factory default: `0x7F`).

## Auto-detecting the motor CAN ID

If you don't know the motor's ID, omit `can_id` — the driver scans IDs 1–127
and raises if zero or multiple motors respond:

```python
with CyberGearMotor() as motor:
    print(f'Found motor at CAN ID {motor.can_id}')
```

To scan without creating a motor instance:

```python
found = CyberGearMotor.scan()
# [(1, b'\x01\x02...'), ...]  — list of (can_id, device_id_bytes)
print(found)
```
