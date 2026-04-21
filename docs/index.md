# cybergear

[![PyPI version](https://img.shields.io/pypi/v/python-cybergear.svg)](https://pypi.org/project/python-cybergear/)
[![Python versions](https://img.shields.io/pypi/pyversions/python-cybergear.svg)](https://pypi.org/project/python-cybergear/)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](https://github.com/grrodre/cybergear/blob/main/LICENSE)

Python driver for the [Xiaomi CyberGear](https://www.mi.com/cyber-gear) brushless motor over CAN bus.

![CyberGear](imgs/cybergear_motors.jpg)

Built on top of [python-can](https://python-can.readthedocs.io/). Tested with SocketCAN on Linux; custom polling threads are used instead of BCM to support other python-can interfaces as well.

---

## Features

- **Five control modes**: operation (MIT-style), position, speed, current, and quick-move
- **Auto-scan**: discovers motors on the bus without needing to specify a CAN ID
- **Async feedback**: `can.Notifier`-based listener delivers `MotorFeedback` snapshots without blocking
- **Background polling**: portable parameter polling thread keeps properties up to date on any interface
- **Event callbacks**: subscribe to feedback, parameter updates, and fault state changes
- **Context-manager API**: guarantees clean resource release
- **Fault detection**: decodes and monitors over-temperature, over-current, undervoltage, and encoder faults

---

## Quick install

```bash
pip install python-cybergear
# or
uv add python-cybergear
```

With the dashboard (requires [Textual](https://textual.textualize.io/) and Rich):

```bash
pip install python-cybergear[dashboard]
# or
uv add python-cybergear[dashboard]
```

Then read [Getting started](getting-started.md) to connect your first motor.

---

## Hardware

Tested with a [CANable USB adapter](https://es.aliexpress.com/item/1005006032351087.html) and an [XT30(2+2) cable](https://de.aliexpress.com/item/1005006046478152.html).

![Canable](imgs/canable_usb.png)
![XT30 cable](imgs/cable_xt30.png)
