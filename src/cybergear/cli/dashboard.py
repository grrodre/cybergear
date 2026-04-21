"""CyberGear motor dashboard."""

from __future__ import annotations

import argparse
import sys

try:
    from textual.app import App, ComposeResult
    from textual.containers import Horizontal, Vertical
    from textual.widgets import Button, Footer, Header, Label
except ImportError:
    print(
        'Error: dashboard requires the dashboard extra.\n'
        '  pip install python-cybergear[dashboard]\n'
        '  uv add python-cybergear[dashboard]',
        file=sys.stderr,
    )
    sys.exit(1)

from cybergear import CyberGearMotor, MotorFeedback


def _row(label: str, value: str, unit: str = '', *, is_number: bool = False) -> str:
    """Rich markup for a key/value row."""
    val_color = '$accent' if is_number else '$secondary'
    val_markup = '[$text-muted]—[/]' if value == '—' else f'[{val_color}]{value}[/]'
    unit_markup = f' [$text-muted]{unit}[/]' if unit else ''
    return f'[$text-muted]{label}[/]  {val_markup}{unit_markup}'


class MotorInfoBox(Horizontal):
    """Hero bar: key motor identity fields displayed inline across the top."""

    BORDER_TITLE = 'Motor'

    _TABLE_FIELDS = [
        ('app_code_version', 'Firmware  '),
        ('app_build_date', 'Build date'),
    ]

    def __init__(self, motor: CyberGearMotor) -> None:
        super().__init__()
        self._motor = motor

    def compose(self) -> ComposeResult:
        yield Label(_row('CAN ID   ', str(self._motor.can_id), is_number=True))
        yield Label(_row('Device ID', self._motor.device_id or '—'))
        for key, label in self._TABLE_FIELDS:
            yield Label(_row(label, '—'), id=f'info-{key}')

    def load(self) -> None:
        t = self._motor._parameters_table  # noqa: SLF001
        for key, label in self._TABLE_FIELDS:
            value = str(t.get(key)) if t.get(key) else '—'
            self.query_one(f'#info-{key}', Label).update(_row(label, value))


class FeedbackBox(Vertical):
    BORDER_TITLE = 'Feedback'

    MODES = {0: 'Reset', 1: 'Calibration', 2: 'Run'}

    def compose(self) -> ComposeResult:
        yield Label(_row('Position   ', '—', 'rad', is_number=True), id='fb-position')
        yield Label(_row('Velocity   ', '—', 'rad/s', is_number=True), id='fb-velocity')
        yield Label(_row('Torque     ', '—', 'Nm', is_number=True), id='fb-torque')
        yield Label(_row('Temperature', '—', '°C', is_number=True), id='fb-temperature')
        yield Label(_row('Mode       ', '—'), id='fb-mode')
        yield Label(_row('Faults     ', '—'), id='fb-faults')

    def update_feedback(self, fb: MotorFeedback) -> None:
        mode = self.MODES.get(fb.mode, str(fb.mode))
        faults = fb.faults.has_fault

        self.query_one('#fb-position', Label).update(
            _row('Position   ', f'{fb.position:+.4f}', 'rad', is_number=True)
        )
        self.query_one('#fb-velocity', Label).update(
            _row('Velocity   ', f'{fb.velocity:+.4f}', 'rad/s', is_number=True)
        )
        self.query_one('#fb-torque', Label).update(
            _row('Torque     ', f'{fb.torque:+.4f}', 'Nm', is_number=True)
        )
        self.query_one('#fb-temperature', Label).update(
            _row('Temperature', f'{fb.temperature:.1f}', '°C', is_number=True)
        )
        self.query_one('#fb-mode', Label).update(_row('Mode       ', mode))

        fault_markup = '[$error]FAULT[/]' if faults else '[$success]OK[/]'
        self.query_one('#fb-faults', Label).update(
            f'[$text-muted]Faults     [/]  {fault_markup}'
        )


class ParametersBox(Vertical):
    BORDER_TITLE = 'Parameters'

    _FIELDS = [
        ('v_bus', 'Bus voltage', 'V'),
        ('mech_pos', 'Mech pos   ', 'rad'),
        ('mech_vel', 'Mech vel   ', 'rad/s'),
        ('iqf', 'Iq current ', 'A'),
        ('rotation', 'Rot (in)   ', ''),
    ]

    def __init__(self, motor: CyberGearMotor) -> None:
        super().__init__()
        self._motor = motor

    def compose(self) -> ComposeResult:
        for key, label, unit in self._FIELDS:
            yield Label(_row(label, '—', unit, is_number=True), id=f'param-{key}')
        yield Label(_row('Rot (out)  ', '—'), id='param-rotation-out')

    def update_param(self, name: str, value: float | int) -> None:
        for key, label, unit in self._FIELDS:
            if key == name:
                self.query_one(f'#param-{key}', Label).update(
                    _row(
                        label,
                        f'{value:.4f}' if isinstance(value, float) else str(value),
                        unit,
                        is_number=True,
                    )
                )
                break

        if name == 'rotation':
            gear_ratio = self._motor._parameters_table.get('gear_ratio')  # noqa: SLF001
            if gear_ratio:
                out = value / float(gear_ratio)
                self.query_one('#param-rotation-out', Label).update(
                    _row('Rot (out)  ', f'{out:.2f}', '', is_number=True)
                )
            else:
                self.query_one('#param-rotation-out', Label).update(
                    _row('Rot (out)  ', '—')
                )


class QuickControlBox(Vertical):
    BORDER_TITLE = 'Quick Control'

    def __init__(self, motor: CyberGearMotor) -> None:
        super().__init__()
        self._motor = motor

    def compose(self) -> ComposeResult:
        with Horizontal(id='quick-move-row'):
            yield Button('◀ -2π rad/s', id='btn-move-neg', variant='warning')
            yield Button('+2π rad/s ▶', id='btn-move-pos', variant='success')
        yield Button('Stop', id='btn-stop', variant='error')

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == 'btn-move-pos':
            self._motor.quick_move(6.2832)  # 2*pi
        elif event.button.id == 'btn-move-neg':
            self._motor.quick_move(-6.2832)  # -2*pi
        elif event.button.id == 'btn-stop':
            self._motor.quick_stop()


class CyberGearDashboard(App):
    TITLE = 'CyberGear Dashboard'
    BINDINGS = [('q', 'quit', 'Quit')]

    CSS = """
    MotorInfoBox   { width: 1fr; height: auto; border: round $primary; padding: 0 2; margin: 1 1 0 1; }
    MotorInfoBox Label { margin-right: 3; }
    #columns       { height: 1fr; margin: 0 1 1 1; padding: 0; }
    FeedbackBox    { width: 1fr; height: 1fr; border: round $primary; padding: 1 2; margin-right: 1; }
    ParametersBox  { width: 1fr; height: 1fr; border: round $primary; padding: 1 2; margin-right: 1; }
    QuickControlBox{ width: 1fr; height: 1fr; border: round $primary; padding: 1 2; }
    #quick-move-row  { height: auto; width: 1fr; margin-bottom: 1; }
    #btn-move-neg { width: 1fr; min-width: 1; margin-right: 1; }
    #btn-move-pos { width: 1fr; min-width: 1; }
    #btn-stop { width: 1fr; }
    """

    def __init__(self, motor: CyberGearMotor) -> None:
        super().__init__()
        self._motor = motor

    def compose(self) -> ComposeResult:
        yield Header()
        yield MotorInfoBox(self._motor)
        with Horizontal(id='columns'):
            yield FeedbackBox()
            yield ParametersBox(self._motor)
            yield QuickControlBox(self._motor)
        yield Footer()

    def on_mount(self) -> None:
        self._motor.add_feedback_listener(self._on_feedback)
        self._motor.add_parameter_listener(self._on_parameter)
        self._motor.fetch_parameter_table()
        self.set_timer(2.0, self.query_one(MotorInfoBox).load)

    def _on_feedback(self, fb: MotorFeedback) -> None:
        self.call_from_thread(self.query_one(FeedbackBox).update_feedback, fb)

    def _on_parameter(self, name: str, value: float | int) -> None:
        self.call_from_thread(self.query_one(ParametersBox).update_param, name, value)

    def on_unmount(self) -> None:
        self._motor.remove_feedback_listener(self._on_feedback)
        self._motor.remove_parameter_listener(self._on_parameter)


def main() -> None:
    parser = argparse.ArgumentParser(
        prog='cybergear-dashboard',
        description='Live CyberGear motor dashboard.',
    )
    parser.add_argument(
        '--interface', '-i', help='python-can interface (e.g. socketcan)'
    )
    parser.add_argument('--channel', '-c', help='CAN channel (e.g. can0)')
    parser.add_argument('--bitrate', '-b', type=int, help='CAN bitrate (e.g. 1000000)')
    parser.add_argument(
        '--can-id', type=int, help='Motor CAN ID (auto-detected if omitted)'
    )
    args = parser.parse_args()

    bus_config: dict | None = None
    if args.interface or args.channel:
        bus_config = {}
        if args.interface:
            bus_config['interface'] = args.interface
        if args.channel:
            bus_config['channel'] = args.channel
        if args.bitrate:
            bus_config['bitrate'] = args.bitrate

    try:
        motor = CyberGearMotor(bus_config=bus_config, can_id=args.can_id)
    except Exception as e:
        print(f'Error connecting to motor: {e}', file=sys.stderr)
        sys.exit(1)

    with motor:
        CyberGearDashboard(motor).run()
