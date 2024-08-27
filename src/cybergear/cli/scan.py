"""CLI entry point for scanning the CAN bus for CyberGear motors."""

import argparse
import sys


def main() -> None:
    parser = argparse.ArgumentParser(
        prog='cybergear-scan',
        description='Scan a CAN bus for connected CyberGear motors.',
    )
    parser.add_argument('--interface', '-i', help='python-can interface (e.g. socketcan)')
    parser.add_argument('--channel', '-c', help='CAN channel (e.g. can0)')
    parser.add_argument('--bitrate', '-b', type=int, help='CAN bitrate (e.g. 1000000)')
    parser.add_argument('--host-id', type=lambda x: int(x, 0), default=0xFD, help='Host CAN ID (default: 0xFD)')
    parser.add_argument('--timeout', type=float, default=0.5, help='Scan timeout in seconds (default: 0.5)')
    parser.add_argument('--send-timeout', type=float, default=0.1, help='TX buffer timeout per frame in seconds (default: 0.1)')
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

    from cybergear import CyberGearMotor

    print(f'Scanning CAN bus (timeout={args.timeout}s)...')
    try:
        motors = CyberGearMotor.scan(
            bus_config=bus_config,
            host_can_id=args.host_id,
            timeout=args.timeout,
            send_timeout=args.send_timeout,
        )
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)

    if not motors:
        print('No motors found.')
        sys.exit(0)

    print(f'\nFound {len(motors)} motor(s):\n')
    print(f'  {"CAN ID":<10} {"MCU Identifier"}')
    print(f'  {"-" * 10} {"-" * 20}')
    for can_id, device_id in motors:
        print(f'  {can_id:<10} {device_id.hex()}')
