from __future__ import annotations

import argparse
import sys

from .commands import build_query_command, build_usb_command, list_commands
from .protocol import bytes_to_hex, parse_hex
from .serial_console import SerialShell, list_serial_ports, monitor_serial, poll_serial, send_and_read
from .usart_remote import build_remote_frame


def _add_serial_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", required=True, help="Serial port, for example COM7.")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate. USB CDC can keep the default.")


def cmd_ports(_args: argparse.Namespace) -> int:
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return 0
    for port in ports:
        print(port)
    return 0


def cmd_commands(_args: argparse.Namespace) -> int:
    for command in list_commands():
        payload = "4float" if command.needs_float_payload else ("optional" if command.optional_float_payload else "empty")
        print(f"{command.name:<22} 0x{command.cmd:02X} {payload:<7} {command.doc}")
    return 0


def cmd_pack(args: argparse.Namespace) -> int:
    frame = build_usb_command(args.name, args.values)
    print(bytes_to_hex(frame))
    return 0


def cmd_send(args: argparse.Namespace) -> int:
    frame = parse_hex(args.raw) if args.raw else build_usb_command(args.name, args.values)
    send_and_read(args.port, args.baud, frame, wait_s=args.wait, json_lines=args.jsonl)
    return 0


def cmd_monitor(args: argparse.Namespace) -> int:
    monitor_serial(args.port, args.baud, json_lines=args.jsonl, raw_rx=args.raw_rx)
    return 0


def cmd_poll(args: argparse.Namespace) -> int:
    poll_serial(args.port, args.baud, args.name, args.rate, count=args.count, json_lines=args.jsonl)
    return 0


def cmd_shell(args: argparse.Namespace) -> int:
    SerialShell(args.port, args.baud).run()
    return 0


def _remote_kwargs(args: argparse.Namespace):
    return {
        "mode": args.mode,
        "arm_enable": args.arm_enable,
        "source_usb": args.source_usb,
        "tool": args.tool,
        "tool_action": args.tool_action,
        "clamp_action": args.clamp_action,
        "chuck_action": args.chuck_action,
        "climb_enable": args.climb_enable,
        "climb_step": args.climb_step,
        "climb_auto": args.climb_auto,
        "chassis": args.chassis,
        "arm_target": args.arm_target,
    }


def cmd_remote_pack(args: argparse.Namespace) -> int:
    frame = build_remote_frame(**_remote_kwargs(args))
    print(bytes_to_hex(frame))
    return 0


def cmd_remote_send(args: argparse.Namespace) -> int:
    frame = build_remote_frame(**_remote_kwargs(args))
    send_and_read(args.port, args.baud, frame, wait_s=args.wait, json_lines=args.jsonl)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot USB serial protocol tool.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("ports", help="List serial ports.")
    p.set_defaults(func=cmd_ports)

    p = sub.add_parser("commands", help="List known USB commands.")
    p.set_defaults(func=cmd_commands)

    p = sub.add_parser("pack", help="Pack a USB frame and print hex.")
    p.add_argument("name", help="Command name or numeric command id.")
    p.add_argument("values", nargs="*", type=float, help="Optional float payload values, padded to 4 floats.")
    p.set_defaults(func=cmd_pack)

    p = sub.add_parser("send", help="Send one USB command or raw hex frame.")
    _add_serial_args(p)
    p.add_argument("name", nargs="?", help="Command name or numeric command id.")
    p.add_argument("values", nargs="*", type=float)
    p.add_argument("--raw", help="Raw hex bytes to send instead of command name.")
    p.add_argument("--wait", type=float, default=0.0, help="Seconds to read replies after sending.")
    p.add_argument("--jsonl", action="store_true", help="Print one JSON object per line.")
    p.set_defaults(func=cmd_send)

    p = sub.add_parser("monitor", help="Read and decode incoming USB frames.")
    _add_serial_args(p)
    p.add_argument("--jsonl", action="store_true")
    p.add_argument("--raw-rx", action="store_true", help="Also print raw RX chunks.")
    p.set_defaults(func=cmd_monitor)

    p = sub.add_parser("poll", help="Periodically send a status query and decode replies.")
    _add_serial_args(p)
    p.add_argument("name", help="Status target, for example ROBOT, CLIMB, SYS.")
    p.add_argument("--rate", type=float, default=10.0, help="Polling rate in Hz.")
    p.add_argument("--count", type=int, default=0, help="Stop after count sends; 0 means forever.")
    p.add_argument("--jsonl", action="store_true")
    p.set_defaults(func=cmd_poll)

    p = sub.add_parser("shell", help="Interactive send/read shell.")
    _add_serial_args(p)
    p.set_defaults(func=cmd_shell)

    for name, func, needs_serial in (
        ("remote-pack", cmd_remote_pack, False),
        ("remote-send", cmd_remote_send, True),
    ):
        p = sub.add_parser(name, help="Build/send old USART remote control frame.")
        if needs_serial:
            _add_serial_args(p)
            p.add_argument("--wait", type=float, default=0.0)
            p.add_argument("--jsonl", action="store_true")
        p.add_argument("--mode", type=int, choices=range(8), help="Chassis mode 0..7. Omit for all-zero stop mode.")
        p.add_argument("--arm-enable", type=int, choices=[0, 1], default=0)
        p.add_argument("--source-usart", dest="source_usb", action="store_const", const=0, default=0)
        p.add_argument("--source-usb", dest="source_usb", action="store_const", const=1)
        p.add_argument("--tool", type=int, choices=[0, 1], default=0, help="0 clamp position, 1 chuck position.")
        p.add_argument("--tool-action", type=int, choices=[0, 1], help="Legacy alias for --clamp-action.")
        p.add_argument("--clamp-action", type=int, choices=[0, 1], default=0, help="0 clamp close, 1 clamp open.")
        p.add_argument("--chuck-action", type=int, choices=[0, 1], default=0, help="0 chuck close, 1 chuck open.")
        p.add_argument("--climb-enable", type=int, choices=[0, 1], default=0)
        p.add_argument("--climb-step", type=int, choices=[0, 1], default=0)
        p.add_argument("--climb-auto", type=int, choices=[0, 1], default=0)
        p.add_argument("--chassis", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("P1", "P2", "P3"))
        p.add_argument("--arm-target", nargs=3, type=float, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "Z"))
        p.set_defaults(func=func)

    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if getattr(args, "raw", None) is None and getattr(args, "cmd", "") == "send" and not args.name:
        parser.error("send requires NAME or --raw")
    try:
        return args.func(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
