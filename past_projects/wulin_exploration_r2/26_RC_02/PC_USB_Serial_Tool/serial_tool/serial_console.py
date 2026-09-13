from __future__ import annotations

import json
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from .commands import build_query_command, build_usb_command, list_commands
from .protocol import UsbFrame, UsbStreamParser, bytes_to_hex, parse_hex, pack_4float_frame
from .status import CLIMB_TEST_ACTIONS, decode_usb_frame


CLIMB_DONE_STATE = 22
CLIMB_ERROR_STATE = 23
DEFAULT_CLIMB_WAIT_TIMEOUT_S = 40.0
DEFAULT_CLIMB_FINAL_TIMEOUT_S = 180.0
DEFAULT_CLIMB_POLL_INTERVAL_S = 0.1
CLIMB_COMMAND_SETTLE_S = 0.05
FLOW_AUTOSAVE_FILE = ".climb_flow_autosave.json"
CLIMB_TEST_ACTION_BY_NAME = {name: action for action, name in CLIMB_TEST_ACTIONS.items()}
CLIMB_TEST_SHORTCUTS = {
    f"climb_{name.lower()}": action
    for action, name in CLIMB_TEST_ACTIONS.items()
    if action != 0
}


def list_serial_ports() -> List[str]:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise RuntimeError("pyserial is not installed. Run: python -m pip install -r requirements.txt") from exc

    ports = []
    for port in list_ports.comports():
        desc = port.description or ""
        hwid = port.hwid or ""
        ports.append(f"{port.device}\t{desc}\t{hwid}")
    return ports


def open_serial(port: str, baud: int, timeout: float = 0.05):
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError("pyserial is not installed. Run: python -m pip install -r requirements.txt") from exc

    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def print_decoded_frame(frame, json_lines: bool = False) -> None:
    decoded = decode_usb_frame(frame)
    if json_lines:
        print(json.dumps(decoded, ensure_ascii=False, separators=(",", ":")), flush=True)
    else:
        print(json.dumps(decoded, ensure_ascii=False, indent=2), flush=True)


def monitor_serial(port: str, baud: int, json_lines: bool = False, raw_rx: bool = False) -> None:
    parser = UsbStreamParser()
    with open_serial(port, baud) as ser:
        print(f"Opened {ser.port} @ {baud}. Press Ctrl+C to stop.", flush=True)
        try:
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                if raw_rx:
                    print(f"RX {bytes_to_hex(chunk)}", flush=True)
                for frame in parser.feed(chunk):
                    print_decoded_frame(frame, json_lines=json_lines)
        except KeyboardInterrupt:
            print("\nStopped.", flush=True)


def send_and_read(port: str, baud: int, frame: bytes, wait_s: float = 0.0, json_lines: bool = False) -> None:
    parser = UsbStreamParser()
    with open_serial(port, baud) as ser:
        ser.write(frame)
        ser.flush()
        print(f"TX {bytes_to_hex(frame)}", flush=True)

        end = time.monotonic() + max(0.0, wait_s)
        while time.monotonic() < end:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                for parsed in parser.feed(chunk):
                    print_decoded_frame(parsed, json_lines=json_lines)


def poll_serial(port: str, baud: int, command: str, rate: float, count: int = 0, json_lines: bool = False) -> None:
    if rate <= 0:
        raise ValueError("rate must be > 0")

    frame = build_query_command(command)
    interval = 1.0 / rate
    parser = UsbStreamParser()
    sent = 0

    with open_serial(port, baud) as ser:
        print(f"Polling {command} at {rate:g} Hz on {ser.port}. Press Ctrl+C to stop.", flush=True)
        next_send = time.monotonic()
        try:
            while count <= 0 or sent < count:
                now = time.monotonic()
                if now >= next_send:
                    ser.write(frame)
                    ser.flush()
                    sent += 1
                    next_send = now + interval

                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    for parsed in parser.feed(chunk):
                        print_decoded_frame(parsed, json_lines=json_lines)
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\nStopped.", flush=True)


class SerialShell:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.parser = UsbStreamParser()
        self.stop_event = threading.Event()
        self.ser = None
        self.reader_thread: Optional[threading.Thread] = None
        self.frame_queue: "queue.Queue[UsbFrame]" = queue.Queue()
        self.flow_name: Optional[str] = None
        self.flow_started_at: Optional[str] = None
        self.flow_steps: List[Dict[str, Any]] = []
        self.last_flow_candidate: Optional[Dict[str, Any]] = None
        self.flow_autosave_path = Path(FLOW_AUTOSAVE_FILE)

    def run(self) -> None:
        self.ser = open_serial(self.port, self.baud)
        print(f"Opened {self.ser.port} @ {self.baud}. Type 'help' for commands.", flush=True)
        self._print_flow_autosave_hint()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        try:
            while not self.stop_event.is_set():
                try:
                    line = input("> ")
                except EOFError:
                    break
                if not self._handle_line(line):
                    break
        finally:
            self.stop_event.set()
            if self.reader_thread:
                self.reader_thread.join(timeout=0.5)
            if self.ser:
                self.ser.close()

    def _reader_loop(self) -> None:
        assert self.ser is not None
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:  # pragma: no cover - serial hardware path
                print(f"\nSerial read error: {exc}", file=sys.stderr, flush=True)
                self.stop_event.set()
                return
            if not chunk:
                continue
            for frame in self.parser.feed(chunk):
                self.frame_queue.put(frame)
                print()
                print_decoded_frame(frame)
                print("> ", end="", flush=True)

    def _write(self, frame: bytes) -> None:
        assert self.ser is not None
        self.ser.write(frame)
        self.ser.flush()
        print(f"TX {bytes_to_hex(frame)}", flush=True)

    def _write_sequence(self, frames: Iterable[bytes], delay_s: float = 0.02) -> None:
        for frame in frames:
            self._write(frame)
            time.sleep(delay_s)

    def _handle_line(self, line: str) -> bool:
        parts = line.strip().split()
        if not parts:
            return True

        op = parts[0].lower()
        args = parts[1:]

        try:
            if op in {"exit", "quit"}:
                return False
            if op == "help":
                self._print_help()
            elif op == "commands":
                self._print_commands()
            elif op == "flow_start":
                self._flow_start(args)
            elif op == "flow_save":
                self._flow_save(args)
            elif op in {"flow_confirm", "flow_ask_save"}:
                self._flow_confirm(args)
            elif op == "flow_add":
                self._flow_add(args)
            elif op == "flow_show":
                self._flow_show()
            elif op == "flow_export":
                self._flow_export(args)
            elif op == "flow_clear":
                self._flow_clear()
            elif op == "flow_recover":
                self._flow_recover(args)
            elif op == "flow_autosave":
                self._flow_autosave_command(args)
            elif op == "pack":
                self._pack(args)
            elif op == "send":
                self._send_named(args)
            elif op == "query":
                self._query(args)
            elif op in {"raw", "hex"}:
                self._write(parse_hex(" ".join(args)))
            elif op == "usb":
                self._write(build_usb_command("SYS_SWITCH_SOURCE", [1.0]))
            elif op == "usart":
                self._write(build_usb_command("SYS_SWITCH_SOURCE", [0.0]))
            elif op == "enable":
                self._write(build_usb_command("SYS_ENABLE"))
            elif op == "disable":
                self._write(build_usb_command("SYS_DISABLE"))
            elif op == "stop":
                self._write(build_usb_command("SYS_STOP"))
            elif op == "status":
                self._write(build_usb_command("ROBOT_GET_STATUS"))
            elif op == "chs_enable":
                self._write(build_usb_command("CHS_ENABLE"))
            elif op == "chs_disable":
                self._write(build_usb_command("CHS_DISABLE"))
            elif op == "chs_stop":
                self._write(build_usb_command("CHS_STOP"))
            elif op == "chs_status":
                self._write(build_usb_command("CHS_GET_STATUS"))
            elif op == "chs_mode":
                self._write(build_usb_command("CHS_SET_MODE", self._need_floats(args, 1, "chs_mode MODE")))
            elif op == "vel":
                values = self._need_floats(args, 3, "vel VX VY YAW_DATA")
                if len(values) < 4:
                    values.append(0.0)
                self._write(build_usb_command("CHS_SET_VEL", values))
            elif op == "pos":
                self._write(build_usb_command("CHS_SET_POS", self._need_floats(args, 3, "pos DX DY YAW_DATA")))
            elif op == "arm_enable":
                self._write(build_usb_command("ARM_ENABLE"))
            elif op == "arm_disable":
                self._write(build_usb_command("ARM_DISABLE"))
            elif op == "arm_stop":
                self._write(build_usb_command("ARM_STOP"))
            elif op == "arm_status":
                self._write(build_usb_command("ARM_GET_STATUS"))
            elif op == "arm_target":
                self._write(build_usb_command("ARM_SET_TARGET", self._need_floats(args, 3, "arm_target X Y Z")))
            elif op == "tool_enable":
                self._write(build_usb_command("TOOL_ENABLE"))
            elif op == "tool_disable":
                self._write(build_usb_command("TOOL_DISABLE"))
            elif op == "tool_stop":
                self._write(build_usb_command("TOOL_STOP"))
            elif op == "tool_status":
                self._write(build_usb_command("TOOL_GET_STATUS"))
            elif op == "tool_chuck":
                self._write(build_usb_command("TOOL_SET_MODE", [1.0]))
            elif op == "tool_clamp":
                self._write(build_usb_command("TOOL_SET_MODE", [0.0]))
            elif op == "tool_close":
                self._write(build_usb_command("TOOL_ACTION", [0.0]))
            elif op == "tool_open":
                self._write(build_usb_command("TOOL_ACTION", [1.0]))
            elif op == "tool_state":
                self._write(build_usb_command("TOOL_SET_STATE", self._need_floats(args, 2, "tool_state DEV ACT")))
            elif op == "chuck_open":
                self._write_sequence([build_usb_command("TOOL_ENABLE"), build_usb_command("TOOL_SET_STATE", [1.0, 1.0]), build_usb_command("TOOL_GET_STATUS")])
            elif op == "chuck_close":
                self._write_sequence([build_usb_command("TOOL_ENABLE"), build_usb_command("TOOL_SET_STATE", [1.0, 0.0]), build_usb_command("TOOL_GET_STATUS")])
            elif op == "clamp_open":
                self._write_sequence([build_usb_command("TOOL_ENABLE"), build_usb_command("TOOL_SET_STATE", [0.0, 1.0]), build_usb_command("TOOL_GET_STATUS")])
            elif op == "clamp_close":
                self._write_sequence([build_usb_command("TOOL_ENABLE"), build_usb_command("TOOL_SET_STATE", [0.0, 0.0]), build_usb_command("TOOL_GET_STATUS")])
            elif op == "tune_start":
                pass_count = float(args[0]) if args else 1.0
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("SYS_ENABLE"),
                        build_usb_command("CHS_ENABLE"),
                        build_usb_command("YAW_TUNE_START", [pass_count]),
                        build_usb_command("YAW_TUNE_GET_STATUS"),
                    ]
                )
            elif op == "tune_status":
                self._write(build_usb_command("YAW_TUNE_GET_STATUS"))
            elif op == "tune_stop":
                self._write_sequence(
                    [
                        build_usb_command("YAW_TUNE_STOP"),
                        build_usb_command("YAW_TUNE_GET_STATUS"),
                    ]
                )
            elif op in {"climb_auto", "climb_run", "climb_up_auto", "climb_up_run", "climb_upstairs_auto", "climb_upstairs_run"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_AUTO"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_auto_wait", "climb_run_wait", "climb_up_auto_wait", "climb_up_run_wait", "climb_upstairs_auto_wait", "climb_upstairs_run_wait"}:
                timeout_s = self._optional_timeout(args, DEFAULT_CLIMB_FINAL_TIMEOUT_S)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_AUTO"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=True)
            elif op in {"climb_down_auto", "climb_down_run", "climb_downstairs_auto", "climb_downstairs_run"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_AUTO"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_down_auto_wait", "climb_down_run_wait", "climb_downstairs_auto_wait", "climb_downstairs_run_wait"}:
                timeout_s = self._optional_timeout(args, DEFAULT_CLIMB_FINAL_TIMEOUT_S)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_AUTO"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=True)
            elif op in {"climb_up_laser_gate", "climb_up_gate"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_GATE"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_down_laser_gate", "climb_down_gate"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_GATE"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_up_auto_pause", "climb_up_pause", "climb_upstairs_auto_pause"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_AUTO_PAUSE"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_down_auto_pause", "climb_down_pause", "climb_downstairs_auto_pause"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_AUTO_PAUSE"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_auto_resume", "climb_resume"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_AUTO_RESUME"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_step", "climb_upstairs_step"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_STEP"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_down_step", "climb_downstairs_step"}:
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_STEP"),
                        build_usb_command("CLIMB_GET_STATUS"),
                    ]
                )
            elif op in {"climb_step_wait", "climb_next", "climb_up_step_wait", "climb_up_next", "climb_upstairs_step_wait", "climb_upstairs_next"}:
                timeout_s = self._optional_timeout(args)
                self._wait_for_climb(timeout_s, final_state=False)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_UP_STEP"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=False)
            elif op in {"climb_down_step_wait", "climb_down_next", "climb_downstairs_step_wait", "climb_downstairs_next"}:
                timeout_s = self._optional_timeout(args)
                self._wait_for_climb(timeout_s, final_state=False)
                self._write_sequence(
                    [
                        build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
                        build_usb_command("CLIMB_ENABLE"),
                        build_usb_command("CLIMB_DOWN_STEP"),
                    ]
                )
                time.sleep(CLIMB_COMMAND_SETTLE_S)
                self._wait_for_climb(timeout_s, final_state=False)
            elif op == "climb_wait":
                self._wait_for_climb(self._optional_timeout(args), final_state=False)
            elif op == "climb_wait_then":
                if not args:
                    raise ValueError("usage: climb_wait_then NAME [float...]")
                self._wait_for_climb(DEFAULT_CLIMB_WAIT_TIMEOUT_S, final_state=False)
                self._send_named(args)
            elif op in {"climb_tests", "climb_test_list"}:
                self._print_climb_tests()
            elif op == "climb_test":
                self._climb_test(args, wait=False)
            elif op == "climb_test_wait":
                self._climb_test(args, wait=True)
            elif op in CLIMB_TEST_SHORTCUTS:
                timeout_s = self._optional_timeout(args) if args else DEFAULT_CLIMB_WAIT_TIMEOUT_S
                self._write_climb_test_action(CLIMB_TEST_SHORTCUTS[op], wait=bool(args), timeout_s=timeout_s)
            elif op == "climb_enable":
                self._write(build_usb_command("CLIMB_ENABLE"))
            elif op == "climb_disable":
                self._write(build_usb_command("CLIMB_DISABLE"))
            elif op == "climb_stop":
                self._write(build_usb_command("CLIMB_STOP"))
            elif op == "climb_status":
                self._write(build_usb_command("CLIMB_GET_STATUS"))
            elif op == "climb_ctrl":
                self._write(build_usb_command("CLIMB_SET_CTRL", self._need_floats(args, 3, "climb_ctrl ENABLE STEP AUTO")))
            elif "activate" in op and (".venv" in op or "scripts" in op):
                print("You are already inside the serial shell. Type 'exit' first, then run PowerShell commands.", flush=True)
            else:
                self._send_direct_or_unknown(parts)
        except Exception as exc:
            print(f"Error: {exc}", flush=True)
        return True

    def _pack(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: pack NAME [float...]")
        frame = build_usb_command(args[0], [float(x) for x in args[1:]])
        print(bytes_to_hex(frame), flush=True)

    def _send_named(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: send NAME [float...]")
        frame = build_usb_command(args[0], [float(x) for x in args[1:]])
        self._write(frame)

    def _send_direct_or_unknown(self, parts: List[str]) -> None:
        try:
            values = [float(x) for x in parts[1:]]
            frame = build_usb_command(parts[0], values)
        except ValueError:
            print(f"Unknown command: {parts[0].lower()}. Type 'help'.", flush=True)
            return
        self._write(frame)

    def _query(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: query ROBOT|TUNE|CLIMB|SYS|CHS|ARM|TOOL")
        self._write(build_query_command(args[0]))

    def _climb_test(self, args: List[str], wait: bool) -> None:
        if not args:
            raise ValueError("usage: climb_test ACTION | climb_test_wait ACTION [timeout_s]")
        if wait and len(args) > 2:
            raise ValueError("usage: climb_test_wait ACTION [timeout_s]")
        if not wait and len(args) > 1:
            raise ValueError("usage: climb_test ACTION")

        action = self._parse_climb_test_action(args[0])
        timeout_s = float(args[1]) if wait and len(args) == 2 else DEFAULT_CLIMB_WAIT_TIMEOUT_S
        self._write_climb_test_action(action, wait=wait, timeout_s=timeout_s)

    def _write_climb_test_action(
        self,
        action: int,
        wait: bool,
        timeout_s: float = DEFAULT_CLIMB_WAIT_TIMEOUT_S,
    ) -> Optional[Dict[str, Any]]:
        name = CLIMB_TEST_ACTIONS.get(action, "UNKNOWN")
        print(f"Climb test action: {action} {name}", flush=True)
        frames = [
            build_usb_command("SYS_SWITCH_SOURCE", [1.0]),
            build_usb_command("CLIMB_ENABLE"),
            build_usb_command("CLIMB_TEST_ACTION", [float(action)]),
        ]
        if not wait:
            frames.append(build_usb_command("CLIMB_GET_STATUS"))
        self._write_sequence(frames)
        result: Optional[Dict[str, Any]] = None
        if wait:
            time.sleep(CLIMB_COMMAND_SETTLE_S)
            result = self._wait_for_climb(timeout_s, final_state=False)
        self.last_flow_candidate = self._build_flow_step(action, wait, timeout_s, result)
        self._flow_autosave()
        print("Use flow_confirm [note] to choose whether to save this action.", flush=True)
        return result

    def _build_flow_step(
        self,
        action: int,
        wait: bool,
        timeout_s: float,
        result: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        name = CLIMB_TEST_ACTIONS.get(action, "UNKNOWN")
        step: Dict[str, Any] = {
            "command": "CLIMB_TEST_ACTION",
            "action_id": action,
            "action_name": name,
            "values": [float(action)],
            "wait": bool(wait),
            "timeout_s": float(timeout_s),
            "done_condition": "CLIMB_GET_STATUS.state_done == 1",
        }
        if result:
            step["result"] = self._summarize_climb_result(result)
        return step

    @staticmethod
    def _summarize_climb_result(payload: Dict[str, Any]) -> Dict[str, Any]:
        keep = (
            "state",
            "state_name",
            "state_done",
            "error_flags",
            "elapsed_ms",
            "test_action",
            "test_action_name",
            "leg_pos_mm",
            "leg_target_mm",
            "drive_pos_mm",
            "drive_target_mm",
        )
        return {key: payload[key] for key in keep if key in payload}

    def _flow_start(self, args: List[str]) -> None:
        self.flow_name = args[0] if args else time.strftime("climb_flow_%Y%m%d_%H%M%S")
        self.flow_started_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self.flow_steps = []
        self.last_flow_candidate = None
        print(f"Flow recording started: {self.flow_name}", flush=True)

    def _flow_save(self, args: List[str]) -> None:
        if self.last_flow_candidate is None:
            raise ValueError("no climb test action to save yet")
        if self.flow_name is None:
            self._flow_start([])

        step = json.loads(json.dumps(self.last_flow_candidate, ensure_ascii=False))
        if args:
            step["note"] = " ".join(args)
        step["index"] = len(self.flow_steps) + 1
        self.flow_steps.append(step)
        self._flow_autosave()
        print(f"Saved flow step {step['index']}: {step['action_id']} {step['action_name']}", flush=True)

    def _flow_confirm(self, args: List[str]) -> None:
        if self.last_flow_candidate is None:
            raise ValueError("no climb test action to confirm yet")

        candidate = self.last_flow_candidate
        note = " ".join(args)
        note_text = f" note={note}" if note else ""
        print(
            f"Candidate: {candidate['action_id']} {candidate['action_name']}{note_text}",
            flush=True,
        )
        answer = input("Save this action to the current flow? [y/N] ").strip().lower()
        if answer in {"y", "yes"}:
            self._flow_save(args)
        else:
            self.last_flow_candidate = None
            self._flow_autosave()
            print("Discarded last flow candidate.", flush=True)

    def _flow_add(self, args: List[str]) -> None:
        if not args:
            raise ValueError("usage: flow_add ACTION [timeout_s] [note...]")
        action = self._parse_climb_test_action(args[0])
        timeout_s = DEFAULT_CLIMB_WAIT_TIMEOUT_S
        note_start = 1
        if len(args) >= 2:
            try:
                timeout_s = float(args[1])
                note_start = 2
            except ValueError:
                note_start = 1
        self.last_flow_candidate = self._build_flow_step(action, True, timeout_s, None)
        self._flow_save(args[note_start:])

    def _flow_show(self) -> None:
        if not self.flow_steps:
            print("Flow is empty.", flush=True)
            return
        for step in self.flow_steps:
            note = f"  # {step['note']}" if step.get("note") else ""
            print(
                f"{step['index']:02d}. {step['action_id']} {step['action_name']} "
                f"wait={int(bool(step.get('wait')))} timeout={step.get('timeout_s')}s{note}",
                flush=True,
            )

    def _flow_data(self) -> Dict[str, Any]:
        return {
            "type": "r2_climb_debug_flow",
            "version": 1,
            "name": self.flow_name or "climb_flow",
            "created_at": self.flow_started_at,
            "step_count": len(self.flow_steps),
            "steps": self.flow_steps,
        }

    def _flow_autosave_data(self) -> Dict[str, Any]:
        data = self._flow_data()
        data["autosave"] = True
        data["autosaved_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        if self.last_flow_candidate is not None:
            data["last_flow_candidate"] = self.last_flow_candidate
        return data

    def _flow_autosave(self) -> None:
        if self.flow_name is None and not self.flow_steps and self.last_flow_candidate is None:
            return
        text = json.dumps(self._flow_autosave_data(), ensure_ascii=False, indent=2)
        with self.flow_autosave_path.open("w", encoding="utf-8") as f:
            f.write(text)
            f.write("\n")

    def _print_flow_autosave_hint(self) -> None:
        if not self.flow_autosave_path.exists():
            return
        try:
            data = json.loads(self.flow_autosave_path.read_text(encoding="utf-8"))
            step_count = int(data.get("step_count", 0))
            name = str(data.get("name", "climb_flow"))
        except Exception:
            step_count = -1
            name = "unknown"
        detail = f"{step_count} saved step(s)" if step_count >= 0 else "unreadable"
        print(
            f"Flow autosave found: {self.flow_autosave_path} ({name}, {detail}). "
            "Use flow_recover to restore it.",
            flush=True,
        )

    def _flow_recover(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: flow_recover [file.json]")
        path = Path(args[0]) if args else self.flow_autosave_path
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("type") != "r2_climb_debug_flow":
            raise ValueError(f"not an r2 climb flow file: {path}")

        steps = data.get("steps", [])
        if not isinstance(steps, list):
            raise ValueError(f"invalid flow steps in {path}")

        restored_steps = json.loads(json.dumps(steps, ensure_ascii=False))
        for index, step in enumerate(restored_steps, 1):
            if isinstance(step, dict):
                step["index"] = index

        candidate = data.get("last_flow_candidate")
        self.flow_name = str(data.get("name") or "climb_flow")
        self.flow_started_at = data.get("created_at")
        self.flow_steps = restored_steps
        self.last_flow_candidate = candidate if isinstance(candidate, dict) else None
        self._flow_autosave()
        print(
            f"Recovered flow '{self.flow_name}' from {path}: {len(self.flow_steps)} step(s).",
            flush=True,
        )
        if self.last_flow_candidate is not None:
            print(
                "Recovered one unsaved candidate. Use flow_confirm [note] or flow_save [note] to keep it.",
                flush=True,
            )

    def _flow_autosave_command(self, args: List[str]) -> None:
        if len(args) > 1:
            raise ValueError("usage: flow_autosave [file.json]")
        if args:
            self.flow_autosave_path = Path(args[0])
        self._flow_autosave()
        print(f"Flow autosave file: {self.flow_autosave_path}", flush=True)

    def _flow_export(self, args: List[str]) -> None:
        data = self._flow_data()
        text = json.dumps(data, ensure_ascii=False, indent=2)
        if args:
            path = args[0]
            with open(path, "w", encoding="utf-8") as f:
                f.write(text)
                f.write("\n")
            self._flow_autosave()
            print(f"Flow exported: {path}", flush=True)
        else:
            print(text, flush=True)

    def _flow_clear(self) -> None:
        self.flow_name = None
        self.flow_started_at = None
        self.flow_steps = []
        self.last_flow_candidate = None
        try:
            self.flow_autosave_path.unlink()
        except FileNotFoundError:
            pass
        print("Flow cleared.", flush=True)

    @staticmethod
    def _parse_climb_test_action(token: str) -> int:
        text = token.strip()
        try:
            value = int(text, 0)
        except ValueError:
            key = text.upper().replace("-", "_")
            for prefix in ("R2_CLIMB_TEST_", "CLIMB_TEST_", "TEST_"):
                if key.startswith(prefix):
                    key = key[len(prefix):]
                    break
            if key not in CLIMB_TEST_ACTION_BY_NAME:
                raise ValueError(f"unknown climb test action: {token}") from None
            value = CLIMB_TEST_ACTION_BY_NAME[key]

        if value not in CLIMB_TEST_ACTIONS:
            raise ValueError(f"unknown climb test action id: {value}")
        if value == 0:
            raise ValueError("action 0 is NONE; use climb_stop if you want to stop")
        return value

    @staticmethod
    def _print_climb_tests() -> None:
        for action, name in CLIMB_TEST_ACTIONS.items():
            print(f"{action:2d} {name}", flush=True)

    def _drain_frames(self) -> None:
        while True:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                return

    def _wait_for_climb(self, timeout_s: float, final_state: bool) -> Dict[str, Any]:
        if timeout_s <= 0:
            raise ValueError("timeout must be > 0")

        self._drain_frames()
        deadline = time.monotonic() + timeout_s
        next_query = 0.0
        last_payload: Optional[Dict[str, Any]] = None
        mode = "DONE" if final_state else "ready"
        print(f"Waiting for climb {mode} (timeout {timeout_s:g}s)...", flush=True)

        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_query:
                self._write(build_usb_command("CLIMB_GET_STATUS"))
                next_query = now + DEFAULT_CLIMB_POLL_INTERVAL_S

            try:
                frame = self.frame_queue.get(timeout=min(0.05, max(0.0, deadline - time.monotonic())))
            except queue.Empty:
                continue

            decoded = decode_usb_frame(frame)
            if decoded.get("cmd_id") != 0x56:
                continue
            payload = decoded.get("payload")
            if not isinstance(payload, dict):
                continue

            last_payload = payload
            state = int(payload.get("state", -1))
            state_name = str(payload.get("state_name", "UNKNOWN"))
            flow_name = str(payload.get("flow_name", "UPSTAIRS"))
            state_done = bool(payload.get("state_done"))
            auto_run = bool(payload.get("auto_run"))
            elapsed = payload.get("elapsed_ms", "?")
            print(
                f"climb: flow={flow_name} state={state_name}({state}) done={int(state_done)} auto={int(auto_run)} elapsed={elapsed}ms",
                flush=True,
            )

            if state == CLIMB_ERROR_STATE or state_name == "ERROR":
                raise RuntimeError(f"climb entered ERROR, flags={payload.get('error_flags')}")
            if final_state:
                if state == CLIMB_DONE_STATE or state_name == "DONE":
                    print("Climb final state is DONE.", flush=True)
                    return payload
            elif state in {0, CLIMB_DONE_STATE} or state_done:
                print("Climb is ready for the next command.", flush=True)
                return payload

        summary = self._format_climb_payload(last_payload)
        raise TimeoutError(f"timeout waiting for climb {mode} after {timeout_s:g}s. last={summary}")

    @staticmethod
    def _format_climb_payload(payload: Optional[Dict[str, Any]]) -> str:
        if not payload:
            return "no CLIMB_GET_STATUS reply"
        return (
            f"state={payload.get('state_name', 'UNKNOWN')}({payload.get('state', '?')}), "
            f"flow={payload.get('flow_name', 'UPSTAIRS')}, "
            f"done={int(bool(payload.get('state_done')))}, "
            f"auto={int(bool(payload.get('auto_run')))}, "
            f"errors={payload.get('error_flags')}"
        )

    @staticmethod
    def _optional_timeout(args: List[str], default: float = DEFAULT_CLIMB_WAIT_TIMEOUT_S) -> float:
        if not args:
            return default
        if len(args) != 1:
            raise ValueError("usage: command [timeout_s]")
        return float(args[0])

    @staticmethod
    def _need_floats(args: List[str], count: int, usage: str) -> List[float]:
        if len(args) < count:
            raise ValueError(f"usage: {usage}")
        return [float(x) for x in args]

    def _print_commands(self) -> None:
        for command in list_commands():
            payload = "4float" if command.needs_float_payload else ("optional" if command.optional_float_payload else "empty")
            print(f"{command.name:<22} 0x{command.cmd:02X} {payload:<7} {command.doc}", flush=True)

    @staticmethod
    def _print_help() -> None:
        print(
            "\n".join(
                [
                    "help",
                    "commands",
                    "flow_start [name] | flow_confirm [note] | flow_save [note] | flow_add ACTION [timeout_s] [note...]",
                    "flow_show | flow_export [file.json] | flow_recover [file.json] | flow_autosave [file.json] | flow_clear",
                    "pack NAME [float...]",
                    "send NAME [float...]",
                    "NAME [float...] also works, for example TOOL_SET_MODE 0",
                    "query ROBOT|TUNE|CLIMB|SYS|CHS|ARM|TOOL",
                    "raw A5 5A ...",
                    "usb | usart | enable | disable | stop | status",
                    "chs_enable | chs_disable | chs_mode MODE | vel VX(forward) VY(left) YAW_DATA(CCW) | pos DX(forward) DY(left) YAW_DATA(CCW) | chs_stop | chs_status",
                    "arm_enable | arm_disable | arm_target X(forward) Y(left) Z(up) | arm_stop | arm_status",
                    "tool_enable | tool_chuck/tool_clamp position | tool_open/tool_close legacy | tool_state DEV ACT | chuck_open/close | clamp_open/close | tool_status",
                    "climb_enable | climb_step | climb_up_auto/climb_auto | climb_ctrl ENABLE STEP AUTO | climb_stop | climb_status",
                    "climb_wait [timeout_s] | climb_step_wait [timeout_s] | climb_up_auto_wait/climb_auto_wait [timeout_s] | climb_wait_then NAME [float...]",
                    "climb_down_step/climb_downstairs_step | climb_down_step_wait [timeout_s] | climb_down_auto/climb_downstairs_auto | climb_down_auto_wait [timeout_s]",
                    "climb_up_laser_gate | climb_down_laser_gate | climb_up_auto_pause | climb_down_auto_pause | climb_auto_resume",
                    "climb_tests | climb_test ACTION | climb_test_wait ACTION [timeout_s]",
                    "climb_<action_name_lower> [timeout_s], for example climb_all_drive_forward_30 or climb_front_drive_forward_30 10",
                    "tune_start [pass_count] | tune_status | tune_stop",
                    "exit",
                ]
            ),
            flush=True,
        )
