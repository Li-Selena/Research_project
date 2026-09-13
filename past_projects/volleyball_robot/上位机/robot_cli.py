from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict
from pathlib import Path

from modbus_client import RobotClient


ROOT = Path(__file__).resolve().parent


def create_client() -> RobotClient:
    config = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
    comm = config["communication"]
    return RobotClient(port=comm.get("serial_port"), baudrate=int(comm["baudrate"]),
                       timeout=float(comm["timeout_s"]), retries=int(comm["retries"]))


def main() -> None:
    parser = argparse.ArgumentParser(description="排球机器人Modbus调试工具")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    speed = sub.add_parser("speed")
    speed.add_argument("vx", type=float)
    speed.add_argument("vy", type=float)
    speed.add_argument("wz", type=float)
    speed.add_argument("--seconds", type=float, default=1.0)
    position = sub.add_parser("position")
    position.add_argument("x", type=float)
    position.add_argument("y", type=float)
    position.add_argument("yaw", type=float)
    position.add_argument("--seconds", type=float, default=5.0)
    for name in ("stop", "reset-odometry", "delta-strike", "delta-home",
                 "rod-prepare", "rod-home", "rod-strike", "emergency-stop"):
        sub.add_parser(name)
    delta_point = sub.add_parser("delta-point")
    delta_point.add_argument("x", type=float)
    delta_point.add_argument("y", type=float)
    delta_point.add_argument("z", type=float)
    delta_point.add_argument("--timeout-ms", type=int, default=300)
    args = parser.parse_args()

    robot = create_client()
    try:
        if args.command == "status":
            print(json.dumps(asdict(robot.read_status()), ensure_ascii=False, indent=2))
        elif args.command == "speed":
            robot.set_chassis_speed(args.vx, args.vy, args.wz)
            robot.start()
            time.sleep(max(0.0, args.seconds))
        elif args.command == "position":
            robot.move_chassis_to(args.x, args.y, args.yaw)
            robot.start()
            time.sleep(max(0.0, args.seconds))
        elif args.command == "stop":
            robot.stop_chassis()
            robot.apply_chassis_command()
        elif args.command == "reset-odometry": robot.reset_odometry()
        elif args.command == "delta-strike": robot.strike_delta()
        elif args.command == "delta-home": robot.home_delta()
        elif args.command == "rod-prepare": robot.prepare_rod()
        elif args.command == "rod-home": robot.home_rod()
        elif args.command == "rod-strike": robot.strike_rod()
        elif args.command == "emergency-stop": robot.emergency_stop()
        elif args.command == "delta-point":
            robot.set_delta_strike_point(args.x, args.y, args.z, args.timeout_ms)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
