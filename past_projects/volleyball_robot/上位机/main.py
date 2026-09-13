from __future__ import annotations

import json
from pathlib import Path

import cv2
from ultralytics import YOLO

from func import (Controller, Detection, Filter, InfoShow, KFBbox, KFV,
                  PnpSolve, Show, TargetTrack, Video, ball_process,
                  filter_data, kf_filter_xyz, resize)
from modbus_client import RobotClient

ROOT = Path(__file__).resolve().parent


def load_config() -> dict:
    with (ROOT / "config.json").open("r", encoding="utf-8") as file:
        return json.load(file)


def path_from_config(config: dict, key: str) -> Path:
    path = ROOT / config[key]
    if not path.exists():
        raise FileNotFoundError(f"{key} does not exist: {path}")
    return path


def main() -> None:
    config = load_config()
    vision = config["vision"]
    control = config["control"]
    comm = config["communication"]

    main_model = YOLO(str(path_from_config(vision, "main_model")))
    delta_model = YOLO(str(path_from_config(vision, "delta_model")))
    main_camera_path = path_from_config(vision, "main_calibration")
    delta_camera_path = path_from_config(vision, "delta_calibration")

    video = Video()
    video.creat_new(int(vision["main_camera_id"]), name="main")
    video.creat_new(int(vision["delta_camera_id"]), name="delta")
    video.init("main", False)
    video.init("delta", "delta")

    show_main, show_delta = Show(), Show()
    pnp = PnpSolve(float(vision["ball_radius_mm"]), str(main_camera_path))
    pnp_delta = PnpSolve(float(vision["ball_radius_mm"]), str(delta_camera_path))
    track, track_delta = TargetTrack(), TargetTrack()
    info = InfoShow(bool(vision["show_overlay"]))
    kf_bbox, kf_bbox_delta = KFBbox(), KFBbox()
    detection_main = Detection(main_model)
    detection_delta = Detection(delta_model)
    detection_main.start()

    kf_list = [KFV(), KFV(), KFV()]
    filters = [Filter(5), Filter(5), Filter(5)]
    distance_filter = Filter(5)

    robot = RobotClient(port=comm.get("serial_port"), baudrate=int(comm["baudrate"]),
                        timeout=float(comm["timeout_s"]), retries=int(comm["retries"]))
    robot.set_delta_strike_point(*map(float, control["delta_strike_xyz_mm"]),
                                 timeout_ms=int(control["delta_strike_timeout_ms"]))
    robot.start()
    controller = Controller(robot, striker=control["default_striker"])
    controller.set_k(20, 15, 10, 10)
    controller.set_y_target(float(control["stay_distance_mm"]),
                            float(control["track_y_distance_mm"]))
    controller.set_track_distance(float(control["track_distance_mm"]))

    try:
        while True:
            img_main = video.get_pic("main")
            img_delta = video.get_pic("delta")
            if img_main is None or img_delta is None:
                if cv2.waitKey(1) == ord("q"):
                    break
                continue

            img_main = resize(img_main, dsize=(1920, 1080))
            img_delta = resize(img_delta)
            detection_main.update_img(img_main)
            detection_delta.update_img(img_delta)
            target, result = detection_main.get_result()
            target_delta, result_delta = detection_delta.get_result()

            ball_process(img_main, target, result, track, pnp, kf_bbox, info, (0.01, 100, 10))
            ball_process(img_delta, target_delta, result_delta, track_delta,
                         pnp_delta, kf_bbox_delta, info, (0.01, 100, 2))
            distance_main = distance_filter.update(float(pnp.get_distance(target)))
            distance_delta = float(pnp_delta.get_distance(target_delta))
            detection_main.update_distance(max(0.0, distance_main))
            detection_delta.update_distance(max(0.0, distance_delta))

            if 0 < distance_main < float(control["delta_detection_mm"]) or not target:
                detection_delta.start()
                info.put_text(img_delta, "Start", (10, 100), (0, 255, 0))
            else:
                detection_delta.close()
                info.put_text(img_delta, "Close", (10, 100), (0, 0, 255))

            if distance_main >= 0:
                x, y, z = pnp.refactor_3d(list(map(float, vision["main_camera_translation_mm"])),
                                          float(vision["main_camera_rotation_deg"]))
                x, y, z = filter_data(filters, x, y, z)
                y -= distance_main * -0.14332551 - 11.86559049
            else:
                x, y, z = float(control["stay_distance_mm"]), 0.0, 0.0

            x_state, y_state, z_state = kf_filter_xyz(kf_list, x, y, z)
            controller.update_xyz(x, y, z)
            controller.update_v(x_state[1], y_state[1], z_state[1])
            controller.update_a(x_state[2], y_state[2], z_state[2])
            controller.update_j(x_state[3], y_state[3], z_state[3])
            controller.send_v(target, target_delta, distance_main, result_delta,
                              float(control["delta_track_distance_mm"]))
            controller.send_delta(
                target, target_delta, distance_main, distance_delta,
                float(control["main_strike_distance_mm"]),
                float(control["delta_strike_distance_mm"]),
                float(control["main_strike_speed_coefficient"]),
                float(control["delta_strike_speed_coefficient"]),
            )
            controller.send(target, target_delta)

            info.put_text(img_main, f"fps:{show_main.get_fps():.2f}", (10, 30), (0, 255, 0))
            info.put_text(img_delta, f"fps:{show_delta.get_fps():.2f}", (10, 30), (0, 255, 0))
            info.put_text(img_main, f"distance:{distance_main / 10:.2f}cm", (200, 30), (0, 0, 255))
            info.put_text(img_delta, f"distance:{distance_delta / 10:.2f}cm", (200, 30), (0, 0, 255))
            show_main(cv2.resize(img_main, (640, 480)))
            show_delta(img_delta)
            if cv2.waitKey(1) == ord("q"):
                break
    finally:
        robot.close()
        detection_main.close()
        detection_delta.close()
        video.close_all_video()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
