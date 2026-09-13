import cv2

from func import *


class Filter:
    def __init__(self, k):
        self.k = k
        self.buffer = [0.0] * k
        self.index = 0

    def update(self, data):
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.k
        return round(sum(self.buffer) / self.k, 10)


def main():
    rotation_angle = 0
    t = [0, 0, 800]
    cap = VidioCapture(2)
    cap.set(cv2.CAP_PROP_FPS, 90)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    fps, w1, h1 = [cap.get(cv2.CAP_PROP_FPS),
                   cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                   cap.get(cv2.CAP_PROP_FRAME_HEIGHT)]
    print(f'\tfps: {fps:.2f} resolution: {w1} x {h1}')
    MODEL = YOLO(r'model\best_delta1v8n.pt')
    detection = Detection(MODEL)
    detection.start()
    track = TargetTrack()
    pnp = PnpSolve(120, r'model\camera6')
    kf_bbox = KFBbox()
    info = InfoShow(True)
    show = Show()
    filter_main_distance = Filter(5)
    filter_x = Filter(5)
    filter_y = Filter(5)
    filter_z = Filter(5)
    kf_x, kf_y, kf_z = KFV(), KFV(), KFV()
    kf_list = [kf_x, kf_y, kf_z]
    controller = Controller(None)
    while cap.isOpened():
        ret, img = cap.read()
        if ret is False:
            cap = VidioCapture(r"D:\WORK\PY_PROJECT\label\vidio_cache\2025-06-23 21.41.48.mp4")
            ret, img = cap.read()
        detection.update_img(img)
        target, result = detection.get_result()
        ball_process(img, target, result, track, pnp, kf_bbox, info, (0.05, 30, 10))
        distance = pnp.get_distance(result)
        distance = filter_main_distance.update(distance)
        x, y, z = pnp.refactor_3d(t, rotation_angle)  # 返回以摄像头方向为x轴正方向的右手坐标系
        x, y, z = filter_data([filter_x, filter_y, filter_z], x, y, z)
        x_state, y_state, z_state = kf_filter_xyz(kf_list, x, y, z)
        controller.update_xyz(x, y, z)
        controller.update_v(x_state[1], y_state[1], z_state[1])
        controller.update_a(x_state[2], y_state[2], z_state[2])
        controller.update_j(x_state[3], y_state[3], z_state[3])
        # controller.send_delta((target_delta or target), distance_main, distance_delta,
        #                       bit_distance_main, bit_distance_delta, bit_v_k)
        info.put_text(img, f'fps:{show.get_fps():.2f}', (10, 30), (0, 255, 0))
        info.put_text(img,
                      f'v_x:{x_state[1] / 10:.2f}cm/s '
                      f'v_y:{y_state[1] / 10:.2f}cm/s '
                      f'v_z:{z_state[1] / 10:.2f}cm/s',
                      (10, 50), (0, 255, 255),
                      fontscale=0.5)
        info.put_text(img, f"distance: {distance / 10:.2f}cm", (200, 30), (0, 0, 255))
        cv2.circle(img, (320, 120), 5, (255, 0, 0), -1)
        show(img)
        if cv2.waitKey(1) == 113:
            detection.close()
            cap.release()
            break


if __name__ == '__main__':
    main()
