import os.path
from pathlib import Path

import cv2
import numpy as np
import math
from scipy.spatial.transform import Rotation as R1

"""
实现pnp测距
"""


def read_params(params_path):
    params_path = os.path.join(params_path + '/params.txt')
    params_list = []
    with open(params_path, 'r', encoding='utf-8') as f:
        params = f.readlines()
        for param in params:
            param.strip()
            params_list.append(param[2:])
    return params_list


def rad2angle(number):
    return (180.0 / math.pi) * number


def angle2rad(angle):
    return (math.pi / 180.0) * angle


def take_p():
    image_path = Path(__file__).resolve().parent / 'pic' / 'test' / 'test.jpg'
    image_path.parent.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        res, img = cap.read()
        cv2.imshow('img', img)
        key = cv2.waitKey(1)
        if key == 113:
            break
        if key == 115:
            cv2.imwrite(str(image_path), img)
            print('save')
    cap.release()
    cv2.destroyAllWindows()


def rotation2euler(matrix):
    rotation = R1.from_matrix(matrix)
    euler_angles = rotation.as_euler('xyz', degrees=True)
    return euler_angles


def get_point(save=True):
    point = []
    root = Path(__file__).resolve().parent

    def mouse_callback(event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            print(x, y)
            point.append([x, y])

    img = cv2.imread(str(root / 'pic' / 'test' / 'test.jpg'))
    cv2.namedWindow('img')
    cv2.setMouseCallback('img', mouse_callback)
    cv2.imshow('img', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    point = np.array(point, dtype=np.float32)
    if save:
        np.save(root / 'model' / '2d_point_test.npy', point)
        print('save 2d_point_test.npy')
    return point


def calculate_distance(thr_D, two_D, path):
    # two_D = np.load(r'model\2d_point_test.npy')
    K = np.load(os.path.join(path, 'K.npy'))
    dist = np.load(os.path.join(path, 'dist.npy'))
    res, r_v, t_v = cv2.solvePnP(thr_D, two_D, K, dist)
    R, _ = cv2.Rodrigues(r_v)
    P = -np.dot(R.T, t_v)
    distance = np.sqrt(np.sum(P[:3] ** 2))
    return distance


def calculate_angle(thr_D, two_D, path):
    K = np.load(os.path.join(path + '/K.npy'))
    dist = np.load(os.path.join(path + '/dist.npy'))
    res, r_v, t_v = cv2.solvePnP(thr_D, two_D, K, dist)
    R, _ = cv2.Rodrigues(r_v)
    angle_x = rad2angle(math.atan2(-R[2][1], R[2][2]))
    angle_y = rad2angle(math.atan2(R[2][0], math.sqrt(R[2][1] ** 2 + R[2][2] ** 2)))
    angle_z = rad2angle(math.atan2(-R[1][0], R[0][0]))
    return angle_x, angle_y, angle_z


def calculate_xyz(thr_D, two_D, path, k, b):
    K = np.load(os.path.join(path + '/K.npy'))
    dist = np.load(os.path.join(path + '/dist.npy'))
    res, r_v, t_v = cv2.solvePnP(thr_D, two_D, K, dist)
    t_v = t_v.ravel()
    dis_z = t_v[2] - (t_v[2] * k + b)
    dis_x, dis_y, dis_z = t_v[0], t_v[1], dis_z
    return dis_x, dis_y, dis_z


def refactor(t: list, angle_xyz: list, point: list):
    point.append(1)  # 转为齐次坐标
    p = np.array([point]).T
    euler_angles = angle_xyz
    rotation = R1.from_euler('xyz', euler_angles, degrees=True)
    r1 = rotation.as_matrix()
    rotation_matrix = np.eye(4)
    rotation_matrix[:3, :3] = r1
    rotation_matrix[:3, 3] = t
    ret = rotation_matrix @ p
    x, y, z = ret[0][0], ret[1][0], ret[2][0]
    return x, y, z


def calculate_euler_rotation(thr_D, two_D, path):
    K = np.load(os.path.join(path + '/K.npy'))
    dist = np.load(os.path.join(path + '/dist.npy'))
    res, r_v, t_v = cv2.solvePnP(thr_D, two_D, K, dist)
    R, _ = cv2.Rodrigues(r_v)
    return rotation2euler(R)


def init_3d(R):
    return np.array([[-R, 0, 0],
                     [R, 0, 0],
                     [0, R, 0],
                     [0, -R, 0]], dtype=np.float32)


if __name__ == '__main__':
    pass
