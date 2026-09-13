# type:ignore
import time
import cv2
import os
import numpy as np
from time import perf_counter
from dataclasses import dataclass
from threading import Thread
from ultralytics import YOLO
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from pnp import refactor, calculate_xyz, calculate_distance, init_3d, read_params


class VidioCapture(cv2.VideoCapture):
    """
    自定 VideoCapture 类
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.args = args
        self.img = None
        self.result = None
        self.run = True

    def get_pic(self):
        return self.img

    def init(self, param):
        """
        初始化相机参数
        :param param:
        :return:
        """
        if param is False:
            self.set(cv2.CAP_PROP_FPS, 91)
            self.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            fps, w, h = [self.get(cv2.CAP_PROP_FPS),
                         self.get(cv2.CAP_PROP_FRAME_WIDTH),
                         self.get(cv2.CAP_PROP_FRAME_HEIGHT)]
            print(f'\tfps: {fps:.2f} resolution: {w} x {h}')
            self.start()
        elif param == 'delta':
            self.set(cv2.CAP_PROP_FPS, 120)
            self.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            fps, w, h = [self.get(cv2.CAP_PROP_FPS),
                         self.get(cv2.CAP_PROP_FRAME_WIDTH),
                         self.get(cv2.CAP_PROP_FRAME_HEIGHT)]
            print(f'\tfps: {fps:.2f} resolution: {w} x {h}')
            self.start()
        else:
            fps, w, h = [self.get(cv2.CAP_PROP_FPS),
                         self.get(cv2.CAP_PROP_FRAME_WIDTH),
                         self.get(cv2.CAP_PROP_FRAME_HEIGHT)]
            print(f'\tfps: {fps:.2f} resolution: {w} x {h}')
            self.start()

    def start(self):
        Threading(target=self.read_pic)  # 启用多线程捕获每一帧

    def read_pic(self):
        while self.run:
            self.result, self.img = self.read()

    def __repr__(self):
        return f'camera id: {self.args[0]}'

    def close(self):
        """
        关闭当前主循环
        :return:
        """
        self.run = False
        time.sleep(0.2)
        self.release()


class Video:
    def __init__(self):
        self.count = 0
        self.videocapture = {}

    def creat_new(self, *args, **kwargs):
        """
        可以使用name关键字给相机命名
        默认名称 0,1,2,3...
        :param args:
        :param kwargs:
        :return:
        """
        if kwargs.get('name', False):
            name = kwargs.pop('name')
            self.videocapture[name] = VidioCapture(*args, **kwargs)
        else:
            self.videocapture[str(self.count)] = VidioCapture(*args, **kwargs)
            self.count += 1

    def get_key_value(self):
        """
        获取相机名称和相机id的键值对
        :return:
        """
        return self.videocapture.items()

    def init(self, video_name, default=True):
        """
        初始化，default=True使用默认，否则使用自定义分辨率和帧率
        :param video_name:
        :param default:
        :return:
        """
        if video_name not in self.videocapture:
            raise NameError(f'no found "{video_name}" in "{self.videocapture.keys()}"')
        print(f'init camera: "{video_name}":')
        self.videocapture[video_name].init(default)

    def get_pic(self, video_name):
        """
        获取名字对应的相机的图像
        :param video_name:
        :return:
        """
        return self.videocapture[video_name].get_pic()

    def close_all_video(self):
        """
        释放所有相机的缓存资源
        :return:
        """
        for i in self.videocapture:
            print(f'waiting close camera: "{i}"...')
            self.videocapture[i].close()
            print('Done')

    def close_single_video(self, video_name):
        """
        释放指定名称的相机的缓存资源
        :param video_name:
        :return:
        """
        if video_name not in self.videocapture:
            raise NameError(f'no found "{video_name}" in "{self.videocapture.keys()}"')
        else:
            print(f'waiting close camera: "{video_name}"...')
            self.videocapture[video_name].close()
            print('Done')


class Threading(Thread):
    """
    重写多线程
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.daemon = True
        self.start()


class Show:
    count = 0  # 计数创建的实例，用于给不同窗口命名

    def __init__(self, *args, **kwargs):
        self.fps = 0
        self.frame_count = 0  # 用于计算fps
        self.t1 = perf_counter()
        self.t2 = perf_counter()
        self.c = self.count  # 当前窗口的标号为创建的实例数-1
        self.windows = cv2.namedWindow(f'img{self.c}', *args, **kwargs)
        self.add()

    def __call__(self, img):
        self.frame_count += 1
        self.t2 = perf_counter()
        cv2.imshow(f'img{self.c}', img)
        if (t := self.t2 - self.t1) > 0.3:  # 帧计数器
            self.fps = self.frame_count / t
            self.t1 = perf_counter()
            self.frame_count = 0

    def get_fps(self) -> float:
        return self.fps

    @classmethod
    def add(cls):
        """
        每创建一个对象，都调用此方法将类变量count自增1
        :return:
        """
        cls.count += 1


class PnpSolve:
    """
    实现Pnp解算
    传入 [球的半径mm, 相机的参数文件]

    """

    def __init__(self, R, camera_params):
        self.thr_D = init_3d(R)
        self.camera_params = camera_params
        self.k, self.b = map(float, read_params(camera_params))
        self._K = np.load(os.path.join(camera_params, 'K.npy'))
        self._dist = np.load(os.path.join(camera_params, 'dist.npy'))
        self.two_D = None
        self.res, self.r_v, self.t_v = None, None, None
        self._loss_target = 0

    def calculate_distance(self, two_D=None):
        """
        根据三维二维点计算距离
        :return:
        """
        if two_D is None:
            res, r_v, t_v = cv2.solvePnP(self.thr_D, self.two_D, self._K, self._dist)
            self.res, self.r_v, self.t_v = res, r_v, t_v
        else:
            res, r_v, t_v = cv2.solvePnP(self.thr_D, two_D, self._K, self._dist)
        R, _ = cv2.Rodrigues(r_v)
        P = -np.dot(R.T, t_v)
        distance = np.sqrt(np.sum(P[:3] ** 2))
        return distance

    def calculate_xyz(self):
        res, r_v, t_v = self.res, self.r_v, self.t_v
        t_v = t_v.ravel()
        dis_z = t_v[2] - (t_v[2] * self.k + self.b)
        dis_x, dis_y, dis_z = t_v[0], t_v[1], dis_z
        return dis_x, dis_y, dis_z

    def correct_loss(self, distance):
        """
        用最小二乘得到的参数纠正线性误差
        :param distance:
        :return:
        """
        loss = self.k * distance + self.b
        distance -= loss
        return distance

    def s_calculate_distance(self, two_D):
        return max(0, self.correct_loss(self.calculate_distance(two_D)))

    def set_two_D(self, two_D):
        """
        更新像素坐标[up, down, left, right]
        :param two_D:
        :return:
        """
        self.two_D = two_D.astype(np.float32)

    def get_distance(self, target) -> float:
        """
        获取距离，初次调用之前必须先调用set_two_D来初始化像素坐标，否则 self.two_D = None
        传入target可以在丢失目标n帧后将距离归零
        :return:
        """
        if target is False:
            self._loss_target += 1
        else:
            self._loss_target = 0
        if self.two_D is None or np.unique(self.two_D, axis=0).shape[0] < 4:
            return -1.0
        try:
            distance = max(0, self.correct_loss(self.calculate_distance()))
        except cv2.error:
            return -1.0
        if self._loss_target > 20:
            distance = -1
        return distance

    def refactor_3d(self, t, rotation_angle):
        """
        将世界坐标系变为相机坐标系，t和rotation_angle为相机的平移向量和旋转角
        :param t:
        :param rotation_angle:
        :return:
        """
        x_c, y_c, z_c = self.calculate_xyz()  # 摄像机坐标系下的xyz
        # 以摄像机为xy原点，摄像机观测在z:t[2]位置的坐标系
        x_w, y_w, z_w = refactor(t, [270.0 + rotation_angle, 0.0, 90.0], [x_c, y_c, z_c])
        return -x_w, y_w, z_w


class KFV(cv2.KalmanFilter):
    """
    ps:I want to eat kfc!!!
    No! this is kalman filter
    Now you are about to adjustable this damn parameters

    状态：[x,v,a,j]
    观测：[x]


    """

    def __init__(self, *args, **kwargs):
        super().__init__(4, 1, *args, **kwargs)

        self.dt = 0.3
        self.first = True
        self.transitionMatrix = np.array([[1, self.dt, 0.5 * self.dt ** 2, (1 / 6) * self.dt ** 3],
                                          [0, 1, self.dt, 0.5 * self.dt ** 2],
                                          [0, 0, 1, self.dt],
                                          [0, 0, 0, 1]], np.float32)
        self.measurementMatrix = np.array([[1, 0, 0, 0]], np.float32)
        self.processNoiseCov = np.array([[0.7, 0, 0, 0],
                                         [0, 0.1, 0, 0],
                                         [0, 0, 0.5, 0],
                                         [0, 0, 0, 0.3]], dtype=np.float32) * 3
        self.measurementNoiseCov = np.array([[1]], dtype=np.float32) * 0.8

    def set_matrix(self, transitionMatrix, measurementMatrix, processNoiseCov, measurementNoiseCov):
        self.transitionMatrix = transitionMatrix
        self.measurementMatrix = measurementMatrix
        self.processNoiseCov = processNoiseCov
        self.measurementNoiseCov = measurementNoiseCov

    def update_get_position(self, new_position):  # [[x]] (1,1) dtype: np.float32 -> x, v, a, j
        if self.first:
            state = np.zeros((4, 1), np.float32)
            state[0, 0] = new_position[0, 0]
            self.statePre = state.astype(np.float32)
            return self.statePre[[0, 1, 2, 3], 0]
        else:

            correct = self.correct(new_position)
            return correct[[0, 1, 2, 3], 0]

    def get_pred_position(self):
        if self.first:
            self.first = False
            # print(self.statePre)
            return self.statePre[[0, 1, 2, 3], 0]
        else:
            pred = self.predict()
            return pred[[0, 1, 2, 3], 0]


class KFBbox(cv2.KalmanFilter):
    """
    作目标跟踪 [x, v, a]
    """

    def __init__(self, *args, **kwargs):
        super().__init__(24, 8, *args, **kwargs)
        self.first = True
        self.count = 0
        self.pred = None
        self.index = [0, 1, 6, 7, 12, 13, 18, 19]
        self.dt = 0.1

        F_block = np.array([
            [1, 0, self.dt, 0, 0.5 * self.dt ** 2, 0],
            [0, 1, 0, self.dt, 0, 0.5 * self.dt ** 2],
            [0, 0, 1, 0, self.dt, 0],
            [0, 0, 0, 1, 0, self.dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ], dtype=np.float32)

        self.transitionMatrix = np.block([
            [F_block, np.zeros((6, 18))],
            [np.zeros((6, 6)), F_block, np.zeros((6, 12))],
            [np.zeros((6, 12)), F_block, np.zeros((6, 6))],
            [np.zeros((6, 18)), F_block]
        ]).astype(np.float32)

        self.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        ], dtype=np.float32)

        Q_block = np.diag([1e-2, 1e-2, 1e-1, 1e-1, 1e-2, 1e-2]).astype(np.float32) * 18

        self.processNoiseCov = np.block([
            [Q_block, np.zeros((6, 18))],
            [np.zeros((6, 6)), Q_block, np.zeros((6, 12))],
            [np.zeros((6, 12)), Q_block, np.zeros((6, 6))],
            [np.zeros((6, 18)), Q_block]
        ]).astype(np.float32)

        self.measurementNoiseCov = np.eye(8, dtype=np.float32) * 0.07
        self.statePre = np.zeros((24, 1), dtype=np.float32)

    def update_get_position(self, new_position):  # [[x1,y1],[x2,y2],[x3,y3],[x4,y4]]
        self.count = 0
        if self.first:
            x1, y1, x2, y2, x3, y3, x4, y4 = new_position
            state = np.zeros((24, 1), np.float32)
            state[0:2, 0] = [x1[0], y1[0]]
            state[6:8, 0] = [x2[0], y2[0]]
            state[12:14, 0] = [x3[0], y3[0]]
            state[18:20, 0] = [x4[0], y4[0]]
            self.statePre = state
            return np.array([x1, y1, x2, y2, x3, y3, x4, y4], np.float32).reshape(4, 2)
        else:
            correct = self.correct(new_position.astype(np.float32))
            return correct[self.index, 0].reshape(4, 2)

    def get_pred_position(self):
        if self.first:
            self.first = False
            return self.statePre[self.index, 0].reshape(4, 2)
        else:
            if self.count < 5:
                self.count += 1
                # self.statePre[2::6] *= 0.7
                # self.statePre[4::6] *= 0.5
                pred = self.predict()
                self.pred = pred
                return pred[self.index, 0].reshape(4, 2)
            else:
                return self.pred[self.index, 0].reshape(4, 2)


class TargetTrack:
    def __init__(self):
        self.old_position = None  # 上一时刻目标的bbox位置
        self.old_d = None  # 上一时刻目标的距离
        self.loss_target = 0

    def _update_d(self, d):
        self.old_d = d

    def _update_position(self, position):
        self.old_position = position

    def _l2_distance(self, position):
        """
        计算欧氏距离
        :param position:
        :return:
        """
        return np.sqrt(np.sum((self.old_position - position) ** 2))

    @staticmethod
    def compute_iou(bbox1, bbox2):
        """
        计算两个边界框的 IoU，每个边界框格式为 4x2 的 numpy 数组，行顺序为 up, down, left, right，
        每行为一个点 (x, y) 的坐标。

        :param bbox1: numpy.ndarray, shape=(4,2)
        :param bbox2: numpy.ndarray, shape=(4,2)
        :return: float, IoU
        """

        # bbox1 边界
        top1 = bbox1[0, 1]
        bottom1 = bbox1[1, 1]
        left1 = bbox1[2, 0]
        right1 = bbox1[3, 0]

        # bbox2 边界
        top2 = bbox2[0, 1]
        bottom2 = bbox2[1, 1]
        left2 = bbox2[2, 0]
        right2 = bbox2[3, 0]

        # 交集区域
        inter_left = max(left1, left2)
        inter_right = min(right1, right2)
        inter_top = max(top1, top2)
        inter_bottom = min(bottom1, bottom2)

        inter_width = max(0.0, inter_right - inter_left)
        inter_height = max(0.0, inter_bottom - inter_top)
        inter_area = inter_width * inter_height

        # 各自面积
        area1 = (right1 - left1) * (bottom1 - top1)
        area2 = (right2 - left2) * (bottom2 - top2)

        # iou
        union_area = area1 + area2 - inter_area
        iou = inter_area / union_area if union_area > 0 else 0.0

        return iou

    def is_target(self, position, d, c1, c2, c3):
        """
        根据上一帧目标点判断当前帧是否为目标
        position: 当前帧目标中点坐标
        d: 当前帧目标与相机的距离
        c1: iou阈值
        c2: 两帧目标距相机距离的差值阈值
        c3: 丢失目标c3帧后设定为新目标
        :param position:
        :param d:
        :param c1:
        :param c2:
        :param c3:
        :return:
        """
        if self.old_position is not None and self.old_d is not None:  # 如果不是首个目标
            iou = self.compute_iou(position, self.old_position)
            # l2_d = self._l2_distance(position)

            if (iou < c1) and (abs(d - self.old_d) > c2):  # 两帧是否为同一目标判断条件,条件成立则不是同一目标
                if self.loss_target > c3:  # 如果连续c3帧丢失目标，将此时的坐标作为新目标
                    self.loss_target = 0
                    self._update_position(position)
                    self._update_d(d)
                    return True
                else:
                    self.loss_target += 1
                    return False
            else:
                self._update_d(d)
                self._update_position(position)
                self.loss_target = 0
                return True

        else:  # 如果是首个目标
            self.loss_target = 0
            self._update_d(d)
            self._update_position(position)
            return True


class PredX:
    """
    暂时弃用
    """

    def __init__(self, max_point, eps, min_samples):
        self.position = np.array([np.nan] * max_point * 2).reshape(-1, 2)
        self.max_point = max_point
        self.index = 0
        self.model = DBSCAN(eps=eps, min_samples=min_samples, metric='euclidean')
        self.scaler = StandardScaler()

    def update(self, position):
        self.position[self.index] = position
        self.index += 1
        if self.index > self.max_point:
            self.index = 0

    @staticmethod
    def filter_nan(x):
        """
        过滤x中有nan的行
        :param x:
        :return:
        """
        true_index = ~np.isnan(x).any(axis=1)
        return x[true_index]

    def get_last_ndata(self, data, n):
        """
        取出当前index的前n个数据
        :param data:
        :param n:
        :return:
        """
        return data[np.arange(self.index - (n - 1), self.index + 1)]

    def pred(self):
        data = self.filter_nan(self.position)
        scaler_data = self.scaler.fit_transform(data)
        self.model.fit(scaler_data)


class PID:
    def __init__(self, kp, ki, kd, max_integral):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._now = None
        self._target = None
        self._error = None
        self._last_error = 0.0
        self._integral = 0.0
        self._max_integral = max_integral

    def set_target(self, target):
        if self._target is None:
            self._integral = 0.0
        self._target = target

    def set_now(self, now):
        self._now = now

    def output(self):
        self._error = self._target - self._now
        self._integral += self._error
        self._integral = abs_clip(self._integral, self._max_integral)
        d = self._error - self._last_error
        output = self._kp * self._error + self._ki * self._integral + self._kd * d
        self._last_error = self._error
        return output

class Controller:
    """
    控制
    """

    def __init__(self, sender, striker='delta'):
        self._sender = sender  # 发送和接受数据
        if striker not in ('delta', 'rod'):
            raise ValueError('striker must be "delta" or "rod"')
        self._striker = striker
        self._kf = KFV()  # 计算合速度
        self._pid_x = PID(0.19, 0.03, 0.07, 10)  # 控制x位移,车的y轴
        self._pid_y = PID(0.17, 0.03, 0.07, 10)  # 控制y位移,车的x轴
        self._pid_x_delta = PID(0.19, 0.03, 0.07, 10)
        self._pid_y_delta = PID(0.19, 0.03, 0.07, 10)
        self._timer1 = Timer(0.45, True, default=False)  # 用于delta收回零位
        self._timer2 = Timer(5, False)  # 用于关闭delta
        self._timer3 = Timer(1.5, True)  # 击球间隔
        self._timer4 = Timer(0.45, True, default=False)  # 用于delta收回缓冲
        self._timer5 = Timer(1, True)  # 重新蓄力计时
        self._timer6 = Timer(2.5, True)  # 激发间隔计时
        self._x, self._y, self._z = None, None, None
        self._vx, self._vy, self._vz = None, None, None
        self._ax, self._ay, self._az = None, None, None
        self._jx, self._jy, self._jz = None, None, None
        self._y_target = 1500.0  # 车与y轴保持的距离
        self._x_target = 0.0  # 车与x轴保持的距离 (对齐)
        self._pid_x.set_target(self._y_target)  # 将x的目标设为保持一定距离
        self._pid_y.set_target(self._x_target)  # 将车的x的目标设为原点
        self._pid_x_delta.set_target(320.0)
        self._pid_y_delta.set_target(120.0)
        self._track_y_distance = 0.0  # 跟球失效距离 大于失效
        self._track_distance = 5500.0  # 对齐失效距离
        self._k_x, self._k_y, self._k_x_d, self._k_y_d = 1.0, 1.0, 1.0, 1.0  # x,y的输出系数
        self._target_count = 0  # 统计丢失目标的帧数，防止一直运动
        self._val_target = 0  # 验证目标
        self._last_a = None
        self._send_data = [0.0] * 6
        self._bit_c = 0

    def set_k(self, k_x, k_y, k_x_d, k_y_d):
        self._k_y = k_x
        self._k_x = k_y
        self._k_x_d = k_x_d
        self._k_y_d = k_y_d

    def update_xyz(self, *args):
        self._x, self._y, self._z = args

    def update_v(self, *args):
        self._vx, self._vy, self._vz = args

    def update_a(self, *args):
        self._ax, self._ay, self._az = args

    def update_j(self, *args):
        self._jx, self._jy, self._jz = args

    def set_track_distance(self, distance):
        """
        设置对齐失效距离
        :param distance:
        :return:
        """
        self._track_distance = distance

    def set_y_target(self, y_target, y_track):
        """
        设置 跟球距离和 跟球失效距离
        :param y_target:
        :param y_track:
        :return:
        """
        if y_target > 100.0:
            self._track_y_distance = y_track
            self._y_target = y_target
            self._pid_x.set_target(y_target)
        else:
            self._track_y_distance = 0.0
            self._y_target = float(y_target)
            self._pid_x.set_target(self._y_target)
            print(f'{self}: longitudinal follow disabled; target set to {y_target}')

    @staticmethod
    def _fix_xy(x, y):
        """
        将摄像头坐标系转换为车坐标系
        :param x:
        :param y:
        :return:
        """
        return float(y), float(-x)

    @staticmethod
    def _feedforward_x(v_x, v_y, a_y, kf_base=0.5, kf_max=3):
        """
        对车的x轴pid做前馈调节
        动态调节 kf，a 越大，kf 越小
        :param v_x:
        :param v_y:
        :param a_y:
        :param kf_base:
        :param kf_max:
        :return:
        """
        acc_factor = 1 / (1e-1 + abs(a_y) * 0.25)
        kf = kf_base * acc_factor
        kf = float(min(kf, kf_max))
        v_x = v_x if v_x < 0 else 0.0
        return 0
        # return abs_clip(abs(v_x * 1.5) * kf * (-v_y / (abs(v_y) + 1e-7)), 500)

    # def _feedforward_y(self,v_x, a_x, kf_base=0.5, kf_max=4, ko=2.5, kv=0.8):
    #     """
    #     对车的y轴做前馈调节
    #     :param v_x:
    #     :param a_x:
    #     :param kf_base:
    #     :param kf_max:
    #     :param kk: 前馈加速度系数
    #     :param kv: 前馈速度系数
    #     :return:
    #     """
    #     v_x = self._filter_ay.update(v_x)
    #     a_x = abs_clip(a_x,25)
    #     acc_factor = 1 / (1e-1 + abs(a_x) * 0.25)
    #     kf = kf_base * acc_factor
    #     kf = float(min(kf, kf_max))
    #     pid = float(abs_clip(-v_x * kv * kf * abs(a_x) * ko, 500))
    #     # if pid < 490:
    #     #     print(f'{pid=} {kf=}')
    #     # else:
    #     #     print(f'{pid=} {v_x=} {abs(a_x)=}')
    #     return pid
    def _feedforward_y(self, v_x, a_x, distance, f_k=1000):
        """
        对车的y轴做前馈调节
        :param v_x:
        :param a_x:
        :param distance:
        :param f_k:
        :return:
        """
        if self._z > 550 and self._x > 900 and self._y_target > 100.0:
            acc_factor = clip(1 / (1e-1 + abs(a_x) * 0.25), 0, 3)
            f1 = f_k / distance
            f2 = f_k / (self._y_target + 1e-7)
            if f1 > f2:
                f1 = -f_k / (distance + self._y_target)
            pid = f1 * -v_x * acc_factor
        else:
            pid = 0.0
        return pid

    def _fix_vx(self, v, distance):
        """
        对齐失效函数&速度补偿
        :param v:
        :param distance:
        :return:
        """
        if distance > self._track_distance:
            v = 0.0
        return v * 1.4 if v > 0 else v

    def _fix_vy(self, v, distance):
        """
        跟球失效函数&速度补偿
        :param v:
        :param distance:
        :return:
        """
        # if self._vx < -10:
        #     if distance < self._y_target:
        #         v += v * (float(self._vx) * 0.05)
        #     else:
        #         v += v * (float(-self._vx) * 0.03)
        # if distance > self._track_y_distance:
        #     v = 0.0

        return v

    def _kfv(self, distance):
        """
        返回 x v a
        :param distance:
        :return:
        """
        state = self._kf.update_get_position(np.array([distance], np.float32).reshape(1, 1)) / 0.3  # 除以dt
        self._kf.get_pred_position()
        return state[0], state[1], state[2]

    @staticmethod
    def _dynamic_bit_distance(main_bit_distance, delta_bit_distance, v, v_k_main, v_k_delta):
        """
        根据球速动态调整delta触发阈值
        :param main_bit_distance:
        :param delta_bit_distance:
        :param v:
        :param v_k:
        :return:
        """
        dynamic_bit_distance_main = clip(-v * (v_k_main + clip(0.0056 * (-v - 300), 0, 1e10)), 0, 900)
        dynamic_bit_distance_delta = clip(-v * v_k_delta, 0, 600)
        bit_main = main_bit_distance + dynamic_bit_distance_main
        bit_delta = delta_bit_distance + dynamic_bit_distance_delta
        return float(bit_main), float(bit_delta)

    def _is_bit(self, main_distance, delta_distance, bit_distance):
        is_distance = ((0 < delta_distance < bit_distance[1]) or (0 < main_distance < bit_distance[0]))
        is_v = self._vx < -100
        is_h = 180 < self._z < 2500
        is_bit_ = is_distance and is_v and is_h
        return is_bit_

    def send_delta(self, target, target_delta, main_distance, delta_distance,
                   main_bit_distance, delta_bit_distance, v_k_main_, v_k_delta_):
        """
        delta激发函数
        :param target:
        :param target_delta:
        :param main_distance:
        :param delta_distance:
        :param main_bit_distance:
        :param delta_bit_distance:
        :param v_k_main_:
        :param v_k_delta_:
        :return:
        """

        if target or target_delta:
            bit_distance_ = self._dynamic_bit_distance(main_bit_distance, delta_bit_distance, self._vx, v_k_main_,
                                                       v_k_delta_)
            if self._is_bit(main_distance, delta_distance, bit_distance_):
                if self._timer3.is_time():
                    self._bit_c += 1
                    t = f'{self._bit_c}: \t{self._vx=:.2f} \n\t {main_bit_distance=:.2f} {delta_bit_distance=:.2f} \n \t{bit_distance_[0]=:.2f} {bit_distance_[1]=:.2f}'
                    print(t)
                    write(t)
                    try:
                        self._sender.stop_chassis()
                        if self._striker == 'delta':
                            self._sender.strike_delta()
                        else:
                            self._sender.strike_rod()
                    except Exception as exc:
                        print(f'strike command failed: {exc}')
                    self._timer3.start()

    def send_xiaomi(self, target, main_distance, xiaomi_bit_distance):
        self._timer5.is_time()
        self._timer6.is_time()
        if not self._timer5.is_open():
            self._sender.prepare_rod()
        if target:
            if main_distance < xiaomi_bit_distance and not self._timer6.is_open() and self._vx < -100 and self._z > 650:
                self._sender.strike_rod()
                self._timer5.start()  # 重新蓄力计时
                self._timer6.start()  # 激发间隔计时

    def send_v(self, target, target_delta, distance, delta_result,delta_track_distance):
        """
         x,y失效距离  跟球保持的距离
        :param target:
        :param target_delta:
        :param distance:主摄像头距离球的距离
        :param delta_result:
        :param delta_track_distance:
        :return:
        """
        if target is False:
            self._target_count += 1
            # print(self._target_count)
        else:
            self._target_count = 0
        if self._x > delta_track_distance:  # 用主摄像头调整
            # 使用前馈补偿调整pid目标值，从而调整输出
            self._pid_x.set_target(self._y_target + self._feedforward_y(self._vx, self._ax, -self._x))
            self._pid_x.set_now(self._x)
            self._pid_y.set_target(self._x_target + self._feedforward_x(self._vx, self._vy, self._ay))
            self._pid_y.set_now(self._y)
            # print(self._y)
            # print(self._pid_y.output())
            mx, my = self._pid_x.output() * self._k_x, self._pid_y.output() * self._k_y  # 乘上系数并限制范围
            mx, my = self._fix_xy(mx, my)  # 将摄像头坐标系转换为车坐标系
            mx = self._fix_vx(mx, distance)
            my = self._fix_vy(my, distance)
            # mx = 0.0
        else:  # 用delta调整
            if target_delta:
                x, y = delta_result[-1]
                self._pid_x_delta.set_now(x)
                self._pid_y_delta.set_now(y)
                mx = self._pid_x_delta.output() * self._k_x_d
                my = self._pid_y_delta.output() * self._k_y_d
            else:
                mx, my = 0.0, 0.0
        mx, my = abs_clip(mx, 3500), abs_clip(my, 3500)
        if self._target_count > 10:
            mx, my = 0.0, 0.0
        self._send_data[:3] = [mx, my, 0.0]

    def send(self, target, target_delta=False):
        """
        如果启用delta发送6个数据，否则发送前3个
        :return:
        """
        if target is False and target_delta is False:
            self._val_target = 0
            self._sender.stop_chassis()
        else:
            self._val_target += 1
        if self._val_target > 5:
            self._sender.set_chassis_speed(*self._send_data[:3])


class Detection:
    """
    开启多线程检测
    """

    def __init__(self, MODEL):
        self._run = False
        self._img = None
        self._target = False
        self._result = None
        self._model = MODEL
        self._new_img = False
        self._distance = 0.0
        self._threading = Threading(target=self.main)

    def main(self):
        while True:
            time.sleep(1e-7)
            if self._run:
                if self._img is not None and self._new_img is True:
                    self._new_img = False
                    self._target, self._result = detection_ball(self._img, self._model, self._distance)

    def get_result(self):
        return self._target, self._result

    def update_distance(self, distance):
        """
        更新距离，用于更新置信度
        :param distance:
        :return:
        """
        self._distance = distance

    def close(self):
        self._target = False
        self._run = False

    def update_img(self, img):
        self._img = img.copy()
        self._new_img = True

    def start(self):
        self._run = True


class Timer:
    """
    计时器 定时器
    """

    def __init__(self, time_step, no_repeat=False, default=True):
        self._time_step = time_step
        self._default = default
        self._open = False
        self._t_start = None
        self._no_repeat = no_repeat  # 如果开启no_repeat，在计时未完成时不重置

    def start(self):
        """
        开始计时
        :return:
        """
        if self._no_repeat:
            if not self._open:
                self._t_start = perf_counter()
                self._open = True
        else:
            self._t_start = perf_counter()
            self._open = True

    def is_open(self):
        return self._open

    def is_time(self):
        """
        判断是否到时间，未开始计时始终返回True，开始计时后没到时间前返回False
        :return:
        """
        now_t = perf_counter()
        if self._open:
            if now_t - self._t_start > self._time_step:
                self._open = False
                return True
            else:
                return False
        else:
            return self._default


class Filter:
    """
    均值滑动滤波
    """

    def __init__(self, k):
        self.k = k
        self.buffer = [0.0] * k
        self.index = 0

    def update(self, data):
        self.buffer[self.index] = data
        self.index = (self.index + 1) % self.k
        return round(sum(self.buffer) / self.k, 10)


def filter_data(filter_list, *args):
    """
    对传入的数据使用对应滤波器滤波
    :param filter_list:
    :param args:
    :return:
    """
    datas = []
    for filter_, data in zip(filter_list, args):
        datas.append(filter_.update(data))
    return datas


def transform_img(img):
    """
    目前没啥用，调色的
    :param img:
    :return:
    """
    img = img.astype(np.int32)
    img -= np.array([43, 41, 43])
    img = 1.05 * img ** 1.05
    img = cv2.convertScaleAbs(img, alpha=1.8)
    return img


def kf_filter_xyz(kf_list, *xyz):
    """
    传入卡尔曼滤波器列表和距离，返回每个轴的[x,v,a,j]
    :param kf_list:
    :param xyz:
    :return:
    """
    state = []
    for kf, distance in zip(kf_list, xyz):
        state_i = kf.update_get_position(np.array([distance], np.float32).reshape(1, 1))
        state.append(state_i / 0.3)  # 除以dt
        kf.get_pred_position()
    return state


def ball_process(img0, target, result, track, pnp, kf_bbox, info, params):
    """
    对检测结果进行处理
    :param img0:
    :param target:
    :param result:
    :param track: 跟踪器
    :param pnp: Pnp测距
    :param kf_bbox: 卡尔曼滤波边界框预测
    :param info: 图像可视化信息
    :param params: 目标跟踪的阈值 iou,distance,fps
    :return:
    """
    target_track = False
    if target:  # 如果找到目标，就使用卡尔曼滤波修正的坐标来结算
        d = pnp.s_calculate_distance(np.array(result[:-1]).reshape(4, 2).astype(np.float32))
        target_track = track.is_target(np.array(result[:4]), d, *params)

        if target_track:
            up, down, left, right = result[:-1]  # 取出上下左右点用于更新卡尔曼滤波
            kf_correct_bbox = kf_bbox.update_get_position(np.array([up, down, left, right], np.float32)
                                                          .reshape(8, 1)).astype(int)
            kf_pred_bbox = kf_bbox.get_pred_position().astype(int)

            # info.draw_ball_point(img0, *result)
            info.draw_ball_point(img0, *kf_correct_bbox, result[-1], color=(0, 255, 0))
            pnp.set_two_D(kf_correct_bbox.astype(np.float64))

    if target is False or target_track is False:  # 如果没有目标或者不是同一个目标，使用之前目标点的数据预测下一时刻
        kf_pred_bbox = kf_bbox.get_pred_position().astype(int)
        mid = (kf_pred_bbox[0, 0], kf_pred_bbox[3, 1])
        info.draw_ball_point(img0, *kf_pred_bbox, mid, color=(255, 0, 0))
        pnp.set_two_D(kf_pred_bbox.astype(np.float64))
    return target


def solve_angle(x, y, z):
    """
    ai从c翻译来的解算delta函数
    :param x:
    :param y:
    :param z:
    :return:
    """

    # 常数定义
    delta_PI = np.pi

    # 结构参数（单位：米）
    under_R = 0.09857  # 电机半径（底座到电机中心的距离）
    move_r = 0.09025  # 动平台半径
    u_L = 0.192  # 主动臂的长度
    u_l = 0.242  # 连杆的长度

    # 丝杆螺距（这里暂未使用到）
    s_S = 4.0  # 单位：mm（或 m，看你使用）

    # 电机角度限制函数
    def deltaLimitMax(input_angle):
        if input_angle > 90.0:
            return 90.0
        elif input_angle < -12.0:
            return -12.0
        else:
            return input_angle

    # Delta 逆解函数：输入末端平台(x, y, z)，返回三个电机角度（单位：度）
    def DeltaInversekinematic(x, y, z):
        # 三个电机安装角度，相位差（弧度）
        phi_list = [0, 2.0 / 3.0 * delta_PI, 4.0 / 3.0 * delta_PI]

        theta = []  # 存放三个电机角度

        for phi in phi_list:
            # 几何公式中 a, b, c 的计算
            a = 2 * u_L * (under_R - move_r - x * np.cos(phi) - y * np.sin(phi))
            b = -2 * u_L * z
            c = x ** 2 + y ** 2 + z ** 2 + u_L ** 2 + (under_R - move_r) ** 2 - u_l ** 2 \
                - 2 * (under_R - move_r) * (x * np.cos(phi) + y * np.sin(phi))

            discriminant = a ** 2 + b ** 2 - c ** 2

            if discriminant < 0:
                return [0.0, 0.0, 0.0]  # 判别式小于0，三角函数无实数解

            temp = (-b - np.sqrt(discriminant)) / (c - a)
            angle_deg = 2 * np.arctan(temp) * 180.0 / delta_PI

            theta.append(deltaLimitMax(angle_deg))

        # 检查结果是否有效
        if any(np.isnan(t) for t in theta):
            return [0.0, 0.0, 0.0]

        return theta

    return DeltaInversekinematic(x, y, z)


def write(text):
    with open('log.txt', 'a', encoding='utf-8') as f:
        f.write(text)
        f.write('\n')


def pred_time_va(x, v, a, xt):
    """
    给定当前状态和目标位置，预测到目标位置的时间
    :param x:
    :param v:
    :param a:
    :param xt:
    :return:
    """
    delta = v ** 2 + 2 * a * (x - xt)
    if delta < 0:
        return 1e10
    else:
        t1 = (-v + np.sqrt(delta)) / (a + 1e-7)
        t2 = (-v - np.sqrt(delta)) / (a + 1e-7)
        t = t1 if t1 > 0 else t2
        if t > 0:
            return t
        else:
            return 1e10


def pred_time_v(x, v, xt):
    """
    简单速度模型
    :param x:
    :param v:
    :param xt:
    :return:
    """
    t = (x - xt) / v
    return t if t > 0 else 1e10


def dynamic_confidence(distance):
    """
    根据目标距离动态调整检测置信度
    :param distance:
    :return:
    """
    BASE_CONF = 0.10  # 基础置信度
    MAX_CONF = 0.25  # 最大置信度
    MID = 1500  # 近距离阈值(mm)
    FAR = 6000  # 远距离阈值(mm)

    if distance <= MID:
        return BASE_CONF + (distance / MID) * 0.05

    elif distance < FAR:
        return 0.15 + (distance / FAR) * (MAX_CONF - 0.15)

    else:
        # 超过FAR后，置信度为0.35
        return 0.35


def detection_ball(img, model, distance):
    """
    用指定模型检测球
    return 检测结果(有目标为True), (up, down, left, right, mid)
    :param img:
    :param model:
    :return:
    """
    conf = dynamic_confidence(distance)  # 动态调整置信度
    # conf = 0.25
    res = model.predict(source=img, verbose=False, conf=conf)[0]
    result = False
    up, down, left, right, mid = None, None, None, None, None
    # res = res.cpu()
    for xyxy, cls in zip(res.boxes.xyxy, res.boxes.cls):
        result = True
        x1, y1, x2, y2 = map(int, xyxy)
        mx, my = (x1 + x2) // 2, (y1 + y2) // 2
        up, down, left, right = (mx, y1), (mx, y2), (x1, my), (x2, my)
        mid = (mx, my)
    return result, (up, down, left, right, mid)


def resize(img, dsize=(640, 480)):
    """
    统一分辨率
    :param img:
    :param dsize:
    :return:
    """
    return cv2.flip(cv2.resize(img, dsize), 1)


def clip(x, x_min, x_max):
    """
    限制标量值范围 [x_min, x_max]
    :param x:
    :param x_min:
    :param x_max:
    :return:
    """
    return max(min(x, x_max), x_min)


def abs_clip(x, x_max):
    """
    限制标量值范围 [-x_max, x_max]
    :param x:
    :param x_max:
    :return:
    """
    return max(min(x, x_max), -x_max)


# 可视化...
class InfoShow:
    def __init__(self, is_show):
        """
        感觉这么判断有点影响性能...想不出来了
        :param is_show:
        """
        self.is_show = is_show

    def draw_circle(self, img, center, color):
        if self.is_show:
            cv2.circle(img, center, 10, color, -1)

    def put_text(self, img, text, org, color, fontscale=1, thinkness=1):
        if self.is_show:
            cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, fontscale, color, thinkness, cv2.LINE_AA)

    def draw_ball_point(self, img, up, down, left, right, mid, color=(0, 0, 255)):
        if self.is_show:
            self.draw_circle(img, up, color)
            self.draw_circle(img, down, color)
            self.draw_circle(img, left, color)
            self.draw_circle(img, right, color)
            self.draw_circle(img, mid, (0, 255, 0))
        else:
            self.is_show = True
            self.draw_ball_point(img, up, down, left, right, mid)
            self.is_show = False


if __name__ == '__main__':
    # pid = PID(0.05, 0.03, 0.1, 10)
    # d = Dynamic_drawing.Drawer(1, 70)
    # now = -305
    # t = 0
    # pid.set_now(now)
    # pid.set_target(t)
    # for i in range(200):
    #     time.sleep(0.3)
    #     now += pid.output()
    #     if i == 30:
    #         pid.set_target(-500)
    #     print(now)
    #     d.update_datas(now)
    #     pid.set_now(now)
    ...
