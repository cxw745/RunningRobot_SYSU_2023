import time
import math
import threading
from enum import Enum, auto

import cv2
import numpy as np

import hiwonder.ActionGroupControl as AGC  # 运行动作文件
import hiwonder.Board as Board  # 控制总线电机

# 关卡状态枚举
class State(Enum):
    StateNone = auto()
    StateDebug = auto()
    StateHole = auto()
    StateBridge = auto()
    StateMine = auto()
    StateObstacle = auto()
    StateDoor = auto()
    StateStair = auto()


# ------------------------- 一、全局变量 ------------------------- #
# 关卡状态
state = State.StateDebug

# 关卡状态切换 [当前关卡: 下一关卡]
StateTrans = {
    State.StateHole: State.StateNone,
    State.StateBridge: State.StateDebug,
    State.StateMine: State.StateObstacle,
    State.StateObstacle: State.StateDoor,
    State.StateDoor: State.StateHole,
    State.StateStair: State.StateNone,
}

# 关卡状态处理函数, 通过 globals().get(StateCalls[state])(colors) 实现调用
StateCalls = {
    State.StateDebug: "state_debug",
    State.StateHole: "state_hole",
    State.StateBridge: "state_bridge",
    State.StateMine: "state_mine",
    State.StateObstacle: "state_obstacle",
    State.StateDoor: "state_door",
    State.StateStair: "state_stair",
}

# 关卡中用到的颜色
StateColors = {
    State.StateDebug: {"this": "white2", "next": "", "obj": ""},
    State.StateHole: {"this": "green", "next": "", "obj": ""},
    State.StateBridge: {"this": "green", "next": "", "obj": ""},
    State.StateMine: {"this": "white", "next": "green", "obj": "black"},
    State.StateObstacle: {"this": "white2", "next": "", "obj": ""},
    State.StateDoor: {"this": "red", "next": "green", "obj": "blue"},
    State.StateStair: {"this": "blue", "next": "", "obj": ["blue", "green", "red"]},
}

# LAB色彩范围
color_range = {
    "green": [(47, 0, 135), (255, 110, 255)],
    "white": [(100, 0, 0), (255, 255, 255)],
    "white2": [(155, 0, 0), (255, 255, 255)],
    "black": [(0, 0, 0), (80, 255, 255)],
    "red": [(0, 138, 88), (255, 255, 255)],
    "blue": [(150, 80, 0), (255, 255, 120)],
    "green_hole": [(47, 0, 135), (255, 110, 255)],
    "blue_bridge": [(0, 0, 0), (255, 175, 94)],
    "ball": [(0, 0, 0), (20, 255, 255)],
    "white_road": [(193, 0, 0), (255, 255, 255)],
    "white_door": [(140, 0, 0), (255, 255, 255)],
    "yellow": [(100, 140, 160), (225, 190, 190)],
    "yellow_start": [(11, 42, 71), (225, 190, 190)],
    "yellow_end": [(0, 0, 151), (225, 141, 255)],
    "ball_hole": [(0, 137, 0), (255, 255, 127)],
    "barrier_y": [(0, 83, 162), (200, 255, 255)],
    "barrier_k": [(0, 0, 0), (28, 255, 255)],
    "gate_bule": [(0, 0, 0), (255, 255, 115)],
    "road_white": [(134, 0, 0), (255, 255, 255)],
}

# 图像帧
frame = None  # 原始帧
frame_cali = None  # 校正帧
frame_flag = False  # 是否准备好
frame_show = False  # 是否显示

# 动作组路径及常用动作
action_path = "/home/pi/AiNexPro/AiNexPro/wsh1/"
Actions = {
    "stand": "Stand",
    "go_s": "LIttlego1",
    "go_L": "STAndgo",
    "go_L_2": "Standgo2",
    "go_L_3": "Standgo3",
    "back_s": "little_back",
    "back_L": "BAck",
    "turn_left_s": "turn_left1",
    "turn_left_m": "turn_left2",
    "turn_left_L": "turn_left3",
    "turn_right_s": "turn_right1",
    "left_go_s": "LEftgo",
    "left_go_L": "LEftgoB1",
    "right_go_s": "Rightgo-1",
    "right_go_m": "Rightgo",
    "right_go_L": "Rightgo1",
    "squat": "squat",
    "obstacle": "overrail",
    "up_and_go": "upANDgo1",
}

# ------------------------- 二、相机 ------------------------- #
# 鱼眼校正参数
calibration_param_path = "/home/pi/AiNexPro/CameraCalibration/"  # 参数路径
param_data = np.load(calibration_param_path + "calibration_param.npz")
dim = tuple(param_data["dim_array"])
k = np.array(param_data["k_array"].tolist())
d = np.array(param_data["d_array"].tolist())
p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None)  # 优化内参和畸变参数
map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), p, dim, cv2.CV_16SC2)


class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.frame_cali = None
        self.opened = False

        # 以子线程的形式获取图像
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self):  # 开启
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("Y", "U", "Y", "V")
            )
            self.cap.set(cv2.CAP_PROP_FPS, 30)  # 帧率
            # self.cap.set(cv2.CAP_PROP_SATURATION, 40) # 饱和度
            self.opened = True
        except Exception as e:
            print("打开摄像头失败:", e)

    def camera_close(self):  # 关闭
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print("关闭摄像头失败:", e)

    def camera_task(self):  # 获取摄像头画面线程
        global frame, frame_cali, frame_flag
        while True:
            try:
                if self.opened and self.cap.isOpened():  # 判断是否开启
                    ret, frame_tmp = self.cap.read()  # 获取画面
                    if ret:
                        # 图像缩放与校正
                        self.frame = cv2.resize(
                            frame_tmp,
                            (self.width, self.height),
                            interpolation=cv2.INTER_NEAREST,
                        )
                        self.frame_cali = cv2.remap(
                            self.frame,
                            map1,
                            map2,
                            interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT,
                        )
                        frame = self.frame
                        frame_cali = self.frame_cali
                        frame_flag = True
                    else:
                        # 如果获取画面失败，则尝试重新打开摄像头
                        self.frame = None
                        self.frame_cali = None
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap

                elif self.opened:
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap
                else:
                    time.sleep(0.01)
            except Exception as e:
                print("获取摄像头画面出错:", e)
                time.sleep(0.01)


# ------------------------- 三、图像处理 ------------------------- #
class line:
    """线段, 可计算中点、长度、角度"""

    def __init__(self, p0, p1):
        self.point0 = p0
        self.point1 = p1

    def mid_point(self):
        return (self.point0[0] + self.point1[0]) // 2, (
            self.point0[1] + self.point1[1]
        ) // 2

    def length(self):
        return math.sqrt(
            math.pow(self.point1[1] - self.point0[1], 2)
            + math.pow(self.point1[0] - self.point0[0], 2)
        )

    def angle(self):
        if self.point0[0] == self.point1[0]:
            return 90
        else:
            return (
                -math.atan(
                    (self.point1[1] - self.point0[1])
                    / (self.point1[0] - self.point0[0])
                )
                * 180
                / math.pi
            )


def get_frame_bin(img, color, ksize_smooth=3, ksize_morph=3):
    """获取二值化图像（颜色）

    Args:
        img (ndarray): 源图像
        color (str): 颜色名称
        ksize_smooth (int, optional): 高斯模糊核大小. Defaults to 3.
        ksize_morph (int, optional): 开闭运算核大小. Defaults to 3.

    Returns:
        ndarray: 二值化图像
    """
    img_smooth = cv2.GaussianBlur(img, (ksize_smooth, ksize_smooth), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(
        img_transform, color_range[color][0], color_range[color][1]
    )
    kernel = np.ones((ksize_morph, ksize_morph), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    return close


def get_maxcontour(contours, area_threshold=1000):
    """获得最大轮廓及其面积

    Args:
        contours (list[cnt]): 轮廓列表
        area_threshold (int, optional): 面积阈值. Defaults to 1000.

    Returns:
        cnt, float: 最大轮廓, 面积
    """
    cnt_max = None
    area_max = 0
    if contours:
        cnt = max(contours, key=cv2.contourArea)

        if cv2.contourArea(cnt) > area_threshold:
            cnt_max = cnt
            area_max = cv2.contourArea(cnt)

    return cnt_max, area_max


def sort_contours_by_LT(cnts, method="left-to-right"):
    """轮廓按左上点排序

    Args:
        cnts (tuple): 轮廓
        method (str, optional): 可选顺序"left-to-right", "right-to-left", "bottom-to-top", "top-to-bottom". Defaults to "left-to-right".

    Returns:
        tuple: (cnts, boundingBoxes)
    """
    # initialize the reverse flag and sort index
    reverse = False
    i = 0
    # handle if sort in reverse
    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True
    # handle if sort against y rather than x of the bounding box
    if method == "bottom-to-top" or method == "top-to-bottom":
        i = 1

    boundingBoxes = [cv2.boundingRect(c) for c in cnts]
    (cnts, boundingBoxes) = zip(
        *sorted(zip(cnts, boundingBoxes), key=lambda b: b[1][i], reverse=reverse)
    )
    return (cnts, boundingBoxes)


def sort_contours_by_A(cnts, reverse=False):
    """轮廓按面积升序排序

    Args:
        cnts (tuple): 轮廓
        reverse (bool, optional): 默认升序, True 为降序. Defaults to False.

    Returns:
        tuple: (cnts, boundingBoxes)
    """

    contourAreas = [cv2.contourArea(c) for c in cnts]
    (cnts, contourAreas) = zip(
        *sorted(zip(cnts, contourAreas), key=lambda b: b[1], reverse=reverse)
    )
    return (cnts, contourAreas)


def color_detect_top(color, grad=1.0, ksizes=None, defaults=None):
    """返回目标区域占比, 顶部角度, 顶部中点[x,y], 顶部左点[x,y], 顶部右点[x,y]

    Args:
        color (str): 颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与开闭运算核. Defaults to [3, 3].
        defaults (list, optional): 缺省[顶部角度, 顶部中点[x, y], 顶部左点[x, y], 顶部右点[x, y]]. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        float, float, list: 目标区域占比, 顶部角度, [顶部中点[x, y], 顶部左点[x, y], 顶部右点[x, y]]
    """
    if ksizes is None:
        ksizes_ = [3, 3]
    else:
        ksizes_ = ksizes

    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    # 图像处理及roi提取
    global frame_cali
    img = frame_cali.copy()
    close = get_frame_bin(img, color, ksizes_[0], ksizes_[1])
    cnts, _ = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))  # 计算目标区域占比

    # 得到上顶点，计算中点及线段角度
    if cnt_max is None:
        top_angle, top_center, top_left, top_right = defaults_
    else:
        top_left = cnt_max[0][0]
        top_right = cnt_max[0][0]
        for c in cnt_max:  # 遍历找到四个顶点
            if c[0][0] + grad * c[0][1] < top_left[0] + grad * top_left[1]:
                top_left = c[0]
            if -c[0][0] + grad * c[0][1] < -top_right[0] + grad * top_right[1]:
                top_right = c[0]
        line_top = line(top_left, top_right)
        top_angle = line_top.angle()
        top_center = line_top.mid_point()  # 需要转换成tuple使用

    # 显示
    if frame_show:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            "area: " + str(percent) + "%",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            f"angle: {top_angle:.2f}",
            (50, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "center(x,y): " + str(top_center[0]) + "," + str(top_center[1]),
            (50, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "left(x,y): " + str(top_left[0]) + "," + str(top_left[1]),
            (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "right(x,y): " + str(top_right[0]) + "," + str(top_right[1]),
            (50, 250),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.line(vis_img, tuple(top_left), tuple(top_right), (255, 255, 255), 3)
        cv2.circle(vis_img, tuple(top_center), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(top_left), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(top_right), 5, (0, 255, 255), 2)
        cv2.imshow("vis_img", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)

    return percent, top_angle, [top_center, top_left, top_right]


def color_detect_bottom(color, grad=1.0, ksizes=None, defaults=None):
    """返回目标区域占比, 底部角度, 底部中点[x,y], 底部左点[x,y], 底部右点[x,y]

    Args:
        color (str): 颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与开闭运算核. Defaults to [3, 3].
        defaults (list, optional): 缺省[底部角度, 底部中点[x, y], 底部左点[x, y], 底部右点[x, y]]. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        float, float, list: 目标区域占比, 底部角度, [底部中点[x, y], 底部左点[x, y], 底部右点[x, y]]
    """
    if ksizes is None:
        ksizes_ = [3, 3]
    else:
        ksizes_ = ksizes

    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    # 图像处理及roi提取
    global frame_cali
    img = frame_cali.copy()
    close = get_frame_bin(img, color, ksizes_[0], ksizes_[1])
    cnts, _ = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))

    if cnt_max is None:
        bottom_angle, bottom_center, bottom_left, bottom_right = defaults_
    else:
        bottom_left = cnt_max[0][0]
        bottom_right = cnt_max[0][0]
        for c in cnt_max:  # 遍历找到四个顶点
            if c[0][0] - grad * c[0][1] < bottom_left[0] - grad * bottom_left[1]:
                bottom_left = c[0]
            if c[0][0] + grad * c[0][1] > bottom_right[0] + grad * bottom_right[1]:
                bottom_right = c[0]
        line_bottom = line(bottom_left, bottom_right)
        bottom_angle = line_bottom.angle()
        bottom_center = line_bottom.mid_point()  # 需要转换成tuple使用

    if frame_show:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            "area: " + str(percent) + "%",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            f"angle: {bottom_angle:.2f}",
            (50, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "center(x,y): " + str(bottom_center[0]) + "," + str(bottom_center[1]),
            (50, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "left(x,y): " + str(bottom_left[0]) + "," + str(bottom_left[1]),
            (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "right(x,y): " + str(bottom_right[0]) + "," + str(bottom_right[1]),
            (50, 250),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.line(vis_img, tuple(bottom_left), tuple(bottom_right), (255, 255, 255), 3)
        cv2.circle(vis_img, tuple(bottom_center), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(bottom_left), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(bottom_right), 5, (0, 255, 255), 2)
        cv2.imshow("vis_img", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)

    return percent, bottom_angle, [bottom_center, bottom_left, bottom_right]


def color_detect_mine_near(color_back, color_obj, ksizes=None, area=None, default=None):
    if ksizes is None:
        ksizes_ = [(3, 3), (3, 3)]
    else:
        ksizes_ = ksizes

    if area is None:
        area_min = 500
        area_max = 3000
    else:
        area_min = area[0]
        area_max = area[1]

    if default is None:
        default_ = (300, 0, 30, 30)
    else:
        default_ = default

    # 提取 roi
    global frame_cali
    img = frame_cali.copy()
    img_bin = get_frame_bin(img, color_back, ksizes_[0][0], ksizes_[0][1])
    cnts, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnt_max, _ = get_maxcontour(cnts)

    if cnt_max is None:
        img_roi = img.copy()
    else:
        cnt_hull = cv2.convexHull(cnt_max)
        img_zero = np.zeros_like(img, np.uint8)
        img_mask = cv2.fillConvexPoly(img_zero, cnt_hull, (255, 255, 255))
        img_mask = cv2.bitwise_not(img_mask, img_mask)
        img_roi = cv2.bitwise_or(img, img_mask)

    img_mine_bin = get_frame_bin(img_roi, color_obj, ksizes_[1][0], ksizes_[1][1])
    cnts, _ = cv2.findContours(img_mine_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if cnts:
        boundingBoxs = [cv2.boundingRect(c) for c in cnts]
        mines = [box for box in boundingBoxs if area_min < box[2] * box[3] < area_max]
        mine_near = max(mines, key=lambda m: m[1])
    else:
        mine_near = default_

    x, y, w, h = mine_near
    mine_x = x + w // 2
    mine_y = y

    if frame_show:
        img_show = img_roi.copy()
        cv2.putText(
            img_show,
            f"mine near (x,y): ({x},{y})",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.rectangle(img_show, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.circle(img_show, (mine_x, mine_y), 2, (0, 255, 255), 2)
        cv2.imshow("mine", img_show)
        cv2.waitKey(1)

    return (mine_x, mine_y)


def color_detect_sill_bottom(color, grad=1.0, ksizes=None, defaults=None):
    if ksizes is None:
        ksizes_ = [3, 3]
    else:
        ksizes_ = ksizes
    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    global frame_cali
    img = frame_cali.copy()
    img_bin = get_frame_bin(img, color, ksizes_[0], ksizes_[1])
    cnts, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(cnts) == 2:
        cnts_sorted, _ = sort_contours_by_LT(cnts, method="bottom-to-top")
        cnt_bottom = cnts_sorted[0]

        bottom_left = cnt_bottom[0][0]
        bottom_right = cnt_bottom[0][0]
        for c in cnt_bottom:  # 遍历找到四个顶点
            if c[0][0] - grad * c[0][1] < bottom_left[0] - grad * bottom_left[1]:
                bottom_left = c[0]
            if c[0][0] + grad * c[0][1] > bottom_right[0] + grad * bottom_right[1]:
                bottom_right = c[0]
        line_bottom = line(bottom_left, bottom_right)
        bottom_angle = line_bottom.angle()
        bottom_center = line_bottom.mid_point()  # 需要转换成tuple使用
    else:
        bottom_angle, bottom_center, bottom_left, bottom_right = defaults_

    if frame_show:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            f"angle: {bottom_angle:.2f}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "center(x,y): " + str(bottom_center[0]) + "," + str(bottom_center[1]),
            (50, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "left(x,y): " + str(bottom_left[0]) + "," + str(bottom_left[1]),
            (50, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "right(x,y): " + str(bottom_right[0]) + "," + str(bottom_right[1]),
            (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.line(vis_img, tuple(bottom_left), tuple(bottom_right), (255, 255, 255), 3)
        cv2.circle(vis_img, tuple(bottom_center), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(bottom_left), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(bottom_right), 5, (0, 255, 255), 2)
        cv2.imshow("sill_bottom", vis_img)  # 显示图像
        cv2.waitKey(1)

    return (bottom_angle, [bottom_center, bottom_left, bottom_right])


def color_detect_door(color_back, color_obj, grad, ksizes=None, defaults=None):
    if ksizes is None:
        ksizes_ = [(3,3), (3,3)]
    else:
        ksizes_ = ksizes
    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    global frame_cali
    img = frame_cali.copy()
    img_bin = get_frame_bin(img, color_back, ksizes_[0][0], ksizes_[0][1])
    cnts, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnt_max, _ = get_maxcontour(cnts)

    if cnt_max is None:
        angle, center, left, right = defaults_
    else:
        cnt_hull = cv2.convexHull(cnt_max)
        img_zero = np.zeros_like(img, np.uint8)
        img_mask = cv2.fillConvexPoly(img_zero, cnt_hull, (255, 255, 255))
        img_roi = cv2.bitwise_and(img, img_mask)

        img_roi_bin = get_frame_bin(img_roi, color_obj, ksizes_[1][0], ksizes_[1][1])
        cnts_roi, _ = cv2.findContours(
            img_roi_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )


        if len(cnts_roi) < 2:
            angle, center, left, right = defaults_
        else:
            cnts_max_two, _ = sort_contours_by_A(cnts_roi)
            cnts_sorted, _ = sort_contours_by_LT(cnts_max_two[:2])
            cnt_left, cnt_right = cnts_sorted

            # 左侧轨迹右下点
            left = cnt_left[0][0]
            for p in cnt_left:
                if p[0][0] + grad * p[0][1] > left[0] + grad * left[1]:
                    left = p[0]

            # 右侧轨迹左下点
            right = cnt_right[0][0]
            for p in cnt_right:
                if -p[0][0] + grad * p[0][1] > -right[0] + grad * right[1]:
                    right = p[0]

            line_door = line(left, right)
            angle = line_door.angle()
            center = line_door.mid_point()

    if frame_show:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            f"angle: {angle:.2f}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "center(x,y): " + str(center[0]) + "," + str(center[1]),
            (50, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "left(x,y): " + str(left[0]) + "," + str(left[1]),
            (50, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "right(x,y): " + str(right[0]) + "," + str(right[1]),
            (50, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.line(vis_img, tuple(left), tuple(right), (255, 255, 255), 3)
        cv2.circle(vis_img, tuple(center), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(left), 5, (0, 255, 255), 2)
        cv2.circle(vis_img, tuple(right), 5, (0, 255, 255), 2)
        cv2.imshow("door", vis_img)  # 显示图像
        cv2.imshow("img_roi", img_roi)
        cv2.imshow("img_roi_bin", img_roi_bin)
        cv2.waitKey(1)

    return (angle, [center, left, right])


# ------------------------- 四、初始化函数 ------------------------- #
def init_action(action_name=Actions["stand"]):
    """初始化动作

    Args:
        action_name (str, optional): 动作名称. Defaults to "stand".
    """
    AGC.runAction(action_name, 1, action_path)  # 站立


def init_camera_platform(left=500, top=500):
    """初始化相机云台

    Args:
        left (int, optional): 左右位置. Defaults to 500.
        top (int, optional): 上下位置. Defaults to 500.
    """
    # setBusServoPulse(id, pulse, use_time) param: id, 位置, 运行时间
    Board.setBusServoPulse(19, left, 500)  # id=19 左右
    time.sleep(0.1)  # 延时时间和运行时间相同
    Board.setBusServoPulse(20, top, 500)  # id=20 上下
    time.sleep(0.1)


# ------------------------- 五、局部调整函数 ------------------------- #
def adjust_x(x, x_range):
    """水平调整x位置

    Args:
        x (int): x
        x_range (list[int]): x范围[min, max]
    """
    if x < x_range[0]:
        print("adjust:x =", x, "left_go")
        AGC.runAction(Actions["left_go_s"], 1, action_path)
        # time.sleep(0.25)
    elif x > x_range[1]:
        print("adjust:x =", x, "right_go")
        AGC.runAction(Actions["right_go_s"], 1, action_path)
        # time.sleep(0.25)
    else:
        print("adjust:x =", x, "stand")


def adjust_angle(angle, angle_range):
    """调整角度

    Args:
        angle (float): 角度
        angle_range (int[float]): 角度范围[min, max]
    """
    if angle < angle_range[0]:
        print("adjust:angle =", angle, "turn_right")
        AGC.runAction(Actions["turn_right_s"], 1, action_path)
        # time.sleep(0.5)
    elif angle > angle_range[1]:
        print("adjust:angle =", angle, "turn_left")
        AGC.runAction(Actions["turn_left_s"], 1, action_path)
        # time.sleep(0.5)
    else:
        print("adjust:angle =", angle, "stand")


# ------------------------- 六、关卡状态 ------------------------- #
def state_debug(colors):
    """state:debug

    Args:
        color_this (str, optional): 关卡颜色. Defaults to "green".
        camera_left (int, optional): 相机云台水平位置. Defaults to 500.
        camera_top (int, optional): 相机云台垂直位置. Defaults to 500.
    """
    cv2.destroyAllWindows()
    global state, frame, frame_cali, frame_show
    frame_show = True
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, (320, 240), (300, 240), (340, 240)]

    print("state:debug:enter")
    while state == State.StateDebug:
        color_detect_top(colors["this"], grad, ksizes, defaults)


def state_hole(colors):
    """state:hole

    Args:
        color_this (str, optional): 关卡颜色. Defaults to "green".
    """
    global state, frame, frame_cali
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, (320, 240), (300, 240), (340, 240)]
    left_range = [210, 230]
    angle_range = [-3, 3]
    left_flag = 0
    camera_flag = 0
    step = 0

    init_action()
    init_camera_platform(500, 400)
    print("state:hole:enter")
    while state == State.StateHole:
        while step == 0:
            time.sleep(0.25)
            _, top_angle, points = color_detect_top(
                colors["this"], grad, ksizes, defaults
            )
            top_center = points[0]
            # 调整角度
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue

            if top_center[1] < 90:
                AGC.runAction(Actions["go_L"], 1, action_path)
                continue

            if top_center[1] < 120:
                AGC.runAction(Actions["go_s"], 1, action_path)
                continue

            # 进入下一步
            print("state:hole:step 0 done, goto step 1")
            step = 1
            break
        while step == 1:
            # 识别顶部角度和左点
            time.sleep(0.25)
            _, top_angle, points = color_detect_top(
                colors["this"], grad, ksizes, defaults
            )
            top_left = points[1]

            # 调整角度
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue

            # 左平移至目标点
            if not left_range[0] < top_left[0] < left_range[1]:
                # 大步左平移
                if left_flag == 0:
                    AGC.runAction(Actions["left_go_L"], 3, action_path)
                    left_flag = 1
                    continue
                # 小步调整
                if left_flag == 1:
                    adjust_x(top_left[0], left_range)
                    continue

            # 进入下一步
            print("state:hole:step 1 done, goto step 2")
            step = 2
            break
        while step == 2:
            # 识别顶部角度和左点
            time.sleep(0.25)
            _, top_angle, points = color_detect_top(
                colors["this"], grad, ksizes, defaults
            )
            top_left = points[1]
            # 调整角度
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue
            # 前进至目标点
            if camera_flag == 0:
                if top_left[1] < 280:
                    # 大步向前
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue
                else:
                    # 低头
                    init_camera_platform(500, 300)
                    camera_flag = 1
                    continue
            
            if camera_flag == 1:    
                if top_left[1] < 340:
                    # 小步向前
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue
                else:
                    # 进入下一关
                    print("state:hole:exit")
                    state = StateTrans[State.StateHole]
                    break


def state_bridge(colors):
    global state, frame, frame_cali
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, (320, 240), (300, 240), (340, 240)]
    center_x_range = [310, 330]
    angle_range = [-3, 3]
    camera_flag = 0
    step = 0

    
    print("state:bridge:enter")
    while state == State.StateBridge:
        if step == -1:
            init_action()
            init_camera_platform(500, 300)
            while step == -1:
                time.sleep(0.25)
                _, top_angle, points = color_detect_bottom(
                    colors["this"], grad, ksizes, defaults
                )
        if step == 0:
            angle_range = [-4, 0]
            center_y_stop = 310
            init_action()
            init_camera_platform(500, 300)
            while step == 0:
                time.sleep(0.25)
                _, angle, points = color_detect_bottom(
                    colors["this"], grad, ksizes, defaults
                )
                center = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if not center_x_range[0] < center[0] < center_x_range[1]:
                    adjust_x(center[0], center_x_range)
                    continue

                if center[1] < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                print("state:bridge:step 0 done, goto step 1")
                step = 1
                break

        if step == 1:
            defaults = [-2, (320, 375), (300, 375), (340, 375)]
            center_y_switch = 370
            center_y_stop = 370
            angle_range = [-3, 3]
            init_camera_platform(500, 500)
            while step == 1:
                time.sleep(0.25)
                _, top_angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                top_center = points[0]

                # 调整角度
                if not angle_range[0] < top_angle < angle_range[1]:
                    adjust_angle(top_angle, angle_range)
                    continue

                if camera_flag == 0:
                    if top_center[1] > center_y_switch:
                        init_camera_platform(500, 400)
                        camera_flag = 1
                        continue
                    else:
                        AGC.runAction(Actions["go_L"], 1, action_path)
                        continue
                else:
                    if top_center[1] > center_y_stop:
                        print("state:bridge:step 1 done, goto step 2")
                        init_camera_platform(500, 300)
                        step = 2
                        break
                    else:
                        AGC.runAction(Actions["go_s"], 1, action_path)
                        continue
        if step == 2:
            defaults = [-2, (320, 361), (300, 361), (340, 361)]
            center_y_stop = 360
            angle_range = [-3, 3]
            while step == 2:
                time.sleep(0.5)
                _, top_angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                top_center = points[0]

                # 调整角度
                if not angle_range[0] < top_angle < angle_range[1]:
                    adjust_angle(top_angle, angle_range)
                    continue

                if top_center[1] < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue
                
                AGC.runAction(Actions["go_L"], 1, action_path)
                print("state:bridge:exit")
                state = StateTrans[State.StateBridge]
                break
                


def state_mine(colors):
    global state, frame, frame_cali
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, (320, 240), (300, 240), (340, 240)]
    angle_range = [-3, 3]
    head_flag = 0
    step = 0

    init_action()
    print("state:mine:enter")
    while state == State.StateMine:
        while step == -1:
            init_camera_platform(500, 340)
            defaults = [-2, (320, 240), (300, 240), (340, 240)]
            # elps_x, elps_y = color_detect_ellipse_near(colors["obj"], ksizes)

        if step == 0:
            defaults = [-2, (320, 410), (300, 410), (340, 410)]
            y_switch = 370
            y_stop = 400

            # 调整相机高度
            if head_flag == 0:
                init_camera_platform(500, 500)
            else:
                init_camera_platform(500, 400)

            while step == 0:
                time.sleep(0.25)
                _, bottom_angle, points = color_detect_bottom(
                    colors["next"], grad, ksizes, defaults
                )
                bottom_center = points[0]

                # 调整角度
                if not angle_range[0] < bottom_angle < angle_range[1]:
                    adjust_angle(bottom_angle, angle_range)
                    continue

                # 低头前，判断是否低头
                if head_flag == 0 and bottom_center[1] > y_switch:
                    head_flag = 1
                    continue

                # 低头后，判断是否停止
                if head_flag == 1 and bottom_center[1] > y_stop:
                    # 进入下一关
                    print("state:mine:exit")
                    state = StateTrans[State.StateMine]
                    break

                # 进入下一步
                print("state:mine:step 0 done, goto step 1")
                step = 1
                break

        if step == 1:
            defaults = [-2, (320, 0), (300, 0), (340, 0)]
            go_s_num = 0
            go_s_num_max = 5
            y_stop = 320
            mine_y_stop = 350
            mine_x_range = [120, 530]

            # 低头看地雷
            init_camera_platform(500, 340)
            while step == 1:
                time.sleep(0.25)
                # 识别门槛底部
                _, bottom_angle, points = color_detect_bottom(
                    colors["next"], grad, ksizes, defaults
                )
                bottom_center = points[0]

                # 判断下一关距离
                if bottom_center[1] > y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    step = 0
                    print("state:mine:step 1 done, goto step 0:1")
                    continue

                # 识别地雷
                mine_x, mine_y = color_detect_mine_near(
                    colors["this"], colors["obj"], ksizes
                )

                # 地雷距离远或没有地雷，向前走
                if mine_y < mine_y_stop:
                    if go_s_num < go_s_num_max:
                        AGC.runAction(Actions["go_s"], 1, action_path)
                        go_s_num += 1
                    else:
                        step = 0
                        print("state:mine:step 1 done, goto step 0:2")
                    continue

                # 地雷距离近且在面前，左右调整
                if mine_x_range[0] < mine_x < mine_x_range[1]:
                    if mine_x > 300:
                        AGC.runAction(Actions["left_go_L"], 1, action_path)
                        continue
                    else:
                        AGC.runAction(Actions["right_go_L"], 1, action_path)
                        continue

                # 进入下一步
                print("state:mine:step 1 done, goto step 2")
                step = 2

        if step == 2:
            defaults = [-2, [320, 240], [300, 240], [340, 240]]
            y_switch = 370
            y_stop = 400

            if head_flag == 0:
                init_camera_platform(500, 500)
            else:
                init_camera_platform(500, 400)

            while step == 2:
                time.sleep(0.25)
                _, bottom_angle, points = color_detect_bottom(
                    colors["next"], grad, ksizes, defaults
                )
                bottom_center = points[0]

                # 调整角度
                if not angle_range[0] < bottom_angle < angle_range[1]:
                    adjust_angle(bottom_angle, angle_range)
                    continue

                AGC.runAction(Actions["go_L"], 1, action_path)
                AGC.runAction(Actions["go_s"], 3, action_path)
                step = 0


def state_obstacle(colors):
    global state, frame, frame_cali
    grad = 1.5
    ksizes = [11, 7]
    defaults = [-2, [320, 240], [300, 240], [340, 240]]
    angle_range = [-1, 3]
    center_x_range = [280, 320]
    step = 3
    print("state:obstacle:enter")
    while state == State.StateObstacle:
        if step == 0:
            init_camera_platform(500, 450)
            while step == 0:
                time.sleep(0.25)
                _, top_angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                top_center = points[0]

                # 调整角度
                if not angle_range[0] < top_angle < angle_range[1]:
                    adjust_angle(top_angle, angle_range)
                    continue

                if not center_x_range[0] < top_center[0] < center_x_range[1]:
                    adjust_x(top_center[0], center_x_range)
                    continue

                if top_center[1] < 140:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue
                
                step = 2
                break
        if step == 2:
            while step == 2:
                AGC.runAction(Actions["obstacle"], 1, action_path)
                AGC.runAction(Actions["back_L"], 7, action_path)
                step = 999
                state = StateTrans[State.StateObstacle]
                print("state:obstacle:exit")
                break

        if step == 3:
            while step == 3:
                # AGC.runAction(Actions["obstacle"], 1, action_path)
                AGC.runAction(Actions["back_L"], 2, action_path)
                AGC.runAction(Actions["turn_left_L"], 1, action_path)
                AGC.runAction(Actions["back_L"], 4, action_path)
                step = 999
                state = StateTrans[State.StateObstacle]
                print("state:obstacle:exit")
                break


def state_door(colors):
    global state, frame, frame_cali
    step = 10
    print("state:door:enter")
    while state == State.StateDoor:
        if step == -1:
            grad = 1.5
            ksizes = [(11, 7), (11, 7)]
            defaults = [-2, [320, 385], [300, 385], [340, 385]]
            init_camera_platform(888, 390)
            while step == -1:
                # _, angle, points = color_detect_top(
                #     colors["this"], grad, ksizes, defaults
                # )
                angle, points = color_detect_door(
                    colors["this"], colors["obj"], grad, ksizes, defaults
                )

        if step == 1:
            grad = 1.5
            ksizes = [(15, 7), (17, 15)]
            defaults = [-2, [320, 385], [300, 385], [340, 385]]
            angle_range = [-6, -1]
            center_x_range = [310, 330]
            init_camera_platform(500, 320)
            while step == 1:
                time.sleep(0.25)
                angle, points = color_detect_door(
                    colors["this"], colors["obj"], grad, ksizes, defaults
                )
                center = points[0]

                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if not center_x_range[0] < center[0] < center_x_range[1]:
                    adjust_x(center[0], center_x_range)
                    continue

                AGC.runAction(Actions["go_L"], 3, action_path)

                print("state:door:step 1 done, goto step 2")
                step = 2
                break


        if step == 10:
            grad = 1.5
            ksizes = [(11, 7), (11, 7)]
            defaults = [-2, [320, 385], [300, 385], [340, 385]]
            angle_range = [-6, -2]
            center_x_range = [350, 380]
            AGC.runAction(Actions["squat"], 1, action_path)
            init_camera_platform(888, 390)
            while step == 10:
                time.sleep(0.5)
                angle, points = color_detect_door(
                    colors["this"], colors["obj"], grad, ksizes, defaults
                )
                center = points[0]
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    time.sleep(0.1)
                    AGC.runAction(Actions["squat"], 1, action_path)
                    time.sleep(0.1)
                    continue

                if center[0] > center_x_range[1]:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    time.sleep(0.1)
                    AGC.runAction(Actions["squat"], 1, action_path)
                    time.sleep(0.1)
                    continue
                elif center[0] < center_x_range[0]:
                    AGC.runAction(Actions["back_s"], 1, action_path)
                    time.sleep(0.1)
                    AGC.runAction(Actions["squat"], 1, action_path)
                    time.sleep(0.1)
                    continue
                
                AGC.runAction(Actions["left_go_L"], 8, action_path)
                AGC.runAction(Actions["turn_left_L"], 6, action_path)
                state = StateTrans[State.StateDoor]
                print("state:door:exit")
                break

def state_stair(colors):
    global state, frame, frame_cali
    step = -1
    print("state:stair:enter")
    while state == State.StateStair:
        if step == -1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, [320, 240], [300, 240], [340, 240]]
            init_action()
            init_camera_platform(500, 320)
            while step == -1:
                _, angle, points = color_detect_top(colors["obj"][1], grad, ksizes, defaults)

        if step == 0:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [0, (310, 240), (300, 240), (320, 240)]
            angle_range = [-3, 3]
            center_x_range = [280, 340]
            center_y_stop = 350
            init_action()
            init_camera_platform(500, 450)
            while step == 0:
                time.sleep(0.25)
                _, angle, points = color_detect_bottom(colors["this"], grad, ksizes, defaults)
                center = points[0]
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if not center_x_range[0] < center[0] < center_x_range[1]:
                    adjust_x(center[0], center_x_range)
                    continue

                if center[1] < center_y_stop:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue
                
                print("state:stair:step 0 done, goto step 1")
                step = 1
                break
        
        if step == 1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [0, (310, 240), (300, 240), (320, 240)]
            angle_range = [-6, 1]
            center_x_range = [300, 340]
            center_y_stop = 250
            init_action()
            init_camera_platform(500, 320)
            while step == 1:
                time.sleep(0.25)
                _, angle, points = color_detect_bottom(colors["this"], grad, ksizes, defaults)
                center = points[0]
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if not center_x_range[0] < center[0] < center_x_range[1]:
                    adjust_x(center[0], center_x_range)
                    continue

                if center[1] < center_y_stop:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue
                
                print("state:stair:step 1 done, goto step 2")
                step = 2
                break

        if step == 2:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-3, (310, 240), (300, 240), (320, 240)]
            angle_range = [-5, -1]
            center_x_range = [300, 340]
            center_y_stop = 180
            init_action()
            init_camera_platform(500, 320)
            while step == 2:
                time.sleep(0.25)
                _, angle, points = color_detect_top(colors["this"], grad, ksizes, defaults)
                center = points[0]
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if not center_x_range[0] < center[0] < center_x_range[1]:
                    adjust_x(center[0], center_x_range)
                    continue

                if center[1] < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue
                
                # AGC.runAction(Actions["go_s"], 1, action_path)
                print("state:stair:step 1 done, goto step 2")
                step = 3
                break

        if step == 3:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-3, (310, 240), (300, 240), (320, 240)]
            angle_range = [-5, -1]
            stair = 0
            init_action()
            init_camera_platform(500, 320)
            while step == 3:
                if stair == 3:
                    print("done")
                    step = 999
                    break

                time.sleep(0.25)

                _, angle, points = color_detect_top(colors["obj"][stair], grad, ksizes, defaults)
                
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # AGC.runAction(Actions["go_s"], 1, action_path)
                AGC.runAction(Actions["up_and_go"], 1, action_path)
                time.sleep(0.1)
                stair += 1



            






# ------------------------- 七、主函数 ------------------------- #
if __name__ == "__main__":
    my_camera = Camera()
    my_camera.camera_open()
    while not frame_flag:
        print("camera:waiting for frame")
        time.sleep(0.1)

    frame_show = True
    state = State.StateStair

    while state != State.StateNone:
        globals().get(StateCalls[state], lambda x: print("Function not found"))(
            StateColors[state]
        )

    my_camera.camera_close()
    cv2.destroyAllWindows()
