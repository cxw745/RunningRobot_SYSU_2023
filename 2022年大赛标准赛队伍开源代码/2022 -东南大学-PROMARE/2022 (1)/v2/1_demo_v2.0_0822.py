import enum
import math
import queue
import threading
import time

import cv2
import hiwonder.ActionGroupControl as AGC  # 运行动作文件
import hiwonder.Board as Board  # 控制总线电机
import numpy as np


# 关卡状态枚举
class State(enum.Enum):
    idle = enum.auto()
    debug = enum.auto()
    hole = enum.auto()
    bridge = enum.auto()
    mine = enum.auto()
    obstacle = enum.auto()
    door = enum.auto()
    stair = enum.auto()


# ------------------------- 一、全局变量 ------------------------- #
# 图像帧
frame = None  # 原始帧
frame_cali = None  # 校正帧
frame_show = True  # 是否显示
frame_ready = False
frames = queue.Queue()

# 关卡状态
state = State.mine

# 关卡状态切换 [当前关卡: 下一关卡]
StateTrans = {
    State.hole: State.mine,
    State.mine: State.idle,
    State.obstacle: State.door,
    State.door: State.bridge,
    State.bridge: State.idle,
    State.stair: State.idle,
}

# 关卡状态处理函数, 通过 globals().get(StateCalls[state])(colors) 实现调用
StateCalls = {
    State.debug: "state_debug",
    State.hole: "state_hole",
    State.bridge: "state_bridge",
    State.mine: "state_mine",
    State.obstacle: "state_obstacle",
    State.door: "state_door",
    State.stair: "state_stair",
}

# 关卡中用到的颜色
StateColors = {
    State.debug: {"this": "green", "next": "", "obj": ""},
    State.hole: {"this": "green", "next": "white", "obj": ""},
    State.bridge: {"this": "green", "next": "", "obj": ""},
    State.mine: {"this": ["white2", "white"], "next": "blue", "obj": "black"},
    State.obstacle: {"this": "white2", "next": "", "obj": ""},
    State.door: {"this": "red", "next": "green", "obj": "blue2"},
    State.stair: {"this": "blue", "next": "", "obj": ["blue", "green", "red"]},
}

# LAB色彩范围
color_range = {
    # 使用中
    "green": [(47, 0, 135), (255, 110, 255)],
    "white": [(90, 0, 0), (255, 255, 255)],
    "white2": [(140, 0, 0), (255, 255, 255)],
    "black": [(0, 0, 0), (80, 255, 255)],
    "red": [(0, 156, 88), (255, 255, 255)],
    "blue": [(50, 0, 0), (255, 255, 115)],
    "blue2": [(0, 0, 0), (255, 255, 125)],
    # 未使用
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

# 动作组路径及常用动作
action_path = "/home/pi/AiNexPro/AiNexPro/wsh1/"
Actions = {
    "stand": "Stand",
    "go_s": "LIttlego1",
    "go_L": "STANdgo",
    "back_s": "little_back_q",
    "back_L": "BACK_q",
    "turn_left_s": "Turn_left1",
    "turn_left_m": "turn_left2",
    "turn_left_L": "turn_left3",
    "turn_right_s": "Turn_right1",
    "turn_right_m": "Turn_right2",
    "turn_right_L": "turn_right3",
    "left_go_s": "LEftgo-1",
    "left_go_m": "LEftgo",
    "left_go_L": "LEftgoB1_q2",
    "right_go_s": "Rightgo-1",
    "right_go_m": "Rightgo1",
    "right_go_L": "Rightgo2",
    "squat": "squat",
    "obstacle": "OVerrail_q2",
    "up_and_go": "upANDgo1",
    "down": "down",
}

# ------------------------- 二、图像采集 ------------------------- #
def capture():
    global frame, frame_cali, frame_ready

    # 鱼眼校正参数
    calibration_param_path = "/home/pi/2022/calibration_param.npz"
    param_data = np.load(calibration_param_path)
    dim = tuple(param_data["dim_array"])
    k = np.array(param_data["k_array"].tolist())
    d = np.array(param_data["d_array"].tolist())
    p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None)
    map = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), p, dim, cv2.CV_16SC2)

    # 打开摄像头
    while True:
        try:
            cap = cv2.VideoCapture(-1)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))
            cap.set(cv2.CAP_PROP_FPS, 30)
            break
        except Exception as e:
            print(f"camera:open failed:{str(e)}")
            time.sleep(0.1)

    # 获取并校正图像
    while True:
        if not cap.isOpened():
            time.sleep(0.1)
            continue

        ret, frame = cap.read()
        if not ret:
            continue

        frame_cali = cv2.remap(
            frame,
            map[0],
            map[1],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

        frame_ready = True


def show(frames):
    while True:
        frame = frames.get()
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break


# ------------------------- 三、图像处理 ------------------------- #
class line:
    """线段, 可计算中点、长度、角度"""

    def __init__(self, p0, p1):
        self.point0 = p0
        self.point1 = p1

    def mid_point(self):
        x = (self.point0[0] + self.point1[0]) // 2
        y = (self.point0[1] + self.point1[1]) // 2
        return (x, y)

    def length(self):
        dx_pow_2 = math.pow(self.point1[0] - self.point0[0], 2)
        dy_pow_2 = math.pow(self.point1[1] - self.point0[1], 2)
        return math.sqrt(dx_pow_2 + dy_pow_2)

    def angle(self):
        if self.point0[0] == self.point1[0]:
            return 90
        ang_tan = (self.point1[1] - self.point0[1]) / (self.point1[0] - self.point0[0])
        return -math.atan(ang_tan) * 180 / math.pi


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
    if not cnts:
        return ([], [])

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
    if not cnts:
        return ([], [])
    contourAreas = [cv2.contourArea(c) for c in cnts]
    (cnts, contourAreas) = zip(
        *sorted(zip(cnts, contourAreas), key=lambda b: b[1], reverse=reverse)
    )
    return (cnts, contourAreas)


def put_text(img, text, pos):
    cv2.putText(
        img,
        text,
        pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )


def color_detect_top(color, grad=1.0, ksizes=None, defaults=None):
    """返回目标区域占比, 顶部角度, 顶部中点[x,y], 顶部左点[x,y], 顶部右点[x,y]

    Args:
        color (str): 颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与形态学运算核. Defaults to [3, 3].
        defaults (list, optional): 缺省[顶部角度, 顶部中点[x, y], 顶部左点[x, y], 顶部右点[x, y]]. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        float, float, list: 目标区域占比, 顶部角度, [顶部中点[x, y], 顶部左点[x, y], 顶部右点[x, y]]
    """
    if ksizes is None:
        ksizes_ = (3, 3)
    else:
        ksizes_ = ksizes

    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    # 图像处理及roi提取
    global frame_cali, frame_ready
    print("detect:wait for frame")
    while not frame_ready:
        time.sleep(0.01)
    print("detect:frame ready")
    frame_ready = False
    img = frame_cali.copy()
    close = get_frame_bin(img, color, ksizes_[0], ksizes_[1])
    cnts, _ = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))  # 计算目标区域占比

    # 得到上顶点，计算中点及线段角度
    if cnt_max is None:
        cnt_max = np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])
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
        img_show = img.copy()
        put_text(img_show, f"area:{percent}%", (50, 50))
        put_text(img_show, f"angle:{top_angle:.2f}", (50, 100))
        put_text(img_show, f"center:{tuple(top_center)} ", (50, 150))
        put_text(img_show, f"left:{tuple(top_left)} ", (50, 200))
        put_text(img_show, f"right:{tuple(top_center)} ", (50, 250))

        cv2.drawContours(img_show, [cnt_max], -1, (0, 255, 255), 2)
        cv2.line(img_show, tuple(top_left), tuple(top_right), (255, 255, 0), 3)
        cv2.circle(img_show, tuple(top_center), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(top_left), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(top_right), 5, (255, 0, 0), 2)

        # 显示图像
        frames.put(img_show)

    return percent, top_angle, [top_center, top_left, top_right]


def color_detect_bottom(color, grad=1.0, ksizes=None, defaults=None):
    """返回目标区域占比, 底部角度, 底部中点[x,y], 底部左点[x,y], 底部右点[x,y]

    Args:
        color (str): 颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与形态学运算核. Defaults to (3, 3).
        defaults (list, optional): 缺省[底部角度, 底部中点[x, y], 底部左点[x, y], 底部右点[x, y]]. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        float, float, list: 目标区域占比, 底部角度, [底部中点[x, y], 底部左点[x, y], 底部右点[x, y]]
    """
    if ksizes is None:
        ksizes_ = (3, 3)
    else:
        ksizes_ = ksizes

    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    # 图像处理及roi提取
    global frame_cali, frame_ready
    print("detect:wait for frame")
    while not frame_ready:
        time.sleep(0.01)
    print("detect:frame ready")
    frame_ready = False
    img = frame_cali.copy()
    close = get_frame_bin(img, color, ksizes_[0], ksizes_[1])
    cnts, _ = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))

    if cnt_max is None:
        cnt_max = np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])
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
        img_show = img.copy()
        put_text(img_show, f"area:{percent}%", (50, 50))
        put_text(img_show, f"angle:{bottom_angle:.2f}", (50, 100))
        put_text(img_show, f"center:{tuple(bottom_center)} ", (50, 150))
        put_text(img_show, f"left:{tuple(bottom_left)} ", (50, 200))
        put_text(img_show, f"right:{tuple(bottom_center)} ", (50, 250))

        cv2.drawContours(img_show, [cnt_max], -1, (0, 255, 255), 2)
        cv2.line(img_show, tuple(bottom_left), tuple(bottom_right), (255, 255, 0), 3)
        cv2.circle(img_show, tuple(bottom_center), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(bottom_left), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(bottom_right), 5, (255, 0, 0), 2)

        # 显示图像
        frames.put(img_show)

    return percent, bottom_angle, [bottom_center, bottom_left, bottom_right]


def color_detect_mine_near(color_back, color_obj, ksizes=None, area=None, default=None):
    """识别最近处的地雷

    Args:
        color_back (str): 背景颜色
        color_obj (str): 地雷颜色
        ksizes (list, optional):  背景和地雷的高斯核与形态学运算核. Defaults to [(3, 3), (3, 3)].
        area (tuple, optional): 地雷面积范围. Defaults to (500, 3000).
        default (tuple, optional): 缺省地雷轮廓外接矩形. Defaults to (300, 0, 30, 30).

    Returns:
        tuple: (mine_x, mine_y)
    """
    if ksizes is None:
        ksizes_ = [(3, 3), (3, 3)]
    else:
        ksizes_ = ksizes

    if area is None:
        area_min = 500
        area_max = 8000
    else:
        area_min = area[0]
        area_max = area[1]

    if default is None:
        default_ = (300, 0, 30, 30)
    else:
        default_ = default

    # 提取 roi
    global frame_cali, frame_ready
    print("detect:wait for frame")
    while not frame_ready:
        time.sleep(0.01)
    print("detect:frame ready")
    frame_ready = False
    img = frame_cali.copy()
    img_bin = get_frame_bin(img, color_back, ksizes_[0][0], ksizes_[0][1])
    cnts, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnt_max, _ = get_maxcontour(cnts)

    if cnt_max is None:
        cnt_max = np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])
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
        if mines:
            mine_near = min(
                mines, key=lambda m: (1.2 * (m[0] - 320)) ** 2 + (m[1] - 480) ** 2
            )
        else:
            mine_near = default_
    else:
        mine_near = default_

    x, y, w, h = mine_near
    mine_x = x + w // 2
    mine_y = y

    if frame_show:
        img_show = img.copy()
        put_text(img_show, f"mine = ({mine_x}, {mine_y})", (50, 50))
        put_text(img_show, f"area = {w * h}", (50, 100))

        cv2.drawContours(img_show, [cnt_max], -1, (0, 255, 255), 2)
        cv2.rectangle(img_show, (x, y), (x + w, y + h), (255, 255, 0), 2)
        cv2.circle(img_show, (mine_x, mine_y), 5, (255, 0, 0), 2)

        # 显示图像
        frames.put(img_show)

    return (mine_x, mine_y)


def color_detect_sill(color, grad=1.0, ksizes=None, defaults=None):
    """识别门槛

    Args:
        color (str): 底边颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与形态学运算核. Defaults to (3, 3).
        defaults (list, optional): 缺省角度, 中点, 左端点和右端点. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        tuple: (top_angle, [top_center, top_left, top_right])
    """
    if ksizes is None:
        ksizes_ = (3, 3)
    else:
        ksizes_ = ksizes
    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    global frame_cali, frame_ready
    print("detect:wait for frame")
    while not frame_ready:
        time.sleep(0.01)
    print("detect:frame ready")
    frame_ready = False
    img = frame_cali.copy()
    img_bin = get_frame_bin(img, color, ksizes_[0], ksizes_[1])
    cnts, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnts_A = [c for c in cnts if cv2.contourArea(c) > 1000]

    if cnts_A:
        cnts_fl = [c for c in cnts_A if (cv2.boundingRect(c)[0] + cv2.boundingRect(c)[2]) > 320]
        if cnts_fl:
            cnts_sorted, _ = sort_contours_by_LT(cnts_fl, method="top-to-bottom")
            cnt_top = cnts_sorted[0]

            top_left = cnt_top[0][0]
            top_right = cnt_top[0][0]
            for c in cnt_top:  # 遍历找到四个顶点
                if c[0][0] + grad * c[0][1] < top_left[0] + grad * top_left[1]:
                    top_left = c[0]
                if -c[0][0] + grad * c[0][1] < -top_right[0] + grad * top_right[1]:
                    top_right = c[0]
            line_top = line(top_left, top_right)
            top_angle = line_top.angle()
            top_center = line_top.mid_point()  # 需要转换成tuple使用
        else:
            cnt_top = np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])
            top_angle, top_center, top_left, top_right = defaults_
    else:
        cnt_top = np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])
        top_angle, top_center, top_left, top_right = defaults_

    if frame_show:
        img_show = img.copy()
        put_text(img_show, f" angle = {top_angle:.2f}", (50, 100))
        put_text(img_show, f"center = {tuple(top_center)} ", (50, 150))
        put_text(img_show, f"  left = {tuple(top_left)} ", (50, 200))
        put_text(img_show, f" right = {tuple(top_center)} ", (50, 250))

        cv2.drawContours(img_show, [cnt_top], -1, (0, 255, 255), 2)
        cv2.line(img_show, tuple(top_left), tuple(top_right), (255, 255, 0), 3)
        cv2.circle(img_show, tuple(top_center), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(top_left), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(top_right), 5, (255, 0, 0), 2)

        # 显示图像
        frames.put(img_show)

    return (top_angle, [top_center, top_left, top_right])


def color_detect_door(color_back, color_obj, grad=1.0, ksizes=None, defaults=None):
    """识别门框

    Args:
        color_back (str): 背景颜色
        color_obj (str): 门框颜色
        grad (float, optional): 斜率
        ksizes (list, optional): 背景和门框的高斯核与形态学运算核. Defaults to [(3, 3), (3, 3)].
        defaults (list, optional): 缺省角度, 中点, 左端点和右端点. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        tuple: (angle, [center, left, right])
    """
    if ksizes is None:
        ksizes_ = [(3, 3), (3, 3)]
    else:
        ksizes_ = ksizes
    if defaults is None:
        defaults_ = [0, (320, 240), (300, 240), (340, 240)]
    else:
        defaults_ = defaults

    global frame_cali, frame_ready
    print("detect:wait for frame")
    while not frame_ready:
        time.sleep(0.01)
    print("detect:frame ready")
    frame_ready = False
    img = frame_cali.copy()
    img_bin = get_frame_bin(img, color_back, ksizes_[0][0], ksizes_[0][1])
    cnts, _ = cv2.findContours(img_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnt_max, _ = get_maxcontour(cnts)

    if cnt_max is None:
        cnt_max = np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])
        cnts_max_two = [np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])]
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
            cnts_max_two = [np.array([[[0, 0]], [[639, 0]], [[639, 479]], [[0, 479]]])]
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
        img_show = img.copy()
        put_text(img_show, f" angle = {angle:.2f}", (50, 100))
        put_text(img_show, f"center = {tuple(center)} ", (50, 150))
        put_text(img_show, f"  left = {tuple(left)} ", (50, 200))
        put_text(img_show, f" right = {tuple(center)} ", (50, 250))

        cv2.drawContours(img_show, [cnt_max], -1, (0, 255, 255), 2)
        cv2.drawContours(img_show, cnts_max_two, -1, (255, 0, 255), 2)
        cv2.line(img_show, tuple(left), tuple(right), (255, 255, 0), 3)
        cv2.circle(img_show, tuple(center), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(left), 5, (255, 0, 0), 2)
        cv2.circle(img_show, tuple(right), 5, (255, 0, 0), 2)

        # 显示图像
        frames.put(img_show)

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
    time.sleep(0.01)  # 延时时间和运行时间相同
    Board.setBusServoPulse(20, top, 500)  # id=20 上下
    time.sleep(0.01)


# ------------------------- 五、局部调整函数 ------------------------- #
def adjust_x(x, x_range):
    """水平调整x位置

    Args:
        x (int): x
        x_range (list[int]): x范围[min, max]
    """
    dx = abs((x_range[1] + x_range[0]) // 2 - x)
    print(f"dx = {dx}")
    if x < x_range[0]:
        if dx > 150:
            AGC.runAction(Actions["left_go_L"], 1, action_path)
        elif dx > 80:
            AGC.runAction(Actions["left_go_m"], 1, action_path)
        else:
            AGC.runAction(Actions["left_go_s"], 1, action_path)
        # time.sleep(0.25)
    elif x > x_range[1]:
        if dx > 150:
            AGC.runAction(Actions["left_go_L"], 1, action_path)
        elif dx > 80:
            AGC.runAction(Actions["right_go_m"], 1, action_path)
        else:
            AGC.runAction(Actions["right_go_s"], 1, action_path)
        # time.sleep(0.25)


def adjust_angle(angle, angle_range):
    """调整角度

    Args:
        angle (float): 角度
        angle_range (int[float]): 角度范围[min, max]
    """
    dAng = abs((angle_range[0] + angle_range[1]) / 2 - angle)
    print(f"dAng = {dAng:.2f}")
    if angle < angle_range[0]:
        if dAng > 15:
            AGC.runAction(Actions["turn_right_L"], 1, action_path)
        elif dAng > 8:
            AGC.runAction(Actions["turn_right_m"], 1, action_path)
        else:
            AGC.runAction(Actions["turn_right_s"], 1, action_path)
        # time.sleep(0.5)
    elif angle > angle_range[1]:
        if dAng > 15:
            AGC.runAction(Actions["turn_left_L"], 1, action_path)
        elif dAng > 8:
            AGC.runAction(Actions["turn_left_m"], 1, action_path)
        else:
            AGC.runAction(Actions["turn_left_s"], 1, action_path)
        # time.sleep(0.5)


# ------------------------- 六、关卡状态 ------------------------- #
def state_debug(colors):
    print("state:debug:enter")
    global state, frame_show
    frame_show = True
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, (320, 240), (300, 240), (340, 240)]

    while state == State.debug:
        color_detect_bottom(colors["this"], grad, ksizes, defaults)


def state_hole(colors):
    print("state:hole:enter")
    global state
    step = 0
    while state == State.hole:
        if step == -1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 240), (300, 240), (340, 240)]
            angle_range = [-3, 3]
            center_x_range = [280, 360]
            center_y_switch = 90
            center_y_stop = 120
            init_action()
            init_camera_platform(500, 500)
            while step == -1:
                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )

        # 0、靠近
        if step == 0:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 240), (300, 240), (340, 240)]
            angle_range = [-4, 4]
            center_x_range = [280, 380]
            center_y_switch = 120
            center_y_stop = 300
            init_action()
            init_camera_platform(500, 500)
            while step == 0:
                # 获取数据
                time.sleep(0.5)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 移动到中间
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 大步走
                if center_y < center_y_switch:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 小步走
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                # 进入下一步
                print("state:hole:step 0 done, goto step 1")
                step = 1
                break

        # 1、左移
        if step == 1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 240), (300, 240), (340, 240)]
            angle_range = [-2, 2]
            center_x_range = [300, 340]
            left_x_range = [210, 250]
            left_flag = 0
            camera_flag = 0
            init_action()
            init_camera_platform(500, 400)
            while step == 1:
                # 获取数据
                time.sleep(0.5)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                left_x, _ = points[1]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 大步左平移三步
                if (not left_x_range[0] < left_x < left_x_range[1]) and left_flag == 0:
                    AGC.runAction(Actions["left_go_L"], 2, action_path)
                    left_flag = 1
                    continue

                # 小步调整
                if (not left_x_range[0] < left_x < left_x_range[1]) and left_flag == 1:
                    adjust_x(left_x, left_x_range)
                    continue

                # 进下一步
                print("state:hole:step 1 done, goto step 2")
                step = 2
                break

        # 2、前进
        if step == 2:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 361), (300, 361), (340, 361)]
            angle_range = [-3, 3]
            camera_flag = 0
            left_y_switch = 360
            left_y_stop = 360
            init_action()
            init_camera_platform(500, 400)
            while step == 2:
                # 获取数据
                time.sleep(0.5)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                _, left_y = points[1]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 大步向前
                if camera_flag == 0 and left_y < left_y_switch:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 低头
                if camera_flag == 0 and left_y > left_y_switch:
                    init_camera_platform(500, 300)
                    camera_flag = 1
                    continue

                # 大步向前
                if camera_flag == 1 and left_y < left_y_stop:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 进入下一步
                print("state:hole:step 2 done, goto step 3")
                step = 3
                break

        # 3、回到中间
        if step == 3:
            # 进入下一关
            # AGC.runAction(Actions["go_s"], 1, action_path)
            AGC.runAction(Actions["right_go_L"], 1, action_path)
            # AGC.runAction(Actions["go_s"], 1, action_path)
            # time.sleep(0.1)
            # AGC.runAction(Actions["right_go_L"], 5, action_path)
            # time.sleep(0.1)
            # AGC.runAction(Actions["turn_right_s"], 1, action_path)
            print("state:hole:exit")
            state = StateTrans[State.hole]
            break


def state_bridge(colors):
    print("state:bridge:enter")
    global state
    step = 0
    while state == State.bridge:
        if step == -1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 321), (300, 321), (340, 321)]
            center_x_range = [300, 360]
            angle_range = [-4, 2]
            center_y_switch = 120
            center_y_stop = 310
            init_action()
            init_camera_platform(500, 300)
            while step == -1:
                time.sleep(0.5)
                _, angle, points = color_detect_bottom(
                    colors["this"], grad, ksizes, defaults
                )
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # dAng = abs((angle_range[0]+angle_range[1])/2 - angle)
                # _, angle, points = color_detect_top(
                #     colors["this"], grad, ksizes, defaults
                # )

        # 0、靠近
        if step == 0:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 321), (300, 321), (340, 321)]
            center_x_range = [300, 360]
            angle_range = [-6, 1]
            center_y_switch = 120
            center_y_stop = 310
            init_action()
            init_camera_platform(500, 300)
            while step == 0:
                # 获取数据
                time.sleep(0.5)
                _, angle, points = color_detect_bottom(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 移动到中间
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 移动到桥边
                if center_y < center_y_switch:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 移动到桥边
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                # 进入下一步
                print("state:bridge:step 0 done, goto step 1")
                step = 1
                break

        # 1、抬头前进
        if step == 1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 375), (300, 375), (340, 375)]
            center_x_range = [300, 360]
            center_y_switch = 300
            center_y_stop = 360
            angle_range = [-6, 1]
            camera_flag = 0
            init_action()
            init_camera_platform(500, 500)
            while step == 1:
                # 获取数据
                time.sleep(0.5)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 移动到中间
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 大步向前
                if camera_flag == 0 and center_y < center_y_switch:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 低头
                if camera_flag == 0 and center_y > center_y_switch:
                    init_camera_platform(500, 400)
                    time.sleep(0.5)
                    camera_flag = 1
                    continue

                # 大步向前
                if camera_flag == 1 and center_y < center_y_switch:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 小步向前
                if camera_flag == 1 and center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                # 进入下一步
                print("state:bridge:step 1 done, goto step 2")
                # init_camera_platform(500, 300)
                step = 2
                break

        # 2、低头前进
        if step == 2:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 361), (300, 361), (340, 361)]
            center_y_switch = 300
            center_y_stop = 360
            angle_range = [-6, 1]
            init_action()
            init_camera_platform(500, 300)
            while step == 2:
                # 获取数据
                time.sleep(0.5)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                _, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 大步前进
                if center_y < center_y_switch:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 小步前进
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                AGC.runAction(Actions["go_L"], 1, action_path)

                # 进入下一关
                print("state:bridge:exit")
                state = StateTrans[State.bridge]
                break


def state_mine(colors):
    print("state:mine:enter")
    global state
    start = time.time()
    self_flag = 0
    camera_flag = 0
    self_x = 320
    step = 0
    time_sleep = 0
    go_num = 0
    go_num_max = 4
    init_action()
    while state == State.mine:
        if step == -1:
            grad = 2.5
            ksizes = [(11, 7), (11, 7)]
            defaults = [-2, (320, 410), (300, 410), (340, 410)]
            init_camera_platform(500, 500)
            while step == -1:
                angle, points = color_detect_sill(
                    colors["next"], grad, ksizes[0], defaults
                )
                # mine_x, mine_y = color_detect_mine_near(
                #     colors["this"][1], colors["obj"], ksizes
                # )

        # 0、抬头调整
        if step == 0:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [0, (320, 410), (300, 410), (340, 410)]

            while step == 0:
                # 调整相机高度
                if camera_flag == 0:
                    angle_range = [-3, 3]
                    center_y_switch = 320
                    color = colors["next"]
                    print("state:mine:camera:A")
                    init_camera_platform(500, 500)
                    time.sleep(0.5)

                if camera_flag == 1:
                    angle_range = [-3, 3]
                    center_y_switch = 200
                    color = colors["next"]
                    print("state:mine:camera:B")
                    init_camera_platform(500, 400)
                    time.sleep(0.5)

                time.sleep(0.5)
                time_sleep += 0.7
                print("--------------- step 0 ---------------")
                print(f"state:mine:sleep: {time_sleep:.2f}")
                print(f"state:mine:total: {time.time() - start:.2f}")

                # 获取数据
                angle, points = color_detect_sill(color, grad, ksizes, defaults)
                center_x, center_y = points[0]
                if center_x < 300:
                    self_flag = 0
                else:
                    self_flag = 1
                self_x = 320 - center_x

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 判断是否低头
                if camera_flag == 0 and center_y > center_y_switch:
                    camera_flag = 1
                    continue

                # 进入下一步 2
                if camera_flag == 1 and center_y > center_y_switch:
                    print("state:mine:step 0 done, goto step 2")
                    step = 2
                    break

                # 进入下一步 1
                print("state:mine:step 0 done, goto step 1")
                step = 1
                break

        # 1、低头前进
        if step == 1:
            grad = 2.5
            ksizes = [(11, 7), (11, 7)]
            defaults = [-2, (320, 0), (300, 0), (340, 0)]
            mine_y_big = 215
            mine_y_little = 265
            mine_y_stop = 365
            mine_x_range = [120, 500]
            mine_x_switch = 65
            m_x_b_range = [-80, 80]

            # 低头看地雷
            init_camera_platform(500, 350)
            time.sleep(0.5)
            while step == 1:
                if go_num > go_num_max:
                    step = 0
                    go_num = 0
                    break

                go_num += 1

                time.sleep(0.5)
                time_sleep += 0.7
                print("--------------- step 1 ---------------")
                print(f"state:mine:sleep: {time_sleep:.2f}")
                print(f"state:mine:total: {time.time() - start:.2f}")

                # 识别地雷
                mine_x, mine_y = color_detect_mine_near(
                    colors["this"][1], colors["obj"], ksizes
                )
                print(f"mine: {mine_x}, {mine_y}")

                mine_x_self = mine_x - 310
                mine_x_base = 1.5 * self_x + 0.5 * mine_x_self
                print(f"mine_x_self: {mine_x_self:.2f}")
                print(f"mine_x_base: {mine_x_base:.2f}")

                if mine_y < mine_y_big:
                    print("state:mine:1")
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                if mine_y < mine_y_little:
                    print("state:mine:2.1")
                    AGC.runAction(Actions["go_s"], 2, action_path)
                    continue

                if mine_y < mine_y_stop:
                    print("state:mine:2.2")
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                if not m_x_b_range[0] < mine_x_base < m_x_b_range[1]:
                    if mine_x_range[0] < mine_x < mine_x_range[1] and self_flag == 0:
                        print("state:mine:3.1")
                        if abs(mine_x_self) < mine_x_switch:
                            print("state:mine:3.1.1")
                            AGC.runAction(Actions["left_go_L"], 1, action_path)
                        else:
                            print("state:mine:3.1.2")
                            AGC.runAction(Actions["left_go_s"], 1, action_path)
                        continue

                    if mine_x_range[0] < mine_x < mine_x_range[1] and self_flag == 1:
                        print("state:mine:4.1")
                        if abs(mine_x_self) < mine_x_switch:
                            print("state:mine:4.1.1")
                            AGC.runAction(Actions["right_go_L"], 1, action_path)
                        else:
                            print("state:mine:4.1.2")
                            AGC.runAction(Actions["right_go_L"], 1, action_path)
                        continue
                else:
                    if mine_x_range[0] < mine_x < mine_x_range[1] and mine_x > 320:
                        print("state:mine:3.2")
                        if abs(mine_x_self) < mine_x_switch:
                            print("state:mine:3.2.1")
                            AGC.runAction(Actions["left_go_L"], 1, action_path)
                        else:
                            print("state:mine:3.2.2")
                            AGC.runAction(Actions["left_go_s"], 1, action_path)
                        continue

                    if mine_x_range[0] < mine_x < mine_x_range[1] and mine_x < 321:
                        print("state:mine:4.2")
                        if abs(mine_x_self) < mine_x_switch:
                            print("state:mine:4.2.1")
                            AGC.runAction(Actions["right_go_L"], 1, action_path)
                        else:
                            print("state:mine:4.2.2")
                            AGC.runAction(Actions["right_go_s"], 1, action_path)
                        continue

                print("state:mine:5")
                print("state:mine:step 1 done, goto step 0")
                AGC.runAction(Actions["go_L"], 1, action_path)
                step = 0
                break

        # 2、衔接
        if step == 2:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, (320, 240), (300, 240), (340, 240)]
            angle_range = [-5, -1]
            center_y_stop = 310

            init_camera_platform(500, 300)
            while step == 2:
                print("--------------- step 2 ---------------")
                time.sleep(0.5)
                angle, points = color_detect_sill(
                    colors["next"], grad, ksizes, defaults
                )
                _, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                print("state:mine:exit")
                state = StateTrans[State.mine]
                break


def state_obstacle(colors):
    print("state:obstacle:enter")
    global state
    step = 12
    while state == State.obstacle:
        # 0、调整位置
        if step == 0:
            grad = 1.5
            ksizes = [11, 7]
            defaults = [-2, (320, 240), (300, 240), (340, 240)]
            angle_range = [-1, 3]
            center_x_range = [280, 320]
            center_y_stop = 140
            init_camera_platform(500, 450)
            while step == 0:
                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 平移到中间
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 前进
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                step = 12
                break

        # 11、翻越, 后退
        if step == 11:
            while step == 11:
                AGC.runAction(Actions["obstacle"], 1, action_path)
                AGC.runAction(Actions["back_L"], 7, action_path)
                step = 999
                state = StateTrans[State.obstacle]
                print("state:obstacle:exit")
                break

        # 12、翻越, 后退, 左转, 后退
        if step == 12:
            while step == 12:
                AGC.runAction(Actions["obstacle"], 1, action_path)
                AGC.runAction(Actions["back_L"], 4, action_path)
                # AGC.runAction(Actions["turn_left_L"], 1, action_path)
                # AGC.runAction(Actions["back_L"], 4, action_path)
                step = 999
                state = StateTrans[State.obstacle]
                print("state:obstacle:exit")
                break


def state_door(colors):
    print("state:door:enter")
    global state
    step = 0
    while state == State.door:
        if step == -1:
            grad = 1.5
            ksizes = [(17, 15), (15, 11)]
            defaults = [-2, [320, 385], [300, 385], [340, 385]]
            init_camera_platform(888, 390)
            while step == -1:
                # _, angle, points = color_detect_top(
                #     colors["this"], grad, ksizes, defaults
                # )
                angle, points = color_detect_door(
                    colors["this"], colors["obj"], grad, ksizes, defaults
                )

        # 11、正过(未完成)
        if step == 11:
            grad = 1.5
            ksizes = [(15, 7), (17, 15)]
            defaults = [-2, [320, 385], [300, 385], [340, 385]]
            angle_range = [-6, -1]
            center_x_range = [310, 330]
            init_camera_platform(500, 320)
            while step == 11:
                time.sleep(0.25)
                angle, points = color_detect_door(
                    colors["this"], colors["obj"], grad, ksizes, defaults
                )
                center_x, _ = points[0]

                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                AGC.runAction(Actions["go_L"], 3, action_path)

                print("state:door:step 1 done, goto step 2")
                step = 2
                break

        # 0、侧过
        if step == 0:
            grad = 1.5
            ksizes = [(17, 15), (15, 11)]
            defaults = [-2, [320, 385], [300, 385], [340, 385]]
            angle_range = [-7, 2]
            center_x_range = [340, 390]

            # 蹲下
            AGC.runAction(Actions["squat"], 1, action_path)

            # 扭头
            init_camera_platform(888, 390)
            while step == 0:
                # 获取数据
                time.sleep(0.5)
                angle, points = color_detect_door(
                    colors["this"], colors["obj"], grad, ksizes, defaults
                )
                center_x, _ = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    time.sleep(0.1)
                    AGC.runAction(Actions["squat"], 1, action_path)
                    time.sleep(0.1)
                    continue

                # 调整位置
                if center_x > center_x_range[1]:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    time.sleep(0.1)
                    AGC.runAction(Actions["squat"], 1, action_path)
                    continue
                elif center_x < center_x_range[0]:
                    AGC.runAction(Actions["back_s"], 1, action_path)
                    time.sleep(0.1)
                    AGC.runAction(Actions["squat"], 1, action_path)
                    continue

                # 左移
                AGC.runAction(Actions["left_go_L"], 8, action_path)

                # 左转
                AGC.runAction(Actions["stand"], 6, action_path)
                AGC.runAction(Actions["turn_left_L"], 5, action_path)

                # 进入下一关
                state = StateTrans[State.door]
                print("state:door:exit")
                break


def state_stair(colors):
    print("state:stair:enter")
    global state
    step = 4
    while state == State.stair:
        if step == -1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-2, [320, 240], [300, 240], [340, 240]]
            init_action()
            init_camera_platform(500, 350)
            while step == -1:
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )

        # 0、大步前进
        if step == 0:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [0, (310, 240), (300, 240), (320, 240)]
            angle_range = [-3, 5]
            center_x_range = [260, 360]
            center_y_stop = 360
            init_action()
            init_camera_platform(500, 450)
            while step == 0:
                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_bottom(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 左右调整
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 大步前进
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 进入下一步
                print("state:stair:step 0 done, goto step 1")
                step = 1
                break

        # 1、低头大步前进
        if step == 1:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [0, (310, 240), (300, 240), (320, 240)]
            angle_range = [-6, 1]
            center_x_range = [260, 360]
            center_y_stop = 150
            init_action()
            init_camera_platform(500, 350)
            while step == 1:
                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_bottom(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 左右调整
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 大步前进
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_L"], 1, action_path)
                    continue

                # 进入下一步
                print("state:stair:step 1 done, goto step 2")
                step = 2
                break

        # 2、低头小步调整
        if step == 2:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-3, (310, 240), (300, 240), (320, 240)]
            angle_range = [-5, 3]
            center_x_range = [300, 340]
            center_y_stop = 200
            init_action()
            init_camera_platform(500, 350)
            while step == 2:
                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_top(
                    colors["this"], grad, ksizes, defaults
                )
                center_x, center_y = points[0]

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 左右调整
                if not center_x_range[0] < center_x < center_x_range[1]:
                    adjust_x(center_x, center_x_range)
                    continue

                # 小步前进
                if center_y < center_y_stop:
                    AGC.runAction(Actions["go_s"], 1, action_path)
                    continue

                # 进入下一步
                # AGC.runAction(Actions["go_s"], 1, action_path)
                print("state:stair:step 2 done, goto step 3")
                step = 3
                break

        # 3、上楼梯
        if step == 3:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-3, (310, 240), (300, 240), (320, 240)]
            angle_range = [-5, 3]
            stair = 0
            init_action()
            init_camera_platform(500, 320)
            while step == 3:
                # 判断位置, 是否进入下一步
                if stair > 2:
                    print("state:stair:step 3 done, goto step 4")
                    step = 4
                    break

                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_top(
                    colors["obj"][stair], grad, ksizes, defaults
                )

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 上台阶
                AGC.runAction(Actions["up_and_go"], 1, action_path)
                time.sleep(0.1)
                stair += 1

        # 4、下楼梯
        if step == 4:
            grad = 2.5
            ksizes = [11, 7]
            defaults = [-3, (310, 240), (300, 240), (320, 240)]
            angle_range = [-5, 3]
            stair = 1
            init_action()
            init_camera_platform(500, 320)
            while step == 4:
                # 判断位置, 是否进入下一步
                if stair < 0:
                    print("state:stair:step 3 done, goto step 4")
                    step = 5
                    break

                # 获取数据
                time.sleep(0.25)
                _, angle, points = color_detect_top(
                    colors["obj"][stair], grad, ksizes, defaults
                )

                # 调整角度
                if not angle_range[0] < angle < angle_range[1]:
                    adjust_angle(angle, angle_range)
                    continue

                # 下台阶
                AGC.runAction(Actions["down"], 1, action_path)
                time.sleep(0.1)
                stair -= 1


# ------------------------- 七、主函数 ------------------------- #
if __name__ == "__main__":
    # 采集线程
    cap_th = threading.Thread(target=capture, args=(), daemon=True)
    cap_th.start()

    # 显示线程
    if frame_show:
        show_th = threading.Thread(target=show, args=(frames,), daemon=True)
        show_th.start()

    # 等待图像
    while not frame_ready:
        print("camera:waiting for frame")
        time.sleep(0.5)

    # 关卡处理
    err_func = lambda _: print(f'funciton "{StateCalls[state]}" not defined.')
    while state != State.idle:
        globals().get(StateCalls[state], err_func)(StateColors[state])

    if frame_show:
        show_th.join()
