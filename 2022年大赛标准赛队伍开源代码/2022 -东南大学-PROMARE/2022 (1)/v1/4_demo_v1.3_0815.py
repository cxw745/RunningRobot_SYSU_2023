import time
import math
import threading
from enum import Enum, auto

import cv2
import numpy as np

import hiwonder.ActionGroupControl as AGC  # 运行动作文件
import hiwonder.Board as Board  # 控制总线电机

# ------------------------- 一、全局变量 ------------------------- #
# 图像帧
frame = None # 原始帧
frame_cali = None # 校正帧
frame_flag = False # 是否准备好
frame_show = False # 是否显示

# LAB色彩范围
color_range = {
    "green": [(47, 0, 0), (255, 110, 255)],
    "white": [(150, 0, 0), (255, 255, 255)],
    "black": [(0, 0, 0), (55, 255, 255)],

    "green_hole": [(47, 0, 135), (255, 110, 255)],
    "blue": [(0, 130, 0), (198, 255, 110)],
    "blue_bridge": [(0, 0, 0), (255, 175, 94)],
    "ball": [(0, 0, 0), (20, 255, 255)],
    "red": [(140, 147, 0), (187, 255, 255)],
    "white_road": [(193, 0, 0), (255, 255, 255)],
    "white_door": [(140, 0, 0), (255, 255, 255)],
    "yellow": [(100, 140, 160), (225, 190, 190)],
    "yellow_start": [(11, 42, 71), (225, 190, 190)],
    "yellow_end": [(0, 0, 151), (225, 141, 255)],
    "ball_hole": [(0, 137, 0), (255, 255, 127)],
    "barrier_y": [(0, 83, 162), (200, 255, 255)],
    "barrier_k": [(0, 0, 0), (28, 255, 255)],
    "gate_bule": [(0, 0, 0), (255, 255, 115)],
    "road_white": [(134, 0, 0), (255, 255, 255)]
}

# 动作组路径及常用动作
action_path = "/home/pi/AiNexPro/AiNexPro/wsh1/"
Actions = {
    "stand": "Stand",
    "go_s": "Littlego",
    "go_L": "Standgo",
    "go_L_2": "Standgo2",
    "go_L_3": "Standgo3",
    "turn_left_s": "turn_left1",
    "turn_right_s": "turn_right1",
    "left_go_s": "Leftgo",
    "left_go_L": "LeftgoB",
    "right_go_s": "rightgo",
    "right_go_L": "rightgo2"
}

# 关卡状态枚举
class State(Enum):
    StateNone = auto()
    StateDebug = auto()
    StateHole = auto()
    StateBridge = auto()
    StateMine = auto()

# 关卡状态
state = State.StateDebug

# 关卡状态切换 [当前关卡: 下一关卡]
StateTrans = {
    State.StateHole: State.StateDebug,
    State.StateBridge: State.StateDebug,
}

# 关卡状态处理函数, 通过 globals().get(StateCalls[state])(colors) 实现调用
StateCalls = {
    State.StateDebug: "state_debug",
    State.StateHole: "state_hole",
    State.StateBridge: "state_bridge",
    State.StateMine: "state_mine"
}

# 关卡中用到的颜色
StateColors = {
    State.StateDebug: {"this": "green", "next": "", "obj": ""},
    State.StateHole: {"this": "green", "next": "", "obj": ""},
    State.StateBridge: {"this": "green", "next": "", "obj": ""},
    State.StateMine: {"this": "white", "next": "", "obj": "black"}
}


# ------------------------- 二、相机 ------------------------- #
# 鱼眼校正参数
calibration_param_path = "/home/pi/AiNexPro/CameraCalibration/" # 参数路径
param_data = np.load(calibration_param_path + "calibration_param.npz")
dim = tuple(param_data["dim_array"])
k = np.array(param_data["k_array"].tolist())
d = np.array(param_data["d_array"].tolist())
p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim ,None) # 优化内参和畸变参数
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

    def camera_open(self): # 开启
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))
            self.cap.set(cv2.CAP_PROP_FPS, 30) # 帧率
            # self.cap.set(cv2.CAP_PROP_SATURATION, 40) # 饱和度
            self.opened = True
        except Exception as e:
            print("打开摄像头失败:", e)

    def camera_close(self): # 关闭
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print("关闭摄像头失败:", e)

    def camera_task(self): # 获取摄像头画面线程
        global frame, frame_cali, frame_flag
        while True:
            try:
                if self.opened and self.cap.isOpened(): # 判断是否开启
                    ret, frame_tmp = self.cap.read() # 获取画面
                    if ret:
                        # 图像缩放与校正
                        self.frame = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
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
    """线段, 可计算中点、长度、角度
    """
    def __init__(self, p0, p1):
        self.point0 = p0
        self.point1 = p1

    def mid_point(self):
        return (self.point0[0]+self.point1[0])//2, (self.point0[1]+self.point1[1])//2

    def length(self):
        return math.sqrt(math.pow(self.point1[1]-self.point0[1], 2) + math.pow(self.point1[0]-self.point0[0], 2))

    def angle(self):
        if self.point0[0] == self.point1[0]:
            return 90
        else:
            return -math.atan((self.point1[1]-self.point0[1]) / (self.point1[0]-self.point0[0])) * 180 / math.pi

def get_maxcontour(contours, area_threshold = 1000):
    """获得最大轮廓及其面积

    Args:
        contours (list[cnt]): 轮廓列表
        area_threshold (int, optional): 面积阈值. Defaults to 1000.

    Returns:
        cnt, float: 最大轮廓, 面积
    """
    cnt_max = None
    area_max = 0
    if len(contours) != 0 :
        cnt = max(contours, key=cv2.contourArea)

        if cv2.contourArea(cnt) > area_threshold:
            cnt_max = cnt
            area_max = cv2.contourArea(cnt)

    return cnt_max, area_max

def color_detcet_top(
    color,
    grad = 1.0,
    ksizes=[3, 3],
    defaults=[0, [320, 240], [300, 240], [340, 240]]
):
    """返回目标区域占比, 顶部角度, 顶部中点[x,y], 顶部左点[x,y], 顶部右点[x,y]

    Args:
        color (str): 颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与开闭运算核. Defaults to [3, 3].
        defaults (list, optional): 缺省[顶部角度, 顶部中点[x, y], 顶部左点[x, y], 顶部右点[x, y]]. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        float, float, list: 目标区域占比, 顶部角度, [顶部中点[x, y], 顶部左点[x, y], 顶部右点[x, y]]
    """
    # 图像处理及roi提取
    global frame_cali
    img = frame_cali.copy()
    img_smooth = cv2.GaussianBlur(img, (ksizes[0], ksizes[0]), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(
        img_transform, color_range[color][0], color_range[color][1]
    )
    kernel = np.ones((ksizes[1], ksizes[1]), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    cnts, _ = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))  # 计算目标区域占比

    # 得到上顶点，计算中点及线段角度
    if cnt_max is None:
        top_angle = defaults[0]
        top_center = defaults[1]
        top_left = defaults[2]
        top_right = defaults[3]
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
            "angle: " + str(top_angle),
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
        cv2.circle(vis_img, tuple(top_center), 5, [0, 255, 255], 2)
        cv2.circle(vis_img, tuple(top_left), 5, [0, 255, 255], 2)
        cv2.circle(vis_img, tuple(top_right), 5, [0, 255, 255], 2)
        cv2.imshow("top_center", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)
    
    return percent, top_angle, [top_center, top_left, top_right]

def color_detcet_bottom(
    color,
    grad = 1.0,
    ksizes=[3, 3],
    defaults=[0, [320, 240], [300, 240], [340, 240]]
):
    """返回目标区域占比, 底部角度, 底部中点[x,y], 底部左点[x,y], 底部右点[x,y]

    Args:
        color (str): 颜色
        grad (float, optional): 斜率. Defaults to 1.0.
        ksizes (list, optional): 高斯核与开闭运算核. Defaults to [3, 3].
        defaults (list, optional): 缺省[底部角度, 底部中点[x, y], 底部左点[x, y], 底部右点[x, y]]. Defaults to [0, (320, 240), (300, 240), (340, 240)].

    Returns:
        float, float, list: 目标区域占比, 底部角度, [底部中点[x, y], 底部左点[x, y], 底部右点[x, y]]
    """
    # 图像处理及roi提取
    global frame_cali
    img = frame_cali.copy()
    img_smooth = cv2.GaussianBlur(img, (ksizes[0], ksizes[0]), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(
        img_transform, color_range[color][0], color_range[color][1]
    )
    kernel = np.ones((ksizes[1], ksizes[1]), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    cnts, _ = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))

    if cnt_max is None:
        bottom_angle = defaults[0]
        bottom_center = defaults[1]
        bottom_left = defaults[2]
        bottom_right = defaults[3]
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
            "angle: " + str(bottom_angle),
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
        cv2.circle(vis_img, tuple(bottom_center), 5, [0, 255, 255], 2)
        cv2.circle(vis_img, tuple(bottom_left), 5, [0, 255, 255], 2)
        cv2.circle(vis_img, tuple(bottom_right), 5, [0, 255, 255], 2)
        cv2.imshow("bottom_center", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)

    return percent, bottom_angle, [bottom_center, bottom_left, bottom_right]

# ------------------------- 四、初始化函数 ------------------------- #
def init_action(action_name=Actions["stand"]):
    """初始化动作

    Args:
        action_name (str, optional): 动作名称. Defaults to "stand".
    """
    AGC.runAction(action_name, 1, action_path)  # 站立
    time.sleep(0.5)

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
        time.sleep(0.25)
    elif x > x_range[1]:
        print("adjust:x =", x, "right_go")
        AGC.runAction(Actions["right_go_s"], 1, action_path)
        time.sleep(0.25)
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
        time.sleep(0.5)
    elif angle > angle_range[1]:
        print("adjust:angle =", angle, "turn_left")
        AGC.runAction(Actions["turn_left_s"], 1, action_path)
        time.sleep(0.5)
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
    global state, frame, frame_cali, frame_show
    frame_show = True
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, [320, 240], [300, 240], [340, 240]]
   
    print("state:debug:enter")
    while state == State.StateDebug:
        color_detcet_top(colors["this"], grad, ksizes, defaults)

def state_hole(colors):
    """state:hole

    Args:
        color_this (str, optional): 关卡颜色. Defaults to "green".
    """
    global state, frame, frame_cali
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, [320, 240], [300, 240], [340, 240]]
    left_range = [220, 240]
    angle_range = [-3, 3]
    left_flag = 0
    camera_flag = 0
    step = 0
    
    init_action()
    init_camera_platform(500, 400)
    print("state:hole:enter")
    while state == State.StateHole:
        if step == 0:
            _, top_angle, points = color_detcet_top(colors["this"], grad, ksizes, defaults)
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
        if step == 1:
            # 识别顶部角度和左点
            _, top_angle, points = color_detcet_top(colors["this"], grad, ksizes, defaults)
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
                    time.sleep(0.2)
                    left_flag = 1
                    continue
                # 小步调整
                if left_flag == 1:
                    adjust_x(top_left[0], left_range)
                    continue
            
            # 进入下一步
            print("state:hole:step 1 done, goto step 2")
            step = 2
        if step == 2:
            # 识别顶部角度和左点
            _, top_angle, points = color_detcet_top(colors["this"], grad, ksizes, defaults)
            top_left = points[1]
            # 调整角度
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue
            # 前进至目标点
            if top_left[1] < 300 and camera_flag == 0:
                # 大步向前
                AGC.runAction(Actions["go_L"], 1, action_path)
                time.sleep(0.5)
                continue
            elif camera_flag == 0:
                # 低头
                init_camera_platform(500, 300)
                time.sleep(0.5)
                camera_flag = 1
                continue
            elif top_left[1] < 350 and camera_flag == 1:
                # 小步向前
                AGC.runAction(Actions["go_s"], 1, action_path)
                time.sleep(0.5)
                continue
            # 关卡完成
            print("state:hole:step 2 done")
            break
    
    # 进入下一关
    state = StateTrans[State.StateHole]
    print("state:hole:exit")

def state_bridge(colors):
    pass


def state_mine(colors):
    global state, frame, frame_cali
    grad = 2.5
    ksizes = [11, 7]
    defaults = [-2, [320, 240], [300, 240], [340, 240]]
    center_y_start = 0
    center_range = [290, 310]
    left_range = [220, 240]
    angle_range = [-3, 3]
    left_flag = 0
    camera_flag = 0
    step = 0
    
    init_action()
    init_camera_platform(500, 500)
    print("state:mine:enter")
    while state == State.StateMine:
        if step == 0:
            _, top_angle, points = color_detcet_top(colors["this"], grad, ksizes, defaults)
            # 调整角度
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue

            if points[0][1] < center_y_start:
                AGC.runAction(Actions["go_s"], 1, action_path)
                continue

            # 进入下一步
            print("state:mine:step 0 done, goto step 1")
            step = 1
        
        if step == 1:
            # 低头
            init_camera_platform(500, 300)
            time.sleep(0.25)
            _, top_angle, points = color_detcet_top(colors["obj"], grad, ksizes, defaults)


# ------------------------- 七、主函数 ------------------------- #
if __name__ == "__main__":
    my_camera = Camera()
    my_camera.camera_open()
    while not frame_flag:
        print("camera:waiting for frame")
        time.sleep(0.1)

    if 0:
        init_action()
        init_camera_platform(500, 400)
        state = State.StateDebug
    else:
        frame_show = True
        state = State.StateMine

    while state != State.StateNone:
        globals().get(StateCalls[state], lambda x:print("Function not found"))(StateColors[state])

    my_camera.camera_close()
    cv2.destroyAllWindows()