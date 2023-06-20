import sys
import cv2
import time
import math
import threading
import numpy as np

from config import *
from function import *

import hiwonder.ActionGroupControl as AGC  # 运行动作文件
import hiwonder.Board as Board  # 控制总线电机

debug = 0
state = 0
step = 0

class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.cali_frame = None
        self.opened = False
        
        # 以子线程的形式获取图像
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self): # 开启
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30) # 帧率
            #self.cap.set(cv2.CAP_PROP_SATURATION, 40) # 饱和度
            self.opened = True
        except Exception as e:
            print('打开摄像头失败:', e)

    def camera_close(self): # 关闭
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('关闭摄像头失败:', e)

    def camera_task(self): # 获取摄像头画面线程
        while True:
            try:
                if self.opened and self.cap.isOpened(): # 判断是否开启
                    ret, frame_tmp = self.cap.read() # 获取画面
                    if ret:
                        self.frame = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST) # 缩放
                        self.cali_frame = cv2.remap(
                            self.frame,
                            map1,
                            map2,
                            interpolation=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT,
                        )                      
                    else:
                        # 如果获取画面失败，则尝试重新打开摄像头
                        self.frame = None
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
                print('获取摄像头画面出错:', e)
                time.sleep(0.01)

my_camera = Camera()
my_camera.camera_open()

# 返回目标区域占比、顶部角度及中点
def color_detcet_top(
    color,
    grad = 2.5,
    angle_none=0, 
    center_none=(340, 480), 
    smooth_size=3, 
    kernerl_size=3,
):
    # 图像处理及roi提取
    img = my_camera.cali_frame.copy()
    img_smooth = cv2.GaussianBlur(img, (smooth_size, smooth_size), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(
        img_transform, color_range[color][0], color_range[color][1]
    )
    kernel = np.ones((kernerl_size, kernerl_size), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    cnts, hieracy = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))  # 计算目标区域占比

    # 得到上顶点，计算中点及线段角度
    if cnt_max is not None:
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

    else:
        top_angle = angle_none
        top_center = center_none

    # 显示
    if debug == 1:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            "area: " + str(percent) + "%",
            (230, 300),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "(x,y): " + str(top_center[0]) + "," + str(top_center[1]),
            (230, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "angle: " + str(top_angle),
            (230, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.circle(vis_img, (top_center[0], top_center[1]), 5, [0, 255, 255], 2)
        cv2.line(vis_img, tuple(top_left), tuple(top_right), (255, 255, 255), 3)
        cv2.imshow("vis_img", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)

    return percent, top_angle, top_center


## 返回目标区域占比、底部角度及中点
def color_detcet_bottom(
    color,
    grad = 2.5,
    angle_none=0, 
    center_none=(340, 480), 
    smooth_size=3, 
    kernerl_size=3
):
    img = my_camera.cali_frame.copy()
    img_smooth = cv2.GaussianBlur(img, (smooth_size, smooth_size), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(
        img_transform, color_range[color][0], color_range[color][1]
    )
    kernel = np.ones((kernerl_size, kernerl_size), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    cnts, hieracy = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))

    if cnt_max is not None:
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

    else:
        bottom_angle = angle_none
        bottom_center = center_none

    if debug == 1:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            "area: " + str(percent) + "%",
            (230, 300),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "(x,y): " + str(bottom_center[0]) + "," + str(bottom_center[1]),
            (230, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "angle: " + str(bottom_angle),
            (230, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.circle(vis_img, (bottom_center[0], bottom_center[1]), 5, [0, 255, 255], 2)
        cv2.imshow("vis_img", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)

    return percent, bottom_angle, bottom_center

# 返回目标区域左上顶点, 角度
def color_detcet_top_left(
    color,
    grad = 2.5,
    angle_none=0, 
    left_none=(340, 480), 
    smooth_size=3, 
    kernerl_size=3,
):
    # 图像处理及roi提取
    img = my_camera.cali_frame.copy()
    img_smooth = cv2.GaussianBlur(img, (smooth_size, smooth_size), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(
        img_transform, color_range[color][0], color_range[color][1]
    )
    kernel = np.ones((kernerl_size, kernerl_size), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    cnts, hieracy = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    cnt_max, area_max = get_maxcontour(cnts)
    percent = round(100 * area_max / ((close.shape[0]) * (close.shape[1])))  # 计算目标区域占比

    # 得到上顶点，计算中点及线段角度
    if cnt_max is not None:
        top_left = cnt_max[0][0]
        top_right = cnt_max[0][0]
        for c in cnt_max:  # 遍历找到四个顶点
            if c[0][0] + grad * c[0][1] < top_left[0] + grad * top_left[1]:
                top_left = c[0]
            if -c[0][0] + grad * c[0][1] < -top_right[0] + grad * top_right[1]:
                top_right = c[0]
        line_top = line(top_left, top_right)
        top_angle = line_top.angle()

    else:
        top_angle = angle_none
        top_left = left_none

    # 显示
    if debug == 1:
        vis_img = img.copy()
        cv2.putText(
            vis_img,
            "(x,y): " + str(top_left[0]) + "," + str(top_left[1]),
            (230, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            vis_img,
            "angle: " + str(top_angle),
            (230, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.circle(vis_img, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
        cv2.line(vis_img, tuple(top_left), tuple(top_right), (255, 255, 255), 3)
        cv2.imshow("vis_img", vis_img)  # 显示图像
        cv2.imshow("close", close)
        cv2.waitKey(1)

    return percent, top_angle, top_left

# 动作
def init_action(name="stand"):
    AGC.runAction(name, 1, action_path)  # 站立
    time.sleep(0.5)
    pass

# 相机云台
def init_camera_platform(left=500, top=500):
    # setBusServoPulse(id, pulse, use_time) param: id, 位置, 运行时间
    Board.setBusServoPulse(19, left, 500)  # id=19 左右
    time.sleep(0.1)  # 延时时间和运行时间相同
    Board.setBusServoPulse(20, top, 500)  # id=20 上下
    time.sleep(0.1)

def adjust_x(x, x_range):
    if x < x_range[0]:
        print("x =", x, "left_go")
        AGC.runAction("left_goN", 1, action_path)
        time.sleep(0.1)
    elif x > x_range[1]:
        print("x =", x, "right_go")
        AGC.runAction("right_goN", 1, action_path)
        time.sleep(0.1)
    else:
        print("x =", x, "stand")

def adjust_angle(angle, angle_range):
    if angle < angle_range[0]:
        print("angle =", angle, "turn_right")
        AGC.runAction("turn_rightN0", 1, action_path)
        time.sleep(0.5)
    elif angle > angle_range[1]:
        print("angle =", angle, "turn_left")
        AGC.runAction("turn_leftN", 1, action_path)
        time.sleep(0.5)
    else:
        print("angle =", angle, "stand")

def state_hole(color_this="green"):
    if not state == 2:
        return

    print("[HOLE]")
    init_action()
    init_camera_platform(500, 400)
    step = 2
    angle_range = [-5, 1]
    big_left_flag = 0
    camera_down = 0
    while state == 2:
        if step == 1:
            _, top_angle, top_left = color_detcet_top_left("green", smooth_size=11, kernerl_size=7)
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue

            if top_left[0] < 210:
                if big_left_flag == 0:
                    AGC.runAction("left_goNe2", 10, action_path)
                    time.sleep(0.2)
                    big_left_flag = 1
                    continue

                if big_left_flag == 1:
                    AGC.runAction("left_goNe2", 2, action_path)
                    time.sleep(0.2)
                    continue
            
            step = 2
            print("[HOLE] step 1 done, goto step 2")
        
        if step == 2:
            _, top_angle, top_left = color_detcet_top_left("green", smooth_size=11, kernerl_size=7)
            if not angle_range[0] < top_angle < angle_range[1]:
                adjust_angle(top_angle, angle_range)
                continue

            if top_left[1] < 350:
                if camera_down == 0:
                    AGC.runAction("standgoN", 1, action_path)
                    time.sleep(0.5)
                    continue
                if camera_down == 1:
                    AGC.runAction("little_goN", 1, action_path)
                    time.sleep(0.5)
                    continue
            else:
                if camera_down == 0:
                    init_camera_platform(500, 300)
                    time.sleep(0.5)
                    camera_down = 1
                    continue
            
            step = -1
            print("[HOLE] step 2 done, goto step 3")

        if step == -1:
            init_camera_platform(500, 300)
            _, top_angle, top_left = color_detcet_top_left("green", smooth_size=11, kernerl_size=7)

    




if __name__ == "__main__":
    init_action()
    init_camera_platform(500, 400)

    debug = 1
    state = 2

    while True:
        state_hole()
        
my_camera.camera_close()
cv2.destroyAllWindows()