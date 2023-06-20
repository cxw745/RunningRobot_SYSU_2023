import cv2
import sys
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
import numpy as np
import time
import threading
import math
from detector import Detector
import Serial_Servo_Running as SSR
import signal
import PWMServo
import pandas as pd
from hiwonder.PID import PID
import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle
from CameraCalibration.CalibrationConfig import *

####a1.duo b 5.duo
#################################################初始化#########################################################
open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
if open_once:
    cap = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
else:
    cap = Camera.Camera()
    cap.camera_open()
Running = True

# state_sel = None
state_sel = 'hole'
Board.setPWMServoPulse(1, 1200, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
Board.setPWMServoPulse(2, 1400, 500)
SSR.running_action_group('0', 1)  # 回中
ret = False  # 读取图像标志位
org_img = None  # 全局变量，原始图像
huizinew_flag=None
flag0=None
detector = None
#count=0
frame_floor_blue=None
stageLeft = ['stairs',  'obstacle',      ##########     剩余关卡
            'mine','huizi_hole' , 'green_bridge',
            'ball_hole',]



################################################读取图像线程###################
def get_img():
    global org_img
    global ret
    global detector
    global cap
    detector = Detector('models/model4.onnx')
    #global orgFrame
    while True:
        ret, org_img = cap.read()
        if ret:
            cv2.imshow('img', org_img)
            #orgFrame = cv2.resize(org_img, (ori_width, ori_height),
                                  #interpolation=cv2.INTER_CUBIC)
            key = cv2.waitKey(1)
            if key == 27:
                break
            else:
                time.sleep(0.01)
# 读取图像线程
th1 = threading.Thread(target=get_img)
th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()

ori_width = int(4 * 160)  # 原始图像640x480
ori_height = int(3 * 160)
r_width = int(4 * 20)  # 处理图像时缩小为80x60,加快处理速度，谨慎修改！
r_height = int(3 * 20)
color_dict = {
    'green': {#hsv
        'Lower': np.array([40, 43, 46]),
        'Upper': np.array([82, 255, 255])
    },
    'ground': {
        'Lower': np.array([0, 123, 0]),
        'Upper': np.array([150, 255, 150])
    },
    'black': {
        'Lower': np.array([0, 25, 0]),
        'Upper': np.array([60, 255, 30])
    },
    'blue': {
        'Lower': np.array([100, 43, 46]),
        'Upper': np.array([124, 255, 255])
    },
    'red': {
        'Lower': np.array([156, 43, 46]),
        'Upper': np.array([180, 255, 255])
    },
    'white': {
        'Lower': np.array([0, 0, 221]),
        'Upper': np.array([180, 30, 255])
    },
    'yellow': {
        'Lower': np.array([15, 43, 46]),
        'Upper': np.array([24, 255, 255])
    },
    'road_white': {
        'Lower': np.array([30, 2, 140]),
        'Upper': np.array([130, 16, 230])
    }
}
color_range = {'yellow_door': [(10, 43, 46), (34, 255, 255)],  # 开门的颜色阈值
               'red_floor1': [(0, 43, 46), (10, 255, 255)],
               'red_floor2': [(156, 43, 46), (180, 255, 255)],
               'green_bridge': [(35, 43, 20), (100, 255, 255)],  # 桥，需要根据比赛现场调整
               'green_hole': [(35, 43, 20), (100, 255, 255)],
               'yellow_hole': [(10, 70, 46), (34, 255, 255)],
               'black_hole': [(0, 0, 0), (180, 255, 80)],
               'black_gap': [(0, 0, 0), (180, 255, 100)],
               'black_dir': [(0, 0, 0), (180, 255, 46)],
               'blue': [(110, 43, 46), (124, 255, 255)],
               'black_door': [(0, 0, 0), (180, 255, 46)],
               'black': [(0, 0, 0), (180, 255, 46)],
               'b': [(15, 45, 0), (125, 255, 225)],  # 雷的阈值
               'red_floor': [(0, 27, 0), (5, 255, 255)],  # 红色阶梯
               'blue_floor': [(104, 0, 0), (118, 255, 167)],  # 蓝色阶梯
               'green_floor': [(31, 46, 172), (179, 255, 229)],  # 绿色阶梯
               'ball': [(0, 0, 202), (179, 65, 255)],  # 球的颜色阈值
               'ball_hole': [(78, 6, 216), (108, 95, 255)],  # 球洞的颜色阈值
               'lei_zhengdui_huizi': [(41, 66, 117), (62, 255, 255)],  #雷区后面为绿色回字时的阈值（绿色的，在现场要测）#################################################
               'ground_left': [(0, 0, 0), (179, 255, 150)], # 地面的阈值，需要测#####################################################################################
               'ground_right': [(0, 0, 0), (179, 255, 150)], # 需要测##############################################################################################
               'ground': [(0, 5, 71), (153, 255, 120)],
                'huizi_black': [(0, 0, 0), (180, 255, 46)],  ###################回字hsv特殊值######################################################################
               }
#################################################  判别程序  #################
def do():
    global org_img
    global detector
    label=None
    time.sleep(1)
    
    if org_img is not None and detector is not None:
        bboxes = detector(org_img[300:,:], threshold=0.5) ##########################################################3截取视角，防止其他关卡干扰,需要测试
        # print(bboxes)
        for class_idx, score, x1, y1, x2, y2 in bboxes:
            # 小数取整
            class_idx, x1, y1, x2, y2 = [int(item) for item in [class_idx, x1, y1, x2, y2]]

            # 获取标签文本
            label = detector.labels[class_idx]
            print(label)
            break
    return label
##############################################  子函数  ########################
#获取最大轮廓以及它的面积
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
#获取最大轮廓以及它的面积
def getAreaMaxContour(contours, area_min = 100):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None
        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c)) #计算轮廓面积
            if contour_area_temp > contour_area_max :
                contour_area_max = contour_area_temp
                if contour_area_temp > area_min:  #最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c
        return area_max_contour, contour_area_max  # 返回最大的轮廓与最大面积
  # 计算面积
def getAreaSumContour(contours):  # 求所有轮廓总面积
    contour_area_sum = 0
    for c in contours:  # 历遍所有轮廓
        contour_area_sum += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
    return contour_area_sum  # 返回总面积
  # 提取最大轮廓
def getAreaMaxContour2(contours, area=100):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  # 面积大于1
                area_max_contour = c
    return area_max_contour
  # 用于门的双框检测，单框难以检测
def getAreaMax_contour_2(contours, area_min=100):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    contour_area_max_2 = 0
    area_max_contour_2 = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max_2 = contour_area_max
            area_max_contour_2 = area_max_contour
            contour_area_max = contour_area_temp
            if contour_area_temp > area_min:  # 最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
        elif contour_area_temp > contour_area_max_2:
            contour_area_max_2 = contour_area_temp
            if contour_area_temp > area_min:  # 最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour_2 = c

    return area_max_contour, contour_area_max, area_max_contour_2, contour_area_max_2,  # 返回最大的轮廓


################################################第一关：开门########################
def start_door():
    global org_img
    r_h = 120
    r_w = 160
    state=1
    if state == 1:  # 初始化
        print('进入start_door')
        step = 0
        Board.setPWMServoPulse(1, 1200, 500)  # NO3机器人，初始云台角度
        Board.setPWMServoPulse(2, 1400, 500)
        time.sleep(1)
    else:
        return
    while state == 1:
        if step == 0:
            border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                        value=(255, 255, 255))  # 扩展白边，防止边界无法识别
            org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
            frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

            frame_door = cv2.inRange(frame_hsv, color_range['yellow_door'][0],
                                     color_range['yellow_door'][1])  # 对原图像和掩模(颜色的字典)进行位运算

            opened = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

            contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            contour_area_sum = getAreaSumContour(contours)

            if contour_area_sum > 800:
                print(contour_area_sum)
                step = 1
        elif step == 1:
            border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                        value=(255, 255, 255))  # 扩展白边，防止边界无法识别
            org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
            frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊化处理
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

            frame_door = cv2.inRange(frame_hsv, color_range['yellow_door'][0],
                                     color_range['yellow_door'][1])  # 对原图像和掩模(颜色的字典)进行位运算

            opened = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算

            contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            contour_area_sum = getAreaSumContour(contours)

        if contour_area_sum > 800:
            print(contour_area_sum)
        else:
            print('go')
            print(contour_area_sum)
            SSR.running_action_group('239new', 1)  # 直行
            step = 0
            state = 2

            # cv2.destroyAllWindows()
##################################################第二关：回字#####################

def square_hole_ground_percent(sub_img, type):
    percent = 0
    if type == 'ground_left':
        frame_gauss = cv2.GaussianBlur(sub_img[:, :320], (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        cv2.drawContours(sub_img, areaMaxContour, -1, (255, 0, 255), 3)  # 画出轮廓
        percent = round(area_max * 100 / (480 * 320), 2)  # 最大轮廓百分比
        # cv2.imshow('frame0',sub_img)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        print('ground_left_percent=', percent)
    if type == 'ground_right':
        frame_gauss = cv2.GaussianBlur(sub_img[:, 320:], (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        cv2.drawContours(sub_img[:, 320:], areaMaxContour, -1, (255, 0, 255), 1)  # 画出轮廓
        percent = round(area_max * 100 / (480 * 320), 2)  # 最大轮廓百分比
        # cv2.imshow('frame1',frame)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        print('ground_right_percent=', percent)
    return percent
def square_hole_ground():
    global org_img
    Board.setPWMServoPulse(1, 1050, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(1)
    while True:
        left = square_hole_ground_percent(org_img, 'ground_left')
        right = square_hole_ground_percent(org_img, 'ground_right')
        # print("square_hole_ground_percent_left:",left)
        # print("square_hole_ground_percent_right:",obs_percent_right)

        if left > 30:  # 20 10 5
            SSR.running_action_group("161", 2)
            # print("square_hole_ground_percent_left-square_hole_ground_percent_right:",left-right)
            print("右移2步")
            time.sleep(0.1)
        if right > 30:  # 20
            # print("square_hole_ground_percent_right-square_hole_ground_percent_left:",right-left)
            SSR.running_action_group("160", 2)
            print("左移2步")
            time.sleep(0.1)

        if right < 30 and left < 30:  # 20 10
            # print("abs(right-left):",abs(right-left))
            print("完成地面边缘识别")
            break

        print("-------------------------")
def square_hole():
    global org_img
    r_w = org_img.shape[1]
    r_h = org_img.shape[0]
    state = 2
    state_sel = 'hole'
    step = 0
    square_hole_ground()
    if (state == 2 or state == 6 or state == 8) and state_sel == 'hole':  # 初始化
        print('进入square_hole')
        step = 0
        Board.setPWMServoPulse(1, 980, 500)  # NO3机器人，初始云台角度
        Board.setPWMServoPulse(2, 1400, 500)
        time.sleep(0.5)
        SSR.running_action_group('stand', 1)
        time.sleep(0.5)  # 等待摄像头稳定

    else:
        return
    close_flag = 0  # 开始靠近回字关卡
    while (state == 2 or state == 6 or state == 8) and state_sel == 'hole':  # 初始化
        if step == 0:
            # close_flag=0#开始靠近回字关卡
            t1 = cv2.getTickCount()
            GaussianBlur_img = cv2.GaussianBlur(org_img, (3, 3), 0)  # 高斯模糊
            # LAB_img = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
            HSV_img = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
            inRange_img = cv2.inRange(HSV_img, color_range['huizi_black'][0],
                                      color_range['huizi_black'][1])  # 根据lab值对图片进行二值化
            opened = cv2.morphologyEx(inRange_img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            opened = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
            # closed = cv2.erode(opened, None, iterations=1)  # 腐蚀
            closed = cv2.dilate(opened, np.ones((3, 3), np.uint8), iterations=10)  # 膨胀
            # opened = cv2.morphologyEx(inRange_img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
            # closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算

            contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour = getAreaMaxContour(contours)[-2]  # 找出最大轮廓
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标
            w = math.sqrt(pow(box[0][0] - box[1][0], 2) + pow(box[0][1] - box[1][1], 2))
            h = math.sqrt(pow(box[0][0] - box[3][0], 2) + pow(box[0][1] - box[3][1], 2))
            area_sum = w * h  # 回字洞的面积
            percent = round(100 * area_sum / (r_w * r_h), 2)  # 回字面积占比

            if areaMaxContour is not None:
                bottom_left = areaMaxContour[0][0]
                bottom_right = areaMaxContour[0][0]
                for c in areaMaxContour:  # 遍历找到四个顶
                    if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                        bottom_left = c[0]
                    if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                        bottom_right = c[0]
                angle_bottom = - math.atan(
                    (bottom_right[1] - bottom_left[1]) / (bottom_right[0] - bottom_left[0])) * 180.0 / math.pi
                bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
                bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            else:
                angle_bottom = 0
                bottom_center_x = 0.5 * r_w
                bottom_center_y = 0
            print('percent=', percent)
            if percent < 20 and close_flag == 0:
                print(
                    '百分比小于n,需要前进')  ###
                SSR.running_action_group('232', 1)
                time.sleep(0.3)
            elif (percent >= 20 and close_flag == 0) or juzhong_flag == 1:
                print('---------靠近回字完成--------------')
                print('bottom_center_y=', bottom_center_y)
                print('angle_bottom=', angle_bottom)
                print('bottom_center_x=', bottom_center_x)
                close_flag = 1
                juzhong_flag = 1

                if bottom_center_y < 400 and close_flag == 1:  # 0.5*r_h 480*0.5    ################
                    print('前进1步，靠近')
                    SSR.running_action_group('232', 1)
                    time.sleep(0.1)
                elif bottom_center_y >= 0.5 * r_h and close_flag == 1:
                    if angle_bottom < -3:
                        SSR.running_action_group('203', 1)
                        print('右转')
                        time.sleep(0.3)
                        # SSR.running_action_group('stand', 1)
                    elif angle_bottom > 3:
                        print('左转')
                        SSR.running_action_group('202', 1)
                        time.sleep(0.3)
                        # SSR.running_action_group('stand', 1)
                    elif -5 <= angle_bottom <= 5:
                        SSR.running_action_group('stand', 1)
                        print("angle_bottom is ok")
                        # print('r_w:',r_w)

                        if bottom_center_x < 0.45 * r_w:  # r_w=640
                            print('左移')
                            SSR.running_action_group('160', 1)
                            time.sleep(0.2)
                        elif bottom_center_x > 0.55 * r_w:
                            print('右移')
                            SSR.running_action_group('161', 1)
                            time.sleep(0.2)
                        else:
                            Board.setPWMServoPulse(1, 980, 500)
                            Board.setPWMServoPulse(2, 1400, 500)
                            SSR.running_action_group('232', 2)
                            SSR.running_action_group('stand', 1)
                            time.sleep(0.3)  ###################################################################
                            SSR.running_action_group('m_passhole', 1)
                            time.sleep(0.3)
                            SSR.running_action_group('stand', 1)
                            # SSR.running_action_group('157', 1)
                            SSR.running_action_group('m_back_onestep', 1)
                            SSR.running_action_group('203', 4)
                            SSR.running_action_group('161', 3)
                            SSR.running_action_group('239new', 2)
                            SSR.running_action_group('157', 1)
                            print('完成居中')
                            step = 1
                            break
################################################第三关：独木桥 #####################
def bridge_zhengdui(type):

    global org_img
    zhengdui_juzhong_flag = 0
    while zhengdui_juzhong_flag == 0:

        time.sleep(1)
        img = org_img[200:, :]
        r_h = org_img.shape[0]
        r_w = org_img.shape[1]

        time.sleep(0.1)
        gauss = cv2.GaussianBlur(img, (3, 3), 0)  # 高斯模糊
        hsv = cv2.cvtColor(gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        Imask0 = cv2.inRange(hsv, color_range[type][0],
                             color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        # opened = cv2.morphologyEx(Imask0, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        # closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # erode = cv2.erode(Imask0, None, iterations=1)  # 腐蚀
        dilate = cv2.dilate(Imask0, np.ones((3, 3), np.uint8), iterations=2)  # 膨胀

        contours, hierarchy = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标

            if areaMaxContour is not None:
                bottom_left = areaMaxContour[0][0]
                bottom_right = areaMaxContour[0][0]
                for c in areaMaxContour:  # 遍历找到四个顶
                    if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                        bottom_left = c[0]
                    if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                        bottom_right = c[0]
                angle_bottom = - math.atan(
                    (bottom_right[1] - bottom_left[1]) / (bottom_right[0] - bottom_left[0])) * 180.0 / math.pi
                bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
                bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            else:
                angle_bottom = 0
                bottom_center_x = 0.5 * r_w
                bottom_center_y = 0
        print('percent=', percent)
        if percent < 5:
            print('percent=', percent)
            SSR.running_action_group('232new', 1)
        elif percent >= 5:
            print('bottom_center_y=', bottom_center_y)
            print('angle_bottom=', angle_bottom)
            print('bottom_center_x', bottom_center_x)
            if angle_bottom < -1.8:
                print('右转')
                SSR.running_action_group('203', 1)
                time.sleep(0.2)
            elif angle_bottom > 2:
                print('左转')
                SSR.running_action_group('202', 1)
                time.sleep(0.2)
            elif -1.8 <= angle_bottom <= 2:
                print('bottom_center_x=', bottom_center_x)
                print('已正对，准备居中')
                if bottom_center_x > 310:
                    print('右移')
                    SSR.running_action_group('161', 1)
                    time.sleep(0.1)
                elif bottom_center_x <280:  # 左移
                    print('左移')
                    SSR.running_action_group('160', 1)
                    time.sleep(0.1)
                elif 280 <= bottom_center_x <= 310:
                    print('stand，准备过桥')
                    SSR.running_action_group('stand', 1)
                    time.sleep(0.2)
                    zhengdui_juzhong_flag = 1
    return zhengdui_juzhong_flag
def bridge_move():
    global org_img
    state=3
    state_sel='bridge'
    step=0

    angle_flag=1
    r_h = org_img.shape[0]
    r_w = org_img.shape[1]
    if (state == 3 or state == 6 or state == 8) and state_sel == 'bridge':  # 初始化
        print("##########################")
        print('进入bridge')
        step = 1
        cnt = 0
        Board.setPWMServoPulse(1, 950, 500)  # NO3机器人，初始云台角度
        Board.setPWMServoPulse(2, 1400, 500)
        time.sleep(0.8)
    else:
        return
    while (state == 3 or state == 6 or state == 8) and state_sel == 'bridge':  # 初始化
        if reset == 1:  # 是否为重置情况
            Board.setPWMServoPulse(1, 950, 500)  # NO3机器人，初始云台角度
            Board.setPWMServoPulse(2, 1400, 500)
            reset = 0
            step = 0
            cnt = 0

            SSR.running_action_group('stand', 1)
        t1 = cv2.getTickCount()
        org_img=org_img[200:,:]
        border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['green_bridge'][0],
                                color_range['green_bridge'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST,
                                                        cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            #center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标


            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - top_right[0]) + 1.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            top_center_x = int((top_right[0] + top_left[0]) / 2)
            top_center_y = int((top_right[1] + top_left[1]) / 2)
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            center_x = int((top_center_x + bottom_center_x) / 2)
            if math.fabs(top_center_x - bottom_center_x) <= 1:  # 得到连线的角度
                angle = 90
            else:
                angle = - math.atan((top_center_y - bottom_center_y) / (top_center_x - bottom_center_x)) * 180.0 / math.pi
        else:
            angle = 90
            center_x = 0.5*r_w

        if step == 1:# 独木桥阶段
            if percent < 1:
                print('接近终点，直行n步离开')# 接近终点，直行2步离开
                c=0
                while c<7:
                    time.sleep(0.2)
                    SSR.running_action_group('239new', 1)
                    c=c+1
                #SSR.running_action_group('239new', 5)
               
                SSR.running_action_group('157', 1)
                state_sel = None
                state += 1
                step = -1 #出
                bridge_flag = 1
                break


            if angle_flag==1:
                c=0
                while c<2:
                    time.sleep(0.1)
                    SSR.running_action_group('232', 1)
                    c=c+1
            #SSR.running_action_group('239new', 3)
            print('percent:',percent)
            print('angle=',angle)
            print('center_x:',center_x)
            print('=============================')
          
            
            if 0 < angle < 88.8:#88
                print("turn right")# 右转
                angle_flag=0
                SSR.running_action_group('stand', 1)
                time.sleep(0.2)
                SSR.running_action_group('203', 1)
                
                time.sleep(0.2)
                SSR.running_action_group('stand', 1)
                
                
            elif -88.8< angle < 0:  # 左转
                print("turn left")
                angle_flag=0
                SSR.running_action_group('stand', 1)
                time.sleep(0.2)
                SSR.running_action_group('202', 1)
                time.sleep(0.2)
                SSR.running_action_group('stand', 1)
            
            elif angle <= -88.8 or angle >= 88.8:
                angle_flag=1
                if center_x > 325:
                    print("dayouyi")
                    SSR.running_action_group('161', 1)
                    time.sleep(0.2)
                    SSR.running_action_group('161', 1)
                    time.sleep(0.2)
                    SSR.running_action_group('stand', 1)
            
                if 325 > center_x > 310:  # 右移 315 310
                    print("youyi")
                    SSR.running_action_group('161', 1)
                    time.sleep(0.1)
                    SSR.running_action_group('stand', 1)
                
                if 260 < center_x <270:#270 280
                    print("zuoyi")
                    SSR.running_action_group('160', 1)
                    time.sleep(0.1)
                    SSR.running_action_group('stand', 1)
                
                elif center_x < 260:
                    print("dazuoyi")# 左移_____________________________
                    SSR.running_action_group('160', 1)
                    time.sleep(0.2)
                    SSR.running_action_group('160', 1)
                    time.sleep(0.3)
                    SSR.running_action_group('stand', 1)
                elif 270 <= center_x <= 310: #310
                    print("go straight")#走三步____________________________
                    c=0
                    while c<3:
                        time.sleep(0.1)
                        SSR.running_action_group('232', 1)
                        c=c+1
                   
                    #SSR.running_action_group('157', 1)
                    time.sleep(0.2)
                    #SSR.running_action_group('239new', 1)
                    cnt += 1
                    if cnt == 2:
                        Board.setPWMServoPulse(1, 950, 500)
                        Board.setPWMServoPulse(2, 1400, 500)
                        time.sleep(0.6)
                
                print('------------------------------------')
def bridge():  # 桥
    Board.setPWMServoPulse(1, 950, 500)
    Board.setPWMServoPulse(2, 1400, 500)
    bridge_zhengdui(type='green_bridge')  #######靠近桥，并正对与居中
    bridge_move()
################################################第四关：雷区 #######################
def obs_percent(sub_img):  # 求障碍物面积占比
    img_area = sub_img.shape[0] * sub_img.shape[1]
    # print(obs_type)
    percent = 0
    if type == 'baffle':
        hsv = cv2.GaussianBlur(sub_img, (3, 3), 0)  # 高斯模糊
        hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        Imask = cv2.inRange(hsv, color_dict['blue']['Lower'],
                            color_dict['blue']['Upper'])  # 对原图像和掩模(颜色的字典)进行位运算
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)  # 腐蚀
        Imask = cv2.erode(Imask, None, iterations=1)  # 膨胀

        cnts = cv2.findContours(Imask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)[-2]  # 找出最初轮廓##
        areaMax_contour, area_max = getAreaMaxContour(cnts)  # 找到最大的轮廓，计算最大面积
        obs_area = area_max
        percent = round(obs_area / img_area * 100, 2)
        print('baffle_percent=', percent)
        # cv2.imshow('imask',Imask)
        # Imask1=cv2.drawContours(orgFrame, areaMax_contour, -1, (255,0,255), 3)  # 画出轮廓
        # cv2.imshow('Imask1',Imask1)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
    if type == 'road_white':
        GaussianBlur_img = cv2.GaussianBlur(sub_img, (3, 3), 0)
        LAB = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
        Imask = cv2.inRange(LAB, color_dict['road_white']['Lower'], color_dict['road_white']['Upper'])  # 根据lab值对图片进行二值化
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
        Imask = cv2.erode(Imask, None, iterations=1)
        cnts = cv2.findContours(Imask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)[-2]
        obs_area = getAreaSumContour(cnts)
        percent = round((1 - obs_area / img_area) * 100, 2)
        print('road_white_percent=', percent)
        # cv2.imshow("Imask",Imask)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
    '''    
    if type == 'lei':
        lab = cv2.cvtColor(sub_img, cv2.COLOR_BGR2LAB)
        lab = cv2.GaussianBlur(lab, (3, 3), 0)

        Imask = cv2.inRange(lab, color_range['black'][0],
                            color_range['black'][1])
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
        # Imask = cv2.erode(Imask, None, iterations=1)
        cnts = cv2.findContours(Imask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)[-2]
        obs_area = getAreaSumContour(cnts)
        percent = round(obs_area / img_area * 100, 2)
        print(percent)
    '''
    if type == 'lei':
        cv2.imwrite("a" + str(d) + ".png", sub_img)
        lab0 = cv2.cvtColor(sub_img, cv2.COLOR_BGR2HSV)
        lab1 = cv2.GaussianBlur(lab0, (3, 3), 0)
        Imask0 = cv2.inRange(lab1, color_range['b'][0],
                             color_range['b'][1])
        # Imask0 = cv2.inRange(lab1, color_range['yellow_door'][0],
        #                color_range['yellow_door'][1])
        Imask1 = cv2.dilate(Imask0, np.ones((3, 3), np.uint8), iterations=2)
        Imask2 = cv2.erode(Imask1, None, iterations=1)
        cv2.imwrite("/home/pi/human_code/picture/" + "deal" + str(d) + ".png", Imask2)
        cnts = cv2.findContours(Imask2, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_NONE)[-2]
        # cv2.imshow("lab0",Imask2)
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        obs_area = getAreaSumContour(cnts)
        percent = round(obs_area / img_area * 100, 2)
        # print(percent)
    if type == 'ground_left':
        frame_gauss = cv2.GaussianBlur(sub_img[:, :320], (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        # cv2.drawContours(sub_img, areaMaxContour, -1, (255, 0, 255), 3)  # 画出轮廓
        percent = round(area_max * 100 / (480 * 320), 2)  # 最大轮廓百分比
        # cv2.imshow('frame0',sub_img)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        print('ground_left_percent=', percent)
    if type == 'ground_right':
        frame_gauss = cv2.GaussianBlur(sub_img[:, 320:], (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        # cv2.drawContours(sub_img[:,320:], areaMaxContour, -1, (255, 0, 255), 1)  # 画出轮廓
        percent = round(area_max * 100 / (480 * 320), 2)  # 最大轮廓百分比
        # cv2.imshow('frame1',frame)
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        print('ground_right_percent=', percent)
    return percent
def lei_zhengdui():
    global huizinew_flag
    global org_img
    #判断是否为挡板
    border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                value=(0, 0, 0))  # 扩展边界，防止边界无法识别
    # org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(border, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

    frame = cv2.inRange(frame_hsv, color_range['lei_zhengdui_huizi'][0],
                        color_range['lei_zhengdui_huizi'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    # print(closed)
    contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    areaMaxContour, area_max0 = getAreaMaxContour(contours)  # 找出最大轮廓
    if area_max0 >5000:  ###############################################################################################################需要测
        type='lei_zhengdui_huizi'
        huizinew_flag='huizi'

    else:
        type='blue'
    #  根据挡板面积是否超过一定的值，判断雷后是挡板还是回字
    # if 读取的结果=='hole'
    # type='lei_zhengdui_huizi'#雷区之后为回字，根据绿色判断以及居中和正对
    # else :
    # type = 'lei_zhengdui_huizi'
    #type = 'blue'

    zhengdui_juzhong_flag = 0
    while zhengdui_juzhong_flag == 0:

        time.sleep(1)
        img = org_img
        # img = org_img[:240, :]
        r_h = org_img.shape[0]
        r_w = org_img.shape[1]

        time.sleep(0.1)

        border = cv2.copyMakeBorder(img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(0, 0, 0))  # 扩展边界，防止边界无法识别
        # org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(border, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
        percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标

            if areaMaxContour is not None:
                bottom_left = areaMaxContour[0][0]
                bottom_right = areaMaxContour[0][0]
                for c in areaMaxContour:  # 遍历找到四个顶
                    if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                        bottom_left = c[0]
                    if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                        bottom_right = c[0]
                angle_bottom = - math.atan(
                    (bottom_right[1] - bottom_left[1]) / (bottom_right[0] - bottom_left[0])) * 180.0 / math.pi
                bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
                bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            else:
                angle_bottom = 0
                bottom_center_x = 0.5 * r_w
                bottom_center_y = 0
        # cv2.drawContours(img, areaMaxContour, -1, (255, 0, 255), 1)  # 画出轮廓
        # cv2.drawContours(org_img, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
        # cv2.imshow('closed', org_img)  # 显示图像
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        print('percent=', percent)
        if percent < 0.02:
            print('percent=', percent)
            SSR.running_action_group('239new', 1)
        elif percent >= 0.02:
            print('bottom_center_y=', bottom_center_y)
            print('angle_bottom=', angle_bottom)
            print('bottom_center_x', bottom_center_x)
            if angle_bottom < -1:
                print('右转')
                SSR.running_action_group('203', 1)
                time.sleep(0.2)
            elif angle_bottom > 1:
                print('左转')
                SSR.running_action_group('202', 1)
                time.sleep(0.2)
            elif -1 <= angle_bottom <= 1:
                print('bottom_center_x=', bottom_center_x)
                # print('0.5 *r_w=', 0.5 * r_w)
                print('已正对，准备居中')
                if bottom_center_x > 350:  # 右移
                    print('右移')
                    SSR.running_action_group('161', 1)
                    time.sleep(0.1)
                elif bottom_center_x < 320:  # 左移
                    print('左移')
                    SSR.running_action_group('160', 1)
                    time.sleep(0.1)
                elif 320 <= bottom_center_x <= 350:
                    print('stand，准备进入雷区')
                    SSR.running_action_group('stand', 1)
                    time.sleep(0.2)
                    zhengdui_juzhong_flag = 1
    return zhengdui_juzhong_flag
def decide(img):  # 确定下一动作 0=直行 1=左转 2=右转
    result = 0

    prior_result = 0
    print("img:", img.shape)
    # cv2.imwrite("first.png", img)
    front_img = img[430:480, 120:450]  # 要调
    # cv2.imwrite("first_cut.png", front_img)
    # big_img = img[90:180, 50:270] #要调
    h, w = front_img.shape[0:2]
    # h2, w2 = big_img.shape[0:2]

    # prior_ground = obs_percent(big_img, obs_type='road_white')

    # if prior_ground >= 20:
    #    big_left = big_img[:, 0:(w2 // 2)]
    #    big_right = big_img[:, (w2 // 2):-1]
    #    #计算地板两边白色占比
    #    ground_left = obs_percent(big_left, obs_type='road_white')
    #    ground_right = obs_percent(big_right, obs_type='road_white')
    #    if ground_left > ground_right:
    #        SSR.running_action_group('zuoyi', 1)
    #        prior_result = 1
    #    else:
    #        SSR.running_action_group('youyi', 1)
    #        prior_result = 2
    # else:
    #    SSR.running_action_group('239new', 1)
    #    prior_result = 0

    # 检测地板占比
    ground_percent = obs_percent(front_img, type='road_white')
    print("ground_percent:", ground_percent)

    # 检测地雷占比
    # print(d)
    time.sleep(1)
    lei_percent = obs_percent(front_img, type='lei')
    print("lei_percent:", lei_percent)
    cv2.imwrite("/home/pi/human_code/picture/" + "b" + str(d) + ".png", front_img)

    if ground_percent >= 22 and lei_percent >= 2.5:
        # count=count+1
        print("front_img:", front_img.shape)

        left_part = front_img[:, 0:(w // 2)]
        left_part_center = front_img[:, (w // 4):(w // 2)]  # 中间偏左
        print("left_part:", left_part.shape)
        print("left_part_center:", left_part_center.shape)

        # cv2.imshow("5555",left_part)
        # cv2.imwrite("front_img"+count+".png", front_img)
        right_part = front_img[:, (w // 2):-1]
        right_part_center = front_img[:, (w // 2):3 * (w // 4)]  # 中间偏右
        # print("right_part:",right_part.shape)
        # print("left_part_center:",left_part_center.shape)

        left_ground = obs_percent(left_part, 'road_white')

        left_lei = obs_percent(left_part, 'lei')
        left_lei_center = obs_percent(left_part_center, 'lei')

        right_ground = obs_percent(right_part, 'road_white')
        # print("right_ground :",right_ground )
        # print("left_ground :",left_ground )

        right_lei = obs_percent(right_part, 'lei')
        right_lei_center = obs_percent(right_part_center, 'lei')

        # print("right_lei_center :",right_lei_center  )
        # print("left_lei_center :",left_lei_center  )
        print("left_lei :", left_lei)
        print("right_lei :", right_lei)

        left_obs = left_ground + left_lei
        right_obs = right_ground + right_lei

        if left_ground - right_ground >= 7:  #############################################################################7
            SSR.running_action_group('160', 10)
            print("left_ground - right_ground>=7:", left_ground - right_ground)
            result = 1
        elif left_ground - right_ground <= -7:  #############################################################################7
            SSR.running_action_group('1611', 10)
            print("left_ground - right_ground<=7:", left_ground - right_ground)
            result = 2
        else:
            if left_obs > right_obs:
                if left_lei_center == 0:

                    SSR.running_action_group("stand", 1)
                    c = 0
                    while c < 6:
                        time.sleep(0.1)
                        SSR.running_action_group('161', 1)
                        c = c + 1
                    SSR.running_action_group("stand", 1)
                    SSR.running_action_group("202", 1)
                    c = 0
                    time.sleep(0.1)
                    while c < 5:
                        time.sleep(0.2)
                        SSR.running_action_group("232", 1)
                        c = c + 1

                    SSR.running_action_group("stand", 1)
                    # SSR.running_action_group("202", 1)
                    # SSR.running_action_group('160', 6)
                    time.sleep(1)
                    print("left_obs > :", left_obs)
                    result = 2
                else:
                    SSR.running_action_group("stand", 1)
                    c = 0
                    while c < 7:
                        time.sleep(0.1)
                        SSR.running_action_group('161', 1)
                        c = c + 1
                    SSR.running_action_group("202", 1)
                    SSR.running_action_group("stand", 1)
                    time.sleep(0.1)
                    c = 0
                    while c < 5:
                        time.sleep(0.2)
                        SSR.running_action_group("232", 1)
                        c = c + 1

                        # SSR.running_action_group("202", 1)
                    # SSR.running_action_group("239new", 5)
                    SSR.running_action_group("stand", 1)
                    # SSR.running_action_group('160', 7)
                    time.sleep(0.5)
                    print("left_obs > little:", left_obs)
                    result = 2

            else:
                if right_lei_center == 0:
                    c = 0
                    while c < 6:
                        time.sleep(0.1)
                        SSR.running_action_group('1601', 1)
                        c = c + 1
                    SSR.running_action_group("202", 1)
                    SSR.running_action_group("stand", 1)
                    time.sleep(0.1)
                    c = 0
                    while c < 5:
                        time.sleep(0.2)
                        SSR.running_action_group("232", 1)
                        c = c + 1
                        # SSR.running_action_group("202", 1)
                    # SSR.running_action_group("239new", 5)
                    SSR.running_action_group("stand", 1)
                    # SSR.running_action_group('161', 5)
                    time.sleep(0.5)
                    print("left_obs< :", left_obs)
                    result = 1
                else:
                    c = 0
                    while c < 7:
                        time.sleep(0.1)
                        SSR.running_action_group('1601', 1)
                        c = c + 1
                    SSR.running_action_group("202", 1)
                    SSR.running_action_group("stand", 1)
                    time.sleep(0.1)
                    c = 0
                    while c < 5:
                        time.sleep(0.2)
                        SSR.running_action_group("232", 1)
                        c = c + 1
                        # SSR.running_action_group("stand", 1)
                    # SSR.running_action_group("202", 1)
                    # SSR.running_action_group("239new", 5)
                    SSR.running_action_group("stand", 1)
                    # SSR.running_action_group('161', 6)
                    time.sleep(1)
                    print("left_obs< little:", left_obs)
                    result = 1

    else:
        # SSR.running_action_group('239new', 1)
        result = 0
    print('result=', result)
    print('prior_result=', prior_result)
    if result == 0:
        return 0
    else:
        if prior_result == 0:
            return result
        else:
            return prior_result
def cross_obs():  # 穿越雷区主函数
    # global org_Frame#, debug
    global org_img
    global d
    d = 0
    Board.setPWMServoPulse(1, 1200, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(0.1)
    lei_zhengdui()
    Board.setPWMServoPulse(1, 1050, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(1)
    while True:

        left = obs_percent(org_img, 'ground_left')
        right = obs_percent(org_img, 'ground_right')
        # print("obs_percent_left:",obs_percent_left)
        # print("obs_percent_right:",obs_percent_right)
        if left - right > 5:  # 20 10 5
            SSR.running_action_group("161", 2)
            print("obs_percent_left-obs_percent_right:", left - right)
            print("右移2步")
            time.sleep(0.1)
        if right - left > 6:  # 20
            print("obs_percent_right-obs_percent_left:", right - left)
            SSR.running_action_group("160", 2)
            print("左移2步")
            time.sleep(0.1)

        if 0 < right - left < 5 or 0 < left - right < 5:  # 20 10
            print("abs(right-left):", abs(right - left))
            print("完成地面边缘识别")
            break

        # print("-------------------------")
    Board.setPWMServoPulse(1, 980, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(0.1)
    SSR.running_action_group("stand", 1)
    print(type(d))
    move_state = decide(org_img)  # 确定动作
    while True:
        d = d + 1
        time.sleep(0.1)

        # if debug:
        # cv2.imshow('org_Frame', org_Frame)  # 显示图像
        # cv2.waitKey(1)
        finish_flag = (
                obs_percent(org_img[:, :], type='baffle') >= 15
        )  # 该关是否结束？ 利用挡板蓝色面积占比判断
        if finish_flag:  # 结束 跳出循环
            print('obs finish!!')
            SSR.running_action_group("stand", 1)
            break
        else:
            print("#####################" + str(d) + "##############")
            print("dangband S:", obs_percent(org_img[:, :], type='baffle'))
            # time.sleep(0.5)
            move_state = decide(org_img)
            print("555555555555555555555")
            SSR.running_action_group("239new", 1)
            # if move_state == 0:  # 直行
            #    SSR.running_action_group("239new", 1)  # 前进
            #    # time.sleep(0.2)
            # elif move_state == 1:  # 左转
            #    SSR.running_action_group("zuoyi", 1)  # 左转
            #    # time.sleep(0.5)
            # else:
            #    SSR.running_action_group("youyi",1)   # 右转
            #    # time.sleep(0.5)
################################################  挡板   ##########################
def zhengdui(type):  # 通用的正对函数

    global org_img
    skip = 1
    img = org_img[:, :480]
    r_h = img.shape[0]
    r_w = img.shape[1]
    while skip == 1:

        time.sleep(0.1)

        border = cv2.copyMakeBorder(img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(0, 0, 0))  # 扩展边界，防止边界无法识别
        # org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(border, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
        # if debug:
        # cv2.imshow('closed', closed)  # 显示图像
        # cv2.waitKey()
        # cv2.destroyAllWindows()
        # cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)  # 画出轮廓
        # cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标
            top_right = areaMaxContour[0][0]  # 右上角
            top_left = areaMaxContour[0][0]  # 左上角
            for c in areaMaxContour:
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - top_right[0]) + 1.5 * top_right[1]:
                    top_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi  # 得到角度
            center_x = (top_right[0] + top_left[0]) / 2  # 得到中心坐标
            center_y = (top_right[1] + top_left[1]) / 2

            # cv2.drawContours(org_img, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
            # print(org_Frame.shape)
            # cv2.imshow('org_Frame',frame_hsv)
            # cv2.waitKey()
            # cv2.destroyAllWindows()

        print("box坐标", box)
        # cv2.imshow('org_Frame', org_img)
        # cv2.waitKey()
        # cv2.destroyAllWindows()

        print('abs(box[0][0]-box[3][0])=', abs(box[0][0] - box[3][0]))
        if (abs(box[0][0] - box[3][0]) >= 90):  ############100
            # print(abs(box[0][1] - box[1][1]))
            if (abs(box[0][1] - box[1][1]) <= 40):  #######  40
                skip = 1
                print("左转")
                SSR.running_action_group("202", 1)
            else:
                skip=0
                SSR.running_action_group("stand", 1)
        else:
            if (abs(box[0][1] - box[1][1]) <= 40):
                skip = 0
                print('stand')
                SSR.running_action_group("stand", 1)
            else:
                skip = 2
                print('右转')
                SSR.running_action_group("203", 1)
    # cv2.imshow('org_Frame',org_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    return skip
    # print(skip)
    # print(percent)
    # return percent,skip     #box[0]最左,,,,box[1]右#####上



def baffler():
    Board.setPWMServoPulse(1, 980, 500)
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(0.5)
    zhengdui(type='blue')
    Board.setPWMServoPulse(1, 1100, 500)
    Board.setPWMServoPulse(2, 1400, 500)
    SSR.running_action_group("232new", 4)
    SSR.running_action_group("kuayuenew1", 1)
    SSR.running_action_group("239new", 1)
    SSR.running_action_group("stand", 1)
    SSR.running_action_group("m_turnleftlarge", 2)
    SSR.running_action_group("m_back", 5)
#############################################   门   ############################

######################################方案二：
# 方案二：
def gate_juzhong(type):

    global org_img
    time.sleep(1)
    img_area = org_img.shape[0] * org_img.shape[1]
    # print(obs_type)
    skip = True
    while skip:
        time.sleep(1)

        hsv0 = cv2.GaussianBlur(org_img, (3, 3), 0)
        hsv1 = cv2.cvtColor(hsv0, cv2.COLOR_BGR2HSV)
        # if obs_type == 'blue':
        Imask0 = cv2.inRange(hsv1, color_dict[type]['Lower'],
                             color_dict[type]['Upper'])
        Imask1 = cv2.erode(Imask0, None, iterations=5)
        Imask2 = cv2.dilate(Imask1, np.ones((3, 3), np.uint8), iterations=1)
        # Imask2 = cv2.erode(Imask1, None, iterations=1)
        contours, hierarchy = cv2.findContours(Imask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        try:
            hierarchy = hierarchy[0]
        except:
            hierarchy = []

        height, width = Imask2.shape
        min_x, min_y = width, height
        max_x = max_y = 0

        for contour, hier in zip(contours, hierarchy):  # 遍历所有轮廓
            (x, y, w, h) = cv2.boundingRect(contour)
            min_x, max_x = min(x, min_x), max(x + w, max_x)
            min_y, max_y = min(y, min_y), max(y + h, max_y)
        print("min_x:", min_x)
        print("max_x:", max_x)

        if max_x > 500 and min_x == 0:
            print("min_x=0,qianjin")
            SSR.running_action_group("232new", 1)
            time.sleep(0.1)
            skip = False
            print("----min_x=0 next----")
            gate_juzhong(type='blue')
            break  ##########################只检测到一边，则前进2步

        '''
        # computes the bounding box for the contour, and draws it on the frame,
        for contour, hier in zip(contours, hierarchy):  # 遍历所有轮廓
            (x, y, w, h) = cv2.boundingRect(contour)
            min_x, max_x = min(x, min_x), max(x + w, max_x)
            min_y, max_y = min(y, min_y), max(y + h, max_y)

            #if w > 80 and h > 80:
            #print("0")
            cv2.rectangle(Imask2, (x, y), (x + w, y + h), (255, 0, 0), 2)  # (x, y)左上角坐标，##(x+w, y+h)右下角坐标
            #print((x, y), (x + w, y + h), w, h)
        '''

        if max_x - min_x > 0 and max_y - min_y > 0:
            # print("1")
            cv2.rectangle(Imask2, (min_x, min_y), (max_x, max_y), (255, 0, 0),
                          2)  # (min_x, min_y)左上角坐标，#####(max_x, max_y)右下角坐标
        # print((min_x, min_y), (max_x, max_y))
        center_x = (min_x + max_x) / 2
        print("center_x:", center_x)
        # print("min_x:",min_x)
        # print("max_x:",max_x)

        if center_x >= 310 and center_x <= 380:  ########################数值范围为猜测，还得测试##################
            skip = False
            print("stand")
            print("juzhong_center_x:", center_x)
            SSR.running_action_group("stand", 1)
        elif center_x < 310:
            print("houtui")
            skip = True
            SSR.running_action_group("m_back", 1)
        else:
            print("qianjin")
            skip = True
            SSR.running_action_group("232new", 1)
            time.sleep(0.1)
        # print(sub_img.shape)
        print("skip:", skip)
        print("--------juzhong--------")
    return skip
    # cv2.imshow('0', Imask2)

    # cv2.waitKey()
    # cv2.destroyAllWindows()
# count_gate = 0
def gate_identify(target_color):
    global org_img
    global count_gate
    time.sleep(0.5)

    img_center_x = org_img.shape[:2][1] / 2  # 获取图像宽度值的一半
    img_center_y = org_img.shape[:2][0] / 2

    GaussianBlur_img = cv2.GaussianBlur(org_img, (3, 3), 0)  # 高斯模糊
    # LAB_img = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    LAB_img = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
    inRange_img = cv2.inRange(LAB_img, color_range[target_color][0], color_range[target_color][1])  # 根据lab值对图片进行二值化
    opened = cv2.morphologyEx(inRange_img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    opened = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    closed = cv2.erode(opened, None, iterations=1)  # 腐蚀
    closed = cv2.dilate(closed, np.ones((3, 3), np.uint8), iterations=1)  # 膨胀
    # opened = cv2.morphologyEx(inRange_img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算
    # closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有外轮廓
    areaMax_contour, contour_area_max, area_max_contour_2, contour_area_max_2 = getAreaMax_contour_2(contours,
                                                                                                     area_min=3)  # 找到最大的轮廓
    cv2.drawContours(org_img, areaMax_contour, -1, (255, 0, 255), 1)  # 画出轮廓
    cv2.drawContours(org_img, area_max_contour_2, -1, (255, 0, 255), 1)  # 画出轮廓

    print("contour_area_max:", contour_area_max)
    print("contour_area_max_2:", contour_area_max_2)
    # cv2.imshow('0',inRange_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    SSR.running_action_group("160", 5)
    if contour_area_max <= 200 or contour_area_max_2 <= 200:
        SSR.running_action_group("160", 2)

        count_gate = count_gate + 2
        return
    if (areaMax_contour is not None) and (area_max_contour_2 is not None):
        rect = cv2.minAreaRect(areaMax_contour)  # rect[0] 矩形中心坐标（x，y）  横上为X  竖下为Y
        X_1 = rect[0][0]
        rect = cv2.minAreaRect(area_max_contour_2)  # rect[0] 矩形中心坐标（x，y）  横上为X  竖下为Y
        X_2 = rect[0][0]

        print("x_1:", X_1)
        print("x_2:", X_2)

    else:
        X_2 = img_center_x
        X_1 = img_center_x
        SSR.running_action_group("160", 3)
        print("前进x_1:", X_1)
        print("前进x_2:", X_2)
        print("160")
        count_gate = count_gate + 2

    check = (X_1 + X_2) / 2
    print(check)
    if (check - img_center_x <= -10) and (check - img_center_x >= -20):
        print("左移动过门")
        SSR.running_action_group("1601", 5)
        count_gate = count_gate + 2
    elif check > img_center_x:
        print("右转")
        SSR.running_action_group("203", 2)
    elif check < img_center_x:
        print("左转")
        SSR.running_action_group("202", 2)
    time.sleep(0.1)
    # cv2.imshow('1',org_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
def gate():
    global count_gate
    time.sleep(0.5)
    count_gate = 0

    Board.setPWMServoPulse(1, 1200, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 2400, 500)
    count = 0
    c = 0

    # zhengdui(type='blue')
    gate_juzhong(type='blue')
    while c < 30:
        time.sleep(0.1)
        SSR.running_action_group("1601", 1)
        c = c + 1

    while True:
        time.sleep(0.5)
        # break
        gate_identify(target_color='blue')
        print("##################", count_gate, "####################")

        if count_gate > 6:
            while c < 24:
                time.sleep(0.1)
                SSR.running_action_group("202", 1)
                c = c + 1

            SSR.running_action_group("m_turnleftlarge", 10)  # 回正
            Board.setPWMServoPulse(1, 980, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
            Board.setPWMServoPulse(2, 1400, 500)
            break
#############################################   球   ###################不需要再改了
"""
#  方案一：活的
def aliveball():
    global org_img
    step=0  # step 为走到  dall_hole  附近的步数，需要测试  ####################################################################
    aliveball_flag=None # 监视器识别到球关卡的 洞，利用洞做一个拐弯的信号
    Board.setPWMServoPulse(1, 980, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 1400, 500)
    # 这里将监视器的识别结果赋给aliveball_flag
    if aliveball_flag is not 'ball_hole':
        SSR.running_action_group("stand", 1)
        Board.setPWMServoPulse(1, 1050, 500)  # 微抬头，拓展视野，便于找到球洞
        Board.setPWMServoPulse(2, 1400, 500)
    else:
        while step< 20:  # 最多走20步就开始转弯  #############################################################################
            SSR.running_action_group("239new", 1)
            step+=1
        SSR.running_action_group("stand", 1)
        SSR.running_action_group("m_turnleft", 7)  # 左转7次，基本90°
            step+=1

aliveball()

"""
# 在直走时纠正轨道，只要改变机器人偏向就行，利用偏向以及动作自身的偏移实现回归方向 ground()函数和雷区的不能共用，单独建立一个函数
#  方案二：死一次的
def ball_ground_percent(sub_img, type):
    percent = 0
    if type == 'ground_left':
        frame_gauss = cv2.GaussianBlur(sub_img[:,:320], (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        cv2.drawContours(sub_img, areaMaxContour, -1, (255, 0, 255), 3)  # 画出轮廓
        percent = round(area_max * 100 / (480*320), 2)  # 最大轮廓百分比
        #cv2.imshow('frame0',sub_img)
        #cv2.waitKey()
        #cv2.destroyAllWindows()
        print('ground_left_percent=', percent)
    if type == 'ground_right':
        frame_gauss = cv2.GaussianBlur(sub_img[:,320:], (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame = cv2.inRange(frame_hsv, color_range[type][0],
                            color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
        contours, hierarchy = cv2.findContours(frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

        cv2.drawContours(sub_img[:,320:], areaMaxContour, -1, (255, 0, 255), 1)  # 画出轮廓
        percent = round(area_max * 100 / (480*320), 2)  # 最大轮廓百分比
        #cv2.imshow('frame1',frame)
        #cv2.waitKey()
        #cv2.destroyAllWindows()
        print('ground_right_percent=', percent)
    return percent
def deadball_gostraight():
    global org_img
    Board.setPWMServoPulse(1, 1050, 500)  # NO3机器人，初始云台角度（1号代表上下，2号代表左右）
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(1)
    while True:
        left = ball_ground_percent(org_img, 'ground_left')
        right = ball_ground_percent(org_img, 'ground_right')
        # print("obs_percent_left:",obs_percent_left)
        # print("obs_percent_right:",obs_percent_right)
        if left - right >= 5:  # 20 10 5
            SSR.running_action_group("161", 2)
            print("obs_percent_left-obs_percent_right:", left - right)
            print("右移2步")
            time.sleep(0.1)
        if right - left >=12:  # 20
            print("obs_percent_right-obs_percent_left:", right - left)
            SSR.running_action_group("160", 2)
            print("左移2步")
            time.sleep(0.1)

        if 0 < right - left < 12 or 0 < left - right < 5:  # 20 10
            print("abs(right-left):", abs(right - left))
            print("-----完成地面边缘识别------")

            break

        print("-------------------------")
def deadball():# 监视器识别到球关卡的 洞，利用洞做一个拐弯的信号
    global org_img
    step0=1 # 步数计数器
    step1=0
    while step0<=6: # 限制走的步数，需要测量  ###################################################################################
        deadball_gostraight()
        SSR.running_action_group("239new", 3)
        SSR.running_action_group("stand", 1)
        if step0%5==0:
            SSR.running_action_group("202", 1)
            print("zhuanzhengjiuzheng")

        # 插入纠正轨道的函数，防止机器人偏移跑到边缘去 也就是ground（）函数
        step0+=1

    SSR.running_action_group("m_turnleftlarge", 9)  #左转90°
    SSR.running_action_group("stand", 1)

    while step1 < 2:  # 限制最多走的步数,靠近台阶  ################################################################################
        SSR.running_action_group("239new", 4)
        deadball_gostraight()

        step1 += 1
    SSR.running_action_group("239new", 2)
#############################################   台阶   ###################不需要再改了
def floor_zhengdui(type):
    global orgFrame
    global debug
    global org_img
    time.sleep(1)
    skip = 0
    img = org_img
    r_h = img.shape[0]
    r_w = img.shape[1]

    time.sleep(0.1)
    gauss = cv2.GaussianBlur(img, (3, 3), 0)  # 高斯模糊
    hsv = cv2.cvtColor(gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
    Imask0 = cv2.inRange(hsv, color_range[type][0],
                         color_range[type][1])  # 对原图像和掩模(颜色的字典)进行位运算
    # opened = cv2.morphologyEx(Imask0, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    # closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    erode = cv2.erode(Imask0, None, iterations=3)  # 腐蚀
    dilate = cv2.dilate(erode, np.ones((3, 3), np.uint8), iterations=1)  # 膨胀
    # erode = cv2.erode(dilate, None, iterations=1)  # 腐蚀
    contours, hierarchy = cv2.findContours(dilate, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    # cv2.drawContours(org_img, contours, 0, (0, 0, 255), 2)

    areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
    # cv2.drawContours(org_img, areaMaxContour, 0, (0, 0, 255), 2)
    # cv2.imshow('org_Frame', org_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)
        # center, w_h, angle = rect  # 中心点 宽高 旋转角度
        box = np.int0(cv2.boxPoints(rect))  # 点的坐标
        top_right = areaMaxContour[0][0]  # 右上角
        top_left = areaMaxContour[0][0]  # 左上角
        for c in areaMaxContour:
            if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                top_left = c[0]
            if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - top_right[0]) + 1.5 * top_right[1]:
                top_right = c[0]
        left_point = np.argmin(box)
        print("left_point:", left_point)
        near1_point = left_point - 1
        near2_point = left_point + 1
        cv2.drawContours(org_img, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
    if left_point == 0:

        center_x = (box[0][0] + box[3][0]) / 2
    else:
        center_x = (box[0][0] + box[1][0]) / 2

    # print("box坐标",box)
    # cv2.imshow('org_Frame', org_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    print("center_x:", center_x)

    if center_x >= 280 and center_x <= 310:  ########################数值范围为猜测，还得测试##################
        skip = 1
        print("stand")
        SSR.running_action_group("stand", 1)
    elif center_x < 280:
        print("左移")
        skip = 0
        SSR.running_action_group("160", 1)
    else:
        print("右移")
        skip = 2
        SSR.running_action_group("161", 1)
    # print(sub_img.shape)

    # cv2.imshow('org_Frame',org_img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    return skip
def floor_juzhong(type):
    global bridge_c
    bridge_c = 0
    floor_juzhong_flag = True
    while floor_juzhong_flag:
        if floor_zhengdui(type=type) == 1:
            # SSR.running_action_group("stand", 1)
            floor_juzhong_flag = False
            break
        elif floor_zhengdui(type=type) == 0:
            # SSR.running_action_group("160", 1)
            time.sleep(0.2)
        else:
            # SSR.running_action_group("161", 1)
            time.sleep(0.2)
        bridge_c = bridge_c + 1
        print("-------------" + str(bridge_c) + "----------")
    return floor_juzhong_flag
def closetofloor(r_w, r_h):
    global org_img
    count = 0
    while True:
        org_img_copy = cv2.resize(org_img, (r_w, r_h), interpolation=cv2.INTER_CUBIC)
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_floor_blue = cv2.inRange(frame_hsv, color_range['blue_floor'][0],
                                       color_range['blue_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        # cv2.imshow('test', frame_floor_blue)
        contours_floor_blue, hierarchy = cv2.findContours(frame_floor_blue, cv2.RETR_EXTERNAL,
                                                          cv2.CHAIN_APPROX_NONE)
        floor_blue_area = getAreaSumContour(contours_floor_blue)
        print("floor_blue_area:", floor_blue_area)
        print("count:", count)
        if floor_blue_area < 10500 and count > 5:
            SSR.running_action_group('232new', 2)
            floor_juzhong('blue_floor')
            SSR.running_action_group('232new', 1)
            return
        else:
            count = count + 1
            SSR.running_action_group('232new', 1)
            time.sleep(1)
def floor():
    r_w = 320
    r_h = 240
    print('进入floor关卡')

    closetofloor(r_w, r_h)  # 靠近台阶边缘

    SSR.running_action_group('up-stairs', 1)
    SSR.running_action_group('232new', 1)
    time.sleep(0.1)
    SSR.running_action_group('up-stairs', 1)
    time.sleep(0.1)
    SSR.running_action_group('232new', 1)

    # SSR.running_action_group('stand', 1)
    SSR.running_action_group('up-stairs', 1)
    SSR.running_action_group('m_downstairs', 1)
    # SSR.running_action_group("m_back", 1)
    time.sleep(0.1)
    SSR.running_action_group('m_downstairs', 1)
    time.sleep(0.5)
    SSR.running_action_group('stand', 1)

    SSR.running_action_group("m_back_onestep", 1)

    floor_juzhong('red_floor2')

    SSR.running_action_group("m_back_onestep", 1)
    SSR.running_action_group('stand', 1)
    SSR.running_action_group("m_downhill1", 1)
    SSR.running_action_group("232", 6)
    SSR.running_action_group("m_downhill2", 1)
#############################################   end_door   ######################
def end_door():
    while True:

        global stop
        r_w = 160
        r_h = 120
        c = 0

        time.sleep(0.8)
        border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

        frame_door = cv2.inRange(frame_hsv, color_range['yellow_door'][0],
                                 color_range['yellow_door'][1])  # 对原图像和掩模(颜色的字典)进行位运算

        opened = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

        contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        contour_area_sum = getAreaSumContour(contours)
        print("contour_area_sum:", contour_area_sum)
        print("c:", c)
        if c == 0 and contour_area_sum > 1000:
            SSR.running_action_group('stand', 1)
            c = 1
        if c == 1 and contour_area_sum < 500:
            SSR.running_action_group('239new', 1)
            stop = stop + 1

        if stop == 20:
            break

        # if contour_area_sum > 800:
        #    print("contour_area_sum:",contour_area_sum)
        #    step = 1
        # else:
        #    print('------go-----')
        #    print("go contour_area_sum:",contour_area_sum)
        #    SSR.running_action_group('239new', 1)  # 直行
        #    step = 2
        #    state = 2

        #    break

#################################################  主程序  ########################
def main():
    global stageLeft
    #start_door()
    label=None
    #print('0')
    Board.setPWMServoPulse(1, 980, 500) 
    Board.setPWMServoPulse(2, 1400, 500)
    time.sleep(1.5)
    label=do()
    #print(label)
    while stageLeft is not None:
        if label is not None:
            if ((label== 'stairs' or label=='blue_step')or(label== 'red_step' or label=='green_step'))or label== 'red_ramp':
                print("####################### 进入台阶关卡、最后的门关卡 ############")
                floor()
                end_door()
                stageLeft.remove('stairs')
            elif label=='mine':
                print("####################### 进入雷区关卡 #####################")
                cross_obs()
                stageLeft.remove('mine')
            elif label=='obstacle'or label=='door':
                print("####################### 进入挡板、门关卡 ####################")
                baffler()
                gate()
                stageLeft.remove('obstacle')
            elif label=='huizi_hole':
                print("####################### 进入回字关卡 #####################")
                square_hole()
                stageLeft.remove('huizi_hole')
            elif label=='green_bridge':
            
                print("####################### 进入独木桥 #####################")
                bridge()
                stageLeft.remove('green_bridge')
            elif label=='ball_hole' or label=='ball':
                print("####################### 进入球关卡 #####################")
                deadball()
                stageLeft.remove('ball_hole')
            else:
                SSR.running_action_group('239new', 1)
                print('此时的关卡是',label)
        else:
            return


main()