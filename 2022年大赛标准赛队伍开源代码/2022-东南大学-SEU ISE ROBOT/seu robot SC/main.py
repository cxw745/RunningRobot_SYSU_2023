#!/usr/bin/env python3
# coding:utf-8

import cv2
import math
import numpy as np
import threading
import time

from tool.CalibrationConfig import *
from tool.RobotConfig import color_range

import rospy
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import CMDcontrol
robot_IP = "10.192.38.171"

camera_out = "chest"
stream_pic = True
action_DEBUG = False
# ################################################初始化#########################################################


box_debug = False
debug = False
img_debug = False

state = 0  # 用数字用来标志第几关
step = 0
stage = 0
state_sel = 'hole'  # 名称来标志第几关
reset = 0
skip = 0
state_lr = 'left'
colo = 'head_blue_stair'
chest_ret = False  # 读取图像标志位--\
ret = False  # 读取图像标志位  |
ChestOrg_img = None  # 原始图像更新    |
HeadOrg_img = None  # 原始图像更新    |这四个都是读取摄像头时的返回参数
ChestOrg_copy = None
HeadOrg_copy = None

chest_r_width = 480
chest_r_height = 640
head_r_width = 640
head_r_height = 480

# ##############################################相机畸变矫正参数加载#################################################
# 加载参数
param_data = np.load(calibration_param_path + '.npz')

# 获取参数
dim = tuple(param_data['dim_array'])
k = np.array(param_data['k_array'].tolist())
d = np.array(param_data['d_array'].tolist())

print('加载参数完成')
print('dim:\n', dim)
print('k:\n', k)
print('d:\n', d)
# 截取区域，1表示完全截取
scale = 1
# 优化内参和畸变参数
p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None)
Knew = p.copy()
if scale:  # change fov
    Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), Knew, dim, cv2.CV_16SC2)

# ##############################################读取图像线程#################################################


class ImgConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub_chest = rospy.Subscriber('/usb_cam_chest/image_raw', Image, self.cb_chest)
        self.sub_head = rospy.Subscriber('/usb_cam_head/image_raw', Image, self.cb_head)
        self.img_chest = None
        self.img_head = None

    def cb_chest(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_chest = cv2_img

    def cb_head(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_head = cv2_img

    def chest_image(self):
        if self.img_chest is not None:
            return True, self.img_chest
        return False, self.img_chest

    def head_image(self):
        if self.img_head is not None:
            return True, self.img_head
        return False, self.img_head


# ###############################################读取图像线程#################################################
rospy.init_node('image_listener')
print('Node init')
image_reader = ImgConverter()


def get_img():
    global ChestOrg_img, chest_ret, HeadOrg_img, ret
    while not rospy.is_shutdown():
        chest_ret, ChestOrg_img = image_reader.chest_image()
        ret, HeadOrg_img = image_reader.head_image()
        if ChestOrg_img is not None:
            # cv2.imshow('1111closed', ChestOrg_img)
            # cv2.imshow('1111111closed', HeadOrg_img)
            # cv2.waitKey(1)
            pass
            # time.sleep(0.05)
            # print("image update ok",ChestOrg_img.shape)
            # cv2.imwrite('./ChestOrg_img.jpg', ChestOrg_img)
        else:
            print("image nome wait")
        time.sleep(0.01)


# 读取图像线程
th1 = threading.Thread(target=get_img)
th1.setDaemon(True)
th1.start()


# ###############################################动作执行线程#################################################
def move_action():
    global step

    CMDcontrol.CMD_transfer()


# 动作执行线程
th2 = threading.Thread(target=move_action)
th2.setDaemon(True)
th2.start()

acted_name = ""


def action_append(act_name):
    global acted_name

    if not action_DEBUG:
        if act_name == "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
            acted_name = "Forwalk02LR"
        elif act_name == "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
            acted_name = "Forwalk02RL"
        elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02LR" or acted_name == "Forwalk02R"):
            CMDcontrol.action_list.append("Forwalk02RS")  # 原来是注释
            acted_name = act_name  # 原来是注释
        elif act_name != "forwardSlow0403" and (acted_name == "Forwalk02RL" or acted_name == "Forwalk02L"):
            CMDcontrol.action_list.append("Forwalk02LS")  # 原来是注释
            acted_name = act_name  # 原来是注释
        elif act_name == "forwardSlow0403":
            acted_name = "Forwalk02R"
        else:
            acted_name = act_name

        CMDcontrol.actionComplete = False
        if len(CMDcontrol.action_list) > 0:
            print("队列超过一个动作")
            CMDcontrol.action_list.append(acted_name)
        else:
            CMDcontrol.action_list.append(acted_name)
        CMDcontrol.action_wait()

    else:
        print("-----------------------执行动作名：", act_name)
        time.sleep(2)

# ###############################################处理函数#################################################


def getAreaMaxContour1(contours):  # 返回轮廓 和 轮廓面积
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  # 只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓


def getAreaValue(contours):
    contour_area = 0
    for c in contours:  # 历遍所有轮廓
        contour_area += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
    return contour_area


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 轮廓面积大于25时最大轮廓才有效
                area_max_contour = c
    return area_max_contour, contour_area_max


def getAreaMaxContour_hole(contour, org_img_copy):
    box_rect_max = []
    r_w = 480
    r_h = 640
    area_max_contour = None
    if contour is not None:
        top_left = contour[0][0]
        top_right = contour[0][0]
        bottom_left = contour[0][0]
        bottom_right = contour[0][0]
        for c in contour:
            if c[0][0] + 1.8 * c[0][1] < top_left[0] + 1.8 * top_left[1]:
                top_left = c[0]
            if (r_w - c[0][0]) + 2.8 * c[0][1] < (r_w - top_right[0]) + 2.8 * top_right[1]:
                top_right = c[0]
            if c[0][0] + 1.8 * (r_h - c[0][1]) < bottom_left[0] + 1.8 * (r_h - bottom_left[1]):
                bottom_left = c[0]
            if c[0][0] + 1.8 * c[0][1] > bottom_right[0] + 1.8 * bottom_right[1]:
                bottom_right = c[0]
        if debug:
            cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
            cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
            cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
            cv2.circle(org_img_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)

        box_rect_max = [top_left, top_right, bottom_right, bottom_left]
    return box_rect_max  # 返回最大的轮廓


def getAreaMaxContour_door(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 500:  # 轮廓面积大于25时最大轮廓才有效
                area_max_contour = c
    return area_max_contour, contour_area_max


def getAreaMaxContour_door2(contours, contour_area_max):
    contour_area_temp = 0
    contour_area_max2 = 0
    area_max_contour2 = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max2 and contour_area_max != contour_area_temp:
            contour_area_max2 = contour_area_temp
            if contour_area_temp > 500:  # 轮廓面积大于25时最大轮廓才有效
                area_max_contour2 = c
    return area_max_contour2, contour_area_max2




def getrobotcenter(r_w, r_h, color):
    global ChestOrg_img, HeadOrg_img
    # dst = cv2.undistort(org_img, mst, dist, None, newmst)
    dst = cv2.remap(HeadOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    org_cut = dst[260:, :]  # y,x
    border = cv2.copyMakeBorder(org_cut, 268, 8, 6, 6, borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255))
    org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)
    frame_color = cv2.inRange(frame_hsv, color_range[color][0], color_range[color][1])
    opened = cv2.morphologyEx(frame_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if len(contours):
        areaMaxContour = max(contours, key=cv2.contourArea)
        if areaMaxContour is not None and cv2.contourArea(areaMaxContour) > 400:
            top_right = areaMaxContour[0][0]
            top_left = areaMaxContour[0][0]
            for c in areaMaxContour:
                if c[0][0] + 1.8 * c[0][1] < top_left[0] + 1.8 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
            center = (top_right[0] + top_left[0]) / 2
            angle = -math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle += 2.5
            dirtowall1 = r_h - int((top_right[1] + top_left[1]) / 2)
            if debug:
                cv2.imshow('closed_center', closed)  # 显示图像
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.imshow('org_img_copy2', org_img_copy)
                cv2.waitKey(1)
            print('center:',center)
            print('angle:', angle)
            return center,angle,dirtowall1
        else:
            return r_w / 2,0,0
    else:
        return r_w / 2,0,0


def getrohongzhuancenter(r_w, r_h, color):
    global ChestOrg_img, HeadOrg_img
    # dst = cv2.undistort(org_img, mst, dist, None, newmst)
    dst = cv2.remap(ChestOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    border = cv2.copyMakeBorder(dst, 8, 8, 6, 6, borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255))
    org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)
    frame_color = cv2.inRange(frame_hsv, color_range[color][0], color_range[color][1])
    opened = cv2.morphologyEx(frame_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if contours != () and contours != None:
        areaMaxContour = max(contours, key=cv2.contourArea)
        if areaMaxContour is not None and cv2.contourArea(areaMaxContour) > 400:
            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 0.5 * c[0][1] > bottom_right[0] + 0.5 * bottom_right[1]:
                    bottom_right = c[0]
            angleb = - math.atan((bottom_right[1] - bottom_left[1]) / (bottom_right[0] - bottom_left[0])) * 180.0 / math.pi
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            right_center_x = int((bottom_right[0] + top_right[0]) / 2)
            dirtoline_bottom = 480 - bottom_center_y
            if debug:
                cv2.imshow('closed_center', closed)  # 显示图像
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.imshow('org_img_copy2', org_img_copy)
                cv2.waitKey(1)
            print('bottom_center_x:',bottom_center_x)
            print('angleb:', angleb)
            print('dirtoline_bottom',dirtoline_bottom)
            return bottom_center_x,angleb,dirtoline_bottom
        else:
            return r_w / 2,0,0
    else:
        return r_w / 2,0,0


def gethongzhuancenter(r_w, r_h, color):
    global ChestOrg_img, HeadOrg_img
    # dst = cv2.undistort(org_img, mst, dist, None, newmst)

    dst = cv2.remap(HeadOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    border = cv2.copyMakeBorder(dst, 8, 8, 6, 6, borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255))
    org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)
    frame_color = cv2.inRange(frame_hsv, color_range[color][0], color_range[color][1])
    opened = cv2.morphologyEx(frame_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))
    (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if contours != ():
        areaMaxContour = max(contours, key=cv2.contourArea)
        if areaMaxContour is not None and cv2.contourArea(areaMaxContour) > 400:
            top_right = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:
                if (r_w - c[0][0]) + 2.1 * c[0][1] < (r_w - top_right[0]) + 2.1 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            angle = - math.atan((top_right[1] - bottom_right[1]) / (top_right[0] - bottom_right[0])) * 180.0 / math.pi
            right_center_x = int((bottom_right[0] + top_right[0]) / 2)
            dirtolin_right = 320 - right_center_x
            if debug:
                cv2.imshow('closed_center', closed)  # 显示图像
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.imshow('org_img_copy2', org_img_copy)
                # cv2.waitKey(1)
            print('dirtolin_right:',dirtolin_right)
            #print('angle:', angle)
            return dirtolin_right
        else:
            return 1
    else:
        return 1


def getwhitecenter(r_w, r_h, color):
    global ChestOrg_img, HeadOrg_img
    # dst = cv2.undistort(org_img, mst, dist, None, newmst)

    dst = cv2.remap(ChestOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    org_cut = dst[260:, :]  # y,x
    frame_gauss = cv2.GaussianBlur(org_cut, (3, 3), 0)
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)
    frame_color = cv2.inRange(frame_hsv, color_range[color][0], color_range[color][1])
    opened = cv2.morphologyEx(frame_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    if len(contours):
        areaMaxContour = max(contours, key=cv2.contourArea)
        if areaMaxContour is not None and cv2.contourArea(areaMaxContour) > 400:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标
            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle += 2.5
            return top_left[0],top_right[0]
        else:
            return 0,640
    else:
        return 0,640

def getrobotcenter_dilei(r_w, r_h, color):
    global ChestOrg_img, HeadOrg_img
    # dst = cv2.undistort(org_img, mst, dist, None, newmst)

    dst = cv2.remap(HeadOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    org_cut = dst[250:, :]  # y,x
    border = cv2.copyMakeBorder(org_cut, 258, 8, 6, 6, borderType=cv2.BORDER_CONSTANT, value=(255, 255, 255))
    org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)
    frame_color = cv2.inRange(frame_hsv, color_range[color][0], color_range[color][1])
    opened = cv2.morphologyEx(frame_color, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours_suit = getbigcontours(contours)
    print('********',len(contours_suit))
    if len(contours_suit):
        areaMaxContour,_ = getpoint_obstacle(contours_suit)
        if areaMaxContour is not None:
            top_right = areaMaxContour[0][0]
            top_left = areaMaxContour[0][0]
            for c in areaMaxContour:
                if c[0][0] + 1.8 * c[0][1] < top_left[0] + 1.8 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
            center = (top_right[0] + top_left[0]) / 2
            angle = -math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle += 2.5
            dirtowall1 = r_h - int((top_right[1] + top_left[1]) / 2)
            if debug:
                cv2.imshow('closed_center', closed)  # 显示图像
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.imshow('org_img_copy2', org_img_copy)
                cv2.waitKey(1)
            print('center:',center)
            print('angle:', angle)
            return center,angle,dirtowall1
        else:
            return r_w / 2,0,0
    else:
        return r_w / 2,0,0


def getbigcontours(contours):
    lis = []
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > 300:
            lis.append(c)
    return lis

def getpoint_obstacle(contours):
    contour_x_temp = 0
    contour_x_max = 0
    area_max_contour = None
    for area in contours:
        b = area[0][0]
        for c in area:
            if (c[0][0]) > (b[0]):
                b = c[0]
                contour_x_temp=b[0]
        if contour_x_temp > contour_x_max:
            contour_x_max = contour_x_temp
            area_max_contour = area
    return area_max_contour,contour_x_max

def getbigcontours_door(contours):
    contour_area_temp = 0
    contour_area_max = 0
    lis = []
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > 1000:
            lis.append(c)
    return lis


def getbigcontours_doo(contours):
    contour_area_temp = 0
    contour_area_max = 0
    lis = []
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > 2000:
            lis.append(c)
    return lis


def getpoint1(contours):
    contour_x_temp = 0
    contour_x_max = 0
    area_max_contour = None
    for area in contours:
        b = area[0][0]
        for c in area:
            if (c[0][1]) > (b[1]):
                b = c[0]
                contour_x_temp=b[0]
        if contour_x_temp > contour_x_max:
            contour_x_max = contour_x_temp
            area_max_contour = area
    return area_max_contour,contour_x_max



def getpoint2(contours,contour_x_max1):
    contour_x_temp = 0
    contour_x_max = 0
    area_max_contour = None
    for area in contours:
        b = area[0][0]
        for c in area:
            if (c[0][1]) > (b[1]):
                b = c[0]
                contour_x_temp = b[0]
        if contour_x_temp > contour_x_max and contour_x_temp != contour_x_max1:
            contour_x_max = contour_x_temp
            area_max_contour = area
    return area_max_contour,contour_x_max


def iscolorgeted(img, color):
    frame_green = cv2.inRange(img, color_range[color][0],
                              color_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    # print(closed)
    (cnts_green, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                               cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    areaMaxContour, area_max = getAreaMaxContour(cnts_green)  # 找出最大轮廓
    if area_max > 1000:
        return True
    else:
        return False


def iniparm():
    global state, step, stage, skip
    step = 0
    stage = 0
    state = 0
    skip = 0


# ###############################################1:起始开门关#################################################
def start_door():
    global HeadOrg_img, step, img_debug
    step = 0
    while True:
        if HeadOrg_img is None:
            continue
        time.sleep(1.5)
        if 1:  # 判断门是否抬起
            t1 = cv2.getTickCount()  # 时间计算
            handling = HeadOrg_img.copy()

            border = cv2.copyMakeBorder(handling, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                        value=(255, 255, 255))  # 扩展白边，防止边界无法识别
            handling = cv2.resize(border, (chest_r_width, chest_r_height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
            frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)  # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间

            frame_door_yellow = cv2.inRange(frame_hsv, color_range['yellow_door'][0],
                                            color_range['yellow_door'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            frame_door = cv2.add(frame_door_yellow, 0)
            open_pic = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 开运算 去噪点
            closed_pic = cv2.morphologyEx(open_pic, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # 闭运算 封闭连接
            # print(closed_pic)

            (contours, hierarchy) = cv2.findContours(closed_pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
            areavalue = getAreaValue(contours)  # 计算面积
            percent = round(100 * areavalue / (chest_r_width * chest_r_height), 2)  # 最大轮廓的百分比
            #cv2.imwrite('./closed_pic.jpg', closed_pic)  # 查看识别情况
            print(percent)
            if areaMaxContour is not None:
                rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
                box = np.int0(cv2.boxPoints(rect))  # 点的坐标
                if img_debug:
                    cv2.drawContours(handling, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上
            if img_debug:
                cv2.putText(handling, 'area: ' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 0, 255), 2)
                t2 = cv2.getTickCount()
                time_r = (t2 - t1) / cv2.getTickFrequency()
                fps = 1.0 / time_r
                cv2.putText(handling, "fps:" + str(int(fps)), (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.imshow('handling',handling)
                cv2.imshow('mask', closed_pic)
                cv2.waitKey(1)
            if step == 0:
                if percent > 2:  # 检测到横杆
                    print(percent, "%")
                    print("有障碍 等待 contours len：", len(contours))
                    time.sleep(0.1)
                    step = 1
                else:
                    print(percent)
            elif step == 1:
                if percent > 1:  # 检测到横杆
                    print(percent)
                    print("222222有障碍 等待 contours len：", len(contours))
                else:
                    print("开启下一关")
                    # is_door_open = True
                    action_append("fastForward02")
                    time.sleep(0.5)
                    action_append("fastForward01")
                    break
    return






# ###############################################2:坑洞关#################################################


def gothroughhole():
    global HeadOrg_img, state, state_sel, step
    global head_r_height, head_r_width
    angle = []
    step = 8
    center_x = []
    center_y = []
    dirtoline = []
    state_sel = 'hole'
    while True:
        if HeadOrg_img is None:
            continue
        org_img = HeadOrg_img.copy()
        dst = cv2.remap(org_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        org_cut = dst[260:,:]  # y,x
        border = cv2.copyMakeBorder(org_cut, 265, 5, 5, 5, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        # org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        org_img_copy = border  # 将图片缩放

        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['head_green'][0],
                                  color_range['head_green'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        # frame_green = cv2.inRange(frame_hsv, (p1,p2,p3),(p4,p5,p6))  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        # contour = contours[1]
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        if areaMaxContour is not None:
            top_left, top_right, bottom_left, bottom_right = getAreaMaxContour_hole(areaMaxContour, org_img_copy)
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle += 2.0
            center_x = (top_right[0] + top_left[0]) / 2
            center_y = (top_right[1] + top_left[1]) / 2
            dirtoline = head_r_height - center_y
        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, areaMaxContour, -1, (255, 0, 255), 3)
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_y:" + str(int(center_y)), (330, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline:" + str(int(dirtoline)), (330, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.imshow('org_img_copy1', org_img_copy)
            cv2.waitKey(1)
        print(angle, center_x, dirtoline)
        if step == 8:
            if angle < -3:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 3 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else: step = 0
        elif step == 0:  # 接近独木桥阶段
            # AGC.runAction('go_forward')
            if dirtoline < 95:  # 绿色区域大于30%
                print("已接坑洞，开始调整角度")
                step = 1
            else:
                print("继续接近坑洞")
                action_append('fastForward00')
                time.sleep(1)
        elif step == 1:  # 独木桥阶段
            angle -= 1.8
            if angle < -2:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 2 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.5)
            elif -2<= angle <= 2:
                step = 2
                print('正對')
        elif step == 2:
            angle -= 1.8
            if angle < -2:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 2 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.5)
            elif -2<= angle <= 2:
                if center_x < 380:
                    print('左移')
                    action_append('Left3move')
                    time.sleep(0.5)
                elif center_x < 410:  # 左转
                    print('左移')
                    action_append('Left1move')
                    time.sleep(0.5)
                elif center_x >= 410:
                    step = 3
        elif step == 3:  # 独木桥阶段
            print("前进就好")
            action_append('Forwalk02part1')
            time.sleep(0.5)
            action_append('Forwalk02part2')
            time.sleep(0.5)
            action_append('Forwalk02part3')
            # action_append('fastForward01')
            # time.sleep(1)
            action_append('fastForward02')
            time.sleep(1)
            action_append('fastForward00')
            time.sleep(1)
            action_append('turn001L')
            time.sleep(1)
            action_append('turn001L')
            break
    return

################################################3:地雷关#################################################


def gothroughdilei():
    global HeadOrg_img, state, state_sel, step, ChestOrg_img, stage,state_lr, skip
    global head_r_height, head_r_width
    step = 8
    skip = 0
    angle = []
    lei1_x = []
    cnt_skip = 0
    cnt = 0
    flag_dilei = 1
    dirtolei = 220
    dirtowall = 200
    while True:
        if ChestOrg_img is None:
            continue
        org_img = ChestOrg_img.copy()
        org_cut = org_img[260:-40, 200:-200]  # y,x

        border = cv2.copyMakeBorder(org_cut, 259, 38, 200, 200, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别 t,b,l,r
        org_img_copy = border  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        if stage == 0:
            flag = iscolorgeted(frame_hsv, 'blue_obstacle_chest')
            if flag:
                stage = 1
                print('靠近墙壁开始')
        else:
            frame_wall = cv2.inRange(frame_hsv, color_range['blue_obstacle_chest'][0],
                                     color_range['blue_obstacle_chest'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            opened_wall = cv2.morphologyEx(frame_wall, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            closed_wall = cv2.morphologyEx(opened_wall, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
            # print(closed)
            (cnts_wall, hierarchy_wall) = cv2.findContours(closed_wall, cv2.RETR_LIST,
                                                           cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
            areaMaxContour_wall, area_max_wall = getAreaMaxContour(cnts_wall)  # 找出最大轮廓
            if areaMaxContour_wall is not None:
                top_left, top_right, bottom_left, bottom_right = getAreaMaxContour_hole(areaMaxContour_wall,
                                                                                        org_img_copy)
                angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
                wall_x = (bottom_right[0] + bottom_left[0]) / 2
                wall_y = (bottom_right[1] + bottom_left[1]) / 2
                # print(lei1_y);
                dirtowall = head_r_height - wall_y
                if debug:
                    cv2.putText(org_img_copy, "wall_x:" + str(int(wall_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(org_img_copy, "wall_y:" + str(int(wall_y)), (480, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(org_img_copy, "dirtolei1:" + str(int(dirtowall)), (480, 100), cv2.FONT_HERSHEY_SIMPLEX,
                                0.65,
                                (0, 0, 0), 2)  # (0, 0, 255)BGR
            if debug:
                cv2.imshow('closed_wall', closed_wall)  # 显示图像
        frame_green = cv2.inRange(frame_hsv, color_range['chest_black_louxia'][0],
                                          color_range['chest_black_louxia'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        # contour = contours[1]
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        # getrobotcenter(640, 480, 'chest_green')
        if areaMaxContour is not None:
            top_left, top_right, bottom_left, bottom_right = getAreaMaxContour_hole(areaMaxContour, org_img_copy)
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            lei1_x = (top_right[0] + top_left[0]) / 2
            lei1_y = (top_right[1] + top_left[1]) / 2
            # print(lei1_y);
            dirtolei = head_r_height - lei1_y
            if area_max > 200:
                flag_dilei = 1
            else:
                flag_dilei = 0
            if debug:
                cv2.imshow('closed', closed)  # 显示图像
                cv2.imshow('cut', org_cut)  # 显示图像
                cv2.putText(org_img_copy, "lei1_x:" + str(int(lei1_x)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 0, 0), 2)  # (0, 0, 255)BGR
                cv2.putText(org_img_copy, "lei1_y:" + str(int(lei1_y)), (480, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 0, 0), 2)  # (0, 0, 255)BGR
                cv2.putText(org_img_copy, "dirtolei1:" + str(int(dirtolei)), (480, 300), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (0, 0, 0), 2)  # (0, 0, 255)BGR
                cv2.imshow('org_img_copy1', org_img_copy)
                cv2.waitKey(1)
        else:
            flag_dilei = 0
            print('None')

        print('lei1_x, dirtolei: ', lei1_x, dirtolei)
        print('stage', stage)
        print('step', step)

        if stage == 0:  # 地雷阶段
            if step == 8:
                if state_sel == 'hole':
                    action_append('fastForward00')
                    time.sleep(0.5)
                    action_append('Right3move')
                    time.sleep(0.5)
                    while True:
                        center1, angle1, _ = getrobotcenter_dilei(640, 480, 'blue_obstacle_head')
                        print('——————————————————————————head_bluewall——————————————————————————————————')
                        print('angle:', angle1)
                        if angle1 > 2.5:
                            action_append('turn001L')
                            time.sleep(1)
                        elif angle1 < -1.5:
                            action_append('turn000R')
                            time.sleep(1)
                        else:
                            if center1 > 330:  # 右移
                                print("右移")
                                action_append('Right3move')
                                time.sleep(0.1)
                            else:  # 走三步
                                print("角度调整完毕，继续过地雷")
                                step = 0
                                time.sleep(1)
                                break
                else:
                    action_append('fastForward00')
                    time.sleep(0.5)
                    action_append('Left3move')
                    time.sleep(0.5)
                    action_append('fastForward00')
                    time.sleep(0.5)
                    action_append('Left3move')
                    action_append('Left1move')
                    step = 0
            elif step == 0:
                if dirtolei <= 120:
                    step = 1
                    if cnt == 0:
                        if lei1_x > 330:
                            state_lr = 'left'
                            print('left:', lei1_x)
                        else:
                            state_lr = 'right'
                            print('right:', lei1_x)
                    else:
                        centerg, angle1, _ = getrobotcenter_dilei(640, 480, 'blue_obstacle_head')
                        if centerg > 350:
                            state_lr = 'right'
                            print('right:', lei1_x)
                        elif centerg < 290:
                            state_lr = 'left'
                            print('left:', lei1_x)
                        else:
                            pass
                elif 120 < dirtolei <= 200:
                    action_append('Forwalk00')
                elif dirtolei > 200:
                    action_append('fastForward00')
                    while True:
                        _, angle1, _ = getrobotcenter_dilei(640, 480, 'blue_obstacle_head')
                        if angle1 > 3:
                            action_append('turn001L')
                            time.sleep(1)
                        elif angle1 < -1.5:
                            action_append('turn000R')
                            time.sleep(1)
                        else:
                            break
            elif step == 1:
                if lei1_x <= 240 and not skip:
                    print('被迫被迫被迫被迫被迫被迫右移')
                    action_append('Right3move')
                    action_append('Stand')
                    action_append('Right1move')
                    cnt_skip += 1
                    if cnt_skip == 2:
                        state_lr = 'left'
                    else:
                        pass
                    step = 2
                elif lei1_x >= 380 and not skip:
                    print('被迫被迫被迫被迫被迫被迫左移')
                    action_append('Left3move')
                    action_append('Stand')
                    action_append('Left1move')
                    cnt_skip -= 1
                    if cnt_skip == -2:
                        state_lr = 'right'
                    else:
                        pass
                    step = 2
                elif state_lr == 'right':
                    if lei1_x > 280:
                        print('右移')
                        action_append('Right3move')
                        skip = 1
                        cnt += 1
                    else:
                        print('右移')
                        action_append('Right3move')
                        action_append('Right3move')
                        print('停止')
                        state_lr = 'left'
                        skip = 0
                        cnt += 1
                        step = 2
                elif state_lr == 'left':
                    if lei1_x < 640 - 280:
                        print('左移')
                        action_append('Left3move')
                        skip = 1
                        cnt += 1
                    else:
                        print('左移')
                        action_append('Left3move')
                        action_append('Left3move')
                        print('停止')
                        state_lr = 'right'
                        skip = 0
                        cnt += 1
                        step = 2
            elif step == 2:
                print('前进')
                action_append('fastForward01')
                time.sleep(1)
                while True:
                    _, angle1, _ = getrobotcenter_dilei(640, 480, 'blue_obstacle_head')
                    if angle1 > 3:
                        action_append('turn001L')
                        time.sleep(1)
                    elif angle1 < -1.5:
                        action_append('turn000R')
                        time.sleep(1)
                    else:
                        break
                dirtolei = 220
                step = 0

            elif step == 11:
                print('特殊左移动')
                if lei1_x < 640 - 270:
                    print('左移')
                    action_append('Left3move')
                    skip = 1
                    cnt += 1
                else:
                    print('左移')
                    action_append('Left3move')
                    action_append('Left3move')
                    print('停止')
                    state_lr = 'right'
                    skip = 0
                    cnt += 1
                    step = 2
            elif step == 12:
                print('特殊右移动')
                if lei1_x > 270:
                    print('右移')
                    action_append('Right3move')
                    skip = 1
                    cnt += 1
                else:
                    print('右移')
                    action_append('Right3move')
                    action_append('Right3move')
                    print('停止')
                    state_lr = 'left'
                    skip = 0
                    cnt += 1
                    step = 2

        if stage == 1:
            print('地雷关结束')
            return
    return
# ###############################################4:翻墙关#################################################


def overwall():
    global state, state_sel, org_img, step, reset, skip, debug,ChestOrg_img
    print('进入翻墙关，翻啊翻啊')
    step = 0
    r_w = head_r_width
    r_h = head_r_height
    time.sleep(0.5)
    while True:  # 初始化
        if ChestOrg_img is None:
            continue
        org_img = ChestOrg_img.copy()
        t1 = cv2.getTickCount()
        dst = cv2.remap(org_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        border = cv2.copyMakeBorder(dst, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['blue_door_chest'][0],
                                  color_range['blue_door_chest'][1])  # 对原图像和掩模(颜色的字典)进行位运算

        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)
        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)
            cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标

            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle_b = - math.atan((bottom_right[1] - bottom_left[1]) / (bottom_right[0] - bottom_left[0])) * 180.0 / math.pi
            top_center_x = int((top_right[0] + top_left[0]) / 2)
            top_center_y = int((top_right[1] + top_left[1]) / 2)
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            center_x = int((top_center_x + bottom_center_x) / 2)
            dirtoline_bottom = head_r_height - bottom_center_y
            dirtoline_top = head_r_height - top_center_y
            if debug:
                cv2.drawContours(org_img_copy, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_center_x, top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_center_x, bottom_center_y), 5, [0, 255, 255], 2)
                cv2.line(org_img_copy, (top_center_x, top_center_y), (bottom_center_x, bottom_center_y), [0, 255, 255],
                         2)  # 画出上下中点连线

        else:
            angle_b = 0
            center_x = 0.5 * r_w
        if debug:
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_top:" + str(int(dirtoline_top)), (430, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_bottom:" + str(int(dirtoline_bottom)), (430, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            # cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
            cv2.waitKey(1)
        print(angle_b,dirtoline_top)
        if step == 0:
            if angle_b < -2:
                print('右转')
                action_append('turn000R')
                time.sleep(0.1)
            elif angle_b > 2:
                print('左转')
                action_append('turn001L')
                time.sleep(0.1)
            else: step = 1
        elif step == 1:  # 接近独木桥阶段
            # AGC.runAction('go_forward')
            if dirtoline_top < 230:  # 绿色区域大于30%
                if angle_b < -2:
                    print('右转')
                    action_append('turn000R')
                    time.sleep(0.1)
                elif angle_b > 2:
                    print('左转')
                    action_append('turn001L')
                    time.sleep(0.1)
                else:
                    step = 3
            else:
                print("继续接近独木桥")
                action_append('fastForward00')
                time.sleep(1)
        elif step == 2:  # 接近独木桥阶段
            flg = gethongzhuancenter(640, 480, 'head_hongzhuan')
            if  flg > 140:
                print('z左移')
                action_append('Left3move')
                time.sleep(1)
            elif flg >110:
                print('z左移')
                action_append('Left1move')
                time.sleep(1)
            else:step = 3
        elif step == 3:  # 接近独木桥阶段
            # AGC.runAction('go_forward')
            if dirtoline_top < 180:  # 绿色区域大于30%
                print("走两步")
                action_append('Forwalk00')
                time.sleep(0.1)
                step = 4
            else:
                print("继续接近独木桥")
                action_append('Forwalk00')
                time.sleep(0.1)
        elif step == 4:  # 独木桥阶段
            print("给我翻！")
            action_append('RollRail')
            break
    return

# ###############################################5:门框关#################################################


def doorframe():
    global ChestOrg_img, state, state_sel, step
    global head_r_height, head_r_width
    step = -1
    if step == -1:
        action_append('Back2Run')
        time.sleep(0.5)
        action_append('turn004L')
        time.sleep(0.1)
        action_append('Back2Run')
        action_append('Back2Run')
        time.sleep(0.1)
        action_append('Stand')
        time.sleep(0.5)
    while True:
        if ChestOrg_img is None:
            continue
        org_img = ChestOrg_img.copy()
        dst = cv2.remap(org_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        border = cv2.copyMakeBorder(dst, 8, 8, 12, 12, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        frame_gauss = cv2.GaussianBlur(border, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['blue_door_chest_dafanwei'][0],
                                  color_range['blue_door_chest_dafanwei'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE

        contours_suit = getbigcontours_doo(contours)
        print(len(contours_suit))

        if len(contours_suit):
            areaMaxContour, x1 = getpoint1(contours_suit)  # 找出最大轮廓
            areaMaxContour2, x2 = getpoint2(contours_suit, x1)
        else:
            action_append('turn001L')
            continue

        if areaMaxContour is not None:
            btm1 = areaMaxContour[0][0]
            for c in areaMaxContour:
                if (c[0][1]) > (btm1[1]):
                    btm1 = c[0]
        if areaMaxContour2 is not None:
            btm2 = areaMaxContour2[0][0]
            for d in areaMaxContour2:
                if (d[0][1]) > (btm2[1]):
                    btm2 = d[0]
        else:
            action_append('turn001L')
            btm2 = (1, 1)
            btm1 = (0, 0)
            continue


        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(border, contours, -1, (255, 0, 255), 3)
            cv2.circle(border, (btm1[0], btm1[1]), 5, [0, 255, 255], 2)
            cv2.circle(border, (btm2[0], btm2[1]), 5, [0, 255, 255], 2)
            cv2.imshow('org', border)
            cv2.waitKey(1)

        if btm1[0] < btm2[0]:
            x_l = btm1[0]
            y_l = btm1[1]
            x_r = btm2[0]
            y_r = btm2[1]
        else:
            x_l = btm2[0]
            y_l = btm2[1]
            x_r = btm1[0]
            y_r = btm1[1]
        angle = - math.atan((y_l - y_r) / (x_l - x_r)) * 180.0 / math.pi
        bottom_center_x = int((x_l + x_r) / 2)
        bottom_center_y = int((y_l + y_r) / 2)
        dirtoline = head_r_height - bottom_center_y
        if debug:
            cv2.putText(border, "angle:" + str(int(angle)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(border, "center_x:" + str(int(bottom_center_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(border, "dirtoline_bottom:" + str(int(dirtoline)), (430, 400), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.imshow('org_img_copy', border)  # 显示图像
            cv2.waitKey(1)
        print(dirtoline, bottom_center_x, angle)
        if step == -2:
            while True:
                center1, angle1, dir1 = getrohongzhuancenter(640, 480, 'chest_red_hongzhuan')
                if angle1 > 2:
                    print("左转")
                    action_append('turn001L')
                    time.sleep(0.1)
                elif angle1 < -2:
                    print("右转")
                    action_append('turn000R')
                    time.sleep(0.1)
                else:
                    break
            while True:
                center1, angle1, dir1 = getrohongzhuancenter(640, 480, 'chest_red_hongzhuan')
                if center1>290:
                    break
                else:
                    print("左移")
                    action_append('Left3move')
                    time.sleep(1)
            while True:
                center1, angle1, dir1 = getrohongzhuancenter(640, 480, 'chest_red_hongzhuan')
                if dir1 < 100:
                    break
                elif dir1 < 170:
                    print("靠近红砖")
                    action_append('fastForward00')
                    time.sleep(1)
                else:
                    print("靠近红砖")
                    action_append('fastForward01')
                    time.sleep(1)
            step = 0
        elif step == -1:  # 接近独木桥阶段
            if angle < -3:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 3 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else:
                step = 2
                print("角度正确")

        elif step == 2:
            if dirtoline < 150:  # 绿色区域大于30%
                print("已抵达门槛，再次确定角度与位置")
                step = 3
            else:
                print("继续接近门槛")
                action_append('Forwalk02')
                time.sleep(1)
        elif step == 3:  # 独木桥阶段
            if angle < -2.5:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 2.5 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else:
                step = 4
        elif step == 4:
            if dirtoline < 100:  # 绿色区域大于30%
                print("已抵达门槛，再次确定角度与位置")
                step = 5
            else:
                print("继续接近门槛")
                action_append('Forwalk00')
                time.sleep(1)
        elif step == 5:  # 独木桥阶段
            if angle < -2.5:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 2.5 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            elif -2.5 <= angle <= 2.5:
                if bottom_center_x > 325:  # 右移
                    print("右移")
                    action_append('Right1move')
                    time.sleep(0.1)
                elif bottom_center_x < 315:  # 左移
                    print("左移")
                    action_append('Left1move')
                    time.sleep(0.1)
                    # time.sleep(0.1)
                elif 315 <= bottom_center_x <= 330:  # 走三步
                    print("角度调整完毕，继续接近独木桥")
                    step = 6
                    time.sleep(0.5)
        elif step == 6:  # 独木桥阶段
            print("前进就好")
            action_append('Forwalk02part1')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part3')
            time.sleep(1)
            action_append('fastForward02')
            break
    return

# ###############################################6:独木桥关#################################################


def breakbridge():
    global state, state_sel, step, reset, skip, debug, HeadOrg_img
    print('进入bridge')
    step = 8
    r_w = head_r_width
    r_h = head_r_height
    time.sleep(0.5)
    dirtoline_top=[]
    center_x = []
    angle = []
    while True:  # 初始化
        if HeadOrg_img is None:
            continue
        time.sleep(1.5)
        t1 = cv2.getTickCount()
        dst = cv2.remap(HeadOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        border = cv2.copyMakeBorder(dst, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['head_green_bridge'][0],
                                  color_range['head_green_bridge'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)
        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)
            cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标
            #print(1)

            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle += 1.5
            top_center_x = int((top_right[0] + top_left[0]) / 2)
            top_center_y = int((top_right[1] + top_left[1]) / 2)
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            center_x = int((top_center_x + bottom_center_x) / 2)
            dirtoline_bottom = head_r_height - bottom_center_y
            dirtoline_top = head_r_height - top_center_y
            if debug:
                cv2.drawContours(org_img_copy, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_center_x, top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_center_x, bottom_center_y), 5, [0, 255, 255], 2)
                cv2.line(org_img_copy, (top_center_x, top_center_y), (bottom_center_x, bottom_center_y), [0, 255, 255],
                         2)  # 画出上下中点连线
        else:
            angle = 0
            center_x = 0.5 * r_w
        if debug:
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_top:" + str(int(dirtoline_top)), (430, 400), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_bottom:" + str(int(dirtoline_bottom)), (430, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            # cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
            cv2.waitKey(1)
        print(dirtoline_top, center_x, angle,areaMaxContour is None)
        print(step)
        # dirtoline_top = 145
        if step == 8:
            if angle < -3:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 3 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else: step = 0
        elif step == 0:  # 接近独木桥阶段
            # AGC.runAction('go_forward')
            if dirtoline_top < 125:  # 绿色区域大于30%
                print("已接近独木桥，开始调整角度")
                step = 1
            else:
                print("继续接近独木桥")
                action_append('Forwalk02')
                print(1111)
                time.sleep(0.1)
        elif step == 1:  # 独木桥阶段
            if angle < -2:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 2 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            elif -2 <= angle <= 2:
                if center_x > 320:  # 右移
                    print("右移")
                    action_append('Right1move')
                    time.sleep(0.1)
                elif center_x < 310:  # 左移
                    print("左移")
                    action_append('Left1move')
                    time.sleep(0.1)
                    # time.sleep(0.1)
                elif 310 <= center_x <= 320:  # 走三步
                    print("角度调整完毕，继续接近独木桥")
                    step = 4
                    time.sleep(1)
        elif step == 4:  # 独木桥阶段
            print("前进就好")
            action_append('Forwalk02part1')
            action_append('Forwalk02part2')
            time.sleep(0.1)
            action_append('Forwalk02part2')
            time.sleep(0.1)
            action_append('Forwalk02part2')
            time.sleep(0.1)
            action_append('Forwalk02part3')
            break
    return


def breakbridge2():
    global state, state_sel, step, reset, skip, debug, ChestOrg_img
    print('进入bridge')
    step = 0
    r_w = head_r_width
    r_h = head_r_height
    time.sleep(0.5)
    state_sel = 'bridge'
    while True:  # 初始化
        if ChestOrg_img is None:
            continue
        time.sleep(1.5)
        t1 = cv2.getTickCount()
        dst = cv2.remap(ChestOrg_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        border = cv2.copyMakeBorder(dst, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['chest_green_bridge'][0],
                                  color_range['chest_green_bridge'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)
        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)
            cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标
            print(1)

            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            top_center_x = int((top_right[0] + top_left[0]) / 2)
            top_center_y = int((top_right[1] + top_left[1]) / 2)
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            center_x = int((top_center_x + bottom_center_x) / 2)
            dirtoline_bottom = head_r_height - bottom_center_y
            dirtoline_top = head_r_height - top_center_y
            if debug:
                cv2.drawContours(org_img_copy, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_center_x, top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_center_x, bottom_center_y), 5, [0, 255, 255], 2)
                cv2.line(org_img_copy, (top_center_x, top_center_y), (bottom_center_x, bottom_center_y), [0, 255, 255],
                         2)  # 画出上下中点连线
        else:
            angle = 0
            center_x = 0.5 * r_w
        if debug:
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_top:" + str(int(dirtoline_top)), (430, 400), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_bottom:" + str(int(dirtoline_bottom)), (430, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            # cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
            cv2.waitKey(1)
        print(dirtoline_top, center_x, angle)
        # dirtoline_top = 145
        if step == 0:
            if angle < -1.5:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 1.5 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else:
                step = 1
        elif step == 1:  # 接近独木桥阶段
            if center_x >= 335:  # 右移
                print("右移")
                action_append('Right1move')
                time.sleep(0.5)
                action_append('Stand')
                time.sleep(0.5)
            elif center_x <= 305:  # 左移
                print("左移")
                action_append('Left1move')
                time.sleep(0.5)
                action_append('Stand')
                time.sleep(0.5)
                # time.sleep(0.1)
            elif 305 < center_x < 335:  # 走三步
                print("角度调整完毕，继续接近独木桥")
                step = 2
                time.sleep(1)
        elif step == 2:  # 独木桥阶段
            print("前进就好")
            action_append('Forwalk02part1')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part3')
            break
    return
# ###############################################7:踢球关#################################################


def closetostair():
    global HeadOrg_img, state, state_sel, step, colo
    global head_r_height, head_r_width
    angle = []
    step = 9
    center_x = []
    center_y = []
    dirtoline = []
    colo = 'head_blue_stair'
    while True:
        if HeadOrg_img is None:
            continue
        print(colo)
        org_img = HeadOrg_img.copy()
        dst = cv2.remap(org_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        border = cv2.copyMakeBorder(dst, 5, 5, 5, 5, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        # org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        org_img_copy = border  # 将图片缩放

        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range[colo][0],
                                  color_range[colo][1])  # 对原图像和掩模(颜色的字典)进行位运算
        # frame_green = cv2.inRange(frame_hsv, (p1,p2,p3),(p4,p5,p6))  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        # contour = contours[1]
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        if areaMaxContour is not None:
            top_left, top_right, bottom_left, bottom_right = getAreaMaxContour_hole(areaMaxContour, org_img_copy)
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            angle += 2.0
            center_x = (top_right[0] + top_left[0]) / 2
            center_y = (top_right[1] + top_left[1]) / 2
            dirtoline = head_r_height - center_y
        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, areaMaxContour, -1, (255, 0, 255), 3)
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_y:" + str(int(center_y)), (330, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline:" + str(int(dirtoline)), (330, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.imshow('org_img_copy1', org_img_copy)
            cv2.waitKey(1)
        print(angle, center_x, dirtoline)
        if step == 9:
            action_append('Forwalk02part1')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part2')
            time.sleep(1)
            action_append('Forwalk02part3')
            time.sleep(1)
            action_append('fastForward02')
            time.sleep(1)
            action_append('turn010L')
            time.sleep(1)
            action_append('turn010L')
            time.sleep(1)
            step = 8
        elif step == 8:
            if angle < -3:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 3 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else:
                action_append('fastForward02')
                step = 0
        elif step == 0:  # 接近独木桥阶段
            if angle < -3:  # 右转
                print("右转")
                action_append('turn000R')
                time.sleep(0.1)
            elif 3 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
            else:
                action_append('fastForward02')
                step = 1
                colo = 'head_green_stair'
        elif step == 1:  # 独木桥阶段
            if  75 < dirtoline < 85:
                action_append('Forwalk00')
                time.sleep(0.1)
            elif dirtoline >= 85:
                action_append('Forwalk02')
                time.sleep(0.1)
            else:
                action_append('Forwalk00')
                time.sleep(0.1)
                action_append('Forwalk00')
                time.sleep(0.1)
                action_append('Forwalk00')
                time.sleep(0.1)
                action_append('Forwalk00')
                time.sleep(0.1)
                step = 2
        elif step == 2:
            if center_x > 335:
                print('右移')
                action_append('Right3move')
                time.sleep(0.5)
            elif center_x <= 320:  # 左转
                print('左移')
                action_append('Left3move')
                time.sleep(0.5)
            else:
                action_append('Forwalk00')
                break
    return
# ###############################################8:上楼梯关#################################################

def upstairgo():
    print('上楼梯开始')
    time.sleep(0.5)
    while True:  # 初始化
        print("上第一级")
        action_append('upstairfinal')
        time.sleep(1)
        action_append('Forwalk00')
        action_append('Forwalk00')
        action_append('Forwalk00')
        time.sleep(0.5)
        print("上第二级")
        action_append('upstairfinal')
        time.sleep(1)
        action_append('Forwalk00')
        action_append('Forwalk00')
        action_append('Forwalk00')
        time.sleep(0.5)
        print("上第三级")
        action_append('upstairfinal')

        print("结束")
        break
    return

# ###############################################9:下楼梯关#################################################


def downstair():
    global state, state_sel, org_img, step, reset, skip, debug,ChestOrg_img
    print('进入bridge')
    step = 0
    cnt = 0
    r_w = head_r_width
    r_h = head_r_height
    time.sleep(0.5)
    while True:  # 初始化
        if ChestOrg_img is None:
            continue
        t1 = cv2.getTickCount()
        org_img = ChestOrg_img.copy()
        dst = cv2.remap(org_img.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        border = cv2.copyMakeBorder(dst, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, color_range['chest_red'][0],
                                  color_range['chest_red'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))  # 闭运算 封闭连接
        # print(closed)
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)
        if debug:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)
            cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标

            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 2.5 * c[0][1] < (r_w - top_right[0]) + 2.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            angle = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            top_center_x = int((top_right[0] + top_left[0]) / 2)
            top_center_y = int((top_right[1] + top_left[1]) / 2)
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            center_x = int((top_center_x + bottom_center_x) / 2)
            dirtoline_bottom = head_r_height - bottom_center_y
            dirtoline_top = head_r_height - top_center_y
            if debug:
                cv2.drawContours(org_img_copy, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_center_x, top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_center_x, bottom_center_y), 5, [0, 255, 255], 2)
                cv2.line(org_img_copy, (top_center_x, top_center_y), (bottom_center_x, bottom_center_y), [0, 255, 255],
                         2)  # 画出上下中点连线
        else:
            angle = 0
            center_x = 0.5 * r_w
        if debug:
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_top:" + str(int(dirtoline_top)), (430, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "dirtoline_bottom:" + str(int(dirtoline_bottom)), (430, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            # cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
            cv2.waitKey(1)
        print(angle)
        if step == 0:  # 接近独木桥阶段
            if angle < -3:  # 右转
                print("右转")
                action_append('turn001R')
                time.sleep(0.1)
                cnt += 1
            elif 3 < angle:  # 左转
                print("左转")
                action_append('turn001L')
                time.sleep(0.1)
                cnt += 1
            else: step = 1
        elif step == 1:  # 独木桥阶段
            if cnt > 0:  # 右转
                step = 2
                action_append('Forwalk00')
            else:
                print("前进一步")
                action_append('turn001L')
                action_append('Forwalk00')
                time.sleep(0.1)
                step = 2
        elif step == 2:
            print("下楼梯")
            action_append('DownBridge')
            time.sleep(0.4)
            print("前进一步")
            action_append('turn001L')
            time.sleep(0.4)
            action_append('Forwalk00')
            time.sleep(0.4)
            print("下楼梯")
            action_append('DownBridge')
            time.sleep(0.4)
            action_append('turn001L')
            time.sleep(0.4)
            action_append('Forwalk00')
            time.sleep(0.4)
            action_append('Forwalk00')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            action_append('Forwalk02')
            time.sleep(0.4)
            break
    return

# ###############################################10:结束门关#################################################


if __name__ == '__main__':
    iniparm()
    start_door()
    iniparm()
    breakbridge()
    iniparm()
    breakbridge2()

    iniparm()
    gothroughdilei()
    iniparm()
    overwall()
    iniparm()
    doorframe()
    iniparm()

    gothroughhole()
    iniparm()
    closetostair()
    iniparm()
    upstairgo()
    iniparm()
    downstair()
