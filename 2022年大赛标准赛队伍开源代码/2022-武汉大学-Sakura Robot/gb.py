import CMDcontrol
import numpy as np
import cv2
import math
import time
import threading
import rospy
from image_converter import ImgConverter



img_debug = True
head_width = 640
head_height = 480
chest_width = 640
chest_height = 480
################################################颜色#################################################
color_range = {
                #道闸
                'yellow_door': [( 101 , 95 , 161 ) ,  (250 , 120 , 255)],
                #过坑
                'green_board':[( 37 , 65 , 107 ) ,  (215 , 107 , 173)],
                'hole_hole':[( 0 , 88 , 95 ) ,  (120 , 144 , 137)],
                # 'blue_board':[],
                #
                'green_bridge': [( 25 , 62 , 82 ) ,  (214 , 105 , 173)],
                'blue_bridge': [( 17 , 47 , 42 ) ,  (224 , 145 , 109)],
                'blue_stair':[( 16 , 19 , 0 ), ( 255 , 250 , 108 )],
                'white_ball':[( 97 , 83 , 89 ), (255, 135, 135)],
                'blue_hole':[( 27 , 41 , 37 ) ,  (179 , 181 , 118)],

                'blue_baf':[( 52 , 40 , 40 ) ,  (116 , 178 , 103)],
                'black_dir':[( 2 , 78 , 93 ) ,  (78 , 133 , 142)],
                'head_blue_door':[( 38 , 87 , 62 ) ,  (170 , 141 , 117)]
               }

color_dist = {'red': {'Lower': np.array([0, 160, 100]), 'Upper': np.array([180, 255, 250])},
               'black_dir': {'Lower': np.array([0, 0, 10]), 'Upper': np.array([170, 170, 45])},
               'black_line': {'Lower': np.array([0, 0, 20]), 'Upper': np.array([100, 160, 80])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'ball_red': {'Lower': np.array([160, 100, 70]), 'Upper': np.array([190, 215, 145])},
              'blue_hole': {'Lower': np.array([100, 130, 80]), 'Upper': np.array([130, 255, 150])},
              }
################################################动作指令#################################################
def action_append(act_name):
    print(f'执行动作: {act_name}')
    # time.sleep(1)
    CMDcontrol.action_append(act_name)

################################################部分函数#################################################
# 得到最大轮廓和对应的最大面积
def getAreaMaxContour1(contours):    # 返回轮廓 和 轮廓面积
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

# 得到大于某面积的最大轮廓
def getAreaMaxContour2(contours, area=1):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  # 面积大于1
                area_max_contour = c
    return area_max_contour

# 将所有面积大于1的轮廓点拼接到一起
def getSumContour(contours, area=1):
    contours_sum = None
    # print(len(contours))
    for c in contours:  #   初始化contours
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > area:
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > area:
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有面积大于1的轮廓点拼接到一起
    return contours_sum

######### 得到所有轮廓的面积##########
def getAreaSumContour(contours):
    contour_area_sum = 0
    for c in contours:  # 历遍所有轮廓
        contour_area_sum += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
    return contour_area_sum  # 返回总面积