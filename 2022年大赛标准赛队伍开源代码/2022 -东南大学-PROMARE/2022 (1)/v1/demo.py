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
import hiwonder.Camera as Camera  # 打开摄像头线程


"""-------------全局变量-----------------"""

############图像读取相关参数##########

ret = False
org_img = None  # 原始图像
cali_img = None  # 校准图像


###########读取设置##############
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 15)  # 帧率设置
print(cap.get(5))
#################################################3


debug = True
state = 0  # 0.5待定
step = 0
varies_flag = None


"""-------------初始化------------------"""
###########运动###############
# def runAction(actNum, times=1, path="/home/pi/AiNexPro/ActionGroups/"):
#     '''
#     运行动作组
#     :param actNum: 动作组名字 ， 字符串类型
#     :param times: 运行次数，当为0时表示循环
#     :return:
#     '''
# 站立
AGC.runAction("stand", 1, action_path)
time.sleep(2)

###########摄像头云台#################
# setBusServoPulse(id, pulse, use_time)
# id：舵机id;
# pulse：位置;
# use_time：运行时间
# 19号电机：云台左右
# 20电机： 云台上下

# Board.setBusServoPulse(19, 500, 500)  # 19号舵机转到500位置，用时500ms
# time.sleep(0.5)  # 延时时间和运行时间相同
Board.setBusServoPulse(19, 500, 500)  # 19号舵机转到500位置，用时500ms
time.sleep(0.5)  # 延时时间和运行时间相同
Board.setBusServoPulse(20, 350, 500)
time.sleep(0.5)
i = 0
while True:
    if cap.isOpened():
        ret, org_img = cap.read()
        if ret:
            cv2.imshow("org_img", org_img)
            if cv2.waitkey(1) == ord("s"):
                cv2.save("pic_" + i + ".png", org_img)
    else:
        time.sleep(0.01)
