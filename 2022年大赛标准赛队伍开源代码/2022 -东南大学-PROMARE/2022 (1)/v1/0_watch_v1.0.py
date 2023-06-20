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


"""-------------全局变量-----------------"""
ret = False
org_img = None  # 原始图像
cali_img = None  # 校准图像


# 摄像头读取设置
cap = cv2.VideoCapture(-1)
print(cap.get(5))

debug = True
state = 0  # 0.5待定
step = 0
varies_flag = None

"""-------------初始化------------------"""

# 动作
def initAction(name="Stand"):
    AGC.runAction(name, 1, action_path)  # 站立
    time.sleep(0.5)
    pass


# 相机云台
def initCameraPlatform(left=500, top=500):
    # setBusServoPulse(id, pulse, use_time) param: id, 位置, 运行时间
    Board.setBusServoPulse(19, left, 500)  # id=19 左右
    time.sleep(0.1)  # 延时时间和运行时间相同
    Board.setBusServoPulse(20, top, 500)  # id=20 上下
    time.sleep(0.1)


"""---------------线程1——读取摄像头--------------"""
# 读取图像org_img，并校准cali_img
def get_img():
    global ret, org_img, cap, cali_img
    while True:
        if cap.isOpened():
            ret, org_img = cap.read()
            if ret:
                frame = org_img.copy()
                cali_img = cv2.remap(
                    frame,
                    map1,
                    map2,
                    interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )
        else:
            time.sleep(0.01)


th1 = threading.Thread(target=get_img)
th1.setDaemon(True)
th1.start()

initAction()
initCameraPlatform(500, 300)
i = 5
while True:
    if cali_img is not None and ret:
        cv2.imshow("cali_img", cali_img)
        if cv2.waitKey(10) & 0xFF == ord("s"):
            cv2.imwrite(f"./pics/pic_{i}.jpg", cali_img) 
            i+=1
    else:
        print("img is not ready")
