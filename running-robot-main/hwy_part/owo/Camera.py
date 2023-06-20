# Usage:
# 提供calibration_param.npz的绝对路径可开启畸变矫正功能
# my_camera.frame获取实时图像

#!/usr/bin/env python3
# encoding:utf-8
import sys
import cv2
import time
import threading
import numpy as np

#这是一个用Opencv获取usb摄像头的画面的封装库

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

class Camera:
    def __init__(self, param_data_path='', camera_name=-1, resolution=(640, 480)):
        self.cap = None
        self.camera_name = camera_name
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False

        self.enable_calibrate = True if param_data_path != '' else False

        # 读取畸变参数
        if self.enable_calibrate:
            param_data = np.load(param_data_path)
            dim = tuple(param_data['dim_array'])
            k = np.array(param_data['k_array'].tolist())
            d = np.array(param_data['d_array'].tolist())
            p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim ,None)
            self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), p, dim, cv2.CV_16SC2)
        
        # 以子线程的形式获取图像
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self): # 开启
        try:
            self.cap = cv2.VideoCapture(self.camera_name)
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
                        frame_tmp = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST) # 缩放
                        if self.enable_calibrate:
                            self.frame = cv2.remap(frame_tmp, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                        else:
                            self.frame = frame_tmp
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



if __name__ == '__main__':
    # 使用例程
    import os
    param_data_path = os.path.join(os.environ['HOME'], 'Desktop/running-robot/include/calibration_param.npz')
    my_camera = Camera(param_data_path=param_data_path)
    my_camera.camera_open()
    print('摄像头原始画面，已做畸变校正')
    while True:
        img = my_camera.frame
        if img is not None:
            cv2.imshow('img', img)
            key = cv2.waitKey(1)
            if key == 27: # ESC
                break  
    my_camera.camera_close()
    cv2.destroyAllWindows()
