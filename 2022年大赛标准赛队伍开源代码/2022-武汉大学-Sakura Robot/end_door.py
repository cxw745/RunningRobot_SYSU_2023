#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy

from image_converter import ImgConverter

chest_r_width = 480
chest_r_height = 640
img_debug = False

action_DEBUG = False

chest_ret = True    
ChestOrg_img = None  

color_range = {
               'yellow_door': [(26 , 234 , 248), (30 , 255 , 255)],
               'black_door': [(0 , 44 , 5), (170 , 255 , 72)],
               }

################################################读取图像线程#################################################

def get_img():
    
    global ChestOrg_img, chest_ret
    image_reader_chest = ImgConverter()
    while True:
        chest_ret, ChestOrg_img = image_reader_chest.chest_image()
        #print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
        #print(ChestOrg_img)
        time.sleep(0.05)

# 读取图像线程
th1 = threading.Thread(target=get_img)
th1.setDaemon(True)
th1.start()



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


###################### 终            点-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
def end_door(action_append):
    global ChestOrg_img, state, state_sel, step, reset, skip, img_debug,end_door_flag
    end_door_flag = 0
    state_sel = 'door'
    state = 1
    if state == 1:  # 初始化
        print("/-/-/-/-/-/-/-/-/-进入end_door")
        step = 0
    else:
        return

    while state == 1 :

        if step == 0: #判断门是否抬起
            t1 = cv2.getTickCount() # 时间计算
            

            handling = ChestOrg_img.copy()
            

            border = cv2.copyMakeBorder(handling, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,value=(255, 255, 255))     # 扩展白边，防止边界无法识别
            handling = cv2.resize(border, (chest_r_width, chest_r_height), interpolation=cv2.INTER_CUBIC)                   # 将图片缩放
            frame_gauss = cv2.GaussianBlur(handling, (21, 21), 0)       # 高斯模糊
            frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)    # 将图片转换到HSV空间
            
            frame_door_yellow = cv2.inRange(frame_hsv, color_range['yellow_door'][0], color_range['yellow_door'][1])    # 对原图像和掩模(颜色的字典)进行位运算
            frame_door_black = cv2.inRange(frame_hsv, color_range['black_door'][0], color_range['black_door'][1])       # 对原图像和掩模(颜色的字典)进行位运算


            frame_door = cv2.add(frame_door_yellow, frame_door_black)    
            open_pic = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((13, 13), np.uint8))      # 开运算 去噪点
            closed_pic = cv2.morphologyEx(open_pic, cv2.MORPH_CLOSE, np.ones((50, 50), np.uint8))   # 闭运算 封闭连接
            open_pic_2 = cv2.morphologyEx(closed_pic, cv2.MORPH_OPEN, np.ones((25, 25), np.uint8))      # 开运算 去噪点
            # print(closed_pic)

            (contours, hierarchy) = cv2.findContours(open_pic_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour1(contours)  # 找出最大轮廓
            percent = round(100 * area_max / (chest_r_width * chest_r_height), 2)  # 最大轮廓的百分比
            if areaMaxContour is not None:
                rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
                box = np.int0(cv2.boxPoints(rect))      # 点的坐标
                if img_debug:
                    cv2.drawContours(handling, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上

            if img_debug:
                cv2.putText(handling, 'area: ' + str(percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                t2 = cv2.getTickCount()
                time_r = (t2 - t1) / cv2.getTickFrequency()
                fps = 1.0 / time_r
                cv2.putText(handling, "fps:" + str(int(fps)), (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.imwrite("./handling.jpg",handling)

                cv2.imwrite("./frame_door_yellow.jpg",frame_door_yellow)
                cv2.imwrite("./frame_door_black.jpg",frame_door_black)
                # cv2.imwrite("./open_after_closed.jpg",haopen_pic_2ndling)



            # 根据比例得到是否前进的信息
            if percent > 5:    #检测到横杆
                print(percent,"%")
                print("有障碍 等待 contours len：",len(contours))
                action_append("Stand")
                end_door_flag = 1
                time.sleep(3)
            else:
                if end_door_flag == 0:
                    print(percent)
                    print("3885L 执行1步")
                    action_append("forwardSlow0403")
                    action_append("turn001R")

                elif end_door_flag == 1 and percent == 0:
                    print(percent)

                    print("3899L 执行快走555")
                    action_append("fastForward05")
                
                    step = 1

                else:
                    print(percent,"%")
                    print("有障碍 等待 contours len：",len(contours))
                    action_append("Stand")
                    time.sleep(3)
                
        elif step == 1:  
            break
    return True

if __name__ == '__main__':
    rospy.init_node('end_door_nodes')
    time.sleep(3)
    end_door()