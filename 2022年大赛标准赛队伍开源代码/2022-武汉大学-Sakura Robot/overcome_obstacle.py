#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy
import datetime

from image_converter import ImgConverter

img_debug = False
box_debug = False


chest_ret = True    
ChestOrg_img = None  

color_range = {
    'blue_baf':[(99 , 100 , 110), (110 , 224 , 255)],
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





def getAreaMaxContour1(contours):    
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  
        contour_area_temp = math.fabs(cv2.contourArea(c))  
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  
                area_max_contour = c
    return area_max_contour, contour_area_max  



###################### 档            板-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
def baffle(action_append):
    global step
    global handling
    print("/-/-/-/-/-/-/-/-/-进入baffle")
    step = 0
    baffle_dis_Y_flag = False
    baffle_angle = 0
    center_x = 0
    while(1):
        if True:
            OrgFrame = ChestOrg_img.copy()
            handling = ChestOrg_img.copy()
            frame = ChestOrg_img.copy()
            center = []

    # 开始处理图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
            Imask = cv2.inRange(hsv, color_range['blue_baf'][0], color_range['blue_baf'][1])
            Imask = cv2.erode(Imask, None, iterations=2)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imshow('BLcolor', Imask)
            cnts, hieracy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            # print("cnts len:",len(cnts))
            if cnts is not None:
                cnt_large , cnt_area = getAreaMaxContour1(cnts)

        
            else:
                print("cnt_large is None")
                continue

            blue_bottom_Y = 0
            if cnt_large is not None:
                rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                
                Ax = box[0, 0]
                Ay = box[0, 1]
                Bx = box[1, 0]
                By = box[1, 1]
                Cx = box[2, 0]
                Cy = box[2, 1]
                Dx = box[3, 0]
                Dy = box[3, 1]
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x = int((pt1_x + pt3_x) / 2)
                center_y = int((pt1_y + pt3_y) / 2)
                center.append([center_x, center_y])
                cv2.drawContours(OrgFrame, [box], -1, [0, 0, 255, 255], 3)
                cv2.circle(OrgFrame, (center_x, center_y), 10, (0, 0, 255), -1)  # 画出中心点
                # 求得大矩形的旋转角度，if条件是为了判断长的一条边的旋转角度，因为box存储的点的顺序不确定\
                if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
                    baffle_angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0])) * 180.0 / math.pi
                else:
                    baffle_angle = - math.atan( (box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0]) ) * 180.0 / math.pi  # 负号是因为坐标原点的问题
                if center_y > blue_bottom_Y:
                    blue_bottom_Y = center_y
            baffle_dis_Y = blue_bottom_Y
            baffle_dis_X = center_x 
            # 数值显示
            # print("baffle_dis_Y = ",baffle_dis_Y)
            # print("baffle_angle = ",baffle_angle)
            if baffle_dis_Y > 160:
                baffle_dis_Y_flag = True


            if img_debug:
                cv2.putText(OrgFrame, "baffle_dis_Y:" + str(baffle_dis_Y),
                            (10, OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            
                cv2.putText(OrgFrame, "baffle_dis_Y_flag:" + str(baffle_dis_Y_flag),
                            (10, OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            
                cv2.putText(OrgFrame, "baffle_angle:" + str(baffle_angle),
                            (10, OrgFrame.shape[0] - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "step:" + str(step), (30, OrgFrame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                            
                cv2.imwrite('./OrgFrame.jpg', OrgFrame)

            
    # 决策执行动作
            if step == 0:
                if baffle_dis_Y <= 170:
                    print("大步前进 Forwalk02")
                    action_append("Forwalk02")
                elif baffle_dis_Y > 170:
                    step=1


            elif step==1:   # 调整角度 -5 ~ 5
                if baffle_angle > 5:
                    if baffle_angle > 8:
                        print("大左转一下  turn001L  baffle_angle:",baffle_angle)
                        action_append("turn001L")
                    else:
                        print("左转 turn001L  baffle_angle:",baffle_angle)
                        action_append("turn001L")
                elif baffle_angle < -5:
                    if baffle_angle < -8:
                        print("大右转一下  turn001R  baffle_angle:",baffle_angle)
                        action_append("turn001R")
                    else:
                        print("右转 turn001R  baffle_angle:",baffle_angle)
                        action_append("turn001R")
                else:
                    step=2
                
            elif step == 2:     # 调整前进位置  调整左右位置
                if baffle_dis_Y < 310:
                    print("大一步前进 forwardSlow0403")
                    action_append("forwardSlow0403")
                elif 310 < baffle_dis_Y < 380:
                    print("向前挪动 Forwalk00")
                    action_append("Forwalk00")
                elif 380 < baffle_dis_Y:
                    step = 3
            elif step == 3: # 调整角度
                if baffle_angle > 2:
                    if baffle_angle > 5:
                        print("大左转一下  turn001L ",baffle_angle)
                        action_append("turn001L")
                    else:
                        print("左转 turn001L")
                        action_append("turn001L")
                elif baffle_angle < -2:
                    if baffle_angle < -5:
                        print("大右转一下  turn001R ",baffle_angle)
                        action_append("turn001R")
                    else:
                        print("右转 turn001R ",baffle_angle)
                        action_append("turn001R")
                elif baffle_dis_Y_flag:
                    step = 4
            elif step == 4: # 跨栏后调整方向

                print("前挪一点点")
                print("翻栏杆 翻栏杆 RollRail")
                action_append("Right3move")
                action_append("Stand")
                action_append("RollRail")
                action_append("Stand")
                
                
                action_append("turn004L")
                action_append("turn004L")
                action_append("turn004L")
                action_append("turn004L")
                action_append("turn001L")
                action_append("Back2Run")
                
                break
    return True


if __name__ == '__main__':
    rospy.init_node('baffle')
    time.sleep(3)
    baffle()