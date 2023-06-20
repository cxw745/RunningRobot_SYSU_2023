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
    'blue_baf':[(40 , 57 , 102), (82 , 146 , 188)],
    'black_dir':[(52 , 113 , 149), (110 , 255 , 255)],
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





def getAreaMaxContour2(contours, area=1):
    contour_area_max = 0
    area_max_contour = None
    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area:  
                area_max_contour = c
    return area_max_contour



###################### 过   地   雷   区-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
Head_L_R_angle = 0
see_flag = False
Bbox_centerY = 0
#####################################已知固定挡板位置情况的过地雷区
def obstacle(action_append):
    #根据边线调整角度

    global Head_L_R_angle,Bbox_centerY,blue_rail
    color = 'black_dir'
    
    print("/-/-/-/-/-/-/-/-/-进入obscle")
    
    left = False  # 左移信号
    left2 = False  # 遇到右边缘且前方有障碍
    right = False  # 右移
    right2 = False  # 遇到左边缘且前方有障碍

    # 初始化 delta
    delta = datetime.datetime.now()
    delta = delta - delta
    blue_rail = False

    while(1):
        if True:    
            Corg_img = ChestOrg_img.copy()
            hsv = cv2.cvtColor(Corg_img, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)


         # blue 分析图像 决策执行
            Bumask = cv2.inRange(hsv, color_range['blue_baf'][0], color_range['blue_baf'][1])
            Bumask = cv2.erode(Bumask, None, iterations=2)
            Bumask = cv2.dilate(Bumask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imshow('Bluemask', Bumask)
            cntsblue, hierarchy = cv2.findContours(Bumask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
            # cv2.imwrite('./Bumask.jpg', Bumask)
            if cntsblue is not None:
                cnt_large = getAreaMaxContour2(cntsblue)    # 取最大轮廓
            else:
                print("1135L cnt_large is None")
                continue

            if cnt_large is not None:
                rect_blue = cv2.minAreaRect(cnt_large)
                box_blue = np.int0(cv2.boxPoints(rect_blue))  # 点的坐标
                Bbox_centerX = int((box_blue[3,0] + box_blue[2,0] + box_blue[1,0] + box_blue[0,0])/4)
                Bbox_centerY = int((box_blue[3,1] + box_blue[2,1] + box_blue[1,1] + box_blue[0,1])/4)
                Bbox_center = [Bbox_centerX,Bbox_centerY]
                cv2.circle(Corg_img, (Bbox_center[0],Bbox_center[1]), 7, (0, 0, 255), -1) # 圆点标记

                cv2.drawContours(Corg_img, [box_blue], -1, (255,0,0), 3)
                obscle_area_blue = 0
                # 当遇到蓝色门槛时停止
                for c in cntsblue:
                    obscle_area_blue += math.fabs(cv2.contourArea(c))
                if  Bbox_centerY >= 260 and obscle_area_blue > 0.05 * 640 * 480 :   # and go_up: # 320  obscle_area_blue > 0.05 * 640 * 480 and

                    if img_debug:
                        cv2.imwrite('./baf.jpg', Corg_img)   #障碍识别情况
                    print("遇到蓝色门槛-----*-----*-----*-----* Bbox_center Y:",Bbox_centerY)
                    action_append("Stand")
                    blue_rail = True
                    

                    cv2.destroyAllWindows()
                    break

         # black 分析图像 决策执行
            Imask = cv2.inRange(hsv, color_range[color][0], color_range[color][1])
            Imask = cv2.erode(Imask, None, iterations=3)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imwrite('./Imask.jpg', Imask)   #二值化后图片显示
            contours, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            # print("contours lens:",len(contours))
            cv2.drawContours(Corg_img, contours, -1, (255, 0, 255), 2)
            # cv2.imwrite('./Corg_img_Imask.jpg', Corg_img)


            left_point = [640,0]
            right_point = [0,0]

            
            if len(contours) != 0:

                Big_battle = [0,0]

                for c in contours:
                    rect = cv2.minAreaRect(c)  # 最小外接矩形
                    box = cv2.boxPoints(rect)   #我们需要矩形的4个顶点坐标box, 通过函数 cv2.cv.BoxPoints() 获得
                    box = np.intp(box)  # 最小外接矩形的四个顶点
                    box_Ax,box_Ay = box[0,0],box[0,1]
                    box_Bx,box_By = box[1,0],box[1,1]
                    box_Cx,box_Cy = box[2,0],box[2,1]
                    box_Dx,box_Dy = box[3,0],box[3,1]
                    box_centerX = int((box_Ax + box_Bx + box_Cx + box_Dx)/4)
                    box_centerY = int((box_Ay + box_By + box_Cy + box_Dy)/4)
                    box_center = [box_centerX,box_centerY]
                    # cv2.circle(Corg_img, (box_centerX,box_centerY), 7, (0, 255, 0), -1) #距离比较点 绿圆点标记
                    # cv2.drawContours(Corg_img, [box], -1, (255,0,0), 3)

                    # 剔除图像上部分点 和底部点
                    if box_centerY < 200 or box_centerY > 550:
                        continue
                    
                    # 遍历点 画圈
                    if box_debug:
                        cv2.circle(Corg_img, (box_centerX,box_centerY), 8, (0, 0, 255), 2) # 圆点标记识别黑点
                        cv2.imwrite('./Corg_img.jpg', Corg_img)
                        
                        
                    # 找出最左点与最右点
                    if  box_centerX < left_point[0]:
                        left_point = box_center
                    if box_centerX > right_point[0]:
                        right_point = box_center

                    if box_centerX <= 80 or box_centerX >= 600 :  # 排除左右边沿点 box_centerXbox_centerX 240
                        continue
                    if math.pow(box_centerX - 240 , 2) + math.pow(box_centerY - 640 , 2) < math.pow(Big_battle[0] - 240 , 2) + math.pow(Big_battle[1] - 640 , 2):
                        Big_battle =  box_center  # 这个是要规避的黑点

                # 显示图
                if img_debug:
                    cv2.circle(Corg_img, (left_point[0],left_point[1]), 7, (0, 255, 0), -1) # 圆点标记
                    cv2.circle(Corg_img, (right_point[0],right_point[1]), 7, (0, 255, 255), -1) # 圆点标记
                    cv2.circle(Corg_img, (Big_battle[0],Big_battle[1]), 7, (255, 255, 0), -1) # 圆点标记
                    cv2.putText(Corg_img, "Head_L_R_angle:" + str(int(Head_L_R_angle)), (230, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "see_flag:" + str(int(see_flag)), (230, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR 
                    cv2.putText(Corg_img, "Bbox_centerY:" + str(int(Bbox_centerY)), (230, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "Big_battle x,y:" + str(int(Big_battle[0])) +', ' + str(int(Big_battle[1])) , (230, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.line(Corg_img, (Big_battle[0],Big_battle[1]), (240,640), (0, 255, 255), thickness=2)

                    cv2.line(Corg_img, (0,500), (480,500), (255, 255, 255), thickness=2)
                    cv2.imwrite('./Corg_img.jpg', Corg_img)

                
                if Big_battle[1] < 310:
                    print("前进靠近 forwardSlow0403 ",Big_battle[1])
                    action_append("forwardSlow0403")

                elif Big_battle[1] < 355:
                    print("慢慢前进靠近 Forwalk01",Big_battle[1])
                    action_append("Stand")
                    action_append("Forwalk01")

                elif (180<= Big_battle[0] and Big_battle[0] < 260):
                    print("右平移一点点 Right02move",Big_battle[0])
                    action_append("Stand")
                    action_append("Right02move")
                elif (260<= Big_battle[0] and Big_battle[0]< 350):
                    print("右平移一步 Right3move",Big_battle[0])
                    action_append("Stand")
                    action_append("Right3move")

                elif (350<= Big_battle[0] and Big_battle[0]< 420):
                    print("向左平移一步 Left3move",Big_battle[0])
                    action_append("Stand")
                    action_append("Left3move")

                elif (420<= Big_battle[0] < 490):
                    print("向左平移一点点 Left02move",Big_battle[0])
                    action_append("Stand")
                    action_append("Left02move")

                else:
                    print("不在调整范围，前进")
                    action_append("Forwalk01")
            else:
                print("未识别到轮廓，继续向前")
                action_append("forwardSlow0403")
                Big_battle = [0,0]

                if img_debug:
                    cv2.circle(Corg_img, (left_point[0],left_point[1]), 7, (0, 255, 0), -1) # 圆点标记
                    cv2.circle(Corg_img, (right_point[0],right_point[1]), 7, (0, 255, 255), -1) # 圆点标记
                    cv2.circle(Corg_img, (Big_battle[0],Big_battle[1]), 7, (255, 255, 0), -1) # 圆点标记
                    cv2.line(Corg_img, (Big_battle[0],Big_battle[1]), (240,640), (0, 255, 255), thickness=2)
                    cv2.line(Corg_img, (0,500), (480,500), (255, 255, 255), thickness=2)
                    cv2.imwrite('./img_debug.jpg', Corg_img)   #显示识别情况

    return True




###############################如果挡板的位置不确定############################################
'''            
def obstacle():
    #根据边线调整角度

    global Head_L_R_angle,Bbox_centerY,blue_rail
    color = 'black_dir'
    
    print("/-/-/-/-/-/-/-/-/-进入obscle")
    
    left = False  # 左移信号
    left2 = False  # 遇到右边缘且前方有障碍
    right = False  # 右移
    right2 = False  # 遇到左边缘且前方有障碍

    # 初始化 delta
    delta = datetime.datetime.now()
    delta = delta - delta
    blue_rail = False

    while(1):
        if True:    
            Corg_img = ChestOrg_img.copy()
            hsv = cv2.cvtColor(Corg_img, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)



         # black 分析图像 决策执行
            Imask = cv2.inRange(hsv, color_range[color][0], color_range[color][1])
            Imask = cv2.erode(Imask, None, iterations=3)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
            # cv2.imwrite('./Imask.jpg', Imask)   #二值化后图片显示
            contours, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            # print("contours lens:",len(contours))
            cv2.drawContours(Corg_img, contours, -1, (255, 0, 255), 2)
            # cv2.imwrite('./Corg_img_Imask.jpg', Corg_img)


            left_point = [640,0]
            right_point = [0,0]

            
            if len(contours) != 0:

                Big_battle = [0,0]

                for c in contours:
                    rect = cv2.minAreaRect(c)  # 最小外接矩形
                    box = cv2.boxPoints(rect)   #我们需要矩形的4个顶点坐标box, 通过函数 cv2.cv.BoxPoints() 获得
                    box = np.intp(box)  # 最小外接矩形的四个顶点
                    box_Ax,box_Ay = box[0,0],box[0,1]
                    box_Bx,box_By = box[1,0],box[1,1]
                    box_Cx,box_Cy = box[2,0],box[2,1]
                    box_Dx,box_Dy = box[3,0],box[3,1]
                    box_centerX = int((box_Ax + box_Bx + box_Cx + box_Dx)/4)
                    box_centerY = int((box_Ay + box_By + box_Cy + box_Dy)/4)
                    box_center = [box_centerX,box_centerY]
                    # cv2.circle(Corg_img, (box_centerX,box_centerY), 7, (0, 255, 0), -1) #距离比较点 绿圆点标记
                    # cv2.drawContours(Corg_img, [box], -1, (255,0,0), 3)

                    # 剔除图像上部分点 和底部点
                    if box_centerY < 200 or box_centerY > 550:
                        continue
                    
                    # 遍历点 画圈
                    if box_debug:
                        cv2.circle(Corg_img, (box_centerX,box_centerY), 8, (0, 0, 255), 2) # 圆点标记识别黑点
                        cv2.imwrite('./Corg_img.jpg', Corg_img)
                        
                        
                    # 找出最左点与最右点
                    if  box_centerX < left_point[0]:
                        left_point = box_center
                    if box_centerX > right_point[0]:
                        right_point = box_center

                    if box_centerX <= 80 or box_centerX >= 600 :  # 排除左右边沿点 box_centerXbox_centerX 240
                        continue
                    if math.pow(box_centerX - 240 , 2) + math.pow(box_centerY - 640 , 2) < math.pow(Big_battle[0] - 240 , 2) + math.pow(Big_battle[1] - 640 , 2):
                        Big_battle =  box_center  # 这个是要规避的黑点

                # 显示图
                if img_debug:
                    cv2.circle(Corg_img, (left_point[0],left_point[1]), 7, (0, 255, 0), -1) # 圆点标记
                    cv2.circle(Corg_img, (right_point[0],right_point[1]), 7, (0, 255, 255), -1) # 圆点标记
                    cv2.circle(Corg_img, (Big_battle[0],Big_battle[1]), 7, (255, 255, 0), -1) # 圆点标记
                    cv2.putText(Corg_img, "Head_L_R_angle:" + str(int(Head_L_R_angle)), (230, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "see_flag:" + str(int(see_flag)), (230, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR 
                    cv2.putText(Corg_img, "Bbox_centerY:" + str(int(Bbox_centerY)), (230, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.putText(Corg_img, "Big_battle x,y:" + str(int(Big_battle[0])) +', ' + str(int(Big_battle[1])) , (230, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR
                    cv2.line(Corg_img, (Big_battle[0],Big_battle[1]), (240,640), (0, 255, 255), thickness=2)

                    cv2.line(Corg_img, (0,500), (480,500), (255, 255, 255), thickness=2)
                    cv2.imwrite('./Corg_img.jpg', Corg_img)

                
                if Big_battle[1] < 310:
                    print("前进靠近 forwardSlow0403 ",Big_battle[1])
                    action_append("forwardSlow0403")


                    if Big_battle[1] == 0 and Big_battle[0] == 0:
                        print("直立")
                        action_append("stand")
                        print("地雷通关")
                        cv2.destroyAllWindows()
                        break

                elif Big_battle[1] < 355:
                    print("慢慢前进靠近 Forwalk01",Big_battle[1])
                    action_append("Stand")
                    action_append("Forwalk01")

                elif (180<= Big_battle[0] and Big_battle[0] < 260):
                    print("右平移一点点 Right02move",Big_battle[0])
                    action_append("Stand")
                    action_append("Right02move")
                elif (260<= Big_battle[0] and Big_battle[0]< 350):
                    print("右平移一步 Right3move",Big_battle[0])
                    action_append("Stand")
                    action_append("Right3move")

                elif (350<= Big_battle[0] and Big_battle[0]< 420):
                    print("向左平移一步 Left3move",Big_battle[0])
                    action_append("Stand")
                    action_append("Left3move")

                elif (420<= Big_battle[0] < 490):
                    print("向左平移一点点 Left02move",Big_battle[0])
                    action_append("Stand")
                    action_append("Left02move")

                else:
                    print("不在调整范围，前进")
                    action_append("Forwalk01")
            else:
                print("未识别到轮廓，继续向前")
                action_append("forwardSlow0403")
                Big_battle = [0,0]

                if img_debug:
                    cv2.circle(Corg_img, (left_point[0],left_point[1]), 7, (0, 255, 0), -1) # 圆点标记
                    cv2.circle(Corg_img, (right_point[0],right_point[1]), 7, (0, 255, 255), -1) # 圆点标记
                    cv2.circle(Corg_img, (Big_battle[0],Big_battle[1]), 7, (255, 255, 0), -1) # 圆点标记
                    cv2.line(Corg_img, (Big_battle[0],Big_battle[1]), (240,640), (0, 255, 255), thickness=2)
                    cv2.line(Corg_img, (0,500), (480,500), (255, 255, 255), thickness=2)
                    cv2.imwrite('./img_debug.jpg', Corg_img)   #显示识别情况

    return True
'''





if __name__ == '__main__':
    rospy.init_node('obstacle')
    time.sleep(3)
    obstacle()

