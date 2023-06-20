#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy

from image_converter import ImgConverter

img_debug = False
box_debug = False

chest_r_width = 480
chest_r_height = 640
head_r_width = 640
head_r_height = 480


chest_ret = True    
ChestOrg_img = None  
real_test = 1
sleep_time_s = 0.01
sleep_time_l = 0.05

color_range = {
    # 'green_bridge':[(78 , 65 , 210), (91 , 109 , 255)],
    'green_bridge':[(71 , 42 , 93), (94 , 99 , 255)],
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



# ###################### 过   独   木   桥-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
def Greenbridge(action_append):
    global state_sel, org_img, step, reset, skip, debug, chest_ret

    colorMask = 'green_bridge'
    r_w = chest_r_width
    r_h = chest_r_height

    step = 0

    print("/-/-/-/-/-/-/-/-/-进入Greenbridge")

    while True:  # 初始化

        # 开始处理图像
        
        chest_copy = ChestOrg_img.copy()
        # chest
        cv2.rectangle(chest_copy, (0, 0), (480, 150), (255, 255, 255), -1)
        border = cv2.copyMakeBorder(chest_copy, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        Chest_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放

        Chest_frame_gauss = cv2.GaussianBlur(Chest_img_copy, (3, 3), 0)  # 高斯模糊
        Chest_frame_hsv = cv2.cvtColor(Chest_frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        Chest_frame_green = cv2.inRange(Chest_frame_hsv, color_range[colorMask][0],
                                        color_range[colorMask][1])  # 对原图像和掩模(颜色的字典)进行位运算
        if img_debug:
            # cv2.imshow("mask", Chest_frame_green)
            cv2.imwrite('./mask.jpg', Chest_frame_green)  # 显示图像
        Chest_opened = cv2.morphologyEx(Chest_frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        Chest_closed = cv2.morphologyEx(Chest_opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

        Chest_contours, hierarchy = cv2.findContours(Chest_closed, cv2.RETR_LIST,
                                                          cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        # print("Chest_contours len:",len(Chest_contours))
        Chest_areaMaxContour, Chest_area_max = getAreaMaxContour1(Chest_contours)  # 找出最大轮廓
        Chest_percent = round(Chest_area_max * 100 / (r_w * r_h), 2)

        if Chest_areaMaxContour is not None:
            found = 1
            Chest_rect = cv2.minAreaRect(Chest_areaMaxContour)
            # center, w_h, Head_angle = rect  # 中心点 宽高 旋转角度
            Chest_box = np.int0(cv2.boxPoints(Chest_rect))  # 点的坐标

            # 初始化四个顶点坐标
            Chest_top_left = Chest_areaMaxContour[0][0]
            Chest_top_right = Chest_areaMaxContour[0][0]
            Chest_bottom_left = Chest_areaMaxContour[0][0]
            Chest_bottom_right = Chest_areaMaxContour[0][0]
            for c in Chest_areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < Chest_top_left[0] + 1.5 * Chest_top_left[1]:
                    Chest_top_left = c[0]
                if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - Chest_top_right[0]) + 1.5 * Chest_top_right[1]:
                    Chest_top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < Chest_bottom_left[0] + 1.5 * (r_h - Chest_bottom_left[1]):
                    Chest_bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > Chest_bottom_right[0] + 1.5 * Chest_bottom_right[1]:
                    Chest_bottom_right = c[0]
            angle_top = - math.atan(
                (Chest_top_right[1] - Chest_top_left[1]) / (Chest_top_right[0] - Chest_top_left[0])) * 180.0 / math.pi
            angle_bottom = - math.atan((Chest_bottom_right[1] - Chest_bottom_left[1]) / (
                        Chest_bottom_right[0] - Chest_bottom_left[0])) * 180.0 / math.pi
            Chest_top_center_x = int((Chest_top_right[0] + Chest_top_left[0]) / 2)
            Chest_top_center_y = int((Chest_top_right[1] + Chest_top_left[1]) / 2)
            Chest_bottom_center_x = int((Chest_bottom_right[0] + Chest_bottom_left[0]) / 2)
            Chest_bottom_center_y = int((Chest_bottom_right[1] + Chest_bottom_left[1]) / 2)
            Chest_center_x = int((Chest_top_center_x + Chest_bottom_center_x) / 2)
            Chest_center_y = int((Chest_top_center_y + Chest_bottom_center_y) / 2)
            if img_debug:
                cv2.drawContours(Chest_img_copy, [Chest_box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(Chest_img_copy, (Chest_top_right[0], Chest_top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_top_left[0], Chest_top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_bottom_right[0], Chest_bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_bottom_left[0], Chest_bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_top_center_x, Chest_top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_bottom_center_x, Chest_bottom_center_y), 5, [0, 255, 255], 2)
                cv2.circle(Chest_img_copy, (Chest_center_x, Chest_center_y), 7, [255, 255, 255], 2)
                cv2.line(Chest_img_copy, (Chest_top_center_x, Chest_top_center_y),
                         (Chest_bottom_center_x, Chest_bottom_center_y), [0, 255, 255], 2)  # 画出上下中点连线
            if math.fabs(Chest_top_center_x - Chest_bottom_center_x) <= 1:  # 得到连线的角度
                Chest_angle = 90
            else:
                Chest_angle = - math.atan((Chest_top_center_y - Chest_bottom_center_y) / (
                            Chest_top_center_x - Chest_bottom_center_x)) * 180.0 / math.pi
        else:
            Chest_angle = 90
            Chest_center_x = -1
            Chest_bottom_center_x = -1
            Chest_bottom_center_y = -1
            Chest_top_center_x = -1
            Chest_top_center_y = -1

            angle_top = 90
            angle_bottom = 90
            found = 0


        if img_debug:
            cv2.drawContours(Chest_img_copy, Chest_contours, -1, (255, 0, 255), 1)
            cv2.putText(Chest_img_copy, 'Chest_percent:' + str(Chest_percent) + '%', (30, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "Chest_angle:" + str(int(Chest_angle)), (30, 55), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "Chest_bottom_center(x,y): " + str(int(Chest_bottom_center_x)) + " , " + str(
                int(Chest_bottom_center_y)), (30, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy,
                        "Chest_top_center(x,y): " + str(int(Chest_top_center_x)) + " , " + str(int(Chest_top_center_y)),
                        (30, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "angle_top:" + str(int(angle_top)), (30, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "angle_bottom:" + str(int(angle_bottom)), (30, 165), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(Chest_img_copy, "step :" + str(int(step)), (30, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.imwrite('./Chest_Camera.jpg', Chest_img_copy)  # 显示图像

        # 决策执行动作

        if step == 0:  # 接近 看下边沿  角度  Chest_percent > 5
            if found == 0 or Chest_percent < 0.1:
                print("step=0 什么也没有看到，向左转90° turn005L")
                if real_test:
                    action_append("turn001L")
                    time.sleep(sleep_time_s)

            elif Chest_percent > 16 and Chest_bottom_center_y >460:
                print("step=0, 上桥了")
                step = 1

            elif angle_bottom > 5:
                if angle_bottom > 7:
                    print("step=0 大左转一下 > 8  turn001L angle_bottom={}".format(angle_bottom))
                    if real_test:
                        action_append("turn001L")
                        time.sleep(sleep_time_s)
                        # if Chest_bottom_center_x > 260 and Chest_bottom_center_y < 400:
                        #     print("1016L 再向右移一些 Right3move angle_bottom={}".format(angle_bottom))
                        #     action_append("Right3move")
                else:
                    print("step=0 小左转 turn001L angle_bottom={}".format(angle_bottom))
                    if real_test:
                        action_append("turn001L")
                # time.sleep(1)
            elif angle_bottom < -5:
                if angle_bottom < -7:
                    print("step=0 大右转一下 < -8  turn001R angle_bottom={}".format(angle_bottom))
                    if real_test:
                        action_append("turn001R")
                        time.sleep(sleep_time_s)
                        # if Chest_bottom_center_x < 200 and Chest_bottom_center_y < 400:
                        #     print("1030L 再向左移一些 Right3move angle_bottom={}".format(angle_bottom))
                        #     action_append("Left3move")
                else:
                    print("step=0 小右转 turn001R angle_bottom={}".format(angle_bottom))
                    if real_test:
                        action_append("turn001R")
                # time.sleep(1)

            elif Chest_bottom_center_x > 260:  # 右移    center_x
                print("向右移 Right3move  x>260 Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                if real_test:
                    action_append("Right3move")
            elif Chest_bottom_center_x < 200:  # 左移  center_x
                print("向左移 Left3move x<200 Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                if real_test:
                    action_append("Left3move")

            elif  Chest_bottom_center_y < 460:
                if Chest_bottom_center_y <330  and 200 <= Chest_bottom_center_x <= 260:
                    print("y<350 step=0 快速前进 fastForward03 Chest_bottom_center_y={}".format(Chest_bottom_center_y))
                    if real_test:
                        action_append("fastForward03")
                else:
                    print("y<460 step=0 大步前进 两步 Forwalk01 Chest_bottom_center_y={}".format(Chest_bottom_center_y))
                    if real_test:
                        action_append("forwardSlow0403")
                        time.sleep(sleep_time_s)

                        if angle_bottom >= 3:
                            action_append("turn001L")
                        elif angle_bottom <= -3:
                            action_append("turn001R")

                        action_append("forwardSlow0403")
                        time.sleep(sleep_time_l)
                        if angle_bottom >= 3:
                            action_append("turn001L")
                        elif angle_bottom <= -3:
                            action_append("turn001R")

            elif 220 <= Chest_bottom_center_x <= 240:  
                print("快走333 forwardSlow0403")
                if real_test:
                    time.sleep(sleep_time_s)
                    action_append("fastForward03")
                    action_append("turn001R")
                    if angle_bottom > 3:
                        action_append("turn001L")
                        action_append("Left1move")
                    elif angle_bottom <-3:
                        action_append("turn001R")
                        action_append("Right1move")

            else:
                print("step = 0 已经到达绿桥边缘，需要进入下一步对准绿桥")
                step = 1


        

        
        elif step == 1:  # 到绿桥边沿，对准绿桥阶段
            if Chest_bottom_center_y > 565:
                print("step = 1, 已经冲到第二阶段了")
                step = 2
            elif angle_bottom > 2:
                if angle_bottom > 6:
                    print("大左转一下 > 6  turn001L ", angle_bottom)
                    if real_test:
                        action_append("turn001L")
                else:
                    print("小左转 turn001L ", angle_bottom)
                    if real_test:
                        action_append("turn001L")
                # time.sleep(1)
            elif angle_bottom < -2:
                if angle_bottom < -6:
                    print("大右转一下 < -6  turn001R ", angle_bottom)
                    if real_test:
                        action_append("turn001R")
                else:
                    print("小右转 turn001R ", angle_bottom)
                    if real_test:
                        action_append("turn001R")
                # time.sleep(1)
            elif Chest_bottom_center_x > 260:  # 右移    center_x
                print("向右移 Right02move  x>250")
                if real_test:
                    action_append("Right02move")
            elif Chest_bottom_center_x < 220:  # 左移  center_x
                print("向左移 Left02move x<220")
                if real_test:
                    action_append("Left02move")
            else:
                print(" 对准 快走 fastForward03")
                if real_test:
                    action_append("fastForward03")
                    # action_append("turn001R")

        
        
        elif step == 2:  # 已经在独木桥阶段  行走独木桥 调整角度 位置  看中线 角度
            if Chest_percent > 2 and Chest_top_center_y > 360:
                print("step = 2, 接近独木桥中点啦，进入第三阶段")
                step = 3
            elif Chest_percent < 2:
                print("step = 2, 接近独木桥终点啦，进入第四阶段")
                action_append("forwardSlow0403")
                action_append("forwardSlow0403")
                step = 4

            elif Chest_bottom_center_x >= 270:  # 右移    center_x
                if Chest_bottom_center_x >=275:
                    print("step =2 接近左边缘， 先往右大移  Right3move Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                    if real_test:
                            action_append("Right3move")

                    if Chest_bottom_center_x >= 300:
                        print("step = 2 可能是方向偏左了， 再往右转 turn001R Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                        if real_test:
                            action_append("turn001R")
                elif Chest_bottom_center_x >= 270:
                    print("step =2 接近左边缘， 先往右移  Right02move Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                    if real_test:
                            action_append("Right02move")
                else:
                    print("step =2 再向右小移 Right1move  Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                    if real_test:
                        action_append("Right1move")

            elif Chest_bottom_center_x <= 240:  # 左移  center_x
                if Chest_bottom_center_x <=220:
                    print("step =2 接近右边缘， 先往左大移  Left3move Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                    if real_test:
                            action_append("Left3move")

                elif Chest_bottom_center_x <= 245:
                    print("step =2 接近右边缘， 先往左移  Left02move Chest_bottom_center_x={}".format(Chest_bottom_center_x))
                    if real_test:
                            action_append("Left02move")
                else:
                    print("step =2 再向左小移 Left1move Chest_bottom_center_x={} ".format(Chest_bottom_center_x))
                    if real_test:
                        # action_append("Left02move")
                        action_append("Left1move")

            elif Chest_percent > 2 and Chest_top_center_y > 100:
                # 调整角度位置  
                if 0 < Chest_angle < 88:  # 右转
                    if Chest_angle < 87:
                        print("step =2 向右转 turn001R Chest_angle:", Chest_angle)
                        if real_test:
                            action_append("turn001R")
                    else:
                        print("step =2 向右小转 turn000R Chest_angle:", Chest_angle)
                        if real_test:
                            action_append("turn001R")
                        # time.sleep(1)   # timefftest
                elif -88 < Chest_angle < 0:  # 左转
                    if Chest_angle >-87:
                        print("step =2 向左转 turn001L Chest_angle:", Chest_angle)
                        if real_test:
                            action_append("turn001L")
                    else:
                        print("step =2 向左小转 turn000L Chest_angle:", Chest_angle)
                        if real_test:
                            action_append("turn001L")
                        # time.sleep(1)   # timefftest
        

                else :  # 走三步
                    print("step =2 上桥后，快走 forwardSlow0403 Ccenter_y:", Chest_center_x)
                    if real_test:
                        action_append("fastForward03")
                        action_append("turn001R")

                        if 0 < Chest_angle < 87:
                            print("歪了，右转， turn001R Chest_angle={}".format(Chest_angle))
                            if real_test:
                                action_append("turn001R")
                        elif -87 < Chest_angle < 0:
                            print("歪了，左转， turn001L Chest_angle={}".format(Chest_angle))
                            if real_test:
                                action_append("turn001L")  


            else:
                print("1229L 已经下桥")
                step = 3
        
        
        elif step == 3:  # 接近 看上边沿  调整角度  Chest_percent > 5
            if Chest_percent < 1 or Chest_top_center_y > 500:
                print("接近桥终点 快走离开桥 forwardSlow0403 * 2")
                if real_test:
                    action_append("forwardSlow0403")
                    action_append("forwardSlow0403")
                    action_append("forwardSlow0403")
                    action_append("forwardSlow0403")
                    action_append("Stand")

                    step = 4

            elif Chest_top_center_x > 250:  # 右移    center_x
                if Chest_top_center_x > 260:
                    print("step = 3 向右移一大步 Right02move")
                    if  real_test:
                        action_append("Right02move")
                else:
                    print("向右移  >250")
                    if real_test:
                        action_append("Right1move")
            elif Chest_top_center_x < 220:  # 左移  center_x
                if Chest_top_center_x < 200:
                    print("step = 3 向左移一大步 Left02move")
                    if real_test:
                        action_append("Left02move")
                else:
                    print("向左移 <220")
                    if real_test:
                        action_append("Left1move")
                        
            elif angle_top > 3:
                if angle_top > 7:
                    print("大左转一下  turn001L angle_top={}".format(angle_top))
                    if real_test:
                        action_append("turn001L")
                else:
                    print("左转 turn001L angle_top")
                    if real_test:
                        action_append("turn001L")
            elif angle_top < -3:
                if angle_top < -7:
                    print("大右转一下  turn001R angle_top={}".format(angle_top))
                    if real_test:
                        action_append("turn001R")
                else:
                    print("右转 turn001R")
                    if real_test:
                        action_append("turn001R")
            elif 220 <= Chest_top_center_x <= 250:
                # print("1802L 前进一步 forwardSlow0403")
                # action_append("forwardSlow0403")
                print("快走 forwardSlow0403")
                if real_test:
                    action_append("forwardSlow0403")
        
        
        elif step == 4:  # 离开独木桥阶段   chest 出现bridge  依据chest调整角度位置
            print("离开桥")
            step = 5
            break

    return True




if __name__ == '__main__':
    rospy.init_node('Take_the_stairs')
    time.sleep(3)
    Greenbridge()