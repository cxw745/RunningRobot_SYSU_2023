#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy

from image_converter import ImgConverter

img_debug = True
box_debug = False

chest_r_width = 480
chest_r_height = 640
head_r_width = 640
head_r_height = 480


chest_ret = True    
ChestOrg_img = None  

color_range = {
    'blue_floor':[(24 , 158 , 251), (30 , 233 , 255)],
    'green_floor':[(78 , 85 , 190), (87 , 117 , 255)],
    'red_floor':[(0 , 112 , 135), (8 , 202 , 244)],
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




#################################################台阶##########################################
def floor(action_append):
    global org_img, state, state_sel, step, debug
    state_sel = 'floor'

    if state_sel == 'floor':  # 初始化
        print("/-/-/-/-/-/-/-/-/-进入floor")
        step = 0
        
    r_w = chest_r_width
    r_h = chest_r_height

    top_angle = 0
    T_B_angle = 0
    topcenter_x = 0.5 * r_w
    topcenter_y = 0
    bottomcenter_x = 0.5 * r_w
    bottomcenter_y = 0

    while state_sel == 'floor':     

 # 分析图像
     
     # chest
        if True:    # 上下边沿

            t1 = cv2.getTickCount()

            OrgFrame = ChestOrg_img.copy()

            # 初始化 bottom_right  bottom_left
            bottom_right = (480,0)
            bottom_left =  (0,0)
            top_right = (480,0)  # 右上角点坐标
            top_left = (0,0)  # 左上角点坐标

            frame = cv2.resize(OrgFrame, (chest_r_width, chest_r_height), interpolation=cv2.INTER_LINEAR)
            frame_copy = frame.copy()
            # 获取图像中心点坐标x, y
            center = []
            # 开始处理图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
            if step == 0:
                Imask = cv2.inRange(hsv, color_range['blue_floor'][0], color_range['blue_floor'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            elif step == 1:
                Imask = cv2.inRange(hsv, color_range['blue_floor'][0], color_range['blue_floor'][1])
            elif step == 2:
                Imask = cv2.inRange(hsv, color_range['green_floor'][0], color_range['green_floor'][1])
            elif step == 3:
                Imask = cv2.inRange(hsv, color_range['red_floor'][0], color_range['red_floor'][1])
            elif step == 4:
                Imask = cv2.inRange(hsv, color_range['green_floor'][0], color_range['green_floor'][1])
            elif step == 5:
                Imask = cv2.inRange(hsv, color_range['blue_floor'][0], color_range['blue_floor'][1])
            elif step == 6:
                frame_1 = cv2.inRange(hsv, color_range['red_XP1'][0],color_range['red_XP1'][1])  # 对原图像和掩模(颜色的字典)进行位运算
                frame_2 = cv2.inRange(hsv, color_range['red_XP2'][0], color_range['red_XP2'][1])
                Imask = cv2.bitwise_or(frame_1, frame_2)
            elif step == 7:
                Imask = cv2.inRange(hsv, color_range['blue_floor'][0], color_range['blue_floor'][1])
            else:
                print("no color")
            

            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)

            cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
            
            cnt_sum, area_max = getAreaMaxContour1(cnts)  # 找出最大轮廓
            C_percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
            cv2.drawContours(frame, cnt_sum, -1, (255, 0, 255), 3)

            if cnt_sum is not None:
                see = True
                rect = cv2.minAreaRect(cnt_sum)#最小外接矩形
                box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
                
                bottom_right = cnt_sum[0][0]  # 右下角点坐标
                bottom_left = cnt_sum[0][0]  # 左下角点坐标
                top_right = cnt_sum[0][0]  # 右上角点坐标
                top_left = cnt_sum[0][0]  # 左上角点坐标
                for c in cnt_sum:

                    if c[0][0] + 1 * (r_h - c[0][1]) < bottom_left[0] + 1 * (r_h - bottom_left[1]):
                        bottom_left = c[0]
                    if c[0][0] + 1 * c[0][1] > bottom_right[0] + 1 * bottom_right[1]:
                        bottom_right = c[0]

                    if c[0][0] + 3 * c[0][1] < top_left[0] + 3 * top_left[1]:
                        top_left = c[0]
                    if (r_w - c[0][0]) + 3 * c[0][1] < (r_w - top_right[0]) + 3 * top_right[1]:
                        top_right = c[0]

                    # if debug:
                    #     handling = ChestOrg_img.copy()
                    #     cv2.circle(handling, (c[0][0], c[0][1]), 5, [0, 255, 0], 2)
                    #     cv2.circle(handling, (bottom_left[0], bottom_left[1]), 5, [255, 255, 0], 2)
                    #     cv2.circle(handling, (bottom_right[0], bottom_right[1]), 5, [255, 0, 255], 2)
                    #     cv2.imwrite('./handling.jpg', handling)  # 显示图像

                bottomcenter_x = (bottom_left[0] + bottom_right[0]) / 2  # 得到bottom中心坐标
                bottomcenter_y = (bottom_left[1] + bottom_right[1]) / 2

                topcenter_x = (top_right[0] + top_left[0]) / 2  # 得到top中心坐标
                topcenter_y = (top_left[1] + top_right[1]) / 2

                bottom_angle =  -math.atan( (bottom_right[1]-bottom_left[1]) / (bottom_right[0]-bottom_left[0]) ) *180.0/math.pi
                top_angle =  -math.atan( (top_right[1]-top_left[1]) / (top_right[0]-top_left[0]) ) *180.0/math.pi
                if math.fabs(topcenter_x - bottomcenter_x) <= 1:  # 得到连线的角度
                    T_B_angle = 90
                else:
                    T_B_angle = - math.atan((topcenter_y - bottomcenter_y) / (topcenter_x - bottomcenter_x)) * 180.0 / math.pi

                if img_debug:
                    cv2.drawContours(frame_copy, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上
                    cv2.line(frame_copy, (bottom_left[0],bottom_left[1]), (bottom_right[0],bottom_right[1]), (255, 255, 0), thickness=2)
                    cv2.line(frame_copy, (top_left[0],top_left[1]), (top_right[0],top_right[1]), (255, 255, 0), thickness=2)
                    cv2.line(frame_copy, (int(bottomcenter_x),int(bottomcenter_y)), (int(topcenter_x),int(topcenter_y)), (255, 255, 255), thickness=2)    # T_B_line

                    cv2.putText(frame_copy, "bottom_angle:" + str(bottom_angle), (30, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    cv2.putText(frame_copy, "top_angle:" + str(top_angle),(30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                    cv2.putText(frame_copy, "T_B_angle:" + str(T_B_angle),(30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                    cv2.putText(frame_copy, "bottomcenter_x:" + str(bottomcenter_x), (30, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    cv2.putText(frame_copy, "y:" + str(int(bottomcenter_y)), (300, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                    cv2.putText(frame_copy, "topcenter_x:" + str(topcenter_x), (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    cv2.putText(frame_copy, "topcenter_y:" + str(int(topcenter_y)), (230, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                    cv2.putText(frame_copy, 'C_percent:' + str(C_percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                    cv2.putText(frame_copy, "step:" + str(step), (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                    
                    cv2.circle(frame_copy, (int(topcenter_x), int(topcenter_y)), 5, [255, 0, 255], 2)
                    cv2.circle(frame_copy, (int(bottomcenter_x), int(bottomcenter_y)), 5, [255, 0, 255], 2)
                    cv2.circle(frame_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                    cv2.circle(frame_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                    cv2.circle(frame_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                    cv2.circle(frame_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                    cv2.imwrite('./Chest_Camera.jpg', frame_copy)  # 显示图像
                    cv2.imwrite('./chest_red_mask.jpg', Imask)
            # 决策执行动作
                angle_ok_flag = False

                if step == 0:   # 前进依据chest 调整大致位置，方向  看底边线调整角度
                    
                    if bottomcenter_y < 300:
                        if bottom_angle > 3:  # 需要左转
                            if bottom_angle > 6:
                                print("4085L 大左转一下  turn001L ",bottom_angle)
                                action_append("turn001L")
                            else:
                                print("4088L bottom_angle > 3 需要小左转 turn001L ",bottom_angle)
                                action_append("turn001L")
                        elif bottom_angle < -3:  # 需要右转
                            if bottom_angle < -6:
                                print("4092L 右da旋转  turn001R < -6 ",bottom_angle)
                                action_append("turn001R")
                            else:
                                print("4095L bottom_angle < -3 需要小右转 turn001R ",bottom_angle)
                                action_append("turn001R")
                        elif -3 <= bottom_angle <= 3:  # 角度正确
                            print("4098L 角度合适")

                            if topcenter_x > 250 or topcenter_x < 230:
                                if topcenter_x > 250:
                                    print("微微右移,",topcenter_x)
                                    action_append("Right3move")
                                elif topcenter_x < 230:
                                    print("微微左移,",topcenter_x)
                                    action_append("Left3move")

                            else:
                                print("位置合适")
                                print("快步走,bottomcenter_y",bottomcenter_y)
                                action_append("fastForward04")

                    elif bottomcenter_y < 380:
                        if bottom_angle > 3:  # 需要左转
                            if bottom_angle > 6:
                                print("4116L 大左转一下  turn001L ",bottom_angle)
                                action_append("turn001L")
                            else:
                                print("4119L bottom_angle > 3 需要小左转 turn001L ",bottom_angle)
                                action_append("turn001L")
                        elif bottom_angle < -3:  # 需要右转
                            if bottom_angle < -6:
                                print("4123L 右da旋转  turn001R < -6 ",bottom_angle)
                                action_append("turn001R")
                            else:
                                print("4126L bottom_angle < -3 需要小右转 turn001R ",bottom_angle)
                                action_append("turn001R")
                        elif -3 <= bottom_angle <= 3:  # 角度正确
                            print("4129L 角度合适")
                            angle_ok_flag = True

                        if angle_ok_flag:
                            if topcenter_x > 250 or topcenter_x < 230:
                                if topcenter_x > 250:
                                    print("微微右移,",topcenter_x)
                                    action_append("Right02move")
                                elif topcenter_x < 230:
                                    print("微微左移,",topcenter_x)
                                    action_append("Left02move")
                            else:
                                print("4141L 继续前行 forwardSlow0403",bottomcenter_y)
                                action_append("forwardSlow0403")

                    elif 380 <= bottomcenter_y <430:
                        if bottom_angle > 3:  # 需要左转
                            if bottom_angle > 6:
                                print("4147L 大左转一下  turn001L ",bottom_angle)
                                action_append("turn001L")
                            else:
                                print("4150L bottom_angle > 3 需要小左转 turn001L ",bottom_angle)
                                action_append("turn001L")
                        elif bottom_angle < -3:  # 需要右转
                            if bottom_angle < -6:
                                print("4154L 右da旋转  turn001R < -6 ",bottom_angle)
                                action_append("turn001R")
                            else:
                                print("4157L bottom_angle < -3 需要小右转 turn001R ",bottom_angle)
                                action_append("turn001R")
                        elif -3 <= bottom_angle <= 3:  # 角度正确
                            print("4160L 角度合适")
                            angle_ok_flag = True

                        if angle_ok_flag:
                            if topcenter_x > 250 or topcenter_x < 230:
                                if topcenter_x > 250:
                                    print("微微右移,",topcenter_x)
                                    action_append("Right02move")
                                elif topcenter_x < 230:
                                    print("微微左移,",topcenter_x)
                                    action_append("Left02move")
                            else:
                                print("4172L 变小步继续前行 Forwalk00",bottomcenter_y)
                                action_append("Forwalk01")

                    elif 430 <= bottomcenter_y <= 550:
                        if bottom_angle > 3:  # 需要左转
                            if bottom_angle > 6:
                                print("4178L 大左转一下  turn001L ",bottom_angle)
                                action_append("turn001L")
                            else:
                                print("4181L bottom_angle > 3 需要小左转 turn001L ",bottom_angle)
                                action_append("turn001L")
                        elif bottom_angle < -3:  # 需要右转
                            if bottom_angle < -6:
                                print("4185L 右da旋转  turn001R < -6 ",bottom_angle)
                                action_append("turn001R")
                            else:
                                print("4188L bottom_angle < -3 需要小右转 turn001R ",bottom_angle)
                                action_append("turn001R")
                        elif -3 <= bottom_angle <= 3:  # 角度正确
                            print("4191L 角度合适")
                            angle_ok_flag = True

                        if angle_ok_flag:
                            if topcenter_x > 250 or topcenter_x < 230:
                                if topcenter_x > 250:
                                    print("微微右移,",topcenter_x)
                                    action_append("Right02move")
                                elif topcenter_x < 230:
                                    print("微微左移,",topcenter_x)
                                    action_append("Left02move")
                            else:
                                print("4203L 到达上台阶边沿，变前挪动 Forwalk00 bottomcenter_y:",bottomcenter_y)
                                action_append("Forwalk00")


                    elif bottomcenter_y > 550:
                        print("然后开始第二步------上第一节台阶")
                        action_append("Forwalk00")  #往前调整一小步
                        step = 1
                        angle_ok_flag = False
                    else:
                        print("error 前进 C_percent:",C_percent)
                        print("bottomcenter_y:",bottomcenter_y)

                elif step == 1: # 看中线调整角度上台阶----第一阶
                    if top_angle < -2:  # 右转
                        print("4236L 右转 turn001R top_angle:",top_angle)
                        action_append("turn001R")
                        time.sleep(0.5)   # timefftest
                    elif top_angle > 2:  # 左转
                        print("4240L 左转 turn001L top_angle:",top_angle)
                        action_append("turn001L")
                        time.sleep(0.5)   # timefftest
                    elif -2 <= top_angle <= 2:
                        print("上台阶前走一小步")
                        action_append("Forwalk00")
                        time.sleep(0.5)
                        print("4247L 上台阶 上台阶 UpBridge2")
                        action_append("UpBridge2")
                        print("————————————————————————开始上第二节台阶")
                        time.sleep(0.5)
                        
                        step=2


                elif step == 2: # 看中线调整角度上台阶----第二阶

                    if top_angle < -2:  # 右转
                        print("4274L 右转 turn001R top_angle:",top_angle)
                        action_append("turn001R")
                        time.sleep(0.5)   # timefftest
                    elif top_angle > 2:  # 左转
                        print("4278L 左转 turn001L top_angle:",top_angle)
                        action_append("turn001L")
                        time.sleep(0.5)   # timefftest
                    elif -2 <= top_angle <= 2:
                        print("4282L 上台阶 上台阶 UpBridge2")
                        action_append("UpBridge2")

                        print("————————————————————————开始上第三节台阶")
                        time.sleep(0.5)

                        step = 3

                elif step == 3: # 看中线调整角度上台阶----第三阶

                    if top_angle < -2:  # 右转
                        print("4310L 右转 turn001R top_angle:",top_angle)
                        action_append("turn001R")
                        time.sleep(0.5)   # timefftest
                    elif top_angle > 2:  # 左转
                        print("4314L 左转 turn001L top_angle:",top_angle)
                        action_append("turn001L")
                        time.sleep(0.5)   # timefftest
                    elif -2 <= top_angle <= 2:
                        print("4318L 上台阶 上台阶 UpBridge2")
                        action_append("UpBridge2")

                        print("————————————————————————上台阶完毕，开始下台阶")
                        print("————————————————————————开始下第一节台阶")
                        time.sleep(0.5)

                        step = 4


                elif step == 4: #  调整角度下台阶----第三阶
                    time.sleep(0.5)
                    if top_angle > 2:  # 需要左转
                        print("4337 top_angle > 2 需要小左转 ",top_angle)
                        action_append("turn001L")
                    elif top_angle < -2:  # 需要右转
                        print("4340 top_angle < -2 需要小右转 ",top_angle)
                        action_append("turn001R")
                    elif -2 <= top_angle <= 2:  # 角度正确
                        print("角度合适")
                        if topcenter_y < 350:
                            print("微微前挪")
                            action_append("Forwalk00")
                        elif topcenter_y > 350:
                            print("4348L 下台阶 下台阶 DownBridge topcenter_y:",topcenter_y)
                            action_append("DownBridge")
                            print("————————————————————————开始下第二节台阶")
                            time.sleep(0.5)
                            step = 5

                elif step == 5: #  调整角度下台阶----第二阶
                    time.sleep(0.5)
                    if top_angle > 2:  # 需要左转
                        print("4357 top_angle > 2 需要小左转 ")
                        action_append("turn001L")
                    elif top_angle < -2:  # 需要右转
                        print("4360 top_angle < -2 需要小右转 ")
                        action_append("turn001R")
                    elif -2 <= top_angle <= 2:  # 角度正确
                        print("角度合适")
                        if topcenter_y < 350:
                            print("微微前挪")
                            action_append("Forwalk00")
                        elif topcenter_y > 350:
                            print("4368L 下台阶 下台阶 DownBridge topcenter_y:",topcenter_y)
                            action_append("DownBridge")
                            print("————————————————————————开始下第二节台阶")
                            time.sleep(0.5)
                            step = 6
                
                elif step == 6:#  调整角度下斜坡----第三阶
                    time.sleep(0.5) 
                    if top_angle > 2:  # 需要左转
                        print("4377 top_angle > 2 需要小左转 y:",topcenter_y)
                        action_append("turn001L")
                    elif top_angle < -2:  # 需要右转
                        print("4380 top_angle < -2 需要小右转 y:",topcenter_y)
                        action_append("turn001R")
                    elif -2 <= top_angle <= 2:  # 角度正确
                        
                        print("角度合适")
                        print("topcenter_x=",topcenter_x)

                        if topcenter_x > 250 or topcenter_x < 230:
                            if topcenter_x > 250:
                                print("微微右移",topcenter_x)
                                action_append("Right02move")
                            elif topcenter_x < 230:
                                print("微微左移",topcenter_x)
                                action_append("Left02move")

                        else:
                            print("位置合适")
                            print("下斜坡")
                            action_append("Stand")
                            action_append("forwardSlow0403")
                            action_append("forwardSlow0403")
                            action_append("forwardSlow0403")
                            action_append("forwardSlow0403")
                            action_append("Stand")
                            action_append("forwardSlow0403")
                            action_append("forwardSlow0403")
                            action_append("forwardSlow0403")
                            action_append("forwardSlow0403")
                            action_append("Stand")

                            print("XP结束")
                            step = 7

                elif step == 7: # 完成
                    print("899L 完成floor")
                    break
            else:
                print("未找到第一届蓝色台阶")
                action_append("Forwalk00")
    
    return True



if __name__ == '__main__':
    rospy.init_node('Take_the_stairs')
    time.sleep(3)
    floor()