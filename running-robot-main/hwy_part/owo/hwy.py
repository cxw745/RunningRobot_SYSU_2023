#!/usr/bin/python3
import cv2
import numpy as np
import time
import math
import hiwonder.ActionGroupControl as AGC
import hiwonder.Board as Board
import hwy_part.owo.Misc as utils
import hwy_part.owo.Camera as Camera
from hwy_part.owo.LABDict import color_range

''' ------------------------------------------------ 初始化 ----------------------------------------------------- '''
DEBUG = True
ret = False
NEAREST = 260

''' ---------------------------------------------- go forward ---------------------------------------------------- '''
def go_forward(times = 1):
    AGC.runActionGroup('sysu_go_forward_start')
    AGC.runActionGroup('sysu_go_forward', times)
    AGC.runActionGroup('sysu_go_forward_end')
    # AGC.runActionGroup('stand_low')
    # AGC.runActionGroup('sysu_speed1', times)

''' --------------------------------------------- 第一关：横杆 ---------------------------------------------------- '''

'''
    检测到横杆抬起的动作后return
    1. 识别黑色、黄色，将掩膜相加
    2. 找到最大轮廓，计算轮廓占比
    3. 若轮廓占比>ruler_down_threshold(横杆还在的阈值)说明横杆还在，计数器counter+1
    4. 若轮廓占比<ruler_up_threshold(横杆抬起阈值)说明横杆已抬起，若计数器counter>times_min返回
'''

def ruler():
    print('—————————————————————————— 进入ruler ——————————————————————————')
    ruler_down_threshold = 35     # 横杆还在的阈值
    ruler_up_threshold = 7    # 横杆抬起阈值
    times_min = 7   # 检测到横杆在的帧数，counter大于此值时才能
    counter = 0

    Board.setBusServoPulse(19, 500, 500)
    Board.setBusServoPulse(20, 300, 500)
    AGC.runActionGroup('stand_low')
    my_camera = Camera.Camera('/home/pi/CameraCalibration/calibration_param.npz')
    my_camera.camera_open()
    while my_camera.frame is None:
        time.sleep(0.5)

    # 等到满足条件：检测到出现横杆后横杆抬起
    while True:
        img = cv2.resize(my_camera.frame, (4 * 40, 3 * 40), interpolation = cv2.INTER_CUBIC)  # 将图片缩放，INTER_CUBIC三次样条插值
        img = cv2.GaussianBlur(img, (3, 3), 0)     # 高斯模糊
        img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        img_yellow = cv2.inRange(img, color_range['ruler_yellow'][0], color_range['ruler_yellow'][1])   
        img_black = cv2.inRange(img, color_range['ruler_black'][0], color_range['ruler_black'][1]) 
        img = cv2.add(img_yellow, img_black)    # 将得到的掩膜二值图相加
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 3)  # 闭运算 封闭连接
        img = cv2.copyMakeBorder(img, 3, 3, 4, 4, borderType = cv2.BORDER_CONSTANT, value = 0)  # 扩展边缘，防止边界无法别
        if DEBUG:
            cv2.imshow('1', img)
            cv2.waitKey(1)
        contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        max_contour_area = utils.get_area_max_contour(contours, 20)[1]  # 找出最大轮廓的面积
        percent = 100 * max_contour_area / (img.shape[0] * img.shape[1])  # 最大轮廓的百分比
        print(percent)
        # 根据比例得到是否前进的信息
        if percent > ruler_down_threshold: # 挡板还在，实际经验得出
            counter += 1
        elif counter > times_min and percent < ruler_up_threshold: 
            cv2.destroyAllWindows()
            break

    my_camera.camera_close()

    time.sleep(1)
    go_forward(6)

    if DEBUG:
        print('ruler完成')


''' -------------------------------------------- 回型坑 ---------------------------------------------------- '''
'''
    step0: 使机器人面向坑那块砖的方向，随后才能看到坑
    step1: 使机器人对齐站在坑前
    左移避开坑
    step2: 过坑，根据脚前正方形区域的地面占比调整方向
    右移回来
    step3: 出坑后对齐（未用）
'''


def hole():
    print('—————————————————————————— 进入hole ——————————————————————————')

    # 调整云台，看脚前
    Board.setBusServoPulse(19, 500, 300)
    Board.setBusServoPulse(20, NEAREST - 5, 300)
    AGC.runActionGroup('stand')

    my_camera = Camera.Camera('/home/pi/CameraCalibration/calibration_param.npz')
    my_camera.camera_open()
    while my_camera.frame is None:
        time.sleep(0.5)

    '''—————————————————————————— 调整机器人面向回型坑 ——————————————————————————'''
    ground_percent_threshold = 74 # 大于该值说明面前是坑

    while True:
        org_img = cv2.GaussianBlur(my_camera.frame, (3, 3), 0)  # 高斯滤波
        img = cv2.cvtColor(org_img, cv2.COLOR_BGR2LAB)  # 将图片转化为lab格式
        Imask = cv2.inRange(img, color_range['hole_green'][0], color_range['hole_green'][1] ) # 根据lab值对图片进行二值化，提取洞的周围
        Imask = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 3)
        Imask = cv2.morphologyEx(Imask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 3)
        contours = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  
        max_contour = utils.get_area_max_contour(contours, 700)[0]   # 找到最大轮廓，即回型坑的轮廓
        hull = cv2.convexHull(max_contour)
        hull_area = cv2.contourArea(hull)

        if DEBUG:
            cv2.drawContours(org_img, [max_contour], -1, (0, 0, 0), 3)
            cv2.imshow("mask", org_img)
            cv2.waitKey(1)

        if 100 * hull_area / (Imask.shape[0] * Imask.shape[1]) > ground_percent_threshold:
            break

        stencil = np.zeros(Imask.shape, np.uint8)
        cv2.fillPoly(stencil, [hull], 255)
        ground_left = stencil[:, 0:(stencil.shape[1] // 2)]
        ground_right = stencil[:, (stencil.shape[1] // 2):-1]
        contours_left = cv2.findContours(ground_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        contours_right = cv2.findContours(ground_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        ground_area_left = utils.get_area_max_contour(contours_left, 30)[1]
        ground_area_right = utils.get_area_max_contour(contours_right, 30)[1]

        if(math.fabs(ground_area_left - ground_area_right) > Imask.shape[0] * Imask.shape[1] * 0.05):  # 左右两边地面占比相差较大
            if ground_area_left > ground_area_right: # 左边多左转，右边多右转
                AGC.runActionGroup('sysu_turnleft')
            else:
                AGC.runActionGroup('sysu_turnright')
        else:   # 左右两边相差不多
            go_forward(2)
    if DEBUG:
        cv2.destroyAllWindows()

    print('hole_step0： 面向回型坑 调整完成')



    '''—————————————————————————— 调整机器人对齐站在坑边 ——————————————————————————'''
    dist_to_hole_min = 135       # 离坑距离的最小值，这个值不能太小，太小的话机器人看不到坑的下面那条边，检测不到矩形陷入思考"max_contour_hole is None" 
    dist_far_away = 240
    center_x_range = (280, 360) # 坑的中点在画面的范围
    angle_range = (-5, 5)       # 坑的倾斜角度范围

    while True:
        max_contour_hole = None
        org_img = None
        while max_contour_hole is None:
            org_img = cv2.GaussianBlur(my_camera.frame, (3, 3), 0)  # 高斯滤波
            img = cv2.cvtColor(org_img, cv2.COLOR_BGR2LAB)  # 将图片转化为lab格式
            Imask = cv2.inRange(img, color_range['hole_green'][0], color_range['hole_green'][1] ) # 根据lab值对图片进行二值化，提取洞的周围
            Imask = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 3)
            Imask = cv2.morphologyEx(Imask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 7)
            stencil  = np.zeros(org_img.shape[:-1], np.uint8)   # 全为0的掩膜
            contours = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]    
            max_contour = utils.get_area_max_contour(contours, 1000)[0]   # 找到最大轮廓
            cv2.fillPoly(stencil, [max_contour], 255)   # 将最大区域填充为白色，即将洞填充为白色
            result = cv2.bitwise_xor(Imask, stencil)    # 异或
            contours = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_hole = utils.get_area_max_contour(contours, 700)[0]  #找到最大轮廓


            if max_contour_hole is None:
                print('max_contour_hole is None')
                Board.setBusServoPulse(20, NEAREST - 10, 300)
                time.sleep(1)


        rect = cv2.minAreaRect(max_contour_hole)   # 获取最小外接矩阵，中心点坐标，宽高，旋转角度
        box = np.int0(cv2.boxPoints(rect))          # 获得最小外接矩形的四个顶点坐标 注意 box中坐标是(w, h)！！！

        # 取最近的两个点box[p]、box[q]
        p = 0
        for i in range(4):
            if box[i][1] > box[p][1]:
                p = i
        q = 1
        for i in range(4):
            if i != p and box[i][1] >= box[q][1]:
                q = i

        l = utils.line(box[p], box[q])              # 矩形下侧边
        center = l.mid_point()                      # 矩形下侧边中点
        angle = l.angle()                           # 矩形下侧边倾斜角度
        dist = org_img.shape[0] - center[1]         # 中点到画面下方的距离
        
        if DEBUG:
            cv2.drawContours(org_img, [max_contour_hole], -1, (0, 255, 0), 1)
            cv2.line(org_img, tuple(box[p]), tuple(box[q]), (255, 255, 0), 5)
            cv2.circle(org_img, center, 1, (255, 255, 255), 3)
            cv2.putText(org_img, str(angle), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(org_img, str(dist), (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('org_img', org_img)
            cv2.waitKey(1)

        if angle_range[0] <= angle <= angle_range[1] and dist <= dist_to_hole_min and center_x_range[0] <= center[0] <= center_x_range[1]: # 方向、左右、前后距离都合适
            cv2.destroyAllWindows()
            break

        elif angle > angle_range[1]:
            AGC.runActionGroup('sysu_turnleft')
        elif angle < angle_range[0]:
            AGC.runActionGroup('sysu_turnright')

        elif center[0] < center_x_range[0]:
            AGC.runActionGroup('sysu_left_move')
        elif center[0] > center_x_range[1]:
            AGC.runActionGroup('sysu_right_move')
        elif dist > dist_far_away:
            go_forward(2)
            time.sleep(0.5)
            AGC.runActionGroup('stand')
        elif dist > dist_to_hole_min:
            go_forward(1)

    print('hole_step1： 对齐坑边 调整完成')



    '''—————————————————————————— 调整完成后右移，避开坑 ——————————————————————————'''
    AGC.runActionGroup('sysu_right_move', 6)

    print('左挪完毕')



    '''————————————————————————————————— 过坑 ———————————————————————————————————'''
    Board.setBusServoPulse(20, NEAREST, 300)
    goforward_ground_percent_min = 90
    walk_ground_percent_min = 84
    leave_ground_percent_min = 5
    diff_range = (-0.10, 0.10)
    diff_old = 0
    turn_flag = False

    while True:
        org_img = my_camera.frame
        roi = org_img[200:480, 210:-220] # ROI为机器人脚前方区域
        # if DEBUG:
        #     cv2.imshow('roi', roi)
        #     cv2.waitKey(1)
        roi = cv2.GaussianBlur(roi, (3, 3), 0)
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)  # 将图片转化为lab格式   
        Imask = cv2.inRange(roi, color_range['hole_green'][0], color_range['hole_green'][1] )       # 根据lab值对图片进行二值化
        Imask = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 3)
        Imask = cv2.morphologyEx(Imask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 3)  
        contours = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  
        max_contour_area = utils.get_area_max_contour(contours, 30)[1]   # 找到最大轮廓的面积
        ground_percent = max_contour_area / (roi.shape[0] * roi.shape[1]) * 100

        if DEBUG:
            # cv2.rectangle(org_img, (160, 320), (480, 480), (255, 0, 255), 1)
            # cv2.rectangle(org_img, (80, 420), (160, 480), (0, 255, 255), 1)
            # cv2.rectangle(org_img, (480, 420), (560, 480), (0, 255, 255), 1)
            cv2.imshow('hole step2', org_img)
            cv2.waitKey(1)
            print(ground_percent)

        if ground_percent > goforward_ground_percent_min:
            go_forward(2)
            turn_flag = False
            time.sleep(0.8)

        elif ground_percent > walk_ground_percent_min: # 绿色区域占比大于walk_ground_percent_min说明已避开坑可以走
            turn_flag = False
            go_forward(1)

        elif ground_percent < leave_ground_percent_min: #绿色区域占比小于leave_ground_percent_min说明已经离开坑
            cv2.destroyAllWindows()
            break

        else:   # 对比左半边和右半边绿色地面的占比

            Imask_left = Imask[:, 0:(Imask.shape[1] // 2)]
            Imask_right = Imask[:, (Imask.shape[1] // 2):-1]
            contours_left = cv2.findContours(Imask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contours_right = cv2.findContours(Imask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_area_left = utils.get_area_max_contour(contours_left, 30)[1]
            max_contour_area_right = utils.get_area_max_contour(contours_right, 30)[1]
            diff = (max_contour_area_left - max_contour_area_right) / (Imask.shape[0] * Imask.shape[1])
            if DEBUG:
                print('diff:', diff)

            if diff > diff_range[1]:  # 左右两边地面占比相差较大
                if turn_flag and diff > diff_old:
                    AGC.runActionGroup('sysu_turnright', 2)
                else:
                    AGC.runActionGroup('sysu_turnleft')
                turn_flag = True
                time.sleep(1)
            elif diff < diff_range[0]:
                AGC.runActionGroup('sysu_turnright')
                turn_flag = True
                time.sleep(1)
            elif diff < 0.04:
                go_forward(2)
                turn_flag = False
            else:   # 准备离开坑，左右两边相差不多
                go_forward(1)
                turn_flag = False
            diff_old = diff
            

        # cv2.waitKey(0)
        # time.sleep(1)

    print('hole_step2： 已离开回型坑')



    '''————————————————————————————————— 出坑后站直，移回来 ———————————————————————————————————'''
    go_forward(1)
    AGC.runActionGroup('sysu_left_move', 7)
    go_forward(1)
    AGC.runActionGroup('stand')

    my_camera.camera_close()
    print('hole结束')


''' -------------------------------------------- 第三关：地雷 ---------------------------------------------------- '''
def mine():
    print('—————————————————————————— 进入mine ——————————————————————————')
    # 调整云台，只看脚前

    Board.setBusServoPulse(19, 500, 300)
    Board.setBusServoPulse(20, NEAREST + 30, 300)
    AGC.runActionGroup('stand')

    my_camera = Camera.Camera('/home/pi/CameraCalibration/calibration_param.npz')
    my_camera.camera_open()
    while my_camera.frame is None:
        time.sleep(0.5)

    dist_to_baffle_min = 170
    dist_no_correct_by_ground = 420
    angle_range = (-7, 7)
    ground_percent_min = 94
    roi_range = (125, -125)
    roi_y_loc = 210
    diff_range = (-0.04, 0.04)

    # state 
    # 1: 检测挡板，根据挡板调整 2:根据地面占比调整 3:结束
    state = 0
    while state != 3:
        # 检测挡板
        state = 1
        while state == 1:
            org_img = my_camera.frame
            img = cv2.GaussianBlur(org_img, (3, 3), 0)     # 高斯模糊
            img = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            img = cv2.inRange(img, color_range['baffle_blue'][0], color_range['baffle_blue'][1])
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 7)
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 7)
            # img = cv2.cv2.copyMakeBorder(img, 3, 3, 4, 4, borderType = cv2.BORDER_CONSTANT, value = 0)
            contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
            max_contour = utils.get_area_max_contour(contours, 500)[0]
            if max_contour is None:
                if DEBUG: 
                    print('can not find baffle')
                state = 2
                break

            rect = cv2.minAreaRect(max_contour)    # 获取最小外接矩阵，中心点坐标，宽高，旋转角度
            box = np.int0(cv2.boxPoints(rect))          # 获得最小外接矩形的四个顶点坐标 注意 box中坐标是(w, h)！！！
            # 获得较长的边
            p = 0
            q = 1 
            if utils.dist2(box[0], box[q]) < utils.dist2(box[2], box[q]):
                p = 2
            l = utils.line(box[p], box[q])  # 取最长的那条边
            center = l.mid_point()
            angle = l.angle()
            dist = img.shape[0] - center[1]

            if DEBUG:
                cv2.drawContours(org_img, [max_contour], -1, (255, 0, 255), 2)
                cv2.line(org_img, tuple(box[p]), tuple(box[q]), (0, 0, 255), 3)
                cv2.circle(org_img, center, 2, (255, 255, 255), 5)
                cv2.imshow('baffle', org_img)
                cv2.waitKey(1)
                print('find blue baffle, angle:', angle, 'dist:', img.shape[0] - center[1])

            if dist < dist_to_baffle_min:   # 到达挡板前
                if DEBUG: 
                    print('get baffle')
                go_forward(1)
                time.sleep(0.3)
                state = 3
                break

            elif dist < dist_no_correct_by_ground:
                if DEBUG:
                    print(' correct by baffle, dist:', dist)

                if angle < angle_range[0]:
                    AGC.runActionGroup('sysu_turnright')
                elif angle > angle_range[1]:
                    AGC.runActionGroup('sysu_turnleft')
                else:
                    state = 1
                    break

            else:    # 虽识别到挡板但还远，不需要对齐挡板矫正方向
                if DEBUG:
                    print('correct by ground, dist:',dist)
                state = 2
                break
        
        if state == 3:
            break

        while state == 2:
            org_img = cv2.GaussianBlur(my_camera.frame, (3, 3), 0) 
            org_img = org_img[:, 180:460]
            img = cv2.cvtColor(org_img, cv2.COLOR_BGR2LAB)
            Imask = cv2.inRange(img, color_range['mine_ground'][0], color_range['mine_ground'][1])
            Imask = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 3)
            Imask = cv2.morphologyEx(Imask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 7)
            contours = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
            max_contour_area = utils.get_area_max_contour(contours, 70)[1]
            ground_percent = 100 * max_contour_area / (Imask.shape[0] * Imask.shape[1])

            if DEBUG:
                print('ground_percent: ', ground_percent)

            if ground_percent > ground_percent_min:
                state = 1
                break 
            
            else:
                Imask_left = Imask[:, 0:(Imask.shape[1] // 2)]
                Imask_right = Imask[:, (Imask.shape[1] // 2):-1]
                contours_left = cv2.findContours(Imask_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                contours_right = cv2.findContours(Imask_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                max_contour_area_left = utils.get_area_max_contour(contours_left, 30)[1]
                max_contour_area_right = utils.get_area_max_contour(contours_right, 30)[1]
                diff = (max_contour_area_left - max_contour_area_right) / (Imask.shape[0] * Imask.shape[1])
                
                if DEBUG:
                    print("ground diff:", diff)

                if diff < diff_range[0]:
                    AGC.runActionGroup('sysu_turnright')
                elif diff > diff_range[1]:
                    AGC.runActionGroup('sysu_turnleft')
                else:   # 左右两边相差不多，没偏差
                    state = 1
                    break
        

        org_img = my_camera.frame
        roi_img = org_img[roi_y_loc:-1, :]   # 脚前的区域
        roi = cv2.GaussianBlur(roi_img, (3, 3), 0)     # 高斯模糊
        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        roi = cv2.inRange(roi, color_range['mine_black'][0], color_range['mine_black'][1])
        roi = cv2.morphologyEx(roi, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 3)
        roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 3)
        roi_center = roi[:, roi_range[0]:roi_range[1]]   # 中心
        contours = cv2.findContours(roi_center, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        max_contour = utils.get_area_max_contour(contours, 600)[0]

        if DEBUG:
            cv2.line(org_img, (0, roi_y_loc), (640, roi_y_loc), (255, 0, 0), 1)
            cv2.rectangle(org_img, (roi_range[0], roi_y_loc), (640+roi_range[1], 480), (0, 255, 0), 1) 
            cv2.imshow('img', org_img)
            cv2.waitKey(1)

        if max_contour is None: # 脚前没地雷
            go_forward(3)
            time.sleep(0.5)
            continue


        # 有地雷的情况
        mine_x, mine_y = utils.get_contour_location(max_contour)

        if DEBUG:
            cv2.circle(org_img, (mine_x + roi_range[0], mine_y + roi_y_loc), 1, (0, 0, 255), 15)
            cv2.imshow('img', org_img)
            cv2.waitKey(1)

        if mine_y <  roi_center.shape[0] * 0.5:   # 还远
            go_forward(1)

        elif math.fabs(mine_x - (roi_center.shape[1] / 2)) > roi_center.shape[1] * 0.14:   # 若地雷出现在屏幕靠边的位置，左右移动避开
            if mine_x > roi_center.shape[1] / 2:
                if DEBUG:
                    print('find mine on right side, left move')
                AGC.runActionGroup('sysu_left_move')
            else:
                if DEBUG:
                    print('find mine on left side, right move')
                AGC.runActionGroup('sysu_right_move')
        else: # 地雷出现在中间，判断两侧地面占比
            
            roi = cv2.GaussianBlur(org_img, (3, 3), 0)
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
            roi = cv2.inRange(roi, color_range['mine_ground'][0], color_range['mine_ground'][1])
            roi_left = roi[:, 0:320]
            roi_right = roi[:, 320:-1]
            roi_left = roi_left.copy()
            roi_right = roi_right.copy()
            contours_left = cv2.findContours(roi_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            contours_right = cv2.findContours(roi_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            max_contour_area_left = utils.get_area_max_contour(contours_left, 40)[1]
            max_contour_area_right = utils.get_area_max_contour(contours_right, 40)[1]


            if max_contour_area_left > max_contour_area_right:
                print("         in center turn left")
                AGC.runActionGroup('sysu_left_move', 3)
            else:
                print("         in center turn right")
                AGC.runActionGroup('sysu_right_move', 3)

    AGC.runActionGroup('sysu_slow_move')
    cv2.destroyAllWindows()

    '''—————————————————————————————— 对齐站在挡板中间 ——————————————————————————————'''
    '''
    flag = False
    while flag == False:
        # 看右边
        Board.setBusServoPulse(19, 280, 300)
        Board.setBusServoPulse(20, NEAREST - 10, 300)
        time.sleep(1)
        org_img_right = my_camera.frame
        img_right = cv2.GaussianBlur(org_img_right[240:-1, :], (3, 3), 0)   # 截取近处画面
        img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2LAB)
        img_right = cv2.inRange(img_right, color_range['baffle_blue'][0], color_range['baffle_blue'][1])
        img_right = cv2.morphologyEx(img_right, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 10)
        img_right = cv2.morphologyEx(img_right, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 6)
        contours_right = cv2.findContours(img_right, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour_area_right = utils.get_area_max_contour(contours_right, 700)[1]

        #看左边
        Board.setBusServoPulse(19, 720, 300)
        Board.setBusServoPulse(20, NEAREST - 10, 300)
        time.sleep(1)
        org_img_left = my_camera.frame
        img_left = cv2.GaussianBlur(org_img_left[240:-1, :], (3, 3), 0)     # 截取近处画面
        img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2LAB)
        img_left = cv2.inRange(img_left, color_range['baffle_blue'][0], color_range['baffle_blue'][1])
        img_left = cv2.morphologyEx(img_left, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations = 10)
        img_left = cv2.morphologyEx(img_left, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations = 6)
        contours_left = cv2.findContours(img_left, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour_area_left = utils.get_area_max_contour(contours_left, 700)[1]

        if DEBUG:
            cv2.drawContours(org_img_left, contours_left, -1, (255, 0, 255), 2)
            cv2.drawContours(org_img_right, contours_right, -1, (255, 0, 255), 2)
            cv2.imshow('left', org_img_left)
            cv2.waitKey(1)
            cv2.imshow('right', org_img_right)
            cv2.waitKey(1)
            print('rate:', math.fabs(max_contour_area_right - max_contour_area_left) / (max_contour_area_right + max_contour_area_left))

        # 根据占比左右移动
        if max_contour_area_right < (max_contour_area_right + max_contour_area_left) * 0.52:
            
            AGC.runActionGroup('sysu_left_move', 3)
            # 移动完毕后怼一步矫正方向
            go_forward(1)
        elif max_contour_area_left < (max_contour_area_right + max_contour_area_left) * 0.24:
            AGC.runActionGroup('sysu_right_move', 2)
            
            # 移动完毕后怼一步矫正方向
            go_forward(1)
        else: # 满足条件
            if DEBUG:
                cv2.destroyAllWindows()
                AGC.runActionGroup('sysu_left_move', 1)
            flag = True
    '''
    my_camera.camera_close()
    Board.setBusServoPulse(19, 140, 350)
    time.sleep(0.5)

    # AGC.runActionGroup('sysu_roll')

if __name__ == '__main__':
    # ruler()
    # hole()
    mine()
