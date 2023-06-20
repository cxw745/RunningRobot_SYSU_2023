#####lwx part#######

import math
import numpy as np
import cv2
import time
import hiwonder.ActionGroupControl as AGC
import threading
import hiwonder.PID as pid
from hiwonder import Board
from sysu import Camera
import inspect
import ctypes

#######lab color range########
lwx_color_range = {
'red': [(0, 145, 128), (255, 187, 177)], 
'green': [(0, 0, 10), (255, 125, 217)], 
'blue': [(0, 0, 0), (255, 255, 120)], 
'black': [(0, 98, 98), (80, 158, 158)], 
'white': [(220, 0, 0), (255, 250, 255)], 
'yellow': [(0, 81, 167), (255, 125, 240)], 
'purple': [(0, 140, 70), (156, 195, 114)], 
'goal': [(0, 140, 120), (169, 171, 172)], 
'red_stair': [(60, 137, 117),(245, 197, 147)],
'green1': [(55, 53, 59), (255, 115, 164)],
'blue1': [(78, 35, 59), (255, 163, 100)],
'red1': [(59, 148, 102), (255, 176, 153)],
}
#############initialize#######################
#camera = Camera.Camera(param_data_path = 'calibration_param.npz')
#camera.camera_open()
#Running = True
#Debug = True
#step = 0
#state_sel = None
#
#Board.setBusServoPulse(1, 500, 500)  # NO3机器人，初始云台角度
#Board.setBusServoPulse(2, 480, 500)
#
#AGC.runAction('stand')
#ret = False  # 读取图像标志位
#org_img = None  # 全局变量，原始图像
#rs_img = None
#org_img_copy = None
#bridge_color = None
#hole_color = None
#find_hole = False
#checkpoint = 1
#img_debug = False
#print('initialize done')
###############constant###########
ori_img_w = int(640)
ori_img_h = int(480)
rs_img_w = int(320)
rs_img_h = int(240)

def _async_raise(tid, exctype):
    """raises the exception, performs cleanup if needed"""
    tid = ctypes.c_long(tid)
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("invalid thread id")
    elif res != 1:
        # """if it returns a number greater than one, you're in trouble,
        # and you should call it again with exc=NULL to revert the effect"""
        ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
        raise SystemError("PyThreadState_SetAsyncExc failed")
def stop_thread(thread):
    _async_raise(thread.ident, SystemExit)


####### camera ###########
#def get_img():
#    global org_img, rs_img
#    while True:
#        if camera.isOpened():
#            #读取图片
#            ret, org_img = camera.read()
#            #将摄像头画面缩小以便处理
#            rs_img = cv2.resize(org_img, (rs_img_w, rs_img_h),
#                                  interpolation=cv2.INTER_CUBIC)
#        else:
#            time.sleep(0.01)
#
#
#th1 = threading.Thread(target=get_img)
#th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
#th1.start()
#
##################得到最大的轮廓及面积################
def getAreaMaxContour(contours, area_min=36):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area_min:  # 只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓


# 得到所有轮廓的面积 #
def getAreaSumContour(contours):
    contour_area_sum = 0
    for c in contours:  # 历遍所有轮廓
        contour_area_sum += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
    return contour_area_sum  # 返回最大的面积


# 得到线形的总的轮廓 #
def getLineSumContour(contours, area=1):
    contours_sum = None
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        rect = cv2.minAreaRect(c)  # 最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
        edge1 = math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2))
        edge2 = math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2))
        ratio = edge1 / edge2
        if (area_temp > area) and (ratio > 3 or ratio < 0.33):
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        rect = cv2.minAreaRect(c)  # 最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
        edge1 = math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2))
        edge2 = math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2))
        ratio = edge1 / edge2
        if (area_temp > area) and (ratio > 3 or ratio < 0.33):
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有轮廓点拼接到一起
    return contours_sum


# 得到全部的总的轮廓 #
def getSumContour(contours, area=1):
    contours_sum = None
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > area:
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > area:
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有轮廓点拼接到一起
    return contours_sum


# 寻找第一、第二大轮廓#
def getAreaMax2Contour(contours, area_min=20):
    contour_area_temp = 0
    contour_area_max1 = 0
    contour_area_max2 = 0
    area_max1_contour = None
    area_max2_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp >= contour_area_max2:
            if contour_area_temp >= contour_area_max1:
                contour_area_max1_copy = contour_area_max1
                area_max1_contour_copy = area_max1_contour
                contour_area_max1 = contour_area_temp
                area_max1_contour = c
                contour_area_max2 = contour_area_max1_copy
                area_max2_contour = area_max1_contour_copy
            else:
                contour_area_max2 = contour_area_temp
                area_max2_contour = c
        else:
            continue
    if contour_area_max2 > area_min:
        return area_max1_contour, contour_area_max1, area_max2_contour, contour_area_max2  # 返回最大的轮廓
    else:
        return area_max1_contour, contour_area_max1, area_max2_contour, contour_area_max2  # 返回最大的轮廓

#############line length#####################
def line_len(contour):
    rect = cv2.minAreaRect(contour)
    w1 = rect[1][0]
    h1 = rect[1][1]
    angle = rect[2]
    box = cv2.boxPoints(rect)
    left_point_x = np.min(box[:, 0])
    right_point_x = np.max(box[:, 0])
    top_point_y = np.min(box[:, 1])
    bottom_point_y = np.max(box[:, 1])
    left_point_y = box[:, 1][np.where(box[:, 0] == left_point_x)][0]
    right_point_y = box[:, 1][np.where(box[:, 0] == right_point_x)][0]
    top_point_x = box[:, 0][np.where(box[:, 1] == top_point_y)][0]
    bottom_point_x = box[:, 0][np.where(box[:, 1] == bottom_point_y)][0]
    
    second_bottom_point_y = 0
    if left_point_y > second_bottom_point_y:
        second_bottom_point_y = left_point_y
    if right_point_y > second_bottom_point_y:
        second_bottom_point_y = right_point_y
    second_bottom_point_x = box[:, 0][np.where(box[:, 1] == second_bottom_point_y)][0]
    line = 0
    if  left_point_y == top_point_y or left_point_y == bottom_point_y:
        line = right_point_x - left_point_x
    else:
        if w1 > h1 and angle >= 45:
            line = math.sqrt(pow(right_point_x - bottom_point_x, 2) + pow(bottom_point_y - right_point_y, 2))
        elif w1 < h1 and angle < 45:
            line = math.sqrt(pow(left_point_x - bottom_point_x, 2) + pow(bottom_point_y - left_point_y, 2))
        elif w1 > h1 and angle < 45:
            line = math.sqrt(pow(left_point_x - bottom_point_x, 2) + pow(bottom_point_y - left_point_y, 2))
        elif w1 < h1 and angle >= 45: 
            line = math.sqrt(pow(right_point_x - bottom_point_x, 2) + pow(bottom_point_y - right_point_y, 2))
    return line

# 映射函数，缩小后的图片处理后得到的坐标，再放大得到原图所对应的点
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def get_Cannyline(img):
    if img is None:
        return  []
    Corg_img = img.copy()
    sp = Corg_img.shape
    r_w = sp[0]
    r_h = sp[1]
    gray = cv2.cvtColor(Corg_img,cv2.COLOR_BGR2GRAY)
   
    edges = cv2.Canny(gray,10,100) #边缘检测
    Imask = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1) #先膨胀
    contours = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓

    img = edges.copy()#我们把轮廓画成一个二值图像
    m,n = img.shape[:2]
    for i in range(m):
        for j in range(n):
            img[i,j]=0
    for cnt in contours:
        for i in range(cnt.shape[0]):
            img[cnt[i][0][1],cnt[i][0][0]] = 255


    dilate = cv2.dilate(img, np.ones((3, 3), np.uint8), iterations=3) #对轮廓膨胀，方便找直线
    

    # 精度：取值范围0.1-2（默认值1）1.3
    accuracy = 1.3
    # 阈值：取值范围1-255（小于200意义不大，默认值200）200
    threshold = 200
    #霍夫概率直线
    lines = cv2.HoughLinesP(dilate, accuracy, np.pi/180, threshold, minLineLength=int((r_w/6.4)), maxLineGap=int((r_h/24)))#原来最后一个参数是70,20
    #original maxlength = 200

    #我们下面过滤得到我们想要的直线
    ret = []
    if lines is None:
        return []
    else:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1 == x2 or (x1 < (r_h/16) and x2 < (r_h/16)) or(x1 > (r_w/2) and x2 >(r_w/2) ):
                continue
            local_angle = - math.atan((y1-y2)/(x1-x2))*180.0/math.pi
            if local_angle < 0 or abs(local_angle) < 5:
                continue
            ret.append(line)
    
    if ret == []:
        return []

    #下面我们找到这些直线里面最长的一个直线
    max_line = ret[0]
    for line in ret:
        x1, y1, x2, y2 = line[0]
        len1=math.sqrt(math.pow(y2-y1,2) + math.pow(x2-x1,2))
        X1,Y1,X2,Y2 = max_line[0]
        len2=math.sqrt(math.pow(Y2-Y1, 2) + math.pow(X2-X1, 2))
        if len1 > len2 :
            max_line = line

    return max_line

def dis(x1,y1,x2,y2):#得到某条由两点确定的直线上当y=480时x的值，以及当x=0时y的值，注意用的时候转化为int类型
    x = ((480-y2)/(y1-y2))*(x1-x2) + x2
    y = ((0-x2)/(x1-x2))*(y1-y2) + y2
    return x,y

def direction_only_angle(r_w,r_h):#方向的调整
    global img_debug
    org_img = camera.frame
    Board.setBusServoPulse(19, 750, 500)  
    Board.setBusServoPulse(20, 300, 500)
    turn_list = []#动作窗口，解决振荡问题
    num = 0
    while org_img is None:
        org_img = camera.frame
    while True:
        org_img = camera.frame
        AGC.runAction('stand')
        time.sleep(0.01)
        Corg_img = org_img.copy()
        img_resize = cv2.resize(Corg_img, (r_w,r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        lines = get_Cannyline(img_resize)
        if len(lines) != 0:
            x1, y1, x2, y2 = lines[0]
            num = 0
        else :
            time.sleep(0.01)
            num +=1
            if num == 4:
                AGC.runAction('sysu_turnleft')
                num = 0
            continue
        x1 = int(leMap(x1, 0, r_w, 0, 640))
        y1 = int(leMap(y1,0,r_h,0, 480))
        x2 = int(leMap(x2, 0, r_w, 0, 640))
        y2 = int(leMap(y2,0,r_h,0, 480))
       
        angle = - math.atan((y1-y2)/(x1-x2))*180.0/math.pi



        if img_debug :
            cv2.line(Corg_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.imshow('L',Corg_img)
            key = cv2.waitKey(1)
            if key == 27:
                break
            angle = round(angle)
            print("angledirection:",angle)
            print("x1,y1,x2,y2:",x1,y1,x2,y2)
        

        if angle > 34 and angle < 39:
            break
        elif angle >= 39:
            AGC.runAction('sysu_turnleft')
            turn_list.append(1)
        elif angle <= 34:
            AGC.runAction('sysu_turnright')
            turn_list.append(2)
        
        #下面解决振荡问题
        if len(turn_list) > 4:
            turn_list.pop(0)
        if len(turn_list) < 4:
            continue
        oscillation = 1
        for i in range(len(turn_list)):
            if i == 0:
                if turn_list[i] == turn_list[i+1]:
                    oscillation = 0
            elif i == len(turn_list) - 1:
                if turn_list[i-1] == turn_list[i]:
                    oscillation = 0
            else :
                if turn_list[i] == turn_list[i+1] or  turn_list[i-1] == turn_list[i]:
                    oscillation = 0
        if oscillation == 1:
            print("this is oscillation")
            break

def direction(r_w,r_h, camera):#位置的调整
    Board.setBusServoPulse(19, 750, 500)  
    Board.setBusServoPulse(20, 300, 500)
    turn_list = []#动作窗口，解决振荡问题
    num = 0
    org_img = camera.frame
    while org_img is None:
        org_img = camera.frame
    while True:
        org_img = camera.frame
        Corg_img = org_img.copy()
        img_resize = cv2.resize(org_img, (r_w,r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        lines = get_Cannyline(img_resize)
        if len(lines) != 0:
            x1, y1, x2, y2 = lines[0]
            num = 0
        else :
            time.sleep(0.01)
            num +=1
            if num == 4:
                AGC.runAction('sysu_turnleft')
                num = 0
            continue
        x1 = int(leMap(x1, 0, r_w, 0, 640))
        y1 = int(leMap(y1,0,r_h,0, 480))
        x2 = int(leMap(x2, 0, r_w, 0, 640))
        y2 = int(leMap(y2,0,r_h,0, 480))

        if x1 > 0 and y1 < 480 :
            x,y = dis(x1,y1,x2,y2)
            if x > 0 :
                x1 = int(x)
                y1 = 480
            else :
                x1 = 0
                y1 = int(y)


        angle = - math.atan((y1-y2)/(x1-x2))*180.0/math.pi



        #if img_debug :
        #    cv2.line(Corg_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #    cv2.imshow('L',Corg_img)
        #    key = cv2.waitKey(1)
        #    if key == 27:
        #        break
        #    angle = round(angle)
        #    print("angle:",angle)
        #    print("x1,y1,x2,y2:",x1,y1,x2,y2)
        

        # if x1 > 0 :
        #     AGC.runAction('sysu_left_move')
        #     turn_list.append(1)
        if  y1 >= 485:
            AGC.runAction('sysu_right_move')
            turn_list.append(1)
        elif y1 <= 420:
            AGC.runAction('sysu_left_move')
            turn_list.append(2)
        else :
            break


        #下面解决振荡问题
        if len(turn_list) > 4:
            turn_list.pop(0)
        if len(turn_list) < 4:
            continue
        oscillation = 1
        for i in range(len(turn_list)):
            if i == 0:
                if turn_list[i] == turn_list[i+1]:
                    oscillation = 0
            elif i == len(turn_list) - 1:
                if turn_list[i-1] == turn_list[i]:
                    oscillation = 0
            else :
                if turn_list[i] == turn_list[i+1] or  turn_list[i-1] == turn_list[i]:
                    oscillation = 0
        if oscillation == 1:
            print("this is oscillation")
            break


#################stair###################
def cross_stair():
    #global org_img, rs_img, checkpoint, state_sel
    AGC.runAction("stand")
    camera = Camera.Camera(param_data_path = '/home/pi/CameraCalibration/calibration_param.npz')
    camera.camera_open()
    Board.setBusServoPulse(20, 320, 500)     #调整云台
    Board.setBusServoPulse(19, 500, 500)
    time.sleep(0.1)
    state_sel = "stair"                      #合并后通过选关确定
    step = 1                                 #初始化step状态，step用于状态转移
    left_cnt = 0
    right_cnt = 0
    org_img = camera.frame
    while org_img is None:
        org_img = camera.frame               #获得图像
    while state_sel == "stair":
        org_img = camera.frame
        #print(org_img)
        rs_img = cv2.resize(org_img, (rs_img_w, rs_img_h), interpolation=cv2.INTER_CUBIC)       #缩放图像，便于处理
        LAB_img = cv2.cvtColor(rs_img, cv2.COLOR_BGR2LAB)
        LAB_img_Gauss = cv2.GaussianBlur(LAB_img, (3,3), 0)
        blue_mask = cv2.inRange(LAB_img_Gauss, lwx_color_range['blue'][0], lwx_color_range['blue'][1])      #获得蓝色掩膜
        blue_mask = cv2.erode(blue_mask, None, iterations = 1)              #腐蚀操作，去除噪点
        blue_mask = cv2.dilate(blue_mask, np.ones((3,3), np.uint8), iterations = 2)     #膨胀，连接轮廓
        blue_contours = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]           #获得蓝色轮廓
        if len(blue_contours) != 0:
            print('find blue stair')
            #blue_contours_sum = getAreaSumContour(blue_contours)                #获得蓝色面积
            blue_max_contour, blue_contours_area = getAreaMaxContour(blue_contours)              #获得蓝色最大轮廓
            rect_left_top_x, rect_left_top_y, rect_w, rect_h = cv2.boundingRect(blue_max_contour)       #获得蓝色最大轮廓的左上角点坐标&宽搞（bound方法）
            rect = cv2.minAreaRect(blue_max_contour)            #蓝色贴合轮廓
            if step == 1:
                print('调整角度')
                angle = - rect[2]
                h1 = rect[1][1]
                w1 = rect[1][0]
                while True:
                    percent = round(100 * blue_contours_area / (rect[1][0] * rect[1][1]), 2)
                    if w1 > h1 and angle < 50 and angle > 5:
                        AGC.runAction('sysu_turnleft')
                        break
                    elif h1 > w1 and angle > 50 and angle < 85:
                        AGC.runAction('sysu_turnright')                    
                        break
                    elif angle <= 5 or angle >= 85:
                        if rect_left_top_x + (rect_w / 2) <= 120 and percent < 80:       #蓝色轮廓在左上角修正
                            AGC.runAction('sysu_turnleft')
                            break
                        elif rect_left_top_x + (rect_w / 2) >= 180 and percent < 80:     #蓝色轮廓在右上角修正
                            AGC.runAction('sysu_turnright')                    
                            break
                        else:
                            step = 2
                            break
            elif step == 2:
                print('修正位置')
                while True:
                    if left_cnt > 5 or right_cnt > 5:
                        left_cnt = 0
                        right_cnt = 0
                        step = 1
                        break
                    #if percent < 80:        #如果矩阵在左右上角斜着但被minAreaRect误认为已矫正修补条件
                    #    step = 1
                    #    break
                    if rect_left_top_x + (rect_w / 2) < 160:
                        left_cnt += 1
                        AGC.runAction('sysu_left_move')
                        break
                    if rect_left_top_x + (rect_w / 2) > 175:
                        right_cnt += 1
                        AGC.runAction("sysu_right_move")
                        break
                    elif rect_left_top_x + (rect_w / 2) >= 160 and rect_left_top_x + (rect_w / 2) <= 175:
                        step = 3
                        break
            elif step == 3:
                print('继续靠近台阶')
                while True:
                    if rect_left_top_y + (rect_h / 2) < 165:    #小于170则认为离台阶较远，快速接近
                        #for i in range(0,4):
                        AGC.runAction('sysu_speed1', 2)
                        AGC.runAction('stand')
                        time.sleep(0.1)
                        step = 1
                        break
                    else:
                        step = 4
                        break
            elif step == 4:
                if rect_left_top_y + (rect_h / 2) >= 192:
                    print('上台阶')
                    #for i in range(0,4):
                    AGC.runAction('sysu_slow_move2', 1)
                    time.sleep(0.5)
                    AGC.runAction('sysu_up_stair')
                    #for i in range(0,3):
                    #AGC.runAction('sysu_slow_move', 2)
                    time.sleep(0.5)
                    AGC.runAction('sysu_up_stair')
                    #for i in range(0,3):
                    #AGC.runAction('sysu_slow_move', 2)
                    time.sleep(0.5)
                    AGC.runAction('sysu_up_stair')
                    ######
                    ####调整云台
                    '''在楼梯上不再作调整
                    PWMServo.setServo()
                    border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                   value=(255, 255, 255))  # 扩展黑边，防止边界无法识别
                    org_img_resize = cv2.resize(border, (rs_img_w, rs_img_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
                    frame_gauss = cv2.GaussianBlur(org_img_resize, (3, 3), 0)  # 高斯模糊
                    frame_lab = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2LAB)  # 将图片转换到LAB空间
                    mark_red_stairs = cv2.inRange(frame_lab, lwx_color_range['red_stair'][0], lwx_color_range['red_stair'][1])  # 红色掩模
                    red_opened = cv2.morphologyEx(mark_red_stairs, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
                    red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
                    red_contours = cv2.findContours(red_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                    contour_max_red, area_max_red = getAreaMaxContour(red_contours)  # 求最大轮廓和最大轮廓面积
                    x1, y1, w, h = cv2.boundingRect(contour_max_red)
                    rect1 = cv2.minAreaRect(contour_max_red)
                    angle = rect1[2]
                    h1 = rect1[1][1]
                    w1 = rect1[1][0]
                    while w1 > h1 and angle < 50 and angle > 5:
                        AGC.runAction(turn_right)
                        if angle < 5 or angle > 85:
                            break
                    while h1 > w1 and angle > 50 and angle < 85:
                        AGC.runAction(turn_left)
                        if angle < 5 or angle > 85:
                            break
                    while x1 + (w / 2) < 130:
                        AGC.runAction(left_shift_big)
                        if (x1 + (w / 2) >= 130) and (x1 + (w / 2) <= 160):
                            break
                    while x1 + (w / 2) > 160:
                        AGC.runAction(right_shift_big)
                        if (x1 + (w / 2) >= 130) and (x1 + (w / 2) <= 160):
                            break
                    '''
                    ###### 
                    AGC.runAction('sysu_go_back', 2)
                    AGC.runAction('stand')
                    #time.sleep(0.5)
                    #AGC.runAction('sysu_go_back')
                    #AGC.runAction('stand')
                    time.sleep(0.5)
                    AGC.runAction('sysu_turnleft', 2)
                    AGC.runAction('stand')
                    print('下台阶')
                    AGC.runAction('sysu_down_stair5')
                    time.sleep(0.5)
                    AGC.runAction('sysu_down_stair5')
                    time.sleep(0.5)
                    AGC.runAction('sysu_left_move', 3)
                    AGC.runAction('sysu_go_forward_start')
                    AGC.runAction('sysu_go_forward', 2)
                    AGC.runAction('sysu_go_forward_end')
                    AGC.runAction('sysu_slow_move1', 9)
                    AGC.runAction('sysu_slow_move', 5)
                    state_sel = 'checkpoint recognize'
                    break
                else:
                    print('继续靠近台阶')                   #170  < dis < 195 小步靠近台阶
                    AGC.runAction('sysu_slow_move2', 3)
                    step = 1
        #else:
        #    print('blue stair not found')
    #direction_only_angle(320, 240)
    # AGC.runAction('sysu_turnleft', 3)
    # AGC.runAction('sysu_left_move', 3)
    AGC.runAction('sysu_turnright', 3)
    camera.camera_close()
    stop_thread(camera.th)
    threading.Thread.join(camera.th)
    #print(state_sel)
    print("done")
    return 
    #camera.th.join()




#######################final gate##########################
def final_gate():
    #global org_img, state, reset, debug
    checkpoint = 9
    if checkpoint == 9:
        step = 0
        
        Board.setBusServoPulse(20, 420, 500)
        Board.setBusServoPulse(19, 500, 500)
        
        AGC.runAction('stand')
        time.sleep(0.01)
    else:
        return
    camera = Camera.Camera(param_data_path = '/home/pi/CameraCalibration/calibration_param.npz')
    camera.camera_open()
    direction(320, 240, camera)
    Board.setBusServoPulse(20, 420, 500)
    Board.setBusServoPulse(19, 500, 500)
    AGC.runAction('sysu_speed1', 4)             #先靠近，再做识别
    org_img = camera.frame
    while org_img is None:
        org_img = camera.frame
    while checkpoint == 9:
        org_img = camera.frame                  #获取图像
        border = cv2.copyMakeBorder(org_img, 12,12,16,16,borderType = cv2.BORDER_CONSTANT, value = (255,255,255))
        org_img_resize = cv2.resize(border, (rs_img_w, rs_img_h), interpolation = cv2.INTER_CUBIC)
        gauss_img = cv2.GaussianBlur(org_img_resize, (3,3), 0)
        img_lab = cv2.cvtColor(gauss_img, cv2.COLOR_BGR2LAB)
        yellow_mask = cv2.inRange(img_lab, lwx_color_range['yellow'][0], lwx_color_range['yellow'][1])
        black_mask = cv2.inRange(img_lab, lwx_color_range['black'][0], lwx_color_range['black'][1])
        full_mask = cv2.add(yellow_mask, black_mask)                #通过黄色和黑色掩膜的叠加，得到完整掩膜
        opened_img = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))      #开运算，去除噪声
        closed_img = cv2.morphologyEx(opened_img, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))    #闭运算，闭合轮廓

        contours = cv2.findContours(closed_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour, area_sum = getAreaMaxContour(contours)
        area_sum = getAreaSumContour(contours)              #算出总体挡板面积
        percent = round(100 * area_sum / (rs_img_w * rs_img_h), 2)      #计算挡板在图像中的面积

        if step == 0:
            if  2 * percent >= 15:          #面积参数还得调
                print('挡板放下')
                step = 1
            else:
                step = 0
        elif step == 1:
            if 2 * percent <= 12:           #面积参数调整
                #for i in range(0, 8):
                print('挡板升起')
                AGC.runAction('sysu_speed1', 8)
                # AGC.runAction('sysu_turnleft', 3)
                # AGC.runAction('sysu_left_move', 3)      #快速通过
                # step = 0
        else:
            time.sleep(0.01)
            
###########random##############
'''
def random_checkpoint_recognize():
    global org_img, rs_img, checkpoint
    
    Board.setBusServoPulse(20, 400, 500)
    Board.setBusServoPulse(19, 500, 500)
    
    if checkpoint == 1:
        return "start door"
    find_red = False
    find_green = False
    find_blue = False
    find_ball = False
    lab_img = cv2.cvtColor(rs_img, cv2.COLOR_BGR2LAB)
    Gauss_img = cv2.GaussianBlur(lab_img, (3,3), 0)

    green_mask = cv2.inRange(Gauss_img, lwx_color_range['green'][0], lwx_color_range['green'][1])
    blue_mask = cv2.inRange(Gauss_img, lwx_color_range['blue'][0], lwx_color_range['blue'][1])
    red_mask = cv2.inRange(Gauss_img, lwx_color_range['red'][0], lwx_color_range['red'][1])
    white_mask = cv2.inRange(Gauss_img, lwx_color_range['white'][0], lwx_color_range['white'][1])
    black_mask = cv2.inRange(Gauss_img,lwx_color_range['black'][0], lwx_color_range['black'][1])

    green_open = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    green_closed = cv2.morphologyEx(green_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    blue_open = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    blue_closed = cv2.morphologyEx(blue_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    red_open = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    red_closed = cv2.morphologyEx(red_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    white_open = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    white_closed = cv2.morphologyEx(white_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    black_open = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    black_closed = cv2.morphologyEx(black_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))

    green_contours = cv2.findContours(green_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    blue_contours = cv2.findContours(blue_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    red_contours = cv2.findContours(red_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    black_conntours = cv2.findContours(black_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

    green_max_contour, green_area = getAreaMaxContour(green_contours)
    blue_max_contour, blue_area= getAreaMaxContour(blue_contours)
    red_max_contour, red_area = getAreaMaxContour(red_contours)
    
    green_percent = round(100 * green_area / (rs_img_w * rs_img_h), 2)
    blue_percent = round(100 * blue_area / (rs_img_w * rs_img_h), 2)
    red_percent = round(100 * red_area / (rs_img_w * rs_img_h), 2)

    edge_detect = cv2.Canny(white_closed, 100, 300)
    circles = cv2.HoughCircles(edge_detect, cv2.HOUGH_GRADIENT, 1, int(max(white_closed.shape[0],white_closed.shape[1])/3), param1=50, param2=10, minRadius=10, maxRadius=50)
    r = None
    x = None
    y = None
    if not circles is None:
        circles = np.uint16(np.around(circles))
        r, x, y = circles[0, 0, 2], circles[0, 0, 0], circles[0, 0, 1]

    if r >= 10:
        find_ball = True
    if green_percent > 5:
        find_green = True
    if blue_percent > 5:
        find_blue = True
    if red_percent > 3:
        find_red = True
    #台阶判断
    if find_green == True and find_blue == True and find_red == True:
        return "stair"
    #独木桥&回型坑判定
    if find_red == False and find_blue == False and find_green == True:
        line = line_len(green_max_contour)
        if line > 200:
            return "hole"
        else:
            return "bridge"
    elif find_red == True and find_blue == False and find_green == True:
        if green_percent >  blue_percent:
            line = line_len(green_max_contour)
            if line > 200:
                return "hole"
            else:
                return "bridge"
    elif find_red == True and find_blue == True and find_green == False:
        line = line_len(blue_max_contour)
        if line > 200:
            return "hole"
        else:
            return "bridge"
    #判断独木桥or回型坑or蓝色挡板or蓝色门
    elif find_red == False and find_blue == True and find_green == True:
        if green_percent >  blue_percent:
            line = line_len(green_max_contour)
            if line > 200:
                return "hole"
            else:
                return "bridge"  
        else:
            if blue_percent >  2 * green_percent:    
                line = line_len(blue_max_contour)
                if line > 200:
                    return "hole"
                else:
                    return "bridge"
            elif len(blue_contours) >= 2:
                blue_rect = cv2.minAreaRect(blue_max_contour)
                angle = - blue_rect[2]
                blue_w = blue_rect[1][0]
                blue_h = blue_rect[1][1]
                if blue_w > blue_h and angle > 15:
                    return "baffle"
                elif blue_h > blue_w and angle < 75:
                    return "baffle"
                elif blue_w < blue_h and angle > 15:
                    return "door"
                elif blue_w > blue_h and angle < 75:
                    return "door"
    #判断蓝色挡板or蓝色门
    elif find_red == False and find_blue == True and find_green == False:
        if blue_percent > 20:
            line = line_len(blue_max_contour)
            if line > 200:
                return "hole"
            else:
                return "bridge"
        elif len(blue_contours) >= 2:
            blue_rect = cv2.minAreaRect(blue_max_contour)
            angle = - blue_rect[2]
            blue_w = blue_rect[1][0]
            blue_h = blue_rect[1][1]
            if blue_w > blue_h and angle > 15:
                return "baffle"
            elif blue_h > blue_w and angle < 75:
                return "baffle"
            elif blue_w < blue_h and angle > 15:
                return "door"
            elif blue_w > blue_h and angle < 75:
                return "door"
    else:
        #判断雷区
        if len(black_conntours) >= 2:
            return "Minefield"
        elif  find_ball == True:
            return "kick ball"
        elif checkpoint == 9:
            return "final gate"
'''


def checkpoint_recognize(checkpoint):
    #global checkpoint
    
    Board.setBusServoPulse(20, 350, 500)
    Board.setBusServoPulse(19, 500, 500)
    
    if checkpoint == 1:
        return 'RULER'
    if checkpoint == 3: #and len(blue_contours) > 1 and find_blue:
        return "MINE"
    if checkpoint == 4: #and len(blue_contours) > 1 and find_blue:
        return "DOOR"
    if checkpoint == 6: #and find_ball == True:
        return "BALL"
    if checkpoint == 8:
        return "GATE"
    if checkpoint == 9:
        return "END"
    find_red = False
    find_green = False
    find_blue = False
    find_ball = False
    camera = Camera.Camera(param_data_path = '/home/pi/CameraCalibration/calibration_param.npz')
    camera.camera_open()
    org_img = camera.frame
    while org_img is None:
        org_img = camera.frame
    rs_img = cv2.resize(org_img, (320,240), interpolation=cv2.INTER_CUBIC)
    rs_img = rs_img[0:150, :]                   #截取图像，防止干扰
    lab_img = cv2.cvtColor(rs_img, cv2.COLOR_BGR2LAB)
    Gauss_img = cv2.GaussianBlur(lab_img, (3,3), 0)
    
    #图像处理，获取各类掩膜
    green_mask = cv2.inRange(Gauss_img, lwx_color_range['green'][0], lwx_color_range['green'][1])
    blue_mask = cv2.inRange(Gauss_img, lwx_color_range['blue'][0], lwx_color_range['blue'][1])
    red_mask = cv2.inRange(Gauss_img, lwx_color_range['red'][0], lwx_color_range['red'][1])
    white_mask = cv2.inRange(Gauss_img, lwx_color_range['white'][0], lwx_color_range['white'][1])
    black_mask = cv2.inRange(Gauss_img, lwx_color_range['black'][0], lwx_color_range['black'][1])
    
    #图像处理，去除噪声，闭合轮廓
    green_open = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    green_closed = cv2.morphologyEx(green_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    blue_open = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    blue_closed = cv2.morphologyEx(blue_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    red_open = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    red_closed = cv2.morphologyEx(red_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    white_open = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    white_closed = cv2.morphologyEx(white_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
    black_open = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
    black_closed = cv2.morphologyEx(black_open, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))

    #找出各色轮廓
    green_contours = cv2.findContours(green_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    blue_contours = cv2.findContours(blue_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    red_contours = cv2.findContours(red_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    black_contours = cv2.findContours(black_closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    
    #找到红绿蓝最大轮廓，最大轮廓对应面积
    green_max_contour, green_area = getAreaMaxContour(green_contours)
    blue_max_contour, blue_area= getAreaMaxContour(blue_contours)
    red_max_contour, red_area = getAreaMaxContour(red_contours)
    
    #计算红绿蓝最大轮廓面积在画面中的占比
    green_percent = round(100 * green_area / (rs_img_w * rs_img_h), 2)
    blue_percent = round(100 * blue_area / (rs_img_w * rs_img_h), 2)
    red_percent = round(100 * red_area / (rs_img_w * rs_img_h), 2)

    #检测小球
    edge_detect = cv2.Canny(white_closed, 100, 300)
    circles = cv2.HoughCircles(edge_detect, cv2.HOUGH_GRADIENT, 1, int(max(white_closed.shape[0],white_closed.shape[1])/3), param1=50, param2=10, minRadius=10, maxRadius=50)
    r = None
    x = None
    y = None
    if not circles is None:
        circles = np.uint16(np.around(circles))
        r, x, y = circles[0, 0, 2], circles[0, 0, 0], circles[0, 0, 1]

    #小球发现条件
    if r is not None and r >= 10:
        find_ball = True
    
    if green_percent > 6:
        find_green = True
    if blue_percent > 6:
        find_blue = True
    if red_percent > 3:
        find_red = True
    
    print(green_percent)
    print(blue_percent)
    print(red_percent)
    
    #台阶判定
    #独木桥&回型坑判定
    if (find_red == False and find_blue == False and find_green == True) or (find_red == True and find_blue == False and find_green == True):
        line = line_len(green_max_contour)
        print(line)
        if line > 200:
            camera.camera_close()
            stop_thread(camera.th)
            threading.Thread.join(camera.th)
            return 'HOLE'
        else:
            camera.camera_close()
            stop_thread(camera.th)
            threading.Thread.join(camera.th)
            return 'BRIDGE'
    #独木桥&回型坑判定
    if (find_red == False and find_blue == True and find_green == False) or (find_red == True and find_blue == True and find_green == False):
        line = line_len(blue_max_contour)
        print(line)
        if line > 200:
            camera.camera_close()
            stop_thread(camera.th)
            threading.Thread.join(camera.th)
            return "HOLE"
        else:
            camera.camera_close()
            stop_thread(camera.th)
            camera.th.join()
            return "BRIDGE"
    if find_green == True and find_blue == True and find_red == True:
        camera.camera_close()
        stop_thread(camera.th)
        threading.Thread.join(camera.th)    
        return 'STAIR'
    # else:
    #     #黑色轮廓大于7个判断为雷区
    #     if len(black_contours) >= 7:
    #         camera.camera_close()
    #         stop_thread(camera.th)
    #         threading.Thread.join(camera.th)
    #         return "minefield"
        #elif checkpoint == 4 and len(blue_contours) > 1 and find_blue == True:
        #    return "baffle"
        #elif checkpoint == 5 and len(blue_contours) > 1 and find_blue == True:
        #    return "cross gate"
        #elif checkpoint == 7 and find_ball == True:
        #    return "kick ball"
        #elif checkpoint == 9:
        #    return "final gate"
        
#cross_stair()
#final_gate()
#while True:
#    state = checkpoint_recognize()
#    print(state)