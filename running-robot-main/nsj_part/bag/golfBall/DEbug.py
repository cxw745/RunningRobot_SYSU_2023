import cv2
import numpy as np
import time
import math
from hiwonder import Board
import hiwonder.ActionGroupControl as AGC
import golfBall.colorRangeNsj as nsj
import golfBall.nsj_processImg as PIG

head_height = 280
#################################展示图像，用来DEbug###########################
def show(img):
    if img is not None:
        cv2.imshow("img",img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
##############################################################################

#################################执行动作##################################
'''
说明flag = 0时使用手机ip摄像头测试
flag = 1是机器人测试
flag = -1时是比赛测试，不做输出
'''
###################摄像头虚拟动作##############################################
def see_forward_floor(flag = 0):
    '''
    看正前方地板
    '''
    if flag == 0:
        input("摄像头看正前方")
    elif flag == 1:
        print("摄像头看正前方")
        Board.setBusServoPulse(19, 500, 500)
    elif flag == -1:
        Board.setBusServoPulse(19, 500, 500)
    time.sleep(1)
    return 
def see_right_floor(flag = 0):
    '''
    看右前方地板
    '''
    if flag == 0:
        input("摄像头看右前方")
    elif flag == 1:
        print("摄像头看右前方")
        Board.setBusServoPulse(19, 290, 500)
    elif flag == -1:
        Board.setBusServoPulse(19, 290, 500)
    time.sleep(1)
    return 
def see_left_floor(flag = 0):
    '''
    看左前方地板
    '''
    if flag == 0:
        input("摄像头看左前方")
    elif flag == 1:
        print("摄像头看左前方")
        Board.setBusServoPulse(19, 710, 500)
    elif flag == -1:
        Board.setBusServoPulse(19, 710, 500)
    time.sleep(1)
    return 
#########################初始化摄像头##########################################
def init_camera_angle(flag):
    if flag == 0:
        input("初始化摄像头角度")
    elif flag == 1:
        print("初始化摄像头角度")
        Board.setBusServoPulse(19, 500, 500)
        Board.setBusServoPulse(20, head_height, 500)
    elif flag == -1:
        Board.setBusServoPulse(19, 500, 500)
        Board.setBusServoPulse(20, head_height, 500)
#############################右转向############################################
def turn_right(flag,n = 1):
    if flag == 0:
        input("右转向一次")
    elif flag == 1:
        print("右转向一次")
        AGC.runAction("sysu_turnright",n)
    elif flag == -1:
        AGC.runAction("sysu_turnright",n)
    time.sleep(0.2)
#############################左转向############################################
def turn_left(flag,n = 1):
    if flag == 0:
        input("左转向一次")
    elif flag == 1:
        print("左转向一次")
        AGC.runAction("sysu_turnleft",n)
    elif flag == -1:
        AGC.runAction("sysu_turnleft",n)
    time.sleep(0.2)
def back_up(flag,n = 1):
    if flag == 0:
        input("sysu_go_back")
    elif flag == 1:
        print("sysu_go_back")
        AGC.runAction("sysu_go_back2",n)
    elif flag == -1:
        AGC.runAction("sysu_go_back2",n)
    time.sleep(1)
#############################左平移############################################
def left_move(flag,n = 1):
    if flag == 0:
        input("左平移一次")
    elif flag == 1:
        print("左平移一次")
        AGC.runAction("sysu_left_move",n)
    elif flag == -1:
        AGC.runAction("sysu_left_move",n)
#############################右平移############################################
def right_move(flag,n = 1):
    if flag == 0:
        input("右平移一次")
    elif flag == 1:
        print("右平移一次")
        AGC.runAction("sysu_right_move",n)
    elif flag == -1:
        AGC.runAction("sysu_right_move",n)
#############################直走##############################################
def go_forward(flag,n = 1):
    AGC.runActionGroup('sysu_go_forward_start')
    if flag == 0:
        input("直走")
    elif flag == 1:
        print("直走")
        AGC.runActionGroup('sysu_go_forward', n)
    elif flag == -1:
        AGC.runActionGroup('sysu_go_forward', n)
    AGC.runActionGroup('sysu_go_forward_end')

#############################踢球##############################################
def kick_ball(flag):
    if flag == 0:
        input("踢球")
        
    elif flag == 1:
        print("踢球")
        AGC.runAction("left_shot",1)
    elif flag == -1:
        AGC.runAction("left_shot",1)    
###############################################################################
#############################slow_move##############################################
def go_slow(flag,n = 1):
    if flag == 0:
        input("slow_move")
    elif flag == 1:
        print("slow_move")
        AGC.runAction("sysu_slow_move",n)
    elif flag == -1:
        AGC.runAction("sysu_slow_move",n)
#############################踢球##############################################

def get_hole(img,LAB_img,dis):
    #转化成二值图
    Imask = cv2.inRange(LAB_img,nsj.color_range['hole'][0],nsj.color_range['hole'][1])
    #得到最大轮廓
    cnts = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts_max,cnts1_area = PIG.getAreaMax_contour(cnts,50)
    
    #检测
    if cnts_max is not None:
        Hole = True
        (circle_x1, circle_y1), radius1 = cv2.minEnclosingCircle(cnts_max)
        center1 = (int(circle_x1), int(circle_y1))
        radius1 = int(radius1)
        hole_dis_x,hole_dis_y = center1[0],center1[1]
        cv2.circle(img, center1, radius1, (0, 255, 255), 2)
        cv2.line(img, dis,center1, (0, 255, 255), 2)
        if(center1[0]==300):
            Hole_angle_robot = 90
        else:
            Hole_angle_robot = math.atan((center1[1])/(center1[0]-300))*180/math.pi
        cv2.putText(img,"hole_angle_robot = %f" % Hole_angle_robot,(30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0),2)
        cv2.putText(img,"(%d,%d)" % (hole_dis_x,hole_dis_y),(30,250),cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0),2)
    return img

def get_ball(img,LAB_img):
    golf_dis_x = 0
    golf_dis_y = 0
    robot_left_feet_x = 300
    ##待定:左脚延伸值
    img_h, img_w = img.shape[:2]
    bottom_center = (robot_left_feet_x, int(img_h))
    ## 得到红色区域掩码
    pxt = cv2.inRange(LAB_img,nsj.color_range['red_floor'][0],nsj.color_range['red_floor'][1])
    ## 进行开闭运算去除白色条纹
    pxt = cv2.dilate(pxt, np.ones((3, 3), np.uint8), iterations=8)
    Imask = cv2.erode(pxt, None, iterations=8)
    # 进行边界填充，适应于泛洪填充,(防止边界出现白色，而导致其他地方被当成孔洞填充成白色)
    Imask = cv2.copyMakeBorder(Imask, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value = 0)
    Imask[img_h+20:img_h+40,0:img_w+40] = 255
    # 孔洞填充，把白色小球填充出来
    Imask = PIG.fillHole(Imask,0,0)
    ## 截取图像，去除填充
    Imask = Imask[20:img_h+20,20:20+img_w]
    ## 进行与运算，得到掩膜后的图像
    img=cv2.add(img, np.zeros(np.shape(img), dtype=np.uint8), mask=Imask)

    img,LAB_img = PIG.process_img(img)
    #得到白色色块的二值图
    Imask = cv2.inRange(LAB_img,nsj.color_range['golf_ball'][0],nsj.color_range['golf_ball'][1])
    Imask = cv2.erode(Imask, None, iterations=10)
    Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=10)
    #cnts是所有轮廓
    cnts2= cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    #得到最大轮廓
    cnts_max2,cnts2_area = PIG.getAreaMax_contour(cnts2,10)
    # Imask = cv2.copyMakeBorder(Imask, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value = (255,255,255))
    #检测
    if cnts_max2 is not None:  
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.medianBlur(gray_img, 5)

        # 与直线检测类似，需要圆心距的最小距离和圆的最小以及最大半径
        circles = cv2.HoughCircles(gray_img,cv2.HOUGH_GRADIENT,1,10,param1=100,param2=30,minRadius=0,maxRadius=200)
        if circles is not None:
            circles = np.uint16(np.around(circles))

            for i in circles[0,:]:
                if i[1] < img_h and i[0] < img_w and  Imask[int(i[1])][int(i[0])] == 255 :
                    cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
                    cv2.line(img, (i[0],i[1]), bottom_center, (0, 255, 255), 2)
                    center1 = (int(i[0]), int(i[1]))
                    golf_dis_x,golf_dis_y = center1[0],center1[1]
                    if(center1[0] == bottom_center[0]):
                        golf_angle_robot = 90
                    else:
                        golf_angle_robot = - math.atan(
                            (center1[1] - bottom_center[1]) / (center1[0] - bottom_center[0])) * 180.0 / math.pi
                        ###使用计算脚和球角度
                    cv2.putText(img,"golf_angle_robot = %f" % golf_angle_robot,(30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0),2)
                    cv2.putText(img,"(%d,%d)" % (golf_dis_x,golf_dis_y),(30,150),cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0),2)
                    break
    return img,(golf_dis_x,golf_dis_y)
