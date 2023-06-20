


import cv2
import numpy as np
import time
import threading
import math

import golfBall.Camera   as Camera
import golfBall.colorRangeNsj as nsj
import golfBall.nsj_processImg as PIG
import golfBall.DEbug as DEbug


class nsj_kick:
    def __init__(self):


        self.FLAG_flag = 1 #调试模式
        '''
        说明flag = 0时使用手机ip摄像头测试模式
        flag = 1是机器人测试模式
        flag = -1时是比赛模式，不做输出
        '''


        ##################################全局变量#########################################################################################################
        if self.FLAG_flag == 0:
            url = "rtsp://192.168.137.20:8554/live"
            self.my_camera =  cv2.VideoCapture(url)
        else:
            param_data_path = '/home/pi/CameraCalibration/calibration_param.npz'
            self.my_camera = Camera.Camera(param_data_path)
            self.my_camera.camera_open()
            print('摄像头原始画面，已做畸变校正')
        time.sleep(2)
        ################
        #高尔夫球坐标
        self.img_weight = 240
        self.img_height = 320


        self.golf_dis_x = 0 
        self.golf_dis_y = 0
        #################
        #球洞坐标
        self.hole_dis_x = 0
        self.hole_dis_y = 0
        #################
        #机器人左脚动作
        self.robot_left_feet_x = 130
        ###############
        self.golf_ball = False #判断是否寻找到高尔夫球
        self.Hole = False #判断是否寻找到洞口
        #################
        #与球相关的变量
        self.golf_angle_robot = 0 #球与机器人角度，最佳射球区间为(-90,-80) (80,90)
        self.best_robot_golf = False #小球与机器人左脚垂直
        self.golf_in_hole = False #小球是否进洞
        #################
        #与洞相关的变量
        self.Hole_angle_golf = 0 #洞与小球角度
        self.Hole_angle_robot = 0 #洞与机器人角度
        self.best_hole_robot = False #洞口与机器人左脚垂直
        #################
        #最佳射球区间为(87,90s)
        self.best_golf_hole = False 
        #最佳滚球角度已就位
        self.left_x = 0 #距离左边边界距离
        self.right_x = 0 #距离右边边界距离
        # 对正赛道角度
        self.best_angle = 60
        # 靠右距离
        self.standar_right_dt = 175
        if self.FLAG_flag == 1:
            th2 = threading.Thread(target=self.show_video)
            th2.setDaemon(True)     # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
            th2.start()

        ##################################################################################################################################################





    ################################DEbug############################################################################################################
    def show_video(self):
        time.sleep(3)
        while True:
            if self.my_camera.frame is not None:
                img = cv2.resize(self.my_camera.frame,(320,240))
                img,LAB_img = PIG.process_img(img)
                img,dis = DEbug.get_ball(img,LAB_img)
                img = DEbug.get_hole(img,LAB_img,dis)
                cv2.imshow("img",img)
                if cv2.waitKey(10)== ord('q'):
                    break
            else:
                print("找不到文件")


    
    #################################################################################################################################################




    ###############################################################图像处理函数##########################################################################

    ###############识别小球函数###############################
    def check_ball(self,LAB_img,img,flag = True):
        '''
        :@LAB_img 图像的LAB模式
        :@img 初始图像
        :直接使用掩码的到红色范围，然后边界填充加上泛洪水填充得到红色地板所有信息，
        然后根据色块得到白色色块的位置,然后根据霍夫圆检测来识别圆形，然后确定圆形圆心是否为白色，若为白色则是小球
        '''
        ##待定:左脚延伸值
        img_h, img_w = img.shape[:2]
        bottom_center = (self.robot_left_feet_x, int(img_h))
        ## 得到红色区域掩码
        pxt = cv2.inRange(LAB_img,nsj.color_range['red_floor'][0],nsj.color_range['red_floor'][1])
        ## 进行开闭运算去除白色条纹
        pxt = cv2.dilate(pxt, np.ones((3, 3), np.uint8), iterations=5)
        Imask = cv2.erode(pxt, None, iterations=5)
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
        Imask = cv2.erode(Imask, None, iterations=5)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=5)
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
                        self.golf_ball = True
                        center1 = (int(i[0]), int(i[1]))
                        radius1 = int(i[2])
                        self.golf_dis_x,self.golf_dis_y = center1[0],center1[1]
                        if(center1[0] == bottom_center[0]):
                            self.golf_angle_robot = 90
                        else:
                            self.golf_angle_robot = - math.atan(
                                (center1[1] - bottom_center[1]) / (center1[0] - bottom_center[0])) * 180.0 / math.pi
                        break
        else:
            self.golf_ball =  False
            self.golf_angle_robot = 0
            self.Hole_angle_golf = 0
        if( 80<= self.golf_angle_robot <= 90 or -90 <= self.golf_angle_robot <= -80) and flag:
            self.best_robot_golf = True
        else:
            self.best_robot_golf = False
        return img
    #########################################################

    ###############识别hole函数##############################
    def check_hole(self,LAB_img,img):
        '''
        :@LAB_img 图像的LAB模式
        :@img 初始图像
        '''
        img_h, img_w = img.shape[:2]
        bottom_center = (self.robot_left_feet_x, int(img_h))
        #转化成二值图
        Imask = cv2.inRange(LAB_img,nsj.color_range['hole'][0],nsj.color_range['hole'][1])
        #得到最大轮廓
        cnts = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts_max,cnts1_area = PIG.getAreaMax_contour(cnts,10)
        
        #检测
        if cnts_max is not None:
            self.Hole = True
            (circle_x1, circle_y1), radius1 = cv2.minEnclosingCircle(cnts_max)
            center1 = (int(circle_x1), int(circle_y1))
            radius1 = int(radius1)
            self.hole_dis_x,self.hole_dis_y = center1[0],center1[1]
            cv2.circle(img, center1, radius1, (0, 255, 255), 2)
            cv2.line(img, (self.golf_dis_x,self.golf_dis_y),center1, (0, 255, 255), 2)
            if(center1[0]==self.robot_left_feet_x):
                self.Hole_angle_robot = 90
            else:
                self.Hole_angle_robot = -math.atan((center1[1]-bottom_center[1])/(center1[0]-bottom_center[0]))*180/math.pi
            # cv2.putText(img,"hole_angle_robot = %f" % self.Hole_angle_robot,(30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0),2)
            # cv2.putText(img,"(%d,%d)" % (self.hole_dis_x,self.hole_dis_y),(30,250),cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0),2)
        else:
            self.best_hole_robot = 0
            self.Hole =  False
        if(85 <= self.Hole_angle_robot<=90 or -90<= self.Hole_angle_robot <= -85):
            self.best_hole_robot = True
        else:
            self.best_hole_robot = False
        return img
    #########################################################

    ####################################################################################################################################################

    ################################################################动作执行函数########################################################################

    ###############转动云台，寻到目标(小球或球洞)############
    def find(self,object = "ball",flag = True):
        '''
        默认为小球
        找到物体后要面向该物体：即物体与左脚垂直,即附带转向
        '''
        if  flag == True:
            DEbug.init_camera_angle(self.FLAG_flag)
        for i in range(3):
            temp = 0 #物体在左边为-1，右边为1，中间为0
            #依次正前方，左前方，右前方
            if flag == True:
                time.sleep(0.1)
                if(i==0):
                    #检测正前方
                    DEbug.see_forward_floor(self.FLAG_flag)
                    time.sleep(0.5)
                elif(i==1):
                    #检测左前方
                    DEbug.see_left_floor(self.FLAG_flag)
                    time.sleep(0.5)
                elif(i==2):
                    #检测右前方
                    DEbug.see_right_floor(self.FLAG_flag)
                    time.sleep(0.5)
            if object == "ball":
                img = cv2.resize(self.my_camera.frame,(320,240))
                img,LAB_img = PIG.process_img(img)
                self.check_ball(LAB_img,img,False)
                if(self.golf_ball == True):
                    if i == 0:
                        if(math.fabs(self.golf_angle_robot > 87)):
                            temp = 0
                        elif(self.golf_angle_robot > 0):
                            temp = 1
                        else:
                            temp = -1
                    elif(i == 1 ):
                        temp = -1
                    else:
                        temp = 1
                    ##摄像头恢复初始位置
                    if flag :
                        DEbug.init_camera_angle(self.FLAG_flag)
                    while(self.best_robot_golf == False):
                        self.best_robot_golf = False
                        while(self.best_robot_golf == False and temp == 1):
                            print(self.golf_angle_robot)
                            ##右转向
                            DEbug.turn_right(self.FLAG_flag)
                            img = cv2.resize(self.my_camera.frame,(320,240))
                            img,LAB_img = PIG.process_img(img)
                            self.check_ball(LAB_img,img)
                            
                            if(0 < self.golf_angle_robot < 85):
                                temp = 1
                            elif(-85 < self.golf_angle_robot < 0):
                                temp = -1
                        while(self.best_robot_golf == False and temp == -1):
                            ##左转向
                            print(self.golf_angle_robot)
                            DEbug.turn_left(self.FLAG_flag)
                            img = cv2.resize(self.my_camera.frame,(320,240))
                            img,LAB_img = PIG.process_img(img)
                            self.check_ball(LAB_img,img)
                            if(0 < self.golf_angle_robot < 85):
                                temp = 1
                            elif(-85 < self.golf_angle_robot < 0):
                                temp = -1
                    img = cv2.resize(self.my_camera.frame,(320,240))
                    img,LAB_img = PIG.process_img(img)
                    self.check_hole(LAB_img,img)
                    break
            else:
                img = cv2.resize(self.my_camera.frame,(320,240))
                img,LAB_img = PIG.process_img(img)
                self.check_hole(LAB_img,img)
                if(self.Hole == True):
                    print("test")
                    if i == 0:
                        if(math.fabs(self.Hole_angle_robot > 85)):
                            best_hole_robot = True
                            temp = 0
                        elif(self.Hole_angle_robot > 0):
                            temp = 1
                        else:
                            temp = -1
                    elif(i == 1):
                        temp = -1
                    else:
                        temp = 1
                    ##摄像头恢复初始位置
                    if flag :
                        DEbug.init_camera_angle(self.FLAG_flag)
                    while(self.best_hole_robot == False and self.Hole):
                        self.best_hole_robot = False
                        print(self.best_hole_robot)
                        print("check0")
                        while(self.best_hole_robot == False and temp == 1 and self.Hole ):
                            ##右转向
                            DEbug.turn_right(self.FLAG_flag,1)
                            print(self.Hole_angle_robot)
                            img = cv2.resize(self.my_camera.frame,(320,240))
                            img,LAB_img = PIG.process_img(img)
                            self.check_hole(LAB_img,img)
                            if(0 < self.Hole_angle_robot < 85):
                                temp = 1
                            elif(-85 < self.Hole_angle_robot < 0):
                                temp = -1
                        while(self.best_hole_robot == False and temp == -1 and self.Hole):
                            ##左转向
                            print(self.Hole_angle_robot)
                            DEbug.turn_left(self.FLAG_flag)
                            img = cv2.resize(self.my_camera.frame,(320,240))
                            img,LAB_img = PIG.process_img(img)
                            self.check_hole(LAB_img,img)
                            if(0 < self.Hole_angle_robot < 85):
                                temp = 1
                            elif(-85 < self.Hole_angle_robot < 0):
                                temp = -1
                        print(self.best_hole_robot)
                        print("check1")
                    break
        img = cv2.resize(self.my_camera.frame,(320,240))
        img,LAB_img = PIG.process_img(img)
        self.check_ball(LAB_img,img)
        self.check_hole(LAB_img,img)
        print(self.best_hole_robot)
        print("check2")
        if flag == True:
            DEbug.init_camera_angle(self.FLAG_flag)
        return False
    #######################################################


    ###############平移到小球机器人与洞三点一线###############
    def translate_to_ball(self):
        while(self.best_robot_golf == False or self.best_hole_robot == False or self.golf_ball == False):
            print(self.best_robot_golf,self.best_hole_robot)
            if(self.golf_angle_robot > 0):
                print(self.golf_angle_robot,self.Hole_angle_robot)
                DEbug.right_move(self.FLAG_flag)
                time.sleep(1)
            else:
                print(self.golf_angle_robot,self.Hole_angle_robot)
                DEbug.left_move(self.FLAG_flag)
                time.sleep(1)
            self.find("hole",False)
            print(self.golf_dis_y)
            if(self.golf_ball==False):
                DEbug.back_up(self.FLAG_flag,3)
            if self.golf_dis_y > 190:
                DEbug.back_up(self.FLAG_flag,3)
        print("已经对正")
        return
    ########################################################

    ################根据边界图像处理角度矫正#################
    def check_angle_straight(self,temp = True):
        ## 云台向右转动
        if temp :
            DEbug.see_right_floor(self.FLAG_flag)
        ## 获取图像
        img = cv2.resize(self.my_camera.frame,(320,240))
        Img = cv2.GaussianBlur(img,(3,3),0)
        LAB_img = cv2.cvtColor(Img,cv2.COLOR_BGR2LAB)
        angle_right = 180-PIG.process_edge_img(LAB_img,Img,"angle","right")
        print("输出right边偏移角%f" % angle_right)
        if math.fabs(self.best_angle-angle_right) < 10:
            print(angle_right)
            return 
        elif(angle_right < self.best_angle) :
            #镜头偏右,机器人向右转
            DEbug.turn_right(self.FLAG_flag)
            self.check_angle_straight(False)
        elif(angle_right > self.best_angle):
            #镜头偏左,机器人向左转
            DEbug.turn_left(self.FLAG_flag)
            self.check_angle_straight(False)
    ########################################################

    #######校准函数#########################################
    def calibration(self,distance = 175,temp = True):
        self.standar_right_dt = distance
        right_distance = 0
        if temp :
            DEbug.see_right_floor(self.FLAG_flag)
        while math.fabs(right_distance-self.standar_right_dt) > 10:
            if(right_distance > self.standar_right_dt):
                DEbug.right_move(self.FLAG_flag)
            else:
                DEbug.left_move(self.FLAG_flag)
            #云台向右转动60，得到右图像
            print("调整位置，头右转")
            img = cv2.resize(self.my_camera.frame,(320,240))
            right_Img,right_LAB = PIG.process_img(img)
            
            #对左右图像进行边缘检测
            right_distance = PIG.process_edge_img(right_LAB,right_Img,"distance","right")
            print(right_distance)
        return True
    ########################################################

    ##################识别地板进入踢球关卡###################
    def check_floor(self):
        img = cv2.resize(self.my_camera.frame,(320,240))
        img,LAB_img = PIG.process_img(img)
        Imask = cv2.inRange(LAB_img,nsj.color_range['red_floor'][0],nsj.color_range['red_floor'][1])
        Imask = cv2.erode(Imask, None, iterations=3)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=3)
        cnts= cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        contour_area_sum = 0
        for c in cnts:  # 历遍所有轮廓
            contour_area_sum += math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        img_h, img_w = img.shape[:2]
        print("%f" % (contour_area_sum/(img_h*img_w)))
        if(contour_area_sum/(img_h*img_w) > 0.35):
            return True
        else:
            return False
    ########################################################

    #################转向下一关函数##########################
    def turn_to_next(self):
        DEbug.turn_left(self.FLAG_flag,n = 15)
        ##面向正前方
        ## 云台右转
        self.check_angle_straight()
        self.calibration(150)
        #self.check_angle_straight()
        return 
    ########################################################

    ###################################################################################################################################################

    ########################################################   
    def kick_ball(self):
        '''
        思路:
        先做好矫正角度，距离，让位置偏左好踢球
        然后直走找到球
        然后.................
        '''
        global golf_ball,Hole,golf_in_hole
        global golf_dis_x,golf_dis_y
        DEbug.init_camera_angle(self.FLAG_flag)
        
        print("进行角度矫正")
        #根据左右进行角度矫正
        self.check_angle_straight()
        #进行位置矫正
        print("角度矫正完毕,进行位置矫正")
        self.calibration()
        self.check_angle_straight()
        DEbug.go_forward(self.FLAG_flag,10)
        print("进行寻球")
        #向前走直到识别到小球
        self.find("ball")
        count = 0 #计数器
        while(self.golf_ball == False):
            #向前走三步
            DEbug.go_forward(self.FLAG_flag,5)
            self.find("ball")
            count += 1
            if count == 1 and self.golf_ball == False: ## 如果找不到球直接走人
                DEbug.turn_left(self.FLAG_flag,12)
                self.check_angle_straight()
                self.calibration(125)
                self.check_angle_straight()
                DEbug.go_forward(self.FLAG_flag,2)
                self.my_camera.camera_close()
                return 
        print("找到小球了,进行拉近距离测试")
        while(self.golf_dis_y < 140):
            DEbug.go_forward(self.FLAG_flag,1)
            img = cv2.resize(self.my_camera.frame,(320,240))
            img,LAB_img = PIG.process_img(img)
            self.check_ball(LAB_img,img)  
        print("拉近小球测试完毕,进行寻洞测试")
        self.find("hole")
        if(self.Hole == False):
            DEbug.turn_right(self.FLAG_flag,3)
            self.find("hole")
        print("寻洞测试完毕,进行三点一线测试")
        self.translate_to_ball()
        print("三点一线测试完毕，继续进行拉近小球测试")
        img = cv2.resize(self.my_camera.frame,(320,240))
        img,LAB_img = PIG.process_img(img)
        self.check_ball(LAB_img,img)
        print("golf_dis_y = %d" % self.golf_dis_y)
        while(self.golf_dis_y < 190):
            print("拉近小球到脚下——————————————————————————————————————————————")
            # DEbug.go_slow(self.FLAG_flag,1)
            DEbug.go_forward(self.FLAG_flag,1)
            img = cv2.resize(self.my_camera.frame,(320,240))
            img,LAB_img = PIG.process_img(img)
            self.check_ball(LAB_img,img)
        print("踢球测试")
        DEbug.kick_ball(self.FLAG_flag)
        DEbug.go_forward(self.FLAG_flag,3)
        self.turn_to_next()
        DEbug.go_forward(self.FLAG_flag,9)
        self.check_angle_straight()
        self.my_camera.camera_close()
        return
    ########################################################


    ######################球不踢版本################################
    def leave(self):
        '''
        思路:
        先做好矫正角度，距离，让位置偏左好踢球
        然后直走找到球
        然后.................
        '''
        DEbug.init_camera_angle(self.FLAG_flag)
        ###########################################################
        print("进行识别关卡测试")
        while(self.check_floor() == False):
            DEbug.go_forward(self.FLAG_flag,3)
        DEbug.go_forward(self.FLAG_flag,3)
        ###########################################################
        print("识别到踢小球关卡,进行角度矫正测试")
        #根据左右进行角度矫正
        self.check_angle_straight()
        #进行位置矫正
        ###########################################################
        print("角度矫正完毕,进行位置矫正测试")
        self.calibration()
        DEbug.go_forward(self.FLAG_flag,9)
        DEbug.turn_left(self.FLAG_flag,15)
        self.check_angle_straight()
        self.calibration()
        #self.check_angle_straight()
        DEbug.go_forward(self.FLAG_flag,1)
        return
    #######################################################
def kick_golf():
    kickball = nsj_kick()
    kickball.kick_ball()

if __name__ == '__main__':
    kickball = nsj_kick()
    kickball.kick_ball()
    input("yes")

