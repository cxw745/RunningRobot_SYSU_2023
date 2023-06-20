from operator import le
from robot import Robot
import math
import cv2

import numpy as np
import time
import copy

class Solver():
    def __init__(self):
        self.robot = Robot()
        self.DEBUG = self.robot.DEBUG
        
        self.h,self.w = self.robot.image.shape[:2] # image shape      

        #-----------------------params-------------#
        self.origin_wh_proportion = {
            "boom" :[],
            "sill" :1.5,
            "door" :1.5,
            "ball" :1.5,
            "hole" :[],
            "step" :1.5,
            "bridge" :1.5,
            "danger" :1.5,
            "bridgedd" :[],
        }
        
        # section 1 danger
        #self.pit_min_distance = 70
        self.pit_min_distance = 100
        #self.pit_bias = 95
        self.pit_bias = 90
        
        
        # section 2 mine
        self.mine_turn = 1
        self.mine_min_shift_bias = 120
        self.mine_distance_bias = 100
        self.max_obstracle_proportion = 0.4
        self.sill_max_distance = 100
        #self.sill_max_distance = 120
        
        
        # section 3 bridge
        
        self.bridge_distance = 50
        self.bridge_min_center = 150
        self.bridge_max_center = 165
        self.bridge_min_bias = -10
        self.bridge_max_bias = 5
        # section 4 ball
        self.waiting_count = 10
        self.maximum_step_kick = 6
        self.walk_coef = 100
        #self.turn_coef = 7
        self.turn_coef = 18
        #self.turn_coef = 21
        self.ball_dist = 120
        self.ball_min_bias = -5
        self.ball_max_bias = 5
        self.kick_min_shift_bias = -70
        self.kick_max_shift_bias = -40
        self.kick_max_distance_bias = 90
        self.kick_min_distance_bias = 40
        self.kick_angle_bias = 4
        self.kick_max_turn_bias = 10
        
        # section 5 stairs
        self.step_distance = 100
        self.step_min_bias = -6
        self.step_max_bias = 6
        self.step_min_center = 150
        self.step_max_center = 170        
        
    def get_size(self,x1,y1,x2,y2):
        """
        input x1,y1,x2,y2,return the area of the target
        """

        return (x2-x1) * (y2 - y1)

    def get_prop(self,x1,y1,x2,y2):
    
        return self.get_size(x1,y1,x2,y2) / self.h / self.w
    
    def analyze_target(self,target,target_type):
        """
        判断目标在机器人的方位 rho,theta
        :param target : 预测的物体
        :param axis: 预测的朝向
        """
        x1,y1,x2,y2 = target
        width_height_proportion = (x2 - x1)/(y2 - y1)
        print(f"width_height_proportion = {width_height_proportion}")
        origin_proportion = self.origin_wh_proportion[target_type]
        
        center = [(x1+x2)//2,(y1+y2)//2]
        bias = center[0] - self.w//2
        
        if width_height_proportion < origin_proportion:
            if bias > 0:
                turn = self.robot.min_turnDegree
            else:
                turn = -self.robot.min_turnDegree
        else:
            turn = 0
        
        return bias, turn
    
    def select_section(self):
        targets = self.robot.detect_object()
        if targets["total"] == 0:
            print("no target find")
            self.robot.Forwark_one()
        else:
            pass 
    # section 1 (pit)

    def section_pit(self):  # 这一关是过坑的相关代码
        count_left = 0  # 标志位
        while True:  # 进入循环
            targets = self.robot.detect_object()  # 获取当先摄像头中识别到了哪些物体
            bridge = targets["bridge"]  # 是否检测到了对应项
            pits = targets["danger"]
            if len(pits) > 0:  # 能检测到坑
                x1, y1, x2, y2 = pits[0][:4]  # 设置参数
                distance = self.h - y2  # 计算距离
                bias, turn = self.analyze_target(pits[0][:4], "danger")  # 分析目标的相对位置
                if distance > self.pit_min_distance:  # 如果离坑还很远，就往前走
                    self.robot.StepForward()
                if bias < self.pit_bias and distance < self.pit_min_distance:  # 走到坑前面就开始横移了
                    self.robot.MoveLeft()
                    count_left += 1  # 标志位置为1
                    if (count_left % 2) == 0:  # 这里要进行对正的一些处理，不然很容易会走掉下去
                        pCheck = self.findParallel_level2()
                        if pCheck == -1:
                            continue
                        elif pCheck == 1:
                            continue
                        elif pCheck == 2:
                            self.robot.TurnRight()
                            print("not parallel need to turn right")
                        elif pCheck == 3:
                            self.robot.TurnLeft()
                            print("not parallel neet to turn left")
                if bias > self.pit_bias:  # 如果偏差较多，需要调整机器人的位置
                    self.robot.TurnRight()
                    self.robot.Forwark_one(1)
                    self.robot.testRight()
                    self.robot.testRight()
                    break
            if len(pits) == 0 and len(bridge) > 0:  # 如果检测不到坑，但检测的到桥
                # 这里要根据框出的桥进行重新计算，思路和上面基本上一样
                x1, y1, x2, y2 = bridge[0][:4]
                distance = self.h - y2
                bias, turn = self.analyze_target(bridge[0][:4], "danger")
                if distance > self.pit_min_distance:
                    self.robot.StepForward()
                if bias < self.pit_bias and distance < self.pit_min_distance:
                    self.robot.MoveLeft()

            if len(pits) == 0:  # 检测不到坑了
                # 这里需要想办法重新找回正确的方向
                pCheck = self.findParallel_level2()  # 测算平行线来调整机器人的站立角度
                # 针对不同的朝向设置阈值（需要测试后进行调整），然后针对不同的数值进行左转右转的方向修正
                if pCheck == -1:
                    self.robot.Forwark_one(1)
                    self.robot.testRight()
                    self.robot.testRight()
                    break
                elif pCheck == 1:
                    self.robot.Forwark_one(1)
                    self.robot.testRight()
                    self.robot.testRight()
                    break
                elif pCheck == 2:
                    self.robot.TurnRight()
                    print("not parallel need to turn right")
                elif pCheck == 3:
                    self.robot.TurnLeft()
                    print("not parallel neet to turn left")

    def section_grass(self):
        '''
        version: 1.0
        description: 识别画面中抬杆抬起后，前进
        '''
        time.sleep(3)
        cd = {"yellow": [(17, 70, 70), (34, 255, 255)],  # 黄色HSV值
        "black": [(0, 0, 0), (180, 255, 35)]}       #黑色HSV数值

        thresSection0 = 0.8
        while True:
            image = self.robot.image[:120,:,:]      #裁剪掉图片下半部分
            yellow = cv2.inRange(image, cd["yellow"][0],cd["yellow"][1])
            black = cv2.inRange(image, cd["black"][0],cd["black"][1])
            bio = np.array(cv2.bitwise_or(yellow, black), dtype=np.float)   #黑黄或，得到二者共同的区域
            #cv2.imshow("res", np.hstack([image, cv2.cvtColor(np.array(bio,dtype = np.uint8),cv2.COLOR_GRAY2BGR)]))
            #cv2.waitKey(1)
            bio/=255
            yellow = np.array(yellow, dtype = np.float)/255
            weightY = int(np.sum(np.sum(bio, axis=1) * np.array(range(bio.shape[0]))) / (np.sum(bio) + 1e-8))
            roiArea = bio[weightY-4:weightY+5, :].sum() #计算黄黑色区域的面积大小

            print(bio.sum() * thresSection0 < roiArea - 10, yellow.sum())
            if bio.sum() * thresSection0 < roiArea - 10 or yellow.sum() > 200:      #面积大于200，代表杆子未升，等待
                continue
            break

        self.robot.Forwark_one()    #杆子升起，可以直行
        #self.robot.Forwark_half(2)
        #self.robot.Forwark_half()

    def section_bridge(self):
        # 这一关是进行桥的识别
        count_turn = 0  # 标志位，标识识别状态
        while True:
            targets = self.robot.detect_object()  # 同理，看是否能检测到桥
            obstracles = targets['bridge']

            if len(obstracles) == 0:  # 如果看不到桥
                print("没看到桥")
                # 根据不同的状态，设置了不同的角度调整方案，找到一个能看得到桥的角度
                if count_turn < 3:
                    self.robot.TurnLeft()
                elif count_turn == 3:
                    for _ in range(3):
                        self.robot.TurnRight()
                elif count_turn >= 4 and count_turn <= 6:
                    self.robot.TurnRight()
                count_turn += 1

            else:
                break  # 如果能看到桥，就退出进行后面的过桥操作

        while True:
            # 先找到桥的位置
            image = self.robot.image
            targets = self.robot.detect_object()
            obstracles = targets['bridge']
            doors = targets['door']
            if len(obstracles) > 0:  # 如果能检测到桥
                obstracles.sort(key=lambda x: x[4], reverse=True)
                bias, turn = self.analyze_target(obstracles[0][:4], "bridge")  # 计算相对位置

                print(f"bias = {bias},degree = {turn}")

                x1, y1, x2, y2, _ = obstracles[0]

                bridge_center = [(x1 + x2) // 2, (y1 + y2) // 2]  # 计算桥的中心位置
                bridge_distance = self.h - y2  # 相对距离
                print(f"bridge_center = {bridge_center} distance = {bridge_distance}")
                if bridge_distance < self.bridge_distance and abs(bias) < 10:  # 满足经过测试后得到的偏差参数就退出
                    break
                else:  # 根据偏差的不同，命令机器人在桥上进行转向，找到比较正的方向之后再向前走
                    if bias < self.bridge_min_bias:
                        self.robot.TurnLeft()
                    elif bias > self.bridge_max_bias:
                        self.robot.TurnRight()

                    elif bridge_center[0] < self.bridge_min_center and abs(bias) < self.bridge_max_bias:
                        self.robot.MoveLeft()
                    elif bridge_center[0] > self.bridge_max_center and abs(bias) < self.bridge_max_bias:
                        self.robot.MoveRight()
                    else:
                        self.robot.Forwark(3)
            else:
                self.robot.Forwark()


    def bridge_mid(self, lineA, LineB):
        '''
        version: 1.0
        description: 辅助函数, 依据两直线在y=320处的两个x值, 求它们的重点值
        '''         
        x1,y1,x2,y2=lineA
        a1,b1,a2,b2=LineB
        point1=x2-(x1-x2)/(y1-y2)*(y2-320)
        point2=a2-(a1-a2)/(b1-b2)*(b2-320)
        midpoint=(point1+point2)/2
        return int(midpoint)

    def bridge_debug(self):
        # 这个函数是对上面桥函数的补充，是走到桥上之后使用的
        print("In bridge_debug!")
        while True:
            time.sleep(1)  # original 2

            lower_color = np.array([50, 45, 33])  # lab1232中实验室bridge(绿色)的HSV
            upper_color = np.array([100, 255, 255])

            img = self.robot.image  # 获取图像
            roi = img
            # 进行颜色处理
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # 从BGR转换到HSV
            mask = cv2.inRange(hsv, lower_color, upper_color)  # inRange()：介于lower/upper之间的为白色，其余黑色
            gaussian = cv2.GaussianBlur(mask, (7, 7), 0)  # 高斯滤波
            edges = cv2.Canny(gaussian, 50, 150)

            # 统计概率霍夫线变换
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)
            if lines is None:
                print('No line found! Do 1 step')
                self.robot.Forwark(1)
                # self.robot.small_step()
                continue
            else:
                # 3.1找到直线后, 筛选, 执行相应动作
                m1 = -1
                m2 = -1
                x_mid = 160  # 如果没找到两条直线, 执行机器人位于bridge中央的代码
                print('line total: ' + str(np.shape(lines)[0]))
                for i in range(0, np.shape(lines)[0]):
                    x1, y1, x2, y2 = lines[i][0]
                    tmp = self.slope_choose(x1, y1, x2, y2)
                    if (tmp == 1 and m1 == -1):  # 向右下斜的直线
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        m1 = i
                    if (tmp == 2 and m2 == -1):  # 向左下斜的直线
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        m2 = i

                if (m1 != -1 and m2 != -1):  # 如果确实找到了两条直线
                    # x, y = self.getCrossPoint(lines[m1][0], lines[m2][0]) # 求交点
                    x_mid = self.bridge_mid(lines[m1][0], lines[m2][0])

                # cv2.circle(drawing, center = (int(x_mid), int(320)), radius = 5, color = (0, 0, 255), thickness = 0) # 画圆，圆心为：(160, 160)，半径为：60，颜色为：point_color，实心线
                # cv2.imshow('lines', drawing)
                # cv2.waitKey(1)

                bias = x_mid - 160  # 桥中心与机器人脚中心的偏差值
                print('Bridge bias:' + str(bias))
                if (m1 + m2 == -2):
                    self.robot.Forwark(3)
                    # self.robot.small_step(4)
                    break  # all line in lines are not fit, quit this section
                elif (m1 == -1 and m2 != -1):
                    self.robot.TurnRight()
                elif (m1 != -1 and m2 == -1):
                    self.robot.TurnLeft()
                else:
                    if bias >= 20:  # 25
                        self.robot.MoveRight()
                    if bias <= -20:  # 25
                        self.robot.MoveLeft()
                    else:
                        # self.robot.small_step(2)
                        self.robot.Forwark(
                            1)  # 如果机器人位于桥中央, 走两步以加快过桥速度(discarded) two step is not stable, might fall off

    def section_bridge_sum(self):
        '''
        version: 1.0
        stable
        description: 将section_bridge和bridge_debug整合, 增加了bridge_debug部分找不到直线时的处理, 
            以期增强程序鲁棒性 
        '''
        print("In section_bridge_sum")
        # find bridge       
        while True:     
            targets = self.robot.detect_object()
            obstracles = targets['bridge']
            if len(obstracles) == 0:
                self.robot.TurnLeftBig()
            else:
                break
        # go to bridge
        print('Already found bridge, get close')
        while True:
            targets = self.robot.detect_object()
            obstracles = targets['bridge']
            
            if len(obstracles) > 0:
                obstracles.sort(key = lambda x:x[4],reverse = True)
                bias,turn =self.analyze_target(obstracles[0][:4],"bridge")
                
                print(f"bias = {bias},degree = {turn}")
                
                x1,y1,x2,y2,_ = obstracles[0]
                
                bridge_center = [(x1+x2)//2,(y1+y2)//2]
                bridge_distance = self.h - y2
                print(f"bridge_center = {bridge_center} distance = {bridge_distance}")    
                if bridge_distance < self.bridge_distance and abs(bias) < 10: # 距离bridge已经很近, 退出寻找bridge部分
                    #self.robot.Forwark_one()
                    #self.robot.Forwark(3)
                    break # already found and got close to bridge
                else:
                    if bias < self.bridge_min_bias :
                        self.robot.TurnLeft()
                    elif bias > self.bridge_max_bias :
                        self.robot.TurnRight()
                        
                    elif bridge_center[0] < self.bridge_min_center and abs(bias) < self.bridge_max_bias:
                        self.robot.MoveLeft()
                    elif bridge_center[0] > self.bridge_max_center and abs(bias) < self.bridge_max_bias:
                        self.robot.MoveRight()
                    else:
                        self.robot.Forwark(3)
        
        # walk through bridge
        print("On bridge")
        nolineCnt = 0 # 增设该变量以增强程序鲁棒性, 如果连续三次都没能检测到任何直线, 则退出section_bridge
        while True:
            time.sleep(1) # original 2

            lower_color = np.array([50, 45, 33])    # lab1232中实验室bridge(绿色)的HSV
            upper_color = np.array([100, 255, 255])

            img = self.robot.image
            # roi = img[40:280, 20:300]                           # 第一个为y的范围，第二个为x值的范围
            roi = img

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)          # 从BGR转换到HSV
            mask = cv2.inRange(hsv, lower_color, upper_color)   # inRange()：介于lower/upper之间的为白色，其余黑色
            gaussian = cv2.GaussianBlur(mask, (7, 7), 0)        # 高斯滤波
            edges = cv2.Canny(gaussian, 50, 150)

            # drawing = np.zeros(roi.shape[:], dtype=np.uint8)

            # 统计概率霍夫线变换
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)
            if lines is None:
                print('No line found! Do 1 step')
                self.robot.Forwark(1)
                nolineCnt += 1
                if nolineCnt == 2: # 3次都找不到任何直线, 退出section_bridge
                    break
                continue
            else:
                nolineCnt = 0 # 找到直线后, 将未找到直线次数清零
                # 找到直线后, 筛选, 执行相应动作
                m1 = -1
                m2 = -1
                x_mid = 160 # 如果没找到两条直线, 执行机器人位于bridge中央的代码
                print('line total: ' + str(np.shape(lines)[0]))
                for i in range(0,np.shape(lines)[0]):
                    x1, y1, x2, y2 = lines[i][0]
                    tmp = self.slope_choose(x1,y1,x2,y2)
                    if(tmp == 1 and m1 == -1): # 向右下斜的直线
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        m1 = i
                    if(tmp == 2 and m2 == -1): # 向左下斜的直线
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        m2 = i

                if(m1!=-1 and m2!=-1): # 如果确实找到了两条直线
                    # x, y = self.getCrossPoint(lines[m1][0], lines[m2][0]) # 求交点
                    x_mid = self.bridge_mid(lines[m1][0], lines[m2][0])

                # cv2.circle(drawing, center = (int(x_mid), int(320)), radius = 5, color = (0, 0, 255), thickness = 0) # 画圆，圆心为：(160, 160)，半径为：60，颜色为：point_color，实心线
                # cv2.imshow('lines', drawing)
                # cv2.waitKey(1)

                bias = x_mid - 160 # 桥中心与机器人脚中心的偏差值
                print('Bridge bias:' + str(bias))
                if(m1 + m2 == -2): # 找不到bridge两侧的两条直线时, 退出
                    self.robot.Forwark(3)
                    break      # all line in lines are not fit, quit this section
                elif(m1==-1 and m2!=-1):
                    self.robot.TurnRight()
                elif(m1!=-1 and m2==-1):
                    self.robot.TurnLeft()
                else:
                    if bias >= 20: # 25
                        self.robot.MoveRight()
                    if bias <= -20: # 25
                        self.robot.MoveLeft()
                    else:
                        self.robot.Forwark(1) # two steps are two dangerous, might fall off from bridge

    def mine_debug(self):
        '''
        version: 0.0
        description: 未被改动过，机器人中自带的调试代码
        '''         
        targets = self.robot.detect_object()
        obstracles = targets['sill']
        if len(obstracles) > 0:
            obstracles.sort(key=lambda x: x[4], reverse=True)
            bias, turn = self.analyze_target(obstracles[0][:4], "sill")
            distance = self.h - obstracles[0][3]
            print(f"distance = {distance}")
            print(f"bias = {bias},degree = {turn}")

            x1, y1, x2, y2, _ = obstracles[0]
            obstracle_proportion = self.get_prop(x1, y1, x2, y2)
            print(f"obstracle_proportion = {obstracle_proportion}")

    def minepoint(self, LineA):
        '''
        version: 1.0
        description: 辅助函数, 找一条直线y=320时的点, 该函数用于检测机器人与section_mine某一边缘的距离
        '''
        x1, y1, x2, y2 = LineA
        point = x2 - (x1 - x2) / (y1 - y2) * (y2 - 320)
        return int(point)


            
    def get_degree_formine(self, x1,y1,x2,y2):
        '''
        version: 1.1
        description: 辅助函数, using slope to turn robot
        '''
        if(x1-x2==0):
            return -1
        k = float(y1-y2) / float(x1-x2)#没有转的
        print('slope = ' + str(k))
        if 0.02 <= k <= 0.3:     #斜率为正, 且满足条件
            return 1
        elif -0.3 <= k <= -0.02: #斜率为负, 且满足条件
            return 2
        elif -0.02 <= k <= 0.02:  #机器人与眼前直线基本平行
            return 3
        else:
            return -1
    def Parallel_check_formine(self, degreeOnly=0, cnt=0):
        '''
        version: 1.1.2
        In Working
        description: judge is robot is parallel to the edge in front, and adjust
            distance to the edge (distance adjust can be turned off)
        '''
        lower_color = np.array([76, 18, 0])  # 菱形蓝花纹地板的HSV值
        upper_color = np.array([128, 255, 255])

        img = self.robot.image
        # roi = img[130:320, :]                           # 第一个为y的范围，第二个为x值的范围
        #roi = img[120:230, :]
        roi = img
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # 从BGR转换到HSV
        mask = cv2.inRange(hsv, lower_color, upper_color)  # inRange()：介于lower/upper之间的为白色，其余黑色
        gaussian = cv2.GaussianBlur(mask, (7, 7), 0)  # 高斯滤波
        edges = cv2.Canny(gaussian, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)

        # drawing = np.zeros(roi.shape[:], dtype=np.uint8)

        if lines is None:
            print('No lines found.')
        else:
            self.sortLines(lines)
            for i in range(0, np.shape(lines)[0]):
                x1, y1, x2, y2 = lines[i][0]
                tmp = self.get_degree(x1, y1, x2, y2)
                if (tmp == 1):
                    # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    # cv2.imwrite('/home/lemon/robot/'+'originalImg_right' + str(cnt) + '.jpg', self.robot.image)
                    # cv2.imwrite('/home/lemon/robot/'+'edgeImg_right' + str(cnt) + '.jpg', drawing)
                    print('TurnRight')
                    self.robot.TurnRight()
                    break
                elif tmp == 2:
                    # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    # cv2.imwrite('/home/lemon/robot/'+'originalImg_left' + str(cnt) + '.jpg', self.robot.image)
                    # cv2.imwrite('/home/lemon/robot/'+'edgeImg_left' + str(cnt) + '.jpg', drawing)
                    print('TurnLeft')
                    self.robot.TurnLeft()
                    break
                elif tmp == 3:
                    print('Parallel to line')
                    return 1
                elif tmp == -1:
                    print('No qualified line')
                    return -1

        # cv2.imshow('edges', edges)
        # cv2.waitKey(1)        
    
    def get_degree_forsill(self, x1, y1, x2, y2):
        '''
        version: 1.0
        description: 找每条线的斜率，目的是为了找出花纹地砖的右侧边缘
        date:2022/8/18
        '''
        if (x1 - x2 == 0):
            return -1
        k = float(y1 - y2) / float(x1 - x2)  # 没有转的
        print('slope = ' + str(k))
        return k


    
    def section_mine(self):
        '''
        version: 1.0
        In Working
        description: 通过地雷并翻过sill的函数, 机器人的移动受到地雷和赛道边缘两个因素的控制
        '''        
        """
        Two Steps:
         Crossing mine zone: move left & right & forward to cross, only turn if has to.
            Adjust, Get over the obstracle, than turn to face the door
        """
        print('arriving mine')
        sill_distance = 100000
        cnt = 1
        Movecnt = 0
        leftEdgeFlag = 0  # 标志位, 代表距离左侧边缘过近, 需要特殊处理
        rightEdgeFlag = 0 # 标志位, 代表距离右侧边缘过近, 需要特殊处理
        while True:
            """
            if facing the wrong direction, turn
            else if a mine in the close front, check the obstracle to decide whether to move left or right
            else step forward
            """
            targets = self.robot.detect_object()        #调用yolo检测地雷
            obstracles = targets['sill']
            bridge = targets['bridge']
            if len(obstracles) > 0:         #识别到了目标
                sill_distance = self.h - obstracles[0][3] # 每次都需要更新

            if cnt >= 2: # 左右移动步数大于2，需要sleep以保证稳定性
                time.sleep(1)
                cnt = 0
                if len(obstracles) > 0:
                    obstracles.sort(key=lambda x: x[4], reverse=True)
                    bias, turn = self.analyze_target(obstracles[0][:4], "sill")
                    # sill_distance = self.h - obstracles[0][3]
                    print(f"bias = {bias},degree = {turn}")
                    if bias > 30:       #与sill的夹角过大，需要左右调整
                        self.robot.TurnRight(3) # change to 3, maybe 4 is to much
                    if bias < -30:
                        self.robot.TurnLeft(3)  # orignially is 4
                    x1, y1, x2, y2, _ = obstracles[0]
                    obstracle_proportion = self.get_prop(x1, y1, x2, y2)
                    # print(f"obstracle_proportion = obstracle_proportion")
                    if abs(turn) > self.robot.min_turnDegree / 2:
                        # print("")
                        pass
                else:
                    self.robot.MoveLeft()
            else:
                obstracle_proportion = 0
                bias = 0
            print('sill_distance:'+str(sill_distance))
            # avoiding
            mines = targets["boom"]

            print('leftEdgeFlag:' + str(leftEdgeFlag))
            print('rightEdgeFlag:' + str(rightEdgeFlag)) 

            skip_flag = False
            for x1, y1, x2, y2, _ in mines:
                mine_center = [(x1 + x2) // 2, (y1 + y2) // 2]  #计算地雷的中心位置
                print("mine_center", abs(mine_center[0] - self.w // 2))  # 相减后判断地雷在机器人中线的左侧还是右侧
                if abs(mine_center[0] - self.w // 2) < self.mine_min_shift_bias and abs(
                        mine_center[1] - self.h) < self.mine_distance_bias:
                    '''找到mines里的第一个地雷后进行处理, 然后直接退出'''
                    # edgeAvoid = self.mineDetectEdge(self.robot.image)
                    # print('edgeAvoid: ' + str(edgeAvoid))
                    # if edgeAvoid == 0:            #距离地雷的位置过近，左右横移
                    if mine_center[0] - self.w // 2 > 0:
                        # self.robot.MoveLeft()
                        self.robot.testLeft()
                        Movecnt=Movecnt+1
                        if Movecnt > 1:#避免连续移动导致机器人摔倒，需sleep
                            time.sleep(1)
                            print('sleeping')
                            Movecnt = 0
                    else:
                        # self.robot.MoveRight()
                        self.robot.testRight()
                        Movecnt=Movecnt+1
                        if Movecnt > 1:
                            time.sleep(1)
                            print('sleeping')
                            Movecnt = 0
                        minePos = mine_center[0] - self.w // 2 # 大于0时地雷靠右, 小于0时地雷靠左
                        if leftEdgeFlag == 0 and minePos > 0:
                            self.robot.MoveLeft() # 无需考虑左边缘, 地雷靠右
                            Movecnt=Movecnt+1
                            if Movecnt > 1:#避免动作摔倒
                                time.sleep(1)
                                print('sleeping')
                                Movecnt = 0
                        elif rightEdgeFlag == 0 and minePos < 0:
                            self.robot.MoveRight() # 无需考虑右边缘, 地雷靠左
                            Movecnt=Movecnt+1
                            if Movecnt > 1:
                                time.sleep(1)
                                print('sleeping')
                                Movecnt = 0                        
                        elif leftEdgeFlag == 1 and minePos > 0:#距离左边缘过近
                            print('left Working!')
                            self.robot.MoveRight() # 距离左边缘过近, 但画面右侧有地雷, 依然向右移
                            Movecnt=Movecnt+1       #由于距离左侧过近，虽然地雷在右，但仍向右不断避让
                            if Movecnt > 1:
                                time.sleep(1)           #避免连续右移导致摔倒
                                print('sleeping')
                                Movecnt = 0                        
                        elif rightEdgeFlag == 1 and minePos < 0:     #距离右侧太近并且左侧有地雷
                            print('right Working!')
                            self.robot.MoveLeft() # 距离右侧边缘过近, 但画面左侧有地雷, 依然向左移
                            Movecnt=Movecnt+1       #同理，向左侧移动直到避开地雷
                            if Movecnt > 1:
                                time.sleep(1)       #保持动作稳定性
                                print('sleeping')
                                Movecnt = 0

                        # 边缘处理标志清除
                        if minePos < 0 and leftEdgeFlag == 1:
                            leftEdgeFlag = 0 # 此时地雷靠左, 已经远离左侧边缘, 继续执行正常流程
                        elif minePos > 0 and rightEdgeFlag == 1:
                            rightEdgeFlag = 0 # 此时地雷靠右, 已经远离右侧边缘, 继续正常流程


                    '''elif edgeAvoid == 1:
                        print('Close to right Edge')
                        # leftEdgeFlag = 1                        
                        self.robot.MoveLeft()
                        self.robot.MoveLeft()
                        time.sleep(0.5)
                        self.robot.MoveLeft()
                    elif edgeAvoid == -1:
                        print('Close to leftt Edge')
                        # rightEdgeFlag = 1                        
                        self.robot.MoveRight()
                        self.robot.MoveRight()
                        time.sleep(0.5)
                        self.robot.MoveRight()'''
                    skip_flag = True
                    break
            if skip_flag: # 处理了一个地雷, 就不进行sill距离的判断了
                continue

            # if close enough to the obstracle ,finetune
            if sill_distance < self.sill_max_distance:  # 与sill的距离小于预设值120，则执行翻越动作
                self.robot.small_step(5) # 先往前蹭几步，保证自己与边缘平行，之后找右边的直线，如果找不到就往右平移
                '''
                while True:
                    print('shuipingduiqi')
                    sill_parallel_flag = self.Parallel_check_formine()
                    if sill_parallel_flag == -1 or sill_parallel_flag == 1 :
                        break
                '''
                failcnt = 0
                while True:
                    sill_flag = self.Vertical_check()   #判断与sill的角度，找到赛道的右边缘线
                    if sill_flag == 1:#距离右边缘线距离合适，说明已经到sill中心，可以开始翻越sill
                        break
                    elif sill_flag == -1:   #离右侧边缘线太远，说明不在sill中间
                        failcnt += 1
                        if failcnt >3:      #平移超过三次则退出，默认已经到sill中心
                            print('fail check line, continue cross sill')
                            break 
                        #print('cross sill end')
                #self.robot.cross_obstracle_test()
                self.robot.cross_obstracle()    #翻过sill
                break  # 退出section_mine()
            else:
                # edgeAvoidForStraight = self.mineDetectEdge(self.robot.image)
                # print('Straight_edgeAvoid: ' + str(edgeAvoidForStraight))
                # if edgeAvoidForStraight == 0:
                self.robot.StepForward()
                '''elif edgeAvoidForStraight == 1:
                    self.robot.MoveLeft()
                    self.robot.MoveLeft()
                    self.robot.MoveLeft()
                elif edgeAvoidForStraight == -1:
                    self.robot.MoveRight()
                    self.robot.MoveRight()
                    self.robot.MoveRight()'''                   
                cnt += 1

    def section_door_Better(self):
        '''
        version: 1.0
        stable
        description: 翻过sill并后退5步后进入该函数, 该函数负责通过门, 并假设门后为section_pit(过坑)路段
            currently door-avoid func is added, but not very useful
            (如果深度学习检测到门，则将机器人中心和门的bounding box中心对比，两者x差值大于一定阈值时左右横移)
        '''
        while True:# turn to pits
            targets = self.robot.detect_object()
            obstracles = targets['danger']
            door = targets['door']

            if len(obstracles) == 0 and len(door) == 0:
                self.robot.TurnLeftBig()
                self.robot.StepForward()
                self.robot.StepForward()
                time.sleep(3)
            else:
                break

        while True:
            '''targets = self.robot.detect_object()
            bridge = targets["bridge"]
            pits = targets["danger"]'''

            targets = self.robot.detect_object()

            # 根据门调整自身的位置, 使机器人从中间走过
            doors = targets['door']
            center = 160
            print('length doors = ' + str(len(doors)))
            if len(doors) > 0: # originally is 1
                print('Adjusting door')
                # targets = self.robot.detect_object()
                # doors = targets['door']
                center = (doors[0][0] + doors[0][2])//2

                print('Center at: ' + str(center))
                cv2.circle(self.robot.image, (int(center), int(160)), radius = 5, color = (0, 0, 255), thickness = 0)
                adjust = center - 160
                print('adjust = ' + str(adjust))
                if adjust > 15:
                    self.robot.MoveRight()
                    continue
                if adjust < -15:
                    self.robot.MoveLeft()
                    continue

            bridge = targets["bridge"]
            pits = targets["danger"]
            objectName = 'danger'
            if len(pits) == 0:
                pits = targets['bridge']
                objectName = 'bridge'
                if len(pits) == 0:
                    print('123456')
                    pits == targets['boom']
                    objectName = 'boom'

            if len(pits) > 0:
                #detect_cnt = 0
                x1,y1,x2,y2 = pits[0][:4]
                distance = self.h - y2

                #print(distance)
                bias,turn = self.analyze_target(pits[0][:4],objectName)
                #print("bias:",bias)
                if distance > self.pit_min_distance:
                   self.robot.StepForward()

                if  bias < self.pit_bias and distance < self.pit_min_distance:
                    self.robot.MoveLeft()
                if bias > self.pit_bias:
                    # self.robot.TurnRight()
                    self.robot.Forwark_one(1)
                    #self.robot.MoveRight(5) original
                    self.robot.MoveRight(2)
                    self.robot.TurnRight(2)
                    print('nb')
                    #self.robot.Forwark_one()
                    #self.robot.TurnRightBig()
                    #self.robot.Forwark_one()
                    #self.robot.cross_obstracle()
                    break
            if len(pits) == 0 and len(bridge) > 0:
                x1,y1,x2,y2 = bridge[0][:4]
                distance = self.h - y2
                bias,turn = self.analyze_target(bridge[0][:4],objectName)
                #print("bias:",bias)
                if distance > self.pit_min_distance:
                   self.robot.StepForward()

                if  bias < self.pit_bias and distance < self.pit_min_distance:
                    self.robot.MoveLeft()



    def get_degree(self, x1,y1,x2,y2):
        '''
        version: 1.1
        description: 辅助函数, using slope to turn robot
        '''
        if(x1-x2==0):
            return -1
        k = float(y1-y2) / float(x1-x2)#没有转的
        print('slope = ' + str(k))
        if 0.1 <= k <= 0.3:     #斜率为正, 且满足条件
            return 1
        elif -0.3 <= k <= -0.1: #斜率为负, 且满足条件
            return 2
        elif -0.1 <= k <= 0.1:  #机器人与眼前直线基本平行
            return 3
        else:
            return -1

    def calcuDistance(self, x1, y1, x2, y2):
        '''
        version: 1.0
        description: 辅助函数，用于调整机器人到眼前边缘的距离
        '''
        A = (y2 - y1)
        B = (x2 - x1)
        C = (y2 - y1) * x1 - (x2 - x1) * y1
        robotX = 160
        robotY = 320
        distance = abs(A * robotX + B * robotY + C) / math.sqrt(A ** 2 + B ** 2) # 求点到直线的距离
        return distance

    def sortLines(self, lines):
        '''
        version: 1.0
        description: 辅助函数，用于将概率霍夫直线识别出的直线按y1值从大到小排序
        '''        
        for i in range(0, np.shape(lines)[0]):
            maxIndex = i
            for j in range(i + 1, np.shape(lines)[0]):
                if lines[j][0][1] > lines[maxIndex][0][1]:
                    maxIndex = j
            temp = copy.deepcopy(lines[i][0]) # 如果此处不用深拷贝, temp将只是lines[i][0]的别名
            lines[i][0] = lines[maxIndex][0]
            lines[maxIndex][0] = temp

    global turnCnt
    turnCnt = 0

    def Parallel_check(self, degreeOnly = 0, cnt = 0):
        '''
        version: 1.1.2
        In Working
        description: judge whether robot is parallel to the edge in front, and adjust
            distance to the edge (distance adjust can be turned off)
        '''
        # lower_color = np.array([43, 0, 21])     # 菱形蓝花纹地板的HSV值
        # upper_color = np.array([145, 255, 255])  

        lower_color = np.array([73, 23, 41])     # competition zone hsv
        upper_color = np.array([95, 255, 103])  

        img = self.robot.image
        # roi = img[130:320, :]                           # 第一个为y的范围，第二个为x值的范围
        roi = img[120:230, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)          # 从BGR转换到HSV
        mask = cv2.inRange(hsv, lower_color, upper_color)   # inRange()：介于lower/upper之间的为白色，其余黑色
        gaussian = cv2.GaussianBlur(mask, (7, 7), 0)        # 高斯滤波
        edges = cv2.Canny(gaussian, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)
        
        # drawing = np.zeros(roi.shape[:], dtype=np.uint8)
        global turnCnt
        
        if lines is None:
            print('No lines found.')
            if turnCnt == 3:
                return -1
            else:
                turnCnt += 1
        else:
            self.sortLines(lines) # 将识别到的所有直线按y1值从大到小排序
            for i in range(0, np.shape(lines)[0]):
                x1, y1, x2, y2 = lines[i][0]
                tmp = self.get_degree(x1, y1, x2, y2)
                if (tmp == 1):
                    # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    # cv2.imwrite('/home/lemon/robot/'+'originalImg_right' + str(cnt) + '.jpg', self.robot.image)
                    # cv2.imwrite('/home/lemon/robot/'+'edgeImg_right' + str(cnt) + '.jpg', drawing)
                    self.robot.TurnRight()
                    if degreeOnly == 1:
                        turnCnt += 1
                    break
                elif tmp == 2:
                    # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    # cv2.imwrite('/home/lemon/robot/'+'originalImg_left' + str(cnt) + '.jpg', self.robot.image)
                    # cv2.imwrite('/home/lemon/robot/'+'edgeImg_left' + str(cnt) + '.jpg', drawing)
                    if degreeOnly == 1:
                        turnCnt += 1                    
                    self.robot.TurnLeft()
                    break
                elif tmp == 3:
                    print('Parallel to line')
                    if degreeOnly == 0: # 使用Parallel_Check全部功能
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        # cv2.imwrite('/home/lemon/robot/'+'originalImg_para' + str(cnt) + '.jpg', self.robot.image)
                        # cv2.imwrite('/home/lemon/robot/'+'edgeImg_para' + str(cnt) + '.jpg', drawing)
                        distToEdge = self.calcuDistance(x1, y1, x2, y2)
                        print('dist to edge: ' + str(distToEdge ))
                        if distToEdge > 325: # original 265
                            self.robot.Forwark(1)
                        elif distToEdge < 275: # original 255
                            self.robot.MoveBack(1)
                        else:
                            return 3 # 距离合适, 开始向左平移过door
                        break # 如果机器人前后位置不合适(没执行上一行的return 3), 则退出本次Parallel_Check
                    else:
                        return 3               
                elif tmp == -1:
                    print('No qualified line')
                    if degreeOnly == 0: # 使用Parallel_Check全部功能
                        if cnt == 4: # 增强程序鲁棒性, 如果一直找不到合适直线, 则向右转, 因为目前机器人翻过sill后都向左偏
                            print('Robust action Triggered!!')
                            self.robot.TurnRight(2)
                        else:
                            continue
                    else:               # 不进行前后调整, 找不到合适直线就退出本函数
                        return -1
                    
        # cv2.imshow('edges', edges)
        # cv2.waitKey(1)

    def section_door_Better_v1_1(self):
        '''
               description: 翻过sill并后退5步后进入该函数, 该函数负责通过门
                   依据翻过sill后正对的直线, 调整机器人的角度以及据该直线的距离, 然后横向通过door
                   通过door的时候，平移6次进行一次平行校正，校正时不调整前后距离
                   通过门后,机器人将向左旋转一次, 准备进入下一个关卡
               '''
        # Get robot into right positon and angel
        k = 0
        while True:
            exitCode = self.Parallel_check(cnt=k)  # make the robot parallel to the line
            # exitCode = self.Parallel_check()
            time.sleep(0.5)
            k = 0 if k == 4 else k + 1  # k == 4时进行一次cnt的reset
            if exitCode == 3:
                break

        print('In position, start to move left')
        # Across the door sideways
        global turnCnt
        turnCnt = 0
        for i in range(1, 11):
            print('Round: ' + str(i))
            # self.robot.MoveLeft(2)   # move left to get through door
            self.robot.testLeft()
            time.sleep(1)
            if i % 3 == 0:
                while True:
                    time.sleep(0.5)
                    exitCode = self.Parallel_check(degreeOnly=1)  # 不进行前后移动, 只转向
                    if exitCode == 3:
                        break
                    elif exitCode == -1:
                        break
            if turnCnt >= 5:
                self.robot.MoveBack(1)
                turnCnt = 0  # reset
        self.robot.MoveBack(1)
        self.robot.MoveBack(1)
        self.robot.TurnLeftBig()  # after crossing the door, turn to next section
        self.robot.MoveBack(1)
        self.robot.TurnLeft(3)
        

    def getdegree_ForPits(self, x1,y1,x2,y2):
        '''
        version: 1.0.1
        description: 辅助函数, 检查该直线的斜率
        '''
        if(x1-x2==0):
            return -1
        k = float(y1-y2) / float(x1-x2)#没有转的
        print('slope = ' + str(k))
        '''if 0.1 <= k<= 0.4:     #斜率为正, 且满足条件
            return 1
        elif -0.4 <= k <= -0.1: #斜率为负, 且满足条件
            return 2'''
        if 0.1 <= k:     #斜率为正, 且满足条件
            return 1
        elif k <= -0.1: #斜率为负, 且满足条件
            return 2
        elif -0.1 <= k <= 0.1:  #机器人与眼前直线基本平行
            return 3
        else:                   #没有找到有效范围内的直线
            return -1
    
    def findParallel(self):
        '''
        version: 1.1.1
        In Working
        description: 这次尝试不以pits的直线作为对齐的基准, 而是以door所在的地板的直线作为基准 
        '''    
        # lower_color = np.array([80, 40, 75])      # 在实验室pits的HSV值ver1
        # upper_color = np.array([115, 215, 255])
        
        # lower_color = np.array([50, 50, 75])      # 在实验室pits的HSV值ver2
        # upper_color = np.array([910, 215, 255])   # 第一个应该是90??

        lower_color = np.array([76, 78, 49])
        upper_color = np.array([170, 255, 255])       

        img = self.robot.image
        roi = img[120:320, :]                           # 第一个为y的范围，第二个为x值的范围

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)          # 从BGR转换到HSV
        mask = cv2.inRange(hsv, lower_color, upper_color)   # inRange()：介于lower/upper之间的为白色，其余黑色
        gaussian = cv2.GaussianBlur(mask, (7, 7), 0)        # 高斯滤波
        edges = cv2.Canny(gaussian, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)
        
        # drawing = np.zeros(roi.shape[:], dtype=np.uint8)

        if lines is None:
            print('No lines found.')
        else:
            x1, y1, x2, y2 = lines[0][0]
            k = math.fabs(float(y1-y2) / float(x1-x2))#没有转的
            xx1,yy1,xx2,yy2 = x1, y1, x2, y2
            for i in range(1, np.shape(lines)[0]):
                x1, y1, x2, y2 = lines[i][0]
                if (x1-x2) == 0:
                    return 1
                else:
                    k1 = math.fabs(float(y1-y2) / float(x1-x2))#没有转的
                    if k1 < k:
                        k = k1
                        xx1,yy1,xx2,yy2 = x1, y1, x2, y2
                    
            tmp = self.getdegree_ForPits(x1, y1, x2, y2) # 这个斜率的允许范围也需要修改
            if (tmp == 1):
                cv2.line(edges, (xx1, yy1), (xx2, yy2), (0, 255, 0), 1)
                self.robot.TurnRight()
                return 2
            elif tmp == 2:
                cv2.line(edges, (xx1, yy1), (xx2, yy2), (0, 255, 0), 1)
                self.robot.TurnLeft()
                return 2
            elif tmp == 3:
                print('Parallel to line')
                cv2.line(edges, (xx1, yy1), (xx2, yy2), (0, 255, 0), 1)
                return 1 # 距离合适, 开始向左平移过door
            elif tmp == -1:
                print('No qualified line')
                return -1
        # cv2.imshow('edges', edges)
        # cv2.waitKey(1)

    def findParallel_level2(self):#找平行线修正方向的函数（主函数根据返回值判断要不要更改标志位）
        '''return -1是找不到线
           return 1是已经找齐了可以直接走了
           return 2是需要右转
           return 3是需要左转
        '''
        lower_color = np.array([76, 78, 49])
        upper_color = np.array([170, 255, 255])       

        img = self.robot.image
        roi = img[120:320, :]                           # 第一个为y的范围，第二个为x值的范围

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)          # 从BGR转换到HSV
        mask = cv2.inRange(hsv, lower_color, upper_color)   # inRange()：介于lower/upper之间的为白色，其余黑色
        gaussian = cv2.GaussianBlur(mask, (7, 7), 0)        # 高斯滤波
        edges = cv2.Canny(gaussian, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)

        if lines is None:
            print('No lines found.')
            return -1#找不到线
        else:#如果找到线了就要选取斜率最小的线，并且符合设置的阈值范围的线
            x1, y1, x2, y2 = lines[0][0]#第一条直线
            if float(x1-x2) == 0:
                return 1#如果有直线是完全水平的，说明已经调的很正了，就可以直接走了
            k_min = math.fabs(float(y1-y2) / float(x1-x2))#没有转的
            xx1,yy1,xx2,yy2 = x1, y1, x2, y2#记录当前的xy值
            for i in range(1, np.shape(lines)[0]):
                x1, y1, x2, y2 = lines[i][0]
                if float(x1-x2) == 0:
                    return 1#如果有直线是完全水平的，说明已经调的很正了，就可以直接走了
                else:
                    k = math.fabs(float(y1-y2) / float(x1-x2))#没有转的
                    if k_min > k:#如果发现了新的斜率更小的线，就把最小的跟新掉
                        k_min = k
                        xx1,yy1,xx2,yy2 = x1, y1, x2, y2
            #for循环结束的时候，k_min中存储的应该是最小的斜率了

            if k_min > 0.5 or k_min < -0.5:
                return -1
            tmp = self.getdegree_ForPits_level2(xx1, yy1, xx2, yy2) 
            if (tmp == 1):
                cv2.line(edges, (xx1, yy1), (xx2, yy2), (0, 255, 0), 1)
                #self.robot.TurnRight()
                return 2#需要右转
            elif tmp == 2:
                cv2.line(edges, (xx1, yy1), (xx2, yy2), (0, 255, 0), 1)
                #self.robot.TurnLeft()
                return 3#需要左转
            elif tmp == 3:
                print('Parallel to line')
                cv2.line(edges, (xx1, yy1), (xx2, yy2), (0, 255, 0), 1)
                return 1 # 可以直接走了这个就
            elif tmp == -1:
                print('No qualified line')
                return -1
        # cv2.imshow('edges', edges)
        # cv2.waitKey(1)

    def getdegree_ForPits_level2(self, x1,y1,x2,y2):
            '''
            version: 1.0.2
            description: 辅助函数, 检查该直线的斜率
            '''
            if float(x1-x2)==0:
                return -1
            k = float(y1-y2) / float(x1-x2)#没有转的
            print('slope = ' + str(k))
            if 0.03 <= k:     #斜率为正, 且满足条件
                return 1
            elif k <= -0.03: #斜率为负, 且满足条件
                return 2
            elif -0.03 <= k <= 0.03:  #机器人与眼前直线基本平行
                return 3
            else:                   #没有找到有效范围内的直线
                return -1    
    
    def pits_debug_level2(self):
        flag1 = 0#标志位，是否已经经历过了看不到pits时的转向
        flag2 = 0#标志位，是否走到了pits的偏左边的位置
        flag3 = 0#cannot find pits before flag2
        count_parallel = 0#标志位，记录对正进行到了第几次
        count_step = 0
        count_sleep = 0
        #count_noline = 0
        count_nopits_noflag2 = 0
        not_found = 0
        #首先这里是转然后检测能不能看到pits
        while True:
            targets = self.robot.detect_object()#赋值过来的字典
            bridge = targets["bridge"]#数组
            pits = targets["danger"]
            sill = targets['sill']
        
            #这个是最新拍摄的最合适的pits的hsv值，之后比赛现场肯定是要调整的
            lower_color = np.array([76, 78, 49])
            upper_color = np.array([170, 255, 255])

            img = self.robot.image
            # roi = img[40:280, 15:300]   # 第一个为y的范围，第二个为x值的范围（看需要截取多少）
            roi = img   #先不截图了
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)          # 从BGR转换到HSV
            mask = cv2.inRange(hsv, lower_color, upper_color)   # inRange()：介于lower/upper之间的为白色，其余黑色
            gaussian = cv2.GaussianBlur(mask, (7, 7), 0)        # 高斯滤波
            edges = cv2.Canny(gaussian, 50, 150)

            if len(pits) == 0 and flag1 == 0:#如果没有检测到pits，那就先小转一下（在没有经历过找pits转向之前先进行转向）
                #（这个函数的问题是可能在移动的过程中再次检测不出pits了）
                #self.robot.TurnLeftBig()
                self.robot.small_step()
                #self.robot.TurnLeft()
                #self.robot.TurnLeft()
                #self.robot.TurnLeft()
        
            '''if len(sill) > 0 and flag2 == 0:
                self.robot.TurnLeft()
                print("sill")'''
               
            '''if len(pits) == 0 and flag2 == 0:
                self.robot.TurnRight()
                if count_nopits_noflag2 == 2:
                    print("nopits,noflag2")
                    self.robot.Forwark_half()
                    for _ in range(3):
                        self.robot.MoveRight()
                        time.sleep(0.5)
                    break
                count_nopits_noflag2 += 1'''
            if len(pits) == 0 and flag3 == 1:
                #self.robot.TurnRight()
                self.robot.small_step()
                print("not found")
                if not_found == 1:
                    flag2 = 1
                    count_parallel = 2
                    self.robot.TurnLeft()
                    self.robot.TurnLeft()
                not_found += 1
                       
            if len(pits) > 0 and flag2 == 0:#这块还是能检测到pits的时候，并且没有走到pits偏左侧的位置
                flag1 = 1
                flag3 = 1
                #能找到pits说明已经经历过转向了，之后不能再转向了
                #首先是这里要进行方向的修正（所以要在这里去找直线，然后对正）
                #因为能检测到pits了，所以这个时候的大方向应该是对正的了
                #这里最开始的设想是需要用循环，因为如果不是循环可能对正一次就结束了，但实际上的效果还是不理想的
                #但是如果是循环的话可能没办法更新图像了，就可能一直在转了
                if count_parallel == 0:#如果没有转到位的话就要一直转
                    pCheck = self.findParallel_level2()
                    if pCheck == -1:
                        continue
                    elif pCheck == 1:
                        count_parallel = 1#转的已经结束了
                        continue
                    elif pCheck == 2:
                        self.robot.TurnRight()
                        print("parallel")
                    elif pCheck == 3:
                        self.robot.TurnLeft()
                        print("parallel")

                x1,y1,x2,y2 = pits[0][:4]#（0，0）是x1，（0，1）是y1，（0，2）是x2，（0，3）是y3
                distance = self.h - y2#机器人到pits的距离
                bias,turn = self.analyze_target(pits[0][:4],"danger")

                #print("bias:",bias)
                if distance > self.pit_min_distance:#距离还没到的话就往前走
                    self.robot.StepForward()
                    print("<distance")
                    #self.robot.small_step()
                
                if  bias < self.pit_bias and distance < self.pit_min_distance:#在距离调整到小于那个区间的时候，就可以开始往左，调整到左边的位置来过这个回型区域
                    self.robot.MoveLeft()
                    print("meidaoweizhi")
                    #print("11111111111111111111111111111111111111")
                    time.sleep(0.5)
                    if count_parallel == 1:#如果没有转到位的话就要一直转
                        pCheck = self.findParallel_level2()
                        if pCheck == -1:
                            continue
                        elif pCheck == 1:
                            #count_parallel = 2#转的已经结束了
                            #flag2 = 1
                            continue
                        elif pCheck == 2:
                            self.robot.TurnRight()
                            print("parallel")
                        elif pCheck == 3:
                            self.robot.TurnLeft()
                            print("parallel")
                    #print(11111)
                elif bias > self.pit_bias:#能检测到坑，但是已经在坑比较靠左边的地方了
                    #if distance > self.pit_min_distance:#距离还没到的话就往前走
                        #self.robot.StepForward()
                    #已经处在偏左的位置上了，这里需要重新调整对正
                    if count_parallel == 1:#如果没有转到位的话就要一直转
                        pCheck = self.findParallel_level2()
                        if pCheck == -1:
                            continue
                        elif pCheck == 1:
                            count_parallel = 2#转的已经结束了
                            flag2 = 1
                            continue
                        elif pCheck == 2:
                            self.robot.TurnRight()
                            print("parallel")
                        elif pCheck == 3:
                            self.robot.TurnLeft()
                            print("parallel")

            if flag2 == 1:#这里是实际上已经走到了pits偏左的位置
                print("in flag2!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print('count_parallel: ' + str(count_parallel))
                flag3 = 2
                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70,minLineLength=20, maxLineGap=500)
                # print(type(lines))
                if lines is None:
                    #continue
                    print("noline1111111")
                    self.robot.StepForward()
                    #self.robot.small_step()
                    count_sleep += 1
                    if count_sleep == 4:
                        #self.robot.small_step(2)
                        self.robot.StepForward()
                        self.robot.StepForward()
                        time.sleep(0.5)
                        self.robot.MoveRight()
                        self.robot.MoveRight()
                        time.sleep(1)
                        '''for _ in range(2):
                            self.robot.MoveRight()
                            time.sleep(0.5)
                            #self.robot.small_step()
                            self.robot.StepForward()
                            time.sleep(0.5)
                        #self.robot.TurnRight(2)'''
                        break
                else:
                    # 3.1将检测的线画出来
                    m1 = -1
                    m2 = -1
                    x = 160
                    y = 0
                    for i in range(0,np.shape(lines)[0]):
                    #for line in lines:
                        x1, y1, x2, y2 = lines[i][0]
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1, lineType=cv2.LINE_AA)
                        tmp = self.slope_choose(x1,y1,x2,y2)
                        #找到两条直线
                        if(tmp==1):#斜率是正的
                            if(m1==-1):#m1是给左边的线的
                                # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)#这里要单独存一下(x1, y1), (x2, y2)，因为后面要计算点到直线的距离（可能）
                                #dis = self.calcuDistance(x1, y1, x2, y2)
                                print("(x1, y1), (x2, y2):"+str(x1)+","+str(y1)+","+str(x2)+","+str(y2)+"正")
                                m1 = i
                        if(tmp==2):#斜率是负的
                            if(m2==-1):#m2是给右边的线的
                                # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                                dis = self.calcuDistance(x1, y1, x2, y2)
                                print("(x1, y1), (x2, y2):"+str(x1)+","+str(y1)+","+str(x2)+","+str(y2)+"负")
                                m2 = i

                    # print(np.shape(lines))
                    # print(lines)
                    #x, y = getCrossPoint(lines[0][0], lines[1][0]) # 求交点
                    #第二种情况（只能看到左边的直线，也就是斜率是正的）
                    if(m1==-1 and m2!=-1):
                        print("Left")
                        if(dis>200):#大于设想的距离就向左走
                            self.robot.MoveLeft()
                            #print("2222222222222222222222222222222222222")
                            time.sleep(0.5)
                        else:#小于那个距离就向右转
                            self.robot.TurnRight()
                    #第一种情况（只能看到右边的直线）
                    elif(m1!=-1 and m2==-1):
                        print("Right")
                        #if(dis>250):
                            #self.robot.MoveRight()
                            #time.sleep(0.5)
                        #else:
                        self.robot.TurnLeft()
                    #第三种情况（能看到两条直线，这里直接用独木桥的代码就ok了，能看到两条直线的部分）
                    elif(m1!=-1 and m2!=-1):
                        print("Both")
                        #x, y = getCrossPoint(lines[m1][0], lines[m2][0]) # 求交点
                        x_mid=self.bridge_mid(lines[m1][0], lines[m2][0])
                        #cv2.circle(drawing, center = (int(x_mid), 320), radius = 5, color = (0, 0, 255), thickness = 0)
                        #print(x_mid)
                        bias = x_mid - 160
                        if bias >= 25: # 25
                        # self.robot.TurnLeft()
                            self.robot.MoveRight()
                            time.sleep(0.5)
                        elif bias <= -25: # 25
                        # self.robot.TurnRight()
                            self.robot.MoveLeft()
                            #print("33333333333333333333333333333")
                            time.sleep(0.5)
                        else:
                            self.robot.StepForward()
                            #self.robot.small_step()
                    #第四种情况
                    elif(m1==-1 and m2==-1):#两条直线都看不到
                        print("None")
                        if count_parallel == 2:#如果没有转到位的话就要一直转
                            pCheck = self.findParallel_level2()
                            if pCheck == -1:
                                #if count_noline < 2:
                                    #self.robot.StepForward()
                                #self.robot.Forwark_one()
                                #self.robot.StepForward()
                                #else:
                                print("noline2222222")
                                self.robot.StepForward()
                                self.robot.StepForward()
                                time.sleep(0.5)
                                self.robot.MoveRight()
                                self.robot.MoveRight()
                                time.sleep(1)
                                #self.robot.Forwark_half()
                                #self.robot.small_step(2)
                                '''for _ in range(2):
                                    self.robot.MoveRight()
                                    time.sleep(0.5)
                                    #self.robot.small_step()
                                    self.robot.StepForward()
                                    time.sleep(0.5)'''
                                #self.robot.TurnRight(2)
                                break
                                #count_noline += 1
                            elif pCheck == 1:
                                #self.robot.Forwark(1)
                                self.robot.StepForward()
                                #self.robot.small_step()
                                if count_step == 3:
                                    count_parallel = 3#转的已经结束了
                                count_step += 1
                                continue
                            elif pCheck == 2:
                                self.robot.TurnRight()
                            elif pCheck == 3:
                                self.robot.TurnLeft()
                        if count_parallel == 3:
                            #self.robot.Forwark_half()
                            #self.robot.small_step(2)
                            self.robot.StepForward()
                            self.robot.StepForward()
                            time.sleep(0.5)
                            self.robot.MoveRight()
                            self.robot.MoveRight()
                            time.sleep(1)
                            '''for _ in range(2):
                                self.robot.MoveRight()
                                time.sleep(0.5)
                                #self.robot.small_step()
                                self.robot.StepForward()
                                time.sleep(0.5)'''
                            #self.robot.TurnRight(2)
                            break
    
    def pits_debug(self):
        '''
        version: 1.1.1
        In Working
        description: try to get through pits, 首先使用door所在的地板进行机器人角度的校准
        '''    
        flag = 0
        flag1 = 0
        '''while True:  # first, make robot parallel to pits, then move to it
            pCheck = self.findParallel()
            if pCheck == 1:
                print('Parallel check finished, find pits')
                break
            if pCheck == -1:
                print('Parallel check failed!!')
                break'''
            
        while True:
            targets = self.robot.detect_object()#赋值过来的字典
            bridge = targets["bridge"]#数组
            pits = targets["danger"]    

            # 机器人视角中的绿色独木桥 hsv范围（感觉这个也是需要hsv去检测颜色的）
            #lower_color = np.array([50, 45, 33])
            #upper_color = np.array([100, 255, 255])
            '''
            lower_color = np.array([80, 40, 75])      # 在实验室pits的HSV值ver1
            upper_color = np.array([115, 215, 255])
        
            lower_color = np.array([50, 50, 75])      # 在实验室pits的HSV值ver2
            upper_color = np.array([90, 215, 255])'''
            
            lower_color = np.array([76, 78, 49])
            upper_color = np.array([170, 255, 255])
            # img = cv2.imread()#读当前拍摄到的照片（）里面需要补一下
            img = self.robot.image
            # roi = img[40:280, 15:300]   # 第一个为y的范围，第二个为x值的范围（看需要截取多少）
            roi = img
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)          # 从BGR转换到HSV
            mask = cv2.inRange(hsv, lower_color, upper_color)   # inRange()：介于lower/upper之间的为白色，其余黑色
            gaussian = cv2.GaussianBlur(mask, (7, 7), 0)        # 高斯滤波
            edges = cv2.Canny(gaussian, 50, 150)
            
            if len(pits) == 0 and flag == 0:
                #self.robot.TurnLeftBig()
                self.robot.TurnLeft()
                self.robot.TurnLeft()
                self.robot.TurnLeft()

            if len(pits) > 0 and flag == 0:#这块还是能检测到pits的时候
                while flag1 == 0:  # first, make robot parallel to pits, then move to it
                    pCheck = self.findParallel()
                    if pCheck == 1:
                        print('Parallel check finished, find pits')
                        flag1 = 1
                        break
                    if pCheck == -1:
                        print('Parallel check failed!!')
                        break
                #detect_cnt = 0
                x1,y1,x2,y2 = pits[0][:4]#（0，0）是x1，（0，1）是y1，（0，2）是x2，（0，3）是y3
                distance = self.h - y2#机器人到pits的距离
                
                #print(distance)
                bias,turn = self.analyze_target(pits[0][:4],"danger")
                #print("bias:",bias)
                if distance > self.pit_min_distance:#距离还没到的话就往前走
                    self.robot.StepForward()
                    
                if  bias < self.pit_bias and distance < self.pit_min_distance:#在距离调整到小于那个区间的时候，就可以开始往左，调整到左边的位置来过这个回型区域
                    self.robot.MoveLeft()
                    time.sleep(0.5)
                    #print(11111)
                if bias > self.pit_bias:#能检测到坑，但是已经在坑比较靠左边的地方了
                    flag = 1
                    '''# 3.统计概率霍夫线变换
                    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70,minLineLength=20, maxLineGap=500)
                    print(type(lines))
                    if lines is None:
                        continue
                        #print('no lines found.')
                    else:
                        # 3.1将检测的线画出来
                        m1 = -1
                        m2 = -1
                        x = 160
                        y = 0
                        for i in range(0,np.shape(lines)[0]):
                        #for line in lines:
                            x1, y1, x2, y2 = lines[i][0]
                            # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1, lineType=cv2.LINE_AA)
                            tmp = self.slope_choose(x1,y1,x2,y2)
                            #找到两条直线
                            if(tmp==1):#斜率是正的
                                if(m1==-1):#m1是给左边的线的
                                    # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)#这里要单独存一下(x1, y1), (x2, y2)，因为后面要计算点到直线的距离（可能）
                                    
                                    print("(x1, y1), (x2, y2):"+str(x1)+","+str(y1)+","+str(x2)+","+str(y2)+"正")
                                    m1 = i
                            if(tmp==2):#斜率是负的
                                if(m2==-1):#m2是给右边的线的
                                    # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                                    dis = self.calcuDistance(x1, y1, x2, y2)
                                    print("(x1, y1), (x2, y2):"+str(x1)+","+str(y1)+","+str(x2)+","+str(y2)+"负")
                                    m2 = i

                        # print(np.shape(lines))
                        # print(lines)
                        #x, y = getCrossPoint(lines[0][0], lines[1][0]) # 求交点
                        #第二种情况（只能看到左边的直线，也就是斜率是正的）
                        if(m1==-1 and m2!=-1):
                            if(dis>10):#大于设想的距离就向左走
                                self.robot.MoveLeft()
                            else:#小于那个距离就向右转
                                self.robot.TurnRight()
                        #第一种情况（只能看到右边的直线）
                        elif(m1!=-1 and m2==-1):
                            self.robot.TurnLeft()
                        #第三种情况（能看到两条直线，这里直接用独木桥的代码就ok了，能看到两条直线的部分）
                        elif(m1!=-1 and m2!=-1):
                            #x, y = getCrossPoint(lines[m1][0], lines[m2][0]) # 求交点
                            x_mid=self.bridge_mid(lines[m1][0], lines[m2][0])
                            #cv2.circle(drawing, center = (int(x_mid), 320), radius = 5, color = (0, 0, 255), thickness = 0)
                            #print(x_mid)
                            bias = x_mid - 160
                            if bias >= 20: # 25
                            # self.robot.TurnLeft()
                                self.robot.MoveRight()
                            if bias <= -20: # 25
                            # self.robot.TurnRight()
                                self.robot.MoveLeft()
                        #第四种情况
                        elif(m1==-1 and m2==-1):#两条直线都看不到
                            #直走
                            self.robot.Forwark(1)'''
            if flag ==1:#len(pits) == 0:#检测不到pits了
                '''while flag1 == 1:  # first, make robot parallel to pits, then move to it
                    pCheck = self.findParallel()
                    if pCheck == 2:
                        continue
                    if pCheck == 1:
                        print('Parallel check finished, find pits')
                        flag1 = 2
                        break
                    if pCheck == -1:
                        print('Parallel check failed!!')
                        break
                    if flag1 == 2:
                        break'''
                #self.robot.Forwark(1)
                # 3.统计概率霍夫线变换
                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70,minLineLength=20, maxLineGap=500)
                print(type(lines))
                if lines is None:
                    continue
                    #print('no lines found.')
                else:
                    # 3.1将检测的线画出来
                    m1 = -1
                    m2 = -1
                    x = 160
                    y = 0
                    count_step = 0
                    for i in range(0,np.shape(lines)[0]):
                    #for line in lines:
                        x1, y1, x2, y2 = lines[i][0]
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1, lineType=cv2.LINE_AA)
                        tmp = self.slope_choose(x1,y1,x2,y2)
                        #找到两条直线
                        if(tmp==1):#斜率是正的
                            if(m1==-1):#m1是给左边的线的
                                # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)#这里要单独存一下(x1, y1), (x2, y2)，因为后面要计算点到直线的距离（可能）
                                #dis = self.calcuDistance(x1, y1, x2, y2)
                                print("(x1, y1), (x2, y2):"+str(x1)+","+str(y1)+","+str(x2)+","+str(y2)+"正")
                                m1 = i
                        if(tmp==2):#斜率是负的
                            if(m2==-1):#m2是给右边的线的
                                # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                                dis = self.calcuDistance(x1, y1, x2, y2)
                                print("(x1, y1), (x2, y2):"+str(x1)+","+str(y1)+","+str(x2)+","+str(y2)+"负")
                                m2 = i

                    # print(np.shape(lines))
                    # print(lines)
                    #x, y = getCrossPoint(lines[0][0], lines[1][0]) # 求交点
                    #第二种情况（只能看到左边的直线，也就是斜率是正的）
                    if(m1==-1 and m2!=-1):
                        print("Left")
                        if(dis>300):#大于设想的距离就向左走
                            self.robot.MoveLeft()
                            time.sleep(0.5)
                        else:#小于那个距离就向右转
                            self.robot.TurnRight()
                    #第一种情况（只能看到右边的直线）
                    elif(m1!=-1 and m2==-1):
                        print("Right")
                        self.robot.TurnLeft()
                    #第三种情况（能看到两条直线，这里直接用独木桥的代码就ok了，能看到两条直线的部分）
                    elif(m1!=-1 and m2!=-1):
                        print("Both")
                        #x, y = getCrossPoint(lines[m1][0], lines[m2][0]) # 求交点
                        x_mid=self.bridge_mid(lines[m1][0], lines[m2][0])
                        #cv2.circle(drawing, center = (int(x_mid), 320), radius = 5, color = (0, 0, 255), thickness = 0)
                        #print(x_mid)
                        bias = x_mid - 160
                        if bias >= 25: # 25
                        # self.robot.TurnLeft()
                            self.robot.MoveRight()
                            time.sleep(0.5)
                        elif bias <= -25: # 25
                        # self.robot.TurnRight()
                            self.robot.MoveLeft()
                            time.sleep(0.5)
                        else:
                            self.robot.Forwark(1)
                    #第四种情况
                    elif(m1==-1 and m2==-1):#两条直线都看不到
                        print("None")
                        #直走
                        #self.robot.Forwark_one()
                        self.robot.Forwark(1)
                        count_step += 1
                        if count_step == 4:
                            self.robot.Forwark_one()
                            break
    
    # section 3 door 
    def section_door(self):
        '''
        version: 0.0
        description: 原过门代码, 没有对门的避障操作, 假设门的下一关是bridge, 但该训练模型几乎识别不到bridge, 因此无法使用
        '''        
        print("section_door")
        # self.robot.switch_camera(False) # switch camera to head
        cv2.imwrite('/home/lemon/robot/'+'Img' + '.jpg', self.robot.image)
        while True:
            # cv2.imshow('cam', self.robot.image)
            # cv2.waitKey(1)
            
            targets = self.robot.detect_object()
            obstracles = targets['danger']
        
            if len(obstracles) == 0:
                self.robot.TurnLeftBig()
                self.robot.StepForward()
                self.robot.StepForward()
                time.sleep(3)
            else:
                break
        while True:
            image = self.robot.image
            targets = self.robot.detect_object()
            obstracles = targets['danger']
            objectName = 'danger'
            doors = targets['door']
            if len(obstracles) == 0:
                obstracles = targets['bridge']
                objectName = 'bridge'
                if len(obstracles) == 0:
                    obstracles == targets['boom']
                    objectName = 'boom'
            """
            if len(doors) > 0:
                obstracles.sort(key = lambda x:x[4],reverse = True)
                
                x1,y1,x2,y2,_ = obstracles[0]
                
                door1_center = [(x1+x2)//2,(y1+y2)//2]
                #x1,y1,x2,y2,_ = obstracles[1]
                #door2_center = [(x1+x2)//2,(y1+y2)//2]
                
                print(f"door1 = {door1_center},")
            """
            if len(obstracles) > 0:
                """
                if len(doors) > 1:
                    #doors = doors[:2]

                    center = [(doors[0][0] + doors[0][2])//2, (doors[1][0] + doors[1][2])//2]
                    
                    if center[0] > center[1]:#is there any latency problem?
                        left_door, right_door = doors[1], doors[0]
                    else:
                        left_door, right_door = doors[0], doors[1]

                    left_image = image[left_door[1]:left_door[3], left_door[0]:left_door[2]]
                    right_image = image[right_door[1]:right_door[3], right_door[0]:right_door[2]]

                    left_image = cv2.Canny(left_image, 10,100)
                    right_image = cv2.Canny(right_image, 10,100)

                    
                    left_lines = cv2.HoughLines(left_image,1,np.pi/180,118)
                    right_lines = cv2.HoughLines(right_image,1,np.pi/180,118)"""

                obstracles.sort(key = lambda x:x[4],reverse = True)
                # bias,turn =self.analyze_target(obstracles[0][:4],"bridge")
                bias,turn =self.analyze_target(obstracles[0][:4],objectName)
                
                print(f"bias = {bias},degree = {turn}")
                
                x1,y1,x2,y2,_ = obstracles[0]
                
                bridge_center = [(x1+x2)//2,(y1+y2)//2]
                bridge_distance = self.h - y2
                print(f"bridge_center = {bridge_center} distance = {bridge_distance}")    
                if bridge_distance < self.bridge_distance and abs(bias) < 10:
                    #self.robot.Forwark_one()
                    #self.robot.Forwark(3)
                    self.section_pit()
                    break
                else:
                    if bias < self.bridge_min_bias :
                        self.robot.TurnLeft()
                    elif bias > self.bridge_max_bias :
                        self.robot.TurnRight()
                        
                    elif bridge_center[0] < self.bridge_min_center and abs(bias) < self.bridge_max_bias:
                        self.robot.MoveLeft()
                    elif bridge_center[0] > self.bridge_max_center and abs(bias) < self.bridge_max_bias:
                        self.robot.MoveRight()
                    else:
                        self.robot.Forwark(3) 
                    #self.robot.TurnL 
                #    self.robot.Turn    
                #    if bridge_distance > self.bridge_distance:
                    
    # section_ball
    def section_ball(self):
        '''
        version: 1.0.1
        In Working
        description: 完成踢球的函数, 会用finalwalk走过第一个板子, 基本到第二块板前会停下, 
            目前调整脚、球、球洞时向右转动过多, 导致丢失球, 无法完成此关任务
        '''        
        print('In Ball kick')
        """
        Two Step:
            first setp : get close to ball & hole,than lower head
            second step : fine turn 
        """     
        """
        targets = self.robot.detect_ball_hole()
        while len(targets["ball"]) == 0:
            self.robot.StepForward()
            targets = self.robot.detect_ball_hole()"""
        
        """
        targets = self.robot.detect_object()
        cnt = 0
        print("first_stage: getting close to ball & hole")
        while targets["total"] == 0:
            print("No target detected")
            targets = self.robot.detect_object()
            self.robot.StepForward()
            cnt += 1
            if cnt > self.waiting_count:
                if self.Debug:
                    raise ValueError("NO object")
                else:
                    self.robot.StepForward()
                    break

        while len(targets["ball"]) == 0:
            self.robot.StepForward()
            targets = self.robot.detect_object()

        bottomCenter = (self.w//2,self.h)

        x1,y1,x2,y2,_ = targets["ball"][0]
        ball_center = [(x1+x2)//2,(y1+y2)//2]
        degree = - int(math.atan((ball_center[0] - bottomCenter[0])/(ball_center[1] -bottomCenter[1]))/math.pi*180)

        for _ in range(int(degree/self.turn_coef)):
            if degree > 0:
                self.robot.TurnRight()
            else:
                self.robot.TurnLeft()"""

        self.robot.Forwark_one()
        """
        cnt = 0
        while len(targets["ball"]) < 1 or len(targets["hole"]) < 1:
            cnt += 1
            if cnt < self.maximum_step_kick:
                self.robot.StepForward()
                targets = self.robot.detect_object()
            else:
                for turnDegree in [-45,45]:
                    pass
        
        
        while True:
            targets = self.robot.detect_object()
            if len(targets['ball']) == 0:
                self.robot.StepForward()
            else:
                break"""
        bottomCenter = (self.w//2,self.h)
        step_cnt1 = 0
        while True:
            time.sleep(0.5)
            targets = self.robot.detect_object()
            if len(targets['ball']) == 0 or len(targets['hole']) == 0:
                self.robot.StepForward()
                step_cnt1 +=1
                if step_cnt1 >=4:
                    print('Leaving section_ball')
                    return
            else:
                
                x1,y1,x2,y2,_ = targets["ball"][0]
                ball_center = [(x1+x2)//2,(y1+y2)//2]
                #ball_size = (x2+y2 - x1 - y1)//2
                x1,y1,x2,y2,_ = targets["hole"][0]
                hole_center = [(x1+x2)//2,(y1+y2)//2]
                dist = math.sqrt((ball_center[0] - bottomCenter[0])**2 + (ball_center[1] - bottomCenter[1])**2)
                degree = - int(math.atan((hole_center[0]- ball_center[0])/(hole_center[1] - ball_center[1]))/math.pi*180)
                
                #bias, turn = self.analyze_target(targets["ball"][0][:4],"ball")
                print(dist)
                #print(bias)
                self.robot.StepForward()
                for _ in range(2):
                    self.robot.MoveLeft()
                """
                for _ in range(int(dist/self.walk_coef)):
                    # self.robot.StepForward()
                    pass"""
                    
                for _ in range(int(degree/self.turn_coef)):
                    self.robot.TurnRight()
                    
                break
                    
                """
                if dist > 150:
                    self.robot.Forwark_one()
                degree = - int(math.atan((hole_center[0]- ball_center[0])/(hole_center[1] - ball_center[1]))/math.pi*180)
                if dist > self.ball_dist:
                    if bias < self.ball_min_bias :
                        self.robot.TurnLeft()
                    elif bias > self.ball_max_bias :
                        self.robot.TurnRight()
                    else:
                        self.robot.StepForward()
                else:
                    break """
                
                # print(f"ball_center = {ball_center}")
                # print(f"distance = {dist},walking{int(dist/self.walk_coef)}")
                # for _ in range(int(dist/self.walk_coef)):
                    #self.robot.StepForward()"""
        bottomCenter = (self.w//2,self.h)

        cnt = 0
                    
        while True:
            time.sleep(0.5)
            targets = self.robot.detect_object()
            #time.sleep(0.2)
            if len(targets["ball"]) < 1: # can't find the ball
                self.robot.MoveBack()
                self.robot.TurnLeft()
                cnt+=1
                if cnt >=3:
                    break
                continue
            '''
            if len(targets["ball"]) > 1:
                print('two object detected')
                print('first ball dir is'+str(targets["ball"][0]))
                print('seccond ball dir is'+str(targets["ball"][1]))
                '''
            if len(targets["hole"]) < 1: # can't find the hole
                self.robot.TurnRight()
                continue
            if len(targets["ball"]) > 1:
                print('two object detected')
                print('first ball dir is'+str(targets["ball"][0]))
                print('seccond ball dir is'+str(targets["ball"][1]))
                if(targets["ball"][0][1]>targets["ball"][1][1]):
                    x1, y1, x2, y2, _ = targets["ball"][0]
                    print('y1'+str(y1))
                else:
                    x1, y1, x2, y2, _ = targets["ball"][1]
                    print('y1'+str(y1))
            else:
                x1, y1, x2, y2, _ = targets["ball"][0]
            ball_center = [(x1+x2)//2, (y1+y2)//2]
            ball_size = (x2+y2 - x1-y1) //2

            x1, y1, x2, y2, _ = targets["hole"][0]
            hole_center = [(x1+x2)//2, (y1+y2)//2]
            print(ball_center[0] - bottomCenter[0])
           
            if ball_center[0] - bottomCenter[0] < self.kick_min_shift_bias: # correct left/right bias
                print(f'bias is {ball_center[0] - bottomCenter[0]}')
                self.robot.MoveLeft()
                continue

            if ball_center[0] - bottomCenter[0] > self.kick_max_shift_bias:
                print(f'bias is {ball_center[0] - bottomCenter[0]}')
                self.robot.MoveRight()
                continue

            distance = bottomCenter[1] - ball_center[1] # get closer to the ball
            print(f"distance = {distance}")
            if distance < self.kick_min_distance_bias:
                print(f'distance = {distance}, too close, steping backward')
                self.robot.MoveBack()
                #cnt += 1
                continue

            degree = -int(math.atan((hole_center[0]-ball_center[0])/(hole_center[1]-ball_center[1]))/math.pi*180) - self.kick_angle_bias # corect the angle bias
            print(f"degree = {degree}")
            if degree > self.kick_max_turn_bias :
                self.robot.TurnRight()
                continue
            if degree < -self.kick_max_turn_bias:
                self.robot.TurnLeft()
           
            if distance > self.kick_max_distance_bias:
                print(f'distance = {distance}, steping forward')
                self.robot.StepForward()
                continue
            """
            if distance < self.kick_max_distance_bias:# and distance > self.kick_min_distance_bias:
                self.robot.StepBackward()
                #self.robot.MoveLeft(2)
                #self.robot.TurnRight(5)
                #self.robot.LeftKill()
                #break
            """
           
               
            self.robot.LeftKill()# kiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiick!
            #self.robot.small_step(2)
            time.sleep(1)
            #self.robot.LeftKill()
            self.robot.TurnLeftBig()
            self.robot.TurnLeftBig()
            break
           
    
    
    def section_stairs(self):
        '''
        version: 1.1
        description: 尚未测试到楼梯
        date:2022/8/18
        '''        
        print('In section_stairs')
        
        while True:     
            targets = self.robot.detect_object()
            obstracles = targets['step']
        
            if len(obstracles) == 0:
                self.robot.TurnLeftBig()#找不到台阶，认为没有对准台阶
            else:
                break
                
        while True:
            targets = self.robot.detect_object()
            
            obstracles = targets['step']  

            if len(obstracles) > 0:
                print('yolo working')       #yolo找到了台阶
                obstracles.sort(key = lambda x:x[4],reverse = True)
                bias,turn =self.analyze_target(obstracles[0][:4],"step")
                
                print(f"bias = {bias},degree = {turn}")     
                x1,y1,x2,y2,_ = obstracles[0]
                
                step_center = [(x1+x2)//2,(y1+y2)//2]       #找到台阶的中心
                step_distance = self.h - y2
                print(f"step_center = {step_center} distance = {step_distance}")
                if step_distance < self.step_distance and abs(bias) < 10:       #识别到台阶并且距离小于步数的距离
                    self.robot.up_stair()
                    break
                else:
                    if bias < self.step_min_bias :      #夹角过大，需要左右转对齐中心
                        self.robot.TurnLeft()
                    elif bias > self.step_max_bias :
                        self.robot.TurnRight()
                        
                    elif step_center[0] < self.step_min_center and abs(bias) < self.step_max_bias:      #不在台阶的中心位置，左右平移对齐中心
                        self.robot.MoveLeft()
                    elif step_center[0] > self.step_max_center and abs(bias) < self.step_max_bias:
                        self.robot.MoveRight()
                    elif step_distance > 150 and abs(bias) < self.step_max_bias:
                        self.robot.Forwark_one()
                        #self.robot.small_step(12)
                        self.robot.up_stair()
                        break
                    else:
                        self.robot.StepForward(1)

            if len(obstracles) == 0:
                print('使用OpenCv进行检测')       #距离台阶过近无法识别到
                img = self.robot.image
                # def Parallel_check():
                lower_color = np.array([11, 92, 78])  # 在实验室pits的HSV值，检测台阶的绿色区域
                upper_color = np.array([84, 219, 179])

                # roi = img[120:320, :]                           # 第一个为y的范围，第二个为x值的范围
                roi = img
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)  # 从BGR转换到HSV
                mask = cv2.inRange(hsv, lower_color, upper_color)  # inRange()：介于lower/upper之间的为白色，其余黑色
                gaussian = cv2.GaussianBlur(mask, (7, 7), 0)  # 高斯滤波
                edges = cv2.Canny(gaussian, 50, 150)
                lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 70, minLineLength=1, maxLineGap=500)

                drawing = np.zeros(roi.shape[:], dtype=np.uint8)
    '''找到对应的直线，用hsv找到对应的台阶直线，找到的情况下根据斜率进行调整，使得机器人与台阶平行，平行后再调整距离，距离合适就可以开始上台阶'''
                if lines is None:
                    print('No lines found.')
                    break
                else:
                    self.sortLines(lines)
                    for i in range(0, np.shape(lines)[0]):
                        x1, y1, x2, y2 = lines[i][0]
                        center = (x1 + x2) // 2
                        tmp = self.get_degree(x1, y1, x2, y2)
                        if tmp != -1:
                            break
                    if tmp == 1:
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        print('self.robot.TurnRight()')
                        self.robot.TurnRight()
                    elif tmp == 2:
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        print('self.robot.TurnLeft()')
                        self.robot.TurnLeft()
                    elif tmp == 3:
                        print('Parallel to line')
                        # cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        print('center' + str(center))
                        if 150 <= center and center <= 170:
                            break
                        elif center < 150:
                            self.robot.MoveLeft()
                        elif center > 170:
                            self.robot.MoveRight()
                    elif tmp == -1:
                        print('No qualified line')
                        break
                        # continue

        self.robot.small_step(3)
        self.robot.up_stair()
           
    def takePhoto(self):
        '''
        version: 1.0
        description: 用于在solver.py中拍照
        '''        
        num = 0
        print('in Photo')
        # self.robot.switch_camera(False) # switch camera to head       
        while True:
            cv2.imshow('Photo', self.robot.image)
            # keyin = cv2.waitKey(1) & 0xFF
            cv2.waitKey(1)
            keyin = str(input('Hi: '))
            if keyin == 's':
                cv2.imwrite('/home/lemon/robot/'+'Img'+ str(num) + '.jpg', self.robot.image)
                num += 1
                print('saved')
    
    def min_turn_degree(self):
        '''
        version: 1.0
        description: 用于检测机器人执行MoveLeft时的旋转角度
        ''' 
        #self.robot.MoveLeft()
        #self.robot.MoveLeft()
        self.robot.TurnRight()
        self.robot.TurnRight()
        time.sleep(1)
        return
    
    # Debug
    def Debug(self):
        if not self.DEBUG:
            print("not in debug mode,pleace check the flag of debug mode")
            return
        while True:
            self.section_grass()
            self.section_pit()
            self.section_mine()
            self.section_door_Better_v1_1() # in use
            self.section_bridge()
            time.sleep(1)
            self.bridge_debug()
            time.sleep(1)
            #self.section_door() # not in use
            self.section_ball() # with pit embedded
            self.section_stairs()

if __name__ == "__main__":
    solver = Solver()
    
    solver.Debug()
 
        


