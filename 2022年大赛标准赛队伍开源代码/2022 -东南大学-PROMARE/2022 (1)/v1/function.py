import cv2
import numpy as np
import math
import time
from config import *
import hiwonder.ActionGroupControl as AGC  # 运行动作文件
import hiwonder.Board as Board  # 控制总线电机
import hiwonder.Camera as Camera  # 打开摄像头线程

###########运动###############
# def runAction(actNum, times=1, path="/home/pi/AiNexPro/ActionGroups/"):
#     '''
#     运行动作组
#     :param actNum: 动作组名字 ， 字符串类型
#     :param times: 运行次数，当为0时表示循环
#     :return:
#     '''
# 站立
#AGC.runAction('stand', 1, path)
#time.sleep(2)

###########摄像头云台#################
# setBusServoPulse(id, pulse, use_time)
# id：舵机id;
# pulse：位置;
# use_time：运行时间
# 19号电机：云台左右
# 20电机： 云台上下

# Board.setBusServoPulse(19, 500, 500)  # 19号舵机转到500位置，用时500ms
# time.sleep(0.5)  # 延时时间和运行时间相同

def for_copy(cali_img):
    img_smooth = cv2.GaussianBlur(cali_img, (3, 3), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(img_transform, color_range[color][0], color_range[color][1])
    kernel = np.ones((3, 3), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    cnts, hieracy = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return cnts


def cnts_incolor(img, color, transform = cv2.COLOR_BGR2HSV, kernel=np.ones((3, 3), np.uint8)):
    img_smooth = cv2.GaussianBlur(img, (3, 3), 0)
    img_transform = cv2.cvtColor(img_smooth,transform)
    img_thresh = cv2.inRange(img_transform,color_range[color][0],color_range[color][1])   #color_range
    open = cv2.morphologyEx(img_thresh,cv2.MORPH_OPEN,kernel)
    close = cv2.morphologyEx(open,cv2.MORPH_CLOSE,kernel)
    cnts, hieracy = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return cnts

# 返回大于阈值的最大轮廓及其面积
def get_maxcontour(contours,area_threshold = 1000):
    cnt_max = None
    area_max = 0
    if len(contours) != 0 :
        cnt = max(contours, key=cv2.contourArea)

        if cv2.contourArea(cnt) > area_threshold:
            cnt_max = cnt
            area_max = cv2.contourArea(cnt)

    return cnt_max, area_max

# 返回所有轮廓的总面积
def get_sumcontour(contours):
    area_sum = 0

    if len(contours) != 0 :
        for cnt in contours:
            area_sum += cv2.contourArea(cnt)

    return area_sum

# 线段中点、长度、角度计算
class line:

    point0 = []
    point1 = []

    def __init__(self, p0, p1):
        self.point0 = p0
        self.point1 = p1

    def mid_point(self):
        return (self.point0[0]+self.point1[0])//2, (self.point0[1]+self.point1[1])//2

    #距离的平方
    def length2(self):
        return math.sqrt(math.pow(self.point1[1]-self.point0[1], 2) + math.pow(self.point1[0]-self.point0[0], 2))

    def angle(self):
        if self.point0[0] == self.point1[0]:
            return 90
        else:
            return -math.atan((self.point1[1]-self.point0[1]) / (self.point1[0]-self.point0[0])) * 180 / math.pi


#根据最小外接矩形判断四个顶点
def get_box_vertex(box):
    box_sort = sorted(box, key=lambda x: x[1])
    bottom = box_sort[2:4]
    top = box_sort[0:2]

    bottom_sort = sorted(bottom, key=lambda x: x[0])
    top_sort = sorted(top, key=lambda x: x[0])

    bottom_left = bottom_sort[0]
    bottom_right = bottom_sort[1]

    top_left = top_sort[0]
    top_right = top_sort[1]

    return bottom_left, bottom_right, top_left, top_right

#根据轮廓判断四个顶点
def get_cnt_vertex_top(cnt,hight,width):
    top_left = cnt[0][0]
    top_right = cnt[0][0]

    for c in cnt:  # 遍历找到四个顶点
        if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
            top_left = c[0]
        if (width - c[0][0]) + 1.5 * c[0][1] < (width - top_right[0]) + 1.5 * top_right[1]:
            top_right = c[0]


    return top_left, top_right

def get_cnt_vertex_bottom(cnt,hight,width):
    bottom_left = cnt[0][0]
    bottom_right = cnt[0][0]
    for c in cnt:  # 遍历找到四个顶点
        if c[0][0] + 1.5 * (hight - c[0][1]) < bottom_left[0] + 1.5 * (hight - bottom_left[1]):
            bottom_left = c[0]
        if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
            bottom_right = c[0]

    return bottom_left, bottom_right



#调整中心和角度
def center_adjustment(angle,center_x,angle_thresh,center_x_thresh,angle_scale=3,x_scale=15,max_time=3):
    run_time = 1
    if angle < angle_thresh[0] :
        run_time = math.ceil(abs(angle_thresh[0]-angle)/angle_scale)
        run_time = min(run_time,max_time)
        AGC.runAction('turn_right', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(2)

    elif angle > angle_thresh[1] :
        run_time = math.ceil(abs(angle_thresh[1]-angle)/angle_scale)
        run_time = min(run_time, max_time)
        AGC.runAction('turn_left', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(2)
    elif center_x < center_x_thresh[0] :
        run_time = math.ceil(abs(center_x_thresh[0]-center_x)/x_scale)
        run_time = min(run_time, max_time)
        AGC.runAction('seu_leftgo', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(3)
    elif center_x > center_x_thresh[1]:
        run_time = math.ceil(abs(center_x_thresh[1] - center_x)/x_scale)
        run_time = min(run_time, max_time)
        AGC.runAction('seu_rightgo_little', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(4)


#调整中心和角度-一步
def center_adjustment_one_step(angle,center_x,angle_thresh,center_x_thresh,angle_scale=3,x_scale=10):
    run_time = 1
    if angle < angle_thresh[0] :

        AGC.runAction('turn_right', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(2)
    elif angle > angle_thresh[1] :
        AGC.runAction('turn_left', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(2)
    elif center_x < center_x_thresh[0] :
        AGC.runAction('seu_leftgo', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(3)
    elif center_x > center_x_thresh[1]:
        AGC.runAction('seu_rightgo_little', run_time, action_path)
        time.sleep(1)
        AGC.runAction('stand', 1, action_path)
        time.sleep(4)


#判断当前关卡是否结束，准备进入下一关
def end_to_next(img,percent_thresh,colors=[]):
    roi_area = 0
    end_flag = False

    img_smooth = cv2.GaussianBlur(img, (3, 3), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)

    for color in colors:
        img_thresh = cv2.inRange(img_transform, color_range[color][0], color_range[color][1])
        cnts, hieracy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        roi_area += get_sumcontour(cnts)                 #选择最大轮廓还是全部轮廓

    percent = round(100*roi_area/(img.shape[0]*img.shape[1]))

    if percent > percent_thresh:
        end_flag = True

    return end_flag

#判断3个随机关卡的位置
def flag_choose(img,hole_color='blue',bridge_color='green',floor_color='red'):
    flags = ['hole', 'bridge', 'floor']
    areas =[]

    img_smooth = cv2.GaussianBlur(img, (3, 3), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)

    hole_thresh = cv2.inRange(img_transform, color_range[hole_color][0], color_range[hole_color][1])
    hole_cnts, hieracy = cv2.findContours(hole_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    hole_area = get_sumcontour(hole_cnts)            #选择最大轮廓还是全部轮廓
    areas.append(1.5*hole_area)

    bridge_thresh = cv2.inRange(img_transform, color_range[bridge_color][0], color_range[bridge_color][1])
    bridge_cnts, hieracy = cv2.findContours(bridge_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    bridge_area = get_sumcontour(bridge_cnts)
    areas.append(1.5*bridge_area)

    floor_thresh = cv2.inRange(img_transform, color_range[floor_color][0], color_range[floor_color][1])
    floor_cnts, hieracy = cv2.findContours(floor_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    floor_area = get_sumcontour(floor_cnts)
    areas.append((hole_area+bridge_area+floor_area))

    index = areas.index(max(areas))
    flag_out = flags[index]

    return flag_out


