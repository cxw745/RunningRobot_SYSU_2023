#!/usr/bin/env python3
import math
import cv2 

def get_area_max_contour(contours, threshold):
    """
    获取面积最大的轮廓
    :param contours: 要比较的轮廓的列表
    :param threshold: 轮廓要求的最小面积阈值
    :return: 面积最大的轮廓, 面积最大的轮廓的面积值
    """
    contours = map(lambda x: (x, math.fabs(cv2.contourArea(x))), contours)  # 计算所有轮廓面积
    contours = list(filter(lambda x: x[1] > threshold, contours))
    if len(contours) > 0:
        return max(contours, key = lambda x: x[1])  # 返回最大的轮廓
    else:
        return None, 0


def get_contour_location(contour):
    '''
    获取轮廓的位置，即轮廓底部的点
    :param contour: 要获取位置的轮廓
    :return x, y: 位置 
    '''
    if contour is not None:
        return max(contour, key = lambda x:x[0][1])[0]
    else:
        return None

def dist2(a, b):
    return ((a[0] - b[0])**2) + ((a[1] - b[1])**2)

class line:
    point0 = []
    point1 = []
    def __init__(self, p0, p1):
        self.point0 = p0
        self.point1 = p1

    def mid_point(self):    # 中点
        return (self.point0[0] + self.point1[0]) // 2, (self.point0[1] + self.point1[1]) // 2

    def length2(self):      # 长度的平方
        return math.pow(self.point1[1] - self.point0[1], 2) + math.pow(self.point1[0] - self.point0[0], 2)

    def angle(self):        # 范围(-90, 90] 水平时为0° 逆时针时正增长，顺时针时逆增长
        if self.point0[0] == self.point1[0]:
            return 90
        else:
            return - math.atan((self.point1[1]-self.point0[1]) / (self.point1[0]-self.point0[0])) * 180 / math.pi
