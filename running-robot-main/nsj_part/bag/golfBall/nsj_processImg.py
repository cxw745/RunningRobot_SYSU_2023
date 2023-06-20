import cv2
import math
import numpy as np
import golfBall.colorRangeNsj as nsj


################获取最大轮廓以及面积#############（包含去除干扰功能）
def getAreaMax_contour(contours, area_min = 75):
    '''
    :@contours 所有轮廓
    :@area_min 轮廓最小值，小于最小值不算
    :功能 返回最大轮廓及其面积
    '''
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours : #历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c)) #计算轮廓面积
        if contour_area_temp > contour_area_max :
            contour_area_max = contour_area_temp
            if contour_area_temp > area_min:  #最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max#返回最大的轮廓
############################################################

#################处理图像函数################################
def process_img(img):
    if(img is not None):
        OrgImg = img
        OrgImg = cv2.GaussianBlur(OrgImg,(3,3),0)
        LAB_img = cv2.cvtColor(OrgImg, cv2.COLOR_BGR2LAB) #将图像转换到LAB空间
        Imask = cv2.GaussianBlur(LAB_img, (3, 3), 0)#高斯滤波，去除噪音
        return OrgImg,Imask
    else:
        return
############################################################

###############孔洞填充#####################################
def fillHole(im_in,x,y): 
	im_floodfill = im_in.copy()
	h, w = im_in.shape[:2]
	mask = np.zeros((h+2, w+2), np.uint8)
	cv2.floodFill(im_floodfill, mask, (0,0), 255)
	im_floodfill_inv = cv2.bitwise_not(im_floodfill)
	im_out = im_in | im_floodfill_inv
	return im_out
###########################################################

###############处理边界图像#############################
def process_edge_img(LAB_img,img,temp = "angle",flag = "left"):
    img_h,img_w = img.shape[:2]
    LAB_img = cv2.GaussianBlur(LAB_img, (3, 3), 0)#高斯滤波，去除噪音
    Imask = cv2.inRange(LAB_img,nsj.color_range['red_floor'][0],nsj.color_range['red_floor'][1])
    ## 去除白色条纹
    Imask = cv2.GaussianBlur(Imask,(3,3),0)
    Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=13)
    Imask = cv2.erode(Imask, None, iterations=13)
    edges = cv2.Canny(Imask,100,500)
    lines = cv2.HoughLines(edges,2,np.pi/180,150)
    if temp == "angle":
        if(lines is not None):
            rho,theta = lines[0][0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
            return theta*180/math.pi
        else:
            if(Imask[img_h//2][img_w//2] == 255):
                return 180
            else:
                return 0
    else:
        if(lines is not None):
            rho,theta = lines[0][0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
            if x1 == x2 : 
                return 10000
            k = (y1-y2)/(x1-x2)
            c = y1-k*x1
            distance = math.fabs(k*img_w//2-img_h+c)/math.sqrt(k*k+1)
            return distance
        else:
            if(Imask[img_h-1][img_w-1] == 255):
                return 10000
            else:
                return 0
#######################################################