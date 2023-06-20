import cv2
import math
import numpy as np
ChestOrg_img=cv2.imread(r"C:\Users\dell\Desktop\shang_wei_ji\floor_ding\capture7.jpg")
color_range = {'blue_stair':[( 16 , 19 , 0 ), ( 255 , 250 , 108 )]}
color = 'blue_stair'
# 得到最大轮廓和对应的最大面积
def getAreaMaxContour1(contours):    # 返回轮廓 和 轮廓面积
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓
def floor():
    global org_img, state, state_sel, step, reset, skip, debug
    global camera_out

    print("/-/-/-/-/-/-/-/-/-进入floor")

    chest_r_width = 640
    chest_r_height = 480
    r_w = chest_r_width
    r_h = chest_r_height

    step = 0
    state_sel = 'floor'
    down_flag0 = 0
    down_flag1 = 0
    move = 0
    while state_sel == 'floor':
        # 分析图像 # chest

        if True:  # 上下边沿

            Corg_img = ChestOrg_img.copy()
            OrgFrame = Corg_img.copy()

            frame = cv2.resize(OrgFrame, (chest_r_width, chest_r_height), interpolation=cv2.INTER_LINEAR)
            frame_copy = frame.copy()
            # 获取图像中心点坐标x, y
            center = []
            # 开始处理图像
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            hsv = cv2.GaussianBlur(hsv, (3, 3), 0)

            Imask = cv2.inRange(hsv, color_range['blue_stair'][0], color_range['blue_stair'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            # opened = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
            # Imask = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
            # Imask = cv2.erode(Imask, None, iterations=2)
            Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)

            _,cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓

            cnt_sum, area_max = getAreaMaxContour1(cnts)  # 找出最大轮廓
            C_percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
            cv2.drawContours(frame, cnt_sum, -1, (255, 0, 255), 3)
            if cnt_sum is not None:
                see = True
                rect = cv2.minAreaRect(cnt_sum)  # 最小外接矩形
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点

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
                    #     cv2.imshow('handling', handling)  # 显示图像
                    #     cv2.waitKey(2)

                bottomcenter_x = (bottom_left[0] + bottom_right[0]) / 2  # 得到bottom中心坐标
                bottomcenter_y = (bottom_left[1] + bottom_right[1]) / 2

                topcenter_x = (top_right[0] + top_left[0]) / 2  # 得到top中心坐标
                topcenter_y = (top_left[1] + top_right[1]) / 2

                bottom_angle = -math.atan(
                    (bottom_right[1] - bottom_left[1]) / (bottom_right[0] - bottom_left[0])) * 180.0 / math.pi
                top_angle = -math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
                if math.fabs(topcenter_x - bottomcenter_x) <= 1:  # 得到连线的角度
                    T_B_angle = 90
                else:
                    T_B_angle = - math.atan(
                        (topcenter_y - bottomcenter_y) / (topcenter_x - bottomcenter_x)) * 180.0 / math.pi

                cv2.drawContours(frame_copy, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上
                cv2.line(frame_copy, (bottom_left[0], bottom_left[1]), (bottom_right[0], bottom_right[1]),
                         (255, 255, 0), thickness=2)
                cv2.line(frame_copy, (top_left[0], top_left[1]), (top_right[0], top_right[1]), (255, 255, 0),
                         thickness=2)
                cv2.line(frame_copy, (int(bottomcenter_x), int(bottomcenter_y)),
                         (int(topcenter_x), int(topcenter_y)), (255, 255, 255), thickness=2)  # T_B_line

                cv2.putText(frame_copy, "bottom_angle:" + str(bottom_angle), (30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                # cv2.putText(frame_copy, "top_angle:" + str(top_angle),(30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                # cv2.putText(frame_copy, "T_B_angle:" + str(T_B_angle),(30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                cv2.putText(frame_copy, "bottom_x:" + str(bottomcenter_x), (30, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                cv2.putText(frame_copy, "bottom_y:" + str(int(bottomcenter_y)), (300, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                cv2.putText(frame_copy, "topcenter_x:" + str(topcenter_x), (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                cv2.putText(frame_copy, "topcenter_y:" + str(int(topcenter_y)), (230, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                # cv2.putText(frame_copy, 'C_percent:' + str(C_percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                # cv2.putText(frame_copy, "step:" + str(step), (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR

                cv2.circle(frame_copy, (int(topcenter_x), int(topcenter_y)), 5, [255, 0, 255], 2)
                cv2.circle(frame_copy, (int(bottomcenter_x), int(bottomcenter_y)), 5, [255, 0, 255], 2)
                cv2.circle(frame_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(frame_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(frame_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(frame_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                frame_copy_small = cv2.resize(frame_copy, (400, 300))
                Imask_small = cv2.resize(Imask, (400, 300))
                cv2.imshow('Chest_Camera', frame_copy_small)  # 显示图像
                cv2.imshow('chest_red_mask', Imask_small)
                cv2.waitKey(10)
if __name__ == '__main__':
    floor()