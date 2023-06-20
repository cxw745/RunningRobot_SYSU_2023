import cv2
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import dlib
import multiprocessing
import time
def ball_recognize():
    # 读入图像
    img = cv.imread('./img/ball2.png')
    # 将图像转为灰度图
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # 将图像从BGR格式转为RGB格式
    rgb_img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    # 复制一份rgb图像，用于在上面绘制检测结果
    new_img = rgb_img.copy()
    # 高斯滤波（可选），去噪声，平滑图像
    # gauss_img = cv.GaussianBlur(rgb_img,ksize=(1,1),sigmaX=10)
    # 对图像进行Canny边缘检测，后面的参数100和400分别为高低阈值
    edges_img = cv.Canny(rgb_img, 100, 400)
    # 使用HoughCircles算法检测图像中的圆形（参数解释见下）
    circle = cv.HoughCircles(edges_img, cv.HOUGH_GRADIENT, dp=1, minDist=100, param1=200, param2=30, minRadius=30,
                             maxRadius=200)
    # 将检测到的结果转换为无符号16位整数类型
    circle = np.uint16(np.around(circle))
    # 输出检测结果
    print(circle)
    # 在new_img上绘制检测结果：对于每一个检测到的圆形，绘制该圆形并标出圆心
    for i in circle[0, :]:
        cv.circle(new_img, (i[0], i[1]), i[2], (255, 0, 0), 10)  # 绘制圆形
        # 标出圆心
        cv.circle(new_img, (i[0], i[1]), 2, (255, 0, 0), 10)

    # 使用Matplotlib库将原图和检测结果在一个窗口中展示出来
    plt.subplot(131)
    plt.imshow(rgb_img)
    plt.title('img')
    plt.axis('off')

    plt.subplot(132)
    plt.imshow(edges_img)
    plt.title('edges')
    plt.axis('off')

    plt.subplot(133)
    plt.imshow(new_img)
    plt.title('rst')
    plt.axis('off')
    plt.show()


def get_ip_camera():
    url = 'http://snow:cxw825@172.26.74.79:8080/video'  # 局域网地址
    cap = cv.VideoCapture(url)  # 获取视频流
    cv.namedWindow("frame", 0)  # 0为可调大小，注意：窗口名必须imshow里面的一窗口名一致
    cv.resizeWindow("frame", 430, 270)  # 设置长和宽
    while cap.isOpened():
        ret, frame = cap.read()  # 逐帧读取 返回第一个参数表示是否返回读取成功 第二个表示读取的帧
        if ret:
            print("success capture")
        # 调整窗口大小
        cv.namedWindow("frame", 0)  # 0可调大小，注意：窗口名必须imshow里面的一窗口名一致
        cv.imshow('frame', frame)  # 现实窗口
        if cv.waitKey(1) & 0xFF == ord('q'):  # 键盘输入q退出
            break
    cap.release()  # 释放
    cv.destroyAllWindows()  # 关闭所有窗口


def color_and_contour():
    # 要注意的是 图像的返回是 位置坐标是(宽,高) 宽对应横向坐标,也即numpy的y 高对应纵向坐标,也即numpy的x
    def draw_contour(event, x, y, flags, param):
        nonlocal DRAWING, ERASE, CONTOUR
        nonlocal img, origin_img
        # 按下左键开始绘画
        if event == cv.EVENT_LBUTTONDOWN:
            DRAWING = True
            img[:100, :400] = origin_img[:100, :400]  # 清空原来的显示内容 下一次显示不重叠
        # 点击右键是橡皮擦功能
        elif event == cv.EVENT_RBUTTONDOWN:
            ERASE = True
        # 左键双击清空
        elif event == cv.EVENT_LBUTTONDBLCLK:
            # img = origin_img 不对
            img = origin_img.copy()
        elif event == cv.EVENT_MOUSEMOVE:
            if DRAWING:
                cv.circle(img, (x, y), 0, (0, 0, 0), -1)
                CONTOUR.append((x, y))
                if len(CONTOUR) > 1:
                    cv.line(img, CONTOUR[-2], CONTOUR[-1], (0, 0, 0), 2)
            if ERASE:
                img[y - 20:y + 20, x - 20:x + 20] = origin_img[y - 20:y + 20, x - 20:x + 20]

        elif event == cv.EVENT_LBUTTONUP:
            # 获得像素点的属性 并且在图像左上角显示
            b, g, r = img[y, x]
            h, s, v = cv.cvtColor(img, cv.COLOR_BGR2HSV)[y, x]
            cv.putText(img, 'B: %d G: %d R: %d' % (b, g, r), (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1,
                       cv.LINE_AA)
            cv.putText(img, 'H: %d S: %d V: %d' % (h, s, v), (20, 50), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1,
                       cv.LINE_AA)

            # 给图形封口
            area, perimeter = 0, 0
            # 能够计算周长和面积
            if len(CONTOUR) > 2:
                cv.line(img, CONTOUR[-1], CONTOUR[0], (0, 0, 0), 2)
                # 获得图形的面积和周长
                contour = np.array(CONTOUR)
                area = cv.contourArea(contour)  # 计算轮廓面积
                perimeter = (cv.arcLength(contour, True))  # 计算轮廓周长
            cv.putText(img, 'Area:%.3f Perimeter:%.3f' % (area, perimeter), (20, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 0, 0), 1, cv.LINE_AA)
            # 画完清空数组
            CONTOUR.clear()
            DRAWING = False
        elif event == cv.EVENT_RBUTTONUP:
            ERASE = False

    # 轮廓数组 存储绘图的点
    CONTOUR = []
    # 两个状态 分别是绘画和擦除
    DRAWING, ERASE, = False, False
    origin_img = cv.imread('./img/cat.png', 1)
    img = origin_img.copy()
    cv.namedWindow('img', 0)
    cv.setMouseCallback('img', draw_contour)
    while True:
        cv.imshow('img', img)
        if cv.waitKey(1) & 0xFF == 27:
            break
    cv.destroyAllWindows()

def multi_tracker():
    OPENCV_OBJECT_TRACKERS = {
        "csrt": cv.legacy.TrackerCSRT_create,
        "kcf": cv.legacy.TrackerKCF_create,
        "boosting": cv.legacy.TrackerBoosting_create,
        "mil": cv.legacy.TrackerMIL_create,
        "tld": cv.legacy.TrackerTLD_create,
        "medianflow": cv.legacy.TrackerMedianFlow_create,
        "mosse": cv.legacy.TrackerMOSSE_create
    }
    tracker_type = 'kcf'
    video = './video/running.mp4'
    trackers = cv.legacy.MultiTracker_create()
    cap = cv.VideoCapture(video)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print('can not read video')
            break
        # 缩放进行处理 使得处理更快
        # h, w = frame.shape[:2]
        # width = 600
        # r = width / float(w)
        # dim = (width, int(h * r))
        # frame = cv.resize(frame, dim, interpolation=cv.INTER_AREA)

        key = cv.waitKey(35) & 0XFF  # 播放速度 通常25ms左右
        # 框出追踪目标
        if key == ord('s'):
            box = cv2.selectROI('tracker', frame, fromCenter=False, showCrosshair=False)  # 绘制ROI
            # 创建一个新的追踪器
            tracker = OPENCV_OBJECT_TRACKERS[tracker_type]()  # 用字典来选择追踪方法
            trackers.add(tracker, frame, box)
        elif key == 27:
            break

        # 更新追踪结果
        success, boxes = trackers.update(frame)  # 第一个元素表示追踪器是否成功更新了目标状态，第二个元素则是目标框的信息

        # 绘制持续的追踪框
        for box in boxes:
            (x, y, w, h) = [int(v) for v in box]
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv.namedWindow('tracker', 0)
        cv.imshow('tracker', frame)

    cap.release()
    cv.destroyAllWindows()


# 更改为你存MobileNetSSD_deploy.prototxt文件的路径，文件群里有呢
protext = "C:/Users/861899567/Desktop/初赛/ssd/MobileNetSSD_deploy.prototxt"
# 更改为你存MobileNetSSD_deploy.caffemodel文件的路径，文件群里有呢
model = "C:/Users/861899567/Desktop/初赛/ssd/MobileNetSSD_deploy.caffemodel"
# 更改为你想要输出视频的路径，MP4格式，不需要输出视频改为None即可
output = "output2.mp4"
# 更改为你存跑步视频的路径，视频群里有呢
video_file = "running2.mp4"
argconfidence = 0.4

CLASSES = ["backgroud", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtalble",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmoitor"]

print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(protext, model)

print("[INFO] starting video stream...")
vs = cv2.VideoCapture(video_file)
writer = None
trackers = []
labels = []
