import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

img_name = './img/ball1.png'
img1_name = './img/Being-alone-is-not-bad-PC.jpg'
img2_name = './img/everything-is-possible-at-home-PC.jpg'
img3_name = './img/dinner-pc.jpg'
cat_img = './img/cat.png'
url = 'http://snow:cxw825@172.26.74.79:8080/video'


def draw_img():
    # 创建一个大小为 (512, 512)，通道数为 3（表示颜色通道）的黑色图像
    img = np.zeros((512, 512, 3), np.uint8)

    # 在图像上绘制一条直线，起点坐标为 (0,0)，终点坐标为 (511,250)，颜色为红色，线宽为 5px
    cv.line(img, (0, 0), (511, 250), (0, 0, 255), 5)

    # 在图像上绘制一个矩形，左上角坐标为 (10,2)，右下角坐标为 (510,128)，颜色为淡蓝色，线宽为 2px
    cv.rectangle(img, (10, 2), (510, 128), (0, 128, 128), 2)

    # 在图像上绘制一个实心圆，圆心坐标为 (250,63)，半径为 63px，颜色为红色
    cv.circle(img, (250, 63), 63, (0, 0, 255), -1)

    # 在图像上绘制一个椭圆，圆心坐标为 (256,256)，长轴长度为 100px，短轴长度为 50px，颜色为深蓝色
    # 椭圆的起始角度为 0 度，结束角度为 200 度（200 度相当于 pi 弧度），线宽为 -1 （表示实心）
    cv.ellipse(img, (256, 256), (100, 50), 0, 0, 200, (255, 0, 2), -1)

    # 定义一个多边形的顶点坐标
    pts = np.array([[10, 5], [20, 30], [70, 20], [50, 10]], np.int32)
    # 将多边形顶点的数组形状改变成 (-1,1,2)，其中“-1”表示自动计算行数，每个元素包含 1 行、2 列的坐标数据。
    # 这里这样处理是为了符合 cv2.polylines() 函数的输入格式要求
    pts = pts.reshape((-1, 1, 2))
    # print(pts.shape)
    # 在图像上绘制一个多边形，多边形的顶点由 pts 指定，颜色为黄色，线宽为 1px，闭合形状
    cv.polylines(img, [pts], True, (0, 255, 255))

    # 在图像上添加文字 “Snow”，位置为 (10,500)，字体为 FONT_HERSHEY_SIMPLEX，大小为 4，颜色为浅紫色
    # 线宽为 6px，线型为 LINE_AA
    cv.putText(img, 'Snow', (10, 500), cv.FONT_HERSHEY_SIMPLEX, 4, (200, 120, 120), 6, cv.LINE_AA)

    # 显示图像
    cv.imshow('img', img)
    # 等待按键，按任意键即可退出窗口
    cv.waitKey()


def draw_with_mouse():
    # 定义鼠标回调函数
    def draw_img(event, x, y, flags, param):
        # 当按下左键时开始绘制图形
        global ix, iy
        if event == cv.EVENT_LBUTTONDOWN:
            ix, iy = x, y  # 捕获鼠标点击的起始位置
        # 当松开左键时，结束绘制
        elif event == cv.EVENT_LBUTTONUP:
            if mode:  # 当前模式为绘制空心矩形
                # 绘制一个矩形
                jx, jy = x, y
                cv.rectangle(img, (ix, iy), (jx, jy), (0, 255, 0), 1)
            else:
                # 绘制一个实心圆形
                cv.circle(img, (x, y), 30, (0, 0, 255), -1)

    # 获取 OpenCV 中所有关于鼠标和键盘事件常量的名称，并存储在 events 列表中
    events = [i for i in dir(cv) if 'EVENT' in i]
    print(events)
    # 用于判断是否按下鼠标和当前绘制模式（矩形 or 圆）
    mode = True  # 如果为真，绘制矩形。按 m 键可以切换到曲线
    # 创建一个黑色的图像，大小为 512 × 512，通道数为 3（表示颜色通道）
    # 数据类型为 unsigned int 8（即 uint8）一个通道8个字节
    img = np.zeros((512, 512, 3), np.uint8)  # 背景板

    # 创建一个名为 "image" 的窗口，并将鼠标事件和鼠标回调函数 draw_circle 绑定到该窗口
    cv.namedWindow('image')
    cv.setMouseCallback('image', draw_img)
    # 进入无限循环，不断显示图像，等待用户按下键盘 ESC 键退出程序
    while True:
        cv.imshow('image', img)  # 在窗口中显示图像
        if cv.waitKey(1) == ord('m'):
            mode = not mode
        # 等待键盘输入，如果输入键值为 27（即 ESC 键），则退出程序
        if cv.waitKey(20) & 0xFF == 27:
            break

    # 关闭所有窗口
    cv.destroyAllWindows()


def color_trackbar():
    def do_nothing():  # 定义一个空的回调函数
        pass

    img = np.zeros((300, 512, 3), np.uint8)  # 创建一张黑色的彩色图像
    cv.namedWindow('image')  # 创建一个名为 'image' 的窗口
    cv.createTrackbar('R', 'image', 0, 255, do_nothing)  # 在 'image' 窗口上创建一个名为 'R' 的滑动条，取值范围为 0~255
    cv.createTrackbar('G', 'image', 0, 255, do_nothing)  # 在 'image' 窗口上创建一个名为 'G' 的滑动条，取值范围为 0~255
    cv.createTrackbar('B', 'image', 0, 255, do_nothing)  # 在 'image' 窗口上创建一个名为 'B' 的滑动条，取值范围为 0~255
    switch = 'OFF:0\\ON:1'
    cv.createTrackbar(switch, 'image', 0, 1, do_nothing)  # 在 'image' 窗口上创建一个名为 switch 的滑动条，取值范围为 0~1
    while True:
        cv.imshow('image', img)  # 在 'image' 窗口中显示图像
        if cv.waitKey(1) & 0xFF == 27:  # 按下 ESC 键，退出程序
            break
        # 得到四条轨迹的当前位置
        r = cv.getTrackbarPos('R', 'image')  # 获取名为 'R' 的滑动条的当前位置
        g = cv.getTrackbarPos('G', 'image')  # 获取名为 'G' 的滑动条的当前位置
        b = cv.getTrackbarPos('B', 'image')  # 获取名为 'B' 的滑动条的当前位置
        s = cv.getTrackbarPos(switch, 'image')  # 获取名为 switch 的滑动条的当前位置
        if s == 0:
            img[:] = 0
        else:
            img[:] = [b, g, r]  # 将三个颜色通道的值分别设置为 b、g、r
    cv.destroyAllWindows()  # 销毁所有窗口


def img_base_op():
    img = cv.imread(img_name, 1)
    # 访问坐标(100,100)的BGR通道 返回一个三维数组
    # 如果是灰度图 则返回灰度值
    px = img[100, 100]
    # 得到B通道的值
    blue = px[0]  # 或img[100,100,0]
    print(px, blue)
    # 修改(100,100)的BRG值
    px = [1, 2, 3]
    print(px)
    # -------------------------------
    # 比单个索引更好的方法
    px = img.item(10, 10, 2)  # (10,10)的R通道值
    img.itemset((10, 10, 2), 128)  # 修改为128
    print(px, img.item(10, 10, 2))
    # -------------------------------
    # 图像属性就是numpy数组属性
    print(img.shape)  # 形状
    print(img.size)  # 总数
    print(img.dtype)  # 数据类型
    # -------------------------------
    # 图像感兴趣的区域 就是截取部分图 使用切片
    part = img[280:340, 330:390]
    img[273:333, 100:160] = part
    cv.imshow('img', img)
    # -------------------------------
    # 拆分和合并
    b, g, r = cv.split(img)  # 耗时 一般索引更好
    img = cv.merge((b, g, r))
    print(b, g, r, img)
    # -------------------------------
    # 为图像设置边框(填充图像)
    '''
    cv.BORDER_CONSTANT：在边界外面添加一个常数值的颜色填充区域。
    cv.BORDER_REPLICATE：将边界像素复制到边界区域。
    cv.BORDER_REFLECT：通过镜像翻转的方式填充边缘区域。
    cv.BORDER_REFLECT_101：通过镜像翻转的方式填充边缘区域，但是不包括外部边界像素。
    cv.BORDER_WRAP：通过复制图像边界的方式填充边缘区域。
    '''

    BLUE = [255, 0, 0]

    # 读取原始图像文件
    img1 = cv.imread(img_name)

    # 使用不同的边界拓展方式将图像进行扩展，并保存到新的变量中
    replicate = cv.copyMakeBorder(img1, 10, 10, 10, 10, cv.BORDER_REPLICATE)
    reflect = cv.copyMakeBorder(img1, 10, 10, 10, 10, cv.BORDER_REFLECT)
    reflect101 = cv.copyMakeBorder(img1, 10, 10, 10, 10, cv.BORDER_REFLECT_101)
    wrap = cv.copyMakeBorder(img1, 10, 10, 10, 10, cv.BORDER_WRAP)
    constant = cv.copyMakeBorder(img1, 10, 10, 10, 10, cv.BORDER_CONSTANT, value=BLUE)

    # 使用Matplotlib库将原始图像以及边界拓展后的图像进行可视化展示
    plt.subplot(231), plt.imshow(img1, 'gray'), plt.title('ORIGINAL')
    plt.subplot(232), plt.imshow(replicate, 'gray'), plt.title('REPLICATE')
    plt.subplot(233), plt.imshow(reflect, 'gray'), plt.title('REFLECT')
    plt.subplot(234), plt.imshow(reflect101, 'gray'), plt.title('REFLECT_101')
    plt.subplot(235), plt.imshow(wrap, 'gray'), plt.title('WRAP')
    plt.subplot(236), plt.imshow(constant, 'gray'), plt.title('CONSTANT')
    plt.show()


def img_base_math_op():
    # 使用图像加法时,图片尺寸要相同
    # 图像加法,使用cv.add(),而非numpy
    x = np.uint8([250])
    y = np.uint8([10])
    # numpy add 模
    print(x + y)  # 250+10=260%256=4
    # opencv add 饱和
    print(cv.add(x, y))  # 250+10=260=255

    # 图像融合 给不同图像赋予不同权重
    img1 = cv.imread(img1_name)
    img2 = cv.imread(img2_name)

    dst = cv.addWeighted(img1, 0.6, img2, 0.3, 0)
    cv.namedWindow('dst', 0)
    cv.resize(dst, [400, 300])
    cv.imshow('dst', dst)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # 按位运算
    # 加载两张图片
    img1 = cv.imread(img1_name)
    img2 = cv.imread(img2_name)
    # 我想把logo放在左上角，所以我创建了ROI
    rows, cols, channels = img2.shape
    roi = img1[0:rows, 0:cols]
    # 现在创建logo的掩码，并同时创建其相反掩码
    img2gray = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)
    ret, mask = cv.threshold(img2gray, 10, 255, cv.THRESH_BINARY)  # cv.THRESH_BINARY表示超过阈值(10)的部分取maxval（255），否则取0（最小值）
    mask_inv = cv.bitwise_not(mask)
    # 现在将ROI中logo的区域涂黑
    img1_bg = cv.bitwise_and(roi, roi, mask=mask_inv)
    # 仅从logo图像中提取logo区域
    img2_fg = cv.bitwise_and(img2, img2, mask=mask)
    # 将logo放入ROI并修改主图像
    dst = cv.add(img1_bg, img2_fg)
    img1[0:rows, 0:cols] = dst
    cv.imshow('res', img1)
    cv.waitKey(0)
    cv.destroyAllWindows()


def show_slide():
    # 读取三张图片
    img1 = cv.imread(img1_name)
    img2 = cv.imread(img2_name)
    img3 = cv.imread(img3_name)

    # 初始化混合系数alpha为0
    alpha = 0

    # 循环处理直到按下ESC键退出
    while alpha <= 1.0:
        # 每次循环检查是否按下了ESC键，是则退出循环
        if cv.waitKey(1) == 27:
            break

        # 利用cv.addWeighted将两张图片按指定的alpha值进行线性混合
        # 这里dst就是输出的混合结果
        dst = cv.addWeighted(img1, 1 - alpha, img2, alpha, 0)

        # 创建一个名为'dst'的窗口，并显示混合结果
        cv.namedWindow('dst', 0)
        cv.resize(dst, [200, 170])
        cv.imshow('dst', dst)

        # 等待10毫秒，然后按照步长0.02递增alpha的值
        cv.waitKey(10)
        alpha += 0.02

        # 当alpha超过1时，交换img1,img2,img3的位置，使其循环
        # 多张图片可以使用链表连接图片和切换图片
        if alpha > 1.0:
            img1, img2, img3 = img2, img3, img1
            alpha = 0
            cv.waitKey(100)

    # 销毁所有窗口
    cv.destroyAllWindows()


def capture_color():
    # 定义三种颜色，分别为绿、蓝、红
    green = np.uint8([[[0, 255, 0]]])
    blue = np.uint8([[[255, 0, 0]]])
    red = np.uint8([[[0, 0, 255]]])

    # 将RGB格式转换成HSV格式，便于之后的颜色过滤
    hsv_green = cv.cvtColor(green, cv.COLOR_BGR2HSV)
    hsv_blue = cv.cvtColor(blue, cv.COLOR_BGR2HSV)
    hsv_red = cv.cvtColor(red, cv.COLOR_BGR2HSV)

    # 获取三种颜色的HSV数值，便于之后的颜色过滤
    green = hsv_green[0][0]
    blue = hsv_blue[0][0]
    red = hsv_red[0][0]

    # 创建VideoCapture对象，打开默认的摄像头
    cap = cv.VideoCapture(0)

    # 循环读取摄像头中的数据
    while True:
        # 读取一帧图像
        ret, frame = cap.read()

        # 将读取到的图像从RGB格式转换成HSV格式
        hsv_frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # 分别定义三种颜色的上下限阈值，用于过滤出对应颜色的像素点
        l_green = np.array(green - [20, 200, 200])
        h_green = np.array(green + [20, 0, 0])

        l_blue = np.array(blue - [20, 200, 200])
        h_blue = np.array(blue + [20, 0, 0])

        l_red = np.array(red - [20, 200, 200])
        h_red = np.array(red + [20, 0, 0])

        # 根据上下限阈值过滤出对应颜色的像素点
        mask_green = cv.inRange(hsv_frame, l_green, h_green)
        mask_blue = cv.inRange(hsv_frame, l_blue, h_blue)
        mask_red = cv.inRange(hsv_frame, l_red, h_red)

        # 将三种颜色的掩模图叠加，得到最终掩模图
        mask = cv.bitwise_or(mask_green, mask_blue)
        mask = cv.bitwise_or(mask, mask_red)

        # 用掩模进行与运算, 得到最终图像
        res = cv.bitwise_and(frame, frame, mask=mask)

        # 显示原图、掩模图和最终图像
        cv.imshow('frame', frame)
        cv.imshow('mask', mask)
        cv.imshow('res', res)

        # 设置窗口大小
        cv.resizeWindow("frame", 400, 300)
        cv.resizeWindow("mask", 400, 300)
        cv.resizeWindow("res", 400, 300)

        # 如果按下ESC键，则退出循环
        if cv.waitKey(5) & 0xFF == 27:
            break

    # 释放资源并关闭窗口
    cv.destroyAllWindows()


def img_perspective_transform():
    # 读入一幅图像
    img = cv.imread(cat_img)

    # 缩放图像
    # 方法1：利用cv.resize()函数进行缩放，获取缩放后的图像
    res1 = cv.resize(img, None, fx=2, fy=2, interpolation=cv.INTER_CUBIC)
    # 方法2：计算缩放倍数并指定输出图像大小进行缩放，获取缩放后的图像
    height, width = img.shape[:2]
    res2 = cv.resize(img, (2 * width, 2 * height), interpolation=cv.INTER_CUBIC)

    # 输出原图像和缩放后的图像的尺寸信息
    print('original shape', img.shape)
    print('resize shape res1', res1.shape)
    print('resize shape res2', res2.shape)

    # 平移图像
    rows, cols = img.shape[:2]  # 取出图像的行数和列数
    # 构建2*3矩阵，表示在x方向上平移100个单位，在y方向上平移50个单位
    M = np.float32([[1, 0, 100], [0, 1, 50]])
    # 利用cv.warpAffine()函数对图像进行平移，并返回平移后的图像
    dst = cv.warpAffine(img, M, (cols, rows))  # 接收32位的矩阵

    # 旋转图像
    # 构建2*3矩阵M，表示将图像按逆时针方向旋转90度
    M = cv.getRotationMatrix2D(((cols - 1) / 2.0, (rows - 1) / 2.0), 90, 1)
    # 利用cv.warpAffine()函数对图像进行旋转，并返回旋转后的图像
    dst = cv.warpAffine(img, M, (cols, rows))

    # 仿射变换
    pts1 = np.float32([[50, 50], [200, 50], [50, 200]])  # 源图像中三个顶点
    pts2 = np.float32([[10, 100], [200, 50], [100, 250]])  # 目标图像中对应三个顶点
    # 利用cv.getAffineTransform()函数计算变换矩阵M
    M = cv.getAffineTransform(pts1, pts2)
    # 利用cv.warpAffine()函数对图像进行仿射变换，并返回变换后的图像
    dst = cv.warpAffine(img, M, (cols, rows))
    # 将原图像和变换后的图像在同一窗口中显示
    plt.subplot(121), plt.imshow(img), plt.title('Input')
    plt.subplot(122), plt.imshow(dst), plt.title('Output')

    # 透视变换
    pts1 = np.float32([[56, 65], [368, 52], [28, 387], [389, 390]])  # 源图像中四个顶点
    pts2 = np.float32([[0, 0], [300, 0], [0, 300], [300, 300]])  # 目标图像中对应四个顶点
    # 利用cv.getPerspectiveTransform()函数计算透视变换矩阵M
    M = cv.getPerspectiveTransform(pts1, pts2)
    # 利用cv.warpPerspective()函数对图像进行透视变换，并返回变换后的图像
    dst = cv.warpPerspective(img, M, (300, 300))
    # 将原图像和变换后的图像在同一窗口中显示
    plt.subplot(121), plt.imshow(img), plt.title('Input')
    plt.subplot(122), plt.imshow(dst), plt.title('Output')


def img_threshold():
    img = cv.imread(cat_img)
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # 简单阈值方式有以下几种
    # cv.THRESH_BINARY
    # cv.THRESH_BINARY_INV
    # cv.THRESH_TRUNC
    # cv.THRESH_TOZERO
    # cv.THRESH_TOZERO_INV

    # 简单阈值
    ret, simple_threshold = cv.threshold(src=gray_img, thresh=127, maxval=255, type=cv.THRESH_BINARY)

    # 自适应阈值
    adaptive_threshold = cv.adaptiveThreshold(src=gray_img, maxValue=255, adaptiveMethod=cv.ADAPTIVE_THRESH_GAUSSIAN_C,
                                              thresholdType=cv.THRESH_BINARY, blockSize=99999, C=1)
    # outs二值化
    ret2, otsu_threshold = cv.threshold(gray_img, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    print(adaptive_threshold == otsu_threshold)
    plt.subplot(141)
    plt.imshow(gray_img, 'gray')
    plt.title('origin')
    plt.subplot(142)
    plt.imshow(simple_threshold, 'gray')
    plt.title('simple')

    plt.subplot(143)
    plt.imshow(adaptive_threshold, 'gray')
    plt.title('adaptive')

    plt.subplot(144)
    plt.imshow(otsu_threshold, 'gray')
    plt.title('outs')
    plt.show()
    # 自适应阈值


def img_filter():
    # 滤波原理
    img = cv.imread(cat_img)
    kernel = np.ones((5, 5), np.float32) / 25
    dst = cv.filter2D(img, -1, kernel)  # 做2d卷积

    plt.subplot(121), plt.imshow(img), plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(dst), plt.title('Averaging')
    plt.xticks([]), plt.yticks([])
    plt.show()

    # 四种模糊方法
    blur = cv.blur(img, (6, 6))  # 平均
    blur = cv.GaussianBlur(img, (5, 5), 0)  # 高斯
    blur = cv.medianBlur(img, 5)  # 中值
    blur = cv.bilateralFilter(img, 9, 75, 75)  # 双边

    plt.subplot(121), plt.imshow(img), plt.title('Original')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(blur), plt.title('Blurred')
    plt.xticks([]), plt.yticks([])
    plt.show()


def img_morphology():
    img = cv.imread('./img/j.png')
    kernel = np.ones((6, 6), np.uint8)
    erosion = cv.erode(img, kernel, iterations=1)  # 腐蚀 与运算 内核中不全是1都清零
    dilation = cv.dilate(img, kernel, iterations=1)  # 扩张 或运算 内核中有1就保留
    opening = cv.morphologyEx(img, cv.MORPH_OPEN, kernel)  # 开运算 先腐蚀后扩张 背景除噪
    closing = cv.morphologyEx(img, cv.MORPH_CLOSE, kernel)  # 闭运算 先扩张后腐蚀 前景除噪
    gradient = cv.morphologyEx(img, cv.MORPH_GRADIENT, kernel)  # 形态学梯度 = 扩张 - 侵蚀
    tophat = cv.morphologyEx(img, cv.MORPH_TOPHAT, kernel)  # 顶帽 = 原图 - 开运算
    blackhat = cv.morphologyEx(img, cv.MORPH_BLACKHAT, kernel)  # 黑帽 = 原图 - 闭运算
    title = ['origin', 'erosion', 'dilation', 'opening', 'closing', 'gradient', 'tophat', 'blackhat']
    imgs = [img, erosion, dilation, opening, closing, gradient, tophat, blackhat]
    for i in range(8):
        plt.subplot(2, 4, i + 1)
        plt.imshow(imgs[i], 'gray')
        plt.title(title[i])
        plt.xticks([]), plt.yticks([])  # 不要坐标轴
    plt.show()


def edge_detection():
    def none(x):
        pass  # 定义一个空函数

    img = cv.imread(img3_name, 0)  # 读取灰度图像
    origin_img = img.copy()  # 备份原图像
    cv.namedWindow('canny', 0)  # 创建窗口，并指定为可以调整大小
    cv.createTrackbar('min val', 'canny', 0, 800, none)  # 创建下拉轨迹条，用于调整边缘检测的最小值阈值
    cv.createTrackbar('max val', 'canny', 0, 800, none)  # 创建下拉轨迹条，用于调整边缘检测的最大值阈值
    cv.createTrackbar('switch', 'canny', 0, 1, none)  # 创建开关按钮，用于切换显示原图像或边缘检测图像

    while True:
        if cv.waitKey(1) & 0xFF == 27:  # 等待ESC键退出程序
            break

        min_val = cv.getTrackbarPos('min val', 'canny')  # 获取最小值阈值
        max_val = cv.getTrackbarPos('max val', 'canny')  # 获取最大值阈值
        switch = cv.getTrackbarPos('switch', 'canny')  # 获取开关状态

        edge_img = cv.Canny(img, min_val, max_val)  # 进行边缘检测

        if switch == 0:  # 如果开关为0，则显示原图像
            cv.imshow('canny', origin_img)
        else:  # 否则，显示边缘检测图像
            cv.imshow('canny', edge_img)

    cv.destroyAllWindows()  # 关闭所有窗口


def find_draw_contour():
    img = cv.imread(img2_name)
    # 获得轮廓线先进行边缘检测或二值检测 传入二值图像
    imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edge = cv.Canny(imgray, 200, 500)
    # 根据二值图像查找轮廓 背景黑色 前景白色
    contours, hierarchy = cv.findContours(edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    # 绘制轮廓
    cv.drawContours(img, contours, -1, (255, 255, 0), 3)

    cv.namedWindow('img', 0)
    cv.resizeWindow('img', 800, 600)
    cv.imshow('img', img)
    cv.waitKey()

    # 获得图像的特征矩
    cnt = contours[0]
    M = cv.moments(cnt)
    # 面积
    area = cv.contourArea(cnt)
    # 周长
    perimeter = cv.arcLength(cnt, True)
    print(area, perimeter, M)


def contour_approx():
    img = cv.imread('./img/contour.png')

    # 获得轮廓线先进行边缘检测或二值检测 传入二值图像
    imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    edge = cv.Canny(imgray, 200, 500)
    # 根据二值图像获得轮廓 背景黑色 前景白色
    contours, hierarchy = cv.findContours(edge, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # 绘制轮廓
    # cv.drawContours(img, contours, -1, (20, 20, 255), 2)

    # 矩形包围框
    # 获得轮廓矩形包围框 的左上角起始点 宽 高
    x, y, w, h = cv.boundingRect(contours[0])
    print(f'顶点及宽高分别为:{x}, {y}, {w}, {h}')
    cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

    # 最小矩形包围框
    rect = cv.minAreaRect(contours[0])
    print(f'返回值rect：{rect}')
    points = cv.boxPoints(rect)
    points = np.int_(points)  # 浮点数取整
    cv.drawContours(img, [points], 0, (0, 255, 255), 2)

    # 圆形包围框
    (x, y), radius = cv.minEnclosingCircle(contours[0])
    center = (int(x), int(y))
    radius = int(radius)
    cv.circle(img, center, radius, (255, 255, 0), 2)

    # 椭圆包围框
    ellipse = cv.fitEllipse(contours[0])
    print(f'ellipse={ellipse}')
    cv.ellipse(img, ellipse, (0, 255, 0), 2)

    # 最优拟合直线
    rows, cols = img.shape[:2]
    vx, vy, x, y = cv.fitLine(contours[0], cv.DIST_L2, 0, 0.01, 0.01)  # vx vy是方向向量 vy/vx是斜率

    # 截距b 图像框左边和右边的截距
    lefty = int((-x * vy / vx) + y)
    righty = int(((cols - x) * vy / vx) + y)
    cv.line(img, (cols - 1, righty), (0, lefty), (0, 255, 100), 2)

    # 最小外包三角形
    area, trgl = cv.minEnclosingTriangle(contours[0])
    print(f'area={area}')  # 面积
    print(f'trgl:{trgl}')  # 点集
    print((trgl[0][0][0], trgl[0][0][1]))
    cv.line(img, (trgl[0][0][0], trgl[0][0][1]), (trgl[1][0][0], trgl[1][0][1]), (255, 255, 255), 2)
    # 未知error 待查
    # for i in range(0, 3):
    #     cv.line(img, tuple(trgl[i][0]), tuple(trgl[(i + 1) % 3][0]), (255, 255, 255), 2)

    # 逼近多边形
    epsilon = 0.006 * cv.arcLength(contours[0], True)
    approx = cv.approxPolyDP(contours[0], epsilon, True)
    cv.drawContours(img, [approx], 0, (50, 50, 255), 2)

    cv.namedWindow('img', 0)
    cv.imshow('img', img)
    cv.waitKey()
    cv.destroyAllWindows()


def img_convex():
    img = cv.imread('./img/hand.png')
    grey_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, grey_img = cv.threshold(grey_img, 20, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(grey_img, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)

    new_img = img.copy()
    cv.drawContours(new_img, contours, 0, (0, 0, 255), 2)
    # 得到凸包的角点 并且绘制凸包
    hull = cv.convexHull(contours[0])  # 返回凸包角点的坐标
    hull_contour = cv.polylines(new_img, [hull], True, (0, 255, 255), 2)

    # 绘制凸缺陷
    hull = cv.convexHull(contours[0], returnPoints=False)  # 返回凸包角点的索引
    convexity_defects = cv.convexityDefects(contours[0], hull)  # 得到凸缺陷点集
    print(convexity_defects)
    new_img = img.copy()
    for i in range(convexity_defects.shape[0]):
        s, e, f, d = convexity_defects[i, 0]
        start = tuple(contours[s][0])
        end = tuple(contours[e][0])
        far = tuple(contours[f][0])
        cv.line(new_img, start, end, [0, 255, 0], 2)
        cv.circle(new_img, start, 5, [0, 0, 255], -1)
        cv.circle(new_img, far, 5, [255, 0, 0], -1)

    cv.imshow('img', img)
    cv.imshow('rst', new_img)
    cv.waitKey()
    cv.destroyAllWindows()


img_convex()
