import time
import math
import threading
import cv2
import queue
import numpy as np


frame = None
frame_cali = None

frames = queue.Queue()


def capture(frames):
    # 鱼眼校正参数
    calibration_param_path = "/home/pi/2022/calibration_param.npz"
    param_data = np.load(calibration_param_path)
    dim = tuple(param_data["dim_array"])
    k = np.array(param_data["k_array"].tolist())
    d = np.array(param_data["d_array"].tolist())
    p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None)
    map = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), p, dim, cv2.CV_16SC2)

    # 打开摄像头
    while True:
        try:
            cap = cv2.VideoCapture(-1)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("Y", "U", "Y", "V"))
            cap.set(cv2.CAP_PROP_FPS, 30)
            break
        except Exception as e:
            print(f"camera:open failed:{str(e)}")
            time.sleep(0.1)

    # 获取并校正图像
    while True:
        if not cap.isOpened():
            time.sleep(0.1)
            continue

        ret, frame = cap.read()
        if not ret:
            continue

        frame_cali = cv2.remap(
            frame,
            map[0],
            map[1],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        frames.put(frame_cali)

def get_frame_bin(img, ksize_smooth=3, ksize_morph=3):
    """获取二值化图像（颜色）

    Args:
        img (ndarray): 源图像
        color (str): 颜色名称
        ksize_smooth (int, optional): 高斯模糊核大小. Defaults to 3.
        ksize_morph (int, optional): 开闭运算核大小. Defaults to 3.

    Returns:
        ndarray: 二值化图像
    """
    img_smooth = cv2.GaussianBlur(img, (ksize_smooth, ksize_smooth), 0)
    img_transform = cv2.cvtColor(img_smooth, cv2.COLOR_BGR2LAB)
    img_thresh = cv2.inRange(img_transform, (47, 0, 135), (255, 110, 255))
    kernel = np.ones((ksize_morph, ksize_morph), np.uint8)
    open = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    close = cv2.morphologyEx(open, cv2.MORPH_CLOSE, kernel)
    return close

def show(frames):
    while True:
        frame = frames.get()
        cnts, _ = cv2.findContours(get_frame_bin(frame), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if cnts:
            print(f"type(cnts): {type(cnts)}, type(cnts[0]): {type(cnts[0])} {cnts[0]}")

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    th_pool = []
    cap_th = threading.Thread(target=capture, args=(frames,), daemon=True)
    th_pool.append(cap_th)

    show_th = threading.Thread(target=show, args=(frames,), daemon=True)
    th_pool.append(show_th)

    for th in th_pool:
        th.start()

    show_th.join()
