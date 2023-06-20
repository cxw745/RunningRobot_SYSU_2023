import time
import multiprocessing as mp
import queue
import os
import gc

import cv2
import numpy as np


def capture(stack, top:int):
    print(f"process:capture:{os.getpid()}")
    
    # 鱼眼校正参数
    calibration_param_path = "/home/pi/AiNexPro/CameraCalibration/"
    param_data = np.load(calibration_param_path + "calibration_param.npz")
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

        stack.append(frame_cali)
        if len(stack) >= top:
            del stack[:]
            gc.collect()


def show(stack, top):
    print(f"process:show:{os.getpid()}")
    while True:
        if len(stack) > 0:
            frame = stack.pop(0)
            cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        
        if len(stack) >= top:
            del stack[:]
            gc.collect()
    
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print(f"process:main:{os.getpid()}")

    stack = mp.Manager().list()

    cap_proc = mp.Process(target=capture, args=(stack, 5), daemon=True)
    cap_proc.start()

    show_proc = mp.Process(target=show, args=(stack, 5))
    show_proc.start()

    show_proc.join()

    cap_proc.terminate()
    cap_proc.join()