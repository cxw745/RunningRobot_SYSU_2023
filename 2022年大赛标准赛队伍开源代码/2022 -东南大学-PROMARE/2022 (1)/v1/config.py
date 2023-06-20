import cv2
import numpy as np

##########色域#######
transform = cv2.COLOR_BGR2LAB

########图像校准#########
#读取矫正参数
calibration_param_path = '/home/pi/AiNexPro/CameraCalibration/calibration_param'
param_data = np.load(calibration_param_path + '.npz')
dim = tuple(param_data['dim_array'])
k = np.array(param_data['k_array'].tolist())
d = np.array(param_data['d_array'].tolist())
#截取区域，1表示完全截取
scale = 1
#优化内参和畸变参数
p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim ,None)
Knew = p.copy()
if scale:
    Knew[(0,1), (0,1)] = scale * Knew[(0,1), (0,1)]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), Knew, dim, cv2.CV_16SC2)

##########动作组路径###########
action_path = '/home/pi/AiNexPro/AiNexPro/wsh1/'


###########lab############
color_range = {
    'ball': [(0, 0, 0), (20, 255, 255)],
    'red': [(140, 147, 0), (187, 255, 255)],
    'green': [(0, 0, 0), (255, 120, 255)],
    'green_hole': [(47, 0, 135), (255, 110, 255)],
    'blue': [(0, 130, 0), (198, 255, 110)],
    'blue_bridge': [(0, 0, 0), (255, 175, 94)],
    'black': [(0, 0, 0), (55, 255, 255)],
    'white': [(120, 0, 0), (255, 255, 255)],
    'white_road': [(193, 0, 0), (255, 255, 255)],
    'white_door': [(140, 0, 0), (255, 255, 255)],
    'yellow':[(100,140,160), (225,190,190)],
    'yellow_start':[(11,42,71), (225,190,190)],
    'yellow_end':[(0,0,151), (225,141,255)],
    'ball_hole':[(0,137,0), (255,255,127)],
    'barrier_y':[(0,83,162), (200,255,255)],
    'barrier_k':[(0,0,0), (28,255,255)],
    'gate_bule':[(0,0,0),(255,255,115)],
    'road_white':[(134,0,0),(255,255,255)]
}