#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import threading
import numpy as np
from hiwonder import Misc
from hiwonder import Board
from hiwonder import Camera
from hiwonder.PID import PID
from hiwonder import yaml_handle
import hiwonder.ActionGroupControl as AGC

# POSITION = 'GRASS'
POSITION = 'DOOR'
BRIDGE_COLOR = 'green'
BRICK = 'red'

# CONFIG
lab_data = None
def load_config():
    global lab_data, servo_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
load_config()


# ROBOT_ACUATOR

# ======================= move base ===========================
bridge_angle = None
bridge_midx = None
bridge_midy = None
def moveToRange(name, range):
    pos = {
        'bridge_angle': bridge_angle,
        'bridge_midx': bridge_midx,
        'bridge_midy': bridge_midy
    }
    actions = {
        'bridge_angle': (['sysu_turnleft_door'], ['sysu_turnright_door']),
        'bridge_midx' : (['sysu_left_move_door'], ['sysu_right_move_door']),
        'bridge_midy' : (['sysu_go_forward_start_door', 'sysu_go_forward_door', 'sysu_go_forward_door', 'sysu_go_forward_end_door'], ['sysu_go_forward_start_door', 'sysu_go_forward_door', 'sysu_go_forward_end_door'])
    }
    if pos[name] < range[0]:
        for act in actions[name][0]:
            AGC.runAction(act)
    elif pos[name] > range[1]:
        for act in actions[name][1]:
            AGC.runAction(act)
    else:
        return False
    return True

bridge_state = ''
stateChange_cnt = 0
def changeState(nextState, cnt, sleep_second):
    global bridge_state, stateChange_cnt
    pulse20 = {
        'COLOR': 340,
        'AIM': 340,
        'PASS_1': 320,
        'PASS_2': 290,
        'OUT': 500
    }
    stateChange_cnt += 1
    if stateChange_cnt < cnt:
        print(f'ready to change {bridge_state}')
        time.sleep(sleep_second)
    else:
        stateChange_cnt = 0
        bridge_state = nextState
        print(f'STATE CHANGED: {bridge_state}')
        Board.setBusServoPulse(20, pulse20[nextState], 500)
        if nextState == 'PASS_1':
            AGC.runAction('sysu_go_forward_start')
            AGC.runAction('sysu_go_forward')
            AGC.runAction('sysu_go_forward_end')
        time.sleep(0.5)

# ======================= move ===========================


def move():
    global bridge_state, bridge_angle, bridge_midx, bridge_midy, stateChange_cnt
    while bridge_state != 'END':
        print(bridge_state, bridge_angle, bridge_midx, bridge_midy)
        # time.sleep(0.1)
        if bridge_state == 'START':
            changeState('COLOR', 1, 0.01)
        elif bridge_state == 'AIM':
            if bridge_angle != None and moveToRange('bridge_angle', (-5, 5)):
                stateChange_cnt = 0
                print(bridge_angle)
                time.sleep(0.8)
                continue
            elif bridge_midx != None and moveToRange('bridge_midx', (350-40, 350+40)):
                stateChange_cnt = 0
                print(bridge_midx)
                AGC.runAction('1')
                continue
            elif bridge_midy != None and moveToRange('bridge_midy', (240, 480)):
                stateChange_cnt = 0
                print(bridge_midy)
                AGC.runAction('1')
                continue
            elif bridge_midx is not None and bridge_midy is not None:
                changeState('PASS_1', 4, 0.05)
            else:
                time.sleep(0.01)
        elif bridge_state == 'PASS_1':
            if bridge_angle != None and moveToRange('bridge_angle', (-7, 7)):
                stateChange_cnt = 0
                continue
            # elif bridge_midx != None and moveToRange('bridge_midx', (320-30, 320+30)):
            #     continue
            elif bridge_midy != None and moveToRange('bridge_midy', (240, 480)):
                stateChange_cnt = 0
                time.sleep(1)
                continue
            elif bridge_angle is not None and bridge_midx is not None and bridge_midy is not None:
                changeState('PASS_2', 4, 0.05)
            else:
                time.sleep(0.01)
        elif bridge_state == 'PASS_2':
            if bridge_angle != None and moveToRange('bridge_angle', (-5, 5)):
                stateChange_cnt = 0
                continue
            # elif bridge_midx != None and moveToRange('bridge_midx', (320-50, 320+50)):
            #     continue
            elif bridge_midy != None and moveToRange('bridge_midy', (240, 480)):
                stateChange_cnt = 0
                time.sleep(1)
                continue
            elif bridge_angle is not None and bridge_midx is not None and bridge_midy is not None:
                changeState('OUT', 1, 0)
            else:
                time.sleep(0.01)
        elif bridge_state == 'OUT':
            print('PASS == No More Action!')
            AGC.runAction('1')
            AGC.runAction('sysu_go_forward_start')
            AGC.runAction('sysu_go_forward', 2)
            AGC.runAction('sysu_go_forward_end')
            AGC.runAction('1')
            bridge_state = 'END'
        else:
            time.sleep(0.1)

## ================= imgproc =========================
def imgproc(camera):
    global bridge_state, bridge_angle, bridge_midx, bridge_midy
    global POSITION, BRIDGE_COLOR

    while bridge_state != 'END':
        img = camera.frame
        if img is not None:
            img_draw = img.copy()
            if bridge_state == 'COLOR':
                blue_cnt = getContour(img, 'bridge_blue')
                green_cnt = getContour(img, 'bridge_green')
                if blue_cnt is not None and green_cnt is not None:
                    blue_area = cv2.contourArea(getContour(img, 'bridge_blue'))
                    green_area = cv2.contourArea(getContour(img, 'bridge_green'))
                    BRIDGE_COLOR = 'green' if green_area > blue_area else 'blue'
                elif blue_cnt is None and green_cnt is not None:
                    BRIDGE_COLOR = 'green'
                elif blue_cnt is not None and green_cnt is None:
                    BRIDGE_COLOR = 'blue'
                else:   # DEFAULT: BLUE
                    BRIDGE_COLOR = 'blue'

                changeState('AIM', 3, 0.01)
            elif bridge_state == 'AIM':
                # point, _ = bridge_poly(img, img_draw)
                _, bridge_angle = brick_poly(img, img_draw)

                bridge_midx = bridge_split(img, img_draw)

                if bridge_midx is not None:
                    probe = None
                    x_bias = 0
                    if bridge_midx < 320-40:
                        x_bias = 40
                    elif bridge_midx > 320+40:
                        x_bias = -40
                    if POSITION == 'DOOR' or POSITION == 'BALL' or (POSITION == "GRASS" and BRIDGE_COLOR != "GREEN"):
                        probe = getContour(img[480-5:480, 320-5+x_bias:320+5+x_bias], 'bridge_probe') # 这个颜色才压得住白色
                        if img_draw is not None:
                            color = (0, 255, 0) if probe is not None else (0, 0, 255)
                            cv2.rectangle(img_draw, (320-5+x_bias, 480-5), (320+5+x_bias, 480), (0, 0, 255), 2)
                        if probe is not None:       # end
                            bridge_midy = 479
                        else:
                            bridge_midy = 0
                    elif POSITION == 'GRASS':
                        probe1 = getContour(img[480-10:480, 0:10], 'grass')
                        probe2 = getContour(img[480-10:480, 640-10:640], 'grass')
                        if img_draw is not None:
                            color = (0, 255, 0) if probe1 is not None else (0, 0, 255)
                            cv2.rectangle(img_draw, (0, 480-10), (10, 480), color, 2)
                            color = (0, 255, 0) if probe2 is not None else (0, 0, 255)
                            cv2.rectangle(img_draw, (640-10, 480-10), (640, 480), color, 2)
                        if probe1 is None and probe2 is None:   # end
                            bridge_midy = 479
                        else:
                            bridge_midy = 0
                else:
                    bridge_midx = 320
                    bridge_midy = 479
                        
                            
            elif bridge_state == 'PASS_1' or bridge_state == 'PASS_2':
                code = bridge_front(img, img_draw, h=30, w=140)
                if code == 'left':
                    bridge_angle = 90
                    bridge_midy = 0
                elif code == 'mid':
                    bridge_angle = 0
                    bridge_midy = 0
                elif code == 'right':
                    bridge_angle = -90
                    bridge_midy = 0
                elif code == 'end':
                    bridge_angle = 0
                    bridge_midy = 480
                else:
                    bridge_angle, bridge_midy = None, None
            else:
                time.sleep(0.01)
            cv2.imshow('img_draw', img_draw)
            if cv2.waitKey(1) == 27:
                bridge_state = 'END'
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
        

## ================= imgproc_base =========================

def colorfilter(img, target_color):
    GaussianBlur_img = cv2.GaussianBlur(img, (3, 3), 0)
    frame_lab = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2LAB)
    frame_mask = cv2.inRange(frame_lab,
        (
            lab_data[target_color]['min'][0],
            lab_data[target_color]['min'][1],
            lab_data[target_color]['min'][2]
        ),
        (
            lab_data[target_color]['max'][0],
            lab_data[target_color]['max'][1],
            lab_data[target_color]['max'][2]
        )
    )
    return frame_mask

def areaMaxContour(img, area_min):
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    if contours is not None and len(contours) > 0:
        areaMax_contour = max(contours, key=(lambda cnt : cv2.contourArea(cnt)))
        if cv2.contourArea(areaMax_contour) < area_min:
            areaMax_contour = None
    else:
        areaMax_contour = None
    return areaMax_contour

def getContour(img, target):
    global BRIDGE_COLOR, BRICK
    color = {
            'bridge_blue': 'bridge_bridge_blue',
            'bridge_green': 'bridge_bridge_green',

            'bridge_probe': 'bridge_bridge_'+BRIDGE_COLOR,
            'bridge_front': 'bridge_bridge_front_'+BRIDGE_COLOR,
            'bridge_front_probe': 'bridge_bridge_front_'+BRIDGE_COLOR,

            'brick_'+BRICK: 'bridge_brick_'+BRICK,
            'grass': 'bridge_grass',
            # 'brick_mine': 'bridge_brick_mine'
            }
    morph = {
        'bridge_blue': [cv2.MORPH_CLOSE, 1],
        'bridge_green': [cv2.MORPH_CLOSE, 1],

        'bridge_probe': [cv2.MORPH_CLOSE, 8],
        'bridge_front': [cv2.MORPH_CLOSE, 1],
        'bridge_front_probe': [cv2.MORPH_CLOSE, 1],

        'brick_red': [cv2.MORPH_CLOSE, 15],
        'brick_mine': [cv2.MORPH_OPEN, 3],
        'brick_white': [cv2.MORPH_CLOSE, 3],
        'grass': [cv2.MORPH_CLOSE, 15],
        # 'brick_mine': [cv2.MORPH_OPEN, 1]
        }
    area_min = {
        'bridge_blue': 300,
        'bridge_green': 300,

        'bridge_probe': 1,
        'bridge_front': 300,
        'bridge_front_probe': 1,
        
        'brick_'+BRICK: 2000,
        'grass': 50,
        # 'brick_mine': 2000
    }
    frame_mask = colorfilter(img, color[target])
    frame_morph = cv2.morphologyEx(frame_mask, morph[target][0], np.ones((3, 3), np.uint8), iterations=morph[target][1])
    return areaMaxContour(frame_morph, area_min[target])


def bridge_split(img, img_draw):
    roi = [
        # (0, 80, 0, 640, 0.2), 3 2 1 1
        (80, 160, 0, 640, 0.2),
        (160, 240, 0, 640, 0.2),
        (240, 320, 0, 640, 0.2),
        (320, 400, 0, 640, 0.2),
        ]
    centerx_sum = 0
    weight_sum = 0
    centers = []
    for r in roi:
        blobs = img[r[0]:r[1], r[2]:r[3]]
        areaMax_contour = getContour(blobs, 'bridge'+'_'+BRIDGE_COLOR)
        if areaMax_contour is not None:
            # centerx
            rect = cv2.minAreaRect(areaMax_contour)
            centers.append(rect[0])
            centerx_sum += rect[0][0] * r[4]
            weight_sum += r[4]
            if img_draw is not None:
                pts = cv2.boxPoints(rect)
                cv2.rectangle(img_draw[r[0]:r[1], r[2]:r[3]], (int(pts[1][0]), int(pts[1][1])), (int(pts[3][0]), int(pts[3][1])), (255, 0, 0), 3)

    if weight_sum is not 0:
        centerx = int(centerx_sum / weight_sum)
        cv2.line(img_draw, (centerx, 0), (centerx, 480), (0, 0, 255), 2)
    else:
        centerx = None
    
    return centerx

def midAngleLine(cnt, pos, img_draw):
    poly = cv2.approxPolyDP(cnt, 50, True)
    if poly is not None:
        pos_lambda = {
            'top': (lambda p : p[0][1]),
            'bot': (lambda p : -p[0][1])
        }
        pts = sorted(sorted(poly,
                    key = pos_lambda[pos])[:2],
                key = (lambda p : p[0][0]))
        if len(pts) < 2:
            return None, None

        line = (pts[0][0], pts[1][0])
        
        diff = line[1] - line[0]
        angle = math.degrees(math.atan2(diff[1], diff[0]))
        mid = np.int32((line[0] + line[1]) / 2)

        if img_draw is not None:
            cv2.drawContours(img_draw, [poly], 0, (255, 0, 0), -1)
            cv2.line(img_draw, tuple(line[0]), tuple(line[1]), (0, 0, 255), 2)
            cv2.circle(img_draw, tuple(mid), 3, (0, 255, 0) , -1)
            cv2.putText(img_draw, f'{angle:4.4}', (int(mid[0]), int(mid[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    else:
        mid, angle = None, None
    
    return mid, angle

def bridge_poly(img, img_draw):
    # cnt = getBridgeContour(img[0:400, 0:640], 'green', 2000)
    cnt = getContour(img[0:400, 0:640], 'bridge'+'_'+BRIDGE_COLOR)
    if cnt is not None:
        mid, angle = midAngleLine(cnt, 'top', img_draw)
    else:
        mid, angle = None, None
    
    return mid, angle

def brick_poly(img, img_draw): 
    cnt = getContour(img[0:480, 0:640], 'brick_'+BRICK)
    if cnt is not None:
        mid, angle = midAngleLine(cnt, 'bot', img_draw)
    else:
        mid, angle = None, None
    return mid, angle

def bridge_front(img, img_draw, h=120, w=120):
    xmid = 360
    probe = getContour(img[480-h-5:480-h, xmid-5:xmid+5], 'bridge_probe')
    if probe is None:
        return 'end'

    left_red = getContour(img[480-h:480, xmid-w:xmid], 'brick_'+BRICK)
    left_green = getContour(img[480-h:480, xmid-w:xmid], 'bridge_front')
    right_red = getContour(img[480-h:480, xmid:xmid+w], 'brick_'+BRICK)
    right_green = getContour(img[480-h:480, xmid:xmid+w], 'bridge_front')

    left_area = cv2.contourArea(left_red) if left_red is not None else 0
    left_area += cv2.contourArea(left_green) if left_green is not None else 0
    right_area = cv2.contourArea(right_red) if right_red is not None else 0
    right_area += cv2.contourArea(right_green) if right_green is not None else 0

    color = (0, 255, 0)
    if (left_green is not None or left_red is not None ) and (right_green is not None or right_red is not None):
        percentage = (left_area + right_area) / (h*w + h*w)
        if percentage < 0.93:
            color = (0, 0, 255)
            if img_draw is not None:
                cv2.rectangle(img_draw, (xmid-w, 480-h), (xmid+w, 480), (0, 0, 255), 2)
            if left_area > right_area:
                res = 'right'
            elif left_area < right_area:
                res = 'left'
            else:
                res = 'mid'
        else:
            res = 'mid'
    elif left_green is None and left_red is None and (right_green is not None or right_red is not None):
        res = 'left'
    elif right_green is None and right_red is None and (left_green is not None or right_red is not None): # right_area is None
        res = 'right'
    else:
        res = 'mid'

    if img_draw is not None:
        cv2.rectangle(img_draw, (xmid-5,480-h-5), (xmid+5, 480-h), (0, 255, 255), 2)
        cv2.rectangle(img_draw, (xmid-w, 480-h), (xmid+w, 480), color, 2)

    return res


# ============== test img =================

IMGPROC_LIST = [bridge_front]

def test_imgproc(camera, imgproc_list):
    while True:
        img = camera.frame
        if img is not None:
            img_draw = img.copy()
            for f in imgproc_list:
                f(img, img_draw)
            cv2.imshow('img_draw', img_draw)
            if cv2.waitKey(1) == 27:
                break
        else:
            time.sleep(0.01)

def test_main():
    my_camera = Camera.Camera()
    my_camera.camera_open()
    
    AGC.runAction('1')
    Board.setBusServoPulse(19, 500, 500)
    Board.setBusServoPulse(20, 350, 500)
    time.sleep(0.5)

    test_imgproc(my_camera, imgproc_list=IMGPROC_LIST)

    my_camera.camera_close()
    cv2.destroyAllWindows()

### ================ 驱动器 ==================
    
def init():
    global bridge_state
    Board.setBusServoPulse(19, 500, 500)
    Board.setBusServoPulse(20, 390, 500)
    time.sleep(0.5)
    AGC.runAction('1')

    bridge_state = 'START'

def pass_bridge(camera):
    init()

    th_move = threading.Thread(target=move, daemon=True)
    th_move.start()

    th_camera_agent = threading.Thread(target=imgproc(camera), daemon=True)
    th_camera_agent.start()

    th_move.join()
    th_camera_agent.join()


def bridge_main(state_cnt):
    global POSITION, BRICK
    my_camera = Camera.Camera()
    my_camera.camera_open()
    # start acuators

    if state_cnt == 2:
        POSITION = 'GRASS'
        BRICK = 'mine'
    elif state_cnt == 5:
        POSITION = 'DOOR'
        BRICK = 'red'
        AGC.runAction('sysu_go_forward_start')
        AGC.runAction('sysu_go_forward', 1)
        AGC.runAction('sysu_go_forward_end')
    elif state_cnt == 7:
        POSITION = 'BALL'
        BRICK = 'white'
        
    else:
        print("ERROR: state_cnt error: ", state_cnt)
    

    pass_bridge(my_camera)

    my_camera.camera_close()
    cv2.destroyAllWindows()

### =======================================

if __name__ == '__main__':
    bridge_main(5)