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
import builtins

# INIT: leftmove 2, 90 degrees, 10 slowmove
# AIM1: angle + x
# PASS1: trans: go_forward 5 (until bridge is None
# AIM2: angle
# PASS2: go_forward 4

# CONFIG
lab_data = None
def load_config():
    global lab_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
load_config()

# ================================== move base ===================================  # TODO: Finalize
door_angle = None
door_x = None
door_y = None
def moveToRange(name, range):
    pos = {
        'door_angle': door_angle,
        'door_x': door_x,
        'door_y': door_y,
    }
    actions = {
        'door_angle': (['sysu_turnleft_door'], ['sysu_turnright_door']),
        'door_x': (['sysu_left_move_door'], ['sysu_right_move_door']),
        'door_y': (['sysu_slow_move'], ['sysu_go_forward_start_door', 'sysu_go_forward_door', 'sysu_go_forward_end_door'])
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

door_state = ''
stateChange_cnt = 0
def changeState(nextState, cnt, sleep_second):
    global door_state, stateChange_cnt
    pulse20 = {         # begin from start
        'INIT': 450,
        'AIM1': 380,
        'PASS1': 350,
        'AIM2': 300,
        'PASS2': 450,
        'END': 500
    }
    stateChange_cnt += 1
    if stateChange_cnt < cnt:
        print(f'ready to change {door_state}')
        time.sleep(sleep_second)
    else:
        stateChange_cnt = 0
        door_state = nextState
        print(f'STATE CHANGED: {door_state}')
        Board.setBusServoPulse(20, pulse20[nextState], 500)
        time.sleep(0.5)
        if door_state == 'AIM1':
            AGC.runAction('sysu_right_move')

def init():
    global door_state, stateChange_cnt
    Board.setBusServoPulse(19, 500, 500)
    Board.setBusServoPulse(20, 350, 500)
    time.sleep(0.5)
    changeState('INIT', 0, 1)
    stateChange_cnt = 0

def move():
    global door_state, stateChange_cnt
    global door_angle, door_x, door_y

    out_cnt = 0
    time.sleep(1)
    while door_state != 'END':
        # print(door_state, door_angle, door_x, door_y)
        time.sleep(0.1)
        if door_state == 'INIT':
            # changeState('AIM1', 0, 1)
            AGC.runAction('1')
            # for i in range(0, 4):
            AGC.runAction('sysu_go_back2', 8)
            AGC.runAction('sysu_right_move_door', 3)
            AGC.runAction('sysu_slow_move', 10)
            AGC.runAction('1')
            AGC.runAction('sysu_turnleft_door', 6)
            AGC.runAction('1')
            changeState('AIM1', 0, 1)
            time.sleep(1)
            continue
        elif door_state == 'AIM1':
            if door_angle is not None and moveToRange('door_angle', (-10, 10)):
                stateChange_cnt = 0
                time.sleep(1)
                continue
            elif door_x is not None and moveToRange('door_x', (320-30, 320+30)):
                stateChange_cnt = 0
                time.sleep(1)
                continue                
            else:
                changeState('PASS1', 2, 1)
        elif door_state == 'PASS1':
            # AGC.runAction('sysu_go_forward_start_door')
            # AGC.runAction('sysu_go_forward_door', 5)
            # AGC.runAction('sysu_go_forward_end_door')
            if door_angle is not None and moveToRange('door_angle', (-10, 10)):
                stateChange_cnt = 0
                time.sleep(1)
                continue
            elif door_y is not None and moveToRange('door_y', (10, 479)):
                stateChange_cnt = 0
                # time.sleep(1)
                continue
            else:
            # AGC.runAction('sysu_slow_move', 1)
                changeState('AIM2', 0, 1)
                time.sleep(1)
        elif door_state == 'AIM2':
            if door_angle is not None and moveToRange('door_angle', (-10, 10)):
                stateChange_cnt = 0
                continue
            else:
                changeState('PASS2', 2, 1)
        elif door_state == 'PASS2':
            AGC.runAction('sysu_go_forward_start_door')
            AGC.runAction('sysu_go_forward_door', 4)
            AGC.runAction('sysu_go_forward_door')
            # AGC.runAction('sysu_slow_move', 10)
            changeState('END', 0, 1)
        else:
            time.sleep(0.01)

# ============================ SIGNAL PROCESSOR ==============================

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
    color = {
        'door_up': 'door_door',
        'door': 'door_door',
        'floor': 'door_floor',
        'green': 'green',
        }
    morph = {
        'door_up': [cv2.MORPH_CLOSE, 15],
        'door': [cv2.MORPH_OPEN, 8],
        'floor': [cv2.MORPH_CLOSE, 15],
        'green': [cv2.MORPH_CLOSE, 1]
        }
    area_min = {
        'door_up': 3000,
        'door': 2000,
        'floor': 2000,
        'green': 300
    }
    frame_mask = colorfilter(img, color[target])
    frame_morph = cv2.morphologyEx(frame_mask, morph[target][0], np.ones((3, 3), np.uint8), iterations=morph[target][1])
    return areaMaxContour(frame_morph, area_min[target])

def getContourDoorMax2(img):
    MINAREA = 2000
    '''返回二元组：两扇门'''
    frame_mask = colorfilter(img, 'door_door')
    frame_morph = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=8)
    contours = cv2.findContours(frame_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    if contours is not None and len(contours) > 0:
        areaSorted_cnts = sorted(contours, key=(lambda cnt : cv2.contourArea(cnt)), reverse=True)
        areaSorted_cnts = list(filter(lambda cnt : cv2.contourArea(cnt) > MINAREA, areaSorted_cnts))
        if areaSorted_cnts is not None and len(areaSorted_cnts) >= 2:
            areaMax_cnts = sorted(areaSorted_cnts[:2], key=lambda cnt : cnt[:, :, 0].max())            
            return tuple(areaMax_cnts)
        else:
            return None
    else:
        return None


def angleLine(cnt, pos, img_draw):
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
            return None
        line = (pts[0][0], pts[1][0])
        
        if img_draw is not None:
            cv2.drawContours(img_draw[0:480, 140:500], [poly], 0, (255, 0, 0), -1)
            cv2.line(img_draw[0:480, 140:500], tuple(line[0]), tuple(line[1]), (0, 0, 255), 2)
        
        diff = line[1] - line[0]
        angle = math.degrees(math.atan2(diff[1], diff[0]))
    return angle


def imgproc(img, img_draw=None):                                        # TODO: Decouple State and Imgproc
    global door_angle, door_y, door_x
    global door_state
    
    if door_state == 'AIM1':
        floor = getContour(img[0:480, 220:420], 'floor')
        door_angle = angleLine(floor, 'top', img_draw) if floor is not None else None
        # TODO: 底端中点值
        ret = getContourDoorMax2(img)
        if ret is not None:
            left, right = getContourDoorMax2(img)
            left_point = left[left[:, :, 0].argmax()][0]
            right_point = right[right[:, :, 0].argmin()][0]
            door_x = (left_point[0] + right_point[0]) / 2
            if img_draw is not None:
                cv2.drawContours(img_draw, [left], 0, (0, 255, 0), -1)
                cv2.drawContours(img_draw, [right], 0, (0, 0, 255), -1)
                cv2.circle(img_draw, (left_point[0], left_point[1]), 5, (0, 0, 255), -1)
                cv2.circle(img_draw, (right_point[0], right_point[1]), 5, (0, 255, 0), -1)
        else:
            door_x = 320
    elif door_state == 'PASS1':
        floor = getContour(img[0:480, 140:500], 'floor')
        door_angle = angleLine(floor, 'top', img_draw) if floor is not None else None

        door = getContour(img, 'door')
        if door is None:
            door_y = 479
        else:
            door_y = 0
    elif door_state == 'AIM2':
        floor = getContour(img[0:480, 140:500], 'floor')
        door_angle = angleLine(floor, 'top', img_draw) if floor is not None else None
    else:
        time.sleep(0.01)


def camera_agent(camera):
    global door_state

    img_draw = None
    while door_state != 'END':
        img = camera.frame
        if img is not None:
            img_draw = img.copy()
            # print('imgprocessing...')
            imgproc(img, img_draw)
            cv2.imshow('front', img_draw)
            if cv2.waitKey(1) == 27:
                door_state = 'END'
                break
    
    print('Exiting....')
    cv2.destroyAllWindows()


def pass_door(camera):
    global door_state
    # Thread 1: move
    init()

    # Thread 2: camera agent
    # camera_agent(camera)
    th_move = threading.Thread(target=move, daemon=True)
    th_move.start()
    th_camera = threading.Thread(target=camera_agent(camera), daemon=True)
    th_camera.start()

    time.sleep(0.5)

    th_move.join()
    th_camera.join()

def door_main():
    my_camera = Camera.Camera()
    my_camera.camera_open()
    pass_door(my_camera)
    my_camera.camera_close()

if __name__ == '__main__':
    door_main()