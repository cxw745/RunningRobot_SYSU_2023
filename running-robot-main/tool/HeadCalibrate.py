DEBUG_LOGIC = True
if not DEBUG_LOGIC:
    from hiwonder import Board
import cv2 as cv
import numpy as np
import time

YAW_ID = 19
PITCH_ID = 20

DEV_RANGE_HALVE = 100

if not DEBUG_LOGIC:
    Board.setBusServoPulse(YAW_ID, 500, 100)
    Board.setBusServoPulse(PITCH_ID, 500, 100)  
    time.sleep(2)

if not DEBUG_LOGIC:
    dev_yaw = Board.getBusServoDeviation(YAW_ID)
    dev_pitch = Board.getBusServoDeviation(PITCH_ID)
else:
    dev_yaw = 50
    dev_pitch = -10

def set_dev_yaw(pos):
    if not DEBUG_LOGIC:
        Board.setBusServoDeviation(YAW_ID, pos)
    print(pos)

def set_dev_pitch(pos):
    if not DEBUG_LOGIC:
        Board.setBusServoDeviation(PITCH_ID, pos)
    print(pos)

cv.namedWindow('adj')
def setServoTrackBar(id, org, onChange):
    cv.createTrackbar(str(id), 'adj', org, DEV_RANGE_HALVE, onChange)
    cv.setTrackbarMin(str(id), 'adj', -DEV_RANGE_HALVE)

setServoTrackBar(YAW_ID, dev_yaw, set_dev_yaw)
setServoTrackBar(PITCH_ID, dev_pitch, set_dev_pitch)

cap = cv.VideoCapture(0)
while True:
    ret, img = cap.read()
    if not ret:
        break
    cv.imshow('adj', img)
    if cv.waitKey(25) == ord('q'):
        break
if not DEBUG_LOGIC:
    Board.saveBusServoDeviation(YAW_ID)
    Board.saveBusServoDeviation(PITCH_ID)
cv.destroyAllWindows()