# ++++++++++++++++++++ README +++++++++++++++++++++++
# ---- in the terminal you can:
# 1. name the dictionary at the very beginning
# 2. name the new dict in the terminal.
# ---- in the window you can:
# 1. <space> to read color in the circle
# 2. w to append the new dictionary to the file: include/lab_dict.py.
# 3. q to quit

import cv2 as cv
import numpy as np
import os, sys
from hiwonder import Board
import time

camera_name = 0


# file to out
outfile_path = os.path.join(os.path.dirname(__file__), 'LABDict.py')
print(os.path.abspath(outfile_path))
outfile = open(outfile_path, "a")

# create a window
def nothing(x):
    pass
cv.namedWindow('image')
cv.createTrackbar('Low L', 'image', 0, 255, nothing)
cv.createTrackbar('Low A', 'image', 0, 255, nothing)
cv.createTrackbar('Low B', 'image', 0, 255, nothing)

cv.createTrackbar('High L', 'image', 0, 255, nothing)
cv.createTrackbar('High A', 'image', 0, 255, nothing)
cv.createTrackbar('High B', 'image', 0, 255, nothing)

cap = cv.VideoCapture(camera_name)

while True:
    ret, img = cap.read()
    if ret is None:
        continue

    # do and show mask
    low_l = cv.getTrackbarPos('Low L', 'image')
    low_a = cv.getTrackbarPos('Low A', 'image')
    low_b = cv.getTrackbarPos('Low B', 'image')

    high_l = cv.getTrackbarPos('High L', 'image')
    high_a = cv.getTrackbarPos('High A', 'image')
    high_b = cv.getTrackbarPos('High B', 'image')

    img_lab = cv.cvtColor(img, cv.COLOR_BGR2LAB)
    mask = cv.inRange(img_lab, (low_l, low_a, low_b), (high_l, high_a, high_b))
    cv.imshow('mask', mask)

    h, w, ch = img.shape
    cv.circle(img,  (w//2, h//2), 15, (0, 0, 255), 2)
    cv.imshow('image', img)


    k = cv.waitKey(1) & 0xFF
    # ------------------- 摄像头控制 -----------------------
    if k == ord('q'): # quit
        break
    if k == ord(' '): # get the lab color
        bgr = img[h//2, w//2]
        print(bgr)
        l, a, b = cv.cvtColor(np.array([[bgr]]), cv.COLOR_BGR2LAB)[0, 0]
        cv.setTrackbarPos('Low L', 'image', l - 30)
        cv.setTrackbarPos('Low A', 'image', a - 30)
        cv.setTrackbarPos('Low B', 'image', b - 30)
        cv.setTrackbarPos('High L', 'image', l + 30)
        cv.setTrackbarPos('High A', 'image', a + 30)
        cv.setTrackbarPos('High B', 'image', b + 30)
    if k == ord('w'): #  write the color item
        color_name = input('input the color name: ')
        color = [(low_l, low_a, low_b), (high_l, high_a, high_b)]
        outfile.write("'" + color_name + "'" + ": " + str(color) + ",\n")
    
    # ------------------- 云台控制
    if k == 81: # left
        x_pulse = Board.getBusServoPulse(19) - 5
        Board.setBusServoPulse(19, x_pulse, 20)
        time.sleep(0.02)
    if k == 82: # up
        y_pulse = Board.getBusServoPulse(20) + 5
        Board.setBusServoPulse(20, y_pulse, 20)
        time.sleep(0.02)
    if k == 83: # right
        x_pulse = Board.getBusServoPulse(19) + 5
        Board.setBusServoPulse(19, x_pulse, 20)
        time.sleep(0.02)
    if k == 84: # down
        y_pulse = Board.getBusServoPulse(20) - 5
        Board.setBusServoPulse(20, y_pulse, 20)
        time.sleep(0.02)

cv.destroyAllWindows()
outfile.close()
