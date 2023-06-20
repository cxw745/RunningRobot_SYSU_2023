import cv2 as cv
import numpy as np


def showKeyBoard():
    img = np.zeros((50, 50, 3), np.uint8)
    while True:
        cv.imshow('img', img)
        k = cv.waitKey(0)
        print(k)
        if k == ord('q'):
            break
