import cv2
import numpy as np
#填写英文路径
#运行程序
#左键点击
#即可显示坐标
# q 关闭程序

img = cv2.imread("path")


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "(%d,%d)" % (x, y)
        print (xy)
        cv2.circle(img, (x, y), 1, (0, 255, 0), thickness = -1)
        if(x<=img.shape[1]/2):
            cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    2.0, (0,255,0), thickness = 1)
        else:
            x1 = x - img.shape[1]//9
            y1 = y
            cv2.putText(img, xy, (x1,y1), cv2.FONT_HERSHEY_PLAIN,
                    2.0, (0,255,0), thickness = 1)
        cv2.imshow("image", img)

cv2.namedWindow("image")
cv2.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
cv2.imshow("image", img)
while True:
    if cv2.waitKey(0) == ord('q'):       
        cv2.destroyAllWindows()
        break