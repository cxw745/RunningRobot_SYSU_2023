# -*- coding: utf-8 -*-
# 点击 "生成色彩范围" 后，相应的色彩范围会以 [Lower, Upper] 的格式写入color_dict.py
import time
import sys
import cv2
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtWidgets import * 
from PyQt5.QtGui import *
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(920, 640)
        Form.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.Imask = QtWidgets.QLabel(Form)
        self.Imask.setGeometry(QtCore.QRect(480, 20, 400, 300))
        self.Imask.setObjectName("Imask")
        self.origin = QtWidgets.QLabel(Form)
        self.origin.setGeometry(QtCore.QRect(40, 20, 400, 300))
        self.origin.setObjectName("origin")
        self.L_low = QtWidgets.QSlider(Form)
        self.L_low.setGeometry(QtCore.QRect(40, 360, 400, 20))
        self.L_low.setMaximum(255)
        self.L_low.setOrientation(QtCore.Qt.Horizontal)
        self.L_low.setObjectName("L_low")
        self.A_low = QtWidgets.QSlider(Form)
        self.A_low.setGeometry(QtCore.QRect(40, 440, 400, 20))
        self.A_low.setMaximum(255)
        self.A_low.setOrientation(QtCore.Qt.Horizontal)
        self.A_low.setObjectName("A_low")
        self.B_low = QtWidgets.QSlider(Form)
        self.B_low.setGeometry(QtCore.QRect(40, 520, 400, 20))
        self.B_low.setMaximum(255)
        self.B_low.setOrientation(QtCore.Qt.Horizontal)
        self.B_low.setObjectName("B_low")
        self.L_high = QtWidgets.QSlider(Form)
        self.L_high.setGeometry(QtCore.QRect(480, 360, 400, 20))
        self.L_high.setMaximum(255)
        self.L_high.setSliderPosition(255)
        self.L_high.setOrientation(QtCore.Qt.Horizontal)
        self.L_high.setObjectName("L_high")
        self.A_high = QtWidgets.QSlider(Form)
        self.A_high.setGeometry(QtCore.QRect(480, 440, 400, 20))
        self.A_high.setMaximum(255)
        self.A_high.setSliderPosition(255)
        self.A_high.setOrientation(QtCore.Qt.Horizontal)
        self.A_high.setObjectName("A_high")
        self.B_high = QtWidgets.QSlider(Form)
        self.B_high.setGeometry(QtCore.QRect(480, 520, 400, 20))
        self.B_high.setMaximum(255)
        self.B_high.setSliderPosition(255)
        self.B_high.setOrientation(QtCore.Qt.Horizontal)
        self.B_high.setObjectName("B_high")
        self.pushButton = QtWidgets.QPushButton(Form)
        self.pushButton.setGeometry(QtCore.QRect(400, 570, 131, 31))
        self.pushButton.setStyleSheet("background-color: rgb(255, 255, 255);\n"
"border-radius: 10px;  \n"
"border: 2px groove gray;")
        self.pushButton.setAutoRepeat(False)
        self.pushButton.setAutoDefault(False)
        self.pushButton.setDefault(False)
        self.pushButton.setFlat(False)
        self.pushButton.setObjectName("pushButton")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.Imask.setText(_translate("Form", "TextLabel"))
        self.origin.setText(_translate("Form", "TextLabel"))
        self.pushButton.setText(_translate("Form", "生成色彩范围"))



class CoperQt(QtWidgets.QMainWindow,Ui_Form):#创建一个Qt对象

    def __init__(self):
        self.timer_camera = QtCore.QTimer() # 定时器
        self.cap = cv2.VideoCapture(0)      # 准备获取图像 打开默认摄像头（序号0）
        
        self.outfile = open('..lab_dict.py', "a")

        QtWidgets.QMainWindow.__init__(self)    # 创建主界面对象
        Ui_Form.__init__(self)                  # 主界面对象初始化
        self.setupUi(self)                      # 配置主界面对象

        # 匹配槽
        self.timer_camera.timeout.connect(self.showCamera)  # 显示摄像头画面
        self.pushButton.clicked.connect(self.get_color_range)       # 生成色彩范围

        self.timer_camera.start(70)     # 开启计数器


    def showCamera(self):
        self.ret, self.image = self.cap.read()
        # print(self.image.shape) ------> (480, 640, 3)
        # show = cv2.resize(self.image,(image_width, image_height))
        img = cv2.resize(self.image, (400, 300))

        img_RGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_RGB = QImage(img_RGB.data, img_RGB.shape[1], img_RGB.shape[0], QImage.Format_RGB888)
        self.origin.setPixmap(QPixmap.fromImage(img_RGB))

        img_LAB = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        L_l = self.L_low.value()
        L_a = self.A_low.value()
        L_b = self.B_low.value()
        H_l = self.L_high.value()
        H_a = self.A_high.value()
        H_b = self.B_high.value()

        Imask = cv2.inRange(img_LAB, (L_l, L_a, L_b), (H_l, H_a, H_b))
        showImage = QImage(Imask.data, Imask.shape[1], Imask.shape[0], QImage.Format_Indexed8)
        self.Imask.setPixmap(QPixmap.fromImage(showImage))



    def get_color_range(self):
        self.timer_camera.stop()                    # 暂停计数
        self.image = self.cap.read()[-1]          # 获取当前画面
        L_l = self.L_low.value()
        L_a = self.A_low.value()
        L_b = self.B_low.value()
        H_l = self.L_high.value()
        H_a = self.A_high.value()
        H_b = self.B_high.value()
        inform = '[('+ str(L_l)+', '+str(L_a)+', '+str(L_b)+'), ('+ str(H_l) +', '+ str(H_a) +', '+ str(H_b) + ')]\n'
        print(inform)
        self.outfile.write(inform)
        time.sleep(0.5)                               # 延时1s
        self.timer_camera.start(70)                 # 开启计数

    def __del__(self):
        self.outfile.close()
        self.timer_camera.stop()        # 关闭计数器
        self.cap.release()
        

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = CoperQt()  # 创建QT对象
    window.show()       # QT对象显示
    sys.exit(app.exec_())