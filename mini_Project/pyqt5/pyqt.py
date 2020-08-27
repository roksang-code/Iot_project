import RPi.GPIO as GPIO
import smbus
import time
import sys
import smbus
import cv2
import threading
from PyQt5.QtCore import *
from PyQt5 import QtGui
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QImage,QPixmap
form_class = uic.loadUiType("pyqt.ui")[0]


sht20_addr = 0x40
SHT20_CMD_MEASURE_TEMP = 0xf3
SHT20_CMD_MEASURE_HUMI = 0xf5
SHT20_SOFT_RESET = 0xfe
bus = smbus.SMBus(1)
bus.write_byte(sht20_addr, SHT20_SOFT_RESET)
time.sleep(0.05)
data = [0, 0]


led = 20
e_pin = 12
p_pin = 4
n_pin = 25


GPIO.setmode(GPIO.BCM)
GPIO.setup(led, GPIO.OUT)
GPIO.setup(e_pin,GPIO.OUT)
GPIO.setup(p_pin,GPIO.OUT)
GPIO.setup(n_pin,GPIO.OUT)


servo_addr = 0x20
OUT_PORT0 = 0x02
OUT_PORT1 = 0x03
CONFIG_PORT0 = 0x06
CONFIG_PORT1 = 0x07
POLARITY_IVE_PORT0 = 0x04
POLARITY_IVE_PORT1 = 0x05
servoUD = 0x01
servoLR = 0x02
Phase_1 =  [0x01, 0x02, 0x03, 0x04]
bus = smbus.SMBus(1)

class cam_thread(QThread):
	dis_val = pyqtSignal(QImage)

	def __init__(self, parent=None):
		super(self.__class__,self).__init__(parent)

	def run(self):
		camera = cv2.VideoCapture(0)
		print("cam")
		while True:
			ret, img = camera.read()
			if ret:
				height, width = img.shape[:2]
				img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
				h,w,c = img.shape
				qImg = QtGui.QImage(img.data, w, h, w*c, QtGui.QImage.Format_RGB888)
				self.dis_val.emit(qImg)

class temp_thread(QThread):
	dis_val = pyqtSignal(tuple)

	def __init__(self, parent=None):
		super(self.__class__,self).__init__(parent)
	def run(self):
		while True:
			bus.write_byte(sht20_addr, SHT20_CMD_MEASURE_TEMP)
			time.sleep(0.26)
			for i in range(2):
				data[i] = bus.read_byte(sht20_addr)
			value = data[0] << 8 | data[1]
			temp = -46.84 + 175.72 / 65536 * int(value)
			bus.write_byte(sht20_addr, SHT20_CMD_MEASURE_HUMI)
			time.sleep(0.26)
			for i in range(2):
				data[i] = bus.read_byte(sht20_addr)
			value = data[0] << 8 | data[1]
			humi = -6.0 + 125.0 / 65536 * int(value)
			temp = (str(round(float(temp),2)))
			humi = (str(round(float(humi),2)))
			result = temp, humi
			self.dis_val.emit(result)

class WindowClass(QMainWindow, form_class) :

	def __init__(self) :
		super().__init__()
		self.setupUi(self)
		self.tempth = temp_thread()
		self.tempth.dis_val.connect(self.connect_temp)
		self.tempth.start()
		self.camth = cam_thread()
		self.camth.dis_val.connect(self.connect_cam)
		self.camth.start()

		self.ledOn.clicked.connect(self.ledOnFunction)
		self.ledOff.clicked.connect(self.ledOffFunction)
		self.leftM.clicked.connect(self.motorL)
		self.stopM.clicked.connect(self.motorS)
		self.rightM.clicked.connect(self.motorR)
		self.up.clicked.connect(self.cam_up)
		self.down.clicked.connect(self.cam_down)
		self.left.clicked.connect(self.cam_left)
		self.right.clicked.connect(self.cam_right)


	def connect_cam(self,qImg):
		self.cam.setPixmap(QPixmap.fromImage(qImg))

	def connect_temp(self, result):
		self.temp.setText(result[0]+"℃")
		self.humi.setText(result[1]+"%")


	def ledOnFunction(self) :
		GPIO.output(led, GPIO.HIGH)

	def ledOffFunction(self) :
		GPIO.output(led, GPIO.LOW)


	def motorL(self) :

		GPIO.output(e_pin, GPIO.HIGH)
		GPIO.output(p_pin, GPIO.LOW)
		GPIO.output(n_pin, GPIO.HIGH)

	def motorR(self) :

		GPIO.output(e_pin, GPIO.HIGH)
		GPIO.output(p_pin, GPIO.HIGH)
		GPIO.output(n_pin, GPIO.LOW)

	def motorS(self) :

		GPIO.output(e_pin, GPIO.LOW)
		GPIO.output(p_pin, GPIO.LOW)
		GPIO.output(n_pin, GPIO.LOW)

	def cam_up(self):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x02)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x03)
		for i in range(2):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])
	def cam_down(self):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x02)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x03)
		for i in range(2):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])
			time.sleep(0.0015)
	def cam_left(self):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x01)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x02)
		for i in range(4):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])
			time.sleep(0.0015)
	def cam_right(self):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x01)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x02)
		for i in range(4):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])

if __name__ == "__main__" :
	app = QApplication(sys.argv)
	myWindow = WindowClass()
	myWindow.show()
	app.exec_()
