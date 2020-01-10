#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from racecar_ws.msg import drive_msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

AUTONOMOUS_MODE=False
count = 0
x, y = []
Rx, Ry = 0


print("Complete to open Drive Node")

class Drive:
	def __init__(self):
		rospy.init_node("potentialField")
		self.data = None
		self.cmd = drive_msg()
                self.camera_sub = rospy.Subscriber("/camera", Image, self.camera_callback)
		self.ML_sub = rospy.Subscriber("/teachable_machine", Float32MultiArray, self.machine_learning_callback)
		self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback) 
		self.drive_pub = rospy.Publisher("/drive", drive_msg, queue_size=1)
		self.cmd.velocity = 100
		self.cmd.drive_angle = 0

		self.cartPoints = [None for x in range(500)]
		self.finalVector = [100, 0]
		self.max= 255

	
	def scan_callback(self, data):
		'''Checks LIDAR data'''
		self.data = data.ranges
		for i in range(500):
			x.append(self.data[i]*cos(i/500*360))
			y.append(self.data[i]*sin(i/500*360))
			print(x[i], y[i])
		for i in range(90, 270):
			x[i] = 0
			y[i] = 0
			
		self.drive_callback()
		
        def camera_callback(self, msg):
		'''camera_callback'''

	def machine_learning_callback(self, msg):
		'''machine_learning_callback'''

	def drive_callback(self):
		'''Publishes drive commands'''
		self.drive_pub.publish(self.cmd)
		Rx = sum(x)
		Ry = sum(y)
		if Rx < 0:
			Rx = Rx * -1
		if Ry < 0:
			Ry = Ry * -1
		if math.atan(float(Ry/Rx))-90 > 0:
			self.cmd.drive_angle = -255
		elif math.atan(float(Ry/Rx))-90 < 0:
			self.cmd.drive_angle = 255
		self.cmd.velocity = 255
		

if __name__ == "__main__":
	try:
		node = Drive()
		rospy.spin()		
	except rospy.ROSInterruptException:
		exit()
