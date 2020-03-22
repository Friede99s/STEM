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
                self.cmd.drive_angle = 100

                self.cartPoints = [None for x in range(500)]
                self.finalVector = [100, 0]
                self.max= 255


        def scan_callback(self, data):
                '''Checks LIDAR data'''
                self.data = data.ranges
                self.drive_callback()

        def camera_callback(self, msg):
                '''camera_callback'''

        def machine_learning_callback(self, msg):
                '''machine_learning_callback'''

        def drive_callback(self):
                '''Publishes drive commands'''
                left = drvcal.calc(self.data[34:100])*100
		
		right = drvcal.calc(self.data[160:250])*100

                front = (drvcal.calc(self.data[0:34] + self.data[250:300]))*100
                error = left - right
                PID_Angle = PID.PIDCalc(error, 0.01)
                print("left:%0.4f, right:%0.4f, front:%0.4f"%(left, right, front))
                if PID_Angle > 255:
                    PID_Angle = 255
                elif PID_Angle < -255:
                    PID_Angle = -255

                self.cmd.velocity = 200
                self.cmd.drive_angle = PID_Angle
                #if front == 0:
                 #   front = 60
                if front < 35:
                    self.cmd.drive_angle = -253
                    self.cmd.velocity = -255

                self.drive_pub.publish(self.cmd)

class FindLeast:
    def calc(self, value):
        least = 10
        for i in range(len(value)):
            if value[i] != 0:
                if value[i] < least:
                    least = value[i]
        return least

class Angle_cal:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.p = 0
        self.i = 0
        self.d = 0
    def PIDCalc(self, error, interval):
        self.p = error * self.Kp
        self.i += (error * self.Ki * interval)
        self.d = (error-self.pre_error) / self.Kd
        self.pre_error = error
        return self.p + self.i + self.d
		

if __name__ == "__main__":
	try:
		PID = Angle_cal(5, 0.001, 16)
                drvcal = FindLeast()
		node = Drive()
		rospy.spin()		
	except rospy.ROSInterruptException:
		exit()
