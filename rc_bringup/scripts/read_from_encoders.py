#! /usr/bin/env python
# coding: utf-8
"""
This is Ros node for reading data from encoders via serial port
"""
import time
import math
import serial
import rospy

from std_msgs.msg import Float64

vel1=float()
vel2=float()
odometry_vel=float()


encoder_vel="/encoder_vel"



if __name__ == "__main__":
	rospy.init_node("rc_control")
	vel_pub = rospy.Publisher(encoder_vel, Float64, queue_size=10)
	ser = serial.Serial(
		port='/dev/ttyACM0',
		baudrate = 9600,
		bytesize = serial.EIGHTBITS,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		xonxoff = False,
		rtscts = False,
		dsrdtr = False,
		timeout = 1)
	try:
		ser.isOpen()
	except:
		print("serial not open")
		exit(0)	

	while not rospy.is_shutdown():
		#vel_pub.publish(read_serial())     #publish car params from topic
		data = ser.readline()
		if len(data) != 0:
			for odometry_vel in data.split('\r\n'):
				if odometry_vel != "":
					try:
						vel_pub.publish(float(odometry_vel))
					except:
						print ("error pub", odometry_vel)
