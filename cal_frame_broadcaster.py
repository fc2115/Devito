#!/usr/bin/env python

####################################################################################
#
# Run this file first.
# Continuously broadcast newly calibrated base frame to ROS /tf
#
####################################################################################

import roslib
# ?? roslib/load_manifest()
import rospy
import thread
import math
import tf
import thread
from sensor_msgs.msg import Joy
import geometry_msgs.msg
import time
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

####################################################################################


if __name__ == '__main__':
	rospy.init_node('cal_frame_broadcaster')  # Initialise node
	listener = tf.TransformListener()  # Creates a listener to obtain the transformation data between the tf frames.
	br = tf.TransformBroadcaster()  # Creates a broadcaster to stream the tf data to ROS /tf

	# Wait for user to be ready and then press Enter to set base frame.
	c = raw_input("MOVE CONTROLLER TO BASE FRAME")
	first_run = True
	rate = rospy.Rate(100.0)

	while not rospy.is_shutdown():
		if first_run:  # On first run find base frame, in subsequent runs only broadcast to ROS /tf
			try:  # Find transformation from lighthouse to left controller.
				(trans_l, quart_l) = listener.lookupTransform('/lighthouse_1', '/left_controller', rospy.Time(0))
				first_run = False  # Ensures base frame is not updated.
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		# Broadcast the base frame to ROS /tf
		br.sendTransform(trans_l, quart_l, rospy.Time.now(), 'cal_frame', '/lighthouse_1')
		rospy.sleep(0.01)