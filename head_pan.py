#!/usr/bin/env python

####################################################################################################
#
# Node to control the movement of DENIRO's screen about the Z axis based on VIVE headset orientation
#
####################################################################################################

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import tf
import numpy as np
import time

####################################################################################################


rospy.init_node("move_baxter_head")
head = baxter_interface.Head()  # Initialise head
listener = tf.TransformListener()  # Creates a listener to obtain the transformation data between the tf frames.
axis = 2  # Select Z axis

done = False

if __name__ == '__main__':
	while not rospy.is_shutdown():
		try:  # Attempt to obtain orientation for DENIRO's head based on the HMD tf
			_, quat = listener.lookupTransform('/hmd', 'cal_frame', rospy.Time(0))
			angle = tf.transformations.euler_from_quaternion(quat)[2]

			# Setting limits for DENIRO neck rotation.
			diff = -angle
			threshold = 1.3
			if diff < -threshold:
				diff = -threshold
			elif diff > threshold:
				diff = threshold

			# Rotate head
			head.set_pan(diff, speed=1.0, timeout=0)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		time.sleep(0.1)