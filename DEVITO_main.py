#!/usr/bin/env python

####################################################################################
#
# DEVITO main file, ran after cal_frame_broadcaster.py, in conjunction with VIVE_grip_cont.py and head_pan.py
# Provides primary functionality in teleoperating DENIRO using HTC VIVE.
#
####################################################################################

import sys
import rospy
import baxter_interface
from trac_ik_python.trac_ik import IK
import thread
import roslib
# ?? roslib/load_manifest()
import rospy
import math
import tf
from sensor_msgs.msg import Joy
import geometry_msgs.msg
import time
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import baxter_external_devices

####################################################################################


# Callback Function: Called every time the subscriber is updated.
# Sends signals from VIVE Buttons to Baxter.
# See Report sections 3.3.3 & 3.5.4 for more details
def callback(data):
	global grip_left, cal, passer, pause
	passer = 0
	if cal:  # When calibrating, the Grip Buttons are used to record and progress. Otherwise null
		if data.buttons[4]:
			print data.buttons[4]
			passer = 1
			time.sleep(0.5)
		else:
			print data.buttons[4]
			passer = 0

	if data.buttons[3]:  # The menu button is used to pause the robot.
		if pause == 1:
			pause = 0
			time.sleep(0.1)
		else:
			pause = 1
			time.sleep(0.1)


# Run continuously, listening for VIVE Joy messages.
# See Report sections 3.3.3 & 3.5.4 for more details
def listener_thread():
	rospy.Subscriber("/vive_left", Joy, callback)
	rospy.spin()


"""
Scale:
Scale is a linear transform of one number in a range to another number in a different range.
Used for scaling XYZ coordinates.

Range of user:
A = person lower bound
B = person upper bound

Range of robot:
C = Baxter lower bound
D = Baxter upper bound

x = tf transform of controller to base frame.
Returns the scaled output for DENIRO
"""

# See Report section 3.3.2 for more details
def scale(x, a, b, c, d):
	return ((x-a)*(d-c)/(b-a)) + c


if __name__ == '__main__':
	global grip_left, passer, pause, cal
	passer = 0
	pause = 0
	grip_left = None
	rospy.init_node('DEVITO')
	thread.start_new_thread(listener_thread, ())  # Branches out into a new thread and listens for controller commands
	print 'listening'

	urdf_str = rospy.get_param('/robot_description')  # Obtain universal robot description format of DENIRO for IK
	ik_solver = IK(base_link="base", tip_link="left_gripper", urdf_string=urdf_str)  # Creates instance of IK solver
	ik_solver.set_joint_limits([-1.70168, -2.147, -3.05418, -0.05, -3.059, -1.5708, -3.059],
								[1.70168, 1.047, 3.05418, 2.618, 3.059, 2.094, 3.059])  # Setting DENIRO's joint limits
	cal = True
	first_run = True  # Sets booleans to True for first run calibration.

	# Initialising the left arm
	left = baxter_interface.Limb('left')
	grip_left = baxter_interface.Gripper('left')
	lj = left.joint_names()
	left.set_joint_position_speed(0.35)  # Set here to prevent jerky movements
	left.move_to_neutral(timeout=15.0)

	# Creates a listener to obtain the transformation data between the tf frames.
	listener = tf.TransformListener()

	# Main Loop
	while not rospy.is_shutdown():

		if cal:
			left.set_joint_position_speed(0.4)  # Set again to achieve smooth movement

			# See Report sections 3.3.2 and 3.3.3 for more details
			print '*** CALIBRATING ***'

			print "Press the Grip Button when the arm is fully extended forwards"
			while passer == 0:  # Passer will remain at 0 until Grip Button is pressed.
				time.sleep(0.3)

			try:  # Attempt to obtain the transform between the calibrated base frame and the controller.
				(cal_lt3, cal_lq2) = listener.lookupTransform('cal_frame', '/left_controller', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			x_extended = cal_lt3[0]  # Save the user's upper bound in X.
			print x_extended  # Sanity check
			time.sleep(0.7)

			# Above steps repeated for Y and Z limits.
			print "Press the Grip Button when the arm is fully extended upwards"
			while passer == 0:
				time.sleep(0.3)

			try:
				(cal_lt4, cal_lq2) = listener.lookupTransform('cal_frame', '/left_controller', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			x_body_pos = cal_lt4[0]  # Lower bound in X
			print 'xbodypos = ', x_body_pos  # Sanity Check
			z_extended_up = cal_lt4[2]  # Upper bound in Z
			print z_extended_up  # Sanity Check
			time.sleep(0.7)

			print "Press Grip Button when the arm is fully extended downwards"
			while passer == 0:
				time.sleep(0.3)

			try:
				(cal_lt5, cal_lq2) = listener.lookupTransform('/cal_frame', '/left_controller', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			z_extended_down = cal_lt5[2]  # Lower bound in Z
			print z_extended_down  # Sanity Check
			time.sleep(0.7)

			print "Press Grip Button when the arm is fully extended left"
			while passer == 0:
				time.sleep(0.3)

			try:
				(cal_lt6, cal_lq2) = listener.lookupTransform('/cal_frame', '/left_controller', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			y_extended_left = cal_lt6[1]  # Lower bound in Y
			print y_extended_left  # Sanity Check
			time.sleep(0.7)

			print "Press Grip Button when the arm is fully extended right"
			while passer == 0:
				time.sleep(0.3)

			try:
				(cal_lt7, cal_lq2) = listener.lookupTransform('/cal_frame', '/left_controller', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

			y_extended_right = cal_lt7[1]  # Upper bound in Y
			print y_extended_right  # Sanity Check

			# BAXTER MOVEMENT BOUNDS see report section 3.4.5
			x_forward = 1.2
			x_hug = 0.1

			y_left = 1.2
			y_right = -0.3

			z_up = 1.4
			z_down = -0.3

			cal = False  # Calibration is complete, do not run this loop again.
			print "***CALIBRATED***"
			print "PRESS GRIP BUTTON WHEN READY TO MOVE"
			while passer == 0:
				time.sleep(0.3)

		"""User is now controlling DENIRO's left arm movement."""

		try:  # Obtain transform between left controller and calibration frame
			(trans_l, quartl) = listener.lookupTransform('/left_controller', '/cal_frame', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		# Fix the rotation between controller and DENIRO. See report section 3.3.2
		rot_l = euler_from_quaternion([quartl[0], quartl[1], quartl[2], quartl[3]])
		quart_l = quaternion_from_euler(1*rot_l[0], 1*rot_l[1] + np.pi/2, 1*rot_l[2])

		# Final tweaking of workspace scaling. See report section 3.3.2
		off_y = abs(y_extended_left) - abs(y_extended_right)
		off_z = abs(z_extended_down) - abs(z_extended_up)
		transl_x = -1*trans_l[0]
		transl_y = -1*trans_l[1] + off_y
		transl_z = -1.2*trans_l[2] - off_z

		# From controller, left, up , forwards = decrease
		# From DENIRO, right, down, back = decrease
		# When calibrating, right, back, down = decrease

		xl = scale(transl_x, x_body_pos, x_extended, x_hug, x_forward)
		yl = scale(transl_y, y_extended_left, y_extended_right, y_left, y_right)
		zl = scale(transl_z, z_extended_down, z_extended_up, z_down, z_up)

		[Qwl, Qxl, Qyl, Qzl] = quart_l
		print 'Vive Co-ordinates = ', transl_x, transl_y, transl_z  # Ensure VIVE is still publishing
		print 'Target DENIRO Co-ordinates = ', xl, yl, zl  # Sanity check

		# Set the pose tolerances. See report section 3.4.5
		bx = by = bz = 0.01
		brx = bry = brz = 0.1

		# Obtain current joint positions
		current_position = left.joint_angles()
		temp = []
		dictlist = []
		for key, value in current_position.iteritems():  # Removing values from dictionary for simple manipulation.
			temp = [key, value]
			dictlist.append(temp[1])

		# Reorder joint angles to comply with solver.
		ordered_joints = [dictlist[5], dictlist[6], dictlist[3], dictlist[4], dictlist[0], dictlist[1], dictlist[2]]

		# Seed current ordered joint angles into IK solver. See report section 3.4.5
		seed_state = ordered_joints

		# Run IK solver to obtain joint solutions.
		joints_from_ik1 = ik_solver.get_ik(seed_state, xl, yl, zl, Qwl, Qxl, Qyl, Qzl, bx, by, bz, brx, bry, brz)
		print joints_from_ik1, 'Joint Solutions'  # Sanity Check

		# Prevent errors if no joint solutions can be found.
		if not joints_from_ik1:
			print 'Passed'
			pass

		# Controlled 5 second movement to first solved position for safety reasons.
		elif first_run:
			joints_from_ik = list(joints_from_ik1)
			positions = dict(zip(left.joint_names(), joints_from_ik))
			left.move_to_joint_positions(positions, timeout=5.0, threshold=0.008726646)
			first_run = False  # Will move in real time in subsequent motion.

		elif joints_from_ik1:
			joints_from_ik = list(joints_from_ik1)
			positions = dict(zip(left.joint_names(), joints_from_ik1))
			left.set_joint_positions(positions)

		# If pause variable set to 1 in listener thread, program will wait in this loop until set back to 0.
		# See report section 3.5.4
		while pause == 1:
			print pause
			time.sleep(0.1)

		time.sleep(0.1)
