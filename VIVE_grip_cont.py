#!/usr/bin/env python

####################################################################################################
#
# Node to control the movement of DENIRO's grippers based on the VIVE controller's Triggers.
#
####################################################################################################

import rospy
from sensor_msgs.msg import Joy
import baxter_external_devices
import baxter_interface
import time
import thread

####################################################################################################


# Called every time the subscriber is updated, listens for button presses.
def left_callback(data):
	global pause_bang, pause

	# Sends "bang" signal into pause_switch() function when Menu (pause) Button pressed
	if data.buttons[3]:
		pause_bang = True
		time.sleep(0.1)
		pause_bang = False

	if not pause:  # Checks to see if system is paused before running operation.
		trig = 100 - (data.axes[0] * 100)  # Scales and maps Trigger angle to gripper contraction.
		# Set safety limits to gripper extension/contraction
		if trig >= 99:
			trig = 99
		if trig <= 1:
			trig = 1
		grip_left.command_position(trig)  # Sets gripper position


# Mirror of left_callback()
def right_callback(data):
	global pause_bang, pause
	if data.buttons[3]:
		pause_bang = True
		time.sleep(0.1)
		pause_bang = False
		print "click"
	if not pause:

		trig = 100 - (data.axes[0] * 100)
		if trig >= 90:
			trig = 90
		if trig <= 10:
			trig = 10
		print type(trig)
		grip_right.command_position(trig)


# Switches the pause boolean when pause bang is detected, pausing or resuming gripper motion.
def pause_switch():
	global pause_bang, pause
	while True:
		if pause_bang:  # Receives signal when Menu Button is pressed.
			if pause:  # Sets to resume if paused.
				pause = False
				time.sleep(0.5)
			else:  # Sets to pause if moving.
				pause = True
				time.sleep(0.5)


# Subscribes to VIVE Joy topic.
def listener():
	rospy.Subscriber("/vive_right", Joy, right_callback)
	rospy.Subscriber("/vive_left", Joy, left_callback)
	rospy.spin()


pause = False
pause_bang = False
if __name__ == '__main__':
	# Initialise conditions
	rospy.init_node('VIVE_Grip')
	grip_left = baxter_interface.Gripper('left')
	grip_right = baxter_interface.Gripper('right')

	# Automatic gripper calibration.
	grip_left.calibrate()
	grip_right.calibrate()

	thread.start_new_thread(pause_switch, ())  # Starts new thread to run callbacks and pause_switch simultaneously.

	listener()
