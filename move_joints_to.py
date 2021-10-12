#!/usr/bin/env python

########################################################################
#
# Movement Test file. If DE NIRO does not move, turn it off and on again
# Never exit a program with Ctrl + X. Only Ctrl + C.
#
########################################################################

import sys
import time
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

def main():
	rospy.init_node("move_joints_to")

	# Initialising the baxter limb, head, gripper
	left = baxter_interface.Limb('left')
	grip_left = baxter_interface.Gripper('left')
	head = baxter_interface.Head()
	left.set_joint_position_speed(0.4)

	# Testing the head movement
	head.set_pan(0.0)
	head.set_pan(1.4)
	head.set_pan(-1.4)
	head.set_pan(0.0)

	# Testing the limb movement
	left.move_to_neutral(timeout=15.0)

	# Testing the gripper movement
	grip_left.calibrate()
	grip_left.close()
	time.sleep(2)
	grip_left.open()

 	lj = left.joint_names()
	print lj
	current_position = left.joint_angles()
	print current_position

	# Pre defined positions from baxter
	positions = dict(zip(left.joint_names(), [-0.9804847644947284, -0.07547962270476974, 2.8779113859306897, 0.6242229914402619, -2.5616644372112836, 1.7232777704927458, -1.0106066157629539]))

	# move to this pre defined position
	left.move_to_joint_positions(positions, timeout=15.0, threshold=0.008726646)
	current_position = left.joint_angles()

	# Check it has moved to the correct joint angles
	print current_position
	rospy.sleep(1.)

	# Return to neutral
	left.move_to_neutral(timeout=15.0)
	current_position = left.joint_angles()

	# Check for neutral
	print current_position

if __name__ == '__main__':
	main()
