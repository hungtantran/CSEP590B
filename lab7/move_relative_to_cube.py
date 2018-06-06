#!/usr/bin/env python3

'''
This is starter code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import sys

from odometry import cozmo_go_to_pose, my_go_to_pose1
sys.path.insert(0, '../lab6')
from pose_transform import get_relative_pose

def move_relative_to_cube(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, when a cube is detected it 
	moves the robot to a given pose relative to the detected cube pose.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while cube is None:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Found a cube, pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")

	desired_pose_relative_to_cube = Pose(0, 100, 0, angle_z=degrees(90))

	# ####
	# TODO: Make the robot move to the given desired_pose_relative_to_cube.
	# Use the get_relative_pose function your implemented to determine the
	# desired robot pose relative to the robot's current pose and then use
	# one of the go_to_pose functions you implemented in Lab 6.
	# ####
	desired_robot_pose = Pose(
		cube.pose.position.x + desired_pose_relative_to_cube.position.x,
		cube.pose.position.y + desired_pose_relative_to_cube.position.y,
		0,
		angle_z = degrees(
			cube.pose.rotation.angle_z.degrees +
			desired_pose_relative_to_cube.rotation.angle_z.degrees)
	)
	desired_robot_relative_pose = get_relative_pose(desired_robot_pose, robot.pose)
	my_go_to_pose1(
		robot,
		desired_robot_relative_pose.position.x,
		desired_robot_relative_pose.position.y,
		desired_robot_relative_pose.rotation.angle_z.degrees)

if __name__ == '__main__':

	cozmo.run_program(move_relative_to_cube)
