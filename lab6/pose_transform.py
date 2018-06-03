#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import cozmo
import numpy
from cozmo.util import degrees
import time
import math

def get_relative_pose(obj_pose, ref_pose):
	new_angle_z = obj_pose.rotation.angle_z - ref_pose.rotation.angle_z

	ref_pos = ref_pose.position
	obj_pos = obj_pose.position

	distance = math.sqrt(
		math.pow(ref_pos.x - obj_pos.x, 2) +
		math.pow(ref_pos.y - obj_pos.y, 2)
	)
	angle = math.radians(90 + ref_pose.rotation.angle_z.degrees - math.degrees(
		math.atan((obj_pos.y - ref_pos.y)/(obj_pos.x - ref_pos.x))))
	rel_x = distance * math.sin(angle)
	rel_y = distance * math.cos(angle)
	new_pos = cozmo.util.Position(rel_x, rel_y, 0)

	rel_pose = cozmo.util.Pose(
		rel_x, rel_y, 0,
		angle_z=new_angle_z)
	return rel_pose

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" %
					get_relative_pose(cube.pose, robot.pose))
				time.sleep(5)
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
