#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius(robot):
	"""Returns the radius of the Cozmo robot's front wheel in millimeters."""
	# To calculate the front wheel radius, I retried using the cozmo_drive_straight
	# method multiple time with different dist parameter, trying to get the front
	# wheel to turn a full circle. In that case, I knew that the dist = circumference
	# of the frontend. I found that the circumference of the wheel is 85mm so its
	# radius is 85mm / (2 * pi) = 13.528
	return 13.528

def get_distance_between_wheels():
	"""Returns the distance between the wheels of the Cozmo robot in millimeters."""
	# ####
	# TODO: Empirically determine the distance between the wheels of the robot using
	# robot.drive_wheels() function. Write a comment that explains how you determined
	# it and any computation you do as part of this function.
	# ####
	# From the formula in class, we can derive that:
	#   distance_between_wheels = (distance_left - distance_right) / angle
	# After multiple tries, I got that running robot.drive_wheels(38, 25, duration=12)
	# made the robot turn a right angle (90 degree, pi/2 radian)
	# So distance_between_wheels = (38*12 - 25*12) / (pi/2)
	return 55

def rotate_front_wheel(robot, angle_deg):
	"""Rotates the front wheel of the robot by a desired angle.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle_deg -- Desired rotation of the wheel in degrees
	"""
	distance = float(angle_deg)/360 * (2 * math.pi * get_front_wheel_radius(robot))
	cozmo_drive_straight(robot, distance, 40)

def my_drive_straight(robot, dist, speed):
	"""Drives the robot straight.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		dist -- Desired distance of the movement in millimeters
		speed -- Desired speed of the movement in millimeters per second
	"""
	duration = float(dist) / speed
	robot.drive_wheels(speed, speed, duration=duration)

def my_turn_in_place(robot, angle, speed):
	"""Rotates the robot in place.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		angle -- Desired distance of the movement in degrees
		speed -- Desired speed of the movement in degrees per second
	"""
	angle_radian = float(angle) / 360 * 2 * math.pi
	dist = angle_radian * get_distance_between_wheels()
	duration = float(abs(angle)) / speed
	drive_speed = dist / duration
	print(duration, drive_speed, dist)
	robot.drive_wheels(-drive_speed, drive_speed, duration=duration)

def my_go_to_pose1(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the my_drive_straight and my_turn_in_place functions. This should
	# include a sequence of turning in place, moving straight, and then turning
	# again at the target to get to the desired rotation (Approach 1).
	# ####

	# Turn to the target
	turn_angle = math.degrees(math.atan(float(y)/x))
	my_turn_in_place(robot, turn_angle, 5)
	print("Turn angle = ", turn_angle)
	# Move to the target
	distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
	my_drive_straight(robot, distance, 10)
	print("Drive straight = ", distance)
	# Turn in place to desired heading
	heading_angle = angle_z - turn_angle
	my_turn_in_place(robot, heading_angle, 5)
	print("Heading angle = ", heading_angle)

def my_go_to_pose2(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# using the robot.drive_wheels() function to jointly move and rotate the 
	# robot to reduce distance between current and desired pose (Approach 2).
	# ####
	pass


def my_go_to_pose3(robot, x, y, angle_z):
	"""Moves the robot to a pose relative to its current pose.
		Arguments:
		robot -- the Cozmo robot instance passed to the function
		x,y -- Desired position of the robot in millimeters
		angle_z -- Desired rotation of the robot around the vertical axis in degrees
	"""
	# ####
	# TODO: Implement a function that makes the robot move to a desired pose
	# as fast as possible. You can experiment with the built-in Cozmo function
	# (cozmo_go_to_pose() above) to understand its strategy and do the same.
	# ####
	pass

def run(robot: cozmo.robot.Robot):

	print("***** Front wheel radius: " + str(get_front_wheel_radius(robot)))
	print("***** Distance between wheels: " + str(get_distance_between_wheels()))

	## Example tests of the functions

	robot.drive_wheels(50, 25, duration=6.4)
	"""cozmo_drive_straight(robot, 85, 10)
	cozmo_turn_in_place(robot, 60, 30)
	cozmo_go_to_pose(robot, 100, 100, 45)

	rotate_front_wheel(robot, 90)
	my_drive_straight(robot, 62, 50)
	my_turn_in_place(robot, 90, 30)

	my_go_to_pose1(robot, 100, 100, 45)
	my_go_to_pose2(robot, 100, 100, 45)
	my_go_to_pose3(robot, 100, 100, 45)"""


if __name__ == '__main__':

	cozmo.run_program(run)



