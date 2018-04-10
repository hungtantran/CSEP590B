#!/usr/bin/env python3

import cv2
import sys
import copy

import numpy as np

try:
	from PIL import Image, ImageDraw, ImageFont
except ImportError:
	sys.exit('install Pillow to run this code')


def check_ball(opencv_image, circle):
	#print(len(opencv_image))
	#print(len(opencv_image[0]))
	total_inside = 0
	applied_inside = 0
	total_outside = 0
	applied_outside = 0

	# Iterate vertically
	for i in range(max(0, circle[1]-circle[2]), min(len(opencv_image)-1, circle[1] + circle[2])):
		# Iterate horizontally
		for j in range(max(0, circle[0]-circle[2]), min(len(opencv_image[0])-1, circle[0] + circle[2])):
			distance = (i - circle[1]) * (i - circle[1]) + (j - circle[0]) * (j - circle[0])
			if distance <= circle[2] * circle[2]:
				total_inside += 1
				if opencv_image[i][j] < 50:
					applied_inside += 1
			else:
				total_outside += 1
				if opencv_image[i][j] < 50:
					applied_outside += 1
	#print(applied, total * 0.8)
	if total_inside == 0 or applied_inside < total_inside * 0.6:
		return False
	if total_outside == 0 or applied_outside > total_outside * 0.4:
		return False
	return True

def find_ball(opencv_image, debug=False):
	"""Find the ball in an image.
		
		Arguments:
		opencv_image -- the image
		debug -- an optional argument which can be used to control whether
				debugging information is displayed.
		
		Returns [x, y, radius] of the ball, and [0,0,0] or None if no ball is found.
	"""

	ball = None
	opencv_image = cv2.medianBlur(opencv_image, 9)
	circles = cv2.HoughCircles(opencv_image,
							   cv2.HOUGH_GRADIENT,
							   dp=1,
							   minDist=20,
							   param1=50,
							   param2=30,
							   minRadius=0,
							   maxRadius=0)
	if circles is None or len(circles) == 0 or len(circles[0]) == 0:
		return None
	circles = np.int16(np.around(circles))
	#print(len(circles[0]))
	circles = circles[0]
	for c in circles:
		if check_ball(opencv_image, c):
			return c
	#return circles[0]
	#return circles
	return None

def display_circles(opencv_image, circles, best=None):
	"""Display a copy of the image with superimposed circles.
		
	   Provided for debugging purposes, feel free to edit as needed.
	   
	   Arguments:
		opencv_image -- the image
		circles -- list of circles, each specified as [x,y,radius]
		best -- an optional argument which may specify a single circle that will
				be drawn in a different color.  Meant to be used to help show which
				circle is ranked as best if there are multiple candidates.
		
	"""
	#make a copy of the image to draw on
	circle_image = copy.deepcopy(opencv_image)
	circle_image = cv2.cvtColor(circle_image, cv2.COLOR_GRAY2RGB, circle_image)
	
	for c in circles:
		# draw the outer circle
		cv2.circle(circle_image,(c[0],c[1]),c[2],(255,255,0),2)
		# draw the center of the circle
		cv2.circle(circle_image,(c[0],c[1]),2,(0,255,255),3) 
		# write coords
		#cv2.putText(circle_image,str(c),(c[0],c[1]),cv2.FONT_HERSHEY_SIMPLEX,
		#			.5,(255,255,255),2,cv2.LINE_AA)            
	
	#highlight the best circle in a different color
	if best is not None:
		# draw the outer circle
		cv2.circle(circle_image,(best[0],best[1]),best[2],(0,0,255),2)
		# draw the center of the circle
		cv2.circle(circle_image,(best[0],best[1]),2,(0,0,255),3) 
		# write coords
		#cv2.putText(circle_image,str(best),(best[0],best[1]),cv2.FONT_HERSHEY_SIMPLEX,
		#			.5,(255,255,255),2,cv2.LINE_AA)            
		
	
	#display the image
	pil_image = Image.fromarray(circle_image)
	pil_image.show()    
	  
if __name__ == "__main__":
	opencv_image = cv2.imread("./imgs/test65.bmp", cv2.COLOR_GRAY2RGB)
	"""for i in range(0, len(opencv_image)):
		for j in range(0, len(opencv_image[0])):
			if opencv_image[i, j] < 40:
				opencv_image[i, j] = 0
			else:
				opencv_image[i, j] = 355"""
	
	#try to find the ball in the image
	ball = find_ball(opencv_image)
	display_circles(opencv_image, [ball])
