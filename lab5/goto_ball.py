#!/usr/bin/env python3

import asyncio
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, '../lab4')
import find_ball

import cozmo
from cozmo.util import degrees, distance_mm, speed_mmps

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


# Define a decorator as a subclass of Annotator; displays battery voltage
class BatteryAnnotator(cozmo.annotate.Annotator):
    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)
        batt = self.world.robot.battery_voltage
        text = cozmo.annotate.ImageText('BATT %.1fv' % batt, color='green')
        text.render(d, bounds)

# Define a decorator as a subclass of Annotator; displays the ball
class BallAnnotator(cozmo.annotate.Annotator):

    ball = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BallAnnotator.ball is not None:
            #double size of bounding box to match size of rendered image
            BallAnnotator.ball = np.multiply(BallAnnotator.ball,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BallAnnotator.ball[0]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[1]-BallAnnotator.ball[2],
                                      BallAnnotator.ball[2]*2, BallAnnotator.ball[2]*2)
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BallAnnotator.ball = None


def run(robot: cozmo.robot.Robot):
    '''The run method runs once the Cozmo SDK is connected.'''

    #add annotators for battery level and ball bounding box
    robot.world.image_annotator.add_annotator('battery', BatteryAnnotator)
    robot.world.image_annotator.add_annotator('ball', BallAnnotator)
    try:
        robot.set_head_angle(degrees(0)).wait_for_completed()

        while True:
            #get camera image
            event = robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

            #convert camera image to opencv format
            opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)

            #find the ball
            ball = None
            for i in range(10):
                ball = find_ball.find_ball(opencv_image)
                if ball is not None:
                    break

            if ball is None:
                robot.turn_in_place(degrees(10)).wait_for_completed()
                continue

            #set annotator ball
            BallAnnotator.ball = ball

            # Turn toward the ball

            # If the ball still far away, going toward it
            while ball[2] < 130:
                robot.drive_straight(distance_mm(10), speed_mmps(50)).wait_for_completed()
                ball = None
                turn = 1
                # Adjust the angel to turn toward the ball
                while ball is None:
                    event = robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)
                    opencv_image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_RGB2GRAY)
                    ball = find_ball.find_ball(opencv_image)
                    # Do some angle adjustment in case the ball is a bit off to the side
                    robot.turn_in_place(degrees(turn)).wait_for_completed()
                    turn = turn * (-2)
                    time.sleep(0.1)
                horizontal_turn = int(float(-ball[0] + len(opencv_image[0])/2) / len(opencv_image[0]) * 25)
                robot.turn_in_place(degrees(horizontal_turn)).wait_for_completed()
            # Once the ball is big enough, drive a little bit toward it and hit it
            robot.drive_straight(distance_mm(10), speed_mmps(50)).wait_for_completed()
            robot.move_lift(5)
            time.sleep(2)
            robot.move_lift(-5)
    except KeyboardInterrupt:
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print('here')
        print(e)



if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)

