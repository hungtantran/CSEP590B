#!/usr/bin/env python

##############
#### Your name: Hung Tran
##############

from imgclassification import *

import threading
from queue import PriorityQueue
import math
import cozmo
import time
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps

import cv2
import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color


def react(robot: cozmo.robot.Robot):
    # load images and train model
    print("Start training")
    img_clf = ImageClassifier()
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    train_data = img_clf.extract_image_features(train_raw)
    img_clf.train_classifier(train_data, train_labels)
    print("Finish training")

    previous_detect_image = 'none'
    num_detect = 0
    while True:
        robot.set_head_angle(degrees(0)).wait_for_completed()
        robot.set_lift_height(0).wait_for_completed()
        time.sleep(.1)

        event = robot.world.wait_for(
            cozmo.camera.EvtNewRawCameraImage, timeout=30)
        # convert camera image to opencv format
        image = np.asarray([np.asarray(event.image)])
        image_features = img_clf.extract_image_features(image)
        detected_images = img_clf.predict_labels(image_features)
        if len(detected_images) != 1:
            time.sleep(0.1)
            continue
        
        detected_image = detected_images[0]
        if detected_image == previous_detect_image:
            num_detect += 1
            time.sleep(0.1)
            if num_detect == 10 and detected_image != 'none':
                print(detected_image)
                robot.say_text(detected_image).wait_for_completed()
                if detected_image == 'drone':
                    robot.play_anim_trigger(
                        cozmo.anim.Triggers.CubePounceLoseSession,
                        ignore_body_track=True).wait_for_completed()
                elif detected_image == 'inspection':
                    robot.play_anim_trigger(
                        cozmo.anim.Triggers.CubePounceLoseHand,
                        ignore_body_track=True).wait_for_completed()
                elif detected_image == 'truck':
                    robot.play_anim_trigger(
                        cozmo.anim.Triggers.CubePounceWinHand,
                        ignore_body_track=True).wait_for_completed()
            continue

        print(detected_image)
        previous_detect_image = detected_image
        num_detect = 0


if __name__ == "__main__":
    cozmo.run_program(
        react, use_viewer=True, force_viewer_on_top=True)
