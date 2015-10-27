#!/usr/bin/env python

""" This is a script that walks through some of the basics of working with images
    with opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class BallTracker(object):
    """ The BallTracker is a Python object that encompasses a ROS node 
        that can process images from the camera and search for a ball within.
        The node will issue motor commands to move forward while keeping
        the ball in the center of the camera's field of view. """

    def __init__(self, image_topic):
        """ Initialize the ball tracker """
        rospy.init_node('ball_tracker')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV

        rospy.Subscriber(image_topic, Image, self.process_image)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        cv2.namedWindow('video_window')

        """ Manual RGB thresholding """
        # cv2.namedWindow('threshold_image')
        # self.red_lower_bound = 0
        # self.red_upper_bound = 0
        # self.green_lower_bound = 0
        # self.green_upper_bound = 0
        # self.blue_lower_bound = 0
        # self.blue_upper_bound = 0
        # cv2.createTrackbar('red: lower', 'threshold_image', 0, 255, self.set_red_lower_bound)
        # cv2.createTrackbar('red: upper', 'threshold_image', 0, 255, self.set_red_upper_bound)
        # cv2.createTrackbar('green: lower', 'threshold_image', 0, 255, self.set_green_lower_bound)
        # cv2.createTrackbar('green: upper', 'threshold_image', 0, 255, self.set_green_upper_bound)
        # cv2.createTrackbar('blue: lower', 'threshold_image', 0, 255, self.set_blue_lower_bound)
        # cv2.createTrackbar('blue: upper', 'threshold_image', 0, 255, self.set_blue_upper_bound)

        """ Manual HSV thresholding """
        # cv2.namedWindow('threshold_image')
        self.hue_lower_bound = 27
        self.hue_upper_bound = 71
        self.sat_lower_bound = 218
        self.sat_upper_bound = 255
        self.val_lower_bound = 121
        self.val_upper_bound = 238
        # cv2.createTrackbar('hue: lower', 'threshold_image', 0, 179, self.set_hue_lower_bound)
        # cv2.createTrackbar('hue: upper', 'threshold_image', 0, 179, self.set_hue_upper_bound)
        # cv2.createTrackbar('sat: lower', 'threshold_image', 0, 255, self.set_sat_lower_bound)
        # cv2.createTrackbar('sat: upper', 'threshold_image', 0, 255, self.set_sat_upper_bound)
        # cv2.createTrackbar('val: lower', 'threshold_image', 0, 255, self.set_val_lower_bound)
        # cv2.createTrackbar('val: upper', 'threshold_image', 0, 255, self.set_val_upper_bound)

        self.center_x = 0.0
        self.center_y = 0.0

    """ Callback functions for manual RGB filtering """
    def set_red_lower_bound(self, val):
        self.red_lower_bound = val

    def set_red_upper_bound(self, val):
        self.red_upper_bound = val

    def set_green_lower_bound(self, val):
        self.green_lower_bound = val

    def set_green_upper_bound(self, val):
        self.green_upper_bound = val

    def set_blue_lower_bound(self, val):
        self.blue_lower_bound = val

    def set_blue_upper_bound(self, val):
        self.blue_upper_bound = val

    """ Callback functions for manual HSV filtering """
    def set_hue_lower_bound(self, val):
        self.hue_lower_bound = val

    def set_hue_upper_bound(self, val):
        self.hue_upper_bound = val

    def set_sat_lower_bound(self, val):
        self.sat_lower_bound = val

    def set_sat_upper_bound(self, val):
        self.sat_upper_bound = val

    def set_val_lower_bound(self, val):
        self.val_lower_bound = val

    def set_val_upper_bound(self, val):
        self.val_upper_bound = val

    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # self.cv_image = cv2.inRange(self.cv_image, (self.blue_lower_bound, self.green_lower_bound, self.red_lower_bound),
        #                                          (self.blue_upper_bound, self.green_upper_bound, self.red_upper_bound))
        self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.cv_image = cv2.inRange(self.cv_image, (self.hue_lower_bound, self.sat_lower_bound, self.val_lower_bound),
                                                   (self.hue_upper_bound, self.sat_upper_bound, self.val_upper_bound))

        moments = cv2.moments(self.cv_image)
        cam_size = self.cv_image.shape
        if moments['m00'] != 0:
            self.center_x, self.center_y = moments['m10']/moments['m00']/cam_size[1], moments['m01']/moments['m00']/cam_size[0]
        self.center_x -= 0.5
        self.center_y -= 0.5

        cv2.imshow('video_window', self.cv_image)
        cv2.waitKey(5)

    def run(self):
        """ The main run loop, in this node it doesn't do anything """
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            # start out not issuing any motor commands
            velocity = Twist()
            if abs(self.center_x) > 0.475:
                pass
            elif abs(self.center_x) > 0.25:
                velocity.angular.z = -self.center_x * 3.0
            else:
                velocity.linear.x = 0.2

            if velocity.angular.z > 0.5:
                velocity.angular.z = 0.5
            if velocity.angular.z < -0.5:
                velocity.angular.z = -0.5

            self.pub.publish(velocity)

            print (velocity.linear.x, velocity.angular.z)

            r.sleep()

if __name__ == '__main__':
    node = BallTracker("/camera/image_raw")
    node.run()