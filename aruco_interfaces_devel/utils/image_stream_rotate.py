#!/usr/bin/env python
"""
Rotates an image stream
Useful for RVIZ visualization of a camera stream
from a camera mounted on its side or upside-down

Developed for: KTH Smart Mobility Lab
"""
__license__ = "MIT"
__maintainer__ = "Kyle Coble"
__email__ = "coble@kth.se"
__status__ = "Development"

# Standard dependencies
import sys
import os
import math
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

"""
Select either rotate 90 deg or rotate 180 deg
If both are True, rotate_180 overwrites rotate_90

Direction doesn't matter for 180 deg rotation
"""
rotate_90 = True
rotate_180 = False
clockwise = True # Direction
reduce_size = True

class ImageRotate():

    def __init__(self):

        sub_img_topic = "/fiducial_images"
        pub_img_topic = "/repub/fiducial_images"

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize publisher
        self.rot_pub = rospy.Publisher(pub_img_topic, Image, queue_size=10)

        # Initialize subscriber to image stream
        rospy.Subscriber(sub_img_topic, Image, self.img_callback)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages


    def img_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if reduce_size:
                image = cv2.resize(image, (0,0), fx = 0.5, fy = 0.5)
            if rotate_90:
                if clockwise:
                    image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
                else:
                    image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if rotate_180:
                image = cv2.rotate(image, cv2.ROTATE_180)

            self.rot_pub.publish(self.bridge.cv2_to_imgmsg(image, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('image_stream_rotate', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    IR = ImageRotate()

    print("running...")
    # Run until KeyboardInterrupt
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()