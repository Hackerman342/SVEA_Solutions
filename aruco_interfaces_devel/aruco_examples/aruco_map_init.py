#!/usr/bin/env python
"""
Example of how to initialize a new map of aruco markers

Developed for: KTH Smart Mobility Lab
"""

__license__ = "MIT"
__maintainer__ = "Kyle Coble"
__email__ = "coble@kth.se"
__status__ = "Development"

import sys
import os
import time
import math
import rospy

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from fiducial_msgs.msg import FiducialTransformArray

from svea.sensors.aruco_interfaces import *

cam_frame = 'arducam'
base_frame = 'base_link'
convergence_count = 10 # int
map_file_path = '/home/kcoble/svea_ws/src/svea_research/src/svea/resources/maps/new_aruco_map.txt'
subscribed_pose = False # Boolean
init_fid = 3 # Fid of marker to build map from


class ArucoMapInit():

    def __init__(self):

        self.aruco_detect = ArucoInterface(vehicle_frame=base_frame, camera_frame=cam_frame).start()

        self.state = State(x=0.0, y=0.0, yaw=math.radians(0), v=0.0)
        self.init_marker = Marker(init_fid)
        self.fid_pose_arr = None
        if subscribed_pose:
            """
            Add subscriber and callback to automatically
            update variables in self.state
            """
            pass


    def marker_map_array(self):
        marker_pose = self.aruco_detect.marker_pose(self.state, self.init_marker)
        if marker_pose:
            x = marker_pose.position.x
            y = marker_pose.position.y
            z = marker_pose.position.z
            quat = (marker_pose.orientation.x,
                    marker_pose.orientation.y,
                    marker_pose.orientation.z,
                    marker_pose.orientation.w)
            r, p, yaw = euler_from_quaternion(quat)
            fid_pose_arr = np.array([self.init_marker.fid, x, y, z, r*180/np.pi, p*180/np.pi, yaw*180/np.pi, 0, 1])
            fid_pose_arr = fid_pose_arr.astype(float)
            return fid_pose_arr


    def fid_pose_converge(self):

        count = 0
        r = rospy.Rate(2) # [hz] Slow rate of measurements - max is 30hz (camera publish rate)

        while (count < convergence_count):
            fid_pose_arr = self.marker_map_array()
            print(fid_pose_arr)
            if fid_pose_arr is not None:
                if (count == 0):
                    fid_pose_conv = np.copy(fid_pose_arr)
                    print(fid_pose_conv)
                else:
                    fid_pose_conv = (fid_pose_conv*count + fid_pose_arr)/(count + 1)
                    print("count: ", count)
                    print(fid_pose_conv)
                count += 1
                print(count)
                fid_pose_arr = None
            r.sleep()
        self.fid_pose_conv = fid_pose_conv


    def aruco_map_place(self):
        """
        structure for line to write:
        id x y z pan tilt roll variance numObservations links
        Opens file and creates file if it is not there
        This will overwrite whatever first line is there
        """
        file = open(map_file_path, "w+")
        file.write("%d %f %f %f %f %f %f %d %d\n"%(self.fid_pose_conv[0],
                            self.fid_pose_conv[1], self.fid_pose_conv[2],
                            self.fid_pose_conv[3], self.fid_pose_conv[4],
                            self.fid_pose_conv[5], self.fid_pose_conv[6],
                            self.fid_pose_conv[7], self.fid_pose_conv[8]))
        file.close()


if __name__ == "__main__":

    rospy.init_node('aruco_map_init', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    new_map = ArucoMapInit()
    rospy.loginfo("Successfully initialized ArucoMapInit")

    new_map.fid_pose_converge()
    new_map.aruco_map_place()
    rospy.sleep(1)

    print("Map initialization complete")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
