#!/usr/bin/env python
"""
Some examples of how to interact with the ArucoSlamInterface() class
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
from nav_msgs.msg import Odometry

from svea.localizers.aruco_slam_interface import *


"""
NOTE: You must change the map file path to match your system
A relative file path should be used when using a roslaunch file
"""
map_file_path = '/home/kcoble/svea_ws/src/svea_research/src/svea/resources/maps/demo_aruco_map.txt'


"""
Some necessary initializations
"""
rospy.init_node('aruco_examples')
aruco_slam = ArucoSlamInterface(map_file_path, vehicle_frame='base_link', camera_frame='arducam').start()
vehicle_state = State() # Defaults to origin
posepub = rospy.Publisher('/map_pose', PoseStamped, queue_size=10)
offset_pub = rospy.Publisher('/offset_pose', PoseStamped, queue_size=10)
pose_array_pub = rospy.Publisher('/marker_poses', PoseArray, queue_size=10)
"""
Read the .txt fiducial_slam map file
"""
map_marker_dict = aruco_slam.read_map_file(marker_key = "map marker ")
other_marker_dict = {}
rospy.sleep(1)


def main():
    """
    Add marker 14 to dictionary with map markers AND to .txt map file
    Add marker 3 to the non-map marker dictionary
    """
    dictionary_examples()

    """
    Check if added markers are in the map
    """
    print('3 in map?', aruco_slam.check_for_marker_in_map(other_marker_dict['floating marker 3']))
    print('14 in map?', aruco_slam.check_for_marker_in_map(map_marker_dict['new map marker 14']))
    rospy.sleep(2)
    """
    Publish all markers in the
    map as a pose array
    """
    marker_pose_array = aruco_slam.markers_in_pose_array(map_marker_dict)
    pose_array_pub.publish(marker_pose_array)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        """
        Get pose of base_link from any observed markers in the map file
        """
        map_pose = aruco_slam.absolute_pose_from_map(map_marker_dict)
        print('map_pose: ', map_pose)

        if map_pose:
            posepub.publish(map_pose)
        rate.sleep()


def dictionary_examples():
    """
    Add a marker to non-map dictionary
    """
    other_marker_dict['floating marker 3'] = Marker(3)
    """
    Add a new map marker to the map dictionary and the map file
    """
    marker_14_pose = Pose()
    marker_14_pose.position.x = 1.0
    marker_14_pose.orientation = Quaternion(*quaternion_from_euler(math.pi/2, 0, -math.pi/2))

    map_marker_dict['new map marker 14'] = Marker(14, pose=marker_14_pose)
    aruco_slam.add_marker_to_map(map_marker_dict['new map marker 14'])
    """
    Modify the pose of a marker in the map file
    """
    map_marker_dict['new map marker 14'].pose.position.y += 1.7
    aruco_slam.modify_marker_pose(map_marker_dict['new map marker 14'])


class OdomSubscriber():
    """
    Uses this class to subscribe to an Odometry
    message and update the vehicle_state with
    pose values from that message
    """
    def __init__(self):
        rospy.Subscriber('/pf_pose', Odometry, self._veh_callback)

    def _veh_callback(self, msg):
        vehicle_state.x = msg.pose.pose.position.x
        vehicle_state.y = msg.pose.pose.position.y
        vehicle_state.z = msg.pose.pose.position.z
        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        _, _, vehicle_state.yaw = euler_from_quaternion(quat)


if __name__ == '__main__':

    main()