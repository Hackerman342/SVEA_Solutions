#!/usr/bin/env python
"""
Some examples of how to interact with the ArucoInterface() class
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

from geometry_msgs.msg import Pose, PoseStamped
from fiducial_msgs.msg import FiducialTransformArray

from svea.sensors.aruco_interfaces import *

"""
Some necessary initializations
"""
rospy.init_node('aruco_examples')
aruco_detect = ArucoInterface(vehicle_frame='base_link', camera_frame='arducam').start()
vehicle_state = State() # Defaults to origin
posepub = rospy.Publisher('/observed_pose', PoseStamped, queue_size=10)
offset_pub = rospy.Publisher('/offset_pose', PoseStamped, queue_size=10)
marker_dict = {}
rospy.sleep(1)


def main():
    """
    Select which of the example functions
    to run or define your own actions
    """
    add_marker_to_dictionary(marker_dict, 3)
    add_marker_to_dictionary(marker_dict, 14)
    print(marker_dict)
    rospy.sleep(1)

    rate = rospy.Rate(10) # [hz]
    while not rospy.is_shutdown():
        """
        Check if a specific Marker() object is currently detected
        """
        marker_3_seen = aruco_detect.check_for_marker(marker_dict['added marker 3'])
        if marker_3_seen:
            print("Marker 3 is detected")
        """
        Publish the pose of all detected markers that are in the dictionary
        """
        publish_detected_markers()
        """
        Publish a pose offset from the aruco marker
        """
        offset = [1.5, 1.0, math.pi/4]
        publish_an_offset_pose(marker_dict['added marker 3'], offset)
        """
        Move the base_link around
        """
        move_state(vehicle_state)

        rate.sleep()


def add_marker_to_dictionary(marker_dict, fid):
    """
    Add a marker to the marker dictionary with any arbitrary pose
    """
    float_pose = Pose()
    float_pose.position.y = 1.3
    float_pose.orientation = Quaternion(*quaternion_from_euler(math.pi/2, 0, -math.pi/2))
    marker_dict['added marker '+str(fid)] = Marker(fid, pose=float_pose, dim=0.14)


def publish_detected_markers():
    """
    Get the fiducial id of all visible markers
    """
    vis_markers = aruco_detect.visible_markers()
    print(vis_markers)

    """
    Get the detected Marker() objects included in the dictionary
    NOTE: If a detected marker is not in the marker
    dictionary, it will not be returned in marker_objects
    """
    marker_objects = aruco_detect.get_markers_from_dict(vis_markers, marker_dict)
    for marker in marker_objects:
        print("Marker id", marker.fid)
        """
        Get marker position as PoseStamped msg
        """
        stamped_pose = aruco_detect.marker_pose_stamped(vehicle_state, marker)
        """
        Publish marker pose (to visualize in RVIZ)
        """
        posepub.publish(stamped_pose)
        """
        Update pose in Marker() object
        """
        marker.pose = stamped_pose.pose


def publish_an_offset_pose(marker, offset_vec):
    """
    Publish a pose offset from the aruco marker
    """
    s = vehicle_state
    m = marker
    vec = offset_vec
    off_pose = aruco_detect.relative_pose_from_marker(s, m, vec, facing_marker=True)
    if off_pose:
        offset_pub.publish(off_pose)


def move_state(state):
    """
    Drives the base_link (state) in a circle
    Marker pose is based on the transform from base_link
    to marker and the base_link (state) pose in the map frame
    """
    state.v = 0.1
    state.x += state.v*math.cos(state.yaw)
    state.y += state.v*math.sin(state.yaw)
    state.yaw += math.pi/45


if __name__ == '__main__':
    main()