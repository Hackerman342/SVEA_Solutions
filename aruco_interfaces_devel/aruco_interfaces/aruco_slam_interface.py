#!/usr/bin/env python
"""
ROS interface object for interacting with the aruco_detect node and the maps
built using fiducial_slam. Inherits from ArucoInterface()

Developed for: KTH Smart Mobility Lab
"""

__license__ = "MIT"
__maintainer__ = "Kyle Coble"
__email__ = "coble@kth.se"
__status__ = "Development"

import sys
import os
import re
import math
import rospy
import numpy as np
import tf2_ros
import tf_conversions
from threading import Thread


from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, Transform
from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix

from svea.sensors.aruco_interfaces import ArucoInterface, Marker, State
from svea.utils.transform_utils import transform_multiply, inverse_transform



class ArucoSlamInterface(ArucoInterface):
    """
    Aruco interface specifically for interacting with the map file from fiducial_slam

    :param file_path: Full file path to the fiducial_slam map file
    :type file_path: string
    :param vehicle_frame: Frame_id of vehicle (must be valid in the tf tree)
                        defaults to 'base_link'
    :type vehicle_frame: str
    :param camera_frame: Frame_id of camera (must be valid in the tf tree)
                        defaults to 'camera_link'
    :type camera_frame: str
    """

    def __init__(self, file_path, vehicle_frame='base_link', camera_frame='camera_link'):

        ArucoInterface.__init__(self, vehicle_frame, camera_frame)

        self.file_path = file_path


    def read_map_file(self, marker_key="marker_"):
        """
        Get a dictionary of Marker() objects with ids and poses read from a
        map.txt file in the format from: http://wiki.ros.org/fiducial_slam
        All markers in the map file will be named in the dictionary with the
        marker_key as a prefix followed by the ficucial id (e.g. 'marker_17')

        NOTE: Marker() objects are intiated with default dimensions

        :param marker_key: Prefix for dict. entries, defaults to 'marker_'
        :type marker_key: string
        :return: Dictionary of Marker() objects
        :rtype: dictionary
        """
        markers = {}
        file = open(self.file_path, "r")
        for line in file:
            params = list(line.split(" "))
            fid = int(params[0])

            pose = Pose()
            pose.position.x = float(params[1])
            pose.position.y = float(params[2])
            pose.position.z = float(params[3])

            r = math.radians(float(params[4]))
            p = math.radians(float(params[5]))
            y = math.radians(float(params[6]))
            pose.orientation = Quaternion(*quaternion_from_euler(r, p, y))

            marker = Marker(fid, pose)
            markers[marker_key + str(fid)] = marker

        file.close()
        return markers


    def check_for_marker_in_map(self, marker):
        """
        Returns whether or not a marker is in the
        map.txt file used to initialize the class

        :param marker: Fiducial marker
        :type marker: Marker() object
        :return: If marker is in map .txt file
        :rtype: boolean
        """
        file = open(self.file_path, "r")
        for line in file:
            params = list(line.split(" "))
            fid = int(params[0])
            if fid == marker.fid:
                file.close()
                return True
        file.close()
        return False


    def add_marker_to_map(self, marker, force_modify=False):
        """
        Adds a marker object to the .txt map file. If the marker is already in
        the map file, the pose will be modified IFF 'force_modify'==True

        :param marker: Fiducial marker
        :type marker: Marker() object
        :param force_modify: If existing marker poses will be overwritten,
                            defaults to False
        :type force_modify: boolean
        """
        if self.check_for_marker_in_map(marker):
            if force_modify:
                rospy.loginfo("Editing existing marker pose in map")
                self.modify_marker_pose(marker)
                return
            else:
                rospy.loginfo("Not editing existing marker pose in map")
                rospy.loginfo("param force_modify set to 'False'")
                return

        """ Write line to add """
        params = []
        params.append(marker.fid)
        quat = (marker.pose.orientation.x,
                marker.pose.orientation.y,
                marker.pose.orientation.z,
                marker.pose.orientation.w)
        rpy_ = euler_from_quaternion(quat)

        params.append("{:.6f}".format(marker.pose.position.x))
        params.append("{:.6f}".format(marker.pose.position.y))
        params.append("{:.6f}".format(marker.pose.position.z))
        params.append("{:.6f}".format(math.degrees(rpy_[0])))
        params.append("{:.6f}".format(math.degrees(rpy_[1])))
        params.append("{:.6f}".format(math.degrees(rpy_[2])))
        params.append("{:.6f}".format(0))
        params.append('1\n')
        new_line = " ".join(map(str, params))

        """ Add line to map """
        file = open(self.file_path, "r")
        lines = file.readlines()
        for idx, line in enumerate(lines):
            params = list(line.split(" "))
            if int(params[0]) > marker.fid:
                index = idx
                break

        lines.insert(index, new_line)
        file = open(self.file_path, "w")
        file.write("".join(lines))
        file.close()


    def modify_marker_pose(self, marker):
        """
        Modifies the pose of a marker in the .txt map file
        IFF the marker already exists in the map

        :param marker: Fiducial marker
        :type marker: Marker() object
        """

        if not self.check_for_marker_in_map(marker):
            rospy.loginfo("Cannot edit pose - marker not in map")
            return

        file = open(self.file_path, "r")
        lines = file.readlines()
        for idx, line in enumerate(lines):
            params = list(line.split(" "))
            fid = int(params[0])
            if fid == marker.fid:
                quat = (marker.pose.orientation.x,
                        marker.pose.orientation.y,
                        marker.pose.orientation.z,
                        marker.pose.orientation.w)
                rpy_ = euler_from_quaternion(quat)

                params[1] = "{:.6f}".format(marker.pose.position.x)
                params[2] = "{:.6f}".format(marker.pose.position.y)
                params[3] = "{:.6f}".format(marker.pose.position.z)
                params[4] = "{:.6f}".format(math.degrees(rpy_[0]))
                params[5] = "{:.6f}".format(math.degrees(rpy_[1]))
                params[6] = "{:.6f}".format(math.degrees(rpy_[2]))

                new_line = " ".join(map(str, params))
                lines[idx] = new_line

        file = open(self.file_path, "w")
        file.write("".join(lines))
        file.close()


    def absolute_pose_from_marker(self, marker, map_frame='map'):
        """
        Get the pose of the vehicle based a detected marker's pose. Pose values
        are in the same coordinate frame as the marker

        :param marker: Fiducial marker
        :type marker: Marker() object
        :param map_frame: frame_id of the marker pose, defaults to 'map'
        :type map_frame: string
        :return: Pose of vehicle
        :rtype: geometry_msgs/PoseStamped
        :return: None (if marker is not detected)
        """
        marker_trans = self.vehicle_to_marker_transform(marker)
        if marker_trans is None:
            return None

        mark_to_veh = inverse_transform(marker_trans)

        map_to_mark = Transform()
        map_to_mark.translation.x = marker.pose.position.x
        map_to_mark.translation.y = marker.pose.position.y
        map_to_mark.translation.z = marker.pose.position.z
        map_to_mark.rotation      = marker.pose.orientation

        map_to_veh = transform_multiply(map_to_mark, mark_to_veh)

        vehicle_pose = PoseStamped()
        vehicle_pose.pose.position.x = map_to_veh.translation.x
        vehicle_pose.pose.position.y = map_to_veh.translation.y
        vehicle_pose.pose.position.z = map_to_veh.translation.z
        vehicle_pose.pose.orientation = map_to_veh.rotation

        vehicle_pose.header.frame_id = map_frame
        vehicle_pose.header.stamp = rospy.Time.now()

        return vehicle_pose


    def absolute_pose_from_map(self, marker_dict, map_frame='map'):
        """
        Get the pose of the vehicle based on all detected markers contained in
        the marker dictionary. Function will automatically exclude markers not
        in the marker dictionary. Pose values are in the same coordinate frame
        as the markers.

        :param marker_dict: Dictionary of ficucial markers
        :type marker_dict: dict
        :param map_frame: frame_id of the marker poses, defaults to 'map'
        :type map_frame: string
        :return: Pose of vehicle
        :rtype: geometry_msgs/PoseStamped
        :return: None (if a marker detection is lost during computation)
        """

        dict_fids = [] # Fiducial id's of markers in dictionary
        for key, marker in marker_dict.items():
            dict_fids.append(marker.fid)

        """
        Define weights of detections based on detection error and pixel area
        """
        raw_weights = []
        vis_fids = []
        for transform in self.fid_transforms.transforms:
            if transform.fiducial_id in dict_fids: # Only include markers in dictionary
                raw_weight = math.sqrt(transform.fiducial_area/transform.object_error)
                raw_weights.append(raw_weight)
                vis_fids.append(transform.fiducial_id)
        unsorted_weights = [float(i)/sum(raw_weights) for i in raw_weights]
        weights = [weight for _,weight in sorted(zip(vis_fids,unsorted_weights))]
        vis_fids.sort()

        x_  = []
        y_  = []
        z_  = []
        ai_ = []
        aj_ = []
        ak_ = []

        """
        Get estimated vehicle pose from each marker
        """
        vis_markers = self.get_markers_from_dict(vis_fids, marker_dict)
        for marker in vis_markers:
            pose_stamped = self.absolute_pose_from_marker(marker)
            if pose_stamped: # Super safety check
                x_.append(pose_stamped.pose.position.x)
                y_.append(pose_stamped.pose.position.y)
                z_.append(pose_stamped.pose.position.z)
                quat = (pose_stamped.pose.orientation.x,
                        pose_stamped.pose.orientation.y,
                        pose_stamped.pose.orientation.z,
                        pose_stamped.pose.orientation.w)

                roll, pitch, yaw = euler_from_quaternion(quat)
                ai_.append(roll)
                aj_.append(pitch)
                ak_.append(yaw)

        """
        Calc and return weighted average of estimated pose
        """
        if len(x_) == 0 or len(x_) != len(weights):
            return None
        else:
            x  = sum([x*weight for x,weight in zip(x_,weights)])
            y  = sum([y*weight for y,weight in zip(y_,weights)])
            z  = sum([z*weight for z,weight in zip(z_,weights)])
            ai = sum([ai*weight for ai,weight in zip(ai_,weights)])
            aj = sum([aj*weight for aj,weight in zip(aj_,weights)])
            ak = sum([ak*weight for ak,weight in zip(ak_,weights)])

            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation = Quaternion(*quaternion_from_euler(ai, aj, ak))

            pose.header.frame_id = map_frame
            pose.header.stamp = rospy.Time.now()

            return pose