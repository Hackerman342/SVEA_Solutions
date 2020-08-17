#!/usr/bin/env python
"""
ROS interface object for interacting with the aruco_detect node
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

from svea.utils.transform_utils import transform_multiply, inverse_transform


class State(object):
    """
    Class representing the state of a vehicle. State is NOT unit-less.
    Should be [m], [m/s], [rad] to interact properly with ArucoInterfaces()

    NOTE: This class should be deleted and instead an actively maintained (and
    compatible) State() class should be imported

    :param x: x position, defaults to 0.0
    :type x: float [m]
    :param y: y position, defaults to 0.0
    :type y: float [m]
    :param yaw: yaw, defaults to 0.0
    :type yaw: float [radians]
    :param v: velocity, defaults to 0.0
    :type v: float [m/s]
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


class Marker(object):
    """
    Class representing key params/info for an aruco marker

    NOTE: A marker name (string) is not stored in the class, but Marker()
    objects will often be referenced by calling the marker's name from a
    dictionary of Marker() objects

    NOTE: None of the ArucoInterface or ArucoSlamInterface functions account for
    the different marker dim yet. This is pretty critical functionality that
    should be added.

    :param fid: fiducial id of the aruco marker
    :type fid: int
    :param pose: 6 DOF pose of the marker in the map frame, defaults to Origin
    :type pose: geometry_msgs/Pose
    :param dim: dimension of the marker (incl. black boundary), defaults to 0.14
    :type dim: float [m]
    """

    def __init__(self, fid, pose=Pose(), dim=0.14):
        self.fid = fid
        self.dim = dim
        self.pose = pose
        if pose == Pose(): # Corrects quat when init with empty Pose()
            self.pose.orientation.w = 1.0


class ArucoInterface(object):
    """
    Standard aruco interface. Makes it easy to interact with the results
    published from aruco_detect node. aruco_detect publishes the relative
    transform between the camera and detected fiducial (aruco) markers as
    fiducial_msgs/FiducialTransform msgs

    NOTE: If camera stream is cut off or aruco_detect dies, all functions will
        be based on markers seen in the last received message from aruco_detect

    :param vehicle_frame: Frame_id of vehicle (must be valid in the tf tree)
                        defaults to 'base_link'
    :type vehicle_frame: str
    :param camera_frame: Frame_id of camera (must be valid in the tf tree)
                        defaults to 'camera_link'
    :type camera_frame: str
    """

    def __init__(self, vehicle_frame='base_link', camera_frame='camera_link'):

        self.vehicle_frame = vehicle_frame
        self.camera_frame = camera_frame

        # Initialize callback variables
        self.fid_transforms = FiducialTransformArray()


    def start(self):
        """
        Spins up ROS background thread; must be called to start
        receiving and sending data

        :return: itself
        :rtype: ArucoInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting Aruco Interface Node: \n"
                      + str(self))

        self._collect_srvs()
        self._start_tf_buffer()
        self._start_listen()

    def _collect_srvs(self):
        # rospy.wait_for_service('service_name')
        # self.service = rospy.ServiceProxy('service_name', ServiceType)
        pass

    def _start_tf_buffer(self):
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        # Lock vehicle -> camera tf transform
        self._lock_veh_to_cam_tf()

    def _start_listen(self):
        rospy.Subscriber("/fiducial_transforms",
                        FiducialTransformArray,
                        self._read_fid_transforms)
        rospy.loginfo("Aruco Interface successfully initialized")
        rospy.spin()

    def _read_fid_transforms(self, msg):
        self.fid_transforms = msg

    def _lock_veh_to_cam_tf(self):
        """
        Lookup and lock transform from vehicle to camera
        Defines a Transform message named 'self.cam_tf'
        """
        try:
            rospy.loginfo("Waiting for transform from vehicle to camera")
            tf_stamped = self.tfBuffer.lookup_transform(self.vehicle_frame,
                                                        self.camera_frame,
                                                        rospy.Time.now(),
                                                        rospy.Duration(10))
            self.cam_tf = tf_stamped.transform
            rospy.loginfo("Transform locked from vehicle to camera")
        except tf2_ros.LookupException:
            rospy.logerr("Could not lookup transform from vehicle to camera (aruco_interfaces.py)")


    def visible_markers(self):
        """
        Get a sorted list of the id's of all detected markers

        :return: Sorted list of marker id's in camera view
        :rtype: list
        """
        list = []
        for transform in self.fid_transforms.transforms:
            list.append(transform.fiducial_id)
        list.sort()
        return list


    def check_for_marker(self, marker):
        """
        Check if a fiducial marker is detected

        :param marker: Fiducial marker
        :type marker: Marker() object
        :return: If marker is in camera's view
        :rtype: boolean
        """
        vis = self.visible_markers()
        return marker.fid in vis


    def get_markers_from_dict(self, fids, marker_dict):
        """
        Get the list of Marker() objects in the marker dictionary
        corresponding to a list of fiducial id's

        NOTE: If a marker fid that is not in the dictionary is passed in, then
        len(markers) != len(fids)

        :param fids: List of marker fiducial id's
        :type fids: list
        :param marker_dict: Dictionary of ficucial markers
        :type marker_dict: dict
        :return: List of Marker() objects
        :rtype: list
        """
        markers = []
        for fid in fids:
            for key, marker in marker_dict.items():
                if marker.fid == fid:
                    markers.append(marker)
                    break
        return markers


    def markers_in_pose_array(self, marker_dict, map_frame='map'):
        """
        Get all markers in the dictionary as a pose array

        :param marker_dict: Dictionary of ficucial markers
        :type marker_dict: dict
        :param map_frame: frame_id of the marker poses, defaults to 'map'
        :type map_frame: string
        :return: PoseArray of all markers in the dictionary
        :rtype: geometry_msgs/PoseArray
        """

        pose_array = PoseArray()
        pose_array.header.frame_id = map_frame

        for marker in marker_dict:
            pose_array.poses.append(marker_dict[marker].pose)

        pose_array.header.stamp = rospy.Time.now()

        return pose_array


    def camera_to_marker_transform(self, marker):
        """
        Get the Transform from the camera to a fiducial marker

        :param marker: Fiducial marker
        :type marker: Marker() object
        :return: Transform from camera to marker
        :rtype: geometry_msgs/Transform
        :return: None (if marker is not detected)
        """
        for transform in self.fid_transforms.transforms:
            if transform.fiducial_id == marker.fid:
                return transform.transform
        return None # If marker is not detected


    def vehicle_to_marker_transform(self, marker):
        """
        Get the Transform from the vehicle to a fiducial marker

        :param marker: Fiducial marker
        :type marker: Marker() object
        :return: Transform from vehicle to marker
        :rtype: geometry_msgs/Transform
        :return: None (if marker is not detected)
        """
        marker_trans = self.camera_to_marker_transform(marker)
        if marker_trans is None:
            return None

        return transform_multiply(self.cam_tf, marker_trans)


    def marker_pose(self, state, marker):
        """
        Get the pose of a marker based on the provided vehicle pose. Pose values
        are in the same coordinate frame as the given state

        NOTE: state.yaw must be in radians!

        :param state: Vehicle state (pose & speed)
        :type state: State() object
        :param marker: Fiducial marker
        :type marker: Marker() object
        :return: Pose of marker
        :rtype: geometry_msgs/Pose
        :return: None (if marker is not detected)
        """
        marker_trans = self.vehicle_to_marker_transform(marker)
        if marker_trans is None:
            return None

        veh_trans = Transform()
        veh_trans.translation.x = state.x
        veh_trans.translation.y = state.y
        veh_trans.translation.z = 0
        veh_trans.rotation = Quaternion(*quaternion_from_euler(0, 0, state.yaw))

        pose_as_trans = transform_multiply(veh_trans, marker_trans)

        marker_pose = Pose()
        marker_pose.position.x = pose_as_trans.translation.x
        marker_pose.position.y = pose_as_trans.translation.y
        marker_pose.position.z = pose_as_trans.translation.z
        marker_pose.orientation = pose_as_trans.rotation

        return marker_pose


    def marker_pose_stamped(self, state, marker, map_frame='map'):
        """
        Get the pose of a marker based on the provided vehicle pose. Pose values
        are in the same coordinate frame as the given state, but map_frame param
        is still necessary for PoseStamped msg

        :param state: Vehicle state (pose & speed)
        :type state: State() object
        :param marker: Fiducial marker
        :type marker: Marker() object
        :param map_frame: frame_id of the marker pose, defaults to 'map'
        :type map_frame: string
        :return: Pose of marker
        :rtype: geometry_msgs/PoseStamped
        :return: None (if marker is not detected)
        """
        pose = self.marker_pose(state, marker)
        if pose is None:
            return None

        msg = PoseStamped()
        msg.pose = pose
        msg.header.frame_id = map_frame
        msg.header.stamp = rospy.Time.now()
        return msg


    def relative_pose_from_marker(self, state, marker, offset_vector, facing_marker=True, map_frame='map'):
        """
        Get a pose relative to a marker based on the provided vehicle pose. The
        offset_vector determines how far from the marker the goal pose should
        be. The offset_vector is [z, x, yaw] in [m], [m], [rad] in the marker's
        coordinate system. z is out of the marker, x is to the marker's left
        hand side, and yaw is about the markers y-axis (ccw looking down). Pose
        values are in the same coordinate frame as the given state, but
        map_frame param is still necessary for PoseStamped msg

        :param state: Vehicle state (pose & speed)
        :type state: State() object
        :param marker: Fiducial marker
        :type marker: Marker() object
        :param offset_vector: Desired offset in marker coord system
                              [z,x,yaw] in [m,m,rad]
        :type offset_vector: list
        :param facing_marker: True -> looking at marker
                             False -> looking away from marker
        :type facing_marker: Boolean
        :param map_frame: frame_id of the marker pose, defaults to 'map'
        :type map_frame: string
        :return: Pose of marker
        :rtype: geometry_msgs/PoseStamped
        :return: None (if marker is not detected)
        """
        marker_pose = self.marker_pose_stamped(state, marker, map_frame=map_frame)
        if marker_pose is None:
            return None

        offset_pose = PoseStamped()

        quat = (marker_pose.pose.orientation.x,
                marker_pose.pose.orientation.y,
                marker_pose.pose.orientation.z,
                marker_pose.pose.orientation.w)
        _, _, marker_yaw = euler_from_quaternion(quat)

        yaw = marker_yaw + math.pi/2

        offset_pose.pose.position.x = (marker_pose.pose.position.x
                                    - offset_vector[0]*math.cos(yaw)
                                    + offset_vector[1]*math.sin(yaw))
        offset_pose.pose.position.y = (marker_pose.pose.position.y
                                    - offset_vector[0]*math.sin(yaw)
                                    - offset_vector[1]*math.cos(yaw))

        if not facing_marker:
            yaw += math.pi # Flip around

        yaw += offset_vector[2] # Angular offset
        offset_pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))

        offset_pose.header.frame_id = map_frame
        offset_pose.header.stamp = rospy.Time.now()
        return offset_pose
