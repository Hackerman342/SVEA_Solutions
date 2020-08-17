#!/usr/bin/env python

"""
Useful funtions for interacting with ROS Transform msgs
"""

__license__ = "MIT"
__maintainer__ = "Kyle Coble"
__email__ = "coble@kth.se"
__status__ = "Development"

# import sys
# import os
# import re
# import math
# import rospy
# import tf2_ros
# import tf_conversions

import numpy as np

from geometry_msgs.msg import Quaternion, Transform
from tf.transformations import quaternion_from_euler, euler_from_quaternion, inverse_matrix
from tf.transformations import translation_matrix, translation_from_matrix
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def transform_multiply(transform_1, transform_2):
    """
    Get the multiplication of two tf transforms
    If using TransformStamped, only provide Transform component
    3 or more Transforms can be multiplied with nested function calls
        i.e. transform_multiply(transform_multiply(tf1, tf2),tf3)

    NOTE: The child frame of transform_1 must match the parent
    frame of transform_2 or the results are meaningless

    :param transform_1: Transform from frame 1 -> frame 2
    :type transform_1: geometry_msgs/Transform
    :param transform_2: Transform from frame 2 -> frame 3
    :type transform_2: geometry_msgs/Transform
    :return: Transform from frame 1 -> frame 3
    :rtype: geometry_msgs/Transform
    """
    tmat_1 = translation_matrix((transform_1.translation.x,
                                 transform_1.translation.y,
                                 transform_1.translation.z))
    tmat_2 = translation_matrix((transform_2.translation.x,
                                 transform_2.translation.y,
                                 transform_2.translation.z))
    qmat_1 = quaternion_matrix((transform_1.rotation.x,
                                transform_1.rotation.y,
                                transform_1.rotation.z,
                                transform_1.rotation.w))
    qmat_2 = quaternion_matrix((transform_2.rotation.x,
                                transform_2.rotation.y,
                                transform_2.rotation.z,
                                transform_2.rotation.w))
    tf_mat_1 = np.dot(tmat_1, qmat_1)
    tf_mat_2 = np.dot(tmat_2, qmat_2)
    tf_mat_full = np.dot(tf_mat_1, tf_mat_2)

    full_trans = Transform()
    trans = translation_from_matrix(tf_mat_full)
    full_trans.translation.x = trans[0]
    full_trans.translation.y = trans[1]
    full_trans.translation.z = trans[2]
    full_trans.rotation = Quaternion(*quaternion_from_matrix(tf_mat_full))

    return full_trans


def inverse_transform(transform):
    """
    Get the inverse of a tf transform
    Get frame 2 -> frame 1 from frame 1 -> frame 2

    :param transform: Transform from frame 1 -> frame 2
    :type transform: geometry_msgs/Transform
    :return: Transform from frame 2 -> frame 1
    :rtype: geometry_msgs/Transform
    """
    tmat = translation_matrix((transform.translation.x,
                               transform.translation.y,
                               transform.translation.z))
    qmat = quaternion_matrix((transform.rotation.x,
                              transform.rotation.y,
                              transform.rotation.z,
                              transform.rotation.w))
    tf_mat = np.dot(tmat, qmat)
    inv_tf_mat = inverse_matrix(tf_mat)

    inv_transform = Transform()
    inv_transform.translation.x = translation_from_matrix(inv_tf_mat)[0]
    inv_transform.translation.y = translation_from_matrix(inv_tf_mat)[1]
    inv_transform.translation.z = translation_from_matrix(inv_tf_mat)[2]
    inv_transform.rotation = Quaternion(*quaternion_from_matrix(inv_tf_mat))

    return inv_transform


def matrix_from_tf(transform):
    """
    Converts a geometry_msgs/Transform or
    geometry_msgs/TransformStamped into a
    4x4 transformation matrix

    :param transform: Transform from parent->child frame
    :type transform: geometry_msgs/Transform(Stamped)
    :return: Transform as 4x4 matrix
    :rtype: Numpy array (4x4)
    """
    if transform._type == 'geometry_msgs/TransformStamped':
        transform = transform.transform

    trans = (transform.translation.x,
             transform.translation.y,
             transform.translation.z)
    quat_ = (transform.rotation.x,
             transform.rotation.y,
             transform.rotation.z,
             transform.rotation.w)

    tmat = translation_matrix(trans)
    qmat = quaternion_matrix(quat_)
    return np.dot(tmat, qmat)