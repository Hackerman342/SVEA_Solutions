#!/usr/bin/env python
"""
Module for particle filtering Aruco SLAM for SVEA cars.
Prediction input is Odometry message (only Twist components used)

Observation inputs comes from ArucoSlamInterface() as PoseStamped
Filtered output is PoseWithCovarianceStamped message

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
import tf
import tf2_ros
import tf_conversions

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from svea.localizers.aruco_slam_interface import *

obs_update_rate = 5 # [hz]
publish_covariance = True


class ArucoParticleFilterRemote():
    """
    Particle filter for SVEA vehicles
    Prediction comes from Twist components of an Odometry msg
    Observations come from aruco marker detections
    """

    def __init__(self):

        # Initialize ArucoSlamInterface()
        param = rospy.search_param("map_file")
        map_file_path = rospy.get_param(param)
        self.aruco_slam = ArucoSlamInterface(map_file_path,
                    vehicle_frame='base_link',camera_frame='arducam').start()
        self.marker_dict = self.aruco_slam.read_map_file()

        # Read particle count
        param = rospy.search_param("particle_count")
        self.pc = rospy.get_param(param)

        # Initialize class variables
        self.innov = np.zeros((self.pc,3))
        self.likeli = np.zeros((self.pc,1))

        # Build the process and observation covariance matrices
        param = rospy.search_param("linear_process_covariance")
        pl_cov = rospy.get_param(param)
        param = rospy.search_param("angular_process_covariance")
        pa_cov = rospy.get_param(param)
        param = rospy.search_param("linear_observation_covariance")
        ol_cov = rospy.get_param(param)
        param = rospy.search_param("angular_observation_covariance")
        oa_cov = rospy.get_param(param)

        self.pcov_matrix = self._cov_matrices_build(pl_cov, pa_cov)
        self.ocov_matrix = self._cov_matrices_build(ol_cov, oa_cov)

        # Initialize array of particle states | # particles x 4 [x, y, theta_z, weight]
        param = rospy.search_param("initial_estimate_covariance")
        init_cov = rospy.get_param(param)

        self.particles = (np.random.rand(self.pc,4)-0.5)*(2*init_cov) # x & y from init_cov
        self.particles[:,2] = (np.random.rand(self.pc,)-0.5)*(2*np.pi) # All yaw on [-pi, pi]
        self.particles[:,3] = np.ones((self.pc,)) # All weights = 1

        # Read initial pose estimate values (and convert to float list)
        param = rospy.search_param("initial_pose_estimate")
        pose_string = rospy.get_param(param)
        pose_string = pose_string.replace('[','')
        pose_string = pose_string.replace(']','')
        pose_list = list(pose_string.split(", "))
        initial_pose = np.asarray(list(map(float, pose_list))) # [x, y, yaw (rad)]

        self.particles[:,0:3] += initial_pose

        # Initialize tf braodcaster for filtered pose
        param = rospy.search_param("map_frame")
        map_frame = rospy.get_param(param)
        param = rospy.search_param("pf_pose_frame")
        pf_frame = rospy.get_param(param)

        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = map_frame
        self.t.child_frame_id = pf_frame

        # Initialize publisher for estimated pose of vehicle in map frame
        param = rospy.search_param("pf_pose_topic")
        pf_pose_top = rospy.get_param(param)
        self.posepub = rospy.Publisher(pf_pose_top, Odometry, queue_size=10)
        self.filt_pose = Odometry()
        self.filt_pose.header.frame_id = map_frame

        # Initialize publisher for pose array
        param = rospy.search_param("pose_array_topic")
        pose_arr_top = rospy.get_param(param)
        self.array_pub = rospy.Publisher(pose_arr_top, PoseArray, queue_size=10)
        self.posearray = PoseArray()
        self.posearray.header.frame_id = map_frame

        # Initialize callback variables
        self.pred_odom = None
        self.time = None
        self.old_time = None

        # Establish subscription to prediction update odometry | Intentionally last
        param = rospy.search_param("prediction_update_topic")
        pred_up_top = rospy.get_param(param)
        rospy.Subscriber(pred_up_top, Odometry, self._pred_up_callback)
        rospy.sleep(0.5) # Delay briefly to allow subscribers to find messages


    def run_pf(self):
        """
        Runs the particle filter update step and publishes/broadcasts
        particles and filtered pose.
        NOTE: Prediction step is called in the subscriber callback function
        """
        rate = rospy.Rate(obs_update_rate)
        while not rospy.is_shutdown():
            self.obs_pose = self.aruco_slam.absolute_pose_from_map(self.marker_dict)
            if self.obs_pose:
                self.obs_update()
                self.weight()
                self.mult_resample()

            self._pose_publish()
            self._tf_broadcast()
            self._posearray_publish()

            rate.sleep()


    def _pred_up_callback(self, msg):
        """
        Callback function for prediction odometry subscription.
        Tracks msg timestamps for a dt calculation in self.predict()
        """
        self.pred_odom = msg
        self.time = self.pred_odom.header.stamp.secs + self.pred_odom.header.stamp.nsecs*1.e-9
        if self.old_time:
            self.predict()
        self.old_time = self.time


    def predict(self):
        """
        Predicts the motion of the vehicle (dead reckoning) based on the twist values
        from a subscribed to Odometry msg
        """
        # Use covariance to calculate gaussian noise for prediction
        pnoise = self._gaussian_noise(self.pcov_matrix)

        # Unpack odometry message
        xvel = self.pred_odom.twist.twist.linear.x
        yvel = self.pred_odom.twist.twist.linear.y
        omega = self.pred_odom.twist.twist.angular.z

        # Calculate timestep from last prediction update
        dt = self.time - self.old_time

        # Update particle pose estimates based on angular and linear velocities from odometry
        self.particles[:,0] = self.particles[:,0] + pnoise[:,0] + xvel*dt*np.cos(self.particles[:,2])
        self.particles[:,1] = self.particles[:,1] + pnoise[:,1] + xvel*dt*np.sin(self.particles[:,2])
        self.particles[:,2] = self.particles[:,2] + pnoise[:,2] + omega*dt
        # Force angles to be on range [-pi, pi]
        self.particles[:,2] = np.remainder(self.particles[:,2]+np.pi,2*np.pi)-np.pi


    def obs_update(self):
        """
        Calculates the likelihood (weight) of all particles based on
        a PoseStamped observation update
        """
        x_obs  = self.obs_pose.pose.position.x
        y_obs  = self.obs_pose.pose.position.y
        quat = (self.obs_pose.pose.orientation.x, self.obs_pose.pose.orientation.y,
                self.obs_pose.pose.orientation.z, self.obs_pose.pose.orientation.w)
        _, _, yaw_obs = euler_from_quaternion(quat)

        self.innov[:,0] = x_obs - self.particles[:,0]
        self.innov[:,1] = y_obs - self.particles[:,1]
        self.innov[:,2] = yaw_obs - self.particles[:,2]
        # Force angles to be on range [-pi, pi]
        self.innov[:,2] = np.remainder(self.innov[:,2]+np.pi,2*np.pi)-np.pi

        # Calculate likelihood
        self.likeli = np.exp(-0.5*np.sum(np.square(self.innov).dot(np.linalg.inv(self.ocov_matrix)), axis=1))
        #*(1/(2*np.pi*np.sqrt(np.linalg.det(self.ocov_matrix)))) # Constant not needed
        self.likeli += 1.e-300 # avoid round-off to zero

        if(sum(self.likeli)<=0):
            rospy.loginfo("Likelihood went to 0 | Filter failed")


    def weight(self):
        """
        Reassign weights to particles
        """
        self.particles[:,3] = self.likeli


    def mult_resample(self):
        """
        Resamples particles using multinomial resampling
        """
        cdf = np.cumsum(self.particles[:,3])
        cdf /= cdf[cdf.size-1]

        temp = self.particles[:,[0,1,2]]
        self.particles = np.zeros((self.pc,4))
        r = np.random.rand(self.pc,1)

        for i in range(cdf.size):
            ind = np.argmax(cdf >= r[i])
            self.particles[i,[0,1,2]] = temp[ind,:]

        self.particles[:,3] = np.ones((self.pc,)) # All weights = 1


    """
    Support Functions
    """

    def _pose_publish(self):
        """
        Publish average of all particles as PoseWithCovarianceStamped
        """
        self.filt_pose.pose.pose.position.x = np.average(self.particles[:,0])
        self.filt_pose.pose.pose.position.y = np.average(self.particles[:,1])
        quat = quaternion_from_euler(0,0,np.average(self.particles[:,2]))
        self.filt_pose.pose.pose.orientation = Quaternion(*quat)

        if publish_covariance:
            self.filt_pose.pose.covariance = self._cov_calc()

        self.filt_pose.header.stamp = rospy.Time.now()
        self.posepub.publish(self.filt_pose)


    def _tf_broadcast(self):
        """
        Broadcast average of all particles as tf frame
        """
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation = self.filt_pose.pose.pose.position
        self.t.transform.rotation = self.filt_pose.pose.pose.orientation
        self.br.sendTransform(self.t)


    def _posearray_publish(self):
        """
        Publish all particles as pose array
        """
        self.posearray.poses = []

        for i in range(self.pc):
            pt = Pose()
            pt.position.x = self.particles[i,0]
            pt.position.y = self.particles[i,1]
            pt.position.z = 0.0
            pt.orientation = Quaternion(*quaternion_from_euler(0, 0, self.particles[i,2]))
            self.posearray.poses.append(pt)

        self.posearray.header.stamp = rospy.Time.now()
        self.array_pub.publish(self.posearray)


    def _cov_calc(self):
        """
        Calculates covariance of particle poses as float64[36]
        list for PoseWithCovarianceStamped
        :return: Covariance of pose estimate
        :rtype: list, float64[36]
        """
        covx = np.cov(self.particles[:,0])
        covy = np.cov(self.particles[:,1])
        covyaw = np.cov(self.particles[:,2])
        cov_list = [covx,   0.0,     0.0,   0.0,   0.0,    0.0,
                    0.0,    covy,    0.0,   0.0,   0.0,    0.0,
                    0.0,    0.0,    99999,  0.0,   0.0,    0.0,
                    0.0,    0.0,     0.0,  99999,  0.0,    0.0,
                    0.0,    0.0,     0.0,   0.0,  99999,   0.0,
                    0.0,    0.0,     0.0,   0.0,   0.0,  covyaw]
        return cov_list


    def _cov_matrices_build(self, lin_cov, ang_cov):
        """
        Builds 3x3 covariance matrices for 2D motion (x, y, & yaw)

        :param lin_cov: linear covariance
        :type lin_cov: float
        :param ang_cov: angular covariance
        :type ang_cov: float
        :return: 3x3 Covariance matrix
        :rtype: numpy array [3x3, float]
        """
        cov_matrix_ = np.array([[lin_cov, 0.0, 0.0],
                                [0.0, lin_cov, 0.0],
                                [0.0, 0.0, ang_cov]])
        return cov_matrix_


    def _gaussian_noise(self, cov_mat):
        """
        Calculate array of gaussian noise from a diagonal covariance matrix
        :param cov_mat: 3x3 Covariance matrix
        :type cov_mat: numpy array [3x3, float]
        :return: Array of noise values
        :rtype: numpy array
        """
        var = np.diagonal(cov_mat)
        noise = np.sqrt(var)*np.random.randn(self.pc, 3)
        return noise


if __name__ == '__main__':

    rospy.init_node('particle_filter_aruco_remote', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    pf = ArucoParticleFilterRemote()
    rospy.loginfo("ArucoParticleFilterRemote class successfully created")

    pf.run_pf()