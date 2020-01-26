#!/usr/bin/env python 
"""
Module for particle filtering Aruco SLAM for SVEA cars.
Prediction input is Odometry message
    - * Only twist components are used for prediction in this configuration
Observation input is PoseWithCovarianceStamped message
Filtered output is PoseWithCovarianceStamped message

Developed for: KTH Smart Mobility Lab
Developed by: Kyle Coble
""" 
import sys
import os
import math
import rospy
import numpy as np
import tf
import tf2_ros
import tf_conversions

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

#import geometry_msgs.msg
#import fiducial_msgs.msg
#from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
#from geometry_msgs.msg import Pose, PoseWithCovariance, TransformStamped, PoseStamped 


class ArucoParticleFilterRemote():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:

        # Read pose observation topic (pose estimated from aruco detection) 
        param = rospy.search_param("pose_observation_topic")
        self.pose_obs_top = rospy.get_param(param)
        # Read prediction update topic (EKF filtered odometry from IMU and ctrl inputs) 
        param = rospy.search_param("prediction_update_topic")
        self.pred_up_top = rospy.get_param(param)
        # Read particle filter estimate topic (output of this node)) 
        param = rospy.search_param("particle_filtered_pose_topic")
        self.pf_pose_top = rospy.get_param(param)
        # Read particle count 
        param = rospy.search_param("particle_count")
        self.pc = rospy.get_param(param)
        
        # Read covariance values 
        param = rospy.search_param("initial_estimate_covariance")
        self.init_cov = rospy.get_param(param)
        param = rospy.search_param("linear_process_covariance")
        self.pl_cov = rospy.get_param(param)
        param = rospy.search_param("angular_process_covariance")
        self.pa_cov = rospy.get_param(param)
        param = rospy.search_param("linear_observation_covariance")
        self.ol_cov = rospy.get_param(param)
        param = rospy.search_param("angular_observation_covariance")
        self.oa_cov = rospy.get_param(param)


        # Initialize callback variables
        self.obs_pose = None
        self.pred_odom = None

        # Initialize class variables
        self.time = None
        self.old_time = None
        self.old_theta = 0
        self.ang_z_obs = 0
        self.obs_pose_old = None
        self.innov = np.zeros((self.pc,3))
        self.likeli = np.zeros((self.pc,1))

        # Establish subscription to observation pose
        rospy.Subscriber(self.pose_obs_top, PoseWithCovarianceStamped, self.obs_pose_callback)
        # Establish subscription to prediction update odometry
        rospy.Subscriber(self.pred_up_top, Odometry, self.pred_up_callback)
        # Delay briefly to allow subscribers to find messages
        rospy.sleep(0.5)

        # Build the process and observation covariance matrices
        self.cov_matrices_build()

        # Initialize array of particle states | # particles x 4 [x, y, theta_z, weight]
        self.particles = (np.random.rand(self.pc,4)-0.5)*(2*self.init_cov)
        # Initialize angles on range [-pi, pi]
        #self.particles[:,2] = (np.random.rand(self.pc,)-0.5)*(2*np.pi)
        # Initialize all angles to 0 | works better for non-global localization
        self.particles[:,2] = np.zeros((self.pc,))
        # Set all particle weights equal
        self.particles[:,3] = np.ones((self.pc,))

        # Initialize publisher for estimated pose of vehicle in map frame
        self.posepub = rospy.Publisher(self.pf_pose_top, PoseWithCovarianceStamped, queue_size=10)
        self.filt_pose = PoseWithCovarianceStamped()


    ##### Primary particle filter functions #####

    # Function to call all functions and run particle filter
    def run_pf(self):
        while not rospy.is_shutdown():            
            # Only predict when a filtered odometry (IMU and ctrl) comes in
            if self.pred_odom != None and np.absolute(self.pred_odom.twist.twist.linear.x) > 0.01:
                # Track prediction message timestamp
                self.time = self.pred_odom.header.stamp.secs + self.pred_odom.header.stamp.nsecs*10**-9
                if self.old_time and self.time > self.old_time:
                    self.predict()
                # Update previous timestamp for comparison to next
                self.old_time = self.time

            # Only update observation when an aruco-based pose measurement comes in
            if self.obs_pose != None and self.obs_pose != self.obs_pose_old:
                self.obs_update()
                self.weight()
                self.mult_resample() # Option to use multinomial resampling
                #self.sys_resample() # Option to use systematic resampling
                self.obs_pose_old = self.obs_pose
            
            self.particle_publish()
    
    # Function for process/prediction step 
    def predict(self):
        # Use covariance to calculate gaussian noise for prediction
        pnoise = self.gaussian_noise(self.pcov_matrix)
        
        # Unpack odometry message
        xvel = self.pred_odom.twist.twist.linear.x
        yvel = self.pred_odom.twist.twist.linear.y
        omega = self.pred_odom.twist.twist.angular.z
        # quat = (self.pred_odom.pose.pose.orientation.x, self.pred_odom.pose.pose.orientation.y,
        #         self.pred_odom.pose.pose.orientation.z, self.pred_odom.pose.pose.orientation.w)
        # _, _, theta = tf.transformations.euler_from_quaternion(quat)
        
        # Calculate timestep from last prediction update
        dt = self.time - self.old_time
        
        # Update particle pose estimates based on angular and linear velocities from odometry 
        self.particles[:,0] = self.particles[:,0] + pnoise[:,0] + xvel*dt*np.cos(self.particles[:,2]) 
        self.particles[:,1] = self.particles[:,1] + pnoise[:,1] + xvel*dt*np.sin(self.particles[:,2])  
        self.particles[:,2] = self.particles[:,2] + pnoise[:,2] + omega*dt
        #self.particles[:,2] = self.particles[:,2] + pnoise[:,2] + (theta - self.old_theta) # Alternative angle calculation
        # Force angles to be on range [-pi, pi]
        self.particles[:,2] = np.remainder(self.particles[:,2]+np.pi,2*np.pi)-np.pi
        # Update old theta for comparison
        # self.old_theta = theta

    # Function for observation update
    def obs_update(self):
        # Unpack observation pose estimates
        x_obs  = self.obs_pose.pose.pose.position.x
        y_obs  = self.obs_pose.pose.pose.position.y
        quat = (self.obs_pose.pose.pose.orientation.x, self.obs_pose.pose.pose.orientation.y,
                self.obs_pose.pose.pose.orientation.z, self.obs_pose.pose.pose.orientation.w)
        _, _, self.ang_z_obs = tf.transformations.euler_from_quaternion(quat)

        # Calculate Innovation (difference from measurement to particle pose)
        self.innov[:,0] = x_obs - self.particles[:,0]
        self.innov[:,1] = y_obs - self.particles[:,1]
        self.innov[:,2] = self.ang_z_obs - self.particles[:,2]
        # Force angles to be on range [-pi, pi]
        self.innov[:,2] = np.remainder(self.innov[:,2]+np.pi,2*np.pi)-np.pi        

        # Calculate likelihood
        self.likeli = np.exp(-0.5*np.sum(np.square(self.innov).dot(np.linalg.inv(self.ocov_matrix)), axis=1))
        print(self.likeli)
        #*(1/(2*np.pi*np.sqrt(np.linalg.det(self.ocov_matrix)))) # Constant not needed
        if(sum(self.likeli)<=0):
            print("Likelihood went to 0 | Filter failed")

    # Function to reassign weights to particles
    def weight(self):
        self.particles[:,3] = self.likeli#/sum(self.likeli) #reweight occurs in resample functions

    # Function to resample particles | Systematic Resampling
    def mult_resample(self):
        # Define cumulative density function
        cdf = np.cumsum(self.particles[:,3])
        cdf /= cdf[cdf.size-1]

        # Temporarily store old particle poses and set new to zero
        temp = self.particles[:,[0,1,2]]
        self.particles = np.zeros((self.pc,4))
        # Systematic Resampling
        r = np.random.rand(self.pc,1)
        for i in range(cdf.size):
            ind = np.argmax(cdf >= r[i])
            self.particles[i,[0,1,2]] = temp[ind,:]
        # Reassign even weight of 1 to all new particles
        self.particles[:,3] = np.ones((self.pc,))


    # Unused | Function to resample particles | Systematic Resampling
    def sys_resample(self):
        # Define cumulative density function
        cdf = np.cumsum(self.particles[:,3])
        cdf /= cdf[cdf.size-1]
        # Temporarily store old particle poses and set new to zero
        temp = self.particles[:,[0,1,2]]
        self.particles = np.zeros((self.pc,4))
        # Systematic Resampling
        r = np.random.rand(1)/self.pc
        for i in range(cdf.size):
            ind = np.argmax(cdf >= (r + (i-1)/self.pc))
            self.particles[i,[0,1,2]] = temp[ind,:]
        # Reassign even weight of 1 to all new particles
        self.particles[:,3] = np.ones((self.pc,))


    ##### Support Functions #####

    # Function to publish average of particle poses
    def particle_publish(self):
        # Linear positions
        self.filt_pose.pose.pose.position.x = np.average(self.particles[:,0])
        self.filt_pose.pose.pose.position.y = np.average(self.particles[:,1])
        # self.filt_pose.pose.pose.position.z = 0
        # Angular orientations
        quat = tf.transformations.quaternion_from_euler(0,0,np.average(self.particles[:,2]))
        self.filt_pose.pose.pose.orientation.x = quat[0] 
        self.filt_pose.pose.pose.orientation.y = quat[1] 
        self.filt_pose.pose.pose.orientation.z = quat[2] 
        self.filt_pose.pose.pose.orientation.w = quat[3] 
        # Covariance from particle set as float64[36] list
        self.filt_pose.pose.covariance = self.cov_calc() 
        self.posepub.publish(self.filt_pose)

    # Function to calculate covariance of 3x3 numpy array and return as float64[36] for Odometry msg
    def cov_calc(self):
        covmat = np.cov(self.particles[:,[0,1,2]])
        cov_list = [covmat[0,0],     0.0,     0.0,   0.0,   0.0,    0.0,
                        0.0,     covmat[1,1], 0.0,   0.0,   0.0,    0.0,
                        0.0,         0.0,    99999,  0.0,   0.0,    0.0,
                        0.0,         0.0,     0.0,  99999,  0.0,    0.0,
                        0.0,         0.0,     0.0,   0.0,  99999,   0.0,
                        0.0,         0.0,     0.0,   0.0,   0.0, covmat[2,2]]
        return cov_list
    
    # Function to build 3x3 process and observation covariance matrices
    def cov_matrices_build(self):
        # Build process covariance matrix
        self.pcov_matrix = np.array([[self.pl_cov, 0.0, 0.0],
                                    [0.0, self.pl_cov, 0.0],
                                    [0.0, 0.0, self.pa_cov]])
        # Build observation covariance matrix
        self.ocov_matrix = np.array([[self.ol_cov, 0.0, 0.0],
                                    [0.0, self.ol_cov, 0.0],
                                    [0.0, 0.0, self.oa_cov]])

    # Function to assign gaussian noise from diagonalcovariance matrix
    def gaussian_noise(self, cov_mat):
        var = np.diagonal(cov_mat)
        noise = np.sqrt(var)*np.random.randn(self.pc, 3)
        return noise

    # Callback function for observation pose subscription (from aruco_detect)
    def obs_pose_callback(self, obs_pose_msg):
        self.obs_pose = obs_pose_msg

    # Callback function for prediction odometry subscription (from EKF)
    def pred_up_callback(self, pred_up_msg):
        self.pred_odom = pred_up_msg

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('particle_filter_aruco_remote', anonymous=True)
    rospy.loginfo("Successful initilization of node")
    
    # Create particle filter class
    pf = ArucoParticleFilterRemote()
    rospy.loginfo("ArucoParticleFilterRemote class successfully created")
    
    # Run particle filter
    pf.run_pf()