#!/usr/bin/env python

"""
Module for map initialization for SVEA ArUco SLAM
This script works in conjunction with 
aruco_detect and fiducial_slam ROS packages

Developed for: KTH Smart Mobility Lab
Developed by: Kyle Coble
"""

import sys
import os
import math
import rospy
import numpy as np
import tf2_ros


from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Pose, PoseWithCovariance
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ArucoMapInit():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Get boolean values for initialization mode vs landmark mode
        param = rospy.search_param("init_mode")
        self.init_mode = rospy.get_param(param)
        param = rospy.search_param("landmark_mode")
        self.landmark_mode = rospy.get_param(param)
        # Read vehicle odom topics
        param = rospy.search_param("vehicle_odom_topic")
        self.vehicle_odom_top = rospy.get_param(param)
        # Read estimated pose topic from fiducial SLAM
        param = rospy.search_param("estimated_pose_topic")
        self.estimated_pose_top = rospy.get_param(param)
        # Read convergence criteria (number of measurements required to verify placement)
        param = rospy.search_param("convergence_criteria")
        self.convergence_criteria = rospy.get_param(param)
        # Read fiducial pose topic parameter
        param = rospy.search_param("fiducial_pose_topic")
        self.fid_pose_topic = rospy.get_param(param)
        # Read camera frame id
        param = rospy.search_param("camera_frame")
        self.cam_frame = rospy.get_param(param)
        # Read map file
        param = rospy.search_param("map_file")
        self.map_file = rospy.get_param(param) 

        # Initialize callback variables
        self.odom_pose = None
        self.fid_pose = None
        # Initialize array for converting to output format
        self.fid_pose_arr = None
        # Intialize print execution flag
        self.print_flag = False
        
        # Establish subscription to Fiducial Pose
        rospy.Subscriber(self.fid_pose_topic, FiducialTransformArray, self.fid_pose_callback)
        # Establish subscription to vehicle odom
        rospy.Subscriber(self.vehicle_odom_top, Odometry, self.vehicle_odom_callback)
        # Delay briefly and initialize odom if subscriber finds no message
        rospy.sleep(2)
        self.odom_pose_create()
        # Broadcast odom and fid poses
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(self.odom_pose)
        br.sendTransform(self.fid_pose)


    # Callback function for fiducial pose subscription (from aruco_detect)
    def fid_pose_callback(self, fid_pose_msg):
        self.fid_pose = fid_pose_msg
        if self.fid_pose.transforms != []:
            self.fid_pose.header.frame_id = self.cam_frame
    
    # Callback function for vehicle odometry subscription (known odom from qualisys, lidar SLAM, etc.)
    def vehicle_odom_callback(self, odom_msg):
        rospy.loginfo("Executed odom_msg callback")
        self.odom_pose = odom_msg.pose.pose
        #return self.odom_pose
    
    # Creates an odom pose at origin if no odom is available
    def odom_pose_create(self):
        if self.odom_pose == None:
            self.odom_pose = Odometry()
            self.odom_pose.header.frame_id = 'temp_aruco_map'
            # Publish message
            #self.odom_pub = rospy.Publisher(self.vehicle_odom_top, Odometry, queue_size=10)
            #rate = rospy.Rate(10)
            #self.odom_pub.publish(self.odom_pose)
            #rate.sleep()
 
    #def odom_publish


    def fidtrans_to_array(self):
        if len(self.fid_pose.transforms) != 0:
            #### Still need to incorporate odom_pose
            self.map_fid_pose = self.fid_pose.transforms[0] #need to still subtract out odom_pose
            fid = self.map_fid_pose.fiducial_id
            lin = self.map_fid_pose.transform.translation
            rot = self.map_fid_pose.transform.rotation
            (r, p, y) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            (r, p, y) = (r*180/np.pi, p*180/np.pi, y*180/np.pi)
            self.fid_pose_arr = np.array([fid, lin.z, lin.y, -1*lin.x+.22, r-90, y+90, p-90, 0, 1])
            #self.fid_pose_arr = np.array([fid, lin.z, lin.y, -1*lin.x+.22, r-90, y+90, p-90, 0, 1])
            self.fid_pose_arr.astype(float)
            return self.fid_pose_arr

    def fid_pose_converge(self):
        count = 0
        fid_reset_count = 0
        while (count < self.convergence_criteria):
            #print(count)
            self.fidtrans_to_array()
            if (self.fid_pose_arr != None):
                print(self.fid_pose_arr[0])
                if (count == 0):
                    fid = self.fid_pose_arr[0]
                    self.fid_pose_conv = self.fid_pose_arr
                    print(self.fid_pose_conv)
                    fid_reset_count = 0
                    count += 1
                elif (self.fid_pose_arr[0] == fid):
                    self.fid_pose_conv = (self.fid_pose_arr*count + self.fid_pose_arr)/(count + 1)
                    print("count", count)
                    print(self.fid_pose_conv)
                    fid_reset_count = 0
                    count += 1
                # Check if another marker is being seen and preferred by aruco detect
                elif (self.fid_pose_arr[0] != fid):
                    fid_reset_count += 1
                # Reset fid_pose_arr so same values aren't used again
                self.fid_pose_arr = None
            # If other marker is seen enough times, initialize that marker instead
            if fid_reset_count > 3:
                count = 0
            # Pause before taking next measurement
            rospy.sleep(.5)



    def check_mode(self):
        # Determine whether initialization mode or landmark mode was set
        if (self.init_mode and self.landmark_mode) or (not self.init_mode and not self.landmark_mode):
            rospy.loginfo("Must choose either initialization or landmarking mode")
            # ??? Is there a way to automatically kill node / would we want to here?
            rospy.sleep(1)
        
    

    def aruco_map_place(self):
        # map_vals get string of values for pose, aruco id, etc.
        # structure for line to write:
        # id x y z pan tilt roll variance numObservations links
          
        # Opens file and creates file if it is not there
        # This will overwrite whatever first line is there
        file = open(self.map_file, "w+")
        file.write("%d %f %f %f %f %f %f %d %d\n" %(self.fid_pose_conv[0], self.fid_pose_conv[1], self.fid_pose_conv[2], self.fid_pose_conv[3], self.fid_pose_conv[4], self.fid_pose_conv[5], self.fid_pose_conv[6], self.fid_pose_conv[7], self.fid_pose_conv[8]))
        file.close()
        # Use if statement and "a+" instead of w+ for append to not overwrite in landmark mode


if __name__ == "__main__":
    
    rospy.init_node('aruco_map_init', anonymous=True)
    rospy.loginfo("Successful initilization of node")
    
    new_map = ArucoMapInit()
    rospy.loginfo("Successful execution of init function")
    
    new_map.fid_pose_converge()
    new_map.aruco_map_place()
    rospy.sleep(1)

    print("Map initialization complete")