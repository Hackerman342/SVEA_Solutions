#!/usr/bin/env python  
"""
Module for kinematic bicycle model for SVEA cars.
Output is Odomtery message with twist (velocity) components filled
Developed for: KTH Smart Mobility Lab
Developed by: Kyle Coble
"""
import sys
import os
import math
import rospy
import numpy as np

from svea_arduino.msg import lli_ctrl
from nav_msgs.msg import Odometry


class Republish():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Read control message topic
        param = rospy.search_param("ctrl_message_topic")
        self.ctrl_msg_top = rospy.get_param(param)
        # Read output message topic
        param = rospy.search_param("output_message_topic")
        self.odom_msg_top = rospy.get_param(param)
        # Read vehicle frame id topic
        param = rospy.search_param("est_frame_id")
        self.veh_frame_id = rospy.get_param(param)
        # Read max speed
        param = rospy.search_param("max_speed_gear_1")
        self.max_speed_g1 = rospy.get_param(param)
        param = rospy.search_param("max_speed_gear_2")
        self.max_speed_g2 = rospy.get_param(param)
        # Read dead zone
        param = rospy.search_param("dead_zone")
        dead_zone = rospy.get_param(param)
        self.dz = [-1*dead_zone, dead_zone]
        # Read wheelbase
        param = rospy.search_param("wheel_base")
        self.wheelbase = rospy.get_param(param)
        # Read covariance values
        param = rospy.search_param("linear_covariance")
        self.lin_cov = rospy.get_param(param)
        param = rospy.search_param("angular_covariance")
        self.ang_cov = rospy.get_param(param)
        # Read publish rate (hz)
        param = rospy.search_param("publish_rate")
        self.publish_rate = rospy.get_param(param)
     
        # Initialize callback variables
        self.ctrl_msg = None
       
        # Initialize class variables
        self.odom_msg = None

        # Establish subscription to control message
        rospy.Subscriber(self.ctrl_msg_top, lli_ctrl, self.ctrl_msg_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(0.5)

        # Establish publisher of converted Odom message
        self.pub = rospy.Publisher(self.odom_msg_top, Odometry, queue_size=10)

    def ctrl_calc_and_pub(self):
        # initialize message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.veh_frame_id
        self.odom_msg.twist.covariance = self.cov_matrix_build()

        # Temporary until 2nd gear calc is added
        self.max_speed = self.max_speed_g1

        # Set rate to publish
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            # Likely need to change this to contunially publish commands when not on remote control
            # RC constantly publishes controls, while computer would send command until it needs to be changed
            if self.ctrl_msg != None:
                # Unpack ctrl message
                # Gear --- HOW TO USE THIS VALUE ?????
                gear = self.ctrl_msg.trans_diff
                # Steering range (radians) - assumed linear across range
                # msg [-127,127] | actual [-pi/4,pi/4] | direction (right,left)
                c_ang = self.ctrl_msg.steering*(math.pi/4)/127
                # Velocity (m/s) - assumed linear across range with deadzone
                # msg [-127,127] | direction (back,forward)
                if self.ctrl_msg.velocity < self.dz[0]:
                    c_vel = (self.ctrl_msg.velocity - self.dz[0])*-1*self.max_speed/(-127 - self.dz[0])
                elif self.ctrl_msg.velocity > self.dz[1]:
                    c_vel = (self.ctrl_msg.velocity - self.dz[1])*self.max_speed/(127 - self.dz[1])
                else:
                    c_vel = 0

                # Apply Bicycyle Model
                B = math.atan2(math.tan(c_ang),2)
                x_vel = c_vel*math.cos(B)
                y_vel = c_vel*math.sin(B)
                ang_vel = (c_vel/(self.wheelbase/2))*math.sin(B)

                # Build Header for current time stamp
                self.odom_msg.header.seq += 1
                self.odom_msg.header.stamp = rospy.Time.now()
                
                # Build Odom Twist using bicycle model
                self.odom_msg.twist.twist.linear.x = x_vel
                self.odom_msg.twist.twist.linear.y = y_vel
                self.odom_msg.twist.twist.angular.z = ang_vel

            # Publish message - whether or not new controls come in
            # This is because controls are not published until a command change occurs
            # This is important for automatic control (negligible for RC [remote control])
            self.pub.publish(self.odom_msg)
            rate.sleep()

    def cov_matrix_build(self):
        self.cov_matrix = [self.lin_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, self.lin_cov, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, self.lin_cov, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, self.ang_cov, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, self.ang_cov, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, self.ang_cov]
        return self.cov_matrix


    # Callback function for fiducial pose subscription (from aruco_detect)
    def ctrl_msg_callback(self, ctrl_msg):
        self.ctrl_msg = ctrl_msg

if __name__ == '__main__':

    rospy.init_node('ctrl_bicycle', anonymous=True)
    rospy.loginfo("Successful initilization of node")

    ctt = Republish()
    ctt.ctrl_calc_and_pub()
