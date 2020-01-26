#!/usr/bin/env python

"""
Module for map initialization used with SVEA ArUco SLAM
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
import tf
import tf2_ros
import tf_conversions


from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Pose, PoseWithCovariance, TransformStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class ArucoMapInit():

    def __init__(self):
        ## Pull necessary ROS parameters from launch file:
        # Read camera frame id
        param = rospy.search_param("camera_frame")
        self.cam_frame = rospy.get_param(param)
        # Read map frame
        param = rospy.search_param("map_frame")
        self.map_frame = rospy.get_param(param)
        # Read fiducial pose topic parameter
        param = rospy.search_param("fiducial_pose_topic")
        self.fid_pose_topic = rospy.get_param(param)
        # Read convergence criteria (number of measurements required to verify placement)
        param = rospy.search_param("convergence_criteria")
        self.convergence_criteria = rospy.get_param(param)
        # Read map file
        param = rospy.search_param("map_file")
        self.map_file = rospy.get_param(param) 

        # Initialize callback variables
        #self.odom_pose = None
        self.fid_pose = None
        # Initialize array for converting to output format
        self.fid_pose_arr = None
        # Intialize print execution flag
        #self.print_flag = False
        
        # Establish subscription to Fiducial Pose
        rospy.Subscriber(self.fid_pose_topic, FiducialTransformArray, self.fid_pose_callback)
        # Delay briefly and initialize odom if subscriber finds no message
        rospy.sleep(.5)

        # Initilize tf2 broadcaster and transform message for camera to aruco marker
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = self.cam_frame
        # if self.cam_frame:
        #     print('if')
        #     self.t.header.frame_id = self.cam_frame
        # else:
        #     print('else')
        #     self.t.header.frame_id = 'qualisys'
        # print(self.t.header.frame_id)


        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)        

        # Get boolean values for initialization mode vs landmark mode
        # param = rospy.search_param("init_mode")
        # self.init_mode = rospy.get_param(param)
        # param = rospy.search_param("landmark_mode")
        # self.landmark_mode = rospy.get_param(param)
        # Read vehicle odom topics
        # param = rospy.search_param("vehicle_odom_topic")
        # self.vehicle_odom_top = rospy.get_param(param)
        # Read estimated pose topic from fiducial SLAM
        # param = rospy.search_param("estimated_pose_topic")
        # self.estimated_pose_top = rospy.get_param(param)
        
        
        #self.odom_pose_create()
        # Broadcast odom and fid poses
        #br = tf2_ros.TransformBroadcaster()
        #br.sendTransform(self.odom_pose)
        #br.sendTransform(self.fid_pose)
        
        # Establish subscription to vehicle odom
        #rospy.Subscriber(self.vehicle_odom_top, Odometry, self.vehicle_odom_callback)


    # Callback function for fiducial pose subscription (from aruco_detect)
    def fid_pose_callback(self, fid_pose_msg):
        self.fid_pose = fid_pose_msg
        if self.fid_pose.transforms != []:
            self.fid_pose.header.frame_id = self.cam_frame
    

 
    def new_fidtrans_to_array(self):
        #r = rospy.Rate(30) # 30hz - matches camera publish rate
        #print("self.fid_pose: ", self.fid_pose)
        # Read fiducial transform array and convert to transform from camera to marker
        if self.fid_pose and len(self.fid_pose.transforms) != 0:
            # Use a temp value so self.fid_pose.transforms does not update before calculations complete
            # Only use primary (first) detected marker
            temp = self.fid_pose.transforms[0]
            self.t.child_frame_id = "fid"+str(temp.fiducial_id)
            self.t.header.stamp = rospy.Time.now()
            self.t.transform = temp.transform
            self.br.sendTransform(self.t)            
        
            # Transform into map coords
            trans = None
            try:
                trans = self.tfBuffer.lookup_transform(self.map_frame, self.t.child_frame_id, rospy.Time(0), rospy.Duration(1.0))
                #print("trans: ", trans)
            except:
                rospy.loginfo('Failure of lookup transfrom from fiducial marker to map')       
            
            # Put into string
            if trans:
                x, y, z = trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z
                quat = (trans.transform.rotation.x, trans.transform.rotation.y, 
                trans.transform.rotation.z, trans.transform.rotation.w)
                r, p, y = tf.transformations.euler_from_quaternion(quat)
                self.fid_pose_arr = np.array([temp.fiducial_id, x, y, z, r*180/np.pi, p*180/np.pi, y*180/np.pi, 0, 1])
                self.fid_pose_arr.astype(float)                
            
            #r.sleep()


    def old_fidtrans_to_array(self):
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
        r = rospy.Rate(2) # [hz] Slow rate of measurements - max is 30hz (camera publish rate)

        while (count < self.convergence_criteria):
            #print(count)
            self.new_fidtrans_to_array()
            print(self.fid_pose_arr)
            if self.fid_pose_arr is not None:
                print(self.fid_pose_arr[0])
                if (count == 0):
                    fid = self.fid_pose_arr[0]
                    self.fid_pose_conv = self.fid_pose_arr
                    print(self.fid_pose_conv)
                    fid_reset_count = 0
                    count += 1
                elif (self.fid_pose_arr[0] == fid):
                    self.fid_pose_conv = (self.fid_pose_arr*count + self.fid_pose_arr)/(count + 1)
                    print("count: ", count)
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
            r.sleep()



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
        #self.fid_pose_conv = np.around(self.fid_pose_conv, decimals = 5)
        #print(self.fid_pose_conv)
        
        #self.fid_pose_conv.tolist()
        #ap_fmt = ['%i','%f', '%f', '%f', '%f', '%f', '%f', '%f', '%i']
        #map_fmt = '%d %f %f %f %f %f %f %f %d'
        #np.savetxt(self.map_file, self.fid_pose_conv[0], newline=" ", fmt= '%i')
        #np.savetxt(self.map_file, self.fid_pose_conv[i], newline=" ", fmt= '%f')

        #CODE FROM init_map.py SCRIPT. SHOULD BE ABLE TO USE SOME FORM OF THIS TO WRITE TO FILE.
        """
        argc = len(sys.argv)
        if argc < 2 or argc > 3:
            print "Usage:  %s fiducial_id [map_file]" % sys.argv[0]
            sys.exit(1)

        fid = int(sys.argv[1])
        if argc == 3:
            map_file = sys.argv[2]
        else: 
            #map_file = os.environ['HOME'] + "/map.txt"
            map_file = os.environ['HOME'] + "/.ros/slam/map.txt"

        dir=os.path.dirname(self.map_file)
        if dir and not os.path.exists(dir):
            os.makedirs(dir)
        if os.path.exists(map_file):
            print "File %s already exists, remove or rename it first" % map_file
            sys.exit(1)
        """        
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
    #ArucoMapInit().__init__()
    rospy.loginfo("Successful execution of init function")
    #while not rospy.is_shutdown():
        # Check/create odom pose if oneis not previously published
        #ArucoMapInit().odom_pose_create()
        #new_map.odom_pose_create()
    new_map.fid_pose_converge()
    new_map.aruco_map_place()
    rospy.sleep(1)

    #ArucoMapInit().fidtrans_to_array()
    print("Map initialization complete")





    """
    Unnecessary Init & Functions
    Delete once script is finalized

    # # Callback function for vehicle odometry subscription (known odom from qualisys, lidar SLAM, etc.)
    # def vehicle_odom_callback(self, odom_msg):
    #     rospy.loginfo("Executed odom_msg callback")
    #     self.odom_pose = odom_msg.pose.pose
    #     #return self.odom_pose
    
    # # Creates an odom pose at origin if no odom is available
    # def odom_pose_create(self):
    #     if self.odom_pose == None:
    #         self.odom_pose = Odometry()
    #         self.odom_pose.header.frame_id = 'temp_aruco_map'
    #         # Publish message
    #         #self.odom_pub = rospy.Publisher(self.vehicle_odom_top, Odometry, queue_size=10)
    #         #rate = rospy.Rate(10)
    #         #self.odom_pub.publish(self.odom_pose)
    #         #rate.sleep()

    # Read vehicle pose topic
    #self.vehicle_pose_top = rospy.get_param("vehicle_pose_topic")
    # Read fiducial vertices topic parameter
    self.fid_vert_topic = rospy.get_param("fiducial_vertices_topic")

    # Establish connection to clear map service
    rospy.wait_for_service('fiducial_slam/clear_map')
    self.clear_map_srv = rospy.ServiceProxy('fiducial_slam/clear_map')
    rospy.loginfo("Connected to clear map server")

    # Establish subscription to Fiducial Vertices
    self.fid_vert_sub = rospy.Subscriber(self.fid_vert_topic, FiducialArray, self.fid_vert_callback)

    def pose_difference_calc(self, odom_pose, est_pose):
        # do stuff
        self.pose_dif = []
        return self.pose_dif

    def pose_difference_average(self, pose_dif):
        self.pose_ave = []
        for i in range(self.convergence_criteria):
            self.pose_ave = (self.pose_ave*i + self.pose_dif)/(i+1)
        return self.pose_ave
    
    # Callback function for fiducial vertices subscription (from aruco_detect)
    def fid_vert_callback(fid_vert_msg):
        global fid_vert
        fid_vert = fid_vert_msg
        return fid_vert

    def clear_map(self):
    # Double check to ensure map is cleared only if set to initialization mode
    if self.init_mode:
        # call clear map service:
        try:
            clear_map_srv()
        except:
            rospy.loginfo("Clear map service failed")

    # Callback function for estimated pose subscription (from fiducial_slam)
    # Current plan is to not use this and use directly from arudco_detect
    def est_pose_callback(pose_msg):
        global est_pose
        est_pose = pose_msg.pose
        return est_pose


    def aruco_pose_calc(self):
        # call aruco detect and get pose between marker and camera

        #vertices_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 1));
        #pose_pub = new ros::Publisher(nh.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 1));

        # If vehicle pose topic is empty, intialize pose_topic parameter
        if self.vehicle_pose_top == "":
            rospy.loginfo("Pose topic is empty. Assigning default value 'SVEA_pose'")
            rospy.set_param("vehicle_pose_topic", 'SVEA_pose')
            
            rospy.loginfo("Placing vehicle at origin of default frame id 'SVEA_map'")

            # create SVEA_pose topic
            # vehicle pose value gets origin
            # frame_id = 'SVEA_map'
        else:
            # read car pose
            if pose_message.frame_id == "":
                rospy.loginfo("Pose message frame id is empty. Assigning default value 'SVEA_map'")
                # frame_id = 'SVEA_map'
                # assumed vehicle pose will be at origin, but may need to assign
                rospy.sleep(1)

        # tf from vehicle pose to camera location

        # tf between camera and detected aruco to get aruco pose in map frame id




    """