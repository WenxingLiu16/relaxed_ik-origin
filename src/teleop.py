#!/usr/bin/env python

#
# This library is to link RelaxedIK with any user interface
#
# Author: Inmo Jang
import rospy
from geometry_msgs.msg import Pose
from RelaxedIK.Utils.colors import bcolors
import RelaxedIK.Utils.transformations as T

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist
import copy

class user_cmd_to_marker:
    def __init__(self, user_interface):
        self.del_pose_by_user = Pose()
        self.del_pose_by_user.orientation.w = 1.0
        self.active = True        
        if user_interface is "joy":
            self.user_cmd_topic_name = "/joy"
            self.user_cmd_msg_type = Joy
            self.cb_user_cmd = self.cb_user_cmd_joy
            self.add_user_cmd = self.add_user_cmd_ros
            print bcolors.WARNING + "Joystick is the user interface" + bcolors.ENDC
        elif user_interface is "vive" or user_interface is "HTC":
            self.user_cmd_topic_name = "/vive/twist1"
            self.user_cmd_msg_type = TwistStamped
            self.cb_user_cmd = self.cb_user_cmd_vive
            self.add_user_cmd = self.add_user_cmd_ros
            self.htc_active = False            
            # Subscriber for buttons
            rospy.Subscriber("/vive/controller_LHR_FF6F7941/joy", Joy, self.cb_htc_button)
            print bcolors.WARNING + "HTC Vice is the user interface" + bcolors.ENDC
        elif user_interface is "Unity" or user_interface is "unity":
            self.user_cmd_topic_name = "/unity/twist1"
            self.user_cmd_msg_type = Twist
            self.cb_user_cmd = self.cb_user_cmd_unity_twist         
            self.add_user_cmd = self.add_user_cmd_unity           
            self.ini_pose_by_marker = Pose()            
            print bcolors.WARNING + "Unity Twist (position-based) is the user interface" + bcolors.ENDC              
        else:
            self.active = False
            self.user_cmd_topic_name = ""
            self.user_cmd_msg_type = TwistStamped
            print bcolors.WARNING + "WARNING: No user interface is defined" + bcolors.ENDC

    ## (1) For Joystick (Joy)
    def cb_user_cmd_joy(self, data):        
        linear_scale = 0.002
        pose = Pose()
        pose.orientation.w = 1.0

        pose.position.x = data.axes[1]*linear_scale
        pose.position.y = data.axes[0]*linear_scale
        pose.position.z = data.axes[4]*linear_scale
        
        angular_scale = 0.01
        roll = (data.buttons[1]-data.buttons[3])*angular_scale
        pitch = (data.buttons[0]-data.buttons[2])*angular_scale
        yaw = (data.buttons[4]-data.buttons[5])*angular_scale
        quat = T.quaternion_from_euler(roll,pitch,yaw)
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]          
        self.del_pose_by_user = pose

    ## (2) For HTC Vive (TwistStamped)      
    # TODO:position is velocity / rotation is absolute      
    def cb_user_cmd_vive(self, data):        
        dt = 0.01
        pose = Pose()
        pose.orientation.w = 1.0

        if (self.htc_active is True):
            pose.position.x = -data.twist.linear.x*dt
            pose.position.y = data.twist.linear.z*dt
            pose.position.z = data.twist.linear.y*dt

            roll = -data.twist.angular.x*dt
            pitch = data.twist.angular.z*dt
            yaw = data.twist.angular.y*dt
            quat = T.quaternion_from_euler(roll,pitch,yaw)
            pose.orientation.w = quat[0]
            pose.orientation.x = quat[1]
            pose.orientation.y = quat[2]
            pose.orientation.z = quat[3]        

        self.del_pose_by_user = pose

    def cb_htc_button(self, data):        
        if(data.buttons[1] == 1):
            self.htc_active = True
        if(data.buttons[1] == 0):
            self.htc_active = False


    ## (3) For Absolute Twist input from Unity (Twist - This is already tranformed to ROS coordinate)
    # Diffferences from "cb_user_cmd_vive" are as follows
    # - The input "data" from the Unity side already include ROS coordinate transformation 
    # - It becomes zero values while the user in Unity side isn't pressing the trigger button. 
    # TODO:position is velocity / rotation is absolute       
    def cb_user_cmd_unity_twist(self, data):    
        pose = Pose()
        pose.orientation.w = 1.0

        pose.position.x = data.linear.x
        pose.position.y = data.linear.y
        pose.position.z = data.linear.z

        roll = data.angular.x
        pitch = data.angular.y
        yaw = data.angular.z
        quat = T.quaternion_from_euler(roll,pitch,yaw)
        pose.orientation.w = quat[0]
        pose.orientation.x = quat[1]
        pose.orientation.y = quat[2]
        pose.orientation.z = quat[3]        

        self.del_pose_by_user = pose

    # To adjust the marker's position by adding a user command from another interface in ROS
    def add_user_cmd_ros(self, pose_by_marker, del_pose_by_user):        
        pose = pose_by_marker # Note: by this, Rviz marker position is also changing by the user device
        pose.position.x += del_pose_by_user.position.x
        pose.position.y += del_pose_by_user.position.y
        pose.position.z += del_pose_by_user.position.z
      
        quat_existing = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        quat_user = [del_pose_by_user.orientation.w, del_pose_by_user.orientation.x, del_pose_by_user.orientation.y, del_pose_by_user.orientation.z]
        
        new_quat = T.quaternion_multiply(quat_user, quat_existing)
        pose.orientation.w = new_quat[0]
        pose.orientation.x = new_quat[1]
        pose.orientation.y = new_quat[2]
        pose.orientation.z = new_quat[3]

        return pose

    # To adjust the marker's position by adding a user command from another interface in Unity
    def add_user_cmd_unity(self, pose_by_marker, del_pose_by_user):     
        if(del_pose_by_user.position.x**2 + del_pose_by_user.position.y**2 + del_pose_by_user.position.z**2 + (del_pose_by_user.orientation.w - 1.0)**2 + del_pose_by_user.orientation.x**2 + del_pose_by_user.orientation.y**2 + del_pose_by_user.orientation.z**2 == 0.0):
            self.ini_pose_by_marker = copy.deepcopy(pose_by_marker)            
            # rospy.loginfo("ini_pose_by_marker update")
        # else:            
            
        #     rospy.loginfo("self.ini_pose_by_marker = " + str(self.ini_pose_by_marker.position))



        
        pose = copy.deepcopy(self.ini_pose_by_marker)
        pose.position.x += del_pose_by_user.position.x
        pose.position.y += del_pose_by_user.position.y
        pose.position.z += del_pose_by_user.position.z
      
        quat_existing = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        quat_user = [del_pose_by_user.orientation.w, del_pose_by_user.orientation.x, del_pose_by_user.orientation.y, del_pose_by_user.orientation.z]
        
        new_quat = T.quaternion_multiply(quat_user, quat_existing)
        pose.orientation.w = new_quat[0]
        pose.orientation.x = new_quat[1]
        pose.orientation.y = new_quat[2]
        pose.orientation.z = new_quat[3]

        # Update "pose_by_marker". This was implemented here due to the Python feature. 
        pose_by_marker_update = pose_by_marker # Note: pose_by_marker is the position where the user lastly dropped the Rviz marker
        pose_by_marker_update.position.x = pose.position.x
        pose_by_marker_update.position.y = pose.position.y
        pose_by_marker_update.position.z = pose.position.z
      
        pose_by_marker_update.orientation.w = new_quat[0]
        pose_by_marker_update.orientation.x = new_quat[1]
        pose_by_marker_update.orientation.y = new_quat[2]
        pose_by_marker_update.orientation.z = new_quat[3]        

        return pose



    def start_sub_user_cmd(self): 
        rospy.Subscriber(self.user_cmd_topic_name, self.user_cmd_msg_type, self.cb_user_cmd)


