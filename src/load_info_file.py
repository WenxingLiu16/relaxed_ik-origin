#! /usr/bin/env python
'''
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 11/4/18

PLEASE DO NOT CHANGE CODE IN THIS FILE.  IF TRYING TO SET UP RELAXEDIK, PLEASE REFER TO start_here.py INSTEAD
AND FOLLOW THE STEP-BY-STEP INSTRUCTIONS THERE.  Thanks!
'''
######################################################################################################


import rospy
from RelaxedIK.Utils.colors import bcolors
import os.path
import os
from start_here import joint_ordering, joint_state_topic
from sensor_msgs.msg import JointState

rospy.init_node('load_info_file')

info_file_name = rospy.get_param('info_file_name', default="")

if info_file_name == "":
    print bcolors.WARNING + 'ERROR: info_file_name is a required field in load_info_file.launch.  Please fill in that parameter and run again.' + bcolors.ENDC
    exit(-1)

path_to_src = os.path.dirname(__file__)
file_exists = os.path.isfile(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name)

if not file_exists:
    print bcolors.WARNING + '{} file could not be found in RelaxedIK/Config/info_files.  Was there a misspelling?'.format(info_file_name) + bcolors.ENDC
    exit(-1)

# Automatic setting for starting_config
if joint_state_topic in dict(rospy.get_published_topics()).keys():
    rospy.loginfo("Getting the current status of the robot....")
    joint_states = rospy.wait_for_message(joint_state_topic, JointState, timeout=3)
    rospy.loginfo("Read the current status of the robot!")
    print bcolors.OKGREEN + 'Read the current status of the robot!' + bcolors.ENDC
    loaded_robot_file = open(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name, 'r+a')
    info_file_content_line_by_line = loaded_robot_file.readlines()
    new_robot_info_file = open(path_to_src + '/RelaxedIK/Config/info_files/new_' + info_file_name, 'w')

    for line in info_file_content_line_by_line:
        if line.find("starting_config") == -1: 
            new_robot_info_file.write(line)
        else:
            joint_positions_reordered = []
            for each_joint_name in joint_ordering:
                joint_positions_reordered.append(joint_states.position[joint_states.name.index(each_joint_name)])
            new_robot_info_file.write("starting_config: " + str(joint_positions_reordered) + "\n")

    new_robot_info_file.close()
    loaded_robot_file.close()
    os.remove(path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name)
    os.rename(path_to_src + '/RelaxedIK/Config/info_files/new_' + info_file_name, path_to_src + '/RelaxedIK/Config/info_files/' + info_file_name)
    # End - Automatic setting for starting_config



print bcolors.OKGREEN + 'info file [{}] successfully loaded!'.format(info_file_name) + bcolors.ENDC
rospy.set_param('relaxedIK/info_file_loaded', True)
rospy.set_param('relaxedIK/loaded_info_file_name', info_file_name)

loaded_robot_file = open(path_to_src + '/RelaxedIK/Config/loaded_robot', 'w')
loaded_robot_file.write(info_file_name)
loaded_robot_file.close()


rospy.spin()