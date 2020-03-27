#!/usr/bin/python

import rospy
import os
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from RelaxedIK.relaxedIK import get_relaxedIK_from_info_file
from RelaxedIK.Utils.interactive_marker_utils import InteractiveMarkerFeedbackUtil, InteractiveMarkerUtil, InteractiveMarkerServerUtil
from relaxed_ik.msg import EEPoseGoals
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose
import RelaxedIK.Utils.transformations as T
from RelaxedIK.Utils.colors import bcolors
from teleop import user_cmd_to_marker

# _user_cmd_to_marker = user_cmd_to_marker(user_interface="joy")
_user_cmd_to_marker = user_cmd_to_marker(user_interface="vive")
# _user_cmd_to_marker = user_cmd_to_marker(user_interface="unity")
if(_user_cmd_to_marker.active is True):
    _user_cmd_to_marker.start_sub_user_cmd()

rospy.init_node('marker_ikgoal_driver')

path_to_src = os.path.dirname(__file__)

relaxedIK = get_relaxedIK_from_info_file(path_to_src)
num_chains = relaxedIK.vars.robot.numChains

init_ee_positions =  relaxedIK.vars.init_ee_positions
init_ee_quats =  relaxedIK.vars.init_ee_quats

server = InteractiveMarkerServer("simple_marker")
ee_pose_goal_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=3)

rospy.sleep(0.2)

int_markers = []

for i in xrange(num_chains):
    int_marker = InteractiveMarkerUtil(init_pos=init_ee_positions[i], init_quat=init_ee_quats[i])
    int_marker.add_6dof_controls()
    int_markers.append(int_marker)

    server.insert(int_marker.interactive_marker, int_marker.feedback_util.feedback_handler)

server.applyChanges()


rate = rospy.Rate(40)
while not rospy.is_shutdown():
    eepg = EEPoseGoals()

    for i in xrange(num_chains):
        if not int_markers[i].feedback_util.active:
            # WARNING: When you turned off this node and rerun it, due to the following lines, the robot will go to "starting_config" (i.e. "init_ee_positions" and "init_ee_quats"), which may cause some safety issues. 
            pose_goal_relative = Pose()
            pose_goal_relative.position.x = 0.0
            pose_goal_relative.position.y = 0.0
            pose_goal_relative.position.z = 0.0

            pose_goal_relative.orientation.w = 1.0
            pose_goal_relative.orientation.x = 0.0
            pose_goal_relative.orientation.y = 0.0
            pose_goal_relative.orientation.z = 0.0
        else:
            # Position/rotation command should be relative to "starting_config". See "relaxedIK.py" line 57 (i.e. def solve). (Commented by Inmo)
            pose_by_marker = int_markers[i].feedback_util.feedback.pose # Pose input via Rviz marker (Note: this value is only updated via Rviz interaction, not by "server.applyChanges()" below)

            pose_goal = _user_cmd_to_marker.add_user_cmd(pose_by_marker, _user_cmd_to_marker.del_pose_by_user) # Adding del pose input via additional user interface  

            # Rviz Visualisation
            server.setPose(int_markers[i].feedback_util.feedback.marker_name, pose_goal)
            server.applyChanges()
            
            # Compute the relative goal position w.r.t. the initial config
            pose_goal_relative = Pose()

            pose_goal_relative.position.x = pose_goal.position.x - init_ee_positions[i][0]
            pose_goal_relative.position.y = pose_goal.position.y - init_ee_positions[i][1]
            pose_goal_relative.position.z = pose_goal.position.z - init_ee_positions[i][2]

            quat_w = pose_goal.orientation.w
            quat_x = pose_goal.orientation.x
            quat_y = pose_goal.orientation.y
            quat_z = pose_goal.orientation.z
            
            quat_goal = [quat_w, quat_x, quat_y, quat_z]

            quat_goal_relative = T.quaternion_multiply(quat_goal,T.quaternion_inverse(init_ee_quats[i]))
            pose_goal_relative.orientation.w = quat_goal_relative[0]
            pose_goal_relative.orientation.x = quat_goal_relative[1]
            pose_goal_relative.orientation.y = quat_goal_relative[2]
            pose_goal_relative.orientation.z = quat_goal_relative[3]

        # Publish to IK solver
        eepg.ee_poses.append(pose_goal_relative)

    ee_pose_goal_pub.publish(eepg)

    rate.sleep()

print bcolors.WARNING + "WARNING: If you rerun this node only, the robot will abruptly go to 'starting_config', which may cause some safety issues. Recommend you to load the robot's current status info again" + bcolors.ENDC