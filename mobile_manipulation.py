#!/usr/bin/env python

import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg

from math import sin, cos

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("vector_move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()



if __name__ == "__main__":
    # Create a node
    rospy.init_node("mobmanip")

    # start moveit part
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planner_id("RRTConnectkConfigDefault")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    

        # <group_state name="extended_up" group="manipulator">
        #     <joint name="ur5_arm_elbow_joint" value="0" />
        #     <joint name="ur5_arm_shoulder_lift_joint" value="-1.5531" />
        #     <joint name="ur5_arm_shoulder_pan_joint" value="0" />
        #     <joint name="ur5_arm_wrist_1_joint" value="-1.6237" />
        #     <joint name="ur5_arm_wrist_2_joint" value="3.1063" />
        #     <joint name="ur5_arm_wrist_3_joint" value="0" />
        # </group_state>

        # <group_state name="cobra" group="manipulator">
        #     <joint name="ur5_arm_elbow_joint" value="1.4825" />
        #     <joint name="ur5_arm_shoulder_lift_joint" value="-2.1179" />
        #     <joint name="ur5_arm_shoulder_pan_joint" value="1.7649" />
        #     <joint name="ur5_arm_wrist_1_joint" value="-2.1885" />
        #     <joint name="ur5_arm_wrist_2_joint" value="-1.48" />
        #     <joint name="ur5_arm_wrist_3_joint" value="-1.6237" />
        # </group_state>

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()

    while not rospy.is_shutdown():
        # Move to first location
        rospy.loginfo("Moving to location1...")
        move_base.goto(2.250, 3.118, 0.0)

        # Move arm to extended_up configuration
        group.clear_pose_targets()
        group_variable_values=group.get_current_joint_values()
		group_variable_values={0,-1.5531,0,-1.6237,3.1063,0}
        group.set_joint_value_target(group_variable_values)
        extended_up = group.plan()
        group.execute(extended_up)
	
        # Move to second location
        rospy.loginfo("Moving to location2...")
        move_base.goto(-3.53, 3.75, 1.57)

		# Move arm to cobra configuration
        group.clear_pose_targets()
		group_variable_values=group.get_current_joint_values()
		group_variable_values={1.4825,-2.1179,1.7649,-2.1885,-1.48,-1.6237}
        group.set_joint_value_target(group_variable_values)
        cobra = group.plan()
        group.execute(cobra)
		