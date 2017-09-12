#!/usr/bin/env python

import actionlib
import rospy
import moveit_commander
import moveit_msgs.msg

from srdfdom.srdf import SRDF

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

def fetch_joint_angles_from_srdf(state_name):
    # Create a dictionary of group_states
    statedict = {}
    for x in range(len(robot.group_states)):
        statedict[robot.group_states[x].name]=x

    group_state_joint_angles=[]
    for x in range(6):
    	group_state_joint_angles.append(float(str(robot.group_states[statedict[state_name]].joints[x].value)[1:-1]))
    return group_state_joint_angles



if __name__ == "__main__":
    # Create a node
    rospy.init_node("mobmanip")

    #Parse the SRDF from the parameter server
    robot = SRDF.from_parameter_server()

    # start moveit part
    group = moveit_commander.MoveGroupCommander("manipulator")
    group.set_planner_id("RRTConnectkConfigDefault")
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    move_base = MoveBaseClient()

    while not rospy.is_shutdown():
        # Move Vectorto first location
        rospy.loginfo("Moving to location1...")
        move_base.goto(2.250, 3.118, 0.0)

        # Move arm to "extended_up" configuration
        group.clear_pose_targets()
        group_variable_values=group.get_current_joint_values()
    	group_variable_values=fetch_joint_angles_from_srdf("extended_up")
        group.set_joint_value_target(group_variable_values)
        extended_up = group.plan()
        group.execute(extended_up)

        # Move Vector to second location
        rospy.loginfo("Moving to location2...")
        move_base.goto(-3.53, 3.75, 1.57)

    	# Move arm to "cobra" configuration
        group.clear_pose_targets()
    	group_variable_values=group.get_current_joint_values()
    	group_variable_values=fetch_joint_angles_from_srdf("cobra")
        group.set_joint_value_target(group_variable_values)
        cobra = group.plan()
        group.execute(cobra)
