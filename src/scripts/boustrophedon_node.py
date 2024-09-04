#! /usr/bin/env python

import actionlib.simple_action_client
import rospy
import sys
import actionlib
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg

def boustrophedon_node():
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = -0.9
    pose.pose.position.y = 2.0
    pose.pose.orientation.z = 0.88
    pose.pose.orientation.w = 0.47
    goal = move_base_msgs.msg.MoveBaseGoal(pose)

    # Sends the goal to the action server.
    client.send_goal(goal)
    print("Goal sent")

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("Goal reached")

    # Prints out the result of executing the action
    return client.get_result() 

if __name__ == '__main__':
    try:
        rospy.init_node('boustrophedon_node')
        result = boustrophedon_node()
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)