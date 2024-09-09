#! /usr/bin/env python

import actionlib.simple_action_client
import rospy
import sys
import actionlib
from geometry_msgs.msg import PoseStamped
import move_base_msgs.msg
from copy import deepcopy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

def generate_boustrophedon_path():
    robot_length = 0.75 #m
    arena_length = 6.0 #m
    arena_width = 6.0 #m
    step = 0.5 #m
    path = []
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = arena_length/2 - robot_length
    pose.pose.position.y = arena_width/2 - robot_length
    yaw = 3.14
    quaternion = quaternion_from_euler(0, 0, yaw)
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    i = 0
    while (abs(pose.pose.position.y) <= arena_width/2 - robot_length):
        path.append(deepcopy(pose))
        i += 1
        if i % 2 == 0:
            pose.pose.position.y -= step
            yaw = 3.14 if yaw == 0 else 0
            quaternion = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
        else:
            pose.pose.position.x = -pose.pose.position.x
    
    return path
        

def boustrophedon_node():
    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    path = generate_boustrophedon_path()
    for pose in path:
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        yaw = euler[2]
        print(f"Sending goal to x: {pose.pose.position.x}, y: {pose.pose.position.y}, yaw: {math.degrees(yaw)}")
        goal = move_base_msgs.msg.MoveBaseGoal(pose)

        # Sends the goal to the action server.
        client.send_goal(goal)
        print("Goal sent")

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        print("Goal reached")

        # Prints out the result of executing the action
        print(client.get_result())

if __name__ == '__main__':
    try:
        rospy.init_node('boustrophedon_node')
        boustrophedon_node()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)