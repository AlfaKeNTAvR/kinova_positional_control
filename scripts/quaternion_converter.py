#!/usr/bin/env python

import rospy
from std_msgs.msg import *
import transformations as T
import math

from relaxed_ik_ros1.msg import EEPoseGoals
from geometry_msgs.msg import Pose



if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("setter", anonymous=True)

    eepg_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)


    # Main loop
    while True:
        euler_list = input().split(" ")
        quaternion_list = T.quaternion_from_euler(math.radians(float(euler_list[0])), math.radians(float(euler_list[1])), math.radians(float(euler_list[2])))
        
        eepg_message = EEPoseGoals()

        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0

        pose.orientation.w = quaternion_list[0]
        pose.orientation.x = quaternion_list[1]
        pose.orientation.y = quaternion_list[2]
        pose.orientation.z = quaternion_list[3]
        
        eepg_message.ee_poses.append(pose)

        eepg_pub.publish(eepg_message)

