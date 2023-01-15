#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ROS_TCP_Endpoint_msgs.msg import ControllerInput
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from kortex_driver.msg import *

# Rotation matrix about the x-axis  
def Rx(theta):
    theta = math.radians(theta)
    return np.matrix([[1, 0, 0],
                      [0, math.cos(theta),-math.sin(theta)],
                      [0, math.sin(theta), math.cos(theta)]])


# Rotation matrix about the y-axis  
def Ry(theta):
    theta = math.radians(theta)
    return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                      [0, 1, 0],
                      [-math.sin(theta), 0, math.cos(theta)]])


# Rotation matrix about the z-axis  
def Rz(theta):
    theta = math.radians(theta)
    return np.matrix([[math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta), math.cos(theta) , 0],
                      [0, 0, 1 ]])


# Right controller topic callback function
# Callback function that subscribes to the xyz left controller positions
def right_callback(data):
    global input_pos_kcs, oculus_kinova_diff, relaxedik_kinova_diff
    global onTrackingStart

    # Transition from Left-handed CS (Unity) to Right-handed CS (Global): swap y and z axis
    # Then swap x and new y (which was z) to have x facing forward
    # Negate new y (which is x) to make it align with a global coordinate system
    input_pos_gcs = np.array([data.controller_pos_z, -1 * data.controller_pos_x, data.controller_pos_y])
    input_rot_gcs = np.array([data.controller_rot_x, data.controller_rot_y, data.controller_rot_z, data.controller_rot_w])

    # Transition from Global CS to Kinova CS: rotate around y and z axis
    R_gcs_to_kcs = Rx(0) * Ry(-45) * Rz(-90)
    input_pos_kcs = np.matmul(R_gcs_to_kcs, input_pos_gcs)

    # TODO: use transformations library
    # origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    # Rx = T.rotation_matrix(0.0, xaxis)
    # Ry = T.rotation_matrix(45.0, yaxis)
    # Rz = T.rotation_matrix(-90.0, zaxis)
    # R_gcs_to_kcs = T.concatenate_matrices(Rx, Ry, Rz)
    
    # Parse input_pos_array from a 2D to a 1D array
    temp_array = input_pos_kcs
    input_pos_kcs = [temp_array[0,0], temp_array[0,1], temp_array[0,2]]
    
    # Start tracking if gripButton was pressed
    if data.gripButton == True:

        # On first press recalculate transition (difference) from oculus to 
        if onTrackingStart == True:
            calculate_controller_ee_diff()

            # Remove the flag
            onTrackingStart = False


        # Target position for relaxedIK: oculus input in KCS + differences
        target_pos_kcs = input_pos_kcs - oculus_kinova_diff + relaxedik_kinova_diff

        # print("Raw Input:", round(input_pos_kcs[0], 3), round(input_pos_kcs[1], 3), round(input_pos_kcs[2], 3))
        # print("EE:", round(ee_array[0], 3), round(ee_array[1], 3), round(ee_array[2], 3))
        # print("Command:", round(target_pos_kcs[0], 3), round(target_pos_kcs[1], 3), round(target_pos_kcs[2], 3))      
        # print()

        # Form a message for relaxedIK (right arm)
        pose_r = geom_msgs.Pose()
        pose_r.position.x = target_pos_kcs[0]
        pose_r.position.y = target_pos_kcs[1]
        pose_r.position.z = target_pos_kcs[2]

        # TODO: add orientation support
        pose_r.orientation.w = 1
        pose_r.orientation.x = 0
        pose_r.orientation.y = 0
        pose_r.orientation.z = 0

        # TODO: Form a message for relaxedIK (left arm)
        pose_l = geom_msgs.Pose()
        pose_l.position.x = 0
        pose_l.position.y = 0
        pose_l.position.z = 0

        pose_l.orientation.w = 1
        pose_l.orientation.x = 0
        pose_l.orientation.y = 0
        pose_l.orientation.z = 0

        # Form full message for relaxedIK
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(pose_r)
        ee_pose_goals.ee_poses.append(pose_l)
        ee_pose_goals.header.seq = 0

        # Publish a message with target position
        setpoint.publish(ee_pose_goals)

    # Stop tracking if gripButton was released
    else:
        # Reset the flag
        onTrackingStart = True


# Callback function that subscribes to the end effector positions
def ee_position_callback(data):
    global ee_array, relaxedik_kinova_diff
    global onStartup
    
    # TODO: Add orientation
    ee_array = np.array([data.base.tool_pose_x, 
                        data.base.tool_pose_y, 
                        data.base.tool_pose_z])

    # Calculate a transition from relaxedIK initial absolute postion (0, 0, 0) to KinovaFK on startup
    # ! ! ! Requires relaxedIK homing ! ! !
    if onStartup == True:
        relaxedik_kinova_diff = np.array([0.0, 0.0, 0.0]) - ee_array

        # Remove the flag
        onStartup = False   


# Calculates the difference in position between the end effector and the controller
def calculate_controller_ee_diff():
    global input_pos_kcs, oculus_kinova_diff, ee_array

    # Calculate tranformation matrices
    oculus_kinova_diff[0] =  input_pos_kcs[0] - ee_array[0]
    oculus_kinova_diff[1] =  input_pos_kcs[1] - ee_array[1]
    oculus_kinova_diff[2] =  input_pos_kcs[2] - ee_array[2]

    # print("Raw Input:", round(input_pos_kcs[0], 3), round(input_pos_kcs[1], 3), round(input_pos_kcs[2], 3))
    # print("EE:", round(ee_array[0], 3), round(ee_array[1], 3), round(ee_array[2], 3))
    # print("Diff:", round(oculus_kinova_diff[0], 3), round(oculus_kinova_diff[1], 3), round(oculus_kinova_diff[2], 3))
    # print()


if __name__ == '__main__':
    # Variables
    ee_array = np.array([0.0, 0.0, 0.0])
    oculus_kinova_diff = np.array([0.0, 0.0, 0.0])
    relaxedik_kinova_diff = np.array([0.0, 0.0, 0.0])

    # Flags
    onStartup = True

    # Initialize the node
    rospy.init_node("coordinate_converter", anonymous=True)

    # Publishing
    setpoint = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)

    # Subscribing
    # rospy.Subscriber("leftHandInfo", HandTracking, left_callback)
    rospy.Subscriber("rightControllerInfo", ControllerInput, right_callback)
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, ee_position_callback)

    while not rospy.is_shutdown():
        pass