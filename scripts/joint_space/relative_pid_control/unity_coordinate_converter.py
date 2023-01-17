#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ROS_TCP_Endpoint_msgs.msg import ControllerInput
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from kortex_driver.msg import *


# Right controller topic callback function
# Callback function that subscribes to the xyz left controller positions
def right_callback(data):
    global input_pos_kcs, oculus_kinova_diff, relaxedik_kinova_diff
    global onTrackingStart

    # # POSITION
    # Transition from Left-handed CS (Unity) to Right-handed CS (Global): swap y and z axis
    # Then swap x and new y (which was z) to have x facing forward
    # Negate new y (which is x) to make it align with a global coordinate system
    input_pos_gcs = np.array([data.controller_pos_z, -1 * data.controller_pos_x, data.controller_pos_y])

    # Transition from Global CS to Kinova CS: rotate around y and z axis
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = T.rotation_matrix(math.radians(0.0), xaxis)
    Ry = T.rotation_matrix(math.radians(-45.0), yaxis)
    Rz = T.rotation_matrix(math.radians(-90.0), zaxis)

    R_gcs_to_kcs = T.concatenate_matrices(Rx, Ry, Rz)
    input_pos_kcs = np.matmul(R_gcs_to_kcs[0:3,0:3], input_pos_gcs)


    # # ORIENTATION
    # Raw quaternion input (Left-handed CS)
    input_rot_gcs = np.array([data.controller_rot_x, data.controller_rot_y, data.controller_rot_z, data.controller_rot_w])
    
    # Transition from Left-handed CS (Unity) to Right-handed CS (Global)
    input_rot_gcs = T.euler_from_quaternion(input_rot_gcs)
    input_rot_gcs = T.euler_matrix(-1 * input_rot_gcs[1], -1 * input_rot_gcs[2], input_rot_gcs[0])
    input_rot_gcs = T.quaternion_from_matrix(input_rot_gcs)

    # More comfortable position (compensation)
    Qy = T.quaternion_about_axis(math.radians(35), yaxis)
    input_rot_gcs = T.quaternion_multiply(input_rot_gcs, Qy)

    Qz = T.quaternion_about_axis(math.radians(15), zaxis)
    input_rot_gcs = T.quaternion_multiply(input_rot_gcs, Qz)

    # Transition from Global CS to Kinova CS: rotate around y and z axis
    input_rot_kcs = np.matmul(R_gcs_to_kcs, input_rot_gcs)
    
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
        pose_r.orientation.x = input_rot_kcs[0]
        pose_r.orientation.y = input_rot_kcs[1]
        pose_r.orientation.z = input_rot_kcs[2]
        pose_r.orientation.w = input_rot_kcs[3]

        # TODO: Form a message for relaxedIK (left arm)
        pose_l = geom_msgs.Pose()
        pose_l.position.x = 0
        pose_l.position.y = 0
        pose_l.position.z = 0

        pose_l.orientation.x = 0
        pose_l.orientation.y = 0
        pose_l.orientation.z = 0
        pose_l.orientation.w = 1

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

        # TODO: stop any robot motion


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

        # # Homing position
        # ja_init = JointAngles()

        # init_joint_1 = JointAngle()
        # init_joint_1.joint_identifier = 0
        # init_joint_1.value = -0.06562561558173297
        # ja_init.joint_angles.append(init_joint_1)

        # init_joint_2 = JointAngle()
        # init_joint_2.joint_identifier = 1
        # init_joint_2.value = 1.8538876875092294
        # ja_init.joint_angles.append(init_joint_2)

        # init_joint_3 = JointAngle()
        # init_joint_3.joint_identifier = 2
        # init_joint_3.value = -3.1234911476605247
        # ja_init.joint_angles.append(init_joint_3)

        # init_joint_4 = JointAngle()
        # init_joint_4.joint_identifier = 3
        # init_joint_4.value = -0.5407607182949867
        # ja_init.joint_angles.append(init_joint_4)

        # init_joint_5 = JointAngle()
        # init_joint_5.joint_identifier = 4
        # init_joint_5.value = 1.6008414444228978
        # ja_init.joint_angles.append(init_joint_5)

        # init_joint_6 = JointAngle()
        # init_joint_6.joint_identifier = 5
        # init_joint_6.value = 1.5267517702646756
        # ja_init.joint_angles.append(init_joint_6)

        # init_joint_7 = JointAngle()
        # init_joint_7.joint_identifier = 6
        # init_joint_7.value = 1.5836379564132042
        # ja_init.joint_angles.append(init_joint_7)

        # angles_pub.publish(ja_init)

        # print("Kinova is homing...")
        # # rospy.sleep(30)
        # print("Homed")

        # Calculate the difference after homing
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
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)

    # Subscribing
    # rospy.Subscriber("leftHandInfo", HandTracking, left_callback)
    rospy.Subscriber("rightControllerInfo", ControllerInput, right_callback)
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, ee_position_callback)

    while not rospy.is_shutdown():
        pass