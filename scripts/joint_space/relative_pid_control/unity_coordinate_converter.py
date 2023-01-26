#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ROS_TCP_Endpoint_msgs.msg import ControllerInput
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from kortex_driver.msg import *
from kortex_driver.srv import *
from kinova_positional_control.srv import *
from relaxed_ik_ros1.srv import activate_ik


def relaxed_ik_publish(target_position, target_orientation):

    # Form a message for relaxedIK (right arm)
    pose_r = geom_msgs.Pose()
    pose_r.position.x = target_position[0]
    pose_r.position.y = target_position[1]
    pose_r.position.z = target_position[2]

    pose_r.orientation.w = target_orientation[0]
    pose_r.orientation.x = target_orientation[1]
    pose_r.orientation.y = target_orientation[2]
    pose_r.orientation.z = target_orientation[3] 

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
    print(target_orientation)
    euler_print = T.euler_from_quaternion(target_orientation)

    print("Published rot:", round(math.degrees(euler_print[0]), 3), round(math.degrees(euler_print[1]), 3), round(math.degrees(euler_print[2]),3))
    print()
    setpoint.publish(ee_pose_goals)


# Right controller topic callback function
# Callback function that subscribes to the xyz left controller positions
def right_callback(data):
    global input_pos_kcs, input_rot_kcs 
    global oculus_kinova_diff_pos, oculus_kinova_diff_rot, relaxedik_kinova_diff_pos, relaxedik_kinova_diff_rot
    global onTrackingStart
    global gripperButtonState, gripperButtonReleased

    # # POSITION
    # Transition from Left-handed CS (Unity) to Right-handed CS (Global): swap y and z axis
    # Then swap x and new y (which was z) to have x facing forward
    # Negate new y (which is x) to make it align with a global coordinate system
    input_pos_gcs = np.array([-1* data.controller_pos_z, data.controller_pos_x, data.controller_pos_y])

    # Transition from Global CS to Kinova CS: rotate around y and z axis
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = T.rotation_matrix(math.radians(0.0), xaxis)
    Ry = T.rotation_matrix(math.radians(-45.0), yaxis)
    Rz = T.rotation_matrix(math.radians(90.0), zaxis)

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
    Qy = T.quaternion_about_axis(math.radians(-45), yaxis)
    input_rot_gcs = T.quaternion_multiply(input_rot_gcs, Qy)

    # Qz = T.quaternion_about_axis(math.radians(15), zaxis)
    # input_rot_gcs = T.quaternion_multiply(input_rot_gcs, Qz)

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
        target_pos_kcs = input_pos_kcs - oculus_kinova_diff_pos + relaxedik_kinova_diff_pos

        # print("Raw Input:", round(input_pos_kcs[0], 3), round(input_pos_kcs[1], 3), round(input_pos_kcs[2], 3))
        # print("EE:", round(ee_array[0], 3), round(ee_array[1], 3), round(ee_array[2], 3))
        # print("Command:", round(target_pos_kcs[0], 3), round(target_pos_kcs[1], 3), round(target_pos_kcs[2], 3))      
        # print()

        relaxed_ik_publish(target_pos_kcs, [1, 0, 0, 0])
        #relaxed_ik_publish(target_pos_kcs, target_rot_kcs)

    # Stop tracking if gripButton was released
    else:
        # Reset the flag
        onTrackingStart = True

        # TODO: stop any robot motion

    if data.triggerButton:
        gripperButtonState = True
        
        # Call gripper state machine
        gripper_sm()

        gripperButtonReleased = False

    else:
        gripperButtonState = False
        
        # Call gripper state machine
        gripper_sm()

        gripperButtonReleased = True

# Callback function that subscribes to the end effector positions
def ee_position_callback(data):
    global ee_array, relaxedik_kinova_diff_pos, relaxedik_kinova_diff_rot
    global onStartup
    
    # TODO: Add orientation
    ee_array = np.array([data.base.tool_pose_x, 
                        data.base.tool_pose_y, 
                        data.base.tool_pose_z,
                        math.radians(data.base.tool_pose_theta_x), 
                        math.radians(data.base.tool_pose_theta_y), 
                        math.radians(data.base.tool_pose_theta_z)])

    # Calculate a transition from relaxedIK initial absolute postion (0, 0, 0) to KinovaFK on startup
    # ! ! ! Requires relaxedIK homing ! ! !
    if onStartup == True:

        # Calculate the difference after homing
        relaxedik_kinova_diff_pos = np.array([0.0, 0.0, 0.0]) - ee_array[0:3]
        relaxedik_kinova_diff_rot = np.array([0.0, 0.0, 0.0]) - ee_array[3:6]

        # Remove the flag
        onStartup = False   

        # print("EE (pos):", ee_array[0:3])
        # print("Diff (pos):", relaxedik_kinova_diff_pos)
        # print()  

        print("EE (rot):", ee_array[3:6].round(3))
        print("Diff (rot):", relaxedik_kinova_diff_rot.round(3))
        print()  


# Calculates the difference in position between the end effector and the controller
def calculate_controller_ee_diff():
    global input_pos_kcs, input_rot_kcs, ee_array
    global oculus_kinova_diff_pos, oculus_kinova_diff_rot

    # Calculate tranformation matrices
    oculus_kinova_diff_pos[0] =  input_pos_kcs[0] - ee_array[0]
    oculus_kinova_diff_pos[1] =  input_pos_kcs[1] - ee_array[1]
    oculus_kinova_diff_pos[2] =  input_pos_kcs[2] - ee_array[2]

    # input_rot_kcs_euler = T.euler_from_quaternion(input_rot_kcs)
    # oculus_kinova_diff_rot[0] =  input_rot_kcs_euler[0] - ee_array[3]
    # oculus_kinova_diff_rot[1] =  input_rot_kcs_euler[1] - ee_array[4]
    # oculus_kinova_diff_rot[2] =  input_rot_kcs_euler[2] - ee_array[5]

    print("Input (pos):", input_pos_kcs)
    print("EE (pos):", ee_array[3:6])
    print("Diff (pos):", oculus_kinova_diff_pos)
    print()  

    # print("Raw Input:", round(input_pos_kcs[0], 3), round(input_pos_kcs[1], 3), round(input_pos_kcs[2], 3))
    # print("EE:", round(ee_array[0], 3), round(ee_array[1], 3), round(ee_array[2], 3))
    # print("Diff:", round(oculus_kinova_diff[0], 3), round(oculus_kinova_diff[1], 3), round(oculus_kinova_diff[2], 3))
    # print()

 # Gripper control: mode=1 - force, mode=2 - velocity, mode=3 - position
def gripper_control(mode, value):

    # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Finger.msg
    finger = Finger()
    finger.finger_identifier = 0
    finger.value = value

    # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Gripper.msg
    gripper = Gripper()
    gripper.finger.append(finger)

    # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/GripperCommand.msg
    gripper_command = SendGripperCommand()
    gripper_command.mode = mode
    gripper_command.duration = 0
    gripper_command.gripper = gripper

    gripper_command_srv(gripper_command)


# Gripper control state machine
def gripper_sm():
    global gripperState
    global gripperButtonState
    global gripperButtonReleased

    if gripperState == "open" and gripperButtonState and gripperButtonReleased:
        print(1)
        # Change gripper state
        gripperState = "close"

        # Close the gripper
        gripper_control(3, 0.6)

    elif gripperState == "close" and gripperButtonState and gripperButtonReleased:
        print(2)
        # Change gripper state
        gripperState = "open"

        # Open the gripper
        gripper_control(3, 0.0)


if __name__ == '__main__':
    # Variables
    ee_array = np.array([0.0, 0.0, 0.0])
    oculus_kinova_diff_pos = np.array([0.0, 0.0, 0.0])
    oculus_kinova_diff_rot = np.array([0.0, 0.0, 0.0])
    relaxedik_kinova_diff_pos = np.array([0.0, 0.0, 0.0])
    relaxedik_kinova_diff_rot = np.array([0.0, 0.0, 0.0])

    # Flags
    onTrackingStart = True
    onStartup = True
    
    gripperState = "open"
    gripperButtonState = False
    gripperButtonReleased = True

    # Initialize the node
    rospy.init_node("coordinate_converter", anonymous=True)

    # Publishing
    setpoint = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)

    # Service
    pid_setpoint_srv = rospy.ServiceProxy('pid_setpoint', pid_setpoint)
    vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', pid_vel_limit)
    activate_ik_srv = rospy.ServiceProxy('relaxed_ik/activate_ik', activate_ik)
    gripper_command_srv = rospy.ServiceProxy('my_gen3/base/send_gripper_command', SendGripperCommand)


    # Deactivate IK for homing
    activate_ik_srv(False)

    # Set 20% velocity
    vel_limit_srv(0.2)

    # Homing position
    print("\nThe arm is homing...\n")

    pid_setpoint_srv(str(math.radians(0.0)) + " " +
                    str(math.radians(128.0)) + " " +
                    str(math.radians(0.0)) + " " +
                    str(math.radians(0.0)) + " " +
                    str(math.radians(90.0)) + " " +
                    str(math.radians(-90.0)) + " " +
                    str(math.radians(90.0)) + " ")

    rospy.sleep(5)

    print("Homed!\n")

    relaxed_ik_publish([0, 0, 0], [1, 0, 0, 0])

    # Activate IK after homing
    activate_ik_srv(True)

    rospy.sleep(3)

    print("RelaxedIK is initialized!\n") 

    # Set 100% velocity
    vel_limit_srv(1.0)

    # Subscribing
    # rospy.Subscriber("leftHandInfo", HandTracking, left_callback)
    rospy.Subscriber("rightControllerInfo", ControllerInput, right_callback)
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, ee_position_callback)

    while not rospy.is_shutdown():
        pass