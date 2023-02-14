#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ros_tcp_endpoint_msgs.msg import ControllerInput
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
from kortex_driver.msg import *
from kortex_driver.srv import *
import kinova_positional_control.srv as posctrl_srv
from relaxed_ik_ros1.srv import activate_ik
import gopher_ros_clearcore.msg as clearcore_msg
import gopher_ros_clearcore.srv as clearcore_srv

# 0 - manual, 1 - proximity, 2 - scaled
CHEST_CONTROL_MODE = 1


# # # Chest # # #
# Check if the chest goal is reachable
def validgoal(goal):

    max_chest = 440
    min_chest = 0.0

    # if the new chest position will be out of range, return False
    if goal > max_chest:
        return max_chest
    elif goal < min_chest:
        return min_chest
    else:
        return goal


# NOTE: chest position is not real time, has some delay
def chest_position_callback(data):
    global chest_pos

    chest_pos = data.z


# Calculates the difference in position between the chest and the controller
def calculate_controller_chest_diff():
    global oculus_chest_diff, operator_arm_boundary
    global isInitialized, onStartup

    chest_max_pos = 440

    # Chest scaled motion mapping
    if CHEST_CONTROL_MODE == 2:
        # Calculate the difference based on maximum positions
        oculus_chest_diff = round(operator_arm_boundary['max'] * 1000, 1) - chest_max_pos


# Runs a callibration procedure do define user arm boundaries (for scaling)
def calibrate_arm_boundaries_sm():
    global right_controller, operator_arm_boundary
    global calibrate_arm_boundaries_state

    # To reduce arm stretching
    boundary_offset = 0.2

    # Start calibration
    if calibrate_arm_boundaries_state == 0:
        print("\nMove your arm straight DOWN, then press the gripButton.")

        # Change state
        calibrate_arm_boundaries_state = 1

        return False
    
    # Press calibration button
    elif calibrate_arm_boundaries_state == 1 and right_controller['gripButton'] == True:
        # Calibrate minimum
        operator_arm_boundary['min'] = right_controller['controller_pos_y'] + boundary_offset

        print("\nMinimum is calibrated to", round(operator_arm_boundary['min'], 3))
        print("\nMove your arm straight UP, then press the gripButton.")

        # Change state
        calibrate_arm_boundaries_state = 2

        return False

    # Release calibration button
    elif calibrate_arm_boundaries_state == 2 and right_controller['gripButton'] == False:

        # Change state
        calibrate_arm_boundaries_state = 3

        return False

    # Press calibration button
    elif calibrate_arm_boundaries_state == 3 and right_controller['gripButton'] == True:
        # Calibrate maximum
        operator_arm_boundary['max'] = right_controller['controller_pos_y'] - boundary_offset

        print("\nMaximum is calibrated to", round(operator_arm_boundary['max'], 3))

        # Change state
        calibrate_arm_boundaries_state = 4

        return False

    # Release calibration button
    elif calibrate_arm_boundaries_state == 4 and right_controller['gripButton'] == False:

        return True


# Implements different chest mapping methods
def chest_mapping():
    global right_controller, input_pos_gcs, chest_pos, goal_chest, operator_arm_boundary
    global onTrackingStart, isInitialized
    global onLowerLimit, onHigherLimit, GoalSet

    if isInitialized:
        # Chest manual joystick control
        if CHEST_CONTROL_MODE == 0:
            if abs(right_controller['joystick_pos_y']) > 0.05:
                chest_vel = np.interp(round(right_controller['joystick_pos_y'], 4), [-1.0, 1.0],[-0.8, 0.8])

            else:
                chest_vel = 0.0
            
            # Publish chest velocity
            msg = geom_msgs.Twist()
            msg.linear.z = chest_vel
            chest_vel_pub.publish(msg)


        elif CHEST_CONTROL_MODE == 1:

            chest_increment = 220
            upper_limit = relaxed_ik_boundary['z_max'] - 0.2
            lower_limit = relaxed_ik_boundary['z_min'] + 0.2

            ee_z_global = relaxed_ik_pos_gcs[2]

            # Start tracking if gripButton was pressed
            if right_controller['gripButton'] == True:
                
                # If end-effector is close to highest boundary of reachability
                if ee_z_global > upper_limit and not GoalSet and chest_pos != 440:
                    
                    # Set higher limit and new goal flag to true 
                    onHigherLimit = True
                    GoalSet = True   

                    # Set the new goal and publish new chest position
                    goal_chest = validgoal(chest_pos + chest_increment)     

                    msg = clearcore_msg.Position()
                    msg.position = validgoal(goal_chest)
                    msg.velocity = 1.0
                    chest_abspos_pub.publish(msg)


                # If end-effector is close to lowest boundary of reachability
                elif ee_z_global < lower_limit and not GoalSet and chest_pos != 0:
                    
                    # Set higher limit and new goal flag to true 
                    onLowerLimit = True
                    GoalSet = True       

                    goal_chest = validgoal(chest_pos - chest_increment)

                    msg = clearcore_msg.Position()
                    msg.position = validgoal(goal_chest)
                    msg.velocity = 1.0
                    chest_abspos_pub.publish(msg)

                elif abs(chest_pos - goal_chest) <= 1:
                    
                    if  onHigherLimit:
                        onHigherLimit = False
                        GoalSet = False

                    elif onLowerLimit:
                        onLowerLimit = False
                        GoalSet = False

                    calculate_controller_ee_diff()

            # Stop tracking if gripButton was released
            else:
                # Set the flag
                onTrackingStart['chest'] = True

                # Stop chest
                msg = geom_msgs.Twist()
                msg.linear.z = 0.0
                chest_vel_pub.publish(msg)

                # Reset flags
                onHigherLimit = False
                onLowerLimit = False
                GoalSet = False  


        # Chest scaled motion mapping
        elif CHEST_CONTROL_MODE == 2:

            # Start tracking if gripButton was pressed
            if right_controller['gripButton'] == True:

                # Map (scale) controller position within user arm motion range onto chest motion range
                scaled_chest_pos = np.interp(input_pos_gcs[2] * 1000 - oculus_chest_diff, 
                                            [operator_arm_boundary['min'] * 1000 - oculus_chest_diff, operator_arm_boundary['max'] * 1000 - oculus_chest_diff], 
                                            [0, 440])

                # Safety feature: only start tracking if controller mapped Z coordinate is within 10 mm of chest current Z coordinate (because of absolute coordinates)
                if onTrackingStart['chest'] == True and abs(chest_pos - scaled_chest_pos) < 10:
                    # Reset the flag
                    onTrackingStart['chest'] = False

                # Tracking has started
                elif onTrackingStart['chest'] == False:
                    msg = clearcore_msg.Position()
                    msg.position = scaled_chest_pos
                    msg.velocity = 1.0
                    chest_abspos_pub.publish(msg)


            # Stop tracking if gripButton was released
            else:
                # Set the flag
                onTrackingStart['chest'] = True

                # Stop chest
                msg = geom_msgs.Twist()
                msg.linear.z = 0.0
                chest_vel_pub.publish(msg)


# # # Kinova # # #
# Transition from Global CS to Kinova CS
def calculate_gcs_kcs_trans():
    global R_kcs_to_gcs, R_gcs_to_kcs

    # Rotate around y and z axis
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = T.rotation_matrix(math.radians(0.0), xaxis)
    Ry = T.rotation_matrix(math.radians(-45.0), yaxis)
    Rz = T.rotation_matrix(math.radians(90.0), zaxis)

    R_gcs_to_kcs = T.concatenate_matrices(Rx, Ry, Rz)
    R_kcs_to_gcs = T.inverse_matrix(R_gcs_to_kcs)


def kinova_mapping():
    global R_gcs_to_kcs, R_kcs_to_gcs
    global input_pos_gcs, relaxed_ik_pos_gcs

    global right_controller, operator_arm_boundary
    global oculus_kinova_pos_diff_gcs, relaxedik_kinova_pos_diff_kcs
    global onTrackingStart, isInitialized
    global gripperButtonState, gripperButtonReleased 
    global onLowerLimit, onHigherLimit, z_diff, prev_z

    if isInitialized:
        
        # Start tracking if gripButton was pressed
        if right_controller['gripButton'] == True: 
            # # POSITION
            # Transition from Left-handed CS (Unity) to Right-handed CS (Global): swap y and z axis
            # Then swap x and new y (which was z) to have x facing forward
            # Negate new y (which is x) to make it align with a global coordinate system
            input_pos_gcs = np.array([-1 * right_controller['controller_pos_z'], right_controller['controller_pos_x'], right_controller['controller_pos_y']])

            # Chest manual joystick control
            if CHEST_CONTROL_MODE == 0:
                # On first press recalculate transition (difference) from oculus to 
                if onTrackingStart['right_arm'] == True:
                    calculate_controller_ee_diff()        

                    # Remove the flag
                    onTrackingStart['right_arm'] = False

                # Compesantion for controller and GCS misalignment
                relaxed_ik_pos_gcs = input_pos_gcs - oculus_kinova_pos_diff_gcs

                relaxed_ik_pub_target_gcs(relaxed_ik_pos_gcs, [1, 0, 0, 0])


            elif CHEST_CONTROL_MODE == 1:
                if onTrackingStart['right_arm'] == True:
                    calculate_controller_ee_diff()        

                    # Remove the flag
                    onTrackingStart['right_arm'] = False
                
                # If end-effector is close to lower or higher boundary
                if onHigherLimit or onLowerLimit:
                    
                    if abs(prev_z - chest_pos) > 1:
                        
                        # Calculate difference between previous and current chest position
                        # And convert to meters
                        delta = abs(prev_z - chest_pos) / 1000

                        # Add compensation for the end-effector
                        if onHigherLimit: z_diff[2] = z_diff[2] - delta
                        elif onLowerLimit: z_diff[2] = z_diff[2] + delta

                        # Publish new position to relaxedIK taking into account compensation 
                        # for the end-effector (e.g. how much the chest has moved up or down)
                        relaxed_ik_pos_gcs = input_pos_gcs + z_diff - oculus_kinova_pos_diff_gcs
                        relaxed_ik_pub_target_gcs(relaxed_ik_pos_gcs, [1, 0, 0, 0])

                        prev_z = chest_pos

                elif not onLowerLimit and not onHigherLimit:

                    relaxed_ik_pos_gcs = input_pos_gcs + z_diff - oculus_kinova_pos_diff_gcs
                    relaxed_ik_pub_target_gcs(relaxed_ik_pos_gcs, [1, 0, 0, 0])

            # Chest scaled motion mapping
            elif CHEST_CONTROL_MODE == 2:
                # Map (scale) controller position within user arm motion range onto Kinova motion range
                scaled_z = np.interp(input_pos_gcs[2], 
                                    [operator_arm_boundary['min'], operator_arm_boundary['max']], 
                                    [relaxed_ik_boundary['z_min'], relaxed_ik_boundary['z_max']])

                # Safety feature: only start tracking if controller mapped Z coordinate is within 0.05 m of Kinova current Z coordinate (because of absolute coordinates)
                if onTrackingStart['right_arm'] == True and abs(relaxed_ik_pos_gcs[2] - scaled_z) < 0.05:
                    calculate_controller_ee_diff()        

                    # Remove the flag
                    onTrackingStart['right_arm'] = False 

                # Tracking has started
                elif onTrackingStart['right_arm'] == False:

                    # Compesantion for controller and GCS misalignment
                    relaxed_ik_pos_gcs[0:2] = input_pos_gcs[0:2] - oculus_kinova_pos_diff_gcs[0:2]

                    # Update Z with new scaled Z
                    relaxed_ik_pos_gcs[2] = scaled_z

                    relaxed_ik_pub_target_gcs(relaxed_ik_pos_gcs, [1, 0, 0, 0])

        # Stop tracking if gripButton was released
        else:
            # Reset the flag
            onTrackingStart['right_arm'] = True

            # TODO: stop any robot motion

        # Gripper
        if right_controller['triggerButton']:
            gripperButtonState = True
            
            # Call gripper state machine
            gripper_sm()

            gripperButtonReleased = False

        else:
            gripperButtonState = False
            
            # Call gripper state machine
            gripper_sm()

            gripperButtonReleased = True


# Publish target position and orientation in World Coordinate system (same as Global CS, but Z contains a chest height)
def relaxed_ik_pub_target_wcs(target_position_wcs, target_orientation_wcs):
    global R_gcs_to_kcs, R_kcs_to_gcs
    global relaxed_ik_pos_gcs, chest_pos

    # Update relaxed_ik global variable
    relaxed_ik_pos_gcs = target_position_wcs.copy()
    relaxed_ik_pos_gcs[2] = target_position_wcs[2] - chest_pos / 1000

    # Recalculate into relaxed IK CS
    relaxed_ik_pos_rikcs = np.matmul(R_gcs_to_kcs[0:3,0:3], target_position_wcs)

    # TODO: orientation
    relaxed_ik_pub_target_rikcs(relaxed_ik_pos_rikcs, target_orientation_wcs)

    # Publish commanded relaxed_ik in world coordinates
    relaxed_ik_pub_commanded_wcs(target_position_wcs, target_orientation_wcs)


# Publish last commanded target position and orientation in a World Coordinate System
def relaxed_ik_pub_commanded_wcs(current_position_wcs, current_orientation_wcs):

    # Combine into a single array
    msg = Float32MultiArray()
    msg.data = current_position_wcs.tolist() + current_orientation_wcs

    # Publish
    relaxed_ik_target_wcs_pub.publish(msg)


# Publish target position and orientation in Global Coordinate system (parallel to the ground: X - forw, Y - left, Z - up)
def relaxed_ik_pub_target_gcs(target_position_gcs, target_orientation_gcs):
    global R_gcs_to_kcs, R_kcs_to_gcs
    global relaxed_ik_pos_gcs, chest_pos
    global relaxed_ik_boundary

    # Update relaxed_ik global variable
    relaxed_ik_pos_gcs = target_position_gcs.copy()

    # Apply limits
    if relaxed_ik_pos_gcs[0] < relaxed_ik_boundary['x_min']:
        relaxed_ik_pos_gcs[0] = relaxed_ik_boundary['x_min']

        calculate_controller_ee_diff()

    elif relaxed_ik_pos_gcs[0] > relaxed_ik_boundary['x_max']:
        relaxed_ik_pos_gcs[0] = relaxed_ik_boundary['x_max']

        calculate_controller_ee_diff()

    if relaxed_ik_pos_gcs[1] < relaxed_ik_boundary['y_min']:
        relaxed_ik_pos_gcs[1] = relaxed_ik_boundary['y_min']

        calculate_controller_ee_diff()

    elif relaxed_ik_pos_gcs[1] > relaxed_ik_boundary['y_max']:
        relaxed_ik_pos_gcs[1] = relaxed_ik_boundary['y_max']

        calculate_controller_ee_diff()

    if relaxed_ik_pos_gcs[2] < relaxed_ik_boundary['z_min']:
        relaxed_ik_pos_gcs[2] = relaxed_ik_boundary['z_min']

        calculate_controller_ee_diff()

    elif relaxed_ik_pos_gcs[2] > relaxed_ik_boundary['z_max']:
        relaxed_ik_pos_gcs[2] = relaxed_ik_boundary['z_max']

        calculate_controller_ee_diff()

    # Recalculate into relaxed IK CS
    relaxed_ik_pos_rikcs = np.matmul(R_gcs_to_kcs[0:3,0:3], target_position_gcs)

    # TODO: orientation
    relaxed_ik_pub_target_rikcs(relaxed_ik_pos_rikcs, target_orientation_gcs)

    # Add chest height to Z for WCS
    relaxed_ik_pos_wcs = target_position_gcs.copy()
    relaxed_ik_pos_wcs[2] = relaxed_ik_pos_wcs[2] + chest_pos / 1000

    relaxed_ik_pub_commanded_wcs(relaxed_ik_pos_wcs, target_orientation_gcs)


# Publish target position and orientation in relaxed IK Coordinate system (Kinova)
def relaxed_ik_pub_target_rikcs(target_position, target_orientation):

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

    # Publish
    setpoint_pub.publish(ee_pose_goals)


# Right controller topic callback function
def right_callback(data):
    global right_controller

    # Update dictionary
    right_controller['primaryButton'] = data.primaryButton
    right_controller['secondaryButton'] = data.secondaryButton
    right_controller['triggerButton'] = data.triggerButton
    right_controller['triggerValue'] = data.triggerValue
    right_controller['gripButton'] = data.gripButton
    right_controller['joystickButton'] = data.joystickButton
    right_controller['joystick_pos_x'] = data.joystick_pos_x
    right_controller['joystick_pos_y'] = data.joystick_pos_y
    right_controller['controller_pos_x'] = data.controller_pos_x
    right_controller['controller_pos_y'] = data.controller_pos_y
    right_controller['controller_pos_z'] = data.controller_pos_z
    right_controller['controller_rot_w'] = data.controller_rot_w
    right_controller['controller_rot_x'] = data.controller_rot_x
    right_controller['controller_rot_y'] = data.controller_rot_y
    right_controller['controller_rot_z'] = data.controller_rot_z


# Calculates the difference between the end effector (relaxed_IK) and the controller coordinates
def calculate_controller_ee_diff():
    global input_pos_gcs, relaxed_ik_pos_gcs, oculus_kinova_pos_diff_gcs  
    global isInitialized, z_diff

    z_diff = np.array([0.0, 0.0, 0.0])

    if isInitialized:
        # Calculate tranformation matrices
        oculus_kinova_pos_diff_gcs =  input_pos_gcs - relaxed_ik_pos_gcs


# Callback function that updates motionFinished flag
def pid_motion_finished_callback(data):
    global motionFinished

    motionFinished = data.data


# Block a code execution until a motionFinished flag is set or a node is shutdown
def wait_motion_finished():
    global motionFinished

    # Allow a motion to start
    rospy.sleep(1)

    # Block the code execution
    while motionFinished != True and not rospy.is_shutdown():
        pass


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

    if isInitialized:
        if gripperState == "open" and gripperButtonState and gripperButtonReleased:
            # Change gripper state
            gripperState = "close"

            # Close the gripper
            gripper_control(3, 0.6)

        elif gripperState == "close" and gripperButtonState and gripperButtonReleased:
            # Change gripper state
            gripperState = "open"

            # Open the gripper
            gripper_control(3, 0.0)


# Callback function that subscribes to the end effector feedback
def ee_callback(data):
    global R_gcs_to_kcs, R_kcs_to_gcs
    global kinova_pos_gcs, kinova_pos_kcs, relaxedik_kinova_pos_diff_kcs  
    global onStartup, isInitialized

    if isInitialized:

        # TODO: Add orientation
        kinova_pos_kcs = np.array([data.base.tool_pose_x, 
                            data.base.tool_pose_y, 
                            data.base.tool_pose_z])

        # In global coordinate system
        kinova_pos_gcs = np.matmul(R_kcs_to_gcs[0:3,0:3], kinova_pos_kcs)
  
        if onStartup['right_arm'] == True:

            # Calculate a transition from relaxedIK initial absolute postion (0, 0, 0) to KinovaFK on startup
            relaxedik_kinova_pos_diff_kcs = np.array([0.0, 0.0, 0.0]) - kinova_pos_kcs

            # Remove the flag
            onStartup['right_arm'] = False   


# This function is called when the node is shutting down
def node_shutdown():
    print("\nNode is shutting down...")

    # Stop arm motion
    stop_arm_srv()

    # Stop chest motion
    chest_stop_srv()

    # Deactivate chest feedback
    chest_logger_srv(False)

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("coordinate_converter", anonymous=True)
    rospy.on_shutdown(node_shutdown)

    # Variables
    isInitialized = False
    onHigherLimit = False
    onLowerLimit = False
    GoalSet = False

    kinova_pos_gcs = np.array([0.0, 0.0, 0.0])
    kinova_pos_kcs = np.array([0.0, 0.0, 0.0])
    relaxed_ik_pos_gcs = np.array([0.0, 0.0, 0.0])

    oculus_kinova_pos_diff_gcs = np.array([0.0, 0.0, 0.0])
    relaxedik_kinova_pos_diff_kcs = np.array([0.0, 0.0, 0.0])

    right_controller = {'primaryButton': False, 
                        'secondaryButton': False,
                        'triggerButton': False,
                        'triggerValue': 0.0,
                        'gripButton': False,
                        'joystickButton': False,
                        'joystick_pos_x': 0.0, 'joystick_pos_y': 0.0,
                        'controller_pos_x': 0.0, 'controller_pos_y': 0.0, 'controller_pos_z': 0.0,
                        'controller_rot_w': 0.0, 'controller_rot_x': 0.0, 'controller_rot_y': 0.0, 'controller_rot_z': 0.0   
                        }

    operator_arm_boundary = {'min': 1.03, 'max': 1.73}
    calibrate_arm_boundaries_state = 0

    relaxed_ik_boundary = {"x_min": -0.3, "x_max": 0.3, "y_min": -np.inf, "y_max": np.inf, "z_min": 0.0, "z_max": 1.3}

    # Chest
    chest_vel = 0.0
    chest_pos = 440.0
    goal_chest = 440.0
    prev_z = 220.0
    oculus_chest_diff = 0.0 

    # Flags
    onTrackingStart = {'right_arm': True, 'chest': True}
    onStartup = {'right_arm': True, 'chest': True}
    
    gripperState = "open"
    gripperButtonState = False
    gripperButtonReleased = True
    gripperValue = 0.0

    motionFinished = False

    # Publishing
    setpoint_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
    relaxed_ik_target_wcs_pub = rospy.Publisher('/relaxed_ik/position_wcs', Float32MultiArray, queue_size=1)
    chest_vel_pub = rospy.Publisher('z_chest_vel', geom_msgs.Twist, queue_size=1)
    chest_abspos_pub = rospy.Publisher('/z_chest_pos', clearcore_msg.Position, queue_size=1)
    
    # Subscribing
    rospy.Subscriber('/pid/motion_finished', Bool, pid_motion_finished_callback)
    rospy.Subscriber('/chest_position', geom_msgs.Point, chest_position_callback)
    rospy.Subscriber("rightControllerInfo", ControllerInput, right_callback)
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, ee_callback)

    # Service
    pid_setpoint_srv = rospy.ServiceProxy('pid_setpoint', posctrl_srv.pid_setpoint)
    pid_vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', posctrl_srv.pid_vel_limit)

    activate_ik_srv = rospy.ServiceProxy('relaxed_ik/activate_ik', activate_ik)

    gripper_command_srv = rospy.ServiceProxy('my_gen3/base/send_gripper_command', SendGripperCommand)
    stop_arm_srv = rospy.ServiceProxy('my_gen3/base/stop', Stop)

    chest_homing_srv = rospy.ServiceProxy('z_chest_home', clearcore_srv.Homing)
    chest_stop_srv = rospy.ServiceProxy('z_chest_stop', clearcore_srv.Stop)
    chest_abspos_srv = rospy.ServiceProxy('z_chest_abspos', clearcore_srv.AbsolutePosition)
    chest_logger_srv = rospy.ServiceProxy('z_chest_logger', clearcore_srv.LoggerControl)

    # Home the chest
    chest_homing_srv(True)

    # Set 30% velocity
    pid_vel_limit_srv(0.3)

    # Homing position
    print("\nHoming has started...\n")

    # Let the node initialized
    rospy.sleep(1)

    # Home using relaxedIK
    relaxed_ik_pub_target_rikcs([0, 0, 0], [1, 0, 0, 0])

    # Block until the motion is finished
    wait_motion_finished()

    print("Homing has finished.\n") 

    # Open the gripper
    gripper_control(3, 0.0)

    # Calculate a transition between Global CS and Kinova CS
    calculate_gcs_kcs_trans()

    # Chest scaled motion mapping
    if CHEST_CONTROL_MODE == 2:
        # Operator arm boundary calibration
        while calibrate_arm_boundaries_sm() != True and not rospy.is_shutdown():
            pass

        # Controller to chest position calibration
        calculate_controller_chest_diff()

        print("\nMoving to the starting position...\n") 

        # Move the chest to middle position
        chest_abspos_srv(220, 1.0)

    if CHEST_CONTROL_MODE == 1 or CHEST_CONTROL_MODE == 2:

        # Move the chest to middle position
        chest_abspos_srv(220, 1.0)

        # Move Kinova to the middle position
        relaxed_ik_pub_target_gcs(np.array([0, -0.1, 0.5]), [1, 0, 0, 0])

        # Block until the motion is finished
        wait_motion_finished()

    # Set 100% velocity
    pid_vel_limit_srv(1.0)

    # Activate chest feedback
    chest_logger_srv(True)

    # Set the flag and finish initialization
    isInitialized = True

    print("System is ready.\n") 

    # Main loop
    while not rospy.is_shutdown():
        chest_mapping()
        kinova_mapping()
