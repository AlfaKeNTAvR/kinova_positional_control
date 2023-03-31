#!/usr/bin/env python
# yapf: disable
import rospy
import roslaunch
import transformations as T
import numpy as np
import math
import argparse

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

parser = argparse.ArgumentParser(description='Process some integers.')
parser.add_argument('-m', '--mode', action='store', default=0, help='chest control mode: 0 - manual, 1 - auto: proximity, 2 - auto: scaling')
args = vars(parser.parse_args())

# 0 - manual, 1 - proximity, 2 - scaling
CHEST_CONTROL_MODE = int(args['mode'])

operator_arm_boundary = {'min': 1.03, 'max': 1.73}
relaxed_ik_boundary = {"x_min": -np.inf, "x_max": 0.4, "y_min": -np.inf, "y_max": np.inf, "z_min": 0.0, "z_max": 1.4}


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
    global right_controller, input_pos_gcs, chest_pos, goal_chest, operator_arm_boundary, relaxed_ik_pos_gcs
    global onTrackingStart, isInitialized
    global onLowerLimit, onHigherLimit, GoalSet, aut_activation_prox, aut_bool
    global isTracking


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

            ee_z_global = relaxed_ik_pos_gcs[2].copy()

            # Start tracking if gripButton was pressed
            if isTracking == True:

                # If end-effector is close to highest boundary of reachability
                if ee_z_global > upper_limit and not GoalSet and abs(chest_pos - 440) > 1:

                    # Autonomy activated increment
                    aut_activation_prox += 1
                    aut_bool = True

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
                elif ee_z_global < lower_limit and not GoalSet and chest_pos > 1:

                    #Autonomy activated increment
                    aut_bool = True

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

                    aut_bool = False


            # Stop tracking if gripButton was released
            else:
                # Set the flag
                onTrackingStart['chest'] = True

                # Stop chest
                msg = geom_msgs.Twist()
                msg.linear.z = 0.0
                chest_vel_pub.publish(msg)

                # Reset flags
                aut_bool = False
                onHigherLimit = False
                onLowerLimit = False
                GoalSet = False

            autonomy_prox = Bool()
            autonomy_prox.data = aut_bool
            autonomy_prox_pub.publish(autonomy_prox)


        # Chest scaled motion mapping
        elif CHEST_CONTROL_MODE == 2:

            # Start tracking if tracking mode was activated
            if isTracking == True:

                # Map (scale) controller position within user arm motion range onto chest motion range
                scaled_chest_pos = np.interp(input_pos_gcs[2] * 1000 - oculus_chest_diff,
                                            [operator_arm_boundary['min'] * 1000 - oculus_chest_diff, operator_arm_boundary['max'] * 1000 - oculus_chest_diff],
                                            [0, 440])

                # Safety feature: only start tracking if controller mapped Z coordinate is within 20 mm of chest current Z coordinate (because of absolute coordinates)
                if onTrackingStart['chest'] == True and abs(chest_pos - scaled_chest_pos) < 20:
                    calculate_controller_ee_diff()

                    # Remove flags
                    onTrackingStart['right_arm'] = False
                    onTrackingStart['chest'] = False

                # Tracking has started
                elif onTrackingStart['chest'] == False:
                    msg = clearcore_msg.Position()
                    msg.position = scaled_chest_pos
                    msg.velocity = 0.8
                    chest_abspos_pub.publish(msg)


            # Stop tracking if tracking mode was deactivated
            else:
                # Set flags
                onTrackingStart['right_arm'] = True
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
    Ry = T.rotation_matrix(math.radians(-48.2), yaxis)
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
    global isTracking

    if isInitialized:

        # Tracking has started
        if isTracking == True:

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

                # Tracking has started
                if onTrackingStart['right_arm'] == False:
                    # Map (scale) controller position within user arm motion range onto Kinova motion range
                    scaled_z = np.interp(input_pos_gcs[2],
                                        [operator_arm_boundary['min'], operator_arm_boundary['max']],
                                        [relaxed_ik_boundary['z_min'], relaxed_ik_boundary['z_max']])

                    # Compesantion for controller and GCS misalignment
                    relaxed_ik_pos_gcs[0:2] = input_pos_gcs[0:2] - oculus_kinova_pos_diff_gcs[0:2]

                    # Scale X and Y
                    relaxed_ik_pos_gcs[0] = relaxed_ik_pos_gcs[0] * 1.0
                    relaxed_ik_pos_gcs[1] = relaxed_ik_pos_gcs[1] * 1.0

                    # Update Z with new scaled Z
                    relaxed_ik_pos_gcs[2] = scaled_z

                    relaxed_ik_pub_target_gcs(relaxed_ik_pos_gcs, [1, 0, 0, 0])

        # If tracking has stopped
        else:
            # Reset the flag
            if CHEST_CONTROL_MODE != 2:
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


        # Call pick and place autonomy state machine
        pick_place_autonomy_sm()

        # Combine into a single array
        msg = Float32MultiArray()
        msg.data = relaxed_ik_pos_gcs.tolist() + [1, 0, 0, 0]

        # Publish
        relaxed_ik_target_gcs_pub.publish(msg)

        # Add chest height to Z for WCS
        relaxed_ik_pos_wcs = relaxed_ik_pos_gcs.copy()
        relaxed_ik_pos_wcs[2] = relaxed_ik_pos_wcs[2] + chest_pos / 1000

        relaxed_ik_pub_commanded_wcs(relaxed_ik_pos_wcs, [1, 0, 0, 0])

# Publish target position and orientation in World Coordinate system (same as Global CS, but Z contains a chest height)
def relaxed_ik_pub_target_wcs(target_position_wcs, target_orientation_wcs):
    global R_gcs_to_kcs, R_kcs_to_gcs
    global relaxed_ik_pos_gcs, chest_pos

    # Update relaxed_ik global variable
    relaxed_ik_pos_gcs = target_position_wcs.copy()
    relaxed_ik_pos_gcs[2] = target_position_wcs[2] - chest_pos / 1000

    # Recalculate into relaxed IK CS
    relaxed_ik_pos_rikcs = np.matmul(R_gcs_to_kcs[0:3,0:3], relaxed_ik_pos_gcs)

    # TODO: orientation
    relaxed_ik_pub_target_rikcs(relaxed_ik_pos_rikcs, target_orientation_wcs)


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
    relaxed_ik_pos_rikcs = np.matmul(R_gcs_to_kcs[0:3,0:3], relaxed_ik_pos_gcs)

    # TODO: orientation
    relaxed_ik_pub_target_rikcs(relaxed_ik_pos_rikcs, target_orientation_gcs)


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
    global right_controller, input_pos_gcs
    global isTracking, isTracking_state

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

    # Publish calibration parameters
    msg = Float32MultiArray()
    msg.data = [operator_arm_boundary['max'], operator_arm_boundary['min']]
    calibration_pub.publish(msg)

    # Transition from Left-handed CS (Unity) to Right-handed CS (Global): swap y and z axis
    # Then swap x and new y (which was z) to have x facing forward
    # Negate new y (which is x) to make it align with a global coordinate system
    input_pos_gcs = np.array([-1 * right_controller['controller_pos_z'], right_controller['controller_pos_x'], right_controller['controller_pos_y']])

    # Pressed
    if isTracking_state == 0 and right_controller['gripButton'] == True:

        isTracking_state = 1

    # Released
    elif isTracking_state == 1 and right_controller['gripButton'] == False:

        # Activate a tracking mode
        isTracking = True
        isTracking_state = 2
        if isInitialized: print("\nTracking is ON.\n")

    # Pressed
    elif isTracking_state == 2 and right_controller['gripButton'] == True:

        isTracking_state = 3

    # Released
    elif isTracking_state == 3 and right_controller['gripButton'] == False:

        # Deactivate a tracking mode
        isTracking = False
        isTracking_state = 0
        if isInitialized: print("\nTracking is OFF.\n")

    # Publish calibration parameters
    msg = Bool()
    msg.data = isTracking
    tracking_pub.publish(msg)

    # HACK 1
    global reachability, pick_place
    msg1 = Int32()
    msg1.data = pick_place
    pick_place_pub.publish(msg1)

    msg2 = Int32()
    msg2.data = reachability
    reachability_pub.publish(msg2)

    # HACK 2
    global relaxed_ik_pos_gcs
    # Combine into a single array
    msg = Float32MultiArray()
    msg.data = relaxed_ik_pos_gcs.tolist() + [1, 0, 0, 0]

    # Publish
    relaxed_ik_target_gcs_pub.publish(msg)

    # Add chest height to Z for WCS
    relaxed_ik_pos_wcs = relaxed_ik_pos_gcs.copy()
    relaxed_ik_pos_wcs[2] = relaxed_ik_pos_wcs[2] + chest_pos / 1000

    relaxed_ik_pub_commanded_wcs(relaxed_ik_pos_wcs, [1, 0, 0, 0])

    # HACK 3
    if CHEST_CONTROL_MODE == 1:
        global aut_activation_prox, aut_bool

        autonomy_prox = Bool()
        autonomy_prox.data = aut_bool
        autonomy_prox_pub.publish(autonomy_prox)


# Left controller topic callback function
def left_callback(data):
    global left_controller
    global isInitialized, isTracking, isTracking_state
    global initButton_state, estopButton_state, clearfaultButton_state
    global pid_launch, relaxed_ik_launch

    # Update dictionary
    left_controller['primaryButton'] = data.primaryButton
    left_controller['secondaryButton'] = data.secondaryButton
    left_controller['triggerButton'] = data.triggerButton
    left_controller['triggerValue'] = data.triggerValue
    left_controller['gripButton'] = data.gripButton
    left_controller['joystickButton'] = data.joystickButton
    left_controller['joystick_pos_x'] = data.joystick_pos_x
    left_controller['joystick_pos_y'] = data.joystick_pos_y
    left_controller['controller_pos_x'] = data.controller_pos_x
    left_controller['controller_pos_y'] = data.controller_pos_y
    left_controller['controller_pos_z'] = data.controller_pos_z
    left_controller['controller_rot_w'] = data.controller_rot_w
    left_controller['controller_rot_x'] = data.controller_rot_x
    left_controller['controller_rot_y'] = data.controller_rot_y
    left_controller['controller_rot_z'] = data.controller_rot_z

    # Emergency stop
    if left_controller['gripButton'] == True and estopButton_state == False:
        e_stop()

        print("\n E-Stop! Shutting down relaxed IK and PID...\n")

        # Shutdown relaxed_IK
        relaxed_ik_launch.shutdown()

        # Shutdown PID nodes
        pid_launch.shutdown()

        print("\n E-Stop! Relaxed IK and PID have shutdown.\n")

        # Reset the flag
        isTracking = False
        isTracking_state = 0

        # Change the state
        estopButton_state = True

    elif left_controller['gripButton'] == False and estopButton_state == True:
        # Change the state
        estopButton_state = False


    # Clear arm faults
    if left_controller['secondaryButton'] == True and clearfaultButton_state == 0:
        clearfaults_arm_srv()

        # Reset the flag
        isInitialized = False

        # Change the state
        clearfaultButton_state = 1

        print("\n Faults cleared, restore an arm position.\n")

    elif left_controller['secondaryButton'] == False and clearfaultButton_state == 1:

        # Change the state
        clearfaultButton_state = 2

    elif left_controller['secondaryButton'] == True and clearfaultButton_state == 2:

        # Change the state
        clearfaultButton_state = 3

    elif left_controller['secondaryButton'] == False and clearfaultButton_state == 3:

        # Change the state
        clearfaultButton_state = 0


    # Initialize the robot, home chest and arm
    if left_controller['primaryButton'] == True and initButton_state == False:

        print("\n Initialization, starting relaxed IK and PID...\n")

        # Start the relaxed IK
        roslaunch_relaxed_ik()

        # Launch PID nodes
        roslaunch_pid()

        rospy.sleep(5)

        print("\n Initialization, relaxed IK and PID have started. Homing...\n")

        # Initialize the robot
        initialization()

        # Change the state
        initButton_state = True

    elif left_controller['primaryButton'] == False and initButton_state == True:
        # Change the state
        initButton_state = False


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
            gripper_control(3, 0.7)

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


# Samples and executes trajectory based on the target goal in WCS and duration
def trajectory_sampler(target_pos_wcs, duration):
    global relaxed_ik_pos_gcs, chest_pos
    global pp_sm_state

    # Sampling and executing frequency
    loop_frequency = 100

    # Convert a current position from GCS to WCS
    relaxed_ik_pos_wcs = relaxed_ik_pos_gcs.copy()
    relaxed_ik_pos_wcs[2] = relaxed_ik_pos_wcs[2] + chest_pos / 1000

    num_samples = int(loop_frequency * duration)
    t = np.linspace(0, duration, num_samples)
    samples = np.zeros((num_samples, 3))

    # NOTE: Blocking code
    for i in range(num_samples):
        # Generate a sample
        samples[i, :] = relaxed_ik_pos_wcs + (target_pos_wcs - relaxed_ik_pos_wcs) * (1 - np.cos(np.pi * t[i] / duration)) / 2

        relaxed_ik_pub_target_wcs(samples[i], [1, 0, 0, 0])

        rospy.Rate(loop_frequency).sleep()

        if rospy.is_shutdown(): break


def generate_grid(rows, columns, bottom_right_x, bottom_right_y, row_distances, column_distances):
    # Generate x coordinates
    x_coordinates = np.linspace(bottom_right_x, bottom_right_x + sum(column_distances), columns)[::-1]

    # Generate y coordinates
    y_coordinates = np.linspace(bottom_right_y, bottom_right_y + sum(row_distances), rows)[::-1]

    # Generate a grid of x and y coordinates using numpy's meshgrid function
    x_coordinates, y_coordinates = np.meshgrid(x_coordinates, y_coordinates)

    # Combine the x and y coordinates into a single grid
    grid = np.stack((x_coordinates, y_coordinates), axis=-1)

    return grid


def find_closest_point(x, y, grid):
    # Calculate the Euclidean distance between each point in the grid and the input x, y coordinates
    distances = np.sqrt((grid[..., 0] - x)**2 + (grid[..., 1] - y)**2)

    # Find the index of the point with the minimum distance
    row, col = np.unravel_index(np.argmin(distances), distances.shape)

    return row, col


def pick_place_autonomy_sm():
    global pp_sm_state, gripperState, right_controller, relaxed_ik_boundary
    global shelf_grid, chest_pos, relaxed_ik_pos_gcs
    global isTracking, isTracking_state
    global reachability, pick_place

    # NOTE: Greater than Max boundary because of left most, bottom position
    z_reach_min_bound = 0.27
    z_reach_max_bound = 0.15

    reachability = -1
    pick_place = -1

    x_pick = 0.53
    x_place = 0.54

    # Controlled manually, waiting for intent autonomy activation
    if pp_sm_state == "manual" and right_controller['primaryButton'] == True:

        isTracking = False
        isTracking_state = 0

        pp_sm_state = "intent"

        # Publish -1 when not autonomy and not reachability intent
        reachability = -1
        pick_place = -1

    # Activate intent autonomy
    elif pp_sm_state == "intent" and right_controller['primaryButton'] == False:

        y = relaxed_ik_pos_gcs[1]
        z = relaxed_ik_pos_gcs[2] + chest_pos / 1000

        # Get row and column of the closest point of grid
        row, col = find_closest_point(y, z, shelf_grid)

        # -1 no pnp
        pick_place = -1

        if gripperState == "open":
            # Reachable goal
            if (
                abs(shelf_grid[row, col][1] - chest_pos / 1000 - relaxed_ik_boundary['z_max']) > z_reach_max_bound and          # If target is far from Z max
                abs(shelf_grid[row, col][1] - chest_pos / 1000 - relaxed_ik_boundary['z_min']) > z_reach_min_bound              # If target is far from Z min
                ):

                # Publish 1 if reachable
                reachability = 1

                # Show intent
                trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1]]), 1)
                trajectory_sampler(np.array([relaxed_ik_boundary['x_max'] + 0.05, shelf_grid[row, col][0], shelf_grid[row, col][1]]), 0.5)
                trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1]]), 0.5)

                pp_sm_state = "confirm"

            # Unreachable goal
            else:
                # Publish 0 if unreachable
                # BUG: does not publish 0
                reachability = 0

                # HACK: allows to print 0s, but blocks
                rospy.sleep(1)

                pp_sm_state = "manual"

        elif gripperState == "close":
            # Reachable goal
            if (
                abs(shelf_grid[row, col][1] + 0.025 - chest_pos / 1000 - relaxed_ik_boundary['z_max']) > z_reach_max_bound and   # If target is far from Z max
                abs(shelf_grid[row, col][1] + 0.025 - chest_pos / 1000 - relaxed_ik_boundary['z_min']) > z_reach_min_bound       # If target is far from Z min
                ):

                # Publish 1 if reachable
                reachability = 1

                # Show intent
                trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 1)
                trajectory_sampler(np.array([relaxed_ik_boundary['x_max'] + 0.05, shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 0.5)
                trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 0.5)

                pp_sm_state = "confirm"

            # Unreachable goal
            else:
                # Publish 0 if unreachable
                # BUG: does not publish 0
                reachability = 0

                # HACK: allows to print 0s, but blocks
                rospy.sleep(1)

                pp_sm_state = "manual"


    # Confirm intent, activate pick and place autonomy
    elif pp_sm_state == "confirm" and right_controller['primaryButton'] == True:

        y = relaxed_ik_pos_gcs[1]
        z = relaxed_ik_pos_gcs[2] + chest_pos / 1000

        # Get row and column of the closest point of grid
        row, col = find_closest_point(y, z, shelf_grid)

        # -1 no reachability intent
        reachability = -1

        # Run picking script
        if gripperState == "open":

            # 0 when picking
            pick_place = 0

            trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1]]), 0.5)
            trajectory_sampler(np.array([x_pick, shelf_grid[row, col][0], shelf_grid[row, col][1]]), 2)
            rospy.sleep(1)
            gripper_control(3, 0.7)
            rospy.sleep(1)
            trajectory_sampler(np.array([x_pick, shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 1)
            trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 1)

            gripperState = "close"

        # Run placing script
        elif gripperState == "close":

            # Publish 1 when placing
            pick_place = 1

            trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 0.5)
            trajectory_sampler(np.array([x_place, shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.025]), 2)

            # Let a block slip
            gripper_control(3, 0.51)
            trajectory_sampler(np.array([x_place, shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.005]), 1)

            # Release a block
            gripper_control(3, 0.0)
            rospy.sleep(1)
            trajectory_sampler(np.array([relaxed_ik_boundary['x_max'], shelf_grid[row, col][0], shelf_grid[row, col][1] + 0.01]), 1)

            gripperState = "open"

        pp_sm_state = "manual"

    # Cancel autonomy, return control
    elif pp_sm_state == "confirm" and (isTracking == True or                                # If gripButton was pressed
        (CHEST_CONTROL_MODE == 0 and abs(right_controller['joystick_pos_y']) > 0.05)):      # OR if control mode is manual and joystick was moved

        pp_sm_state = "manual"

        # Publish -1 when not autonomy and not intent
        pick_place = -1
        reachability = -1


def initialization():
    global isInitialized

    # Clear arm faults
    clearfaults_arm_srv()

    # Home the chest
    chest_homing_srv(True)

    # Set 20% velocity
    pid_vel_limit_srv(0.2)

    # Homing position
    print("\nHoming has started...\n")

    # Let the node initialized
    rospy.sleep(2)

    # Home using relaxedIK
    relaxed_ik_pub_target_rikcs([0, 0, 0], [1, 0, 0, 0])

    # Block until the motion is finished
    wait_motion_finished()

    print("\nHoming has finished.\n")

    # Calculate a transition between Global CS and Kinova CS
    calculate_gcs_kcs_trans()

    print("\nMoving to the starting position...\n")

    # Move the chest to middle position
    chest_abspos_srv(220, 0.6)

    # Move Kinova to the middle position
    relaxed_ik_pub_target_gcs(np.array([0, -0.1, 0.5]), [1, 0, 0, 0])

    # Block until the motion is finished
    wait_motion_finished()

    # # Move Kinova to the middle position
    # trajectory_sampler(np.array([0, -0.1, 0.5 + 0.44]), 2)

    # # Block until the motion is finished
    # rospy.sleep(3)

    # Set 100% velocity
    pid_vel_limit_srv(1.0)

    # Chest scaled motion mapping
    if CHEST_CONTROL_MODE == 2:
        # Operator arm boundary calibration
        while calibrate_arm_boundaries_sm() != True and not rospy.is_shutdown():
            pass

        # Controller to chest position calibration
        calculate_controller_chest_diff()

    # Set the flag and finish initialization
    isInitialized = True

    print("\nSystem is ready.\n")


def e_stop():
    global isTracking, isInitialized

    # E-stop arm motion
    estop_arm_srv()

    # Stop chest motion
    chest_stop_srv()

    # Reset the flag
    isTracking = False
    isInitialized = False

    # Pause datarecoding
    try:
        resume_data_srv(False)

    except Exception as e:
        print(e)


# Roslaunch PID
def roslaunch_pid():
    global pid_launch

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    pid_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/kinova_pid/launch/arm_controller.launch'])
    pid_launch.start()


# Roslaunch relaxed IK
def roslaunch_relaxed_ik():
    global relaxed_ik_launch

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    relaxed_ik_launch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/relaxed_ik_ros1/launch/relaxed_ik.launch'])
    relaxed_ik_launch.start()


# This function is called when the node is shutting down
def node_shutdown():
    global pid_launch, relaxed_ik_launch
    print("\nNode is shutting down...")

    # Stop arm motion
    stop_arm_srv()

    # Stop chest motion
    chest_stop_srv()

    # Deactivate chest feedback
    chest_logger_srv(False)

    # Shutdown PID and relaxed IK nodes
    pid_launch.shutdown()
    relaxed_ik_launch.shutdown()

    # Pause datarecoding
    try:
        resume_data_srv(False)

    except Exception as e:
        print(e)

    print("\nNode has shut down.")



if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("coordinate_converter", anonymous=True)
    rospy.on_shutdown(node_shutdown)

    # Variables
    isInitialized = False
    onHigherLimit = False
    onLowerLimit = False
    GoalSet = False
    aut_bool = False
    reachability = -1
    pick_place = -1

    input_pos_gcs = np.array([0.0, 0.0, 0.0])
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

    left_controller = {'primaryButton': False,
                        'secondaryButton': False,
                        'triggerButton': False,
                        'triggerValue': 0.0,
                        'gripButton': False,
                        'joystickButton': False,
                        'joystick_pos_x': 0.0, 'joystick_pos_y': 0.0,
                        'controller_pos_x': 0.0, 'controller_pos_y': 0.0, 'controller_pos_z': 0.0,
                        'controller_rot_w': 0.0, 'controller_rot_x': 0.0, 'controller_rot_y': 0.0, 'controller_rot_z': 0.0
                        }

    calibrate_arm_boundaries_state = 0

    # Chest
    chest_vel = 0.0
    chest_pos = 440.0
    goal_chest = 440.0
    prev_z = 220.0
    oculus_chest_diff = 0.0
    aut_activation_prox = 0

    # Flags
    isTracking = False
    isTracking_state = 0
    onTrackingStart = {'right_arm': True, 'chest': True}
    onStartup = {'right_arm': True, 'chest': True}

    gripperState = "open"
    gripperButtonState = False
    gripperButtonReleased = True
    gripperValue = 0.0

    initButton_state = False
    estopButton_state = False
    clearfaultButton_state = False

    motionFinished = False

    # Launch files
    roslaunch_pid()
    roslaunch_relaxed_ik()
    rospy.sleep(2)

    # Autonomy
    shelf_grid = generate_grid(5, 4, -0.224, 0.449, [0.298, 0.290, 0.305, 0.280], [0.15, 0.15, 0.15])
    pp_sm_state = "manual"

    # Publishing
    setpoint_pub = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
    relaxed_ik_target_wcs_pub = rospy.Publisher('/relaxed_ik/position_wcs', Float32MultiArray, queue_size=1)
    relaxed_ik_target_gcs_pub = rospy.Publisher('/relaxed_ik/position_gcs', Float32MultiArray, queue_size=1)
    chest_vel_pub = rospy.Publisher('z_chest_vel', geom_msgs.Twist, queue_size=1)
    chest_abspos_pub = rospy.Publisher('/z_chest_pos', clearcore_msg.Position, queue_size=1)
    autonomy_prox_pub = rospy.Publisher('/autonomy_proximity', Bool, queue_size=1)
    reachability_pub = rospy.Publisher('/reachability', Int32, queue_size=1)
    pick_place_pub = rospy.Publisher('/pick_place', Int32, queue_size=1)
    calibration_pub = rospy.Publisher('/calibration_param', Float32MultiArray, queue_size=1)
    tracking_pub = rospy.Publisher('/tracking', Bool, queue_size=1)

    # Subscribing
    rospy.Subscriber('/pid/motion_finished', Bool, pid_motion_finished_callback)
    rospy.Subscriber('/chest_position', geom_msgs.Point, chest_position_callback)
    rospy.Subscriber("rightControllerInfo", ControllerInput, right_callback)
    rospy.Subscriber("leftControllerInfo", ControllerInput, left_callback)
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, ee_callback)

    # Service
    pid_setpoint_srv = rospy.ServiceProxy('pid_setpoint', posctrl_srv.pid_setpoint)
    pid_vel_limit_srv = rospy.ServiceProxy('pid_vel_limit', posctrl_srv.pid_vel_limit)

    activate_ik_srv = rospy.ServiceProxy('relaxed_ik/activate_ik', activate_ik)

    gripper_command_srv = rospy.ServiceProxy('my_gen3/base/send_gripper_command', SendGripperCommand)
    stop_arm_srv = rospy.ServiceProxy('my_gen3/base/stop', Stop)
    estop_arm_srv = rospy.ServiceProxy('my_gen3/base/apply_emergency_stop', ApplyEmergencyStop)
    clearfaults_arm_srv = rospy.ServiceProxy('my_gen3/base/clear_faults', Base_ClearFaults)

    chest_homing_srv = rospy.ServiceProxy('z_chest_home', clearcore_srv.Homing)
    chest_stop_srv = rospy.ServiceProxy('z_chest_stop', clearcore_srv.Stop)
    chest_abspos_srv = rospy.ServiceProxy('z_chest_abspos', clearcore_srv.AbsolutePosition)
    chest_logger_srv = rospy.ServiceProxy('z_chest_logger', clearcore_srv.LoggerControl)

    resume_data_srv = rospy.ServiceProxy('/data_collector/resume_data', posctrl_srv.resume_record)

    # Activate chest feedback
    chest_logger_srv(True)

    # Initialize the robot
    initialization()

    # Open the gripper
    gripper_control(3, 0.0)

    # Main loop
    while not rospy.is_shutdown():
        chest_mapping()
        kinova_mapping()
