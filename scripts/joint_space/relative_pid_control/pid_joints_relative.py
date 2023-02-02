#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import *
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
from kinova_positional_control.srv import *


# Get current relative joint positions
def feedback_callback(data):
    global current_abs_pos, start_abs_pos, continuous_joint_indices
    global isInitialized, kinovaInitialized

    # Set kinovaInitialized flag once feedback is received
    kinovaInitialized = True

    # Update start_abs_pos from zero to current arm position on initialization
    if not isInitialized:
        start_abs_pos = data.position

    else:
        # Current relative joint positions
        current_rel_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Current absolute joint positions
        current_abs_pos = data.position

        '''
        Recalculate joint feedback to be relative to the starting absolute position, where start_abs_pos is relative origin (zero).

        BUG: if the joint overshoots at 180 commanded relative position it might get stuck in the infinite 360 loop (keep rotating).
        '''

        for joint_index in range(7):

            # Recalculate joint relative feedback for continuous roation joints
            if joint_index in continuous_joint_indices:

                # Commanded relative position is within (-180; 180]
                if -1 * math.pi < current_abs_pos[joint_index] - start_abs_pos[joint_index] <= math.pi:
                    current_rel_pos[joint_index] = current_abs_pos[joint_index] - start_abs_pos[joint_index]
                    
                # Commanded relative position is crossing -180/180 border from the negative sign to the positive sign (Example -20: -170 to 170)
                elif current_abs_pos[joint_index] - start_abs_pos[joint_index] > math.pi:
                    current_rel_pos[joint_index] = current_abs_pos[joint_index] - start_abs_pos[joint_index] - 2 * math.pi

                # Commanded relative position is crossing -180/180 border from the positive sign to the negative sign (Example +20: 170 to -170)
                elif current_abs_pos[joint_index] - start_abs_pos[joint_index] <= -1 * math.pi:
                    current_rel_pos[joint_index] = current_abs_pos[joint_index] - start_abs_pos[joint_index] + 2 * math.pi
            
            else:
                current_rel_pos[joint_index] = current_abs_pos[joint_index] - start_abs_pos[joint_index]

        # Publish relative feedback
        state_1.publish(round(current_rel_pos[0], 4))
        state_2.publish(round(current_rel_pos[1], 4))
        state_3.publish(round(current_rel_pos[2], 4))
        state_4.publish(round(current_rel_pos[3], 4))
        state_5.publish(round(current_rel_pos[4], 4))
        state_6.publish(round(current_rel_pos[5], 4))
        state_7.publish(round(current_rel_pos[6], 4))
 
    
# Update joint velocities
def control_effort_callback_1(data):
    global goal_vel
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:
        goal_vel[0] = data.data

def control_effort_callback_2(data):
    global goal_vel
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:
        goal_vel[1] = data.data

def control_effort_callback_3(data):
    global goal_vel
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:
        goal_vel[2] = data.data

def control_effort_callback_4(data):
    global goal_vel
    goal_vel[3] = data.data

def control_effort_callback_5(data):
    global goal_vel
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:
        goal_vel[4] = data.data

def control_effort_callback_6(data):
    global goal_vel
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:
        goal_vel[5] = data.data

def control_effort_callback_7(data):
    global goal_vel
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:
        goal_vel[6] = data.data


# Calculates geodesic (shortest) angular distance and sets the goal positions relative to current positions
def relative_setpoint(goal_positions):
    global goal_abs_pos, goal_rel_pos, current_abs_pos, start_abs_pos, continuous_joint_indices

    # Goal absolute coordinates input
    goal_abs_pos = goal_positions

    # Set origin positions
    start_abs_pos = current_abs_pos

    for joint_index in range(7):

        # Recalculate joint relative feedback for continuous roation joints 
        if joint_index in continuous_joint_indices:

            # Calculate geodesic angular distance http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html#Geodesic-distance-and-interpolation
            if -1 * math.pi < goal_abs_pos[joint_index] - current_abs_pos[joint_index] <= math.pi:
                goal_rel_pos[joint_index] = goal_abs_pos[joint_index] - current_abs_pos[joint_index]
            
            elif goal_abs_pos[joint_index] - current_abs_pos[joint_index] > math.pi:
                goal_rel_pos[joint_index] = goal_abs_pos[joint_index] - current_abs_pos[joint_index] - 2 * math.pi

            elif goal_abs_pos[joint_index] - current_abs_pos[joint_index] <= -1 * math.pi:
                goal_rel_pos[joint_index] = goal_abs_pos[joint_index] - current_abs_pos[joint_index] + 2 * math.pi

        else:
            goal_rel_pos[joint_index] = goal_abs_pos[joint_index] - current_abs_pos[joint_index]


# RelaxedIK callback function
def relative_setpoint_callback(data):
    global isInitialized

    # Block callback function until all components are initialized
    if isInitialized:

        # Form input array
        goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

        goal_positions[0] = data.joint_angles[0].value
        goal_positions[1] = data.joint_angles[1].value
        goal_positions[2] = data.joint_angles[2].value
        goal_positions[3] = data.joint_angles[3].value
        goal_positions[4] = data.joint_angles[4].value
        goal_positions[5] = data.joint_angles[5].value
        goal_positions[6] = data.joint_angles[6].value

        # Calculate relative setpoint based on the input absolute goal positions
        relative_setpoint(goal_positions)


# Calculates geodesic (shortest) angular distance and sets the goal positions relative to current positions
def pid_setpoint_handler(req):
    global isInitialized, motionFinished

    # Block callback function until all components are initialized
    if isInitialized:

        # Form input array
        input = req.setpoint.split(" ")
        goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

        goal_positions[0] = float(input[0])
        goal_positions[1] = float(input[1])
        goal_positions[2] = float(input[2])
        goal_positions[3] = float(input[3])
        goal_positions[4] = float(input[4])
        goal_positions[5] = float(input[5])
        goal_positions[6] = float(input[6])

        # Calculate relative setpoint based on the input absolute goal positions
        relative_setpoint(goal_positions)

        return True

    return False


# Set a velocity limit in percentage (0.0 to 1.0)
def pid_vel_limit_handler(req):
    global vel_limit, max_vel, min_vel

    # Block callback function until all components are initialized
    if isInitialized:

        if req.data > max_vel:
            vel_limit = max_vel
        
        elif req.data < min_vel:
            vel_limit = min_vel

        else:
            vel_limit = req.data

        return True

    return False


# Check if all joints have an absolute velocity value lower than a threshold and returns True
def motion_finished(velocities):
    for vel in velocities:
        if abs(vel) > 0.01:
            return False
    
    return True


# This function is called when the node is shutting down
def node_shutdown():
    print("\nNode is shutting down...")

    # Stop arm movement
    stop_arm_srv()


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("pid_joints", anonymous=True)
    rospy.on_shutdown(node_shutdown)

    # Flags
    isInitialized = False
    kinovaInitialized = False
    motionFinished = True

    # Continous rotation joint indices
    continuous_joint_indices = (0, 2, 4, 6)

    # Starting velocities (received from PID as control effort)
    goal_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Absolute goal positions
    goal_abs_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    goal_rel_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Absolute current (feedback) joint positions
    current_abs_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Abs joint positions at the moment of setting a new goal. Relative current joint positions are relative to these positions
    start_abs_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 

    # Other variables
    max_vel = 1.0
    min_vel = 0.1
    vel_limit = 1.0
    vel_smoothing = 1.0

    # Joint velocity limits in radians
    max_velocities = [1.396, 1.396, 1.396, 1.396, 1.222, 1.222, 1.222]

    # Publishing
    state_1 = rospy.Publisher('/joint_1/state', Float64, queue_size=1)
    state_2 = rospy.Publisher('/joint_2/state', Float64, queue_size=1)
    state_3 = rospy.Publisher('/joint_3/state', Float64, queue_size=1)
    state_4 = rospy.Publisher('/joint_4/state', Float64, queue_size=1)
    state_5 = rospy.Publisher('/joint_5/state', Float64, queue_size=1)
    state_6 = rospy.Publisher('/joint_6/state', Float64, queue_size=1)
    state_7 = rospy.Publisher('/joint_7/state', Float64, queue_size=1)

    setpoint_1 = rospy.Publisher('/joint_1/setpoint', Float64, queue_size=1)
    setpoint_2 = rospy.Publisher('/joint_2/setpoint', Float64, queue_size=1)
    setpoint_3 = rospy.Publisher('/joint_3/setpoint', Float64, queue_size=1)
    setpoint_4 = rospy.Publisher('/joint_4/setpoint', Float64, queue_size=1)
    setpoint_5 = rospy.Publisher('/joint_5/setpoint', Float64, queue_size=1)
    setpoint_6 = rospy.Publisher('/joint_6/setpoint', Float64, queue_size=1)
    setpoint_7 = rospy.Publisher('/joint_7/setpoint', Float64, queue_size=1)

    joint_velocity_pub = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=1)

    pid_motion_pub = rospy.Publisher('/pid/motion_finished', Bool, queue_size=1)
    
    # Subscribing
    rospy.Subscriber('/my_gen3/base_feedback/joint_state', JointState, feedback_callback)

    rospy.Subscriber('/joint_1/control_effort', Float64, control_effort_callback_1)
    rospy.Subscriber('/joint_2/control_effort', Float64, control_effort_callback_2)
    rospy.Subscriber('/joint_3/control_effort', Float64, control_effort_callback_3)
    rospy.Subscriber('/joint_4/control_effort', Float64, control_effort_callback_4)
    rospy.Subscriber('/joint_5/control_effort', Float64, control_effort_callback_5)
    rospy.Subscriber('/joint_6/control_effort', Float64, control_effort_callback_6)
    rospy.Subscriber('/joint_7/control_effort', Float64, control_effort_callback_7)

    rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, relative_setpoint_callback)  # Global setpoint topic

    # Services
    setpoint_srv = rospy.Service('pid_setpoint', pid_setpoint, pid_setpoint_handler)
    vel_limit_srv = rospy.Service('pid_vel_limit', pid_vel_limit, pid_vel_limit_handler)

    stop_arm_srv = rospy.ServiceProxy('my_gen3/base/stop', Stop)

    # Wait for kinova arm to send position feedback
    print("\nWaiting for kinova position feedback...\n")

    while not kinovaInitialized:
        pass

    print("\nFeedback is received!\n")

    # Set isInitialized flag
    isInitialized = True

    print("\nPID is initialized!\n")

    # Main loop
    while not rospy.is_shutdown():

        # Wait for all components to be initialized
        if isInitialized:

            # Form a velocity message
            velocity_message = Base_JointSpeeds()

            velocities_array = []

            # For each joint
            for i in range(0, 7):
                joint_velocity = JointSpeed()
                joint_velocity.joint_identifier = i

                # Limit output velocity
                if abs(goal_vel[i]) > max_velocities[i] * vel_limit:

                    if goal_vel[i] >= 0:
                        goal_vel[i] = max_velocities[i] * vel_limit

                    else:
                        goal_vel[i] = -1 * max_velocities[i] * vel_limit

                # Velocity smoothing
                goal_vel[i] = goal_vel[i] * vel_smoothing

                joint_velocity.value = goal_vel[i]
                joint_velocity.duration = 0   # Or 0.000333s
                velocities_array.append(joint_velocity)

            velocity_message.joint_speeds = velocities_array
            velocity_message.duration = 0

            # Publish a velocity message
            joint_velocity_pub.publish(velocity_message)

            # Publish new goals to PIDs
            setpoint_1.publish(round(goal_rel_pos[0], 4))
            setpoint_2.publish(round(goal_rel_pos[1], 4))
            setpoint_3.publish(round(goal_rel_pos[2], 4))
            setpoint_4.publish(round(goal_rel_pos[3], 4))
            setpoint_5.publish(round(goal_rel_pos[4], 4))
            setpoint_6.publish(round(goal_rel_pos[5], 4))
            setpoint_7.publish(round(goal_rel_pos[6], 4))

            # Check if all joint have finished their motion
            pid_motion_pub.publish(motion_finished(goal_vel))
