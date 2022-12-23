#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
from relaxed_ik_ros1.msg import JointAngles

import math

# Get current relative joint positions
def feedback_callback(data):
    global current_abs_pos, start_abs_pos, continuous_joint_indices

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
    state_1.publish(current_rel_pos[0])
    state_2.publish(current_rel_pos[1])
    state_3.publish(current_rel_pos[2])
    state_4.publish(current_rel_pos[3])
    state_5.publish(current_rel_pos[4])
    state_6.publish(current_rel_pos[5])
    state_7.publish(current_rel_pos[6])
    
# Update joint velocities
def control_effort_callback_1(data):
    global goal_vel
    goal_vel[0] = data.data

def control_effort_callback_2(data):
    global goal_vel
    goal_vel[1] = data.data

def control_effort_callback_3(data):
    global goal_vel
    goal_vel[2] = data.data

def control_effort_callback_4(data):
    global goal_vel
    goal_vel[3] = data.data

def control_effort_callback_5(data):
    global goal_vel
    goal_vel[4] = data.data

def control_effort_callback_6(data):
    global goal_vel
    goal_vel[5] = data.data

def control_effort_callback_7(data):
    global goal_vel
    goal_vel[6] = data.data


# Calculates geodesic (shortest) angular distance and sets the goal positions relative to current positions
def relative_setpoint_callback(data):
    global goal_abs_pos, goal_rel_pos, current_abs_pos, start_abs_pos, continuous_joint_indices

    #goal_rel_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Goal absolute coordinates input
    goal_abs_pos = data.data.split("_")

    for i in range(len(goal_abs_pos)):
        goal_abs_pos[i] = math.radians(float(goal_abs_pos[i]))

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

        # print(math.degrees(goal_abs_pos[joint_index]), math.degrees(current_pos[joint_index]), math.degrees(goal_rel_pos[joint_index]))
        # print("Joint:", joint_index, "Goal:", round(goal_abs_pos[joint_index], 3), "Current:", round(current_abs_pos[joint_index], 3), "Relative:", round(goal_rel_pos[joint_index], 3))

    # print()


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("pid_joints", anonymous=True)

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
    rospy.Subscriber('/setpoint', String, relative_setpoint_callback)  # Single joint setpoint for debugging

    # Main loop
    while not rospy.is_shutdown():

        # Form a velocity message
        velocity_message = Base_JointSpeeds()

        velocities_array = []

        # For each joint
        for i in range(0, 7):
            joint_velocity = JointSpeed()
            joint_velocity.joint_identifier = i
            joint_velocity.value = goal_vel[i]
            joint_velocity.duration = 0   # Or 0.000333s
            velocities_array.append(joint_velocity)

        velocity_message.joint_speeds = velocities_array
        velocity_message.duration = 0

        # Publish a velocity message
        joint_velocity_pub.publish(velocity_message)

        # Publish new goals to PIDs
        setpoint_1.publish(goal_rel_pos[0])
        setpoint_2.publish(goal_rel_pos[1])
        setpoint_3.publish(goal_rel_pos[2])
        setpoint_4.publish(goal_rel_pos[3])
        setpoint_5.publish(goal_rel_pos[4])
        setpoint_6.publish(goal_rel_pos[5])
        setpoint_7.publish(goal_rel_pos[6])