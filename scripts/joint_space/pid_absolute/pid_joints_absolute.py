#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import JointState
from relaxed_ik_ros1.msg import JointAngles

import math

# Get current joint positions
def feedback_callback(data):
    global joint_current_positions
    joint_current_positions = data.position

    state_1.publish(round(data.position[0], 5))
    state_2.publish(round(data.position[1], 5))
    state_3.publish(round(data.position[2], 5))
    state_4.publish(round(data.position[3], 5))
    state_5.publish(round(data.position[4], 5))
    state_6.publish(round(data.position[5], 5))
    state_7.publish(round(data.position[6], 5))

# Update joint velocities
def control_effort_callback_1(data):
    global joint_goal_velocities
    joint_goal_velocities[0] = data.data

def control_effort_callback_2(data):
    global joint_goal_velocities
    joint_goal_velocities[1] = data.data

def control_effort_callback_3(data):
    global joint_goal_velocities
    joint_goal_velocities[2] = data.data

def control_effort_callback_4(data):
    global joint_goal_velocities
    joint_goal_velocities[3] = data.data

def control_effort_callback_5(data):
    global joint_goal_velocities
    joint_goal_velocities[4] = data.data

def control_effort_callback_6(data):
    global joint_goal_velocities
    joint_goal_velocities[5] = data.data

def control_effort_callback_7(data):
    global joint_goal_velocities
    joint_goal_velocities[6] = data.data


# Update goal joint positions
def setpoint_callback(data):
    global joint_goal_positions
    joint_goal_positions[0] = data.angles.data[0]
    joint_goal_positions[1] = data.angles.data[1]
    joint_goal_positions[2] = data.angles.data[2]
    joint_goal_positions[3] = data.angles.data[3]
    joint_goal_positions[4] = data.angles.data[4]
    joint_goal_positions[5] = data.angles.data[5]
    joint_goal_positions[6] = data.angles.data[6]
    

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("pid_joints", anonymous=True)

    # Starting velocities (received from PID as control effort)
    joint_goal_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # Starting (home) position (received from IK)
    joint_goal_positions = [-3.14159, 0.0, -1.57, -1.57, 0.0, 0.0, 1.5708]

    # Current (feedback) joint positions
    joint_current_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

    rospy.Subscriber('/relaxed_ik/joint_angle_solutions', JointAngles, setpoint_callback)  # Global setpoint topic

    # Main loop
    while not rospy.is_shutdown():

        # Form a velocity message
        velocity_message = Base_JointSpeeds()

        velocities_array = []

        # For each joint
        for i in range(0, 7):
            joint_velocity = JointSpeed()
            joint_velocity.joint_identifier = i
            joint_velocity.value = joint_goal_velocities[i]
            joint_velocity.duration = 0   # Or 0.000333s
            velocities_array.append(joint_velocity)

        velocity_message.joint_speeds = velocities_array
        velocity_message.duration = 0

        # Publish a velocity message
        joint_velocity_pub.publish(velocity_message)

        # Publish new goals to PIDs
        setpoint_1.publish(joint_goal_positions[0])
        setpoint_2.publish(joint_goal_positions[1])
        setpoint_3.publish(joint_goal_positions[2])
        setpoint_4.publish(joint_goal_positions[3])
        setpoint_5.publish(joint_goal_positions[4])
        setpoint_6.publish(joint_goal_positions[5])
        setpoint_7.publish(joint_goal_positions[6])