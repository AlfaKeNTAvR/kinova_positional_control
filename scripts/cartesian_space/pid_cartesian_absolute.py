#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from kortex_driver.srv import *
from kortex_driver.msg import *

def feedback_callback(data):
    state_x.publish(round(data.base.commanded_tool_pose_x, 3))
    state_y.publish(round(data.base.commanded_tool_pose_y, 3))
    state_z.publish(round(data.base.commanded_tool_pose_z, 3))
    #state_theta_x.publish(round(data.base.commanded_tool_pose_theta_x, 3))
    #state_theta_y.publish(round(data.base.commanded_tool_pose_theta_y, 3))
    #state_theta_z.publish(round(data.base.commanded_tool_pose_theta_z, 3))

def control_effort_callback_x(data):
    global linear_x
    linear_x = data.data

def control_effort_callback_y(data):
    global linear_y
    linear_y = data.data

def control_effort_callback_z(data):
    global linear_z
    linear_z = data.data

def control_effort_callback_theta_x(data):
    global angular_x
    angular_x = data.data

def control_effort_callback_theta_y(data):
    global angular_y
    angular_y = data.data

def control_effort_callback_theta_z(data):
    global angular_z
    angular_z = data.data

def setpoint_callback(data):
    global target_x, target_y, target_z, target_theta_x, target_theta_y, target_theta_z
    target_list = data.data.split(" ")
    target_x = float(target_list[0])
    target_y = float(target_list[1])
    target_z = float(target_list[2])
    # target_theta_x = float(target_list[3])
    # target_theta_y = float(target_list[4])
    # target_theta_z = float(target_list[5])

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node("test_listener", anonymous=True)

    # Starting velocities
    linear_x = 0.0
    linear_y = 0.0
    linear_z = 0.0
    angular_x = 0.0
    angular_y = 0.0
    angular_z = 0.0

    # Starting  (home) position 0.357 -0.4 -0.052 
    target_x = 0.494
    target_y = -0.4
    target_z = 0.046
    target_theta_x = -79.9
    target_theta_y = -137.7
    target_theta_z = 166.2

    # Publishing
    state_x = rospy.Publisher('/x/state', Float64, queue_size=10)
    state_y = rospy.Publisher('/y/state', Float64, queue_size=10)
    state_z = rospy.Publisher('/z/state', Float64, queue_size=10)
    state_theta_x = rospy.Publisher('/theta_x/state', Float64, queue_size=10)
    #state_theta_y = rospy.Publisher('/theta_y/state', Float64, queue_size=10)
    state_theta_z = rospy.Publisher('/theta_z/state', Float64, queue_size=10)

    setpoint_x = rospy.Publisher('/x/setpoint', Float64, queue_size=10)
    setpoint_y = rospy.Publisher('/y/setpoint', Float64, queue_size=10)
    setpoint_z = rospy.Publisher('/z/setpoint', Float64, queue_size=10)
    #setpoint_theta_x = rospy.Publisher('/theta_x/setpoint', Float64, queue_size=10)
    #setpoint_theta_y = rospy.Publisher('/theta_y/setpoint', Float64, queue_size=10)
    #setpoint_theta_z = rospy.Publisher('/theta_z/setpoint', Float64, queue_size=10)

    velocity = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=10)
    
    # Subscribing
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, feedback_callback)

    rospy.Subscriber('/x/control_effort', Float64, control_effort_callback_x)
    rospy.Subscriber('/y/control_effort', Float64, control_effort_callback_y)
    rospy.Subscriber('/z/control_effort', Float64, control_effort_callback_z)
    #rospy.Subscriber('/theta_x/control_effort', Float64, control_effort_callback_theta_x)
    #rospy.Subscriber('/theta_y/control_effort', Float64, control_effort_callback_theta_y)
    #rospy.Subscriber('/theta_z/control_effort', Float64, control_effort_callback_theta_z)

    rospy.Subscriber('/setpoint_topic', String, setpoint_callback)

    # Main loop
    while not rospy.is_shutdown():
        test_msg = TwistCommand()
        test_msg.reference_frame = 3
        test_msg.twist.linear_x = linear_x
        test_msg.twist.linear_y = linear_y
        test_msg.twist.linear_z = linear_z
        test_msg.twist.angular_x = angular_x
        test_msg.twist.angular_y = angular_y
        test_msg.twist.angular_z = angular_z
        test_msg.duration = 0

        velocity.publish(test_msg)

        setpoint_x.publish(target_x)
        setpoint_y.publish(target_y)
        setpoint_z.publish(target_z)
        #setpoint_theta_x.publish(target_theta_x)
        #setpoint_theta_y.publish(target_theta_y)
        #setpoint_theta_z.publish(target_theta_z)






