#!/usr/bin/env python

import rospy
from std_msgs.msg import *

if __name__ == '__main__':

    rospy.init_node("setter", anonymous=True)

    setpoint_pub = rospy.Publisher('/setpoint', String, queue_size=1)

    # Main loop
    while not rospy.is_shutdown():
        setpoint_pub.publish("0_0_0_0_0_0_90")
        input()
        setpoint_pub.publish("0_0_0_0_0_0_-90")
        input()

