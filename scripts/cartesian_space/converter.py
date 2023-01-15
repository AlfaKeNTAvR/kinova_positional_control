#!/usr/bin/env python
from turtle import right
import rospy
from std_msgs.msg import *
import numpy as np
import math
from kortex_driver.msg import *


left_data=""
right_data=""
pub = rospy.Publisher('/chatter', String, queue_size=10)
#Node
#2 Subscribers (left + right)
#1 Publisher

def cvt_to_string(data):
    output = str(data.x)+str(data.y)+str(data.z)
    return output

def callback_left(data):
    global left_data
    left_data = cvt_to_string(data)

def callback_right(data):
    global right_data
    right_data = cvt_to_string(data)

def timer_callback(event):
    global pub,left_data,right_data
    message_string = left_data + ";" + right_data + ":"
    pub.publish(message_string)

def converter():
    rospy.Subscriber("pos_rot_left", PosRot, callback_left)
    rospy.Subscriber("pos_rot_right", PosRot, callback_right)
    rospy.init_node('converter', anonymous=True)
    timer = rospy.Timer(rospy.Duration(0.5), timer_callback)
    rospy.spin()    
    timer.shutdown()



if __name__ == '__main__':
    converter()




