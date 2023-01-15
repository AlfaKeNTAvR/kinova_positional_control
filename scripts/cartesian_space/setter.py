#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from gopher_pos_ctrl.msg import *


import numpy as np
import math
  
def Rx(theta):
    theta = math.radians(theta)
    return np.matrix([[1, 0, 0],
                      [0, math.cos(theta),-math.sin(theta)],
                      [0, math.sin(theta), math.cos(theta)]])
  
def Ry(theta):
    theta = math.radians(theta)
    return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                      [0, 1, 0],
                      [-math.sin(theta), 0, math.cos(theta)]])
  
def Rz(theta):
    theta = math.radians(theta)
    return np.matrix([[math.cos(theta), -math.sin(theta), 0],
                      [math.sin(theta), math.cos(theta) , 0],
                      [0, 0, 1 ]])

def callback_left(data):
    global input_array
    input_array = np.array([data.pos_x, data.pos_z, data.pos_y]) 


if __name__ == '__main__':
    rospy.init_node("test_setter", anonymous=True)

    R_oculus_to_kinova = Rz(-90) * Ry(-90)
    R_kinova_to_oculus = np.transpose(R_oculus_to_kinova)
    R_forw = Rx(0) * Ry(-42) * Rz(-90) #* Rz(-90) * Ry(-90)
    R_back = np.transpose(R_forw)
    test = np.round(np.matmul(R_back, np.array([357, -400, -52])), 1)
    print(test)
    # test1 = np.round(np.matmul(R_forw, test[0]), 1)
    # print(test1)

    # Publishing
    setpoint = rospy.Publisher('/setpoint_topic', String, queue_size=10)

    # Subscribing
    rospy.Subscriber("pos_rot_left", PosRot, callback_left)

    input_array = np.array([0, 0, 0])
    
    
    # Main loop
    while(True):

        # # Read an input string
        # input_string = input()

        # # Parse to a list
        # input_list = input_string.split(" ")

        # # Convert to float
        # for i in range(len(input_list)):
        #     input_list[i] = float(input_list[i])

        # # Convert to a numpy array
        # input_array = np.array(input_list)
        
        # Rotate the array
        rot_array = np.matmul(R_forw, input_array)

        final_str = ""

        rot_array[0, 0] += 1.75
        rot_array[0, 1] -= 0.75
        rot_array[0, 2] -= 1.35

        for i in range(rot_array.size):
            final_str += str(np.round(rot_array[0, i], 3)) + " "

        setpoint.publish(final_str)

        print(final_str)






        # y = 0.0
        # z = 0.5
        
        # setpoint.publish("0.57 " + str(y) + " " + str(z))
        # rospy.sleep(5)

        # for i in range(6):
        #     if i < 3:
        #         z += i/10
        #     if i >= 3:
        #         y += i/10
        #     setpoint.publish("0.57 " + str(y) + " " + str(z))
        #     rospy.sleep(0.5)
        # rospy.sleep(5)



