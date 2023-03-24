#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
from kortex_driver.msg import *
from kortex_driver.srv import *
import kinova_positional_control.srv as posctrl_srv

class Kinova:
    
    def __init__(self):
        
        self.onStartup = True
        self.ee_array = np.array([0.0, 0.0, 0.0])
        
        # Subscribe
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.ee_position_callback)
        
    def ee_position_callback(self, data):
        
        self.ee_array = np.array([data.base.tool_pose_x, 
                            data.base.tool_pose_y, 
                            data.base.tool_pose_z,
                            math.radians(data.base.tool_pose_theta_x), 
                            math.radians(data.base.tool_pose_theta_y), 
                            math.radians(data.base.tool_pose_theta_z)])

        # Calculate a transition from relaxedIK initial absolute postion (0, 0, 0) to KinovaFK on startup
        if self.onStartup == True:

            # Calculate the difference after homing
            self.relaxedik_kinova_diff_pos = - self.ee_array[0:3]
            self.relaxedik_kinova_diff_rot = - self.ee_array[3:6]

            # Remove the flag
            self.onStartup = False   
