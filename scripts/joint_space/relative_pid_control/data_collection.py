#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ros_tcp_endpoint_msgs.msg import ControllerInput
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from kortex_driver.msg import *
from kortex_driver.srv import *
from kinova_positional_control.msg import AutonomyInfo
import kinova_positional_control.srv as posctrl_srv
from relaxed_ik_ros1.srv import activate_ik
import gopher_ros_clearcore.msg as clearcore_msg
import gopher_ros_clearcore.srv as clearcore_srv

# Class for Kinova
class Arm:
    def jointstate_cb(self, data):

        self.joint_states = data.position[0:7]
        gripper_value = data.position[8]

        if gripper_value < 0.1 and gripper_value > 0:
            self.gripper_state = 'OPEN'
        else:
            self.gripper_state = 'CLOSED'
        
    def relaxedik_wcs_cb(self, data):

        self.ee_pos_wcs = data

    def relaxedik_gcs_cb(self, data):
        #TODO: Check this on other side
        self.ee_pos_gcs = data

    def relaxedik_cb(self, data):

        ee_pos = data.ee_poses[0].position
        self.ee_pos = [ee_pos.x, ee_pos.y, ee_pos.z]

    def error_cb(self, data):
        
        self.kinova_pos_kcs = np.array([data.base.tool_pose_x, 
                    data.base.tool_pose_y, 
                    data.base.tool_pose_z])

        error_joint1 = data.actuators[0].fault_bank_a
        error_joint2 = data.actuators[1].fault_bank_a
        error_joint3 = data.actuators[2].fault_bank_a
        error_joint4 = data.actuators[3].fault_bank_a
        error_joint5 = data.actuators[4].fault_bank_a
        error_joint6 = data.actuators[5].fault_bank_a
        error_joint7 = data.actuators[6].fault_bank_a
        
        if error_joint1 != 0 or error_joint2 != 0 or error_joint3 != 0 or error_joint4 != 0 or error_joint5 != 0 or error_joint6 != 0 or error_joint7 != 0:
            self.error = 1
        else:
            self.error = 0

# Class for Chest
class Chest:
    def __init__(self):

        self.pos = 220.0        # Initial chest position
        self.init_chest_pos = 0.0
        self.dis_per_activation_proximity = 0.0

    def position_cb(self, data):

        self.pos = data.z

    def autonomy_state_cb(self, data):

        # Number of autonomy activations
        self.aut_activation_prox = data.aut_activations
        
        if data.aut_on == True:
            self.aut_activation_prox += 1

            self.current_chest_pos = chest.pos

            # Calculate distance moved since manual activation
            self.dis_per_activation_proximity = round(abs(self.current_chest_pos - self.init_chest_pos), 2)

        else:
            self.init_chest_pos = chest.pos


# Class for controller
class Controller:

    def __init__(self):
        
        self.chestactivations = 0.0
        self.init_chest_pos = 0.0
        self.trackingactivations = 0.0
        self.Starting = True
        self.Starting_gripper = True
        
    def right_callback(self, data):

        # Tracking state and activations
        gripButton = data.gripButton

        if gripButton == True:

            self.tracking = 1

            if self.Starting_gripper:

                self.trackingactivations += 1
                self.Starting_gripper = False

        else:
            self.tracking = 0
            self.Starting_gripper = True

        # Manual chest control activations
        if abs(data.joystick_pos_y) > 0.0:
            if self.Starting:

                self.chestactivations += 1

                self.Starting = False
                
            self.current_chest_pos = chest.pos

            # Calculate distance moved since manual activation
            self.distance_per_activation = abs(self.current_chest_pos - self.init_chest_pos)

        if abs(data.joystick_pos_y) == 0.0:
            
            self.init_chest_pos = chest.pos

            self.Starting = True

if __name__ == '__main__':
    
    chest = Chest()
    arm = Arm()
    controller = Controller()

    #Subscribe to topics
    # Joint angles and Gripper value
    rospy.Subscriber('/my_gen3/joint_states', JointState, arm.jointstate_cb)
    # EE position
    rospy.Subscriber('/relaxed_ik/position_wcs', Float32MultiArray, arm.relaxedik_wcs_cb)
    rospy.Subscriber('/relaxed_ik/position_gcs', Float32MultiArray, arm.relaxedik_gcs_cb)
    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, arm.relaxedik_cb)
    # Arm errors
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, arm.error_cb)
    # Chest position
    rospy.Subscriber('/chest_position', geom_msgs.Point, chest.position_cb)
    rospy.Subscriber('/autonomy_proximity', AutonomyInfo, chest.autonomy_state_cb)
    # Controller
    rospy.Subscriber("rightControllerInfo", ControllerInput, controller.right_callback)


    # Initialize the node
    rospy.init_node("data_collection", anonymous=True)

    #TODO: Start recording when pressing a key 

    # Main loop
    while not rospy.is_shutdown():
        pass
        

    #Export to CSV file with name partecipant, Sheet with Chest control mode
