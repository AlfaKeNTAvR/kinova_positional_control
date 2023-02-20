#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math
import argparse
import os
import time
import pandas as pd
from datetime import datetime

from ros_tcp_endpoint_msgs.msg import ControllerInput
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
from sensor_msgs.msg import JointState
from kortex_driver.msg import *
from kortex_driver.srv import *
from kinova_positional_control.msg import AutonomyInfo


# Class for Kinova
class Arm:
    def __init__(self):
        
        self.FirstTime1 = True
        self.FirstTime2 = True
        self.FirstTime3 = True
        self.FirstTime4 = True
        self.FirstTime5 = True

        # Joint state
        self.jointstate_array = []
        # Gripper state
        self.gripper_array = []

        # EE Position
        # [x, y, z] in KCS
        self.ee_array = []
        # Relaxed IK in KCS
        self.ee_rikcs_array = []
        # in GCS
        self.ee_gcs_array = []
        # in WCS
        self.ee_wcs_array = []

        # Robot state
        self.faultstate_array = []

    def jointstate_cb(self, data):

        joint_states = data.position[0:7]

        # If gripper is open: 0
        if data.position[8] < 0.1 and data.position[8] > 0:
            gripper_value = 0
        # If gripper is closed: 1
        else:
            gripper_value = 1

        if self.FirstTime1:
            self.gripper_array.append(gripper_value)
            self.jointstate_array.append(joint_states)

            self.FirstTime1 = False
        
    def relaxedik_wcs_cb(self, data):
        xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
        Rx = T.rotation_matrix(math.radians(0.0), xaxis)
        Ry = T.rotation_matrix(math.radians(-48.2), yaxis)
        Rz = T.rotation_matrix(math.radians(90.0), zaxis)
        R_gcs_to_kcs = T.concatenate_matrices(Rx, Ry, Rz)

        ee_pos_wcs = list(data.data[0:3])

        if self.FirstTime2:
            ee_pos_rikcs = np.matmul(R_gcs_to_kcs[0:3,0:3], ee_pos_wcs)

            self.ee_rikcs_array.append(ee_pos_rikcs)
            self.ee_wcs_array.append(ee_pos_wcs)

            self.FirstTime2 = False

    def relaxedik_gcs_cb(self, data):
        
        ee_pos_gcs = list(data.data[0:3])

        if self.FirstTime3:
            
            self.ee_gcs_array.append(ee_pos_gcs)

            self.FirstTime3 = False

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
            # 1 if it's in a fault state
            robot_state = 1

        else:
            # 0 if it's not in a fault state
            robot_state = 0

        if self.FirstTime5:

            self.ee_array.append(self.kinova_pos_kcs)
            self.faultstate_array.append(robot_state)

            self.FirstTime5 = False


# Class for Chest
class Chest:
    def __init__(self):

        self.FirstTime1 = True
        self.FirstTime2 = True
        self.FirstTime3 = True

        self.pos = 220.0        # Initial chest position
        self.prev_pos = 220.0
        self.init_chest_pos = 0.0
        self.dis_per_activation_proximity = 0.0
        self.tot_trav_distance_scaling = 0.0
        self.tot_trav_distance_proximity = 0.0

        # Chest position
        self.chest_pos_array = []

        # Total travelled distance for Mode 1 
        self.tot_dist_chest_array = []

        # Distance travelled per activation for Mode 1
        self.dist_per_activation_array = []
         
        # Autonomy chest activations for Mode 1
        self.chest_activations_array = []

    def position_cb(self, data):

        self.pos = data.z

        if self.FirstTime1:
  
            self.chest_pos_array.append(self.pos)

            self.FirstTime1 = False

        if CHEST_CONTROL_MODE == 2:
            if abs(self.pos - self.prev_pos) > 0.5:
                self.tot_trav_distance_scaling += abs(self.pos - self.prev_pos)

                self.prev_pos = self.pos

            if self.FirstTime2:

                self.tot_dist_chest_array.append(self.tot_trav_distance_scaling)

                self.FirstTime2 = False

    def autonomy_state_cb(self, data):

        if CHEST_CONTROL_MODE == 1:
            # Number of autonomy activations
            self.aut_activation_prox = data.aut_activations

            if data.aut_on == True:

                # Calculate distance moved since aut activation
                self.dis_per_activation_proximity = abs(self.pos - self.init_chest_pos)
                
                if abs(self.pos - self.prev_pos) > 0.5:
                    self.tot_trav_distance_proximity += abs(self.pos - self.prev_pos)

                    self.prev_pos = self.pos

            else:
                self.init_chest_pos = chest.pos
                # if autonomy is not on, set distance per activation equal to 0
                self.dis_per_activation_proximity = 0.0

            if self.FirstTime3:
                print(self.dis_per_activation_proximity)
                self.chest_activations_array.append(self.aut_activation_prox)
                self.dist_per_activation_array.append(self.dis_per_activation_proximity)
                self.tot_dist_chest_array.append(self.tot_trav_distance_proximity)

                self.FirstTime3 = False


# Class for controller       
class Controller:

    def __init__(self):
        
        self.FirstTime1 = True
        self.FirstTime2 = True
        self.FirstTime3 = True
        self.FirstTime4 = True
        self.FirstTime5 = True

        self.prev_intent = -1
        self.prev_pickplace = -1
        self.chestactivations = 0.0
        self.init_chest_pos = 0.0
        self.prev_pos = 220.0
        self.tot_trav_distance_manual = 0.0
        self.distance_per_activation = 0.0
        self.trackingactivations = 0.0
        self.Starting = True
        self.Starting_gripper = True
        
        # Tracking state
        self.tracking_array = []

        # Tracking activations
        self.tracking_act_array = []

        # Chest activations for manual control
        self.chest_activations_array = []

        # Total travelled distance for Mode 0 
        self.tot_dist_chest_array = []

        self.reachability_array = []
        self.pickplace_array = []

        # Controller position
        self.controller_array = []

        # Distance travelled per activation for Mode 0
        self.dist_per_activation_array = []

    def right_callback(self, data):
        
        # Controller coords
        input_pos_gcs = np.array([-1 * data.controller_pos_z, data.controller_pos_x, data.controller_pos_y])

        if CHEST_CONTROL_MODE == 0:
            # Manual chest control activations
            if abs(data.joystick_pos_y) > 0.0:
                if self.Starting:

                    self.chestactivations += 1

                    self.Starting = False

                # Calculate distance moved since manual activation
                self.distance_per_activation = abs(chest.pos - self.init_chest_pos)

                if abs(chest.pos - self.prev_pos) > 0.5:
                    self.tot_trav_distance_manual += abs(chest.pos - self.prev_pos)

                    self.prev_pos = chest.pos

            if abs(data.joystick_pos_y) == 0.0:
                self.init_chest_pos = chest.pos

                self.distance_per_activation = 0
                self.Starting = True

            if self.FirstTime1:
                
                self.chest_activations_array.append(self.chestactivations)
                self.dist_per_activation_array.append(self.distance_per_activation)
                self.tot_dist_chest_array.append(self.tot_trav_distance_manual)

                self.FirstTime1 = False

        if self.FirstTime2:
            self.controller_array.append(input_pos_gcs)

            self.FirstTime2 = False

    def tracking_cb(self, data):
        
        # Tracking state and activations
        if data.data:
            # 1 if tracking
            tracking_value = 1
            
            if self.Starting_gripper:

                self.trackingactivations += 1

                self.Starting_gripper = False

        else:
            # 0 if not tracking
            tracking_value = 0
            self.Starting_gripper = True

        if self.FirstTime5:
            self.tracking_array.append(tracking_value)
            self.tracking_act_array.append(self.trackingactivations)

            self.FirstTime5 = False

    def pick_place_cb(self, data):
        
        pickplace = data.data

        if self.FirstTime3:
            print("PNP:", pickplace)
            self.pickplace_array.append(pickplace)

            self.FirstTime3 = False

    def reachability_cb(self, data):
        reachable = data.data
        
        if self.FirstTime4:
            print("Reach:", reachable)
            self.reachability_array.append(reachable)

            self.FirstTime4 = False

    def calibr_cb(self, data):
            
        self.calibration_max = round(data.data[0], 2)
        
        self.calibration_min = round(data.data[1], 2)


def FindMaxLength(lst):
    
    maxLength = max(len(x) for x in lst)
 
    return maxLength

def store_data(CHEST_CONTROL_MODE, participant_index, trial, time_array):

    root = os.path.abspath('src/kinova_positional_control/Data')
    file_type = ".csv"

    if (not os.path.exists(root)):
        os.mkdir(root)

    if (not os.path.exists(root + '/' + participant_index)):
        os.mkdir(root + '/' + participant_index)

    path = root + '/' + participant_index + '/mode ' + str(CHEST_CONTROL_MODE) + ' - trial ' + str(trial) + file_type
    
    participant_array = [participant_index] * len(time_array)
    control_mode_array = [CHEST_CONTROL_MODE] * len(time_array)
    trial_array = [trial] * len(time_array)
    calibration_max_array = [controller.calibration_max] * len(time_array)
    calibration_min_array = [controller.calibration_min] * len(time_array)
    NA_array = ['N/A'] * len(time_array)

    if CHEST_CONTROL_MODE == 0:
        chest_activations = controller.chest_activations_array
        chest_total_distance = controller.tot_dist_chest_array
        chest_distance_per_act = controller.dist_per_activation_array
        calibration_min = NA_array
        calibration_max = NA_array

    elif CHEST_CONTROL_MODE == 1:
        chest_activations = chest.chest_activations_array
        chest_total_distance = chest.tot_dist_chest_array
        chest_distance_per_act = chest.dist_per_activation_array
        calibration_min = NA_array
        calibration_max = NA_array

    elif CHEST_CONTROL_MODE == 2:
        chest_activations = NA_array
        chest_total_distance = chest.tot_dist_chest_array
        chest_distance_per_act = NA_array
        calibration_min = calibration_min_array
        calibration_max = calibration_max_array
    
    arrays = [date_array,
              time_array, 
              participant_array,
              control_mode_array,
              trial_array,
              arm.ee_rikcs_array, 
              arm.ee_gcs_array, 
              arm.ee_wcs_array, 
              arm.ee_array, 
              arm.jointstate_array,
              arm.gripper_array, 
              arm.faultstate_array, 
              controller.tracking_array, 
              controller.tracking_act_array, 
              chest.chest_pos_array, 
              chest_activations, 
              chest_total_distance, 
              chest_distance_per_act,
              controller.controller_array,
              controller.reachability_array,
              controller.pickplace_array,
              calibration_min,
              calibration_max]
    
    maxLength = FindMaxLength(arrays)

    for item in arrays:
        if item is Empty:
            print("ERROR:", item, 'is empty!!!')

        if len(item) != maxLength:
            diff_length = abs(maxLength - len(item))
            for i in range(0, diff_length):
                item.append(item[-1])

    # print(len(date_array))
    # print(len(time_array))
    # print(len(participant_array))
    # print(len(control_mode_array))
    # print(len(arm.ee_rikcs_array))
    # print(len(arm.ee_gcs_array))
    # print(len(arm.ee_wcs_array))
    # print(len(arm.ee_array))
    # print(len(arm.jointstate_array))
    # print(len(arm.gripper_array))
    # print(len(arm.faultstate_array))
    # print(len(controller.tracking_array))
    # print(len(controller.tracking_act_array))
    # print(len(chest.chest_pos_array))
    # print(len(chest_activations))
    # print(len(chest_total_distance))
    # print(len(chest_distance_per_act))
    # print(len(controller.controller_array))
    # print(len(calibration_min))

    df = pd.DataFrame({
                    'date': arrays[0],
                    'elapsed_time': arrays[1],
                    'participant': arrays[2],
                    'control_mode': arrays[3],
                    'n_trial': arrays[4],
                    'ee_rikcs': arrays[5],
                    'ee_gcs': arrays[6],
                    'ee_wcs': arrays[7],
                    'ee': arrays[8],
                    'joint_angles': arrays[9],
                    'gripper_state': arrays[10],
                    'robot_state': arrays[11],
                    'tracking': arrays[12],
                    'n_tracking_act': arrays[13],
                    'chest_position': arrays[14],
                    'chest_activations': arrays[15],
                    'tot_distance_chest': arrays[16],
                    'distance_per_activation': arrays[17],
                    'controller_position': arrays[18],
                    'reachability': arrays[19],
                    'pick_and_place': arrays[20],
                    'calibration_min': arrays[21],
                    'calibration_max': arrays[22]
                    })

    # Export to csv file 
    df.to_csv(path)

def reset_flags():

    # Flags for Controller class
    controller.FirstTime1 = True
    controller.FirstTime2 = True
    controller.FirstTime3 = True
    controller.FirstTime4 = True
    controller.FirstTime5 = True

    # Flags for Arm class
    arm.FirstTime1 = True
    arm.FirstTime2 = True
    arm.FirstTime3 = True
    arm.FirstTime4 = True
    arm.FirstTime5 = True

    # Flags for Chest class
    chest.FirstTime1 = True
    chest.FirstTime2 = True
    chest.FirstTime3 = True

if __name__ == '__main__':
    

    # Initialize the node
    rospy.init_node("data_collection", anonymous=True)

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('-p', '--participant', action='store', default=20, help='participant index (starting from 0)')
    parser.add_argument('-m', '--mode', action='store', default=0, help='chest control mode: 0 - manual, 1 - auto: proximity, 2 - auto: scaling')
    parser.add_argument('-t', '--trial', action='store', default=0, help='1 - first trial, 2 - second trial')

    args = vars(parser.parse_args())

    # 0 - manual, 1 - proximity, 2 - scaling
    CHEST_CONTROL_MODE = int(args['mode'])
    participant_index = str(args['participant'])
    trial = int(args['trial'])

    chest = Chest()
    arm = Arm()
    controller = Controller()

    # Subscribe to topics
    # Joint angles and Gripper value
    rospy.Subscriber('/my_gen3/joint_states', JointState, arm.jointstate_cb)
    # EE position
    rospy.Subscriber('/relaxed_ik/position_wcs', Float32MultiArray, arm.relaxedik_wcs_cb)
    rospy.Subscriber('/relaxed_ik/position_gcs', Float32MultiArray, arm.relaxedik_gcs_cb)
    # Arm errors
    rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, arm.error_cb)
    # Chest position
    rospy.Subscriber('/chest_position', geom_msgs.Point, chest.position_cb)
    rospy.Subscriber('/autonomy_proximity', AutonomyInfo, chest.autonomy_state_cb)
    # Controller
    rospy.Subscriber('rightControllerInfo', ControllerInput, controller.right_callback)
    rospy.Subscriber('/tracking', Bool, controller.tracking_cb)
    # Autonomy and reachability
    rospy.Subscriber('/reachability_intent', Int32, controller.reachability_cb)
    rospy.Subscriber('/pick_place', Int32, controller.pick_place_cb)
    # Arm calibration values for Mode 2
    rospy.Subscriber('/calibration_param', Float32MultiArray, controller.calibr_cb)

    time_array = []
    date_array = []

    start = time.time()
    r = rospy.Rate(2)
    
    # Main loop
    while not rospy.is_shutdown():

        elapsed_time = round(time.time() - start, 2)
        time_array.append(elapsed_time)

        date_array.append(datetime.today().strftime('%Y-%m-%d %H:%M:%S'))

        print()
        print(elapsed_time)

        reset_flags()
        r.sleep()        

    store_data(CHEST_CONTROL_MODE, participant_index, trial, time_array)

    print("Data saved successfully under", 'Data/' + participant_index + '/mode ' + str(CHEST_CONTROL_MODE) + ' - trial ' + str(trial) + '.csv')

        
