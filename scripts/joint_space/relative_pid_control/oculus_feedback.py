#!/usr/bin/env python

import rospy
import transformations as T
import numpy as np
import math

from ROS_TCP_Endpoint_msgs.msg import ControllerInput
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
import gopher_ros_clearcore.msg as clearcore_msg
import gopher_ros_clearcore.srv as clearcore_srv

from utils import *


class Oculus:

    def __init__(self):

        # Initialize dictionary for right controller variables
        self.right_controller = {
            'primaryButton': False,
            'secondaryButton': False,
            'triggerButton': False,
            'triggerValue': 0.0,
            'gripButton': False,
            'joystickButton': False,
            'joystick_pos_x': 0.0,
            'joystick_pos_y': 0.0,
            'controller_pos_x': 0.0,
            'controller_pos_y': 0.0,
            'controller_pos_z': 0.0,
            'controller_rot_w': 0.0,
            'controller_rot_x': 0.0,
            'controller_rot_y': 0.0,
            'controller_rot_z': 0.0
        }

        # Initialize dictionary for left controller variables
        self.left_controller = {
            'primaryButton': False,
            'secondaryButton': False,
            'triggerButton': False,
            'triggerValue': 0.0,
            'gripButton': False,
            'joystickButton': False,
            'joystick_pos_x': 0.0,
            'joystick_pos_y': 0.0,
            'controller_pos_x': 0.0,
            'controller_pos_y': 0.0,
            'controller_pos_z': 0.0,
            'controller_rot_w': 0.0,
            'controller_rot_x': 0.0,
            'controller_rot_y': 0.0,
            'controller_rot_z': 0.0
        }

        # Subscribers
        rospy.Subscriber(
            "rightControllerInfo", ControllerInput, self.right_callback
        )
        rospy.Subscriber(
            "leftControllerInfo", ControllerInput, self.left_callback
        )

        # Publishers
        self.right_grip_button_pub = rospy.Publisher(
            '/right_grip_button',
            Bool,
            queue_size=1,
        )
        self.right_trigger_button_pub = rospy.Publisher(
            '/right_trigger_button',
            Bool,
            queue_size=1,
        )
        self.right_joystick_pos_y_pub = rospy.Publisher(
            '/right_joystick_pos_y',
            Float64,
            queue_size=1,
        )
        self.left_grip_button_pub = rospy.Publisher(
            '/left_grip_button',
            Bool,
            queue_size=1,
        )
        self.left_primary_button_pub = rospy.Publisher(
            '/left_primary_button',
            Bool,
            queue_size=1,
        )
        self.left_secondary_button_pub = rospy.Publisher(
            '/left_secondary_button',
            Bool,
            queue_size=1,
        )
        self.chest_vel_pub = rospy.Publisher(
            'z_chest_vel',
            geom_msgs.Twist,
            queue_size=1,
        )

        self.input_position_gcs_pub = rospy.Publisher(
            '/input_position_gcs',
            Float32MultiArray,
            queue_size=1,
        )

    def right_callback(self, data):
        """Callback function for right controller info. Transform the controller input position from 
        Left-Handed Coordinate system to Right-handed Coordinate system: 
        1) swap y and z axis;
        2) swap x and new y (which was z) to have x facing forward;
        3) negate new y (which is x) to make it align with global coordinate system.
        """

        # Update dictionary
        self.right_controller['primaryButton'] = data.primaryButton
        self.right_controller['secondaryButton'] = data.secondaryButton
        self.right_controller['triggerButton'] = data.triggerButton
        self.right_controller['triggerValue'] = data.triggerValue
        self.right_controller['gripButton'] = data.gripButton
        self.right_controller['joystickButton'] = data.joystickButton
        self.right_controller['joystick_pos_x'] = data.joystick_pos_x
        self.right_controller['joystick_pos_y'] = data.joystick_pos_y
        self.right_controller['controller_pos_x'] = data.controller_pos_x
        self.right_controller['controller_pos_y'] = data.controller_pos_y
        self.right_controller['controller_pos_z'] = data.controller_pos_z
        self.right_controller['controller_rot_w'] = data.controller_rot_w
        self.right_controller['controller_rot_x'] = data.controller_rot_x
        self.right_controller['controller_rot_y'] = data.controller_rot_y
        self.right_controller['controller_rot_z'] = data.controller_rot_z

        publish_bool_message(
            self.right_controller['gripButton'],
            self.right_grip_button_pub,
        )
        publish_bool_message(
            self.right_controller['triggerButton'],
            self.right_trigger_button_pub,
        )
        publish_float64_message(
            self.right_controller['joystick_pos_y'],
            self.right_joystick_pos_y_pub,
        )

        # Transition from Left-handed CS (Unity) to Right-handed CS (Global) for controller position
        input_pos_gcs = np.array(
            [
                -1 * data.controller_pos_z,
                data.controller_pos_x,
                data.controller_pos_y,
            ]
        )

        publish_multiarray_message(
            input_pos_gcs,
            self.input_position_gcs_pub,
        )

        # # ORIENTATION
        # Raw quaternion input (Left-handed CS)
        input_rot_gcs = np.array(
            [
                data.controller_rot_x,
                data.controller_rot_y,
                data.controller_rot_z,
                data.controller_rot_w,
            ]
        )

        # Transition from Left-handed CS (Unity) to Right-handed CS (Global)
        input_rot_gcs = left_to_right_handed(input_rot_gcs)

        # More comfortable position (compensation)
        Qy = T.quaternion_about_axis(
            math.radians(-45),
            (0, 1, 0),
        )
        input_rot_gcs = T.quaternion_multiply(
            input_rot_gcs,
            Qy,
        )

        # Transition from Global CS to Kinova CS: rotate around y and z axis
        self.input_rot_kcs = global_to_kinova(input_rot_gcs)

    # Left controller topic callback function
    def left_callback(self, data):
        """Callback function for left controller info.
        """

        # Update dictionary
        self.left_controller['primaryButton'] = data.primaryButton
        self.left_controller['secondaryButton'] = data.secondaryButton
        self.left_controller['triggerButton'] = data.triggerButton
        self.left_controller['triggerValue'] = data.triggerValue
        self.left_controller['gripButton'] = data.gripButton
        self.left_controller['joystickButton'] = data.joystickButton
        self.left_controller['joystick_pos_x'] = data.joystick_pos_x
        self.left_controller['joystick_pos_y'] = data.joystick_pos_y
        self.left_controller['controller_pos_x'] = data.controller_pos_x
        self.left_controller['controller_pos_y'] = data.controller_pos_y
        self.left_controller['controller_pos_z'] = data.controller_pos_z
        self.left_controller['controller_rot_w'] = data.controller_rot_w
        self.left_controller['controller_rot_x'] = data.controller_rot_x
        self.left_controller['controller_rot_y'] = data.controller_rot_y
        self.left_controller['controller_rot_z'] = data.controller_rot_z

        publish_bool_message(
            self.left_controller['gripButton'],
            self.left_grip_button_pub,
        )
        publish_bool_message(
            self.left_controller['primaryButton'],
            self.left_primary_button_pub,
        )
        publish_bool_message(
            self.left_controller['primaryButton'],
            self.left_secondary_button_pub,
        )