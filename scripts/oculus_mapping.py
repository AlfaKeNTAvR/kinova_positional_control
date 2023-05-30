#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations

from std_msgs.msg import (Bool)
from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (ControllerButtons)


class OculusMapping:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        controller_side='right',
        headset_mode='table',
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.CONTROLLER_SIDE = controller_side
        self.HEADSET_MODE = headset_mode

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.__oculus_buttons = ControllerButtons()

        # # Public variables:

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__teleoperation_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/input_pose',
            Pose,
            queue_size=1,
        )
        self.__teleoperation_tracking_button = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/tracking_button',
            Bool,
            queue_size=1,
        )
        self.__teleoperation_gripper_button = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/gripper_button',
            Bool,
            queue_size=1,
        )
        self.__teleoperation_mode_button = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/mode_button',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/oculus/pose',
            Pose,
            self.__oculus_pose_callback,
        )
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/oculus/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_pose_callback(self, message):
        """

        """

        self.__input_pose['position'][0] = message.position.x
        self.__input_pose['position'][1] = message.position.y
        self.__input_pose['position'][2] = message.position.z

        self.__input_pose['orientation'][0] = message.orientation.w
        self.__input_pose['orientation'][1] = message.orientation.x
        self.__input_pose['orientation'][2] = message.orientation.y
        self.__input_pose['orientation'][3] = message.orientation.z

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

    # # Private methods:
    def __publish_teleoperation_pose(self):
        """
        
        """

        corrected_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        corrected_input_pose['position'] = (
            self.__input_pose['position'].copy()
        )
        corrected_input_pose['orientation'] = (
            self.__input_pose['orientation'].copy()
        )

        # # STEP 1: Table or Head mode correction.
        # If the headset is located on table invert position X and Y axis,
        # rotate orientation quaternion by 180 degrees around Z.
        if self.HEADSET_MODE == 'table':
            corrected_input_pose['position'][0] = (
                -1 * self.__input_pose['position'][0]
            )
            corrected_input_pose['position'][1] = (
                -1 * self.__input_pose['position'][1]
            )

            corrected_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_about_axis(
                        np.deg2rad(180),
                        (0, 0, 1),
                    ),
                    corrected_input_pose['orientation'],
                )
            )

        # # STEP 2: Comfortable orientation correction.
        # TODO:

        pose_message = Pose()
        pose_message.position.x = corrected_input_pose['position'][0]
        pose_message.position.y = corrected_input_pose['position'][1]
        pose_message.position.z = corrected_input_pose['position'][2]

        pose_message.orientation.w = corrected_input_pose['orientation'][0]
        pose_message.orientation.x = corrected_input_pose['orientation'][1]
        pose_message.orientation.y = corrected_input_pose['orientation'][2]
        pose_message.orientation.z = corrected_input_pose['orientation'][3]

        self.__teleoperation_pose.publish(pose_message)

    # # Public methods:
    def main_loop(self):
        """
        
        """
        self.__publish_teleoperation_pose()
        self.__teleoperation_tracking_button.publish(
            self.__oculus_buttons.grip_button
        )
        self.__teleoperation_gripper_button.publish(
            self.__oculus_buttons.trigger_button
        )
        self.__teleoperation_mode_button.publish(
            self.__oculus_buttons.primary_button
        )

    def node_shutdown(self):
        """
        
        """

        print(
            f'\n/{self.ROBOT_NAME}/oculus_mapping: node is shutting down...\n'
        )

        print(f'\n/{self.ROBOT_NAME}/oculus_mapping: node is shut down.\n')


def main():
    """

    """

    rospy.init_node('oculus_kinova_mapping')

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    oculus_kinova_mapping = OculusMapping(
        robot_name=kinova_name,
        controller_side=controller_side,
        headset_mode='table',
    )

    rospy.on_shutdown(oculus_kinova_mapping.node_shutdown)

    print(f'\n/{kinova_name}/oculus_mapping: ready.\n')

    while not rospy.is_shutdown():
        oculus_kinova_mapping.main_loop()


if __name__ == '__main__':
    main()
