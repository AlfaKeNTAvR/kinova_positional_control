#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations

from geometry_msgs.msg import (Pose)

from oculus.msg import (ControllerButtons)
from kinova_positional_control.srv import (
    GripperForceGrasping,
    GripperPosition,
)


class OculusKinovaMapping:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        controller_side='right',
        tracking_mode='press',
        headset_mode='table',
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        if tracking_mode not in ['hold', 'press']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.CONTROLLER_SIDE = controller_side
        self.TRACKING_MODE = tracking_mode
        self.HEADSET_MODE = headset_mode

        # # Private variables:
        self.__oculus_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.__oculus_buttons = ControllerButtons()

        self.__tracking_state_machine_state = 0
        self.__gripper_state_machine_state = 0
        self.__mode_state_machine_state = 0
        self.__control_mode = 'position'

        # # Public variables:
        self.pose_tracking = False

        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_relaxed_ik_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # This difference is calculated each time the tracking is started and
        # subracted from future inputs during current tracking to compensate for
        # linear misalignment between the global and relaxed_ik coordinate
        # systems.
        self.oculus_relaxed_ik_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # ROS node:

        # # Service provider:

        # # Service subscriber:
        self.__gripper_force_grasping = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/gripper/force_grasping',
            GripperForceGrasping,
        )
        self.__gripper_position = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/gripper/position',
            GripperPosition,
        )

        # # Topic publisher:
        self.__kinova_pose = rospy.Publisher(
            f'{self.ROBOT_NAME}/input_pose',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/pose',
            Pose,
            self.__oculus_pose_callback,
        )
        rospy.Subscriber(
            f'oculus/{self.CONTROLLER_SIDE}/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )
        rospy.Subscriber(
            'relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __oculus_pose_callback(self, message):
        """

        """

        negation = 1

        if self.HEADSET_MODE == 'head':
            negation = -1

        self.__oculus_pose['position'][0] = negation * message.position.x
        self.__oculus_pose['position'][1] = negation * message.position.y
        self.__oculus_pose['position'][2] = message.position.z

        self.__oculus_pose['position'][0] = message.position.x
        self.__oculus_pose['position'][1] = message.position.y
        self.__oculus_pose['position'][2] = message.position.z

        self.__oculus_pose['orientation'][0] = message.orientation.w
        self.__oculus_pose['orientation'][1] = message.orientation.x
        self.__oculus_pose['orientation'][2] = message.orientation.y
        self.__oculus_pose['orientation'][3] = message.orientation.z

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

        if self.TRACKING_MODE == 'hold':
            self.tracking = self.__oculus_buttons.grip_button

    def __commanded_pose_callback(self, message):
        """
        
        """

        self.last_relaxed_ik_pose['position'][0] = message.position.x
        self.last_relaxed_ik_pose['position'][1] = message.position.y
        self.last_relaxed_ik_pose['position'][2] = message.position.z

        self.last_relaxed_ik_pose['orientation'][0] = message.orientation.w
        self.last_relaxed_ik_pose['orientation'][1] = message.orientation.x
        self.last_relaxed_ik_pose['orientation'][2] = message.orientation.y
        self.last_relaxed_ik_pose['orientation'][3] = message.orientation.z

    # # Private methods:
    def __tracking_state_machine(self, button):
        """
        
        """

        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):
            self.__tracking_state_machine_state = 1

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):
            self.__tracking_state_machine_state = 2

            self.__calculate_compensation()
            self.pose_tracking = True

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):
            self.__tracking_state_machine_state = 3

            self.pose_tracking = False

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):
            self.__tracking_state_machine_state = 0

    def __mode_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):
            self.__mode_state_machine_state = 1

            self.__control_mode = 'full'
            self.__calculate_compensation()

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):
            self.__mode_state_machine_state = 3

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 3 and button):
            self.__mode_state_machine_state = 4

            self.__control_mode = 'position'

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 4 and not button):
            self.__mode_state_machine_state = 0

    def __gripper_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__gripper_state_machine_state == 0 and button):
            self.__gripper_force_grasping(0.0)  # 0.0 for default current.
            self.__gripper_state_machine_state = 1

        # State 1: Button was released. Force grasping is activated.
        elif (self.__gripper_state_machine_state == 1 and not button):
            self.__gripper_state_machine_state = 2

        # State 2: Button was pressed. Open the gripper.
        elif (self.__gripper_state_machine_state == 2 and button):
            self.__gripper_position(0.0)  # 0.0 for open position.
            self.__gripper_state_machine_state = 3

        # State 3: Button was released.
        elif (self.__gripper_state_machine_state == 3 and not button):
            self.__gripper_state_machine_state = 0

    def __calculate_compensation(self):
        """Calculates the compensation for coordinate systems misalignment.
        
        """

        self.oculus_relaxed_ik_difference['position'] = (
            self.__oculus_pose['position']
            - self.last_relaxed_ik_pose['position']
        )
        self.oculus_relaxed_ik_difference['orientation'] = (
            transformations.quaternion_multiply(
                self.last_relaxed_ik_pose['orientation'],
                transformations.quaternion_inverse(
                    self.__oculus_pose['orientation']
                ),
            )
        )

    # # Public methods:
    def main_loop(self):
        """
        
        """

        if self.TRACKING_MODE == 'press':
            self.__tracking_state_machine(self.__oculus_buttons.grip_button)

        if self.pose_tracking:
            self.publish_kinova_pose()

        self.__gripper_state_machine(self.__oculus_buttons.trigger_button)
        self.__mode_state_machine(self.__oculus_buttons.primary_button)

    def publish_kinova_pose(self):
        """
        
        """

        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        compensated_input_pose['position'] = (
            self.__oculus_pose['position']
            - self.oculus_relaxed_ik_difference['position']
        )

        # Use fixed orientation.
        compensated_input_pose['orientation'] = (
            self.last_relaxed_ik_pose['orientation']
        )

        # Use oculus orientation.
        if self.__control_mode == 'full':
            compensated_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    self.oculus_relaxed_ik_difference['orientation'],
                    self.__oculus_pose['orientation'],
                )
            )

        pose_message = Pose()
        pose_message.position.x = compensated_input_pose['position'][0]
        pose_message.position.y = compensated_input_pose['position'][1]
        pose_message.position.z = compensated_input_pose['position'][2]

        pose_message.orientation.w = compensated_input_pose['orientation'][0]
        pose_message.orientation.x = compensated_input_pose['orientation'][1]
        pose_message.orientation.y = compensated_input_pose['orientation'][2]
        pose_message.orientation.z = compensated_input_pose['orientation'][3]

        self.__kinova_pose.publish(pose_message)


def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    # TODO: Stop arm motion.

    print('\nNode is shut down.\n')


def main():
    """

    """

    # # ROS node:
    rospy.init_node('oculus_kinova_mapping')
    rospy.on_shutdown(node_shutdown)

    right_arm_mapping = OculusKinovaMapping(
        robot_name='my_gen3',
        controller_side='right',
        tracking_mode='press',
        headset_mode='table',
    )

    print('\nOculus-Kinova mapping is ready.\n')

    while not rospy.is_shutdown():
        right_arm_mapping.main_loop()


if __name__ == '__main__':
    main()
