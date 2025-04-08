#!/usr/bin/env python
"""Implements Oculus Quest 2 controller to teleoperation mapping module.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    2. Lorena Genua (lorenagenua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import numpy as np
import transformations
import copy

from std_msgs.msg import (
    Bool,
    Float64,
    String,
)
from geometry_msgs.msg import (Pose)

from std_srvs.srv import (Empty)

from oculus_ros.msg import (ControllerButtons)


class OculusMapping:
    """
    
    """

    def __init__(
        self,
        robot_name,
        controller_side,
        headset_mode,
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        if headset_mode not in ['table', 'head']:
            raise ValueError('headset_mode should be either "table" or "head".')

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

        self.__preset_pose_selection_mode = False
        self.__preset_poses_state_machine_state = 0
        self.__preset_pose_index = 0
        self.__pressed_button = ''
        self.__preset_poses = {
            'none':
                None,
            'home':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/home_pose',
                    Empty,
                ),
            'front_xy':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/front_xy_grasp_pose',
                    Empty,
                ),
            'front_xz':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/front_xz_grasp_pose',
                    Empty,
                ),
            'top_yz':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/top_yz_grasp_pose',
                    Empty,
                ),
            'top_xz':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/top_xz_grasp_pose',
                    Empty,
                ),
            'side_yx':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/side_yx_grasp_pose',
                    Empty,
                ),
            'side_yz':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/side_yz_grasp_pose',
                    Empty,
                ),
            'narrow':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/narrow_pose',
                    Empty,
                ),
            'side':
                rospy.ServiceProxy(
                    f'/{self.ROBOT_NAME}/preset_poses/side_arm_pose',
                    Empty,
                ),
        }
        self.__trajectory_finished = False
        self.__trajectory_fraction = 0.0

        self.__reset_state_machine_state = 0

        # # Public variables:
        self.is_initialized = True

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/oculus_mapping/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'teleoperation': False,
        }

        self.__dependency_status_topics = {
            'teleoperation':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
                    Bool,
                    self.__teleoperation_callback,
                ),
        }

        # # Service provider:

        # # Service subscriber:
        self.__pause_relaxed_ik = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/pause_relaxed_ik',
            Empty,
        )
        self.__resume_relaxed_ik = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/resume_relaxed_ik',
            Empty,
        )
        self.__reset_relaxed_ik = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/reset_relaxed_ik',
            Empty,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/oculus_mapping/is_initialized',
            Bool,
            queue_size=1,
        )

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

        self.__teleoperation_gripper_button_long = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/gripper_button_long',
            Bool,
            queue_size=1,
        )
        self.__teleoperation_mode_button_long = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/mode_button_long',
            Bool,
            queue_size=1,
        )

        self.__preset_pose_mode = rospy.Publisher(
            f'/{self.ROBOT_NAME}/oculus_mapping/preset_pose_mode',
            Bool,
            queue_size=1,
        )
        self.__preset_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/oculus_mapping/preset_pose',
            String,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/pose',
            Pose,
            self.__oculus_pose_callback,
        )
        rospy.Subscriber(
            f'/{self.CONTROLLER_SIDE}/controller_feedback/buttons',
            ControllerButtons,
            self.__oculus_buttons_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/preset_poses/trajectory_finished',
            Bool,
            self.__trajectory_finished_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/preset_poses/trajectory_fraction',
            Float64,
            self.__trajectory_fraction_callback,
        )

    # # Dependency status callbacks:
    def __teleoperation_callback(self, message):
        """Monitors teleoperation is_initialized topic.
        
        """

        self.__dependency_status['teleoperation'] = message.data

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

    def __trajectory_finished_callback(self, message):
        """

        """

        self.__trajectory_finished = message.data

    def __trajectory_fraction_callback(self, message):
        """

        """

        self.__trajectory_fraction = message.data

    # # Private methods:
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (
                            f'/{self.ROBOT_NAME}/oculus_mapping: '
                            f'lost connection to {key}!'
                        )
                    )

                    # # Emergency actions on lost connection:
                    # NOTE (optionally): Add code, which needs to be executed if
                    # connection to any of dependencies was lost.

                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key}...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.ROBOT_NAME}/oculus_mapping:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/oculus_mapping: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __publish_teleoperation_pose(self):
        """
        
        """

        corrected_input_pose = copy.deepcopy(self.__input_pose)

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

        pose_message = Pose()
        pose_message.position.x = corrected_input_pose['position'][0]
        pose_message.position.y = corrected_input_pose['position'][1]
        pose_message.position.z = corrected_input_pose['position'][2]

        pose_message.orientation.w = corrected_input_pose['orientation'][0]
        pose_message.orientation.x = corrected_input_pose['orientation'][1]
        pose_message.orientation.y = corrected_input_pose['orientation'][2]
        pose_message.orientation.z = corrected_input_pose['orientation'][3]

        self.__teleoperation_pose.publish(pose_message)

    def __preset_poses_state_machine(self):
        """
        
        """

        # State: Waiting for preset selection activation.
        if (
            self.__preset_poses_state_machine_state == 0
            and self.__oculus_buttons.primary_button_long
        ):
            self.__preset_poses_state_machine_state = 1

            # rospy.loginfo(
            #     f'/{self.ROBOT_NAME}/oculus_mapping: '
            #     f'in preset pose selection mode.'
            # )

        elif (
            self.__preset_poses_state_machine_state == 1
            and not self.__oculus_buttons.primary_button
        ):
            self.__preset_pose_selection_mode = True
            self.__preset_pose_index = 0
            self.__pressed_button = ''

            rospy.loginfo(
                f'/{self.ROBOT_NAME}/oculus_mapping: '
                f'selected pose: '
                f'{list(self.__preset_poses.keys())[self.__preset_pose_index]}'
            )

            self.__preset_poses_state_machine_state = 2

        # State: Preset selection and confirmation.
        elif (self.__preset_poses_state_machine_state == 2):
            # Next preset (Press secondary button).
            if self.__oculus_buttons.secondary_button:
                self.__pressed_button = 'secondary_button'

                self.__preset_poses_state_machine_state = 3

            # Previous preset (Press primary button).
            elif self.__oculus_buttons.primary_button:
                self.__pressed_button = 'primary_button'

                self.__preset_poses_state_machine_state = 3

        # State: Button was released.
        elif (self.__preset_poses_state_machine_state == 3):
            if self.__oculus_buttons.primary_button_long:
                # rospy.loginfo(
                #     f'/{self.ROBOT_NAME}/oculus_mapping: '
                #     f'confirmed pose: '
                #     f'{list(self.__preset_poses.keys())[self.__preset_pose_index]}'
                # )

                self.__preset_poses_state_machine_state = 4

            # Forward selection.
            elif (
                self.__pressed_button == 'primary_button'
                and not self.__oculus_buttons.primary_button
            ):
                self.__preset_pose_index += 1

                # Loop the selection.
                if self.__preset_pose_index > len(self.__preset_poses) - 1:
                    self.__preset_pose_index = 0

                rospy.loginfo(
                    f'/{self.ROBOT_NAME}/oculus_mapping: '
                    f'selected pose: '
                    f'{list(self.__preset_poses.keys())[self.__preset_pose_index]}'
                )

                self.__pressed_button = ''
                self.__preset_poses_state_machine_state = 2

            # Backward selection.
            elif (
                self.__pressed_button == 'secondary_button'
                and not self.__oculus_buttons.secondary_button
            ):
                self.__preset_pose_index -= 1

                # Loop the selection.
                if self.__preset_pose_index < 0:
                    self.__preset_pose_index = len(self.__preset_poses) - 1

                rospy.loginfo(
                    f'/{self.ROBOT_NAME}/oculus_mapping: '
                    f'selected pose: '
                    f'{list(self.__preset_poses.keys())[self.__preset_pose_index]}'
                )

                self.__pressed_button = ''
                self.__preset_poses_state_machine_state = 2

        # State: Long press button was released, motion has started.
        elif (
            self.__preset_poses_state_machine_state == 4
            and not self.__oculus_buttons.primary_button
        ):
            self.__preset_poses_state_machine_state = 5

            if self.__preset_pose_index == 0:
                return

            # rospy.loginfo(
            #     f'/{self.ROBOT_NAME}/oculus_mapping: '
            #     f'move to confirmed preset pose...'
            # )
            self.__pause_relaxed_ik()

            key = list(self.__preset_poses.keys())[self.__preset_pose_index]
            self.__preset_poses[key]()

        elif (
            self.__preset_poses_state_machine_state == 5
            and self.__trajectory_finished
        ):
            self.__preset_poses_state_machine_state = 0
            self.__preset_pose_selection_mode = False

            if self.__preset_pose_index == 0:
                return

            self.__reset_relaxed_ik()

            # if self.__trajectory_fraction == 1.0:
            #     rospy.loginfo(
            #         f'/{self.ROBOT_NAME}/oculus_mapping: '
            #         f'motion has finished.'
            #     )

            # else:
            #     rospy.logwarn(
            #         f'/{self.ROBOT_NAME}/oculus_mapping: '
            #         f'motion planning has failed.'
            #     )

    def __reset_relaxed_ik_state_machine(self):
        """Resets relaxed IK and clears Kinova faults.
        
        """

        # State 0: Long press secondary button.
        if (
            self.__reset_state_machine_state == 0
            and self.__oculus_buttons.secondary_button_long
        ):
            self.__reset_state_machine_state = 1

            try:
                self.__reset_relaxed_ik()

            except Exception as ex:
                rospy.logerr(
                    f'/{self.ROBOT_NAME}/oculus_mapping: '
                    f'\nError calling self.__reset_relaxed_ik(): {ex}'
                )

        # State 1: Release secondary button.
        elif (
            self.__reset_state_machine_state == 1
            and not self.__oculus_buttons.secondary_button_long
        ):
            self.__reset_state_machine_state = 0

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            self.__reset_relaxed_ik_state_machine()
            return

        self.__preset_poses_state_machine()

        self.__preset_pose_mode.publish(self.__preset_pose_selection_mode)
        self.__preset_pose.publish(
            list(self.__preset_poses.keys())[self.__preset_pose_index]
        )

        if self.__preset_pose_selection_mode:
            return

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

        self.__teleoperation_gripper_button_long.publish(
            self.__oculus_buttons.trigger_button_long
        )
        self.__teleoperation_mode_button_long.publish(
            self.__oculus_buttons.primary_button_long
        )

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/oculus_mapping: node is shutting down...',
        )

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/oculus_mapping: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'oculus_mapping',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    headset_mode = rospy.get_param(
        param_name=f'{rospy.get_name()}/headset_mode',
        default='table',
    )

    oculus_kinova_mapping = OculusMapping(
        robot_name=kinova_name,
        controller_side=controller_side,
        headset_mode=headset_mode,
    )

    rospy.on_shutdown(oculus_kinova_mapping.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        oculus_kinova_mapping.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
