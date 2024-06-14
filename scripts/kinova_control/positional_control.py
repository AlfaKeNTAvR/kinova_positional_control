#!/usr/bin/env python
"""Implements Kinova Gen3 positional control module.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import math
import numpy as np
import transformations
import copy
from ast import (literal_eval)

from std_msgs.msg import (Bool)
from geometry_msgs.msg import (Pose)

from relaxed_ik_ros1.msg import (EEPoseGoals)


class KinovaPositionalControl:
    """
    
    """

    def __init__(
        self,
        robot_name,
        mounting_angles_deg,
        starting_pose,
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name

        # Rotation matrices around axes. If the arm is mounted on the table,
        # mounting angles should all be zeros.
        self.ROTATE_X = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[0]),
            (1, 0, 0),
        )
        self.ROTATE_Y = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[1]),
            (0, 1, 0),
        )
        self.ROTATE_Z = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[2]),
            (0, 0, 1),
        )

        # Rotation matrices from Global Coordinate System (parallel to the
        # floor, X facing forward globally) to Relaxed IK Coordinate System
        # (parallel to Kinova Arm base, X facing according to Kinova Arm) and
        # back. If the arm is mounted on the table, no rotations will be
        # applied.
        self.ROTATE_GCS_TO_RIKCS = transformations.concatenate_matrices(
            self.ROTATE_X,
            self.ROTATE_Y,
            self.ROTATE_Z,
        )
        self.ROTATE_RIKCS_TO_GCS = transformations.inverse_matrix(
            self.ROTATE_GCS_TO_RIKCS
        )

        self.STARTING_POSE = starting_pose
        self.STARTING_POSE['position'] = (
            np.array(self.STARTING_POSE['position'])
        )
        self.STARTING_POSE['orientation'] = (
            np.array(
                transformations.quaternion_from_euler(
                    np.deg2rad(self.STARTING_POSE['orientation'][0]),
                    np.deg2rad(self.STARTING_POSE['orientation'][1]),
                    np.deg2rad(self.STARTING_POSE['orientation'][2]),
                )
            )
        )

        # # Private variables:
        self.__is_homed = False

        # Input pose in Global and Relaxed IK coordinate systems.
        self.__input_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }

        # Last commanded Relaxed IK pose.
        self.__last_relaxed_ik_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'rikcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                }
        }

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'relaxed_ik': False,
        }

        self.__dependency_status_topics = {
            'relaxed_ik':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/relaxed_ik/is_initialized',
                    Bool,
                    self.__relaxed_ik_callback,
                ),
        }

        # # Service provider:

        # # Service subscriber:
        # self.__stop_arm = rospy.ServiceProxy(
        #     f'/{self.ROBOT_NAME}/base/stop',
        #     Stop,
        # )

        # # Topic publisher:
        self.__relaxed_ik_target_rikcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            queue_size=1,
        )
        self.__relaxed_ik_commanded_gcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/positional_control/input_pose',
            Pose,
            self.__input_pose_callback,
        )

    # # Dependency status callbacks:
    def __relaxed_ik_callback(self, msg):
        """
        
        """

        self.__dependency_status['relaxed_ik'] = msg.data

    # # Service handlers:

    # # Topic callbacks:
    def __input_pose_callback(self, msg):
        """
        
        """

        self.__input_pose['gcs']['position'][0] = msg.position.x
        self.__input_pose['gcs']['position'][1] = msg.position.y
        self.__input_pose['gcs']['position'][2] = msg.position.z

        self.__input_pose['gcs']['orientation'][0] = msg.orientation.w
        self.__input_pose['gcs']['orientation'][1] = msg.orientation.x
        self.__input_pose['gcs']['orientation'][2] = msg.orientation.y
        self.__input_pose['gcs']['orientation'][3] = msg.orientation.z

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
                            f'/{self.ROBOT_NAME}/positional_control: '
                            f'lost connection to {key}!'
                        )
                    )

                    # # Emergency actions on lost connection:
                    # NOTE: Add code, which needs to be executed if connection
                    # to any of dependencies was lost.

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
                    f'/{self.ROBOT_NAME}/positional_control:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized and self.__is_homed):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/positional_control: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __compose_pose_message(self, target_pose):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        # NOTE: These two checks might not be needed, check function usage.
        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'key {key} not found in target_pose.')

        pose_message = Pose()
        pose_message.position.x = target_pose['position'][0]
        pose_message.position.y = target_pose['position'][1]
        pose_message.position.z = target_pose['position'][2]

        pose_message.orientation.w = target_pose['orientation'][0]
        pose_message.orientation.x = target_pose['orientation'][1]
        pose_message.orientation.y = target_pose['orientation'][2]
        pose_message.orientation.z = target_pose['orientation'][3]

        return pose_message

    def __homing(self):
        """
        
        """

        # Wait for dependencies to initialize.
        if not self.__dependency_initialized:
            return

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: dependencies have initialized.',
        )

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: homing has started...',
        )

        # Let the node get initialized.
        rospy.sleep(2)

        # Starting pose.
        self.__input_pose['gcs'] = copy.deepcopy(self.STARTING_POSE)
        self.__set_target_pose(self.__input_pose['gcs'], 'gcs')

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: homing has finished.',
        )

        self.__is_homed = True

    def __set_target_pose(self, target_pose, coordinate_system):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'Key {key} was not found in target_pose.')

            if not isinstance(target_pose[key], np.ndarray):
                raise TypeError(
                    'Dictionary values should be of type np.ndarray.'
                )

        # Check if coordinates are within the arm's workspace.
        # TODO: Which coordinates system: GCS or RIKCS?
        # target_pose['position'] = self.__check_boundaries(
        #     target_pose['position']
        # )

        self.__last_relaxed_ik_pose['gcs'] = copy.deepcopy(target_pose)
        self.__last_relaxed_ik_pose['rikcs'] = copy.deepcopy(target_pose)

        if coordinate_system == 'gcs':
            # Update target pose in Relaxed IK CS.
            self.__last_relaxed_ik_pose['rikcs']['position'] = np.matmul(
                self.ROTATE_GCS_TO_RIKCS[0:3, 0:3],
                target_pose['position'],
            )
            self.__last_relaxed_ik_pose['rikcs']['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_from_matrix(
                        self.ROTATE_GCS_TO_RIKCS
                    ),
                    target_pose['orientation'],
                ),
            )[0]

        # TODO: Test this segment.
        elif coordinate_system == 'rikcs':
            raise ValueError(
                'Setting target orientation in RIKCS was not tested yet.'
            )

            # Update target pose in Global IK CS.
            self.__last_relaxed_ik_pose['gcs']['position'] = np.matmul(
                self.ROTATE_RIKCS_TO_GCS[0:3, 0:3],
                target_pose['position'],
            )
            self.__last_relaxed_ik_pose['gcs']['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_from_matrix(
                        self.ROTATE_RIKCS_TO_GCS
                    ),
                    target_pose['orientation'],
                ),
            )[0]

        else:
            raise ValueError('Invalid coordinate_system value.')

        # Form a message for the right arm.
        right_arm_pose_rikcs = self.__compose_pose_message(
            self.__last_relaxed_ik_pose['rikcs']
        )

        # TODO: Add left arm support.
        # Form a message for the left arm.
        left_arm_pose_rikcs = self.__compose_pose_message(
            self.__last_relaxed_ik_pose['rikcs']
        )

        # Form a message for the relaxed IK setpoint topic.
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(right_arm_pose_rikcs)
        ee_pose_goals.ee_poses.append(left_arm_pose_rikcs)
        ee_pose_goals.header.seq = 0

        self.__relaxed_ik_target_rikcs.publish(ee_pose_goals)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_homed:
            self.__homing()

        if not self.__is_initialized:
            return

        self.__set_target_pose(self.__input_pose['gcs'], 'gcs')

        # Publish a commanded target position in Global CS.
        self.__relaxed_ik_commanded_gcs.publish(
            self.__compose_pose_message(self.__last_relaxed_ik_pose['gcs'])
        )

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/positional_control: node is shutting down...',
        )

        # Stop the arm motion.
        # self.__stop_arm()

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/positional_control: node has shut down.',
        )


def main():
    """
    
    """

    rospy.init_node(
        'positional_control',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    # TODO: Add type check.
    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    mounting_angles_deg = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/mounting_angles_deg',
            default='[0.0, 0.0, 0.0]',
        )
    )

    starting_pose = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/starting_pose',
            default=
            "{'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}"
        )
    )

    pose_controller = KinovaPositionalControl(
        robot_name=kinova_name,
        mounting_angles_deg=mounting_angles_deg,
        starting_pose=starting_pose,
    )

    rospy.on_shutdown(pose_controller.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        pose_controller.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
