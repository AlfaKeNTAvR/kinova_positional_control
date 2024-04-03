#!/usr/bin/env python
"""Implements Kinova Gen3 positional control module.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

TODO: 
    1. Fix Z jump when enabling and disabling chest_z_compensation. 

"""

import rospy
import math
import numpy as np
import transformations
import copy
from ast import (literal_eval)
from threading import (Timer)

from std_msgs.msg import (
    Bool,
    Float32,
)
from std_srvs.srv import (SetBool)
from geometry_msgs.msg import (Pose)
from sensor_msgs.msg import (JointState)

from kortex_driver.msg import (
    Twist,
    TwistCommand,
)
from kortex_driver.srv import (Stop)
from kinova_positional_control.srv import (PidVelocityLimit)
from relaxed_ik_ros1.msg import (EEPoseGoals)


class KinovaPositionalControl:
    """
    
    """

    def __init__(
        self,
        robot_name,
        mounting_angles_deg,
        safe_homing_z,
        starting_pose,
        enable_chest_compensation,
    ):
        """
        
        """

        # # Private constants:
        self.__RELAXED_IK_STARTING_CONFIG = (
            np.array([0.0, 0.2619, 3.1415, -2.2690, 0.0, 0.9598, 1.5707])
        )
        self.__ENABLE_CHEST_COMPENSATION = enable_chest_compensation

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

        self.SAFE_HOMING_Z = safe_homing_z
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
        self.__is_motion_finished = True

        # Homing timeout timer.
        self.__homing_timeout = {
            'timer': None,
            'is_timer_running': False,
            'is_timed_out': False,
        }

        # Input pose in Global and Relaxed IK coordinate systems.
        self.__input_pose = {
            'wcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }

        # Last commanded Relaxed IK pose.
        self.__last_relaxed_ik_pose = {
            'wcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
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

        # Forward Kinematics pose.
        self.__forward_kinematics_pose = {
            'wcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'rikcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'kcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }
        self.__kinova_joint_positions_feedback = np.zeros(7)

        # Calculate screw axes:
        w = np.zeros([3, 7])
        v = np.zeros([3, 7])

        w[:, 1] = np.array([0, 1, 0])
        v[:, 1] = (
            -self.__calculate_skew_symmetric_matrix(w[:, 1])
            @ np.array([0, 0, (156.4 + 128.4) / 1000])
        )

        w[:, 2] = np.array([0, 0, -1])
        v[:, 2] = (
            -self.__calculate_skew_symmetric_matrix(w[:, 2])
            @ np.array([0, -(5.4 + 6.4) / 1000, 0])
        )

        w[:, 3] = np.array([0, 1, 0])
        v[:, 3] = (
            -self.__calculate_skew_symmetric_matrix(w[:, 3])
            @ np.array([0, 0, (156.4 + 128.4 + 210.4 + 210.4) / 1000])
        )

        w[:, 4] = np.array([0, 0, -1])
        v[:, 4] = (
            -self.__calculate_skew_symmetric_matrix(w[:, 4])
            @ np.array([0, -(5.4 + 6.4 + 6.4 + 6.4) / 1000, 0])
        )

        w[:, 5] = np.array([0, 1, 0])
        v[:, 5] = (
            -self.__calculate_skew_symmetric_matrix(w[:, 5]) @ np.array(
                [0, 0, (156.4 + 128.4 + 210.4 + 210.4 + 208.4 + 105.9) / 1000]
            )
        )

        w[:, 6] = np.array([0, 0, -1])
        v[:, 6] = (
            (
                -self.__calculate_skew_symmetric_matrix(w[:, 6])
                @ np.array([0, -(5.4 + 6.4 + 6.4 + 6.4) / 1000, 0])
            )
        )

        self.__screw_axes = np.zeros([6, 7])
        self.__screw_axes[:, 0] = np.array([0, 0, -1, 0, 0, 0])

        for i in range(1, 7):
            self.__screw_axes[:, i] = np.concatenate((w[:, i], v[:, i]))

        # Home configuration matrix.
        self.__home_configuration = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, -0.025],
                [0, 0, 1, 1.1873 + 0.120],  # 0.120 for the gripper.
                [0, 0, 0, 1],
            ]
        )

        # From Kinova CS (KCS) to Relaxed IK CS (RIKCS) for pose misalignment
        # calculation.
        self.__kcs_rikcs_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # Forward Kinematics solution in Relaxed IK starting configuration,
        # where Relaxed IK pose is [0, 0, 0], [1, 0, 0, 0].
        forward_kinematics = self.__forward_kinematics(
            self.__screw_axes,
            self.__home_configuration,
            self.__RELAXED_IK_STARTING_CONFIG,
        )

        self.__kcs_rikcs_difference['position'] = (
            forward_kinematics[0:3, 3] - np.array([0, 0, 0])
        )
        self.__kcs_rikcs_difference['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_inverse(
                    transformations.quaternion_from_matrix(forward_kinematics)
                ),
                np.array([1, 0, 0, 0]),
            )
        )

        # Forward Kinematics - Relaxed IK pose misalignment in RIKCS.
        self.__kinova_relaxed_ik_misalignment = {
            'wcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0])
                },
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0])
                },
        }

        # Chest compensation:
        self.__chest_position = 0.0

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
            'joints_control': False,
            'relaxed_ik': False,
        }

        self.__dependency_status_topics = {
            'joints_control':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/joints_control/is_initialized',
                    Bool,
                    self.__joints_control_callback,
                ),
            'relaxed_ik':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/relaxed_ik/is_initialized',
                    Bool,
                    self.__relaxed_ik_callback,
                ),
        }

        if self.__ENABLE_CHEST_COMPENSATION:
            self.__dependency_status['chest_control'] = False
            self.__dependency_status_topics['chest_control'] = (
                rospy.Subscriber(
                    '/chest_control/is_initialized',
                    Bool,
                    self.__chest_control_callback,
                )
            )

        # # Service provider:

        # # Service subscriber:
        self.__pid_velocity_limit = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/joints_control/velocity_limit',
            PidVelocityLimit,
        )
        self.__enable_pid = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/joints_control/enable_pid',
            SetBool,
        )

        self.__stop_arm = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/stop',
            Stop,
        )

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
        self.__commanded_pose_wcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_wcs',
            Pose,
            queue_size=1,
        )
        self.__kinova_forward_kinematics_gcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/forward_kinematics_gcs',
            Pose,
            queue_size=1,
        )
        self.__kinova_forward_kinematics_wcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/forward_kinematics_wcs',
            Pose,
            queue_size=1,
        )

        self.__kinova_cartesian_velocity = rospy.Publisher(
            f'/{self.ROBOT_NAME}/in/cartesian_velocity',
            TwistCommand,
            queue_size=1,
        )

        self.__misalignment = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/kinova_relaxed_ik_missalignment',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/positional_control/input_pose',
            Pose,
            self.__input_pose_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joints_control/motion_finished',
            Bool,
            self.__pid_motion_finished_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/base_feedback/joint_state',
            JointState,
            self.__joint_state_callback,
        )

        rospy.Subscriber(
            '/chest_logger/current_position',
            Float32,
            self.__chest_position_callback,
        )

    # # Dependency status callbacks:
    def __joints_control_callback(self, msg):
        """
        
        """

        self.__dependency_status['joints_control'] = msg.data

    def __relaxed_ik_callback(self, msg):
        """
        
        """

        self.__dependency_status['relaxed_ik'] = msg.data

    def __chest_control_callback(self, msg):
        """
        
        """

        self.__dependency_status['chest_control'] = msg.data

    # # Service handlers:

    # # Topic callbacks:
    def __input_pose_callback(self, msg):
        """
        
        """

        self.__input_pose['wcs']['position'][0] = msg.position.x
        self.__input_pose['wcs']['position'][1] = msg.position.y
        self.__input_pose['wcs']['position'][2] = msg.position.z

        self.__input_pose['wcs']['orientation'][0] = msg.orientation.w
        self.__input_pose['wcs']['orientation'][1] = msg.orientation.x
        self.__input_pose['wcs']['orientation'][2] = msg.orientation.y
        self.__input_pose['wcs']['orientation'][3] = msg.orientation.z

    def __pid_motion_finished_callback(self, msg):
        """
        
        """

        if not self.__is_initialized:
            self.joint_control_initialized = True

        self.__is_motion_finished = msg.data

    def __joint_state_callback(self, message):
        """
        
        """

        self.__kinova_joint_positions_feedback = message.position[0:7]

    def __chest_position_callback(self, message):
        """

        """

        self.__chest_position = message.data

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

    def __wait_for_motion(self):
        """Blocks code execution until the flag is set or a node is shut down.
        
        """

        rospy.sleep(1)  # Allow a motion to start.

        while not self.__is_motion_finished and not rospy.is_shutdown():
            pass

    def __publish_cartesian_z_velocity(self, z_velocity):
        """
        
        """

        twist_message = Twist()
        twist_message.linear_x = 0.0
        twist_message.linear_y = 0.0
        twist_message.linear_z = z_velocity
        twist_message.angular_x = 0.0
        twist_message.angular_y = 0.0
        twist_message.angular_z = 0.0

        cartesian_velocity_message = TwistCommand()
        cartesian_velocity_message.reference_frame = 0
        cartesian_velocity_message.twist = twist_message
        cartesian_velocity_message.duration = 0

        self.__kinova_cartesian_velocity.publish(cartesian_velocity_message)

    def __homing_timeout_timer(self, timeout=5):
        """
          
        """

        # No timer was
        if not self.__homing_timeout['is_timer_running']:

            # Cancel any running timmers and start a new one.
            if self.__homing_timeout['timer']:
                self.__homing_timeout['timer'].cancel()

            self.__homing_timeout['timer'] = Timer(
                timeout,
                self.__set_timeout,
            )
            self.__homing_timeout['timer'].start()
            self.__homing_timeout['is_timer_running'] = True
            self.__homing_timeout['is_timed_out'] = False

    def __set_timeout(self):
        """

        """

        self.__homing_timeout['is_timed_out'] = True

        rospy.logwarn(
            f'/{self.ROBOT_NAME}/positional_control: '
            'safe Z homing timed out!\n'
            f'- Current Z position: {round(self.__forward_kinematics_pose["kcs"]["position"][2], 3)}\n'
            f'- Target (safe) Z position: {round(self.SAFE_HOMING_Z, 3)}\n'
        )

    def __homing(self):
        """
        
        """

        # Wait for dependencies to initialize.
        if not self.__dependency_initialized:
            return

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: dependencies have initialized.',
        )

        # Move to a safe Z position before homing.
        # TODO: Add upper limit.
        if self.SAFE_HOMING_Z > 0:
            # Disable PID joints control to use kinova cartesian velocity.
            self.__enable_pid(False)

            rospy.logwarn(
                f'/{self.ROBOT_NAME}/positional_control: '
                'moving to safe Z before homing...\n'
                f'- Current Z position: {round(self.__forward_kinematics_pose["kcs"]["position"][2], 3)}\n'
                f'- Target (safe) Z position: {round(self.SAFE_HOMING_Z, 3)}\n'
            )

            if (
                self.SAFE_HOMING_Z >
                self.__forward_kinematics_pose['kcs']['position'][2]
            ):
                while (
                    self.__forward_kinematics_pose['kcs']['position'][2] <
                    self.SAFE_HOMING_Z
                ):
                    self.__publish_cartesian_z_velocity(0.05)
                    self.__homing_timeout_timer(5)

                    if self.__homing_timeout['is_timed_out']:
                        break

            elif (
                self.SAFE_HOMING_Z <
                self.__forward_kinematics_pose['kcs']['position'][2]
            ):
                while (
                    self.__forward_kinematics_pose['kcs']['position'][2] >
                    self.SAFE_HOMING_Z
                ):
                    self.__publish_cartesian_z_velocity(-0.05)
                    self.__homing_timeout_timer(5)

                    if self.__homing_timeout['is_timed_out']:
                        break

            self.__publish_cartesian_z_velocity(0.0)
            rospy.loginfo(f'/{self.ROBOT_NAME}/positional_control: at safe Z.',)
            self.__enable_pid(True)

        # Limit joint velocities to 20% for homing.
        self.__pid_velocity_limit(0.2)

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: homing has started...',
        )

        # Let the node get initialized.
        rospy.sleep(2)

        # Starting pose.
        self.__input_pose['gcs'] = copy.deepcopy(self.STARTING_POSE)
        self.__input_pose['wcs'] = copy.deepcopy(self.STARTING_POSE)

        if self.__ENABLE_CHEST_COMPENSATION:
            self.__input_pose['wcs']['position'][2] += (self.__chest_position)

        self.__set_target_pose(self.__input_pose['gcs'], 'gcs')
        self.__wait_for_motion()

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: homing has finished.',
        )

        self.__pid_velocity_limit(1.0)

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

        # GCS is used for initial homing to calculate WCS (WCS_Z = GCS_Z +
        # Chest_Z).
        if coordinate_system == 'gcs':

            # Initialize GCS and WCS.
            self.__last_relaxed_ik_pose['gcs'] = copy.deepcopy(target_pose)
            self.__last_relaxed_ik_pose['wcs'] = copy.deepcopy(target_pose)

            if self.__ENABLE_CHEST_COMPENSATION:
                self.__last_relaxed_ik_pose['wcs']['position'][2] = (
                    self.__last_relaxed_ik_pose['wcs']['position'][2]
                    + self.__chest_position
                )

            # Convert into RIKCS.
            self.__last_relaxed_ik_pose['rikcs']['position'] = np.matmul(
                self.ROTATE_GCS_TO_RIKCS[0:3, 0:3],
                self.__last_relaxed_ik_pose['gcs']['position'],
            )

            self.__last_relaxed_ik_pose['rikcs']['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_from_matrix(
                        self.ROTATE_GCS_TO_RIKCS
                    ),
                    self.__last_relaxed_ik_pose['gcs']['orientation'],
                ),
            )[0]

        # WCS is used for all inputs except for the homing phase.
        elif coordinate_system == 'wcs':
            # Initialize GCS and WCS.
            self.__last_relaxed_ik_pose['gcs'] = copy.deepcopy(target_pose)
            self.__last_relaxed_ik_pose['wcs'] = copy.deepcopy(target_pose)

            if self.__ENABLE_CHEST_COMPENSATION:
                # Update GCS.
                self.__last_relaxed_ik_pose['gcs']['position'][2] = (
                    self.__last_relaxed_ik_pose['wcs']['position'][2]
                    - self.__chest_position
                )

            # Convert into RIKCS.
            self.__last_relaxed_ik_pose['rikcs']['position'] = np.matmul(
                self.ROTATE_GCS_TO_RIKCS[0:3, 0:3],
                self.__last_relaxed_ik_pose['gcs']['position'],
            )

            self.__last_relaxed_ik_pose['rikcs']['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_from_matrix(
                        self.ROTATE_GCS_TO_RIKCS
                    ),
                    self.__last_relaxed_ik_pose['gcs']['orientation'],
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

    def __publish_kinova_relaxed_ik_misalignment(self):
        """
        
        """

        # GCS:
        self.__kinova_relaxed_ik_misalignment['gcs']['position'] = np.round(
            self.__last_relaxed_ik_pose['gcs']['position']
            - self.__forward_kinematics_pose['gcs']['position'],
            3,
        )
        self.__kinova_relaxed_ik_misalignment['gcs']['orientation'] = np.round(
            transformations.quaternion_multiply(
                transformations.quaternion_inverse(
                    self.__last_relaxed_ik_pose['gcs']['orientation']
                ),
                self.__forward_kinematics_pose['gcs']['orientation'],
            ),
            3,
        )

        # WCS:
        self.__kinova_relaxed_ik_misalignment['wcs']['position'] = np.round(
            self.__last_relaxed_ik_pose['wcs']['position']
            - self.__forward_kinematics_pose['wcs']['position'],
            3,
        )
        self.__kinova_relaxed_ik_misalignment['wcs']['orientation'] = np.round(
            transformations.quaternion_multiply(
                transformations.quaternion_inverse(
                    self.__last_relaxed_ik_pose['wcs']['orientation']
                ),
                self.__forward_kinematics_pose['wcs']['orientation'],
            ),
            3,
        )

        pose_message = Pose()
        pose_message.position.x = (
            self.__kinova_relaxed_ik_misalignment['gcs']['position'][0]
        )
        pose_message.position.y = (
            self.__kinova_relaxed_ik_misalignment['gcs']['position'][1]
        )
        pose_message.position.z = (
            self.__kinova_relaxed_ik_misalignment['gcs']['position'][2]
        )
        pose_message.orientation.w = (
            self.__kinova_relaxed_ik_misalignment['gcs']['orientation'][0]
        )
        pose_message.orientation.x = (
            self.__kinova_relaxed_ik_misalignment['gcs']['orientation'][1]
        )
        pose_message.orientation.y = (
            self.__kinova_relaxed_ik_misalignment['gcs']['orientation'][2]
        )
        pose_message.orientation.z = (
            self.__kinova_relaxed_ik_misalignment['gcs']['orientation'][3]
        )

        self.__misalignment.publish(pose_message)

    def __calculate_skew_symmetric_matrix(self, omega):
        """
            
        """

        return np.array(
            [
                [0, -omega[2], omega[1]],
                [omega[2], 0, -omega[0]],
                [-omega[1], omega[0], 0],
            ]
        )

    def __axis_to_angle_rotation(self, omega, theta):
        """
        
        """

        omega_skew_symmetric = self.__calculate_skew_symmetric_matrix(omega)

        rotation = (
            np.eye(3) + np.sin(theta) * omega_skew_symmetric +
            (1 - np.cos(theta)) * omega_skew_symmetric @ omega_skew_symmetric
        )

        return rotation

    def __twist_to_homogeneous_transfomation(self, screw_axis, theta):
        """
        
        """

        omega = screw_axis[0:3]
        v = screw_axis[3:6]

        omega_skew_symmetric = self.__calculate_skew_symmetric_matrix(omega)

        # rotation = axis_angle_to_rotation(omega, angle)
        translation = np.array(
            theta * np.eye(3) + (1 - np.cos(theta)) * omega_skew_symmetric
            + (theta - np.sin(theta)) * omega_skew_symmetric
            @ omega_skew_symmetric
        ) @ v

        transformation = np.zeros([4, 4])
        transformation[0:3, 0:3] = self.__axis_to_angle_rotation(omega, theta)
        transformation[0:3, 3] = translation
        transformation[3, 3] = 1

        return transformation

    def __calculate_adjoint_transformation(self, transformation):
        """
            
        """

        rotation = transformation[0:3, 0:3]
        translation = transformation[0:3, 3]
        adjoint_transformation = np.zeros([6, 6])

        skew_symmetric_translation = self.__calculate_skew_symmetric_matrix(
            translation.T
        )
        adjoint_transformation[0:3, 0:3] = rotation
        adjoint_transformation[3:6, 0:3] = skew_symmetric_translation @ rotation
        adjoint_transformation[3:6, 3:6] = rotation

        return adjoint_transformation

    def __calculate_jacobian(self, screw_axes, joint_angles):
        """
            
        """

        jacobian = np.zeros([6, len(joint_angles)])
        adjoint_transformations = np.zeros([6, 6, len(joint_angles) - 1])

        jacobian[:, 0] = screw_axes[:, 0]

        for i in range(1, len(joint_angles)):
            transformation = self.__twist_to_homogeneous_transfomation(
                screw_axes[:, i - 1],
                joint_angles[i - 1],
            )
            adjoint_transformations[:, :, i - 1] = (
                self.__calculate_adjoint_transformation(transformation)
            )
            final_adjoint_transformation = adjoint_transformations[:, :, 0]

            for j in range(1, i):
                final_adjoint_transformation = (
                    final_adjoint_transformation
                    @ adjoint_transformations[:, :, j]
                )

            jacobian[:, i] = (final_adjoint_transformation @ screw_axes[:, i])

        return jacobian

    def __forward_kinematics(
        self,
        screw_axes,
        home_configuration,
        joint_angles,
    ):
        """

        """

        forward_kinematics = np.zeros([4, 4])

        for i in range(0, 7):
            transformation = self.__twist_to_homogeneous_transfomation(
                screw_axes[:, i],
                joint_angles[i],
            )

            if i == 0:
                forward_kinematics = transformation

            else:
                forward_kinematics = forward_kinematics @ transformation

        forward_kinematics = forward_kinematics @ home_configuration

        return forward_kinematics

    def __calculate_forward_kinematics(self, joint_angles):
        """
        
        """

        # Forward Kinematics:
        forward_kinematics = self.__forward_kinematics(
            self.__screw_axes,
            self.__home_configuration,
            joint_angles,
        )

        self.__forward_kinematics_pose['kcs']['position'] = (
            forward_kinematics[0:3, 3]
        )

        self.__forward_kinematics_pose['kcs']['orientation'] = (
            transformations.quaternion_from_matrix(forward_kinematics)
        )

        if self.__is_homed:
            # Convert Forward Kinematics from KCS to RIKCS.
            self.__forward_kinematics_pose['rikcs']['position'] = (
                self.__forward_kinematics_pose['kcs']['position']
                - self.__kcs_rikcs_difference['position']
            )
            self.__forward_kinematics_pose['rikcs']['orientation'] = (
                transformations.quaternion_multiply(
                    self.__forward_kinematics_pose['kcs']['orientation'],
                    self.__kcs_rikcs_difference['orientation'],
                )
            )

            # Convert Forward Kinematics from RIKCS to GCS and WCS.
            self.__forward_kinematics_pose['gcs']['position'] = np.matmul(
                self.ROTATE_RIKCS_TO_GCS[0:3, 0:3],
                self.__forward_kinematics_pose['rikcs']['position'],
            )
            self.__forward_kinematics_pose['gcs']['orientation'] = (
                transformations.quaternion_multiply(
                    transformations.quaternion_from_matrix(
                        self.ROTATE_RIKCS_TO_GCS
                    ),
                    self.__forward_kinematics_pose['rikcs']['orientation'],
                ),
            )[0]

            self.__forward_kinematics_pose['wcs'] = copy.deepcopy(
                self.__forward_kinematics_pose['gcs']
            )

            if self.__ENABLE_CHEST_COMPENSATION:
                self.__forward_kinematics_pose['wcs']['position'][2] += (
                    self.__chest_position
                )

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_homed:
            self.__homing()

        if not self.__is_initialized:
            return

        self.__set_target_pose(self.__input_pose['wcs'], 'wcs')

        # Publish a commanded target position in Global CS.
        self.__relaxed_ik_commanded_gcs.publish(
            self.__compose_pose_message(self.__last_relaxed_ik_pose['gcs'])
        )
        self.__commanded_pose_wcs.publish(
            self.__compose_pose_message(self.__last_relaxed_ik_pose['wcs'])
        )

        # Publish Forward Kinematics and in GCS and WCS and misalignment:
        self.__calculate_forward_kinematics(
            self.__kinova_joint_positions_feedback
        )

        self.__kinova_forward_kinematics_gcs.publish(
            self.__compose_pose_message(self.__forward_kinematics_pose['gcs'])
        )
        self.__kinova_forward_kinematics_wcs.publish(
            self.__compose_pose_message(self.__forward_kinematics_pose['wcs'])
        )

        self.__publish_kinova_relaxed_ik_misalignment()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/positional_control: node is shutting down...',
        )

        # Stop the arm motion.
        self.__stop_arm()

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
    safe_homing_z = rospy.get_param(
        param_name=f'{rospy.get_name()}/safe_homing_z',
        default=0.0,
    )
    starting_pose = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/starting_pose',
            default=
            "{'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0]}"
        )
    )
    enable_chest_compensation = rospy.get_param(
        param_name=f'{rospy.get_name()}/enable_chest_compensation',
        default=False,
    )

    pose_controller = KinovaPositionalControl(
        robot_name=kinova_name,
        mounting_angles_deg=mounting_angles_deg,
        safe_homing_z=safe_homing_z,
        starting_pose=starting_pose,
        enable_chest_compensation=enable_chest_compensation,
    )

    rospy.on_shutdown(pose_controller.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        pose_controller.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
