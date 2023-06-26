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
from std_srvs.srv import (SetBool)
from geometry_msgs.msg import (Pose)

from kortex_driver.msg import (
    BaseCyclic_Feedback,
    Twist,
    TwistCommand,
)
from kortex_driver.srv import (
    Stop,
    Base_ClearFaults,
    # ApplyEmergencyStop,
)
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
        # ee_starting_position=(0.57, 0.0, 0.43),
        # workspace_radius=1.2,
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

        # self.WORKSPACE_CENTER = np.negative(ee_starting_position)
        # self.WORKSPACE_RADIUS = workspace_radius

        # # Private variables:
        self.__is_homed = False
        self.__is_motion_finished = True

        # # Public variables:

        # Input pose in Global and Relaxed IK coordinate systems.
        self.input_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }

        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_relaxed_ik_pose = {
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

        # Pose, which is received directly from Kinova feedback.
        self.kinova_feedback_pose = {
            'kcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    # 'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'rikcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    # 'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }

        # Difference between kinova and relaxed_ik CS origins on start up.
        self.kcs_rikcs_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            # 'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False
        self.__kcs_rikcs_difference_calculated = False

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
        self.__clear_arm_faults = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/clear_faults',
            Base_ClearFaults,
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

        self.__kinova_cartesian_velocity = rospy.Publisher(
            f'/{self.ROBOT_NAME}/in/cartesian_velocity',
            TwistCommand,
            queue_size=1,
        )

        self.__motion_finished = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/motion_finished',
            Bool,
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
            f'/{self.ROBOT_NAME}/base_feedback',
            BaseCyclic_Feedback,
            self.__base_feedback_callback,
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

    # # Service handlers:

    # # Topic callbacks:
    def __input_pose_callback(self, msg):
        """
        
        """

        self.input_pose['gcs']['position'][0] = msg.position.x
        self.input_pose['gcs']['position'][1] = msg.position.y
        self.input_pose['gcs']['position'][2] = msg.position.z

        self.input_pose['gcs']['orientation'][0] = msg.orientation.w
        self.input_pose['gcs']['orientation'][1] = msg.orientation.x
        self.input_pose['gcs']['orientation'][2] = msg.orientation.y
        self.input_pose['gcs']['orientation'][3] = msg.orientation.z

    def __pid_motion_finished_callback(self, msg):
        """
        
        """

        if not self.__is_initialized:
            self.joint_control_initialized = True

        self.__is_motion_finished = msg.data

    def __base_feedback_callback(self, message):
        """
        
        """

        self.__kinova_feedback_z = message.base.tool_pose_z

        self.kinova_feedback_pose['kcs']['position'][0] = (
            message.base.tool_pose_x
        )
        self.kinova_feedback_pose['kcs']['position'][1] = (
            message.base.tool_pose_y
        )
        self.kinova_feedback_pose['kcs']['position'][2] = (
            message.base.tool_pose_z
        )

        if self.__kcs_rikcs_difference_calculated:
            self.kinova_feedback_pose['rikcs']['position'] = (
                self.kinova_feedback_pose['kcs']['position']
                - self.kcs_rikcs_difference['position']
            )

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

        # If feedback from kortex_driver (implied by joints_control is
        # initialized) and relaxed IK was received and the robot is at home
        # position - once calculate difference between KCS and RIKCS.
        if (
            not self.__kcs_rikcs_difference_calculated and self.__is_homed
            and self.__dependency_status['relaxed_ik']
            and self.__dependency_status['joints_control']
        ):
            self.kcs_rikcs_difference['position'] = (
                self.kinova_feedback_pose['kcs']['position']
                - self.last_relaxed_ik_pose['rikcs']['position']
            )

            self.__kcs_rikcs_difference_calculated = True

        # NOTE: Add more initialization criterea if needed.
        if (
            self.__dependency_initialized and self.__is_homed
            and self.__kcs_rikcs_difference_calculated
        ):
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

    # def __check_boundaries(self, position):
    #     """

    #     """

    #     # Calculate the vector from the center to the position.
    #     vector_to_position = [
    #         position[i] - self.WORKSPACE_CENTER[i]
    #         for i in range(len(self.WORKSPACE_CENTER))
    #     ]

    #     # Calculate the distance from the center to the position.
    #     distance_to_position = math.sqrt(
    #         sum([x**2 for x in vector_to_position])
    #     )

    #     # If the position is inside the sphere, return its coordinates.
    #     if distance_to_position <= self.WORKSPACE_RADIUS:
    #         return position

    #     # Calculate the vector from the center to the closest position on the
    #     # sphere surface.
    #     vector_to_surface = [
    #         vector_to_position[i] * self.WORKSPACE_RADIUS / distance_to_position
    #         for i in range(len(self.WORKSPACE_CENTER))
    #     ]

    #     # Calculate the coordinates of the closest position on the sphere
    #     # surface.
    #     closest_position = np.array(
    #         [
    #             self.WORKSPACE_CENTER[i] + vector_to_surface[i]
    #             for i in range(len(self.WORKSPACE_CENTER))
    #         ]
    #     )

    #     return closest_position

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

    def __homing(self):
        """
        
        """

        # Wait for dependencies to initialize.
        if not self.__dependency_initialized:
            return

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: dependencies have initialized.',
        )

        self.__clear_arm_faults()

        # Disable PID joints control to use kinova cartesian velocity.
        self.__enable_pid(False)

        # Move to a safe Z position before homing.
        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: '
            'moving to safe Z before homing...'
        )

        while (
            self.kinova_feedback_pose['kcs']['position'][2] < self.SAFE_HOMING_Z
        ):
            self.__publish_cartesian_z_velocity(0.05)

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
        self.input_pose['gcs'] = copy.deepcopy(self.STARTING_POSE)
        self.set_target_pose(self.input_pose['gcs'], 'gcs')
        self.__wait_for_motion()

        rospy.loginfo(
            f'/{self.ROBOT_NAME}/positional_control: homing has finished.',
        )

        self.__pid_velocity_limit(1.0)

        self.__is_homed = True

    def __publish_pose_motion_finished(self):
        """
        
        """

        motion_finished = True

        if (
            (
                round(
                    self.last_relaxed_ik_pose['rikcs']['position'][0]
                    - self.kinova_feedback_pose['rikcs']['position'][0],
                    2,
                ) > 0.01
            ) or (
                round(
                    self.last_relaxed_ik_pose['rikcs']['position'][1]
                    - self.kinova_feedback_pose['rikcs']['position'][1],
                    2,
                ) > 0.01
            ) or (
                round(
                    self.last_relaxed_ik_pose['rikcs']['position'][2]
                    - self.kinova_feedback_pose['rikcs']['position'][2],
                    2,
                ) > 0.02
            )
        ):
            motion_finished = False

        self.__motion_finished.publish(motion_finished)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_homed:
            self.__homing()

        if not self.__is_initialized:
            return

        self.set_target_pose(self.input_pose['gcs'], 'gcs')

        # Publish a commanded target position in Global CS.
        self.__relaxed_ik_commanded_gcs.publish(
            self.__compose_pose_message(self.last_relaxed_ik_pose['gcs'])
        )

        self.__publish_pose_motion_finished()

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

    def set_target_pose(self, target_pose, coordinate_system):
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

        self.last_relaxed_ik_pose['gcs'] = copy.deepcopy(target_pose)
        self.last_relaxed_ik_pose['rikcs'] = copy.deepcopy(target_pose)

        if coordinate_system == 'gcs':
            # Update target pose in Relaxed IK CS.
            self.last_relaxed_ik_pose['rikcs']['position'] = np.matmul(
                self.ROTATE_GCS_TO_RIKCS[0:3, 0:3],
                target_pose['position'],
            )
            self.last_relaxed_ik_pose['rikcs']['orientation'] = (
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
            self.last_relaxed_ik_pose['gcs']['position'] = np.matmul(
                self.ROTATE_RIKCS_TO_GCS[0:3, 0:3],
                target_pose['position'],
            )
            self.last_relaxed_ik_pose['gcs']['orientation'] = (
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
            self.last_relaxed_ik_pose['rikcs']
        )

        # TODO: Add left arm support.
        # Form a message for the left arm.
        left_arm_pose_rikcs = self.__compose_pose_message(
            self.last_relaxed_ik_pose['rikcs']
        )

        # Form a message for the relaxed IK setpoint topic.
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(right_arm_pose_rikcs)
        ee_pose_goals.ee_poses.append(left_arm_pose_rikcs)
        ee_pose_goals.header.seq = 0

        self.__relaxed_ik_target_rikcs.publish(ee_pose_goals)


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

    pose_controller = KinovaPositionalControl(
        robot_name=kinova_name,
        mounting_angles_deg=mounting_angles_deg,
        safe_homing_z=safe_homing_z,
        starting_pose=starting_pose,
    )

    rospy.on_shutdown(pose_controller.node_shutdown)

    while not rospy.is_shutdown():
        pose_controller.main_loop()


if __name__ == '__main__':
    main()
