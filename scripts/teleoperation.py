#!/usr/bin/env python
"""Implements teleoperation module for positional control.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.
    2. Lorena Genua (lorena.genua@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import numpy as np
import transformations
from ast import (literal_eval)
import tf2_ros
import tf2_geometry_msgs  # Is required for tf2_ros.Buffer.transform().

from std_msgs.msg import (Bool)
from std_srvs.srv import (
    SetBool,
    Empty,
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
)

from kinova_positional_control.srv import (
    GripperForceGrasping,
    GripperPosition,
)
from relaxed_ik_ros1.msg import (EEPoseGoals)
from kortex_driver.srv import (Base_ClearFaults)


class KinovaTeleoperation:
    """
    
    """

    def __init__(
        self,
        robot_name,
        tracking_mode,
        compensate_orientation,
        convenience_compensation,
        maximum_input_position_change,
        maximum_input_orientation_change,
    ):
        """
        
        """

        if tracking_mode not in ['hold', 'toggle']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.TRACKING_MODE = tracking_mode
        self.COMPENSATE_ORIENTATION = compensate_orientation
        self.CONVENIENCE_COMPENSATION = convenience_compensation
        self.MAXIMUM_INPUT_POSITION_CHANGE = maximum_input_position_change
        self.MAXIMUM_INPUT_ORIENTATION_CHANGE = maximum_input_orientation_change

        # # Private variables:
        self.__input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.__tracking_button = False
        self.__gripper_button = False
        self.__mode_button = False

        self.__tracking_state_machine_state = 0
        self.__gripper_state_machine_state = 0
        self.__mode_state_machine_state = 0
        self.__control_mode = 'position'

        self.__pose_tracking = False

        # If the pose_tracking and control_mode are controlled through a service
        # call. These conditions are tracked to prevent manual switch with
        # buttons while in automatic mode.
        self.__tracking_service_active = False
        self.__control_mode_service_active = False

        # # Public variables:
        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_relaxed_ik_pose = None

        # This difference is calculated each time the tracking is started and
        # subracted from future inputs during current tracking to compensate for
        # linear misalignment between the global and relaxed_ik coordinate
        # systems.
        self.input_relaxed_ik_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {}
        self.__dependency_status_topics = {}

        self.__dependency_status['chest_control'] = False
        self.__dependency_status_topics['chest_control'] = (
            rospy.Subscriber(
                f'/chest_control/is_initialized',
                Bool,
                self.__chest_control_callback,
            )
        )
        self.__dependency_status['joints_control'] = False
        self.__dependency_status_topics['joints_control'] = (
            rospy.Subscriber(
                f'/{self.ROBOT_NAME}/joints_control/is_initialized',
                Bool,
                self.__joints_control_callback,
            )
        )
        self.__dependency_status['gripper_control'] = False
        self.__dependency_status_topics['gripper_control'] = (
            rospy.Subscriber(
                f'/{self.ROBOT_NAME}/gripper_control/is_initialized',
                Bool,
                self.__gripper_control_callback,
            )
        )

        # # Service provider:
        rospy.Service(
            f'/{self.ROBOT_NAME}/teleoperation/enable_tracking',
            SetBool,
            self.__enable_tracking_handler,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/teleoperation/enable_full_mode',
            SetBool,
            self.__enable_full_mode_handler,
        )

        rospy.Service(
            f'/{self.ROBOT_NAME}/teleoperation/pause_relaxed_ik',
            Empty,
            self.__pause_relaxed_ik_handler,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/teleoperation/resume_relaxed_ik',
            Empty,
            self.__resume_relaxed_ik_handler,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/teleoperation/reset_relaxed_ik',
            Empty,
            self.__reset_relaxed_ik_handler,
        )

        # # Service subscriber:
        self.__gripper_force_grasping = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper_control/force_grasping',
            GripperForceGrasping,
        )
        self.__gripper_position = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper_control/position',
            GripperPosition,
        )
        self.__enable_pid = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/joints_control/enable_pid',
            SetBool,
        )
        self.__reset_rik = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/relaxed_ik/reset',
            Empty,
        )
        self.__clear_arm_faults = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/clear_faults',
            Base_ClearFaults,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__relaxed_ik_target = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/teleoperation/input_pose',
            Pose,
            self.__input_pose_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/teleoperation/tracking_button',
            Bool,
            self.__tracking_button_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/teleoperation/gripper_button',
            Bool,
            self.__gripper_button_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/teleoperation/mode_button',
            Bool,
            self.__mode_button_callback,
        )

        # # TF broadcaster:

        # # TF listener:
        self.__tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        tf2_ros.TransformListener(self.__tf_buffer)

    # # Dependency status callbacks:
    def __chest_control_callback(self, message):
        """Monitors chest_control is_initialized topic.
        
        """

        self.__dependency_status['chest_control'] = message.data

    def __joints_control_callback(self, message):
        """Monitors joints_control is_initialized topic.
        
        """

        self.__dependency_status['joints_control'] = message.data

    def __gripper_control_callback(self, message):
        """Monitors gripper_control is_initialized topic.
        
        """

        self.__dependency_status['gripper_control'] = message.data

    # # Service handlers:
    def __enable_tracking_handler(self, request):
        """
        
        """

        message = ''
        success = False

        if not self.__is_initialized:
            return success, message

        if request.data and not self.__tracking_service_active:
            self.__tracking_service_active = True

            rospy.logwarn(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    'pose_tracking was enabled by a service call.\n'
                    'Button pose_tracking is now disabled.\n'
                ),
            )

            self.__calculate_compensation()
            self.__pose_tracking = True
            self.__tracking_state_machine_state = 0

            message = 'pose_tracking was enabled.'
            success = True

        elif not request.data and self.__tracking_service_active:
            self.__pose_tracking = False
            self.__tracking_service_active = False

            rospy.logwarn(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    'pose_tracking was disabled by a service call.\n'
                    'Button pose_tracking is now enabled.\n'
                ),
            )

            message = 'pose_tracking was disabled.'
            success = True

        return success, message

    def __enable_full_mode_handler(self, request):
        """
        
        """

        message = ''
        success = False

        if not self.__is_initialized:
            return success, message

        if request.data and not self.__control_mode_service_active:
            self.__control_mode_service_active = True

            rospy.logwarn(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    'control_mode was set to "full" by a service call.\n'
                    'Button control_mode change is now disabled.\n'
                ),
            )

            self.__control_mode = 'full'
            self.__calculate_compensation()
            self.__mode_state_machine_state = 0

            message = 'Full (position + orientation) mode.'
            success = True

        elif not request.data and self.__control_mode_service_active:
            self.__control_mode = 'position'
            self.__control_mode_service_active = False

            rospy.logwarn(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    'control_mode was set to "position" by a service call.\n'
                    'Button control_mode change is now enabled.\n'
                ),
            )

            message = 'Position only mode.'
            success = True

        return success, message

    def __pause_relaxed_ik_handler(self, request):
        """

        """

        self.__pause_relaxed_ik()

    def __resume_relaxed_ik_handler(self, request):
        """

        """

        self.__resume_relaxed_ik()

    def __reset_relaxed_ik_handler(self, request):
        """

        """

        self.__reset_relaxed_ik()

    # # Topic callbacks:
    def __input_pose_callback(self, message):
        """

        """

        self.__input_pose['position'][0] = message.position.x
        self.__input_pose['position'][1] = message.position.y
        self.__input_pose['position'][2] = message.position.z

        self.__input_pose['orientation'][0] = message.orientation.w
        self.__input_pose['orientation'][1] = message.orientation.x
        self.__input_pose['orientation'][2] = message.orientation.y
        self.__input_pose['orientation'][3] = message.orientation.z

    def __tracking_button_callback(self, message):
        """

        """

        self.__tracking_button = message.data

    def __gripper_button_callback(self, message):
        """

        """

        self.__gripper_button = message.data

    def __mode_button_callback(self, message):
        """

        """

        self.__mode_button = message.data

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
                            f'/{self.ROBOT_NAME}/teleoperation: '
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
                    f'/{self.ROBOT_NAME}/teleoperation:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        if self.__dependency_status['chest_control']:
            self.__get_relaxed_ik_last_pose()

        # NOTE: Add more initialization criterea if needed.
        if (
            self.__dependency_initialized and self.last_relaxed_ik_pose != None
        ):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/teleoperation: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                # Stop tracking.
                self.__tracking_state_machine_state = 0
                self.__pose_tracking = False

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __tracking_state_machine(self, button):
        """
        
        """

        # Do not allow manual tracking control while the tracking is controlled
        # using the service.
        if self.__tracking_service_active:
            return

        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):

            self.__tracking_state_machine_state = 1

            if self.TRACKING_MODE == 'hold':
                self.__calculate_compensation()
                self.__pose_tracking = True

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):

            if self.TRACKING_MODE == 'toggle':
                self.__tracking_state_machine_state = 2
                self.__calculate_compensation()
                self.__pose_tracking = True

            elif self.TRACKING_MODE == 'hold':
                self.__tracking_state_machine_state = 0
                self.__pose_tracking = False

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):

            self.__tracking_state_machine_state = 3
            self.__pose_tracking = False

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):

            self.__tracking_state_machine_state = 0

    def __mode_state_machine(self, button):
        """
        
        """

        # Do not allow manual mode control while the mode is controlled
        # using the service.
        if self.__control_mode_service_active:
            return

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):

            self.__mode_state_machine_state = 1
            self.__control_mode = 'full'
            self.__calculate_compensation()

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):

            self.__mode_state_machine_state = 2

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 2 and button):

            self.__mode_state_machine_state = 3
            self.__control_mode = 'position'

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 3 and not button):

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

        self.input_relaxed_ik_difference['position'] = (
            self.__input_pose['position']
            - self.last_relaxed_ik_pose['position']
        )

        self.input_relaxed_ik_difference['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_inverse(
                    self.__input_pose['orientation']
                ),
                self.last_relaxed_ik_pose['orientation'],
            )
        )

    def __stop_tracking(self):
        """
        
        """

        if self.__tracking_service_active:
            self.__tracking_service_active = False

            rospy.logwarn(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    'tracking service is now not active.\n'
                    'Button pose_tracking is now enabled.\n'
                ),
            )

        # Deactivate control mode service:
        if self.__control_mode_service_active:
            self.__control_mode_service_active = False

            rospy.logwarn(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    'control_mode service is now not active.\n'
                    'Button control_mode change is now enabled.'
                    f'Current control_mode is {self.__control_mode}\n'
                ),
            )

        self.__tracking_state_machine_state = 0
        self.__pose_tracking = False

    def __publish_kinova_pose(self):
        """
        
        """

        # Protection against 0, 0, 0 controller input values.
        # Controller loses connection, goes into a sleep mode etc.
        if (
            self.__input_pose['position'][0] == 0
            and self.__input_pose['position'][1] == 0
            and self.__input_pose['position'][2] == 0 and self.__pose_tracking
        ):
            self.__stop_tracking()

            rospy.logerr(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    f'\n(0, 0, 0) position while active tracking! '
                    '\nStopped input tracking.'
                ),
            )

            return

        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        compensated_input_pose['position'] = (
            self.__input_pose['position']
            - self.input_relaxed_ik_difference['position']
        )

        # Use fixed (last commanded) orientation.
        compensated_input_pose['orientation'] = (
            self.last_relaxed_ik_pose['orientation']
        )

        # Use oculus orientation.
        if self.__control_mode == 'full':
            compensated_input_pose['orientation'] = (
                self.__input_pose['orientation']
            )

            if self.COMPENSATE_ORIENTATION:
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        self.__input_pose['orientation'],
                        self.input_relaxed_ik_difference['orientation'],
                    )
                )

            else:
                # # Convenience orientation corrections:
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        compensated_input_pose['orientation'],
                        transformations.quaternion_about_axis(
                            np.deg2rad(self.CONVENIENCE_COMPENSATION[1]),
                            (0, 1, 0),  # Around Y.
                        ),
                    )
                )
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        compensated_input_pose['orientation'],
                        transformations.quaternion_about_axis(
                            np.deg2rad(self.CONVENIENCE_COMPENSATION[2]),
                            (0, 0, 1),  # Around Z.
                        ),
                    )
                )
                compensated_input_pose['orientation'] = (
                    transformations.quaternion_multiply(
                        compensated_input_pose['orientation'],
                        transformations.quaternion_about_axis(
                            np.deg2rad(self.CONVENIENCE_COMPENSATION[0]),
                            (1, 0, 0),  # Around X.
                        ),
                    )
                )

        # Protection against too big positional input changes. Controller loses
        # connection, goes out-of-sight, goes into a sleep mode, user input is
        # unsafe.
        input_position_difference = np.linalg.norm(
            compensated_input_pose['position']
            - self.last_relaxed_ik_pose['position']
        )

        if (input_position_difference > self.MAXIMUM_INPUT_POSITION_CHANGE):
            self.__stop_tracking()

            rospy.logerr(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    f'\nChange in input POSITION exceeded maximum allowed value! '
                    f'\n- Current input: {np.round(compensated_input_pose["position"], 3)}'
                    f'\n- Previous input: {np.round(self.last_relaxed_ik_pose["position"], 3)}'
                    f'\n- Difference (absolute): {np.round(input_position_difference, 3)}'
                    f'\n- Allowed difference threshold: {np.round(self.MAXIMUM_INPUT_POSITION_CHANGE, 3)}'
                    '\nStopped input tracking.\n'
                ),
            )

            return

        # Protection against too big angular input changes. Controller loses
        # connection, goes out-of-sight, goes into a sleep mode,  user input is
        # unsafe.
        input_orientation_missalignment = transformations.quaternion_multiply(
            transformations.quaternion_inverse(
                compensated_input_pose['orientation'],
            ),
            self.last_relaxed_ik_pose['orientation'],
        )

        input_angular_difference = round(
            np.rad2deg(
                2 * np.arctan2(
                    np.linalg.norm(input_orientation_missalignment[1:4]),
                    input_orientation_missalignment[0],
                )
            ),
            2,
        )

        if input_angular_difference > 180:
            input_angular_difference = 360 - input_angular_difference

        if (input_angular_difference > self.MAXIMUM_INPUT_ORIENTATION_CHANGE):
            self.__stop_tracking()

            rospy.logerr(
                (
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    f'\nChange in input ORIENTATION exceeded maximum allowed value! '
                    f'\n- Current input: {np.round(compensated_input_pose["orientation"], 3)}'
                    f'\n- Previous input: {np.round(self.last_relaxed_ik_pose["orientation"], 3)}'
                    f'\n- Difference (absolute): {np.round(input_angular_difference, 3)}'
                    f'\n- Allowed difference threshold: {np.round(self.MAXIMUM_INPUT_ORIENTATION_CHANGE, 3)}'
                    '\nStopped input tracking.\n'
                ),
            )

            return

        # Prepare compensated_pose (target_pose) for convertion to
        # kinova/base_link frame:
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.stamp = rospy.Time.now()
        target_pose_stamped.header.frame_id = 'base_link'
        target_pose_stamped.pose.position.x = (
            compensated_input_pose['position'][0]
        )
        target_pose_stamped.pose.position.y = (
            compensated_input_pose['position'][1]
        )
        target_pose_stamped.pose.position.z = (
            compensated_input_pose['position'][2]
        )

        target_pose_stamped.pose.orientation.w = (
            compensated_input_pose['orientation'][0]
        )
        target_pose_stamped.pose.orientation.x = (
            compensated_input_pose['orientation'][1]
        )
        target_pose_stamped.pose.orientation.y = (
            compensated_input_pose['orientation'][2]
        )
        target_pose_stamped.pose.orientation.z = (
            compensated_input_pose['orientation'][3]
        )

        # Convert target_pose (compensated pose) from base_link to
        # kinova/base_link:
        try:
            # NOTE: Requires import tf2_geometry_msgs to work.
            pose_in_base_link = self.__tf_buffer.transform(
                object_stamped=target_pose_stamped,
                target_frame=f'{self.ROBOT_NAME}/base_link',
            )

            pose_message = Pose()
            pose_message.position.x = pose_in_base_link.pose.position.x
            pose_message.position.y = pose_in_base_link.pose.position.y
            pose_message.position.z = pose_in_base_link.pose.position.z

            pose_message.orientation.x = pose_in_base_link.pose.orientation.x
            pose_message.orientation.y = pose_in_base_link.pose.orientation.y
            pose_message.orientation.z = pose_in_base_link.pose.orientation.z
            pose_message.orientation.w = pose_in_base_link.pose.orientation.w

            ee_pose_goals = EEPoseGoals()
            ee_pose_goals.ee_poses.append(pose_message)
            ee_pose_goals.ee_poses.append(pose_message)

            self.__relaxed_ik_target.publish(ee_pose_goals)

        except Exception as ex:
            rospy.logerr_throttle(
                1, f'/{self.ROBOT_NAME}/teleoperation:'
                f'\nError in __publish_kinova_pose: {ex}'
            )

    def __transform_lookup(
        self,
        buffer: tf2_ros.Buffer,
        from_frame: str,
        to_frame: str,
    ):
        """

        """

        try:
            transform = buffer.lookup_transform(
                target_frame=from_frame,
                source_frame=to_frame,
                time=rospy.Time(0),
            )

            return transform

        except Exception as ex:
            rospy.logerr_throttle(
                1,
                f'/{self.ROBOT_NAME}/teleoperation:'
                f'\nError in __transform_lookup: {ex}',
            )
            return None

    def __get_relaxed_ik_last_pose(self):
        """
        
        """

        # Get current relaxed_ik_target transform in base_link frame:
        # Position:
        transform = self.__transform_lookup(
            self.__tf_buffer,
            'base_link',
            f'{self.ROBOT_NAME}/relaxed_ik_target',
        )

        if transform:
            position = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ]
            )
            orientation = np.array(
                [
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                ]
            )

            self.last_relaxed_ik_pose = {}
            self.last_relaxed_ik_pose['position'] = position
            self.last_relaxed_ik_pose['orientation'] = orientation

    def __pause_relaxed_ik(self):
        """
        
        """

        self.__enable_pid(False)

    def __resume_relaxed_ik(self):
        """
        
        """

        self.__reset_rik()
        self.__enable_pid(True)

    def __reset_relaxed_ik(self):
        """
        
        """

        self.__pause_relaxed_ik()
        self.__clear_arm_faults()
        self.__resume_relaxed_ik()

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__tracking_state_machine(self.__tracking_button)
        self.__gripper_state_machine(self.__gripper_button)
        self.__mode_state_machine(self.__mode_button)

        if self.__pose_tracking:
            self.__publish_kinova_pose()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/teleoperation node is shutting down...',
        )

        # TODO: Stop the arm motion.
        # self.__stop_arm()

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/teleoperation: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'teleoperation',
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

    tracking_mode = rospy.get_param(
        param_name=f'{rospy.get_name()}/tracking_mode',
        default='toggle',
    )

    compensate_orientation = rospy.get_param(
        param_name=f'{rospy.get_name()}/compensate_orientation',
        default=True,
    )

    maximum_input_position_change = rospy.get_param(
        param_name=f'{rospy.get_name()}/maximum_input_position_change',
        default=0.1,
    )

    maximum_input_orientation_change = rospy.get_param(
        param_name=f'{rospy.get_name()}/maximum_input_orientation_change',
        default=30,
    )

    convenience_compensation = literal_eval(
        rospy.get_param(
            param_name=f'{rospy.get_name()}/convenience_compensation',
            default='[0.0, 0.0, 0.0]',
        )
    )

    kinova_teleoperation = KinovaTeleoperation(
        robot_name=kinova_name,
        tracking_mode=tracking_mode,
        compensate_orientation=compensate_orientation,
        convenience_compensation=convenience_compensation,
        maximum_input_position_change=maximum_input_position_change,
        maximum_input_orientation_change=maximum_input_orientation_change,
    )

    rospy.on_shutdown(kinova_teleoperation.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        kinova_teleoperation.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
