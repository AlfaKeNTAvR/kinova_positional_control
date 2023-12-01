#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import math
import transformations
import copy
from threading import (Timer)

from std_msgs.msg import (Bool)
from std_srvs.srv import (SetBool)
from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (ControllerJoystick)


class OculusJoystickMapping:
    """
    
    """

    def __init__(
        self,
        node_name,
        robot_name,
        controller_side,
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        # # Private constants:
        self.__DIFFERENCE_THRESHOLD = {
            'linear': 0.025,  # [meters].
            'angular': 5.0,  # [degrees].
        }

        # A mounting angle of the scoop rod, relative to the ground.
        self.__SCOOP_ROD_MOUNT_ANGLE = 148  # [degrees].
        self.__EE_SCOOP_EDGE_LENGTH = 0.4  # [meters].
        eecs_dx, eecs_dz = (
            np.array(
                self.__polar_to_catresian(
                    self.__EE_SCOOP_EDGE_LENGTH,
                    (self.__SCOOP_ROD_MOUNT_ANGLE
                     - 15),  # Positional control Y startting orientation.
                )
            ).round(3)
        )

        # Transition matrices:
        self.__T_GCS_TO_HPCS = np.array(
            [
                [1, 0, 0, 0.25],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1],
            ]
        )
        self.__T_HPCS_TO_GCS = np.linalg.inv(self.__T_GCS_TO_HPCS)

        self.__T_EECS_TO_VPCS = np.array(
            [
                [1, 0, 0, eecs_dx],
                [0, 1, 0, 0.0],
                [0, 0, 1, eecs_dz],
                [0, 0, 0, 1],
            ]
        )
        self.__T_VPCS_TO_EECS = np.linalg.inv(self.__T_EECS_TO_VPCS)

        # # Public constants:
        self.NODE_NAME = node_name
        self.ROBOT_NAME = robot_name
        self.CONTROLLER_SIDE = controller_side

        # # Private variables:
        self.__oculus_joystick = {
            'right': ControllerJoystick(),
            'left': ControllerJoystick(),
        }
        self.__last_relaxed_ik_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'hpcs':
                {
                    'radius': 0.0,
                    'angle': 0.0,
                    'z_position': 0.0,
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'eecs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'vpcs':
                {
                    'radius': self.__EE_SCOOP_EDGE_LENGTH,
                    'angle': 0.0,
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }
        self.__target_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'hpcs':
                {
                    'radius': 0.0,
                    'angle': 0.0,
                    'z_position': 0.0,
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'eecs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'vpcs':
                {
                    'radius': self.__EE_SCOOP_EDGE_LENGTH,
                    'angle': 0.0,
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }

        # Linear (vector) and angular difference (angle) between Kinova nd
        # Relaxed IK calculated based on missaligment.
        self.__kinova_relaxed_ik_difference = {
            'linear': 0.0,
            'angular': 0.0,
        }
        self.__difference_threshold = copy.deepcopy(self.__DIFFERENCE_THRESHOLD)

        self.__mode_state_machine_state = 0
        self.__control_mode = 'polar'

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False
        self.__last_relaxed_ik_pose_recieved = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            'teleoperation': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            'teleoperation':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/teleoperation/is_initialized',
                    Bool,
                    self.__teleoperation_callback,
                ),
        }

        # # Service provider:
        # rospy.Service(
        #     f'{self.NODE_NAME}/service_name1',
        #     ServiceType1,
        #     self.__service_name1_handler,
        # )

        # # Service subscriber:
        self.__teleoperation_enable_tracking = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/enable_tracking',
            SetBool,
        )
        self.__teleoperation_enable_full_mode = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/teleoperation/enable_full_mode',
            SetBool,
        )

        # # Topic publisher:
        self.__teleoperation_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/teleoperation/input_pose',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/right/controller_feedback/joystick',
            ControllerJoystick,
            self.__right_oculus_joystick_callback,
        )
        rospy.Subscriber(
            f'/left/controller_feedback/joystick',
            ControllerJoystick,
            self.__left_oculus_joystick_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/positional_control/kinova_relaxed_ik_missalignment',
            Pose,
            self.__kinova_relaxed_ik_missalignment_callback,
        )

        # # Timers:
        rospy.Timer(rospy.Duration(1.0 / 100), self.__update_position_timer)

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __teleoperation_callback(self, message):
        """Monitors teleoperation is_initialized topic.
        
        """

        self.__dependency_status['teleoperation'] = message.data

    # # Service handlers:
    # def __service_name1_handler(self, request):
    #     """

    #     """

    #     response = True

    #     return response

    # # Topic callbacks:
    def __right_oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick['right'].position_x = message.position_x
        self.__oculus_joystick['right'].position_y = message.position_y

        self.__oculus_joystick['right'].button = message.button

    def __left_oculus_joystick_callback(self, message):
        """

        """

        self.__oculus_joystick['left'].position_x = message.position_x
        self.__oculus_joystick['left'].position_y = message.position_y

        self.__oculus_joystick['left'].button = message.button

    def __commanded_pose_callback(self, message):
        """
        
        """

        # Global CS:
        self.__last_relaxed_ik_pose['gcs']['position'][0] = message.position.x
        self.__last_relaxed_ik_pose['gcs']['position'][1] = message.position.y
        self.__last_relaxed_ik_pose['gcs']['position'][2] = message.position.z

        self.__last_relaxed_ik_pose['gcs']['orientation'][0] = (
            message.orientation.w
        )
        self.__last_relaxed_ik_pose['gcs']['orientation'][1] = (
            message.orientation.x
        )
        self.__last_relaxed_ik_pose['gcs']['orientation'][2] = (
            message.orientation.y
        )
        self.__last_relaxed_ik_pose['gcs']['orientation'][3] = (
            message.orientation.z
        )

        # Horizontal Polar CS:
        point_hpcs = self.__gcs_to_hpcs(
            np.array(
                [
                    message.position.x,
                    message.position.y,
                    message.position.z,
                ]
            ),
        )
        self.__last_relaxed_ik_pose['hpcs']['radius'] = point_hpcs[0]
        self.__last_relaxed_ik_pose['hpcs']['angle'] = point_hpcs[1]
        self.__last_relaxed_ik_pose['hpcs']['z_position'] = point_hpcs[2]

        # Orientation:
        self.__last_relaxed_ik_pose['hpcs']['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(
                    -np.deg2rad(self.__last_relaxed_ik_pose['hpcs']['angle']),
                    (0, 0, 1),  # Around Z.
                ),
                self.__last_relaxed_ik_pose['gcs']['orientation'],
            )
        )

        # Vertical Polar CS:
        self.__last_relaxed_ik_pose['vpcs']['angle'] = (
            self.__SCOOP_ROD_MOUNT_ANGLE - np.rad2deg(
                transformations.euler_from_quaternion(
                    self.__last_relaxed_ik_pose['gcs']['orientation']
                )
            )[1]
        )

        # Orientation:
        self.__last_relaxed_ik_pose['vpcs']['orientation'] = (
            copy.deepcopy(self.__last_relaxed_ik_pose['hpcs']['orientation'])
        )

        # End-Effector CS:
        point_vpcs = np.array(
            [
                self.__last_relaxed_ik_pose['vpcs']['radius'],
                self.__last_relaxed_ik_pose['vpcs']['angle'],
            ]
        )
        self.__last_relaxed_ik_pose['eecs']['position'] = self.__vpcs_to_eecs(
            point_vpcs
        )
        self.__last_relaxed_ik_pose['eecs']['orientation'] = (
            copy.deepcopy(self.__last_relaxed_ik_pose['hpcs']['orientation'])
        )

        if not self.__last_relaxed_ik_pose_recieved:
            self.__target_pose = copy.deepcopy(self.__last_relaxed_ik_pose)

            self.__last_relaxed_ik_pose_recieved = True

    def __kinova_relaxed_ik_missalignment_callback(self, message):
        """

        """

        kinova_relaxed_ik_missalignment = {}
        kinova_relaxed_ik_missalignment['position'] = np.array(
            [
                message.position.x,
                message.position.y,
                message.position.z,
            ]
        )
        kinova_relaxed_ik_missalignment['orientation'] = np.array(
            [
                message.orientation.w,
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
            ]
        )

        # Calculate linear and angular differences between Relaxed IK and Kinova
        # using position and quaternion misalignment:
        self.__kinova_relaxed_ik_difference['linear'] = round(
            np.linalg.norm(kinova_relaxed_ik_missalignment['position']),
            3,
        )
        angular_difference = round(
            np.rad2deg(
                2 * np.arctan2(
                    np.linalg.norm(
                        kinova_relaxed_ik_missalignment['orientation'][1:4]
                    ),
                    kinova_relaxed_ik_missalignment['orientation'][0],
                )
            ),
            3,
        )

        if angular_difference > 180:
            self.__kinova_relaxed_ik_difference['angular'] = (
                360 - angular_difference
            )

        else:
            self.__kinova_relaxed_ik_difference['angular'] = angular_difference

    # # Timer functions:
    def __update_position_timer(self, event):
        """
        
        """

        if not self.__is_initialized:
            return

        self.__update_position_hpcs()
        # self.__update_position_gcs()

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
                        (f'{self.NODE_NAME}: '
                         f'lost connection to {key}!')
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
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (
            self.__dependency_initialized
            and self.__last_relaxed_ik_pose_recieved
        ):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.NODE_NAME}: ready.\033[0m',)

                try:
                    # self.__teleoperation_enable_tracking(True)
                    self.__teleoperation_enable_full_mode(True)
                except Exception as e:
                    print(e)

                self.__is_initialized = True

                # Set the timer for 1 second to call enable_tracking function to
                # prevent (0, 0, 0) input to teleoperation.
                Timer(
                    1,
                    self.__teleoperation_enable_tracking,
                    args=[True],
                ).start()

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                self.__last_relaxed_ik_pose_recieved = False

                try:
                    self.__teleoperation_enable_tracking(False)
                    self.__teleoperation_enable_full_mode(False)
                except Exception as e:
                    print(e)

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __cartesian_to_polar(self, x, y):
        """
        
        """

        radius = math.sqrt(x**2 + y**2)
        angle_radians = math.atan2(y, x)
        angle_degrees = math.degrees(angle_radians)

        return radius, angle_degrees

    def __polar_to_catresian(self, radius, angle_degrees):
        """
        
        """

        angle_in_radians = math.radians(angle_degrees)
        x = radius * math.cos(angle_in_radians)
        y = radius * math.sin(angle_in_radians)

        return x, y

    def __gcs_to_hpcs(self, point_gcs):
        """
        From Global CS to Horizontal Polar CS.

        tcs - Tank Coordinate system.

        """

        # Linear transition:
        point_gcs = np.insert(point_gcs, 3, 1)
        point_tcs = self.__T_GCS_TO_HPCS @ point_gcs

        # Conversion to polar coordinates (X-Y plane):
        radius, angle_degrees = self.__cartesian_to_polar(
            point_tcs[0], point_tcs[1]
        )

        point_hpcs = np.array([radius, angle_degrees, point_gcs[2]])

        return point_hpcs

    def __hpcs_to_gcs(self, point_hpcs):
        """
        From Horizontal Polar CS to Global CS.

        tcs - Tank Coordinate system.

        """

        # Conversion to Cartesian coordinates (X-Y plane):
        x_tcs, y_tcs = self.__polar_to_catresian(point_hpcs[0], point_hpcs[1])

        # Linear transition:
        point_tcs = np.array([x_tcs, y_tcs, point_hpcs[2], 1])
        point_gcs = self.__T_HPCS_TO_GCS @ point_tcs
        point_gcs = np.delete(point_gcs, 3)

        return point_gcs

    def __eecs_to_vpcs(self, point_eecs):
        """

        scs - Scoop Coordinate System.
        
        """

        # Linear transition:
        point_eecs[1] = 0
        point_eecs = np.insert(point_eecs, 3, 1)
        point_scs = self.__T_EECS_TO_VPCS @ point_eecs

        # Conversion to polar coordinates (X-Z plane):
        radius, angle_degrees = self.__cartesian_to_polar(
            point_scs[0], point_scs[2]
        )

        # Add dummy Z.
        point_hpcs = np.array([radius, angle_degrees])

        return point_hpcs

    def __vpcs_to_eecs(self, point_vpcs):
        """

        scs - Scoop Coordinate System.
        
        """

        # Coversion to Cartesian coordinates (X-Z plane):
        angle_in_radians = math.radians(point_vpcs[1])
        x_scs = point_vpcs[0] * math.cos(angle_in_radians)
        z_scs = point_vpcs[0] * math.sin(angle_in_radians)

        # Linear transition:
        point_scs = np.array([x_scs, 0, z_scs, 1])
        point_eecs = (self.__T_VPCS_TO_EECS @ point_scs)
        point_eecs = np.delete(point_eecs, 3)

        return point_eecs

    def __mode_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):

            self.__mode_state_machine_state = 1

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):

            self.__mode_state_machine_state = 2
            self.__control_mode = 'height'

            try:
                self.__teleoperation_enable_tracking(True)
                self.__teleoperation_enable_full_mode(True)
            except Exception as e:
                print(e)

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 2 and button):

            self.__mode_state_machine_state = 3

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 3 and not button):

            self.__mode_state_machine_state = 0

            self.__control_mode = 'polar'

            try:
                self.__teleoperation_enable_tracking(True)
                self.__teleoperation_enable_full_mode(True)
            except Exception as e:
                print(e)

    def __update_position_gcs(self):
        """

        Joystick velocity control in Global CS.

        """

        # Constants
        dt = 1.0 / 100  # time delta, seconds
        v_scale = 0.1  # units/second

        # Calculate displacement based on velocity and time delta.
        dx = self.__oculus_joystick['right'].position_y * v_scale * dt
        dy = -self.__oculus_joystick['right'].position_x * v_scale * dt

        # Update the position:
        self.__target_pose['gcs']['position'][0] = (
            self.__last_relaxed_ik_pose['gcs']['position'][0] + dx
            # self.__target_pose['gcs']['position'][0] + dx
        )
        self.__target_pose['gcs']['position'][1] = (
            self.__last_relaxed_ik_pose['gcs']['position'][1] + dy
            # self.__target_pose['gcs']['position'][1] + dy
        )

    def __update_position_hpcs(self):
        """
        Joystick velocity control in Polar CS.
        
        """

        # Constants:
        dt = 1.0 / 100  # Time delta, seconds
        radius_scale = 0.1  # Units/second.
        height_scale = 0.075  # Units/second.
        angle_scale = {
            'z': 15.0,
            'y': 5.0,
        }

        d_radius = 0
        d_z_angle = 0
        d_y_angle = 0

        x_position = copy.deepcopy(
            self.__last_relaxed_ik_pose['gcs']['position'][0]
        )
        y_position = copy.deepcopy(
            self.__last_relaxed_ik_pose['gcs']['position'][1]
        )
        z_position = copy.deepcopy(
            self.__last_relaxed_ik_pose['gcs']['position'][2]
        )

        # if (
        #     self.__kinova_relaxed_ik_difference['linear'] >
        #     self.__difference_threshold['linear']
        # ):
        #     return

        # Update Y orientation:
        d_y_angle = (
            self.__oculus_joystick['left'].position_y * angle_scale['y'] * dt
        )
        self.__target_pose['vpcs']['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(
                    np.deg2rad(-d_y_angle),
                    (0, 1, 0),  # Around Y.
                ),
                self.__target_pose['vpcs']['orientation'],
            )
        )

        self.__target_pose['vpcs']['angle'] = (
            self.__target_pose['vpcs']['angle'] + d_y_angle
        )
        point_vpcs = np.array(
            [
                self.__target_pose['vpcs']['radius'],
                self.__target_pose['vpcs']['angle'],
            ]
        )
        self.__target_pose['eecs']['position'] = self.__vpcs_to_eecs(point_vpcs)
        self.__target_pose['eecs']['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(
                    np.deg2rad(d_y_angle),
                    (0, 1, 0),  # Around Y.
                ),
                self.__target_pose['eecs']['orientation'],
            )
        )

        # Calculate displacement based on velocity and time delta.
        # Update X and Y:
        if self.__control_mode == 'polar':
            d_radius = (
                self.__oculus_joystick['right'].position_y * radius_scale * dt
            )
            d_z_angle = (
                -self.__oculus_joystick['right'].position_x * angle_scale['z']
                * dt
            )

            self.__target_pose['hpcs']['radius'] = (
                self.__target_pose['hpcs']['radius'] + d_radius
            )
            self.__target_pose['hpcs']['angle'] = (
                self.__target_pose['hpcs']['angle'] + d_z_angle
            )

            # Protection against crossing PCS limits:
            radius_limit = {
                'upper': 0.33,
                'lower': -0.33,
            }
            if self.__target_pose['hpcs']['radius'] > radius_limit['upper']:
                self.__target_pose['hpcs']['radius'] = radius_limit['upper']

            if self.__target_pose['hpcs']['radius'] < radius_limit['lower']:
                self.__target_pose['hpcs']['radius'] = radius_limit['lower']

        # Update Z:
        elif self.__control_mode == 'height':
            d_height = (
                self.__oculus_joystick['right'].position_y * height_scale * dt
            )
            self.__target_pose['hpcs']['z_position'] = (
                self.__target_pose['hpcs']['z_position'] + d_height
            )

            # Protection against crossing PCS limits:
            # height_limit = {
            #     'upper': -0.047,
            #     'lower': -0.47,
            # }
            # if (
            #     self.__target_pose['hpcs']['z_position'] > height_limit['upper']
            #     + self.__target_pose['eecs']['position'][2]
            # ):
            #     self.__target_pose['hpcs']['z_position'] = height_limit['upper']

            # if (
            #     self.__target_pose['hpcs']['z_position'] < height_limit['lower']
            #     + self.__target_pose['eecs']['position'][2]
            # ):
            #     self.__target_pose['hpcs']['z_position'] = height_limit['lower']

        # Update target position:
        point_gcs = self.__hpcs_to_gcs(
            np.array(
                [
                    (
                        self.__target_pose['hpcs']['radius']
                        + self.__target_pose['eecs']['position'][0]
                    ),
                    self.__target_pose['hpcs']['angle'],
                    0,
                ]
            )
        )

        x_position, y_position = point_gcs[0], point_gcs[1]
        z_position = self.__target_pose['hpcs']['z_position']

        self.__target_pose['gcs']['position'] = np.array(
            [
                x_position,
                y_position,
                z_position,
            ]
        )

        # Update target orientation:
        # First convert PCS to GCS to apply rotation around Y.
        self.__target_pose['gcs']['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(
                    np.deg2rad(self.__target_pose['hpcs']['angle']),
                    (0, 0, 1),  # Around Z.
                ),
                self.__target_pose['vpcs']['orientation'],
            )
        )

        # Second apply additional rotation around Z.
        self.__target_pose['gcs']['orientation'] = (
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(
                    np.deg2rad(d_z_angle),
                    (0, 0, 1),  # Around Z.
                ),
                self.__target_pose['gcs']['orientation'],
            )
        )

    def __publish_target_pose(self):
        """
        
        """

        pose_message = Pose()
        pose_message.position.x = (self.__target_pose['gcs']['position'][0])
        pose_message.position.y = (self.__target_pose['gcs']['position'][1])
        pose_message.position.z = (
            self.__target_pose['gcs']['position'][2]
            + self.__target_pose['eecs']['position'][2]
        )

        pose_message.orientation.w = (
            self.__target_pose['gcs']['orientation'][0]
        )
        pose_message.orientation.x = (
            self.__target_pose['gcs']['orientation'][1]
        )
        pose_message.orientation.y = (
            self.__target_pose['gcs']['orientation'][2]
        )
        pose_message.orientation.z = (
            self.__target_pose['gcs']['orientation'][3]
        )

        # rospy.loginfo_throttle(10, pose_message)
        self.__teleoperation_pose.publish(pose_message)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        self.__mode_state_machine(self.__oculus_joystick['right'].button)

        self.__publish_target_pose()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.
        # try:
        self.__teleoperation_enable_tracking(False)
        self.__teleoperation_enable_full_mode(False)
        # except Exception as e:
        #     print(e)

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'oculus_joystick_mapping',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )

    robot_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    oculus_joystick_mapping = OculusJoystickMapping(
        node_name=node_name,
        robot_name=robot_name,
        controller_side=controller_side,
    )

    rospy.on_shutdown(oculus_joystick_mapping.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        oculus_joystick_mapping.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
