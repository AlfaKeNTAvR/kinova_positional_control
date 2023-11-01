#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
from copy import (copy)
import quaternion
import transformations

from std_msgs.msg import (Bool)
from geometry_msgs.msg import (Pose)
from std_srvs.srv import (Trigger)

from kinova_positional_control.msg import (Waypoint)
from kinova_positional_control.srv import (UploadTrajectory)


class KinovaTrajectory:
    """
    
    """

    def __init__(
        self,
        node_name,
        robot_name,
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.NODE_NAME = node_name
        self.ROBOT_NAME = robot_name
        self.MAX_LINEAR_SPEED = 0.2  # [meters/s].
        self.MAX_ANGULAR_SPEED = 90  # [deg/s].
        self.MIN_LOOP_FREQUENCY = 100

        # # Private variables:
        self.__resume_trajectory = False
        self.__motion_is_sampled = False
        self.__waypoint_index = 0
        self.__sample_index = 0
        self.__trajectory_is_finished = True
        self.__trajectory = []
        self.__coordinate_system = 'gcs'

        self.__loop_frequency = copy(self.MIN_LOOP_FREQUENCY)

        self.__current_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.__target_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.__waypoint_index = 0
        self.__path_duration = 0.0
        self.__path_precision = {
            'linear': 0.0,
            'angular': 0.0,
        }
        self.__point_precision = {
            'linear': 0.0,
            'angular': 0.0,
        }

        # Missalignment between Kinova and Relaxed IK.
        self.__kinova_relaxed_ik_missalignment = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.__last_relaxed_ik_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # End-Effector Corrdinate System:
        self.__gcs_to_eecs = transformations.quaternion_matrix(
            np.array([1, 0, 0, 0])
        )
        self.__eecs_to_gcs = transformations.inverse_matrix(self.__gcs_to_eecs)
        self.__eecs_origin = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {
            'positional_control': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            'positional_control':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/positional_control/is_initialized',
                    Bool,
                    self.__positional_control_callback,
                ),
        }

        # # Service provider:
        rospy.Service(
            f'{self.NODE_NAME}/upload_trajectory',
            UploadTrajectory,
            self.__upload_trajectory_handler,
        )
        rospy.Service(
            f'{self.NODE_NAME}/resume_trajectory',
            Trigger,
            self.__resume_trajectory_handler,
        )
        rospy.Service(
            f'{self.NODE_NAME}/pause_trajectory',
            Trigger,
            self.__pause_trajectory_handler,
        )
        rospy.Service(
            f'{self.NODE_NAME}/cancel_trajectory',
            Trigger,
            self.__cancel_trajectory_handler,
        )

        # # Service subscriber:

        # # Topic publisher:
        self.__kinova_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/positional_control/input_pose',
            Pose,
            queue_size=1,
        )
        self.__trajectory_finished = rospy.Publisher(
            f'{self.NODE_NAME}/trajectory_finished',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/positional_control/kinova_relaxed_ik_missalignment',
            Pose,
            self.__kinova_relaxed_ik_missalignment_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __positional_control_callback(self, message):
        """Monitors positional_control is_initialized topic."""

        self.__dependency_status['positional_control'] = message.data

    # # Service handlers:
    def __upload_trajectory_handler(self, request):
        """
        
        """

        self.__resume_trajectory = False
        self.__trajectory_is_finished = True

        self.__trajectory = []
        self.__coordinate_system = request.coordinate_system

        if self.__coordinate_system not in ['gcs', 'eecs']:
            response = False

            return response

        if self.__coordinate_system == 'eecs':
            self.__eecs_to_gcs = transformations.quaternion_matrix(
                self.__last_relaxed_ik_pose['orientation']
            )
            self.__gcs_to_eecs = transformations.inverse_matrix(
                self.__eecs_to_gcs
            )
            self.__eecs_origin = copy(self.__last_relaxed_ik_pose)

        # Append current Kinova position as a first trajectory waypoint.
        current_pose_waypoint = self.__get_current_pose_waypoint()

        self.__trajectory.append(current_pose_waypoint)

        # Parse waypoint array.
        for waypoint in request.trajectory.waypoints:
            trajectory_waypoint = {
                'pose':
                    {
                        'position':
                            np.array(
                                [
                                    waypoint.pose.position.x,
                                    waypoint.pose.position.y,
                                    waypoint.pose.position.z,
                                ]
                            ),
                        'orientation':
                            np.array(
                                [
                                    waypoint.pose.orientation.w,
                                    waypoint.pose.orientation.x,
                                    waypoint.pose.orientation.y,
                                    waypoint.pose.orientation.z,
                                ]
                            ),
                    },
                'point_lin_prc': waypoint.point_lin_prc,
                'point_ang_prc': waypoint.point_ang_prc,
                'path_lin_prc': waypoint.path_lin_prc,
                'path_ang_prc': waypoint.path_ang_prc,
                'speed_frac': waypoint.speed_frac,
            }

            if self.__coordinate_system == 'eecs':
                trajectory_waypoint['pose']['position'] = (
                    self.__eecs_origin['position'] + np.matmul(
                        self.__eecs_to_gcs[0:3, 0:3],
                        trajectory_waypoint['pose']['position'],
                    )
                )
                trajectory_waypoint['pose']['orientation'] = (
                    transformations.quaternion_multiply(
                        self.__eecs_origin['orientation'],
                        trajectory_waypoint['pose']['orientation'],
                    )
                )

            self.__trajectory.append(trajectory_waypoint)

        # Reset trajectory sampler and executor.
        self.__motion_is_sampled = False
        self.__waypoint_index = 0
        self.__sample_index = 0

        response = True

        return response

    def __resume_trajectory_handler(self, request):
        """
        
        """

        success = False
        message = 'Error: Trajectory is not loaded.'

        if self.__trajectory:
            self.__resume_trajectory = True

            success = True
            message = 'Trajectory execution is resumed.'

        return success, message

    def __pause_trajectory_handler(self, request):
        """
        
        """
        success = False
        message = 'Error: Trajectory is not loaded.'

        if self.__trajectory:
            self.__resume_trajectory = False

            success = True
            message = 'Trajectory execution is paused.'

        return success, message

    def __cancel_trajectory_handler(self, request):
        """
        
        """

        success = False
        message = 'Error: Trajectory is not loaded.'

        # Protection against the robot moving to a point with waypoint_index = 0
        # and sample_index = 0.
        if self.__resume_trajectory:
            success = False
            message = 'Error: Pause the trajectory before canceling.'

            return success, message

        if self.__trajectory:
            self.__trajectory_is_finished = True
            self.__trajectory[0] = self.__get_current_pose_waypoint()

            # Reset trajectory sampler and executor.
            self.__motion_is_sampled = False
            self.__waypoint_index = 0
            self.__sample_index = 0

            success = True
            message = 'Trajectory execution is cancelled.'

        return success, message

    # # Topic callbacks:
    def __kinova_relaxed_ik_missalignment_callback(self, message):
        """

        """

        self.__kinova_relaxed_ik_missalignment['position'] = np.array(
            [
                message.position.x,
                message.position.y,
                message.position.z,
            ]
        )
        self.__kinova_relaxed_ik_missalignment['orientation'] = np.array(
            [
                message.orientation.w,
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
            ]
        )

    def __commanded_pose_callback(self, message):
        """
        
        """

        self.__last_relaxed_ik_pose['position'][0] = message.position.x
        self.__last_relaxed_ik_pose['position'][1] = message.position.y
        self.__last_relaxed_ik_pose['position'][2] = message.position.z

        self.__last_relaxed_ik_pose['orientation'][0] = message.orientation.w
        self.__last_relaxed_ik_pose['orientation'][1] = message.orientation.x
        self.__last_relaxed_ik_pose['orientation'][2] = message.orientation.y
        self.__last_relaxed_ik_pose['orientation'][3] = message.orientation.z

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
                            f'/{self.ROBOT_NAME}{self.NODE_NAME}: '
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
                    waiting_for += f'\n- waiting for {key} node...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.ROBOT_NAME}{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

            # Reset trajectory sampler and executor.
            self.__motion_is_sampled = False
            self.__waypoint_index = 0
            self.__sample_index = 0

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}{self.NODE_NAME}: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __get_current_pose_waypoint(self):
        """
        
        """

        current_pose_waypoint = {
            'pose':
                {
                    'position':
                        np.array(
                            [
                                self.__last_relaxed_ik_pose['position'][0],
                                self.__last_relaxed_ik_pose['position'][1],
                                self.__last_relaxed_ik_pose['position'][2],
                            ]
                        ),
                    'orientation':
                        np.array(
                            [
                                self.__last_relaxed_ik_pose['orientation'][0],
                                self.__last_relaxed_ik_pose['orientation'][1],
                                self.__last_relaxed_ik_pose['orientation'][2],
                                self.__last_relaxed_ik_pose['orientation'][3],
                            ]
                        ),
                },
            'point_lin_prc': 0.0,
            'point_ang_prc': 0.0,
            'path_lin_prc': 0.0,
            'path_ang_prc': 0.0,
            'speed_frac': 0.0,
        }

        return current_pose_waypoint

    def __sample_motion(self):
        """
        
        """

        if self.__waypoint_index == len(self.__trajectory) - 1:
            self.__trajectory_is_finished = True
            # self.__waypoint_index = 0
            return

        self.__trajectory_is_finished = False

        # Assign variables according to current trajectory point.
        self.__current_pose['orientation'] = (
            self.__trajectory[self.__waypoint_index]['pose']['orientation']
        )
        self.__current_pose['position'] = (
            self.__trajectory[self.__waypoint_index]['pose']['position']
        )

        self.__target_pose['orientation'] = (
            self.__trajectory[self.__waypoint_index + 1]['pose']['orientation']
        )
        self.__target_pose['position'] = (
            self.__trajectory[self.__waypoint_index + 1]['pose']['position']
        )

        self.__point_precision['linear'] = (
            self.__trajectory[self.__waypoint_index + 1]['point_lin_prc']
        )
        self.__point_precision['angular'] = (
            self.__trajectory[self.__waypoint_index + 1]['point_ang_prc']
        )
        self.__path_precision['linear'] = (
            self.__trajectory[self.__waypoint_index + 1]['path_lin_prc']
        )
        self.__path_precision['angular'] = (
            self.__trajectory[self.__waypoint_index + 1]['path_ang_prc']
        )

        # Calculate position A to B path duration:
        position_difference = np.linalg.norm(
            self.__current_pose['position'] - self.__target_pose['position']
        )
        position_path_duration = (
            position_difference / (
                self.MAX_LINEAR_SPEED
                * self.__trajectory[self.__waypoint_index + 1]['speed_frac']
            )
        )

        # Calculate orientation A to B path duration based on angular distance
        # between two quaternions:
        quaternion_distance = transformations.quaternion_multiply(
            transformations.quaternion_inverse(
                self.__target_pose['orientation']
            ),
            self.__current_pose['orientation'],
        )
        angular_distance = np.rad2deg(
            2 * np.arctan2(
                np.linalg.norm(quaternion_distance[1:4]),
                quaternion_distance[0],
            )
        )
        orientation_path_duration = (
            angular_distance / (
                self.MAX_ANGULAR_SPEED
                * self.__trajectory[self.__waypoint_index + 1]['speed_frac']
            )
        )

        # Select the maximum path duration (position or orientation) for
        # trajectory sampling:
        self.__path_duration = max(
            position_path_duration,
            orientation_path_duration,
        )

        # Protection against almost identical poses in trajectoty, which cannot
        # be sampled because of too small distance difference.
        if self.__path_duration < 0.1:
            self.__waypoint_index += 1
            return

        self.__num_samples = int(self.__loop_frequency * self.__path_duration)
        self.__t = np.linspace(0, self.__path_duration, self.__num_samples)

        # Quaternion trajectory is sampled all at once in advance using
        # interpolation, positional samples are calculated in __execute_motion
        # function using a formula.
        self.__interpolated_quaternions = quaternion.slerp(
            quaternion.as_quat_array(self.__current_pose['orientation']),
            quaternion.as_quat_array(self.__target_pose['orientation']),
            self.__t[0],
            self.__t[-1],
            self.__t,
        )

        self.__sample_index = 0
        self.__motion_is_sampled = True

        self.__waypoint_index += 1

    def __execute_motion(self):
        """
        
        """

        rospy.Rate(self.__loop_frequency).sleep()

        # yapf: disable
        sample = (
            self.__current_pose['position']
            + (self.__target_pose['position'] - self.__current_pose['position'])
            * (1 - (np.cos(np.pi * self.__t[self.__sample_index] / self.__path_duration)))
            / 2
        )
        # yapf: enable

        pose_message = Pose()
        pose_message.position.x = sample[0]
        pose_message.position.y = sample[1]
        pose_message.position.z = sample[2]
        pose_message.orientation.w = (
            self.__interpolated_quaternions[self.__sample_index].w
        )
        pose_message.orientation.x = (
            self.__interpolated_quaternions[self.__sample_index].x
        )
        pose_message.orientation.y = (
            self.__interpolated_quaternions[self.__sample_index].y
        )
        pose_message.orientation.z = (
            self.__interpolated_quaternions[self.__sample_index].z
        )

        self.__kinova_pose.publish(pose_message)

        # Calculate linear and angular differences between Relaxed IK and Kinova
        # using position and quaternion misalignment:
        linear_difference = round(
            np.linalg.norm(self.__kinova_relaxed_ik_missalignment['position']),
            2,
        )
        angular_difference = round(
            np.rad2deg(
                2 * np.arctan2(
                    np.linalg.norm(
                        self.__kinova_relaxed_ik_missalignment['orientation']
                        [1:4]
                    ),
                    self.__kinova_relaxed_ik_missalignment['orientation'][0],
                )
            ),
            2,
        )

        if angular_difference > 180:
            angular_difference = 360 - angular_difference

        # print(
        #     f'Linear difference: {linear_difference}\n'
        #     f'Angular difference: {angular_difference}\n'
        # )

        # Current sample is the final sample.
        if self.__sample_index == self.__num_samples - 1:
            # Final sample was reached.
            if (
                abs(linear_difference) <= self.__point_precision['linear']
                and abs(angular_difference) <= self.__point_precision['angular']
            ):
                self.__motion_is_sampled = False

                return

        else:
            # Intermidiate sample was reached.
            if (
                abs(linear_difference) <= self.__path_precision['linear']
                and abs(angular_difference) <= self.__path_precision['angular']
            ):
                self.__sample_index += 1

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__trajectory_finished.publish(self.__trajectory_is_finished)

        if not self.__resume_trajectory:
            return

        if not self.__motion_is_sampled:
            self.__sample_motion()

        else:
            self.__execute_motion()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}{self.NODE_NAME}: node is shutting down...',
        )

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}{self.NODE_NAME}: node has shut down.',
        )


def define_waypoint(
    position,
    orientation,
    point_lin_prc=0.05,
    point_ang_prc=1.0,
    path_lin_prc=0.1,
    path_ang_prc=5.0,
    speed_frac=0.5,
):
    """
        
    """

    orientation_quat = transformations.quaternion_from_euler(
        np.deg2rad(orientation[0]),
        np.deg2rad(orientation[1]),
        np.deg2rad(orientation[2]),
    )

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = orientation_quat[0]
    pose.orientation.x = orientation_quat[1]
    pose.orientation.y = orientation_quat[2]
    pose.orientation.z = orientation_quat[3]

    waypoint = Waypoint()
    waypoint.pose = pose
    waypoint.point_lin_prc = point_lin_prc
    waypoint.point_ang_prc = point_ang_prc
    waypoint.path_lin_prc = path_lin_prc
    waypoint.path_ang_prc = path_ang_prc
    waypoint.speed_frac = speed_frac

    return waypoint


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'trajectory',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_name = rospy.get_name()

    robot_name = rospy.get_param(
        param_name=f'{node_name}/robot_name',
        default='my_gen3',
    )

    kinova_trajectory = KinovaTrajectory(
        node_name=node_name,
        robot_name=robot_name,
    )

    rospy.on_shutdown(kinova_trajectory.node_shutdown)

    while not rospy.is_shutdown():
        kinova_trajectory.main_loop()


if __name__ == '__main__':
    main()