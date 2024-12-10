#!/usr/bin/env python
"""

Author(s):

TODO:

"""

# # Standart libraries:
import rospy
import sys
import moveit_commander
from moveit_commander import (
    RobotCommander,
    PlanningSceneInterface,
    MoveGroupCommander,
)
import tf2_ros
import tf2_geometry_msgs  # Is required for tf2_ros.Buffer.transform().ge

import tf.transformations as tf
import numpy as np
from copy import (deepcopy)

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float64,
)
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
)
from sensor_msgs.msg import (JointState)
from std_srvs.srv import (Empty)

# # Third party messages and services:
from moveit_msgs.msg import (DisplayTrajectory)


class PresetPoses:
    """
    
    """

    def __init__(
        self,
        node_name,
        robot_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__ROBOT_NAME = robot_name

        # # Public CONSTANTS:
        self.PUBLIC_CONTANT = 1

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__move_group_is_initialized = False
        self.__plan = None

        self.__trajectory_plan_fraction = 0.0
        self.__trajectory_execution_finished = True

        # # Public variables:
        self.public_variable = 1

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.__ROBOT_NAME}/preset_poses/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {}

        self.__dependency_status['kortex_driver'] = False

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['kortex_driver'] = (
            rospy.Subscriber(
                f'/{self.__ROBOT_NAME}/base_feedback/joint_state',
                JointState,
                self.__kinova_feedback_callback,
            )
        )

        # # Service provider:
        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/side_arm_pose',
            Empty,
            self.__side_arm_pose_handler,
        )
        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/narrow_pose',
            Empty,
            self.__narrow_pose_handler,
        )
        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/home_pose',
            Empty,
            self.__home_pose_handler,
        )

        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/front_xy_grasp_pose',
            Empty,
            self.__front_xy_grasp_pose_handler,
        )
        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/front_xz_grasp_pose',
            Empty,
            self.__front_xz_grasp_pose_handler,
        )
        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/top_xz_grasp_pose',
            Empty,
            self.__top_xz_grasp_pose_handler,
        )
        rospy.Service(
            f'/{self.__ROBOT_NAME}/preset_poses/top_yz_grasp_pose',
            Empty,
            self.__top_yz_grasp_pose_handler,
        )

        # # Service subscriber:

        # # Topic publisher:
        self.__trajectory_finished = rospy.Publisher(
            f'/{self.__ROBOT_NAME}/preset_poses/trajectory_finished',
            Bool,
            queue_size=1,
        )
        self.__trajectory_fraction = rospy.Publisher(
            f'/{self.__ROBOT_NAME}/preset_poses/trajectory_fraction',
            Float64,
            queue_size=1,
        )

        # # Topic subscriber:

        # # Timers:

        # # TF broadcaster:

        # # TF listener:
        self.__tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        tf2_ros.TransformListener(self.__tf_buffer)

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __kinova_feedback_callback(self, message):
        """Monitors /<node_name>/is_initialized topic.
        
        """

        self.__dependency_status['kortex_driver'] = True

    # # Service handlers:
    def __side_arm_pose_handler(self, request):
        """
        
        """

        target_pose = Pose()
        target_pose.position.x = 0.24
        target_pose.position.y = -0.36
        target_pose.position.z = -0.76

        if self.__ROBOT_NAME == 'left_arm':
            target_pose.position.y = -target_pose.position.y

        rpy_deg = np.deg2rad(np.array([135.0, 0.0, 90.0]))
        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='chest_link',
        )

        return []

    def __narrow_pose_handler(self, request):
        """
        
        """

        target_pose = Pose()
        target_pose.position.x = 0.5
        target_pose.position.y = -0.1
        target_pose.position.z = -0.35

        rpy_deg = np.deg2rad(np.array([90, 0, 90 + 10]))

        if self.__ROBOT_NAME == 'left_arm':
            rpy_deg = np.deg2rad(np.array([90, 0, 90 - 10]))
            target_pose.position.y = -target_pose.position.y

        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='chest_link',
        )

        return []

    def __home_pose_handler(self, request):
        """
        
        """

        target_pose = Pose()
        target_pose.position.x = 0.6
        target_pose.position.y = -0.36
        target_pose.position.z = -0.35

        rpy_deg = np.deg2rad(np.array([90, 0, 90 + 10]))
        if self.__ROBOT_NAME == 'left_arm':
            rpy_deg = np.deg2rad(np.array([90, 0, 90 - 10]))
            target_pose.position.y = -target_pose.position.y

        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='chest_link',
        )

        return []

    def __front_xy_grasp_pose_handler(self, request):
        """
        
        """

        # Current pose in kinova/base_link.
        current_pose = self.__arm_group.get_current_pose().pose

        # Current pose in base_link.
        current_pose = self.__convert_pose(
            pose=current_pose,
            from_frame=f'{self.__ROBOT_NAME}/base_link',
            to_frame='base_link',
        )

        target_pose = Pose()

        # Keep the same position.
        target_pose.position = current_pose.position

        # Update the orientation:
        rpy_deg = np.deg2rad(np.array([90, 0, 90]))
        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='base_link',
        )

        return []

    def __front_xz_grasp_pose_handler(self, request):
        """
        
        """

        # Current pose in kinova/base_link.
        current_pose = self.__arm_group.get_current_pose().pose

        # Current pose in base_link.
        current_pose = self.__convert_pose(
            pose=current_pose,
            from_frame=f'{self.__ROBOT_NAME}/base_link',
            to_frame='base_link',
        )

        target_pose = Pose()

        # Keep the same position.
        target_pose.position = current_pose.position

        # Update the orientation:
        rpy_deg = np.deg2rad(np.array([90, -90, 90]))
        if self.__ROBOT_NAME == 'left_arm':
            rpy_deg = np.deg2rad(np.array([90, 90, 90]))

        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='base_link',
        )

        return []

    def __top_xz_grasp_pose_handler(self, request):
        """
        
        """

        # Current pose in kinova/base_link.
        current_pose = self.__arm_group.get_current_pose().pose

        # Current pose in base_link.
        current_pose = self.__convert_pose(
            pose=current_pose,
            from_frame=f'{self.__ROBOT_NAME}/base_link',
            to_frame='base_link',
        )

        target_pose = Pose()

        # Keep the same position.
        target_pose.position = current_pose.position

        # Update the orientation:
        rpy_deg = np.deg2rad(np.array([180, 0, 0]))
        if self.__ROBOT_NAME == 'left_arm':
            rpy_deg = np.deg2rad(np.array([180, 0, 180]))

        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='base_link',
        )

        return []

    def __top_yz_grasp_pose_handler(self, request):
        """
        
        """

        # Current pose in kinova/base_link.
        current_pose = self.__arm_group.get_current_pose().pose

        # Current pose in base_link.
        current_pose = self.__convert_pose(
            pose=current_pose,
            from_frame=f'{self.__ROBOT_NAME}/base_link',
            to_frame='base_link',
        )

        target_pose = Pose()

        # Keep the same position.
        target_pose.position = current_pose.position

        # Update the orientation:
        rpy_deg = np.deg2rad(np.array([-180, 0, 90]))
        quaternion = tf.quaternion_from_euler(
            rpy_deg[0],
            rpy_deg[1],
            rpy_deg[2],
        )

        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]

        self.__set_target_pose(
            target_pose=target_pose,
            target_frame='base_link',
        )

        return []

    # # Topic callbacks:

    # # Timer callbacks:

    # # Private methods:
    # NOTE: By default all new class methods should be private.
    def __check_initialization(self):
        """Monitors required criteria and sets is_initialized variable.

        Monitors nodes' dependency status by checking if dependency's
        is_initialized topic has at most one publisher (this ensures that
        dependency node is alive and does not have any duplicates) and that it
        publishes True. If dependency's status was True, but get_num_connections
        is not equal to 1, this means that the connection is lost and emergency
        actions should be performed.

        Once all dependencies are initialized and additional criteria met, the
        nodes' is_initialized status changes to True. This status can change to
        False any time to False if some criteria are no longer met.
        
        """

        self.__dependency_initialized = True

        for key in self.__dependency_status:
            if self.__dependency_status_topics[key].get_num_connections() != 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        (
                            f'/{self.__ROBOT_NAME}/preset_poses: '
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
                    f'/{self.__ROBOT_NAME}/preset_poses:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        if (
            self.__dependency_initialized
            and not self.__move_group_is_initialized
        ):
            # Initialize MoveIt commander.
            moveit_commander.roscpp_initialize(sys.argv)
            self.__intialize_moveit()

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized and self.__move_group_is_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __intialize_moveit(self):
        """
        
        """

        try:
            self.__is_gripper_present = rospy.get_param(
                rospy.get_namespace() + 'is_gripper_present', False
            )

            if self.__is_gripper_present:
                gripper_joint_names = rospy.get_param(
                    rospy.get_namespace() + 'gripper_joint_names', []
                )
                self.__gripper_joint_name = gripper_joint_names[0]

            else:
                self.__gripper_joint_name = ''

            # Create the MoveItInterface necessary objects.
            self.__robot = RobotCommander('robot_description')
            self.__scene = PlanningSceneInterface(ns=rospy.get_namespace())

            self.__arm_group = MoveGroupCommander(
                'arm',
                ns=rospy.get_namespace(),
            )
            self.__arm_group.set_planner_id('BiTRRT')

            self.__display_trajectory_pub = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path',
                DisplayTrajectory,
                queue_size=20,
            )

            if self.__is_gripper_present:
                self.__gripper_group = MoveGroupCommander(
                    'gripper',
                    ns=rospy.get_namespace(),
                )

        except Exception as e:
            print(e)
            self.__move_group_is_initialized = False

        else:
            self.__move_group_is_initialized = True

    def __convert_pose(
        self,
        pose: Pose,
        from_frame: str,
        to_frame: str,
    ) -> Pose:
        """
        
        """

        # 1. Convert to PoseStamped().
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = from_frame
        pose_stamped.pose.position = pose.position
        pose_stamped.pose.orientation = pose.orientation

        # 2. Convert from "from_frame" to "to_frame".
        try:
            # NOTE: Requires import tf2_geometry_msgs to work.
            pose_in_new_frame = self.__tf_buffer.transform(
                object_stamped=pose_stamped, target_frame=to_frame
            )

            pose_message = Pose()
            pose_message.position = pose_in_new_frame.pose.position
            pose_message.orientation = pose_in_new_frame.pose.orientation

            return pose_message

        except Exception as ex:
            rospy.logerr(
                f'/{self.__ROBOT_NAME}/preset_poses:'
                f'\nError in __convert_pose: {ex}'
            )

            return None

    def __set_target_pose(
        self,
        target_pose: Pose,
        target_frame: str,
        velocity_scaling_factor: float = 1.0,
        acceleration_scaling_factor: float = 0.75,
    ):
        """

        """

        pose_message = self.__convert_pose(
            pose=target_pose,
            from_frame=target_frame,
            to_frame=f'{self.__ROBOT_NAME}/base_link',
        )

        waypoints = [pose_message]

        try:
            target_reached = False
            number_attempts = 5

            for attempt_ix in range(number_attempts):
                if target_reached:
                    break

                eef_step = 0.005
                best_eef_step = 0.0
                best_fraction = 0.0
                best_plan = None

                while True:
                    if best_fraction == 1.0 or eef_step > 0.1:
                        break

                    (plan, fraction) = self.__arm_group.compute_cartesian_path(
                        waypoints=waypoints,
                        eef_step=eef_step,
                    )

                    eef_step += 0.001

                    if fraction > best_fraction:
                        best_fraction = fraction
                        best_eef_step = eef_step
                        best_plan = deepcopy(plan)

                        rospy.loginfo(
                            f'/{self.__ROBOT_NAME}/preset_poses: '
                            f'\nPlan fraction (cartesian): {round(best_fraction, 3)}'
                            f'\neef_step: {round(best_eef_step, 3)}'
                        )

                rospy.logwarn(
                    f'/{self.__ROBOT_NAME}/preset_poses: '
                    f'\nAttempt: {attempt_ix+1} / {number_attempts}'
                    f'\nBest fraction: {round(best_fraction, 3)}'
                    f'\nBest eef_step: {round(best_eef_step, 3)}'
                )

                self.__trajectory_plan_fraction = best_fraction

                if best_fraction > 0.0:
                    # Retime the trajectory to account for velocity and accleration
                    # scales:
                    plan = self.__arm_group.retime_trajectory(
                        self.__robot.get_current_state(),
                        best_plan,
                        velocity_scaling_factor,  #1.0
                        acceleration_scaling_factor,  #0.9
                        # algorithm='time_optimal_trajectory_generation',
                    )

                    self.__trajectory_execution_finished = False
                    self.__trajectory_execution_finished = (
                        self.__arm_group.execute(plan, wait=True)
                    )

                    if best_fraction == 1.0:
                        target_reached = True

                else:
                    rospy.logerr(
                        f'/{self.__ROBOT_NAME}/preset_poses: unreachable pose!'
                        f'\nPlan fraction: {round(fraction, 3)}'
                        f'\neef_step: {round(eef_step, 3)}'
                    )

        except Exception as ex:
            rospy.logerr(
                f'/{self.__ROBOT_NAME}/preset_poses:'
                f'\nError in __set_target_pose: {ex}'
            )

    def __display_trajectory(self):
        """
        
        """

        if self.__plan:
            display_trajectory = DisplayTrajectory()
            display_trajectory.trajectory_start = self.__arm_group.get_current_state(
            )
            display_trajectory.trajectory.append(self.__plan)
            self.__display_trajectory_pub.publish(display_trajectory)

    def __update_scene(self):
        """
        
        """

        box_pose_stamped = PoseStamped()
        box_pose_stamped.header.stamp = rospy.Time.now()
        box_pose_stamped.header.frame_id = 'base_link'
        box_pose_stamped.pose.position.x = 0
        box_pose_stamped.pose.position.y = 0
        box_pose_stamped.pose.position.z = 0.36 / 2
        box_pose_stamped.pose.orientation.x = 0
        box_pose_stamped.pose.orientation.y = 0
        box_pose_stamped.pose.orientation.z = 0
        box_pose_stamped.pose.orientation.w = 1

        self.__scene.add_box(
            name='mobile_base',
            pose=box_pose_stamped,
            size=(0.58, 0.54, 0.36),
        )

    # # Public methods:
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.

        self.__display_trajectory()
        self.__update_scene()

        self.__trajectory_finished.publish(self.__trajectory_execution_finished)
        self.__trajectory_fraction.publish(self.__trajectory_plan_fraction)

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.__ROBOT_NAME}/preset_poses: node is shutting down...',
        )

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.

        self.__arm_group.stop()

        rospy.loginfo_once(
            f'/{self.__ROBOT_NAME}/preset_poses: node has shut down.',
        )


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'preset_poses',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    robot_name = rospy.get_param(
        param_name=f'{node_name}/robot_name',
        default='my_gen3',
    )

    class_instance = PresetPoses(
        node_name=node_name,
        robot_name=robot_name,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
