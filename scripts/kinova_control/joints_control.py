#!/usr/bin/env python
"""

"""

import rospy
import math

from std_msgs.msg import (Float64, Bool)
from sensor_msgs.msg import (JointState)

from kortex_driver.msg import (
    Base_JointSpeeds,
    JointSpeed,
    JointAngles,
)
from kortex_driver.srv import (Stop)
from kinova_positional_control.srv import (PidVelocityLimit)


class KinovaJointsControl:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        continous_joints_indices=(0, 2, 4, 6),
        max_speeds=[1.396, 1.396, 1.396, 1.396, 1.222, 1.222, 1.222],
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.MAX_SPEEDS = max_speeds  # Rad/s
        self.CONTINOUS_JOINTS_INDICES = continous_joints_indices
        self.JOINTS_NUMBER = 7

        # # Private variables:
        # Absolute joint positions at the moment of setting a new goal. Relative
        # current joint positions are relative to these positions
        self.__start_absolute_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Absolute current (feedback) joint positions.
        self.__current_absolute_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.__goal_absolute_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # These velocities are the output of PIDs, which will be sent to Kinova
        # joint velocity topic.
        self.__goal_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # # Public variables:
        self.velocity_fraction_limit = 1.0
        self.motion_finished_threshold = 0.01

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joints_control/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'kortex_driver': False,
        }

        self.__dependency_status_topics = {
            'kortex_driver':
                rospy.Subscriber(
                    f'/{self.ROBOT_NAME}/base_feedback/joint_state',
                    JointState,
                    self.__absolute_feedback_callback,
                ),
        }

        # # Service provider:
        rospy.Service(
            f'/{self.ROBOT_NAME}/joints_control/velocity_limit',
            PidVelocityLimit,
            self.__pid_velocity_limit_handler,
        )

        # # Service subscriber:
        self.__stop_arm_srv = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/stop',
            Stop,
        )

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joints_control/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__joint_velocity = rospy.Publisher(
            f'/{self.ROBOT_NAME}/in/joint_velocity',
            Base_JointSpeeds,
            queue_size=1,
        )

        self.__pid_motion_finished = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joints_control/motion_finished',
            Bool,
            queue_size=1,
        )

        self.__state_1 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_1/state',
            Float64,
            queue_size=1,
        )
        self.__state_2 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_2/state',
            Float64,
            queue_size=1,
        )
        self.__state_3 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_3/state',
            Float64,
            queue_size=1,
        )
        self.__state_4 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_4/state',
            Float64,
            queue_size=1,
        )
        self.__state_5 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_5/state',
            Float64,
            queue_size=1,
        )
        self.__state_6 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_6/state',
            Float64,
            queue_size=1,
        )
        self.__state_7 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_7/state',
            Float64,
            queue_size=1,
        )

        self.__setpoint_1 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_1/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_2 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_2/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_3 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_3/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_4 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_4/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_5 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_5/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_6 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_6/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_7 = rospy.Publisher(
            f'/{self.ROBOT_NAME}/joint_7/setpoint',
            Float64,
            queue_size=1,
        )

        # # Topic subscriber:
        self.__kortex_feedback = rospy.Subscriber(
            f'/{self.ROBOT_NAME}/base_feedback/joint_state',
            JointState,
            self.__absolute_feedback_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/relaxed_ik/joint_angle_solutions',
            JointAngles,
            self.__absolute_setpoint_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_1/control_effort',
            Float64,
            self.__control_effort_1_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_2/control_effort',
            Float64,
            self.__control_effort_2_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_3/control_effort',
            Float64,
            self.__control_effort_3_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_4/control_effort',
            Float64,
            self.__control_effort_4_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_5/control_effort',
            Float64,
            self.__control_effort_5_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_6/control_effort',
            Float64,
            self.__control_effort_6_callback,
        )
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/joint_7/control_effort',
            Float64,
            self.__control_effort_7_callback,
        )

    # # Dependency status callbacks:

    # # Service handlers:
    def __pid_velocity_limit_handler(self, req):
        """
        Sets a velocity limit in fractions (0.1 to 1.0).
        
        """

        # Block callback function until all components are initialized.
        if not self.__is_initialized:
            return False

        if req.data > 1.0:
            self.velocity_fraction_limit = 1.0

        elif req.data < 0.1:
            self.velocity_fraction_limit = 0.1

        else:
            self.velocity_fraction_limit = req.data

        return True

    # # Topic callbacks:
    def __absolute_feedback_callback(self, msg):
        """
        
        """

        # Update from zero to the current arm position on initialization.
        if not self.__is_initialized:
            self.__start_absolute_positions = list(msg.position)
            self.__goal_absolute_positions = list(msg.position)

            self.__dependency_status['kortex_driver'] = True

            rospy.loginfo(
                (
                    f'/{self.ROBOT_NAME}/joints_control: '
                    'kortex_driver was initialized!'
                ),
            )

            return

        self.__current_absolute_positions = msg.position

    def __absolute_setpoint_callback(self, msg):
        """
        
        """

        for joint_index in range(self.JOINTS_NUMBER):
            self.__goal_absolute_positions[joint_index] = (
                msg.joint_angles[joint_index].value
            )

    def __control_effort_1_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.__is_initialized:
            self.__goal_velocities[0] = msg.data

    def __control_effort_2_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.__is_initialized:
            self.__goal_velocities[1] = msg.data

    def __control_effort_3_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.__is_initialized:
            self.__goal_velocities[2] = msg.data

    def __control_effort_4_callback(self, msg):

        # Block callback function until all components are initialized.
        self.__goal_velocities[3] = msg.data

    def __control_effort_5_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.__is_initialized:
            self.__goal_velocities[4] = msg.data

    def __control_effort_6_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.__is_initialized:
            self.__goal_velocities[5] = msg.data

    def __control_effort_7_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.__is_initialized:
            self.__goal_velocities[6] = msg.data

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
                            f'/{self.ROBOT_NAME}/joints_control: '
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
                    f'/{self.ROBOT_NAME}/joints_control:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/joints_control: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __relative_feedback(self):
        """
        Recalculates joint feedback to be relative to the starting absolute
        position with start_absolute_positions being a relative origin (zero).
        
        """

        current_relative_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for joint_index in range(self.JOINTS_NUMBER):
            if joint_index not in self.CONTINOUS_JOINTS_INDICES:

                current_relative_positions[joint_index] = (
                    self.__current_absolute_positions[joint_index]
                    - self.__start_absolute_positions[joint_index]
                )

                continue

            # Recalculate joint relative feedback for continuous rotation joints
            # using geodesic distance.
            current_relative_positions[joint_index] = (
                self.__geodesic_distance(
                    self.__current_absolute_positions[joint_index],
                    self.__start_absolute_positions[joint_index]
                )
            )

        # BUG: if the joint overshoots at 180 commanded relative position it
        # might get stuck in the infinite 360 loop (keep rotating).

        # Publish relative position feedbacks to PIDs.
        self.__state_1.publish(round(current_relative_positions[0], 4))
        self.__state_2.publish(round(current_relative_positions[1], 4))
        self.__state_3.publish(round(current_relative_positions[2], 4))
        self.__state_4.publish(round(current_relative_positions[3], 4))
        self.__state_5.publish(round(current_relative_positions[4], 4))
        self.__state_6.publish(round(current_relative_positions[5], 4))
        self.__state_7.publish(round(current_relative_positions[6], 4))

    def __relative_setpoint(self):
        """
        Calculates geodesic (shortest) angular distance and sets the goal
        positions relative to current positions.

        """

        goal_relative_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Set origin positions.
        self.__start_absolute_positions = self.__current_absolute_positions

        for joint_index in range(self.JOINTS_NUMBER):
            if joint_index not in self.CONTINOUS_JOINTS_INDICES:
                goal_relative_positions[joint_index] = (
                    self.__goal_absolute_positions[joint_index]
                    - self.__current_absolute_positions[joint_index]
                )

                continue

            # Recalculate joint relative setpoint for continuous rotation joints
            # using geodesic distance.
            goal_relative_positions[joint_index] = (
                self.__geodesic_distance(
                    self.__goal_absolute_positions[joint_index],
                    self.__current_absolute_positions[joint_index]
                )
            )

        # Publish relative position setpoints to PIDs.
        self.__setpoint_1.publish(round(goal_relative_positions[0], 4))
        self.__setpoint_2.publish(round(goal_relative_positions[1], 4))
        self.__setpoint_3.publish(round(goal_relative_positions[2], 4))
        self.__setpoint_4.publish(round(goal_relative_positions[3], 4))
        self.__setpoint_5.publish(round(goal_relative_positions[4], 4))
        self.__setpoint_6.publish(round(goal_relative_positions[5], 4))
        self.__setpoint_7.publish(round(goal_relative_positions[6], 4))

    def __geodesic_distance(self, position1, position2):
        """

        http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html#Geodesic-distance-and-interpolation
        
        """

        __geodesic_distance = 0.0

        # Resulting position is within (-180; 180].
        if (-1 * math.pi < position1 - position2 <= math.pi):
            __geodesic_distance = (position1 - position2)

        # Resulting position is crossing -180/180 border from the negative sign
        # to the positive sign (Example: -20: -170 to 170).
        elif (position1 - position2 > math.pi):
            __geodesic_distance = (position1 - position2 - 2 * math.pi)

        # Resulting position is crossing 180/-180 border from the positive sign
        # to the negative sign (Example: +20: 170 to -170).
        elif (position1 - position2 <= -1 * math.pi):
            __geodesic_distance = (position1 - position2 + 2 * math.pi)

        return __geodesic_distance

    def __publish_goal_velocities(self):
        """
        
        """

        # Form a velocity message.
        velocity_message = Base_JointSpeeds()
        velocities = []

        for joint_index in range(self.JOINTS_NUMBER):
            joint_velocity = JointSpeed()
            joint_velocity.joint_identifier = joint_index

            # Limit the output velocity.
            if (
                abs(self.__goal_velocities[joint_index]) >
                self.MAX_SPEEDS[joint_index] * self.velocity_fraction_limit
            ):

                if self.__goal_velocities[joint_index] >= 0:
                    self.__goal_velocities[joint_index] = (
                        self.MAX_SPEEDS[joint_index]
                        * self.velocity_fraction_limit
                    )

                else:
                    self.__goal_velocities[joint_index] = (
                        -1 * self.MAX_SPEEDS[joint_index]
                        * self.velocity_fraction_limit
                    )

            # Remove constant back and forth motion when stationary.
            if (abs(self.__goal_velocities[joint_index]) < 0.005):
                self.__goal_velocities[joint_index] = 0.0

            joint_velocity.value = self.__goal_velocities[joint_index]
            joint_velocity.duration = 0  # Or 0.000333s
            velocities.append(joint_velocity)

        velocity_message.joint_speeds = velocities
        velocity_message.duration = 0

        # Publish a velocity message.
        self.__joint_velocity.publish(velocity_message)

        rospy.logdebug_throttle_identical(
            10,
            (
                f'j1_vel: {round(velocity_message.joint_speeds[0].value, 2)} '
                f'j2_vel: {round(velocity_message.joint_speeds[1].value, 2)} '
                f'j3_vel: {round(velocity_message.joint_speeds[2].value, 2)} '
                f'j4_vel: {round(velocity_message.joint_speeds[3].value, 2)} '
                f'j5_vel: {round(velocity_message.joint_speeds[4].value, 2)} '
                f'j6_vel: {round(velocity_message.joint_speeds[5].value, 2)} '
                f'j7_vel: {round(velocity_message.joint_speeds[6].value, 2)} '
            ),
        )

    def __publish_joint_motion_finished(self):
        """
        Checks if all joints have an absolute velocity value lower than a
        threshold.
        
        """

        motion_finished = True

        for velocity in self.__goal_velocities:
            if abs(velocity) > self.motion_finished_threshold:
                motion_finished = False
                break

        self.__pid_motion_finished.publish(motion_finished)

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        # Recalculate absolute feedback and setpoint into relative coordinates.
        self.__relative_feedback()
        self.__relative_setpoint()

        self.__publish_joint_motion_finished()
        self.__publish_goal_velocities()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/joints_control: node is shutting down...',
        )

        # Stop arm movement
        self.__stop_arm_srv()

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/joints_control: node has shut down.',
        )


def main():
    """
    
    """

    rospy.init_node(
        'joints_control',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    joints_control = KinovaJointsControl(robot_name=kinova_name)

    rospy.on_shutdown(joints_control.node_shutdown)

    while not rospy.is_shutdown():
        joints_control.main_loop()


if __name__ == '__main__':
    main()
