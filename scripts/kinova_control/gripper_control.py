#!/usr/bin/env python
"""Implements Kinova Gen3 7DoF gripper control module.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2023.

"""

import rospy
import numpy as np
from threading import (Timer)

from std_msgs.msg import (
    Bool,
    Float32,
    String,
)

from kortex_driver.msg import (
    BaseCyclic_Feedback,
    Finger,
    Gripper,
)
from kortex_driver.srv import (SendGripperCommand)
from kinova_positional_control.srv import (
    GripperPosition,
    GripperForceGrasping,
)


class KinovaGripperControl:
    """
    
    """

    def __init__(
        self,
        robot_name,
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name

        # # Private variables:
        self.__activate_force_grasping = False
        self.__force_grasping_stage = 'grasping'
        self.__force_grasping_state = 0
        self.__force_grasping_status = 'empty'

        self.__delay = {
            'timer': None,
            'is_timer_running': False,
            'is_finished': False,
        }

        self.__target_gripper_current = 0.0
        self.__gripper_current_feedback = 0.0
        self.__gripper_position_feedback = 0.0
        self.__gripper_velocity_feedback = 0.0

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.ROBOT_NAME}/gripper_control/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'kortex_driver': False,
        }

        # # Service provider:
        rospy.Service(
            f'/{self.ROBOT_NAME}/gripper_control/position',
            GripperPosition,
            self.__gripper_position_handler,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/gripper_control/force_grasping',
            GripperForceGrasping,
            self.__gripper_force_grasping_handler,
        )

        # # Service subscriber:
        self.__gripper_command = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/send_gripper_command',
            SendGripperCommand,
        )

        # # Topic publisher:
        self.__gripper_position = rospy.Publisher(
            f'/{self.ROBOT_NAME}/gripper_control/position_feedback',
            Float32,
            queue_size=1,
        )
        self.__gripper_velocity = rospy.Publisher(
            f'/{self.ROBOT_NAME}/gripper_control/velocity_feedback',
            Float32,
            queue_size=1,
        )
        self.__gripper_current = rospy.Publisher(
            f'/{self.ROBOT_NAME}/gripper_control/current_feedback',
            Float32,
            queue_size=1,
        )
        self.__force_grasping = rospy.Publisher(
            f'/{self.ROBOT_NAME}/gripper_control/force_grasping_status',
            String,
            queue_size=1,
        )

        # # Topic subscriber:
        self.__kortex_feedback = rospy.Subscriber(
            f'/{self.ROBOT_NAME}/base_feedback',
            BaseCyclic_Feedback,
            self.__kinova_feedback_callback,
        )

    # # Dependency status callbacks:

    # # Service handlers:
    def __gripper_position_handler(self, request):
        """

        """

        self.__activate_force_grasping = False

        gripper_position = request.position

        if gripper_position < 0.0:
            gripper_position = 0.0

        if gripper_position > 1.0:
            gripper_position = 1.0

        self.__gripper_control(
            mode=3,
            value=gripper_position,
        )

        self.__force_grasping_status = 'empty'

        response = True

        return response

    def __gripper_force_grasping_handler(self, request):
        """

        """

        self.__activate_force_grasping = True

        target_gripper_current = np.clip(
            request.target_current,
            a_min=0.1,
            a_max=1.0,
        )

        self.__target_gripper_current = np.interp(
            target_gripper_current,
            [0.1, 1.0],
            [0.04, 0.08],
        ).round(3)

        response = True

        return response

    # # Topic callbacks:
    def __kinova_feedback_callback(self, message):
        """
        
        """

        if not self.__is_initialized:
            self.__dependency_status['kortex_driver'] = True

            rospy.loginfo(
                (
                    f'/{self.ROBOT_NAME}/gripper_control: '
                    'kortex_driver was initialized!'
                ),
            )

        self.__gripper_current_feedback = (
            message.interconnect.oneof_tool_feedback.gripper_feedback[0].
            motor[0].current_motor
        )

        self.__gripper_position_feedback = (
            message.interconnect.oneof_tool_feedback.gripper_feedback[0].
            motor[0].position / 100
        )

        self.__gripper_velocity_feedback = (
            message.interconnect.oneof_tool_feedback.gripper_feedback[0].
            motor[0].velocity / 100
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

        if self.__kortex_feedback.get_num_connections() != 1:
            if self.__dependency_status['kortex_driver']:
                rospy.logerr(
                    (
                        f'/{self.ROBOT_NAME}/gripper_control: '
                        f'lost connection to kortex_driver!'
                    )
                )

                # # Emergency actions on lost connection:
                # NOTE (optionally): Add code, which needs to be executed if
                # connection to any of dependencies was lost.

            self.__dependency_status['kortex_driver'] = False

        if not self.__dependency_status['kortex_driver']:
            self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = ''
            for key in self.__dependency_status:
                if not self.__dependency_status[key]:
                    waiting_for += f'\n- waiting for {key}...'

            rospy.logwarn_throttle(
                15,
                (
                    f'/{self.ROBOT_NAME}/gripper_control:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/gripper_control: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __gripper_control(self, mode, value):
        """

        Gripper control: mode=2 - velocity, mode=3 - position.
        
        """

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Finger.msg
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/Gripper.msg
        gripper = Gripper()
        gripper.finger.append(finger)

        # https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_driver/msg/generated/base/GripperCommand.msg
        gripper_command = SendGripperCommand()
        gripper_command.mode = mode
        gripper_command.duration = 0
        gripper_command.gripper = gripper

        self.__gripper_command(gripper_command)

    def __delay_timer(self, delay):
        """
          
        """

        # No timer was started yet.
        if not self.__delay['is_timer_running']:

            # Cancel any running timmers and start a new one.
            if self.__delay['timer']:
                self.__delay['timer'].cancel()

            self.__delay['timer'] = Timer(
                delay,
                self.__timer_finished,
            )
            self.__delay['timer'].start()
            self.__delay['is_timer_running'] = True
            self.__delay['is_finished'] = False

    def __timer_finished(self):
        """

        """

        self.__delay['is_finished'] = True
        self.__delay['is_timer_running'] = False

    def __force_grasping_state_machine(self):
        """
        
        """

        if not self.__activate_force_grasping:
            self.__force_grasping_state = 0
            self.__force_grasping_stage = 'grasping'
            return

        # State 0: Activate force grasping.
        if (self.__force_grasping_state == 0):

            # Close the gripper using grasping velocity.
            self.__gripper_control(
                mode=2,
                value=-0.08,
            )

            # Timer to skip initial (motion start) current spike.
            self.__delay_timer(delay=0.5)

            self.__force_grasping_state = 1

        # State 1: Force grasping and verification.
        elif (self.__force_grasping_state == 1 and self.__delay['is_finished']):
            if (
                self.__gripper_current_feedback < self.__target_gripper_current
            ):
                if self.__force_grasping_stage == 'verification':
                    # Close the gripper using verification velocity.
                    self.__gripper_control(
                        mode=2,
                        value=-0.02,
                    )

            else:
                # Stop the gripper motion.
                self.__gripper_control(
                    mode=2,
                    value=0.0,
                )

                # Timer before the verification stage.
                self.__delay_timer(delay=0.5)

                if self.__force_grasping_stage == 'grasping':
                    self.__force_grasping_state = 2

                elif self.__force_grasping_stage == 'verification':
                    self.__force_grasping_stage = 'grasping'

                    if self.__gripper_position_feedback < 0.97:
                        self.__force_grasping_status = 'grasped'

                    else:
                        self.__force_grasping_status = 'failed'

                    self.__activate_force_grasping = False
                    self.__force_grasping_state = 0

        # State 2: Waiting for the delay timer to start verification.
        elif (self.__force_grasping_state == 2 and self.__delay['is_finished']):
            self.__force_grasping_stage = 'verification'
            self.__force_grasping_state = 1

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__force_grasping_state_machine()

        self.__gripper_position.publish(self.__gripper_position_feedback)
        self.__gripper_velocity.publish(self.__gripper_velocity_feedback)
        self.__gripper_current.publish(self.__gripper_current_feedback)
        self.__force_grasping.publish(self.__force_grasping_status)

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/gripper_control: node is shutting down...',
        )

        # Stop the gripper motion.
        self.__gripper_control(
            mode=2,
            value=0.0,
        )

        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/gripper_control: node has shut down.',
        )


def main():
    """
    
    """

    rospy.init_node(
        'gripper_control',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    gripper_control = KinovaGripperControl(robot_name=kinova_name)

    rospy.on_shutdown(gripper_control.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        gripper_control.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
