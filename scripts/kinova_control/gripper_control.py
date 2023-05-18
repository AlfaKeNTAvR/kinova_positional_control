#!/usr/bin/env python
"""

"""

import rospy

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
        robot_name='my_gen3',
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name

        # # Private variables:
        self.__gripper_current = 0.0
        self.__activate_force_grasping = False
        self.__target_gripper_current = 0.0

        # # Public variables:

        # # Service provider:
        rospy.Service(
            f'/{self.ROBOT_NAME}/gripper/position',
            GripperPosition,
            self.__gripper_position_handler,
        )
        rospy.Service(
            f'/{self.ROBOT_NAME}/gripper/force_grasping',
            GripperForceGrasping,
            self.__gripper_force_grasping_handler,
        )

        # # Service subscriber:
        self.__gripper_command = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/send_gripper_command',
            SendGripperCommand,
        )

        # # Topic publisher:

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/base_feedback',
            BaseCyclic_Feedback,
            self.__kinova_feedback_callback,
        )

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

        response = True

        return response

    def __gripper_force_grasping_handler(self, request):
        """

        """

        self.__activate_force_grasping = True
        self.__target_gripper_current = request.target_current

        response = True

        return response

    # # Topic callbacks:
    def __kinova_feedback_callback(self, message):
        """
        
        """

        self.__gripper_current = (
            message.interconnect.oneof_tool_feedback.gripper_feedback[0].
            motor[0].current_motor
        )

    # # Private methods:
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

    def __gripper_force_grasping(self):
        """
        
        """

        if self.__activate_force_grasping:

            if self.__target_gripper_current < 0.04:
                self.__target_gripper_current = 0.04

            # Close the gripper until the current raises to a value higher than
            # 0.04, indicating contact with an object.
            if (self.__gripper_current < self.__target_gripper_current):
                # Close the gripper using a velocity command.
                self.__gripper_control(
                    mode=2,
                    value=-0.08,
                )

            else:
                # Stop the gripper motion.
                self.__gripper_control(
                    mode=2,
                    value=0.0,
                )

                self.__activate_force_grasping = False

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__gripper_force_grasping()

    def node_shutdown(self):
        """
        
        """

        print(
            f'\n/{self.ROBOT_NAME}/gripper_control: node is shutting down...\n'
        )

        # Stop the gripper motion.
        self.__gripper_control(
            mode=2,
            value=0.0,
        )

        print(f'\n/{self.ROBOT_NAME}/gripper_control: node has shut down.\n')


def main():
    """
    
    """

    rospy.init_node('gripper_control')

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    gripper_control = KinovaGripperControl(robot_name=kinova_name)

    rospy.on_shutdown(gripper_control.node_shutdown)

    print(f'\n/{kinova_name}/gripper_control: ready.\n')

    while not rospy.is_shutdown():
        gripper_control.main_loop()


if __name__ == '__main__':
    main()