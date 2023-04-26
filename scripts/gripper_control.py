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
        name='my_gen3',
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = name

        # # Private variables:
        self.__gripper_current = 0.0

        # # Public variables:

        # # ROS node:
        rospy.init_node(f'{self.ROBOT_NAME}_gripper_control')
        rospy.on_shutdown(self.__node_shutdown)

        # # Service provider:
        rospy.Service(
            f'{self.ROBOT_NAME}/gripper/position',
            GripperPosition,
            self.__gripper_position_handler,
        )
        rospy.Service(
            f'{self.ROBOT_NAME}/gripper/force_grasping',
            GripperForceGrasping,
            self.__gripper_force_grasping_handler,
        )

        # # Service subscriber:
        self.__gripper_command = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/base/send_gripper_command',
            SendGripperCommand,
        )

        # # Topic publisher:

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.ROBOT_NAME}/base_feedback',
            BaseCyclic_Feedback,
            self.__kinova_feedback_callback,
        )

    # # Service handlers:
    def __gripper_position_handler(self, request):
        """

        """

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

        self.__gripper_force_grasping(request.target_current)

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
    def __node_shutdown(self):
        """
        
        """

        print('\nNode is shutting down...\n')

        # Stop the gripper motion.
        self.__gripper_control(
            mode=2,
            value=0.0,
        )

        print('\nNode is shut down.\n')

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

    def __gripper_force_grasping(self, target_current):
        """
        
        """

        if target_current < 0.04:
            target_current = 0.04

        # Close the gripper until the current raises to a value higher than
        # 0.04, indicating contact with an object.
        while (
            self.__gripper_current < target_current and not rospy.is_shutdown()
        ):
            # Close the gripper using a velocity command.
            self.__gripper_control(
                mode=2,
                value=-0.1,
            )

        # Stop the gripper motion.
        self.__gripper_control(
            mode=2,
            value=0.0,
        )

    # # Public methods:
    def main_loop(self):
        """
        
        """

        pass


def main():
    """
    
    """

    gripper_control = KinovaGripperControl()

    print('\nGripper control is ready.\n')

    while not rospy.is_shutdown():
        gripper_control.main_loop()


if __name__ == '__main__':
    main()