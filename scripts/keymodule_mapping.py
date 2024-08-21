#!/usr/bin/env python
"""Implements Oculus Quest 2 controller to teleoperation mapping module.

TODO: Add detailed description.

Author (s):
    1. Nikita Boguslavskii (bognik3@gmail.com), Human-Inspired Robotics (HiRo)
       lab, Worcester Polytechnic Institute (WPI), 2024.

"""

# # Standart libraries:
import rospy
from threading import (Thread)
from pynput.keyboard import (
    Key,
    Listener,
)

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (Bool)

# # Third party messages and services:


class KeymoduleMapping:
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
        self.__NODE_NAME = node_name
        self.__ROBOT_NAME = robot_name

        # # Public constants:


        # # Private variables:
        self.__alt_pressed = False
        self.__tracking_key_vk = 269025093  # F14
        self.__tracking_key_state = False
        self.__mode_key_vk = 269025094  # F15
        self.__mode_key_state = False

        self.__listener_thread = Thread(target=self.__listen_for_keys)
        self.__listener_thread.start()

        # # Public variables:
        self.is_initialized = True

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'/{self.__ROBOT_NAME}{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__dependency_status = {
            'teleoperation': False,
        }

        self.__dependency_status_topics = {
            'teleoperation':
                rospy.Subscriber(
                    f'/{self.__ROBOT_NAME}/teleoperation/is_initialized',
                    Bool,
                    self.__teleoperation_callback,
                ),
        }

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__node_is_initialized = rospy.Publisher(
            f'/{self.__ROBOT_NAME}{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        self.__teleoperation_tracking_button = rospy.Publisher(
            f'/{self.__ROBOT_NAME}/teleoperation/tracking_button',
            Bool,
            queue_size=1,
        )
        self.__teleoperation_mode_button = rospy.Publisher(
            f'/{self.__ROBOT_NAME}/teleoperation/mode_button',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:

    # # Dependency status callbacks:
    def __teleoperation_callback(self, message):
        """Monitors teleoperation is_initialized topic.

        """

        self.__dependency_status['teleoperation'] = message.data

    # # Service handlers:

    # # Topic callbacks:

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
                            f'/{self.__ROBOT_NAME}{self.__NODE_NAME}: '
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
                    f'/{self.__ROBOT_NAME}{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE: Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(
                    f'\033[92m/{self.__ROBOT_NAME}{self.__NODE_NAME}: ready.\033[0m',
                )

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.
                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __on_press(self, key):
        """

        """

        # Check if any Alt key is pressed
        if key == Key.alt_l or key == Key.alt_r:
            self.__alt_pressed = True

        # Check if the F14 (269025093) key is pressed (using its reported keycode).
        if self.__alt_pressed and hasattr(key, 'vk') and key.vk == self.__tracking_key_vk:
            self.__tracking_key_state = True

        if self.__alt_pressed and hasattr(key, 'vk') and key.vk == self.__mode_key_vk:
            self.__mode_key_state = True

    def __on_release(self, key):
        """

        """

        # if key == Key.alt_l or key == Key.alt_r:
        #     # Reset the state when Alt is released.
        #     self.__alt_pressed = False

        if hasattr(key, 'vk'):
            if key.vk == self.__tracking_key_vk:
                self.__tracking_key_state = False
                self.__alt_pressed = False

            elif key.vk == self.__mode_key_vk:
                self.__mode_key_state = False
                self.__alt_pressed = False

    def __listen_for_keys(self):
        """

        """

        with Listener(
            on_press=self.__on_press,
            on_release=self.__on_release,
        ) as self.__listener:
            self.__listener.join()


    # # Public methods:
    def main_loop(self):
        """

        """

        self.__check_initialization()

        if not self.__is_initialized:
            return

        self.__teleoperation_tracking_button.publish(
            self.__tracking_key_state
        )
        self.__teleoperation_mode_button.publish(
            self.__mode_key_state
        )

    def node_shutdown(self):
        """

        """

        rospy.loginfo_once(
            f'/{self.__ROBOT_NAME}{self.__NODE_NAME}: node is shutting down...',
        )

        self.__listener.stop()

        rospy.loginfo_once(
            f'/{self.__ROBOT_NAME}{self.__NODE_NAME}: node has shut down.',
        )


def main():
    """

    """

    rospy.init_node(
        'keymodule_mapping',
        log_level=rospy.INFO,  # TODO: Make this a launch file parameter.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    oculus_kinova_mapping = KeymoduleMapping(
        node_name=node_name,
        robot_name=kinova_name,
    )

    rospy.on_shutdown(oculus_kinova_mapping.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        oculus_kinova_mapping.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
