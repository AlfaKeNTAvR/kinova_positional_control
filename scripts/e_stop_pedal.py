#!/usr/bin/env python
"""

Author(s):

TODO:

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
from std_srvs.srv import (Empty)

# # Third party messages and services:


class EStopPedal:
    """
    
    """

    def __init__(
        self,
        node_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__alt_pressed = False
        self.__e_stop_initialized = False
        self.__e_stop_reset = True
        self.__e_stop_state = False

        self.__listener_thread = Thread(target=self.__listen_for_keys)
        self.__listener_thread.start()

        # # Public variables:

        # # Initialization and dependency status topics:
        self.__is_initialized = False
        self.__dependency_initialized = False

        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized',
            Bool,
            queue_size=1,
        )

        # NOTE: Specify dependency initial False initial status.
        self.__dependency_status = {}

        # self.__dependency_status['<dependency_node_name>'] = False

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        # self.__dependency_status_topics['<dependency_node_name>'] = (
        #     rospy.Subscriber(
        #         f'/<dependency_node_name>/is_initialized',
        #         Bool,
        #         self.__<dependency_name>_callback,
        #     )
        # )

        # # Service provider:
        rospy.Service(
            f'{self.__NODE_NAME}/activate_e_stop',
            Empty,
            self.__activate_e_stop_handler,
        )

        # # Service subscriber:

        # # Topic publisher:
        self.__e_stop = rospy.Publisher(
            f'{self.__NODE_NAME}/e_stop_state',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:

        # # Timers:
        rospy.Timer(
            rospy.Duration(1.0 / 10),
            self.__check_initialization_timer,
        )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.

    # # Service handlers:
    def __activate_e_stop_handler(self, request):
        """

        """

        if self.__is_initialized:
            self.__e_stop_state = True
            self.__e_stop_reset = False

        return []

    # # Topic callbacks:

    # # Timer callbacks:
    def __check_initialization_timer(self, event):
        """Calls <some_function> on each timer callback with 100 Hz frequency.
        
        """

        self.__check_initialization()

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
                        (f'{self.__NODE_NAME}: '
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
                    f'{self.__NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        if not self.__e_stop_initialized:
            rospy.logwarn_throttle(
                15,
                (
                    f'{self.__NODE_NAME}: '
                    f'Initialize the E-Stop by pressing the pedal.'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized and self.__e_stop_initialized):
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

    def __on_press(self, key):
        """

        """

        # Check if any Alt key is pressed
        if key == Key.alt_l or key == Key.alt_r:
            self.__alt_pressed = True

        # Check if the F13 key is pressed (using its reported keycode).
        if self.__alt_pressed and hasattr(key, 'vk') and key.vk == 269025153:
            if not self.__is_initialized:
                self.__e_stop_initialized = True

            else:
                self.__e_stop_state = not self.__e_stop_state

                if not self.__e_stop_state:
                    self.__e_stop_reset = False

    def __on_release(self, key):
        """

        """

        if key == Key.alt_l or key == Key.alt_r:
            # Reset the state when Alt is released.
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
    # NOTE: By default all new class methods should be private.
    def main_loop(self):
        """
        
        """

        if not self.__is_initialized:
            return

        # NOTE: Add code (function calls), which has to be executed once the
        # node was successfully initialized.
        if self.__e_stop_state:
            rospy.logwarn_throttle(
                5,
                (
                    f'{self.__NODE_NAME}: '
                    f'In E-Stop state. Reset by pressing the pedal.'
                ),
            )

        if not self.__e_stop_state and not self.__e_stop_reset:
            rospy.loginfo((f'{self.__NODE_NAME}: '
                           f'The E-Stop was reset.'))

            self.__e_stop_reset = True

        self.__e_stop.publish(Bool(self.__e_stop_state))

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        self.__listener.stop()

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'e_stop_pedal',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=100,
    )

    class_instance = EStopPedal(node_name=node_name,)

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
