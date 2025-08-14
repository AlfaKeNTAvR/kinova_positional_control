#!/usr/bin/env python
"""

Author(s):

TODO:

"""

# # Standart libraries:
import rospy
import tf2_ros
import tf2_geometry_msgs  # Is required for tf2_ros.Buffer.transform().
from collections import deque
import numpy as np

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (Bool)
from std_srvs.srv import (SetBool)
from geometry_msgs.msg import (
    WrenchStamped,
    Vector3Stamped,
    Vector3,
)

# # Third party messages and services:
from kortex_driver.msg import (BaseCyclic_Feedback)


class KinovaWrench:
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

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__wrench_input = WrenchStamped()
        self.__is_tracking = False

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

        self.__dependency_status['kortex_driver'] = False

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {
            'kortex_driver':
                rospy.Subscriber(
                    f'/{self.__ROBOT_NAME}/base_feedback',
                    BaseCyclic_Feedback,
                    self.__kinova_feedback_callback,
                ),
        }

        # # Service provider:
        # rospy.Service(
        #     f'{self.__NODE_NAME}/<service_name1>',
        #     SetBool,
        #     self.__service_name1_handler,
        # )

        # # Service subscriber:
        # self.__service = rospy.ServiceProxy(
        #     '/<service_name2>',
        #     ServiceType2,
        # )

        # # Topic publisher:
        self.__transformed_wrench = rospy.Publisher(
            f'{self.__NODE_NAME}/wrench',
            WrenchStamped,
            queue_size=1,
        )

        self.__falcon_force = rospy.Publisher(
            f'/falconForce',
            Vector3,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.__ROBOT_NAME}/base_feedback',
            BaseCyclic_Feedback,
            self.__kinova_feedback_callback,
        )
        rospy.Subscriber(
            f'/{self.__ROBOT_NAME}/teleoperation/is_tracking',
            Bool,
            self.__teleoperation_is_tracking_callback,
        )

        # # Timers:
        # rospy.Timer(
        #     rospy.Duration(1.0 / 100),
        #     self.__some_function_timer,
        # )

        # # TF broadcaster:

        # # TF listener:
        self.__tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        tf2_ros.TransformListener(self.__tf_buffer)

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    # def __dependency_name_callback(self, message):
    #     """Monitors /<node_name>/is_initialized topic.

    #     """

    #     # self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:
    # def __service_name1_handler(self, request):
    #     """

    #     """

    #     success = True
    #     message = ''

    #     return success, message

    # # Topic callbacks:
    def __kinova_feedback_callback(self, message: BaseCyclic_Feedback):
        """

        """

        if not self.__dependency_status['kortex_driver']:
            self.__dependency_status['kortex_driver'] = True

        self.__wrench_input.header.frame_id = f'{self.__ROBOT_NAME}/base_link'

        self.__wrench_input.wrench.force.x = message.base.tool_external_wrench_force_x
        self.__wrench_input.wrench.force.y = message.base.tool_external_wrench_force_y
        self.__wrench_input.wrench.force.z = message.base.tool_external_wrench_force_z

        self.__wrench_input.wrench.torque.x = (
            message.base.tool_external_wrench_torque_x
        )
        self.__wrench_input.wrench.torque.y = (
            message.base.tool_external_wrench_torque_y
        )
        self.__wrench_input.wrench.torque.z = (
            message.base.tool_external_wrench_torque_z
        )

    def __teleoperation_is_tracking_callback(self, message: Bool):
        """
        
        """

        self.__is_tracking = message.data

    # # Timer callbacks:
    # def __some_function_timer(self, event):
    #     """Calls <some_function> on each timer callback with 100 Hz frequency.

    #     """

    #     self.__some_function()

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

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
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

    def __publish_transformed_wrench(
        self,
        wrench_stamped: WrenchStamped,
        target_frame: str,
    ):
        """
        Transforms a WrenchStamped message into the target frame using TF2.

        Args:
            wrench_stamped (geometry_msgs.msg.WrenchStamped): The original wrench message.
            target_frame (str): The frame to transform the wrench into.

        Returns:
            geometry_msgs.msg.WrenchStamped: The transformed wrench in the target frame,
            or None if the transform fails.
        """

        if not self.__is_tracking:
            vector3_message = Vector3()
            self.__falcon_force.publish(vector3_message)

            return

        try:
            # Lookup the transform from wrench frame to target frame
            transform = self.__tf_buffer.lookup_transform(
                target_frame,
                wrench_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )

            # Transform force
            force_in = Vector3Stamped()
            force_in.header = wrench_stamped.header
            force_in.vector = wrench_stamped.wrench.force
            force_out = tf2_geometry_msgs.do_transform_vector3(
                force_in,
                transform,
            )

            # Transform torque
            torque_in = Vector3Stamped()
            torque_in.header = wrench_stamped.header
            torque_in.header.stamp = rospy.Time.now()
            torque_in.vector = wrench_stamped.wrench.torque
            torque_out = tf2_geometry_msgs.do_transform_vector3(
                torque_in,
                transform,
            )

            # Assemble new WrenchStamped message in target frame
            transformed_wrench = WrenchStamped()
            transformed_wrench.header.stamp = wrench_stamped.header.stamp
            transformed_wrench.header.frame_id = target_frame
            transformed_wrench.wrench.force = force_out.vector
            transformed_wrench.wrench.torque = torque_out.vector

            self.__transformed_wrench.publish(transformed_wrench)

            # Convert force to numpy array
            force_vector = np.array(
                [
                    force_out.vector.x,
                    force_out.vector.y,
                    force_out.vector.z,
                ]
            )

            vector3_message = Vector3()
            vector3_message.x = -force_vector[1] * 0.05
            vector3_message.y = force_vector[2] * 0.1
            vector3_message.z = -force_vector[0] * 0.05

            rospy.loginfo_throttle(0.5, f'Force Scaled: {vector3_message.y}')

            self.__falcon_force.publish(vector3_message)

            # # Initialize buffers on first run
            # if not hasattr(self, '_force_buffer_1000'):
            #     self._force_buffer_1000 = deque(maxlen=100)
            #     self._force_buffer_100 = deque(maxlen=10)
            #     self._last_force_vector = force_vector  # Initialize last seen force

            # # Only update long-term buffer on rising edge
            # if np.all(force_vector >= self._last_force_vector):
            #     self._force_buffer_1000.append(force_vector)

            # # Always update short-term buffer (for trend)
            # self._force_buffer_100.append(force_vector)

            # # Update last seen vector
            # self._last_force_vector = force_vector

            # # Compute means
            # avg_1000 = np.mean(self._force_buffer_1000, axis=0
            #                   ) if self._force_buffer_1000 else np.zeros(3)
            # avg_100 = np.mean(self._force_buffer_100, axis=0
            #                  ) if self._force_buffer_100 else np.zeros(3)

            # # Compensate force
            # compensated_force = avg_1000 - avg_100

            # vector3_message = Vector3()
            # vector3_message.x = -compensated_force[1] * -0.25
            # vector3_message.y = compensated_force[2] * -0.25
            # vector3_message.z = -compensated_force[0] * -0.25

            # self.__falcon_force.publish(vector3_message)

        except Exception as ex:
            rospy.logerr_throttle(
                1, f'/{self.__ROBOT_NAME}/kinova_wrench:'
                f'\nError in __publish_transformed_wrench: {ex}'
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
        self.__publish_transformed_wrench(self.__wrench_input, 'base_link')

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'kinova_wrench',
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

    class_instance = KinovaWrench(
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
