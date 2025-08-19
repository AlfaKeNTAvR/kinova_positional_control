#!/usr/bin/env python
"""

Author(s):

TODO:

"""

# # Standart libraries:
import rospy
import tf2_ros
import math
import time

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    String,
    Float32,
)
from geometry_msgs.msg import (Vector3)
from std_srvs.srv import (
    SetBool,
    Empty,
)

# # Third party messages and services:
from kinova_positional_control.srv import (
    SetString,
    GripperPosition,
)
from gopher_ros_clearcore.srv import (MovePosition)


class ClassName:
    """
    
    """

    def __init__(
        self,
        node_name,
        robot_name,
        mode_name,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__NODE_NAME = node_name
        self.__ROBOT_NAME = robot_name
        self.__MODE_NAME = mode_name  # 'docking', 'fixed', 'dynamic', 'clutching'

        # # Public CONSTANTS:

        # # Private variables:
        # NOTE: By default all new class variables should be private.

        self.__activation_radius = 0.05  # meters

        self.__state = 0
        self.__cleaning_state = 0
        self.__state_name = 'Idle'
        self.__gripper_opened = False
        self.__trajectory_finished = False
        self.__motion_finished = True

        self.__in_constrain_zone = {
            'right_socket': False,
            'left_socket': False,
        }
        self.__ee_distance_to_frame = {
            'right_socket': 0.0,
            'left_socket': 0.0,
            'right_socket_zone': 0.0,
            'left_socket_zone': 0.0,
        }
        self.__is_latched = False
        self.__is_tracking = False
        self.__falcon_pos = [0.0, 0.0, 0.0]

        self.__torso_position = 0.44
        self.__torso_velocity = 0.0
        self.__task_end = False
        self.__enable_constraint = False
        self.__autonomous = False

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
        # rospy.Service(
        #     f'{self.__NODE_NAME}/<service_name1>',
        #     SetBool,
        #     self.__service_name1_handler,
        # )

        # # Service subscriber:
        self.__enable_tracking = rospy.ServiceProxy(
            f'/{self.__ROBOT_NAME}/teleoperation/enable_tracking',
            SetBool,
        )
        self.__allow_tracking = rospy.ServiceProxy(
            f'/{self.__ROBOT_NAME}/teleoperation/allow_tracking',
            SetBool,
        )
        self.__enable_fixed_orientation = rospy.ServiceProxy(
            f'/{self.__ROBOT_NAME}/teleoperation/enable_fixed_orientation_mode',
            SetBool,
        )

        self.__haptic_unlatch = rospy.ServiceProxy(
            f'/haptic/unlatch',
            Empty,
        )

        self.__use_right_socket = rospy.ServiceProxy(
            f'/line_follower/use_right_socket',
            SetBool,
        )
        self.__start_point = rospy.ServiceProxy(
            f'/line_follower/start_point',
            Empty,
        )
        self.__start_segment = rospy.ServiceProxy(
            f'/line_follower/start_segment',
            Empty,
        )
        self.__retract_point = rospy.ServiceProxy(
            f'/line_follower/retract_point',
            Empty,
        )

        self.__reset_polyline = rospy.ServiceProxy(
            f'/line_follower/reset',
            Empty,
        )
        self.__disable_polyline = rospy.ServiceProxy(
            f'/line_follower/disable',
            Empty,
        )

        self.__use_falcon_pose = rospy.ServiceProxy(
            f'/{self.__ROBOT_NAME}/oculus_mapping/use_falcon_pose',
            SetBool,
        )
        self.__move_to_frame = rospy.ServiceProxy(
            f'/{self.__ROBOT_NAME}/oculus_mapping/move_to_frame',
            SetString,
        )

        self.__move_torso = rospy.ServiceProxy(
            f'/chest_control/move_absolute_position',
            MovePosition,
        )

        # # Topic publisher:
        self.__state_pub = rospy.Publisher(
            f'/task/state',
            String,
            queue_size=1,
        )
        self.__constraint_pub = rospy.Publisher(
            f'/task/constraint',
            Bool,
            queue_size=1,
        )

        self.__autonomous_execution = rospy.Publisher(
            f'/autonomous_execution',
            Bool,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/haptic/latched',
            Bool,
            self.__haptic_latched_callback,
        )
        rospy.Subscriber(
            f'/{self.__ROBOT_NAME}/teleoperation/is_tracking',
            Bool,
            self.__is_tracking_callback,
        )
        rospy.Subscriber(
            f'/right_arm/gripper_control/position_feedback',
            Float32,
            self.__gripper_position_callback,
        )
        rospy.Subscriber(
            f'/{self.__ROBOT_NAME}/preset_poses/trajectory_finished',
            Bool,
            self.__trajectory_finished_callback,
        )
        rospy.Subscriber(
            f'/falconPos_converted',
            Vector3,
            self.__falcon_pos_callback,
        )
        rospy.Subscriber(
            f'/chest_logger/current_position',
            Float32,
            self.__torso_position_callback,
        )
        rospy.Subscriber(
            f'/chest_logger/current_velocity',
            Float32,
            self.__torso_velocity_callback,
        )
        rospy.Subscriber(
            f'/task/end',
            Bool,
            self.__task_end_callback,
        )
        rospy.Subscriber(
            f'/{self.__ROBOT_NAME}/joints_control/motion_finished',
            Bool,
            self.__motion_finished_callback,
        )

        # # Timers:
        rospy.Timer(
            rospy.Duration(1.0 / 10),
            self.__publisher_timer,
        )

        # # TF listener:
        self.__tf_buffer = tf2_ros.Buffer(rospy.Duration(1))
        tf2_ros.TransformListener(self.__tf_buffer)

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors /<node_name>/is_initialized topic.
        
        """

        # self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:
    # def __service_name1_handler(self, request):
    #     """

    #     """

    #     success = True
    #     message = ''

    #     return success, message

    # # Topic callbacks:
    def __haptic_latched_callback(self, message):
        """

        """

        self.__is_latched = message.data

    def __is_tracking_callback(self, message):
        """

        """

        self.__is_tracking = message.data

    def __gripper_position_callback(self, message):
        """

        """

        self.__gripper_opened = message.data < 0.5

    def __trajectory_finished_callback(self, message: Bool):
        """

        """

        self.__trajectory_finished = message.data

    def __falcon_pos_callback(self, message: Vector3):
        """
        
        """

        self.__falcon_pos[0] = message.x
        self.__falcon_pos[1] = message.y
        self.__falcon_pos[2] = message.z

    def __torso_position_callback(self, message: Float32):
        """
        
        """

        self.__torso_position = message.data

    def __torso_velocity_callback(self, message: Float32):
        """
        
        """

        self.__torso_velocity = message.data

    def __task_end_callback(self, message):
        """
        
        """

        self.__task_end = message.data

    def __motion_finished_callback(self, message):
        """
        
        """

        self.__motion_finished = message.data

    # # Timer callbacks:
    def __publisher_timer(self, event):
        """Calls <some_function> on each timer callback with 100 Hz frequency.
        
        """

        self.__state_pub.publish(self.__state_name)
        self.__constraint_pub.publish(self.__enable_constraint)
        self.__autonomous_execution.publish(self.__autonomous)

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

    def __check_in_constrain_zone(self, socket):
        """
        
        """
        try:
            transform1 = self.__tf_buffer.lookup_transform(
                f'{self.__ROBOT_NAME}/tool_frame',
                f'{socket}_zone',
                rospy.Time(),
            )
            transform2 = self.__tf_buffer.lookup_transform(
                f'{self.__ROBOT_NAME}/tool_frame',
                f'{socket}',
                rospy.Time(),
            )

            # Extract translation
            t1 = transform1.transform.translation
            t2 = transform2.transform.translation

            # Compute vector length (Euclidean norm)
            dist1 = math.sqrt(t1.x**2 + t1.y**2 + t1.z**2)
            dist2 = math.sqrt(t2.x**2 + t2.y**2 + t2.z**2)

            self.__ee_distance_to_frame[f'{socket}_zone'] = dist1
            self.__ee_distance_to_frame[socket] = dist2

            # rospy.loginfo_throttle(1, f'{socket}: {dist1:.3f}, {dist2:.3f}')

            # Enters the zone:
            if dist1 <= self.__activation_radius or dist2 <= self.__activation_radius * 2:
                if not self.__in_constrain_zone[socket]:
                    rospy.loginfo(
                        f'{self.__NODE_NAME}: entered {socket} constraint zone.',
                    )
                    self.__in_constrain_zone[socket] = True
                return

            # Leaves the zone:
            if self.__in_constrain_zone[socket]:
                rospy.loginfo(
                    f'{self.__NODE_NAME}: left {socket} constraint zone.',
                )

                self.__in_constrain_zone[socket] = False

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass

    def __constraint_state_machine(self):
        """
        
        """

        if (self.__state == 0):
            # Prevent tracking:
            self.__allow_tracking(False)

            self.__state_name = 'Idle'

            # Autonomous AR:
            self.__autonomous = True

            # Raise the torso:
            self.__move_torso(0.44, 1.0)

            if (
                self.__torso_position < 0.43
                or abs(self.__torso_velocity) > 0.01
            ):
                return

            rospy.sleep(2.0)

            # Send the robot to the horizontal:
            self.__move_to_frame('start_point')
            rospy.sleep(0.1)

            self.__state = 1
            return

        elif (
            self.__state == 1 and self.__trajectory_finished
            and self.__motion_finished
        ):
            # Autonomous AR:
            self.__autonomous = False

            # Enable fixed orientation:
            rospy.set_param(
                f'/{self.__ROBOT_NAME}/teleoperation/fixed_mode',
                'fixed_front',
            )
            self.__enable_fixed_orientation(True)

            if self.__MODE_NAME in ['fixed', 'dynamic']:
                self.__use_falcon(True, 8)

            elif self.__MODE_NAME in ['docking']:
                self.__use_falcon(False, 1)

            # Allow tracking:
            self.__allow_tracking(True)

            self.__target_socket = 'left_socket'

            self.__state = 2
            return

        elif (self.__state == 2 and self.__is_tracking):
            self.__state_name = 'Gross'
            self.__state = 3
            return

        elif (
            self.__state == 3 and
            self.__ee_distance_to_frame[f'{self.__target_socket}_zone'] <= 0.05
        ):
            # Prevent tracking:
            self.__allow_tracking(False)

            # Disable tracking:
            self.__enable_tracking(False)

            # Switch input to Falcon position:
            if self.__MODE_NAME in ['dynamic', 'docking']:
                self.__use_falcon(True, 1)

            elif self.__MODE_NAME in ['fixed']:
                self.__use_falcon(True, 8)

            # Send Falcon to the current position:
            if self.__target_socket == 'left_socket':
                self.__use_right_socket(False)

            else:
                self.__use_right_socket(True)

            self.__start_point()

            # Constraint AR:
            self.__enable_constraint = True

            if self.__MODE_NAME == 'docking':
                self.__state_name = 'Docking'
                self.__state = 4
                return
            else:
                self.__state_name = (
                    'Unplug'
                    if self.__target_socket == 'left_socket' else 'Plug'
                )
                rospy.sleep(0.5)

                # Allow tracking:
                self.__allow_tracking(True)

            self.__state = 5
            return

        elif (self.__state == 4 and self.__is_latched):
            self.__state_name = (
                'Unplug' if self.__target_socket == 'left_socket' else 'Plug'
            )

            # Allow tracking:
            self.__allow_tracking(True)

            self.__state = 5
            return

        elif (self.__state == 5 and self.__is_tracking):
            # Force tracking:
            self.__enable_tracking(True)

            # Start following the line:
            self.__start_segment()

            self.__retract_distance = 0.04

            if self.__MODE_NAME in ['docking']:
                self.__retract_distance = 0.03

            self.__state = 6
            return

        elif (
            self.__state == 6
            and self.__ee_distance_to_frame[f'{self.__target_socket}']
            >= self.__retract_distance
        ):
            if (
                self.__target_socket == 'left_socket' and self.__gripper_opened
            ):
                return

            elif (
                self.__target_socket == 'right_socket'
                and not self.__gripper_opened
            ):
                return

            # Constraint AR:
            self.__enable_constraint = False

            # Stop tracking:
            self.__enable_tracking(False)

            if self.__MODE_NAME == 'docking':
                # Unlatch:
                self.__haptic_unlatch()
                rospy.sleep(0.5)

                # Reset line follower:
                self.__retract_point()
                rospy.sleep(0.25)
                self.__reset_polyline()

                # Switch input back to Oculus position:
                self.__use_falcon_pose(False)
                rospy.sleep(1.0)

            if self.__target_socket == 'left_socket':
                self.__target_socket = 'right_socket'

                if self.__MODE_NAME in ['fixed', 'dynamic']:
                    self.__use_falcon(True, 8)

                # Allow tracking:
                self.__allow_tracking(True)

                self.__state_name = 'Gross'
                self.__state = 3

                return

            self.__state_name = 'Idle'
            self.__state = 7

        elif (self.__state == 7):
            self.__cleaning_state_machine()
            return

    def __use_falcon(self, enable, falcon_scale):
        """
        
        """
        rospy.set_param(
            f'/{self.__ROBOT_NAME}/oculus_mapping/falcon_scale',
            falcon_scale,
        )
        self.__use_falcon_pose(enable)

    def __cleaning_state_machine(self):
        """
        
        """

        if (self.__cleaning_state == 0):
            # Autonomous AR:
            self.__autonomous = True

            # Enable fixed top orientation:
            rospy.set_param(
                f'/{self.__ROBOT_NAME}/teleoperation/fixed_mode',
                'fixed_top',
            )
            self.__enable_fixed_orientation(True)

            # Move the arm to the vertical position:
            before = time.time()
            self.__move_to_frame('start_point2')
            after = time.time()

            if after - before > 1:
                self.__cleaning_state = 1
                rospy.sleep(1)

            return

        elif (
            self.__cleaning_state == 1 and self.__trajectory_finished
            and self.__motion_finished
        ):
            # Lower the torso:
            self.__move_torso(0.0, 1.0)

            self.__cleaning_state = 2
            return

        elif (
            self.__cleaning_state == 2 and self.__torso_position < 0.01
            and abs(self.__torso_velocity) < 0.01
        ):
            # Autonomous AR:
            self.__autonomous = False

            # Allow tracking:
            self.__allow_tracking(True)

            if self.__MODE_NAME == 'dynamic':
                self.__use_falcon(True, 4)

            self.__state_name = 'Cleaning'

            self.__cleaning_state = 3
            return

        elif (self.__cleaning_state == 3 and self.__task_end):
            self.__cleaning_state = 4
            return

        elif (self.__cleaning_state == 4):
            self.__state_name = 'Idle'
            return

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
        self.__check_in_constrain_zone('right_socket')
        self.__check_in_constrain_zone('left_socket')

        self.__constraint_state_machine()

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        # NOTE: Placing a service call inside of a try-except block here causes
        # the node to stuck.
        self.__enable_tracking(False)
        self.__enable_fixed_orientation(False)
        self.__use_falcon_pose(False)
        self.__disable_polyline()

        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'contrained_motion_node',
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
        default='right_arm',
    )
    mode_name = rospy.get_param(
        param_name=f'{node_name}/mode_name',
        default='docking',
    )

    class_instance = ClassName(
        node_name=node_name,
        robot_name=robot_name,
        mode_name=mode_name,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
