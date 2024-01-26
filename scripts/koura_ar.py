#!/usr/bin/env python
"""

Author(s):

TODO:

"""

# # Standart libraries:
import rospy
import cv2
import numpy as np
from cv_bridge import (
    CvBridge,
    CvBridgeError,
)

# # Third party libraries:

# # Standart messages and services:
from std_msgs.msg import (
    Bool,
    Float32,
    String,
)
from sensor_msgs.msg import (Image)

# # Third party messages and services:


class KouraAR:
    """
    
    """

    def __init__(
        self,
        node_name,
        image_topic,
        enable_imshow,
    ):
        """
        
        """

        # # Private CONSTANTS:
        # NOTE: By default all new class CONSTANTS should be private.
        self.__IMAGE_TOPIC = image_topic
        self.__ENABLE_IMSHOW = enable_imshow
        self.__BRIDGE = CvBridge()

        # # Public CONSTANTS:
        self.NODE_NAME = node_name

        # # Private variables:
        # NOTE: By default all new class variables should be private.
        self.__cv_image = None

        self.__scoop_orientation = 0.0
        self.__scoop_height = 0.0
        self.__linear_missalignment = 0.0
        self.__control_mode = 'polar'

        # # Public variables:
        self.public_variable = 1

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
            'image_topic': False,
        }

        # NOTE: Specify dependency is_initialized topic (or any other topic,
        # which will be available when the dependency node is running properly).
        self.__dependency_status_topics = {}

        self.__dependency_status_topics['image_topic'] = (
            rospy.Subscriber(
                f'{self.__IMAGE_TOPIC}',
                Image,
                self.__camera_callback,
            )
        )

        # # Service provider:

        # # Service subscriber:

        # # Topic publisher:
        self.__augmented_image = rospy.Publisher(
            f'{self.NODE_NAME}/augmented_image',
            Image,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'{self.__IMAGE_TOPIC}',
            Image,
            self.__camera_callback,
        )
        rospy.Subscriber(
            f'/my_gen3/koura_ar/scoop_orientation',
            Float32,
            self.__scoop_orientation_callback,
        )
        rospy.Subscriber(
            f'/my_gen3/koura_ar/scoop_height',
            Float32,
            self.__scoop_height_callback,
        )
        rospy.Subscriber(
            f'/my_gen3/koura_ar/linear_missalignment',
            Float32,
            self.__linear_missalignment_callback,
        )
        rospy.Subscriber(
            f'/my_gen3/koura_ar/control_mode',
            String,
            self.__control_mode_callback,
        )

        # # Timers:
        # rospy.Timer(
        #     rospy.Duration(1.0 / 100),
        #     self.__some_function_timer,
        # )

    # # Dependency status callbacks:
    # NOTE: each dependency topic should have a callback function, which will
    # set __dependency_status variable.
    def __dependency_name_callback(self, message):
        """Monitors <node_name>/is_initialized topic.
        
        """

        # self.__dependency_status['dependency_node_name'] = message.data

    # # Service handlers:

    # # Topic callbacks:
    def __camera_callback(self, message):
        """
        
        """

        try:
            self.__cv_image = self.__BRIDGE.imgmsg_to_cv2(
                message,
                'bgr8',
            )
            # # TODO: Uncomment if the image appears bluish:
            # self.__cv_image = cvtColor(
            #     self.__cv_image,
            #     COLOR_BGR2RGB,
            # )

            if not self.__is_initialized:
                self.__dependency_status['image_topic'] = True

        except CvBridgeError as e:
            print(e)

    def __scoop_orientation_callback(self, message):
        """
        
        """

        self.__scoop_orientation = message.data

    def __scoop_height_callback(self, message):
        """
        
        """

        self.__scoop_height = message.data

    def __linear_missalignment_callback(self, message):
        """
        
        """

        self.__linear_missalignment = message.data

    def __control_mode_callback(self, message):
        """
        
        """

        if message.data == 'polar':
            self.__control_mode = 'Polar'

        elif message.data == 'height':
            self.__control_mode = 'Height'

    # Timer callbacks:

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
                        (f'{self.NODE_NAME}: '
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
                    f'{self.NODE_NAME}:'
                    f'{waiting_for}'
                    # f'\nMake sure those dependencies are running properly!'
                ),
            )

        # NOTE (optionally): Add more initialization criterea if needed.
        if (self.__dependency_initialized):
            if not self.__is_initialized:
                rospy.loginfo(f'\033[92m{self.NODE_NAME}: ready.\033[0m',)

                self.__is_initialized = True

        else:
            if self.__is_initialized:
                # NOTE (optionally): Add code, which needs to be executed if the
                # nodes's status changes from True to False.

                pass

            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    def __display_scoop(self, cv_image):
        """
        
        """

        color = (255, 140, 0)

        if self.__scoop_height < -0.19 and self.__scoop_height > -0.56:
            color = (0, 165, 255)

        elif self.__scoop_height <= -0.56:
            color = (0, 0, 255)

        # Define a rotated rectangle (center, size, angle).
        rotated_rect = (
            (1075, 125),
            (150, 100),
            -self.__scoop_orientation,
        )

        # Get the four corners of the rectangle.
        rect_points = cv2.boxPoints(rotated_rect)
        rect_points = np.int0(rect_points)

        # Draw the scoop:
        cv2.line(
            cv_image,
            tuple(rect_points[0]),
            tuple(rect_points[(0 + 1) % 4]),
            (255, 140, 0),
            5,
        )
        cv2.line(
            cv_image,
            tuple(rect_points[3]),
            tuple(rect_points[(3 + 1) % 4]),
            (255, 140, 0),
            5,
        )

        median_x = (rect_points[1][0] + rect_points[2][0]) // 2
        median_y = (rect_points[1][1] + rect_points[2][1]) // 2
        median_point = (median_x, median_y)

        cv2.line(
            cv_image,
            rect_points[1],
            median_point,
            (255, 140, 0),
            5,
        )
        cv2.line(
            cv_image,
            median_point,
            rect_points[3],
            (255, 140, 0),
            5,
        )

        # Draw the ground level:
        cv2.line(
            cv_image,
            (950, 250),
            (1200, 250),
            color,
            5,
        )
        for i in range(950, 1200, 40):
            cv2.line(
                cv_image,
                (i, 250),
                (i - 20, 270),
                color,
                5,
            )

        return cv_image

    def __display_control_mode(self, cv_image):
        """
        
        """

        color = (255, 140, 0)

        if self.__control_mode == 'Height':
            color = (128, 0, 0)

        cv2.putText(
            cv_image,
            text=f'Mode: {self.__control_mode}',
            org=(75, 75),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1.5,
            color=color,
            thickness=3,
            lineType=cv2.LINE_AA,
        )

        color = (255, 140, 0)

        if (
            self.__linear_missalignment > 0.025
            and self.__linear_missalignment <= 0.075
        ):
            color = (0, 165, 255)

        elif self.__linear_missalignment > 0.075:
            color = (0, 0, 255)

        cv2.putText(
            cv_image,
            text=f'Error: {round(self.__linear_missalignment, 3)}',
            org=(75, 125),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1,
            color=color,
            thickness=2,
            lineType=cv2.LINE_AA,
        )

        return cv_image

    def __display_visual_cues(self):
        """
        
        """

        if self.__cv_image is None:
            return

        cv_image = self.__display_scoop(self.__cv_image)

        cv_image = self.__display_control_mode(cv_image)

        # Optionally show the frame.
        if self.__ENABLE_IMSHOW:
            cv2.imshow(
                'self.__cv_image',
                cv_image,
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):
                pass

        return cv_image

    def __publish_augumented_image(self, cv_image):
        """
        
        """

        image_message = self.__BRIDGE.cv2_to_imgmsg(cv_image, 'bgr8')

        self.__augmented_image.publish(image_message)

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
        cv_image = self.__display_visual_cues()

        self.__publish_augumented_image(cv_image)

    def node_shutdown(self):
        """
        
        """

        rospy.loginfo_once(f'{self.NODE_NAME}: node is shutting down...',)

        # NOTE: Add code, which needs to be executed on nodes' shutdown here.
        # Publishing to topics is not guaranteed, use service calls or
        # set parameters instead.

        rospy.loginfo_once(f'{self.NODE_NAME}: node has shut down.',)


def main():
    """
    
    """

    # # Default node initialization.
    # This name is replaced when a launch file is used.
    rospy.init_node(
        'koura_ar',
        log_level=rospy.INFO,  # rospy.DEBUG to view debug messages.
    )

    rospy.loginfo('\n\n\n\n\n')  # Add whitespaces to separate logs.

    # # ROS launch file parameters:
    node_name = rospy.get_name()

    node_frequency = rospy.get_param(
        param_name=f'{rospy.get_name()}/node_frequency',
        default=1000,
    )
    image_topic = rospy.get_param(
        param_name=f'{rospy.get_name()}/image_topic',
        default='/camera/color/image_raw',
    )
    enable_imshow = rospy.get_param(
        param_name=f'{node_name}/enable_imshow',
        default=False,
    )

    class_instance = KouraAR(
        node_name=node_name,
        image_topic=image_topic,
        enable_imshow=enable_imshow,
    )

    rospy.on_shutdown(class_instance.node_shutdown)
    node_rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        node_rate.sleep()


if __name__ == '__main__':
    main()
