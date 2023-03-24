#!/usr/bin/env python

import rospy
import roslaunch
import numpy as np

from utils import *
from kortex_feedback import Kinova
from oculus_feedback import Oculus
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *
from kortex_driver.msg import *
from kortex_driver.srv import *
import kinova_positional_control.srv as posctrl_srv
from relaxed_ik_ros1.srv import activate_ik
import gopher_ros_clearcore.srv as clearcore_srv


class Kinova_Oculus_control():

    def __init__(self):

        # Initialize the node
        rospy.init_node(
            "coordinate_converter",
            anonymous=True,
        )
        rospy.on_shutdown(self.node_shutdown)

        # Variables
        self.isInitialized = False

        self.motionFinished = False
        self.initButton_state = False
        self.estopButton_state = False
        self.clearfaultButton_state = False

        self.onTrackingStart = True
        self.isTracking_state = 0
        self.isTracking = False

        self.gripperState = "open"
        self.gripperButtonState = False
        self.gripperButtonReleased = True

        self.target_pos = np.array([0.0, -0.1, 0.5])

        # Launch files
        self.roslaunch_pid()
        self.roslaunch_relaxed_ik()
        rospy.sleep(2)

        # Publishers
        self.angles_pub = rospy.Publisher(
            '/relaxed_ik/joint_angle_solutions',
            JointAngles,
            queue_size=10,
        )
        self.relaxed_ik_target_wcs_pub = rospy.Publisher(
            '/relaxed_ik/position_wcs',
            Float32MultiArray,
            queue_size=1,
        )
        self.relaxed_ik_target_gcs_pub = rospy.Publisher(
            '/relaxed_ik/position_gcs',
            Float32MultiArray,
            queue_size=1,
        )

        # Subscribers
        rospy.Subscriber(
            '/pid/motion_finished',
            Bool,
            self.pid_motion_finished_callback,
        )
        rospy.Subscriber(
            '/right_grip_button',
            Bool,
            self.right_grip_button_callback,
        )
        rospy.Subscriber(
            '/right_trigger_button',
            Bool,
            self.right_trigger_button_callback,
        )
        rospy.Subscriber(
            '/right_joystick_pos_y',
            Float64,
            self.right_joystick_pos_y_callback,
        )
        rospy.Subscriber(
            '/left_grip_button',
            Bool,
            self.left_grip_button_callback,
        )
        rospy.Subscriber(
            '/left_primary_button',
            Bool,
            self.left_primary_button_callback,
        )
        rospy.Subscriber(
            '/left_secondary_button',
            Bool,
            self.left_secondary_button_callback,
        )
        rospy.Subscriber(
            '/input_position_gcs', Float32MultiArray,
            self.input_position_gcs_callback
        )

        # EDIT! MOCAP
        rospy.Subscriber(
            '/mocap_ee_pose_simple', geom_msgs.Point,
            self.mocap_position_callback
        )

        # Service
        self.pid_setpoint_srv = rospy.ServiceProxy(
            'pid_setpoint', posctrl_srv.pid_setpoint
        )
        self.pid_vel_limit_srv = rospy.ServiceProxy(
            'pid_vel_limit', posctrl_srv.pid_vel_limit
        )

        self.stop_arm_srv = rospy.ServiceProxy(
            'my_gen3/base/stop',
            Stop,
        )
        self.resume_data_srv = rospy.ServiceProxy(
            '/data_collector/resume_data',
            posctrl_srv.resume_record,
        )
        self.estop_arm_srv = rospy.ServiceProxy(
            'my_gen3/base/apply_emergency_stop',
            ApplyEmergencyStop,
        )

        self.activate_ik_srv = rospy.ServiceProxy(
            'relaxed_ik/activate_ik',
            activate_ik,
        )
        self.clearfaults_arm_srv = rospy.ServiceProxy(
            'my_gen3/base/clear_faults',
            Base_ClearFaults,
        )
        self.chest_homing_srv = rospy.ServiceProxy(
            'z_chest_home',
            clearcore_srv.Homing,
        )
        self.chest_stop_srv = rospy.ServiceProxy(
            'z_chest_stop',
            clearcore_srv.Stop,
        )
        self.chest_abspos_srv = rospy.ServiceProxy(
            'z_chest_abspos',
            clearcore_srv.AbsolutePosition,
        )
        self.chest_logger_srv = rospy.ServiceProxy(
            'z_chest_logger',
            clearcore_srv.LoggerControl,
        )

    def right_grip_button_callback(self, msg):

        self.right_grip_button = msg.data

    def right_trigger_button_callback(self, msg):

        self.right_trigger_button = msg.data

    def right_joystick_pos_y_callback(self, msg):

        self.right_joystick_pos_y = msg.data

    def left_grip_button_callback(self, msg):

        self.left_grip_button = msg.data

    def left_primary_button_callback(self, msg):

        self.left_primary_button = msg.data

    def left_secondary_button_callback(self, msg):

        self.left_secondary_button = msg.data

    def input_position_gcs_callback(self, msg):

        self.input_pos_gcs = msg.data

    # EDIT! MOCAP
    def mocap_position_callback(self, data):

        self.target_mocap = [
            data.x,
            data.y,
            data.z,
        ]

    def pid_motion_finished_callback(self, data):
        """Callback function for the '/pid/motion_finished' topic that updates 
        the 'motionFinished' flag variable.
        """

        self.motionFinished = data.data

    def wait_motion_finished(self):
        """Blocks code execution until the 'motionFinished' flag is set or the ROS node
        is shutdown.
        """

        # Allow motion to start
        rospy.sleep(1)

        # Block code execution
        while not self.motionFinished and not rospy.is_shutdown():
            pass

    def roslaunch_pid(self):
        """Launches the 'arm_controller.launch' file from the 'kinova_pid' package. 
        This function initializes the Kinova PID controller node.
        """

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.pid_launch = roslaunch.parent.ROSLaunchParent(
            uuid,
            [
                '/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/kinova_pid/launch/arm_controller.launch'
            ],
        )
        self.pid_launch.start()

    def roslaunch_relaxed_ik(self):
        """Launches the 'relaxed_ik.launch' file from the 'relaxed_ik_ros1' package. 
        This function initializes the RelaxedIK node.
        """

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.relaxed_ik_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [
                '/home/fetch/catkin_workspaces/oculus_relaxedik_ws/src/relaxed_ik_ros1/launch/relaxed_ik.launch'
            ]
        )

        self.relaxed_ik_launch.start()

    def node_shutdown(self):
        """This function is called when the node is shutting down. It stops the arm motion and shuts down the PID and 
        relaxed IK nodes.
        """

        print("\nNode is shutting down...")

        # Stop arm motion by calling the stop_arm_srv() service
        self.stop_arm_srv()

        # Stop chest motion
        self.chest_stop_srv()

        # Deactivate chest feedback
        self.chest_logger_srv(False)

        # Shutdown PID and relaxed IK nodes
        self.pid_launch.shutdown()
        self.relaxed_ik_launch.shutdown()

        # Pause datarecoding
        try:
            self.resume_data_srv(False)

        except Exception as e:
            print(e)

        print("\nNode has shut down.")

    def initialization(self):
        """Initializes the system by homing the robotic arm, with velocity limit
        set to 20%, to a pre-specified initial configuration (starting_config in
        kortex_info.yaml) with relaxedIK.
        """

        self.target_pos = np.array([0.0, -0.1, 0.5])

        # Clear arm faults
        self.clearfaults_arm_srv()

        # Home the chest
        self.chest_homing_srv(True)

        # Set 20% velocity
        self.pid_vel_limit_srv(0.2)

        # Homing position
        print("\nHoming has started...\n")

        # Let the node initialize
        rospy.sleep(2)

        # Home using relaxedIK
        relaxedik_publish(
            [0, 0, 0],
            [1, 0, 0, 0],
        )

        # Block until the motion is finished
        self.wait_motion_finished()

        print("\nHoming has finished.\n")

        print("\nMoving to the starting position...\n")

        # Move the chest to middle position
        self.chest_abspos_srv(220, 0.6)

        # Move Kinova to the middle position
        relaxedik_publish(
            global_to_kinova([0, -0.1, 0.5]),
            [1, 0, 0, 0],
        )

        # Block until the motion is finished
        self.wait_motion_finished()

        # Set 100% velocity
        self.pid_vel_limit_srv(1.0)

        # Set the flag and finish initialization
        self.isInitialized = True

        print("\nSystem is ready.\n")

    def tracking_sm(self):
        """State machine for tracking.
        """

        # If the grip Button is pressed
        if self.isTracking_state == 0 and self.right_grip_button:

            # Set state to indicate that the button is pressed
            self.isTracking_state = 1

        # If the grip Button is released after being pressed
        elif self.isTracking_state == 1 and not self.right_grip_button:

            # Activate tracking mode
            self.isTracking = True
            self.isTracking_state = 2

            if self.isInitialized:
                print("\nTracking is ON.\n")

        # If the grip Button is pressed again while tracking is on
        elif self.isTracking_state == 2 and self.right_grip_button:

            # Set state to indicate that the button is pressed again
            self.isTracking_state = 3

        # If the grip Button is released after being pressed again
        elif self.isTracking_state == 3 and not self.right_grip_button:

            # Deactivate tracking mode
            self.isTracking = False
            self.isTracking_state = 0

            if self.isInitialized:
                print("\nTracking is OFF.\n")

    def activate_e_stop(self):

        # E-stop arm motion
        self.estop_arm_srv()

        # Stop chest motion
        self.chest_stop_srv()

        # Reset the flag
        self.isTracking = False
        self.isInitialized = False

        # Pause datarecoding
        try:
            self.resume_data_srv(False)

        except Exception as e:
            print(e)

    def emergency_stop(self):

        # Emergency stop
        if self.left_grip_button and self.estopButton_state == False:

            self.activate_e_stop()

            print("\n E-Stop! Shutting down relaxed IK and PID...\n")

            # Shutdown relaxed_IK
            self.relaxed_ik_launch.shutdown()

            # Shutdown PID nodes
            self.pid_launch.shutdown()

            print("\n E-Stop! Relaxed IK and PID have shutdown.\n")

            # Reset the flag
            self.isTracking = False
            self.isTracking_state = 0

            # Change the state
            self.estopButton_state = True

        elif not self.left_grip_button and self.estopButton_state == True:

            # Change the state
            self.estopButton_state = False

        # Clear arm faults
        if self.left_secondary_button and self.clearfaultButton_state == 0:

            self.clearfaults_arm_srv()

            # Reset the flag
            self.isInitialized = False

            # Change the state
            self.clearfaultButton_state = 1

            print("\n Faults cleared, restore an arm position.\n")

        elif not self.left_secondary_button and self.clearfaultButton_state == 1:

            # Change the state
            self.clearfaultButton_state = 2

        elif self.left_secondary_button and self.clearfaultButton_state == 2:

            # Change the state
            self.clearfaultButton_state = 3

        elif not self.left_secondary_button and self.clearfaultButton_state == 3:

            # Change the state
            self.clearfaultButton_state = 0

        # Initialize the robot, home chest and arm
        if self.left_primary_button and self.initButton_state == False:

            print("\n Initialization, starting relaxed IK and PID...\n")

            # Start the relaxed IK
            self.roslaunch_relaxed_ik()

            # Launch PID nodes
            self.roslaunch_pid()

            rospy.sleep(5)

            print(
                "\n Initialization, relaxed IK and PID have started. Homing...\n"
            )

            # Initialize the robot
            self.initialization()

            # Change the state
            self.initButton_state = True

        elif not self.left_primary_button and self.initButton_state == True:
            # Change the state
            self.initButton_state = False

    # Gripper control state machine
    def gripper_sm(self):

        if self.gripperState == "open" and self.gripperButtonState and self.gripperButtonReleased:
            # Change gripper state
            self.gripperState = "close"

            # Close the gripper
            gripper_control(3, 0.6)

        elif self.gripperState == "close" and self.gripperButtonState and self.gripperButtonReleased:
            # Change gripper state
            self.gripperState = "open"

            # Open the gripper
            gripper_control(3, 0.0)

    def tracking(self):
        """Publishes the desired end-effector position and orientation to 
        relaxedIK and provides gripper control. It tracks the target position 
        by calculating the difference between the oculus input and controller 
        compensation.
        """

        if self.isInitialized:

            # Call the state machine for tracking and gripper control
            self.tracking_sm()
            self.gripper_sm()

            # If tracking mode is on
            if self.isTracking:

                # On first press recalculate transition (difference) from oculus input to target position of kinova end-effector
                if self.onTrackingStart:

                    self.oculus_kinova_diff_pos = calculate_controller_ee_diff(
                        self.input_pos_gcs, self.target_pos
                    )

                    # Remove flag
                    self.onTrackingStart = False

                # Target position for relaxedIK: oculus input in GCS (global coordinate system) + difference previously calculated
                self.target_pos = self.input_pos_gcs - self.oculus_kinova_diff_pos

                # EDIT! MOCAP
                # target_pos_kcs = global_to_kinova(Oculus.target_pos)
                # target_pos_kcs = global_to_kinova(system.target_mocap)
                # relaxedik_publish(global_to_kinova(system.target_mocap), [1, 0, 0, 0])

                relaxedik_publish(
                    global_to_kinova(self.target_pos), [1, 0, 0, 0]
                )

                #relaxed_ik_publish(target_pos_kcs, target_rot_kcs)

            # If tracking mode is off
            else:

                # Reset the flag
                self.onTrackingStart = True

            # Control the gripper if the trigger button is pressed
            if self.right_trigger_button:
                self.gripperButtonState = True

                # Call gripper state machine
                self.gripper_sm()

                self.gripperButtonReleased = False

            # If the trigger button is released
            else:
                self.gripperButtonState = False

                # Call gripper state machine
                self.gripper_sm()

                self.gripperButtonReleased = True

            # Chest joystick control
            if abs(self.right_joystick_pos_y) > 0.05:
                chest_vel = np.interp(
                    round(self.right_joystick_pos_y, 4),
                    [-1.0, 1.0],
                    [-0.8, 0.8],
                )

            else:
                chest_vel = 0.0

            # Publish chest velocity
            msg = geom_msgs.Twist()
            msg.linear.z = chest_vel
            Headset.chest_vel_pub.publish(msg)


if __name__ == '__main__':

    Arm = Kinova()
    Headset = Oculus()
    system = Kinova_Oculus_control()

    # Activate chest feedback
    system.chest_logger_srv(True)

    # Initialize the robot
    system.initialization()

    # Open the gripper
    gripper_control(3, 0.0)

    # Main loop
    while not rospy.is_shutdown():

        system.emergency_stop()
        system.tracking()
