import rospy
import numpy as np
import transformations as T
import math
from kortex_driver.msg import *
from kortex_driver.srv import *
from relaxed_ik_ros1.msg import EEPoseGoals
import geometry_msgs.msg as geom_msgs
from std_msgs.msg import *

# Publishers
setpoint_pub = rospy.Publisher(
    '/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1
)

#Service
gripper_command_srv = rospy.ServiceProxy(
    'my_gen3/base/send_gripper_command',
    SendGripperCommand,
)


def relaxedik_publish(target_position, target_orientation):
    """
    This function publishes target position and orientation in the relaxed IK Coordinate system (Kinova) to the topic /relaxed_ik/ee_pose_goals
    
    Args:
        target_position (numpy array): target position in [x, y, z] format
        target_orientation (numpy array): target orientation in [w, x, y, z] format
    """

    # Form a message for relaxedIK (right arm)
    pose_r = geom_msgs.Pose()
    pose_r.position.x = target_position[0]
    pose_r.position.y = target_position[1]
    pose_r.position.z = target_position[2]

    pose_r.orientation.w = target_orientation[0]
    pose_r.orientation.x = target_orientation[1]
    pose_r.orientation.y = target_orientation[2]
    pose_r.orientation.z = target_orientation[3]

    # Form a message for relaxedIK (left arm)
    pose_l = geom_msgs.Pose()
    pose_l.position.x = 0
    pose_l.position.y = 0
    pose_l.position.z = 0

    pose_l.orientation.w = 1
    pose_l.orientation.x = 0
    pose_l.orientation.y = 0
    pose_l.orientation.z = 0

    # Form full message for relaxedIK
    ee_pose_goals = EEPoseGoals()
    ee_pose_goals.ee_poses.append(pose_r)
    ee_pose_goals.ee_poses.append(pose_l)
    ee_pose_goals.header.seq = 0

    # Publish
    setpoint_pub.publish(ee_pose_goals)


def left_to_right_handed(input_rot_gcs):
    """
    Converts quaternions from Left-handed coordinate system to right-handed coordinate system
    
    Args:
        input_rot_gcs (numpy array): quaternions [w, x, y, z]
        
    Returns:
        numpy array: right-handed quaternions
    """

    # Convert to Euler angles
    input_rot_gcs = T.euler_from_quaternion(input_rot_gcs)

    # Swap and negate from (X, Y, Z) to (-Y, -Z, X)
    input_rot_gcs = T.euler_matrix(
        -1 * input_rot_gcs[1],
        -1 * input_rot_gcs[2],
        input_rot_gcs[0],
    )

    # Convert to quaternions
    input_rot_gcs = T.quaternion_from_matrix(input_rot_gcs)

    return input_rot_gcs


def calculate_controller_ee_diff(input_pos_gcs, target_pos):
    """Calculate transition (difference) from oculus input to target position of kinova end-effector
    Args:
        input_pos_gcs (numpy array): controller position [x, y, z]
        target_pos (numpy array): target position [x, y, z]
    Returns:
        numpy array: difference between controller and target
    """

    return input_pos_gcs - target_pos


def global_to_kinova(input_pos_gcs):
    """This function convers position from Global coordinate system to 
    Kinova Coordinate System (mounted on the chest)
    Args:
        input_pos_gcs (numpy array): position in global coordinate system [x, y, z]
    Returns:
        numpy array: controller in Kinova coordinate system
    """

    # Transition from Global CS to Kinova CS: rotate around y and z axis
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    Rx = T.rotation_matrix(math.radians(0.0), xaxis)
    Ry = T.rotation_matrix(math.radians(-45.0), yaxis)
    Rz = T.rotation_matrix(math.radians(90.0), zaxis)

    R_gcs_to_kcs = T.concatenate_matrices(Rx, Ry, Rz)

    # For position
    if len(input_pos_gcs) == 3:
        input_kcs = np.matmul(R_gcs_to_kcs[0:3, 0:3], input_pos_gcs)

    # For orientation
    elif len(input_pos_gcs) == 4:
        input_kcs = np.matmul(R_gcs_to_kcs, input_pos_gcs)

    return input_kcs


# Gripper control: mode=1 - force, mode=2 - velocity, mode=3 - position
def gripper_control(mode, value):
    """Control the gripper by sending a gripper command to the Kinova robot arm.
    
    Args:
        mode (int): The mode of the gripper command (1 for force, 2 for velocity, 3 for position)
        value (float): The value of the gripper command (force, velocity, or position)
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

    gripper_command_srv(gripper_command)


def publish_bool_message(bool, publisher):

    bool_message = Bool()
    bool_message.data = bool
    publisher.publish(bool_message)


def publish_float64_message(float, publisher):

    float_message = Float64()
    float_message.data = float
    publisher.publish(float_message)


def publish_multiarray_message(array, publisher):

    array_message = Float32MultiArray()
    array_message.data = array
    publisher.publish(array_message)
