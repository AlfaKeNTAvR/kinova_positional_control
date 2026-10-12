#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Pose
from oculus_ros.msg import ControllerButtons
import time
import signal
import sys

SAVE_PATH = "axis_dataset.npz"

SAMPLE_RATE = 100.0
SEQUENCE_LENGTH = 50
MAX_RECORD_TIME = 8.0  # safety cap per motion

AXES = ["x", "y", "z", "rot_x", "rot_y", "rot_z", "non_axis"]

class AxisDataCollector:

    def __init__(self):
        rospy.init_node("axis_data_collector")

        self.pose = None
        self.prev_pose = None
        self.buttons = None

        self.dataset_X = []
        self.dataset_Y = []

        self.shutdown_requested = False

        rospy.Subscriber("/right/controller_feedback/pose", Pose, self.pose_cb)
        rospy.Subscriber("/right/controller_feedback/buttons", ControllerButtons, self.button_cb)

        signal.signal(signal.SIGINT, self.handle_exit)

    def handle_exit(self, sig, frame):
        print("\nCtrl+C detected. Saving dataset...")
        self.save()
        sys.exit(0)

    def pose_cb(self, msg):
        self.pose = msg

    def button_cb(self, msg):
        self.buttons = msg

    def compute_6dof_velocity(self):
        if self.prev_pose is None:
            return None

        dt = 1.0 / SAMPLE_RATE

        # Linear velocity
        vx = (self.pose.position.x - self.prev_pose.position.x) / dt
        vy = (self.pose.position.y - self.prev_pose.position.y) / dt
        vz = (self.pose.position.z - self.prev_pose.position.z) / dt

        # Quaternion difference
        q1 = [
            self.prev_pose.orientation.x,
            self.prev_pose.orientation.y,
            self.prev_pose.orientation.z,
            self.prev_pose.orientation.w
        ]
        q2 = [
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ]

        dq = tft.quaternion_multiply(q2, tft.quaternion_inverse(q1))

        # Ensure dq is normalized
        dq = dq / np.linalg.norm(dq)

        # Convert quaternion delta to angular velocity
        angle = 2 * np.arccos(np.clip(dq[3], -1.0, 1.0))

        if angle < 1e-6:
            wx = wy = wz = 0.0
        else:
            axis = dq[:3] / np.sin(angle / 2.0)
            wx, wy, wz = axis * (angle / dt)

        vel = np.array([vx, vy, vz, wx, wy, wz])

        # Normalize entire 6D vector (direction only classification)
        mag = np.linalg.norm(vel)
        if mag > 1e-6:
            vel = vel / mag

        return vel

    def label_to_onehot(self, label):
        y = np.zeros(len(AXES))
        y[AXES.index(label)] = 1.0
        return y

    def record_motion(self, label):

        print("\n===================================")
        print(f"Perform motion: {label.upper()}")
        print("Press and HOLD trigger to record")
        print("===================================")

        rate = rospy.Rate(SAMPLE_RATE)

        # Wait for trigger press
        while not rospy.is_shutdown():
            if self.buttons and self.buttons.trigger_button:
                break
            rate.sleep()

        print("Recording...")

        sequence_buffer = []
        start_time = time.time()

        while not rospy.is_shutdown():

            if not self.buttons.trigger_button:
                break

            if (time.time() - start_time) > MAX_RECORD_TIME:
                break

            if self.pose is None:
                rate.sleep()
                continue

            if self.prev_pose is None:
                self.prev_pose = self.pose
                rate.sleep()
                continue

            vel = self.compute_6dof_velocity()
            self.prev_pose = self.pose

            if vel is not None:
                sequence_buffer.append(vel)

                if len(sequence_buffer) == SEQUENCE_LENGTH:
                    self.dataset_X.append(np.array(sequence_buffer))
                    self.dataset_Y.append(self.label_to_onehot(label))
                    sequence_buffer = []

            rate.sleep()

        print("Recording stopped.")

    def save(self):
        if len(self.dataset_X) == 0:
            print("No data collected.")
            return

        np.savez(SAVE_PATH,
                 X=np.array(self.dataset_X),
                 Y=np.array(self.dataset_Y))

        print(f"\nSaved dataset to {SAVE_PATH}")
        print(f"Total samples: {len(self.dataset_X)}")

    def run(self):

        print("\nAutomatic Axis Data Collection Started")
        print("Cycle order:", AXES)
        print("Press Ctrl+C anytime to save and exit.\n")

        while not rospy.is_shutdown():
            for axis in AXES:
                self.record_motion(axis)


if __name__ == "__main__":
    collector = AxisDataCollector()
    collector.run()