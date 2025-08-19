#!/usr/bin/env python
"""
Filtered Kinova Wrench Node

Pipeline:
  1) Baseline capture (when not tracking) over a sliding window (200).
  2) On tracking start, freeze baseline; subtract baseline from incoming wrench.
  3) Soft deadband using baseline sigma to kill small jitter.
  4) Adaptive low-pass: cutoff selected by a spectral "fuzziness" metric
     (HF power ratio + spectral flatness) so motion-induced noise is suppressed.

Publishes:
  /<robot_name>/kinova_wrench/wrench/processed  (geometry_msgs/WrenchStamped)

Author(s):
  TODO
"""

# # Standard libraries:
import math
from collections import deque

# # Third party libraries:
import numpy as np
import rospy

# # Standard messages and services:
from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped


def _soft_deadband(x, d):
    """
    Smooth deadzone: inside [-d, d] -> 0; outside -> shrink by d.
    Vectorized for numpy arrays.
    """
    ax = np.abs(x)
    return np.sign(x) * np.maximum(ax - d, 0.0)


class FilteredKinovaWrench:
    """
    Subscribes to /<robot_name>/kinova_wrench/wrench and republishes a
    processed version on /<robot_name>/kinova_wrench/wrench/processed.
    Tracks /<robot_name>/teleoperation/is_tracking.
    """

    def __init__(self, node_name, robot_name, node_frequency):
        self.__NODE_NAME = node_name
        self.__ROBOT_NAME = robot_name
        self.__FS = float(node_frequency)  # Hz

        # ---- Runtime state
        self.__wrench_input = None
        self.__is_tracking = False
        self.__was_tracking = False

        # Baseline capture (frozen on tracking start)
        self.__baseline_buffer = deque(maxlen=200)
        self.__baseline_mean = None
        self.__baseline_sigma = None

        # Analysis window for fuzziness metric
        self.__analysis_window = deque(maxlen=128)

        # Low-pass filter state (6 axes: fx,fy,fz,tx,ty,tz)
        self.__lp_state = np.zeros(6, dtype=float)

        # ---- Parameters (tunable via ROS params)
        pget = rospy.get_param

        # Deadband
        self.__deadband_sigma_mult = float(
            pget(f'{node_name}/deadband_sigma_mult', 2.5)
        )
        self.__deadband_min = float(pget(f'{node_name}/deadband_min', 0.5))

        # Fuzziness analysis
        self.__hf_split_hz = float(pget(f'{node_name}/hf_split_hz', 6.0))
        self.__hf_ratio_thresh = float(
            pget(f'{node_name}/hf_ratio_thresh', 0.65)
        )
        self.__sfm_thresh = float(pget(f'{node_name}/sfm_thresh', 0.45))

        # Adaptive LPF cutoffs
        self.__fc_low_hz = float(pget(f'{node_name}/fc_low_hz', 1.0))
        self.__fc_high_hz = float(pget(f'{node_name}/fc_high_hz', 8.0))

        self.__was_tracking = False
        self.__warmup_counter = 0  # NEW: suppress frames after rising edge

        # Init status flags
        self.__is_initialized = False
        self.__dependency_initialized = False

        # ---- Publishers
        self.__node_is_initialized = rospy.Publisher(
            f'{self.__NODE_NAME}/is_initialized', Bool, queue_size=1
        )
        self.__processed_wrench_pub = rospy.Publisher(
            f'/{self.__ROBOT_NAME}/kinova_wrench/wrench_processed',
            WrenchStamped,
            queue_size=1,
        )

        # ---- Dependency tracking
        self.__dependency_status = {
            "kinova_wrench": False,
            "teleoperation": False
        }
        self.__dependency_status_topics = {
            "kinova_wrench":
                rospy.Subscriber(
                    f'/{self.__ROBOT_NAME}/kinova_wrench/wrench_raw',
                    WrenchStamped,
                    self.__wrench_callback,
                ),
            "teleoperation":
                rospy.Subscriber(
                    f'/{self.__ROBOT_NAME}/teleoperation/is_tracking',
                    Bool,
                    self.__is_tracking_callback,
                ),
        }

    # -----------------------------
    # Callbacks
    # -----------------------------
    def __wrench_callback(self, msg: WrenchStamped):
        self.__wrench_input = msg
        self.__dependency_status["kinova_wrench"] = True

        # Collect baseline while NOT tracking
        if not self.__is_tracking:
            self.__baseline_buffer.append(self._to_vec(msg))
            # keep analysis window fresh too (helps characterize idle jitter)
            self.__analysis_window.append(self._to_vec(msg))

        else:
            # While tracking, we analyze the processed stream (after baseline/DB)
            pass

    def __is_tracking_callback(self, msg: Bool):
        new_state = bool(msg.data)
        self.__dependency_status["teleoperation"] = True

        if new_state and not self.__was_tracking:
            # Rising edge: freeze baseline
            self.__warmup_counter = 5
            self.__freeze_baseline()
            self.__was_tracking = True
            # rospy.loginfo(
            #     f"{self.__NODE_NAME}: tracking started; baseline frozen."
            # )
            # Reset analysis window to observe tracking-era behavior
            self.__analysis_window.clear()

        if not new_state and self.__was_tracking:
            # Falling edge: reset for next baseline capture
            self.__was_tracking = False
            self.__baseline_mean = None
            self.__baseline_sigma = None
            self.__warmup_counter = 0
            self.__baseline_buffer.clear()
            self.__analysis_window.clear()
            # rospy.loginfo(
            #     f"{self.__NODE_NAME}: tracking stopped; baseline reset."
            # )

        self.__is_tracking = new_state

    # -----------------------------
    # Helpers
    # -----------------------------
    @staticmethod
    def _to_vec(w: WrenchStamped) -> np.ndarray:
        return np.array(
            [
                w.wrench.force.x,
                w.wrench.force.y,
                w.wrench.force.z,
                w.wrench.torque.x,
                w.wrench.torque.y,
                w.wrench.torque.z,
            ],
            dtype=float,
        )

    @staticmethod
    def _from_vec_like(header, v: np.ndarray) -> WrenchStamped:
        out = WrenchStamped()
        out.header = header
        out.header.stamp = rospy.Time.now()
        out.wrench.force.x = float(v[0])
        out.wrench.force.y = float(v[1])
        out.wrench.force.z = float(v[2])
        out.wrench.torque.x = float(v[3])
        out.wrench.torque.y = float(v[4])
        out.wrench.torque.z = float(v[5])
        return out

    def __freeze_baseline(self):
        if len(self.__baseline_buffer) == 0:
            self.__baseline_mean = np.zeros(6)
            self.__baseline_sigma = np.ones(6) * 0.5
            rospy.logwarn(
                f"{self.__NODE_NAME}: no baseline samples; using zeros/min sigma."
            )
            return

        data = np.vstack(self.__baseline_buffer)  # [N,6]
        self.__baseline_mean = np.mean(data, axis=0)
        # Robust sigma: use standard deviation; clamp to avoid zeros
        sigma = np.std(data, axis=0)
        self.__baseline_sigma = np.maximum(sigma, 1e-3)

        rospy.loginfo(
            f"{self.__NODE_NAME}: baseline mean={np.round(self.__baseline_mean,3)}, "
            f"sigma={np.round(self.__baseline_sigma,3)}"
        )

    def __check_initialization(self):
        self.__dependency_initialized = True

        for key, sub in self.__dependency_status_topics.items():
            if sub.get_num_connections() < 1:
                if self.__dependency_status[key]:
                    rospy.logerr(
                        f'{self.__NODE_NAME}: lost connection to {key}!'
                    )
                self.__dependency_status[key] = False

            if not self.__dependency_status[key]:
                self.__dependency_initialized = False

        if not self.__dependency_initialized:
            waiting_for = [
                k for k, v in self.__dependency_status.items() if not v
            ]
            rospy.logwarn_throttle(
                15, f'{self.__NODE_NAME}: waiting for {waiting_for}'
            )

        if self.__dependency_initialized and not self.__is_initialized:
            rospy.loginfo(f'\033[92m{self.__NODE_NAME}: ready.\033[0m')
            self.__is_initialized = True

        if not self.__dependency_initialized and self.__is_initialized:
            rospy.logwarn(f'{self.__NODE_NAME}: lost initialization')
            self.__is_initialized = False

        self.__node_is_initialized.publish(self.__is_initialized)

    # -----------------------------
    # Fuzziness metric & adaptive LPF
    # -----------------------------
    def __compute_fuzziness(self, window_2d: np.ndarray):
        """
        window_2d: shape [N,6], already detrended around baseline and deadbanded.
        Returns: (hf_ratio, spectral_flatness) aggregated across axes.
        """
        eps = 1e-12
        n = window_2d.shape[0]
        if n < 16:
            return 0.0, 0.0

        # Detrend per axis (remove mean)
        X = window_2d - np.mean(window_2d, axis=0, keepdims=True)

        # Aggregate across axes by averaging spectra (equal weight per axis)
        # Use rfft (one-sided)
        freqs = np.fft.rfftfreq(n, d=1.0 / self.__FS)
        hf_mask = freqs >= self.__hf_split_hz

        P_all = []
        sfm_all = []
        for i in range(6):
            p = np.abs(np.fft.rfft(X[:, i]))**2
            P_all.append(p)
            # Spectral flatness for this axis
            gm = np.exp(np.mean(np.log(p + eps)))
            am = np.mean(p + eps)
            sfm_all.append(gm / am)

        P_mean = np.mean(
            np.vstack(P_all), axis=0
        )  # average spectrum across axes
        hf_ratio = float(np.sum(P_mean[hf_mask]) / max(np.sum(P_mean), eps))
        sfm = float(np.mean(sfm_all))

        return hf_ratio, sfm

    def __adaptive_alpha(self, fuzziness_score):
        """
        Map fuzziness in [0,1] to cutoff in [fc_low, fc_high] (more fuzzy -> lower cutoff).
        """
        fc = self.__fc_low_hz + (1.0 - fuzziness_score
                                ) * (self.__fc_high_hz - self.__fc_low_hz)
        # First-order LPF coefficient derived from continuous-time pole:
        # y += alpha * (x - y), alpha = 1 - exp(-2*pi*fc/fs)
        alpha = 1.0 - math.exp(-2.0 * math.pi * fc / self.__FS)
        return max(0.0, min(alpha, 1.0))

    # -----------------------------
    # Processing
    # -----------------------------
    def __process_wrench(self, wrench: WrenchStamped) -> WrenchStamped:
        raw = self._to_vec(wrench)

        # If not tracking, pass-through (and baseline is updating)
        if not self.__is_tracking or self.__baseline_mean is None:
            # Keep LP state close to current (stable output even before tracking)
            self.__lp_state = 0.9 * self.__lp_state + 0.1 * raw
            return self._from_vec_like(wrench.header, self.__lp_state * 0)

        # 1) Baseline subtraction
        z = raw - self.__baseline_mean  # zeroed

        # 2) Soft deadband
        deadband = np.maximum(
            self.__deadband_min,
            self.__deadband_sigma_mult * self.__baseline_sigma
        )
        z_db = _soft_deadband(z, deadband)

        # Keep analysis buffer for fuzziness decision (tracking-era signal)
        self.__analysis_window.append(z_db.copy())

        # 3) Adaptive low-pass based on fuzziness
        z_for_analysis = None
        if len(self.__analysis_window) >= self.__analysis_window.maxlen:
            window = np.vstack(self.__analysis_window)  # [N,6]
            hf_ratio, sfm = self.__compute_fuzziness(window)
            # Convert metrics into a single score in [0,1]
            # Two-threshold soft mapping to be smooth:
            # - hf_ratio: 0.5 -> 0, 0.9 -> 1
            # - sfm:      0.3 -> 0, 0.7 -> 1
            hf_norm = np.clip((hf_ratio - 0.5) / (0.9 - 0.5), 0.0, 1.0)
            sfm_norm = np.clip((sfm - 0.3) / (0.7 - 0.3), 0.0, 1.0)
            fuzziness = 0.5 * hf_norm + 0.5 * sfm_norm
            alpha = self.__adaptive_alpha(fuzziness)

            # LPF update per axis
            self.__lp_state = self.__lp_state + alpha * (z_db - self.__lp_state)

            # Optional: mild amplitude de-emphasis during high fuzziness
            # (helps when motion noise is overwhelmingly broadband)
            gain = 1.0 - 0.3 * fuzziness  # keep at least 70%
            processed = self.__lp_state * gain
        else:
            # Not enough samples yet → use the higher cutoff to stay responsive
            alpha = self.__adaptive_alpha(0.0)  # not fuzzy
            self.__lp_state = self.__lp_state + alpha * (z_db - self.__lp_state)
            processed = self.__lp_state

        if self.__warmup_counter > 0:
            self.__warmup_counter -= 1
            return self._from_vec_like(wrench.header, np.zeros(6))
        return self._from_vec_like(wrench.header, processed)

    # -----------------------------
    # Main loop & shutdown
    # -----------------------------
    def main_loop(self):
        self.__check_initialization()
        if not self.__is_initialized or self.__wrench_input is None:
            return

        processed_wrench = self.__process_wrench(self.__wrench_input)
        self.__processed_wrench_pub.publish(processed_wrench)

    def node_shutdown(self):
        rospy.loginfo_once(f'{self.__NODE_NAME}: node is shutting down...')
        rospy.loginfo_once(f'{self.__NODE_NAME}: node has shut down.')


def main():
    rospy.init_node('filtered_kinova_wrench', log_level=rospy.INFO)
    rospy.loginfo('\n\n\n\n\n')

    node_name = rospy.get_name()
    node_frequency = rospy.get_param(f'{node_name}/node_frequency', 100)
    robot_name = rospy.get_param(f'{node_name}/robot_name', 'my_gen3')

    class_instance = FilteredKinovaWrench(node_name, robot_name, node_frequency)

    rospy.on_shutdown(class_instance.node_shutdown)
    rate = rospy.Rate(node_frequency)

    while not rospy.is_shutdown():
        class_instance.main_loop()
        rate.sleep()


if __name__ == '__main__':
    main()
