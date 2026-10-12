#!/usr/bin/env python
"""Teleoperation node — service-driven robot API for the Kinova arms.

All button-mapping and control logic lives in .mapping.json files executed by
mapping_player.py.  This node exposes clean services so that mapping files can
start/stop tracking, toggle the gripper, and change the control mode without
any hardcoded button logic here.

Services provided
-----------------
  start_tracking               (Empty)    begin tracking; calculate compensation
  stop_tracking                (Empty)    stop tracking; reset relaxed-IK
  gripper_toggle               (SetBool)  True = close (force/position grasp), False = open
  set_full_mode                (SetBool)  True = full (pos+ori), False = position only
  allow_tracking               (SetBool)  enable/disable tracking permission externally
  enable_velocity_mode         (SetBool)  True = velocity control, False = position
  enable_fixed_orientation_mode(SetBool)  True = fixed orientation (reads /<robot>/teleoperation/fixed_mode param)
  pause_relaxed_ik             (Empty)
  resume_relaxed_ik            (Empty)
  reset_relaxed_ik             (Empty)
  gripper/set_open_position    (GripperPosition)
  gripper/set_close_position   (GripperPosition)
  gripper/enable_position_mode (SetBool)

Topics IN
---------
  input_pose             (Pose)    desired EE pose; consumed each tick while tracking
  teleoperation/floor_z  (Float64) minimum EE height in global base_link Z (m).
                                   Publishes a std_msgs/Float64 at any time to update.
                                   Set to a very negative number to disable (default: -inf).
                                   Acts as a motion constraint: downward commands are
                                   suppressed when the EE is at or below this height;
                                   tracking is never interrupted by the floor.

Topics OUT
----------
  is_initialized         (Bool)
  is_tracking            (Bool)
  position_only          (Bool)
  current_ee_pose        (Pose)    EE pose in global base_link frame
  current_ee_pose_local  (Pose)    EE pose in {robot_name}/base_link frame — used by
                                   the joystick mapping's arm-local integrators so that
                                   accumulated positions stay in the arm's own frame,
                                   eliminating axis cross-talk from the TF rotation
  floor_z_input_space    (Float64) The motion floor translated into input_pose Z space:
                                   floor_z + diff.z  — the raw input Z value that
                                   corresponds to the arm being exactly at floor_z.
                                   Pose-mapping clients clamp their raw controller Z to
                                   this value so the 'goal' never drifts below the floor
                                   while the floor constraint is active.
                                   Published as -inf when floor is disabled, tracking is
                                   stopped, or control mode is velocity.

Parameters
----------
  robot_name                          (str)   default 'my_gen3'
  compensate_orientation              (bool)  default True
  convenience_compensation            (list)  default [0,0,0]
  maximum_input_position_change       (float) default 0.1 m
  maximum_input_orientation_change    (float) default 30 deg
  node_frequency                      (int)   default 1000 Hz
  input_pose_frame                    (str)   'base_link' (default) or 'arm_local'
    'base_link':  input_pose is in the global base_link frame.  The node applies
                  a TF transform to {robot_name}/base_link before publishing to
                  relaxed-IK.  Original behaviour.
    'arm_local':  input_pose is already in {robot_name}/base_link.  The TF transform
                  step is skipped.  Use this together with current_ee_pose_local seeding
                  and the joystick mapping's arm-local integrators to eliminate the axis
                  cross-talk that occurs when global-frame integrators are rotated into
                  the arm's base frame.  All internal references (compensation diff, jump
                  check) run in the arm-local frame when this mode is active.

Authors:
    1. Nikita Boguslavskii (bognik3@gmail.com), HiRo lab, WPI, 2023.
    2. Lorena Genua (lorena.genau@gmail.com), HiRo lab, WPI, 2023.
"""

import rospy
import numpy as np
import transformations
from ast import literal_eval
from copy import deepcopy
import tf2_ros
import tf2_geometry_msgs  # required for tf2_ros.Buffer.transform()

from std_msgs.msg import Bool, Float64
from std_srvs.srv import SetBool, Empty
from geometry_msgs.msg import Pose, PoseStamped

from kinova_positional_control.srv import GripperForceGrasping, GripperPosition
from relaxed_ik_ros1.msg import EEPoseGoals
from kortex_driver.srv import Base_ClearFaults


class KinovaTeleoperation:

    def __init__(
        self,
        robot_name,
        compensate_orientation,
        convenience_compensation,
        maximum_input_position_change,
        maximum_input_orientation_change,
        input_pose_frame='base_link',
    ):
        self.ROBOT_NAME = robot_name
        self.COMPENSATE_ORIENTATION = compensate_orientation
        self.CONVENIENCE_COMPENSATION = convenience_compensation
        self.MAX_POS_CHANGE = maximum_input_position_change
        self.MAX_ORI_CHANGE = maximum_input_orientation_change

        # 'base_link'  — default: input_pose is in global base_link; TF applied before publish
        # 'arm_local'  — input_pose is in {robot_name}/base_link; no TF needed.
        #                All compensation and jump-check maths run in arm-local frame.
        self.INPUT_POSE_FRAME = input_pose_frame

        # ── State ─────────────────────────────────────────────────────────────
        self.__input_pose = {
            'position':    np.array([0., 0., 0.]),
            'orientation': np.array([1., 0., 0., 0.]),
        }
        self.__pose_tracking  = False
        self.__allow_tracking = True
        # 'position' | 'full' | 'velocity' | 'fixed_front' | 'fixed_top'
        self.__control_mode   = 'position'

        # ── Reference poses ───────────────────────────────────────────────────
        # last_rik_pose  : EE pose in global base_link frame.
        #                  Always maintained — used for floor-constraint Z comparison
        #                  and publishing current_ee_pose.
        # _local_rik_pose: EE pose in {robot_name}/base_link frame.
        #                  Used by arm_local mode for compensation, jump-check, and
        #                  publishing current_ee_pose_local.
        self.last_rik_pose   = None   # global base_link
        self._local_rik_pose = None   # {robot_name}/base_link

        # Compensation offset (diff = input − EE_ref at tracking start)
        # Lives in the same frame as input_pose (either global or arm-local).
        self.__diff = {
            'position':    np.array([0., 0., 0.]),
            'orientation': np.array([1., 0., 0., 0.]),
        }

        # Velocity mode state
        self.__input_pose_neutral = None
        self.__previous_time      = None

        # Gripper config
        self.__gripper_open_pos  = 0.0
        self.__gripper_close_pos = 1.0
        self.__gripper_mode      = 'force'   # 'force' | 'position'

        # ── Motion floor ──────────────────────────────────────────────────────
        # Minimum EE height in global base_link Z (metres).
        # When the EE is at or below this value, downward commands are suppressed
        # (comp Z is held at the current EE Z) while tracking continues normally.
        # Updated at runtime via /{robot_name}/teleoperation/floor_z.
        self.__floor_z = float('-inf')

        # Dependency tracking
        self.__is_init  = False
        self.__dep_init = False
        self.__dep_status = {
            'chest_control':  False,
            'joints_control': False,
            'gripper_control':False,
        }
        self.__dep_topics = {
            'chest_control': rospy.Subscriber(
                '/chest_control/is_initialized', Bool,
                lambda m: self.__dep_cb('chest_control', m)),
            'joints_control': rospy.Subscriber(
                f'/{robot_name}/joints_control/is_initialized', Bool,
                lambda m: self.__dep_cb('joints_control', m)),
            'gripper_control': rospy.Subscriber(
                f'/{robot_name}/gripper_control/is_initialized', Bool,
                lambda m: self.__dep_cb('gripper_control', m)),
        }

        # ── Services provided ─────────────────────────────────────────────────
        rospy.Service(f'/{robot_name}/teleoperation/start_tracking',
                      Empty, self.__start_tracking_handler)
        rospy.Service(f'/{robot_name}/teleoperation/stop_tracking',
                      Empty, self.__stop_tracking_handler)
        rospy.Service(f'/{robot_name}/teleoperation/gripper_toggle',
                      SetBool, self.__gripper_toggle_handler)
        rospy.Service(f'/{robot_name}/teleoperation/set_full_mode',
                      SetBool, self.__set_full_mode_handler)
        rospy.Service(f'/{robot_name}/teleoperation/allow_tracking',
                      SetBool, self.__allow_tracking_handler)
        rospy.Service(f'/{robot_name}/teleoperation/enable_velocity_mode',
                      SetBool, self.__enable_velocity_mode_handler)
        rospy.Service(f'/{robot_name}/teleoperation/enable_fixed_orientation_mode',
                      SetBool, self.__enable_fixed_orientation_handler)
        rospy.Service(f'/{robot_name}/teleoperation/pause_relaxed_ik',
                      Empty, lambda _: (self.__pause_rik(), [])[1])
        rospy.Service(f'/{robot_name}/teleoperation/resume_relaxed_ik',
                      Empty, lambda _: (self.__resume_rik(), [])[1])
        rospy.Service(f'/{robot_name}/teleoperation/reset_relaxed_ik',
                      Empty, lambda _: (self.__reset_rik_full(), [])[1])
        rospy.Service(f'/{robot_name}/teleoperation/gripper/set_open_position',
                      GripperPosition, self.__set_open_handler)
        rospy.Service(f'/{robot_name}/teleoperation/gripper/set_close_position',
                      GripperPosition, self.__set_close_handler)
        rospy.Service(f'/{robot_name}/teleoperation/gripper/enable_position_mode',
                      SetBool, self.__gripper_mode_handler)

        # ── Service clients ───────────────────────────────────────────────────
        self.__svc_grip_force = rospy.ServiceProxy(
            f'/{robot_name}/gripper_control/force_grasping', GripperForceGrasping)
        self.__svc_grip_pos   = rospy.ServiceProxy(
            f'/{robot_name}/gripper_control/position', GripperPosition)
        self.__svc_pid        = rospy.ServiceProxy(
            f'/{robot_name}/joints_control/enable_pid', SetBool)
        self.__svc_rik_reset  = rospy.ServiceProxy(
            f'/{robot_name}/relaxed_ik/reset', Empty)
        self.__svc_clear_faults = rospy.ServiceProxy(
            f'/{robot_name}/base/clear_faults', Base_ClearFaults)

        # ── Publishers ────────────────────────────────────────────────────────
        self.__pub_init         = rospy.Publisher(
            f'/{robot_name}/teleoperation/is_initialized', Bool, queue_size=1)
        self.__pub_rik          = rospy.Publisher(
            f'/{robot_name}/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=1)
        self.__pub_is_tracking  = rospy.Publisher(
            f'/{robot_name}/teleoperation/is_tracking', Bool, queue_size=1)
        self.__pub_pos_only     = rospy.Publisher(
            f'/{robot_name}/teleoperation/position_only', Bool, queue_size=1)
        self.__pub_current_pose = rospy.Publisher(
            f'/{robot_name}/teleoperation/current_ee_pose', Pose, queue_size=1)
        self.__pub_current_pose_local = rospy.Publisher(
            f'/{robot_name}/teleoperation/current_ee_pose_local', Pose, queue_size=1)
        self.__pub_floor_z_eff = rospy.Publisher(
            f'/{robot_name}/teleoperation/floor_z_input_space', Float64, queue_size=1)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber(
            f'/{robot_name}/teleoperation/input_pose',
            Pose, self.__input_pose_cb)
        rospy.Subscriber(
            f'/{robot_name}/teleoperation/floor_z',
            Float64, self.__floor_z_cb)

        # ── TF ────────────────────────────────────────────────────────────────
        self.__tf_buf = tf2_ros.Buffer(rospy.Duration(1))
        tf2_ros.TransformListener(self.__tf_buf)

    # ─────────────────────────────────────────────────────────────────────────
    # Reference pose helper
    # ─────────────────────────────────────────────────────────────────────────

    @property
    def _ref_pose(self):
        """The EE reference pose in the SAME FRAME as input_pose.

        All compensation, jump-check, and mode computations must use this so
        that frame-mixed arithmetic never occurs.
        """
        if self.INPUT_POSE_FRAME == 'arm_local':
            return self._local_rik_pose
        return self.last_rik_pose

    # ─────────────────────────────────────────────────────────────────────────
    # Dependency callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def __dep_cb(self, key, msg):
        self.__dep_status[key] = msg.data

    # ─────────────────────────────────────────────────────────────────────────
    # Topic callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def __input_pose_cb(self, msg):
        self.__input_pose['position'][0] = msg.position.x
        self.__input_pose['position'][1] = msg.position.y
        self.__input_pose['position'][2] = msg.position.z
        self.__input_pose['orientation'][0] = msg.orientation.w
        self.__input_pose['orientation'][1] = msg.orientation.x
        self.__input_pose['orientation'][2] = msg.orientation.y
        self.__input_pose['orientation'][3] = msg.orientation.z

    def __floor_z_cb(self, msg):
        """Update the minimum allowed EE height (global base_link Z) at runtime."""
        self.__floor_z = float(msg.data)
        rospy.loginfo_throttle(
            5.0,
            f'/{self.ROBOT_NAME}/teleoperation: floor_z → {self.__floor_z:.3f} m')

    # ─────────────────────────────────────────────────────────────────────────
    # Service handlers
    # ─────────────────────────────────────────────────────────────────────────

    def __start_tracking_handler(self, _):
        if self.__is_init and self.__allow_tracking:
            self.__calculate_compensation()
            self.__pose_tracking = True
            rospy.loginfo(f'/{self.ROBOT_NAME}/teleoperation: tracking STARTED')
        return []

    def __stop_tracking_handler(self, _):
        self.__pose_tracking = False
        if self.__is_init:
            try:
                self.__svc_rik_reset()
            except Exception as ex:
                rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: rik reset on stop: {ex}')
        rospy.loginfo(f'/{self.ROBOT_NAME}/teleoperation: tracking STOPPED')
        return []

    def __gripper_toggle_handler(self, req):
        if not self.__is_init:
            return False, 'not initialized'
        if req.data:
            if self.__gripper_mode == 'force':
                self.__svc_grip_force(0.0)
            else:
                self.__svc_grip_pos(self.__gripper_close_pos)
            return True, 'gripper closed'
        else:
            self.__svc_grip_pos(self.__gripper_open_pos)
            return True, 'gripper opened'

    def __set_full_mode_handler(self, req):
        if not self.__is_init:
            return False, 'not initialized'
        self.__control_mode = 'full' if req.data else 'position'
        if req.data:
            self.__calculate_compensation()
        mode_str = 'full' if req.data else 'position'
        rospy.loginfo(f'/{self.ROBOT_NAME}/teleoperation: mode = {mode_str}')
        return True, mode_str

    def __allow_tracking_handler(self, req):
        self.__allow_tracking = req.data
        if not req.data and self.__pose_tracking:
            self.__pose_tracking = False
            try:
                self.__svc_rik_reset()
            except Exception as ex:
                rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: rik reset on disallow: {ex}')
        state = 'allowed' if req.data else 'disallowed'
        rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: tracking {state}')
        return True, state

    def __enable_velocity_mode_handler(self, req):
        self.__control_mode = 'velocity' if req.data else 'position'
        if req.data and self.__pose_tracking:
            self.__calculate_compensation()
        rospy.logwarn(
            f'/{self.ROBOT_NAME}/teleoperation: '
            f'control_mode → {self.__control_mode} (velocity mode service)')
        return True, self.__control_mode

    def __enable_fixed_orientation_handler(self, req):
        if req.data:
            try:
                self.__control_mode = rospy.get_param(
                    f'/{self.ROBOT_NAME}/teleoperation/fixed_mode')
            except KeyError:
                rospy.logwarn(
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    f'fixed_mode param not set, defaulting to fixed_front')
                self.__control_mode = 'fixed_front'
        else:
            self.__control_mode = 'position'
        if self.__pose_tracking:
            try:
                self.__calculate_compensation()
            except Exception as ex:
                rospy.logerr(
                    f'/{self.ROBOT_NAME}/teleoperation: '
                    f'compensation failed after mode change: {ex}')
                self.__stop_tracking_internal()
        rospy.logwarn(
            f'/{self.ROBOT_NAME}/teleoperation: '
            f'control_mode → {self.__control_mode} (fixed orientation service)')
        return True, self.__control_mode

    def __set_open_handler(self, req):
        self.__gripper_open_pos = float(np.clip(req.position, 0., 1.))
        return True

    def __set_close_handler(self, req):
        self.__gripper_close_pos = float(np.clip(req.position, 0., 1.))
        return True

    def __gripper_mode_handler(self, req):
        self.__gripper_mode = 'position' if req.data else 'force'
        return True, self.__gripper_mode

    # ─────────────────────────────────────────────────────────────────────────
    # Private methods
    # ─────────────────────────────────────────────────────────────────────────

    def __check_initialization(self):
        self.__dep_init = True
        for key in self.__dep_status:
            if self.__dep_topics[key].get_num_connections() != 1:
                if self.__dep_status[key]:
                    rospy.logerr(
                        f'/{self.ROBOT_NAME}/teleoperation: '
                        f'lost connection to {key}!')
                self.__dep_status[key] = False
            if not self.__dep_status[key]:
                self.__dep_init = False

        if not self.__dep_init:
            waiting = ''.join(
                f'\n- waiting for {k}...'
                for k, v in self.__dep_status.items() if not v)
            rospy.logwarn_throttle(
                15, f'/{self.ROBOT_NAME}/teleoperation:{waiting}')

        if self.__dep_status['chest_control']:
            self.__update_rik_pose()

        if self.__dep_init and self._ref_pose is not None:
            if not self.__is_init:
                rospy.loginfo(
                    f'\033[92m/{self.ROBOT_NAME}/teleoperation: ready.\033[0m')
                self.__is_init = True
        else:
            if self.__is_init:
                self.__pose_tracking = False
            self.__is_init = False

        self.__pub_init.publish(self.__is_init)

    def __update_rik_pose(self):
        """Read the current relaxed-IK target from TF (only when not tracking).

        Maintains both last_rik_pose (global base_link) and _local_rik_pose
        ({robot_name}/base_link) so callers can always use _ref_pose regardless
        of input_pose_frame without risking frame-mixed arithmetic.
        """
        if self.__pose_tracking:
            return

        # ── Global pose (base_link) ───────────────────────────────────────────
        try:
            tf = self.__tf_buf.lookup_transform(
                'base_link',
                f'{self.ROBOT_NAME}/relaxed_ik_target',
                rospy.Time(0))
            t = tf.transform.translation
            r = tf.transform.rotation
            self.last_rik_pose = {
                'position':    np.array([t.x, t.y, t.z]),
                'orientation': np.array([r.w, r.x, r.y, r.z]),
            }
        except Exception:
            pass

        # ── Arm-local pose ({robot_name}/base_link) ───────────────────────────
        try:
            tf_l = self.__tf_buf.lookup_transform(
                f'{self.ROBOT_NAME}/base_link',
                f'{self.ROBOT_NAME}/relaxed_ik_target',
                rospy.Time(0))
            tl = tf_l.transform.translation
            rl = tf_l.transform.rotation
            self._local_rik_pose = {
                'position':    np.array([tl.x, tl.y, tl.z]),
                'orientation': np.array([rl.w, rl.x, rl.y, rl.z]),
            }
        except Exception:
            pass

    def __calculate_compensation(self):
        """Compute the offset between input_pose and current EE pose.

        Both quantities are expressed in the same frame (determined by
        INPUT_POSE_FRAME), so the subtraction is always frame-consistent.
        """
        ref = self._ref_pose
        if ref is None:
            return

        if self.__control_mode == 'velocity':
            self.__input_pose_neutral = deepcopy(self.__input_pose)
            self.__previous_time = rospy.Time.now()
            return

        self.__diff['position'] = (
            self.__input_pose['position'] - ref['position'])
        self.__diff['orientation'] = transformations.quaternion_multiply(
            transformations.quaternion_inverse(self.__input_pose['orientation']),
            ref['orientation'])

    def __stop_tracking_internal(self):
        """Internal stop — does not call back into service handlers."""
        self.__pose_tracking = False
        try:
            self.__svc_rik_reset()
        except Exception as ex:
            rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: rik reset: {ex}')

    def __pause_rik(self):
        try:
            self.__svc_pid(False)
        except Exception as ex:
            rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: pause_rik: {ex}')

    def __resume_rik(self):
        try:
            self.__svc_rik_reset()
            self.__svc_pid(True)
        except Exception as ex:
            rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: resume_rik: {ex}')

    def __reset_rik_full(self):
        self.__pause_rik()
        try:
            self.__svc_clear_faults()
        except Exception as ex:
            rospy.logwarn(f'/{self.ROBOT_NAME}/teleoperation: clear_faults: {ex}')
        self.__resume_rik()

    def __publish_current_pose(self):
        """Publish EE pose in global base_link frame."""
        if self.last_rik_pose is None:
            return
        msg = Pose()
        p, q = self.last_rik_pose['position'], self.last_rik_pose['orientation']
        msg.position.x = float(p[0]); msg.position.y = float(p[1]); msg.position.z = float(p[2])
        msg.orientation.w = float(q[0]); msg.orientation.x = float(q[1])
        msg.orientation.y = float(q[2]); msg.orientation.z = float(q[3])
        self.__pub_current_pose.publish(msg)

    def __publish_current_pose_local(self):
        """Publish EE pose in {robot_name}/base_link frame.

        Joystick-mapping integrators seed from this topic so that accumulated
        positions stay in the arm's own frame, eliminating axis cross-talk that
        would otherwise arise when the TF rotation between frames mixes axes.
        """
        if self._local_rik_pose is None:
            return
        msg = Pose()
        p, q = self._local_rik_pose['position'], self._local_rik_pose['orientation']
        msg.position.x = float(p[0]); msg.position.y = float(p[1]); msg.position.z = float(p[2])
        msg.orientation.w = float(q[0]); msg.orientation.x = float(q[1])
        msg.orientation.y = float(q[2]); msg.orientation.z = float(q[3])
        self.__pub_current_pose_local.publish(msg)

    def __publish_floor_z_effective(self):
        """Publish the floor Z threshold translated into input_pose Z space.

        Pose-mapping clients subscribe to this topic and clamp their raw
        controller Z to the published value.  This prevents the 'goal' (the
        input_pose being published by the mapping) from drifting below the
        floor while teleoperation.py's constraint holds the arm at the floor —
        eliminating the dead-zone the user would otherwise have to travel back
        through before the arm responds to upward motion again.

        The relationship is:
            floor_z_input_space = floor_z + diff.z
        because:
            comp.z = input_pose.z - diff.z
            comp.z >= floor_z  ⟺  input_pose.z >= floor_z + diff.z

        Published as -inf when:
          • floor is disabled (floor_z == -inf)
          • not currently tracking (diff is stale / meaningless)
          • velocity mode (diff.z is not a position offset in velocity mode)
        In those states max(raw_z, -inf) == raw_z, so the mapping is a no-op.
        """
        if (self.__floor_z <= float('-inf')
                or not self.__pose_tracking
                or self.__control_mode == 'velocity'):
            eff = float('-inf')
        else:
            eff = self.__floor_z + float(self.__diff['position'][2])
        msg = Float64()
        msg.data = eff
        self.__pub_floor_z_eff.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Floor constraint helper
    # ─────────────────────────────────────────────────────────────────────────

    def __apply_floor_constraint(self, comp):
        """Suppress downward motion when the EE is at or below floor_z.

        floor_z is always measured in global base_link Z.  When the constraint
        activates, the Z component of the commanded position is held at the
        current EE Z so the arm cannot move further downward — but it CAN move
        upward freely, and horizontal motion is unaffected.  Tracking is never
        interrupted.

        Works in both input_pose_frame modes:
          base_link  — comp['position'][2] is global Z; clamp directly.
          arm_local  — comp['position'][2] is arm-local Z; compare commanded
                       arm-local Z delta against a threshold derived from the
                       current global EE Z.  If global Z is at/below the floor
                       and the arm-local command would move it lower, freeze the
                       arm-local Z at the current value.

        Returns True if the constraint was active this tick (for logging).
        """
        if self.__floor_z <= float('-inf'):
            return False

        if self.INPUT_POSE_FRAME == 'arm_local':
            if self._local_rik_pose is None or self.last_rik_pose is None:
                return False

            # Global Z of the current EE (last known)
            current_global_z = self.last_rik_pose['position'][2]
            # Only engage when the EE is at or below the floor in global frame
            if current_global_z > self.__floor_z:
                return False

            # Commanded arm-local Z vs current arm-local Z
            commanded_local_z = comp['position'][2]
            current_local_z   = self._local_rik_pose['position'][2]

            if commanded_local_z < current_local_z:
                # Command would push the arm-local Z lower → suppress it
                comp['position'][2] = current_local_z
                return True

        else:
            # base_link mode: comp['position'][2] IS the commanded global Z
            if comp['position'][2] < self.__floor_z:
                comp['position'][2] = self.__floor_z
                return True

        return False

    # ─────────────────────────────────────────────────────────────────────────
    # Main pose computation
    # ─────────────────────────────────────────────────────────────────────────

    def __publish_kinova_pose(self):
        """Compute compensated pose and send to relaxed-IK.

        Frame consistency
        ~~~~~~~~~~~~~~~~~
        All maths (compensation, mode computations, jump check) use _ref_pose,
        which lives in the same frame as input_pose (global base_link when
        INPUT_POSE_FRAME=='base_link', arm-local when 'arm_local').  This
        eliminates the frame-mixed subtraction bug that was present when
        last_rik_pose (global) was used directly in arm_local mode.

        Floor constraint
        ~~~~~~~~~~~~~~~~
        After computing comp, __apply_floor_constraint() suppresses any
        downward Z component when the EE is at or below floor_z (global Z).
        The constraint never stops tracking; it simply freezes the Z command.

        Publishing
        ~~~~~~~~~~
        base_link mode: comp is in global frame → TF-transform to arm-local → publish.
        arm_local mode: comp is already in arm-local frame → publish directly.
        In both cases, both last_rik_pose (global) and _local_rik_pose are kept
        in sync with our own commands for the next-tick jump check and floor check.
        """
        ref = self._ref_pose
        if ref is None:
            return

        comp = {
            'position':    np.array([0., 0., 0.]),
            'orientation': np.array([1., 0., 0., 0.]),
        }

        # ── Velocity mode ────────────────────────────────────────────────────
        if self.__control_mode == 'velocity':
            scale = 2
            vel = self.__velocity_from_deviation(
                current=self.__input_pose['position'],
                setpoint=self.__input_pose_neutral['position'],
                axis_scale=np.array([scale, scale, scale]),
                dead_zone_radius=0.01,
            )
            dt = (rospy.Time.now() - self.__previous_time).to_sec()
            comp['position'] = ref['position'] + vel * dt
            self.__previous_time = rospy.Time.now()
            comp['orientation'] = np.array([0.5, 0.5, 0.5, 0.5])

        # ── Fixed orientation modes ──────────────────────────────────────────
        elif self.__control_mode == 'fixed_front':
            comp['position']    = self.__input_pose['position'] - self.__diff['position']
            comp['orientation'] = np.array([0.5, 0.5, 0.5, 0.5])

        elif self.__control_mode == 'fixed_top':
            comp['position']    = self.__input_pose['position'] - self.__diff['position']
            comp['orientation'] = np.array([0., -0.707, -0.707, 0.])

        # ── Position-only mode ───────────────────────────────────────────────
        elif self.__control_mode == 'position':
            comp['position']    = self.__input_pose['position'] - self.__diff['position']
            comp['orientation'] = ref['orientation']

        # ── Full (position + orientation) mode ───────────────────────────────
        elif self.__control_mode == 'full':
            comp['position'] = self.__input_pose['position'] - self.__diff['position']
            if self.COMPENSATE_ORIENTATION:
                comp['orientation'] = transformations.quaternion_multiply(
                    self.__input_pose['orientation'],
                    self.__diff['orientation'])
            else:
                q = self.__input_pose['orientation'].copy()
                for deg, axis in zip(self.CONVENIENCE_COMPENSATION,
                                     [(1,0,0), (0,1,0), (0,0,1)]):
                    q = transformations.quaternion_multiply(
                        q, transformations.quaternion_about_axis(
                            np.deg2rad(deg), axis))
                comp['orientation'] = q

        # ── Motion floor constraint ───────────────────────────────────────────
        # Applied before the jump check so that holding Z at the floor never
        # triggers a spurious large-jump stop.
        floor_active = self.__apply_floor_constraint(comp)
        if floor_active:
            rospy.logwarn_throttle(
                1.0,
                f'/{self.ROBOT_NAME}/teleoperation: '
                f'floor constraint active '
                f'(global EE Z ≈ {self.last_rik_pose["position"][2]:.3f} m, '
                f'floor {self.__floor_z:.3f} m)')

        # ── Position jump protection ─────────────────────────────────────────
        # Comparison uses ref_pose (same frame as comp) — no frame mixing.
        pos_diff = np.linalg.norm(comp['position'] - ref['position'])
        if pos_diff > self.MAX_POS_CHANGE:
            self.__stop_tracking_internal()
            rospy.logerr(
                f'/{self.ROBOT_NAME}/teleoperation: '
                f'position jump {pos_diff:.3f} m — tracking stopped.')
            return

        # ── Orientation jump protection ──────────────────────────────────────
        q_err = transformations.quaternion_multiply(
            transformations.quaternion_inverse(comp['orientation']),
            ref['orientation'])
        ang = np.rad2deg(2 * np.arctan2(np.linalg.norm(q_err[1:4]), q_err[0]))
        if ang > 180:
            ang = 360 - ang
        if ang > self.MAX_ORI_CHANGE:
            self.__stop_tracking_internal()
            rospy.logerr(
                f'/{self.ROBOT_NAME}/teleoperation: '
                f'orientation jump {ang:.1f}° — tracking stopped.')
            return

        # ── Publish to relaxed-IK ─────────────────────────────────────────────

        if self.INPUT_POSE_FRAME == 'arm_local':
            # comp is already in {robot_name}/base_link → publish directly
            pose_msg = Pose()
            pose_msg.position.x    = float(comp['position'][0])
            pose_msg.position.y    = float(comp['position'][1])
            pose_msg.position.z    = float(comp['position'][2])
            pose_msg.orientation.w = float(comp['orientation'][0])
            pose_msg.orientation.x = float(comp['orientation'][1])
            pose_msg.orientation.y = float(comp['orientation'][2])
            pose_msg.orientation.z = float(comp['orientation'][3])
            goals = EEPoseGoals()
            goals.ee_poses.append(pose_msg)
            goals.ee_poses.append(pose_msg)
            self.__pub_rik.publish(goals)

            # Keep _local_rik_pose in sync with our commands (used by jump
            # check and floor constraint on the next tick).
            self._local_rik_pose = {
                'position':    comp['position'].copy(),
                'orientation': comp['orientation'].copy(),
            }
            # Keep last_rik_pose (global) updated so the floor Z comparison
            # remains valid even while tracking in arm_local mode.
            try:
                stamped = PoseStamped()
                stamped.header.stamp    = rospy.Time(0)
                stamped.header.frame_id = f'{self.ROBOT_NAME}/base_link'
                stamped.pose.position.x    = float(comp['position'][0])
                stamped.pose.position.y    = float(comp['position'][1])
                stamped.pose.position.z    = float(comp['position'][2])
                stamped.pose.orientation.w = float(comp['orientation'][0])
                stamped.pose.orientation.x = float(comp['orientation'][1])
                stamped.pose.orientation.y = float(comp['orientation'][2])
                stamped.pose.orientation.z = float(comp['orientation'][3])
                pg = self.__tf_buf.transform(stamped, 'base_link')
                self.last_rik_pose = {
                    'position': np.array([
                        pg.pose.position.x,
                        pg.pose.position.y,
                        pg.pose.position.z,
                    ]),
                    'orientation': np.array([
                        pg.pose.orientation.w,
                        pg.pose.orientation.x,
                        pg.pose.orientation.y,
                        pg.pose.orientation.z,
                    ]),
                }
            except Exception as ex:
                rospy.logwarn_throttle(
                    1, f'/{self.ROBOT_NAME}/teleoperation: global pose sync: {ex}')

        else:
            # base_link mode: transform comp from global to arm-local frame
            stamped = PoseStamped()
            stamped.header.stamp    = rospy.Time(0)
            stamped.header.frame_id = 'base_link'
            stamped.pose.position.x    = float(comp['position'][0])
            stamped.pose.position.y    = float(comp['position'][1])
            stamped.pose.position.z    = float(comp['position'][2])
            stamped.pose.orientation.w = float(comp['orientation'][0])
            stamped.pose.orientation.x = float(comp['orientation'][1])
            stamped.pose.orientation.y = float(comp['orientation'][2])
            stamped.pose.orientation.z = float(comp['orientation'][3])

            try:
                p = self.__tf_buf.transform(
                    object_stamped=stamped,
                    target_frame=f'{self.ROBOT_NAME}/base_link')
                pose_msg = Pose()
                pose_msg.position.x    = p.pose.position.x
                pose_msg.position.y    = p.pose.position.y
                pose_msg.position.z    = p.pose.position.z
                pose_msg.orientation.x = p.pose.orientation.x
                pose_msg.orientation.y = p.pose.orientation.y
                pose_msg.orientation.z = p.pose.orientation.z
                pose_msg.orientation.w = p.pose.orientation.w
                goals = EEPoseGoals()
                goals.ee_poses.append(pose_msg)
                goals.ee_poses.append(pose_msg)
                self.__pub_rik.publish(goals)

                # Keep last_rik_pose in sync with our commands (global frame).
                self.last_rik_pose = {
                    'position':    comp['position'].copy(),
                    'orientation': comp['orientation'].copy(),
                }

            except Exception as ex:
                rospy.logerr_throttle(
                    1, f'/{self.ROBOT_NAME}/teleoperation: publish pose: {ex}')

    def __velocity_from_deviation(
        self,
        current,
        setpoint,
        axis_scale,
        dead_zone_radius=0.0,
        max_speed=np.array([1., 1., 1.]),
    ):
        """Velocity proportional to distance outside a dead-zone sphere."""
        current    = np.asarray(current,    dtype=float)
        setpoint   = np.asarray(setpoint,   dtype=float)
        axis_scale = np.asarray(axis_scale, dtype=float)

        e    = current - setpoint
        dist = np.linalg.norm(e)

        if dist <= dead_zone_radius or dist == 0.:
            return np.zeros(3, dtype=float)

        excess    = dist - dead_zone_radius
        direction = e / dist
        v = (excess * direction) * axis_scale

        if max_speed is not None:
            max_speed = np.asarray(max_speed, dtype=float)
            v = np.clip(v, -np.abs(max_speed), np.abs(max_speed))

        return v

    # ─────────────────────────────────────────────────────────────────────────
    # Public
    # ─────────────────────────────────────────────────────────────────────────

    def main_loop(self):
        self.__check_initialization()
        self.__publish_current_pose()
        self.__publish_current_pose_local()
        self.__publish_floor_z_effective()

        if not self.__is_init:
            return

        if self.__pose_tracking:
            self.__publish_kinova_pose()

        rospy.logdebug_throttle(
            1.0,
            f'/{self.ROBOT_NAME}/teleoperation: '
            f'is_tracking={self.__pose_tracking}  '
            f'input_pose=({self.__input_pose["position"][0]:.3f},'
            f'{self.__input_pose["position"][1]:.3f},'
            f'{self.__input_pose["position"][2]:.3f})')
        self.__pub_is_tracking.publish(self.__pose_tracking)
        self.__pub_pos_only.publish(
            self.__control_mode in ('position', 'velocity', 'fixed_front', 'fixed_top'))

    def node_shutdown(self):
        rospy.loginfo_once(
            f'/{self.ROBOT_NAME}/teleoperation: shutting down.')


def main():
    rospy.init_node('teleoperation', log_level=rospy.INFO)
    rospy.loginfo('\n\n\n\n\n')

    rn   = rospy.get_param(f'{rospy.get_name()}/robot_name',                      'my_gen3')
    co   = rospy.get_param(f'{rospy.get_name()}/compensate_orientation',          True)
    mpc  = rospy.get_param(f'{rospy.get_name()}/maximum_input_position_change',   0.1)
    moc  = rospy.get_param(f'{rospy.get_name()}/maximum_input_orientation_change',30)
    cc   = literal_eval(rospy.get_param(
        f'{rospy.get_name()}/convenience_compensation', '[0.0, 0.0, 0.0]'))
    freq = rospy.get_param(f'{rospy.get_name()}/node_frequency', 1000)
    # 'base_link' (default): input_pose in global frame; TF transform applied.
    # 'arm_local': input_pose already in {robot_name}/base_link; no TF.
    #   Requires the joystick mapping to read current_ee_pose_local and the
    #   teleoperation launch file to set this parameter.
    ipf  = rospy.get_param(f'{rospy.get_name()}/input_pose_frame', 'base_link')

    node = KinovaTeleoperation(
        robot_name=rn,
        compensate_orientation=co,
        convenience_compensation=cc,
        maximum_input_position_change=mpc,
        maximum_input_orientation_change=moc,
        input_pose_frame=ipf,
    )
    rospy.on_shutdown(node.node_shutdown)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        node.main_loop()
        rate.sleep()


if __name__ == '__main__':
    main()