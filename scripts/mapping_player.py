#!/usr/bin/env python
"""ROS Mapping Player — executes a .mapping.json node graph.

The mapping file contains ALL control logic for an interface.
This file is a pure graph executor — it has no robot-specific knowledge.

Named output ports
------------------
Connections specify both from_port and to_port.  When a source node
returns a dict, from_port indexes into it (e.g. tracking_mutex outputs
{'right_btn':bool, 'left_btn':bool, 'right_owns':bool, 'left_owns':bool}
and downstream nodes connect to specific named ports).
Single-output nodes return a scalar/dict directly and use from_port='out'.

Node types
----------
  input            subscribe to a ROS topic → dict each tick
                   params:
                     stale_timeout        seconds without msg before None (default 0.5, 0=disable)
                     reconnect_blanking   seconds to suppress output after reconnect (default 0.3)
                                          Prevents spurious rising edges when VR headset is
                                          put on/off.  Set 0 to disable.
  output           publish to a ROS topic
                   supported msg_types: std_msgs/Bool, Float32, Float64, String
                                        geometry_msgs/Pose, geometry_msgs/Twist
  constant         fixed value (float | bool | string | quat preset)
  extract          dotted-path field from dict
  math             scalar: multiply, add, subtract, negate, clamp, abs, deadzone,
                   scale_deadzone
                   subtract: v = in - in2   (in2 port, dynamic)
                   deadzone param zeros input when abs(v) < deadzone before applying op
  logic            bool: and, or, not, rising_edge, falling_edge, toggle
  compare          scalar/string threshold → bool (eq supports strings)
                   in2 port overrides static threshold with a dynamic value
  gate             pass 'in' when 'enable' is True, else emit default
  select           choose port 'a' or 'b' based on bool 'select'
  decompose_pose   Pose dict → named scalars: pos_x/y/z ori_w/x/y/z
  build_pose       named scalars → Pose dict
  build_twist      named scalars → Twist dict
                   inputs: linear_x/y/z  angular_x/y/z
                   output: out (dict consumed by output node with msg_type geometry_msgs/Twist)
  quaternion_mult  q_a * q_b  (dict or scalar ports a_w/x/y/z b_w/x/y/z)
  integrate_pos    scalar rate/s → accumulated float (with clamp)
  integrate_quat   roll/pitch/yaw_rate → orientation; outputs w/x/y/z + 'out'
                   seed_w/x/y/z ports: when reset fires, seed quaternion from
                   these ports (e.g. current EE orientation) instead of identity.
  floor_constraint enforce a minimum-Z motion floor; two modes:
                   mode='velocity' (default): blocks negative 'in' when ee_z <= floor_z,
                     allowing recovery upward; use for joystick/rate-based control.
                   mode='position': clamps 'in' to max(in, floor_z); use for direct
                     pose control where 'in' is a Z coordinate.
                   Inputs: in (value), floor_z (threshold), ee_z (current height,
                           only used in velocity mode)
                   Output: out
  state_machine    FSM driven by rising edges on named trigger ports
  tracking_mutex   two grip booleans → exclusive tracking passthrough
                   outputs: right_btn, left_btn, right_owns, left_owns
  ros_service      call a ROS service on rising edge of 'trigger' port
  pose_selector    scrollable pose menu that calls preset_poses services directly
                   inputs: open_menu, next, prev, confirm
                   outputs: is_open, selected_idx, pose_name, confirmed

Topic substitutions
-------------------
  {robot_name}       resolved from node's robot_id in meta.robots
  {controller_side}  resolved from node's robot_id in meta.robots

Hot-swap
--------
Publishing a std_msgs/String containing a new file path to
{node_name}/file_path tears down all current subscribers/publishers and
loads that file instead, without restarting the node.

Usage
-----
  rosrun <pkg> mapping_player.py _mapping_file:=/path/to/file.mapping.json
"""

import rospy, json, numpy as np, os, threading
from std_msgs.msg import Bool, Float32, Float64, String
from geometry_msgs.msg import Pose, Twist


# Period between aggregated "input topic has no data" warnings, in seconds.
SILENT_INPUT_REPORT_PERIOD_S = 15.0

# Upper bound on a single preset-pose request, in seconds.  Planning and
# execution are both bounded in normal operation, so this only fires when
# move_group stops answering.  It exists so the arm is always resumed rather
# than left paused indefinitely.
PRESET_POSE_TIMEOUT_S = 120.0


# ─────────────────────────────────────────────────────────────────────────────
# Quaternion helpers
# ─────────────────────────────────────────────────────────────────────────────

def _qmul(a, b):
    w1,x1,y1,z1=a; w2,x2,y2,z2=b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])

def _qnorm(q):
    n=np.linalg.norm(q); return q/n if n>1e-9 else np.array([1.,0.,0.,0.])

def _qaxis(axis, rad):
    ax=np.asarray(axis,float); n=np.linalg.norm(ax)
    if n<1e-9: return np.array([1.,0.,0.,0.])
    ax/=n; s=np.sin(rad/2)
    return np.array([np.cos(rad/2),ax[0]*s,ax[1]*s,ax[2]*s])

_QUAT_PRESETS = {
    'rz180':   _qnorm(_qaxis([0,0,1],np.pi)),
    'rz90':    _qnorm(_qaxis([0,0,1],np.pi/2)),
    'rx180':   _qnorm(_qaxis([1,0,0],np.pi)),
    'ry180':   _qnorm(_qaxis([0,1,0],np.pi)),
    'identity':np.array([1.,0.,0.,0.]),
}
def _q2d(q): return {'w':float(q[0]),'x':float(q[1]),'y':float(q[2]),'z':float(q[3])}
def _d2q(d): return np.array([d.get('w',1.),d.get('x',0.),d.get('y',0.),d.get('z',0.)])


# ─────────────────────────────────────────────────────────────────────────────
# Scalar helpers
# ─────────────────────────────────────────────────────────────────────────────

def _f(v, d=0.):
    try: return float(v)
    except: return d

def _b(v):
    if isinstance(v, bool): return v
    if isinstance(v,(int,float)): return bool(v)
    return False

def _deep_get(d, path):
    cur=d
    for p in path.split('.'):
        if isinstance(cur,dict): cur=cur.get(p)
        else: return None
    return cur

def _res(tmpl, subs):
    for k,v in subs.items(): tmpl=tmpl.replace('{'+k+'}',str(v))
    return tmpl

def _msg_cls(mt):
    if '/' not in mt: return None
    pkg,name=mt.split('/',1)
    try:
        if pkg=='std_msgs':       import std_msgs.msg as m
        elif pkg=='geometry_msgs':import geometry_msgs.msg as m
        elif pkg=='sensor_msgs':  import sensor_msgs.msg as m
        elif pkg=='oculus_ros':
            try: import oculus_ros.msg as m
            except ImportError:
                rospy.logwarn_throttle(30,f'[player] oculus_ros unavailable, shim: {mt}')
                return None
        else:
            rospy.logwarn_throttle(30,f'[player] unknown pkg: {pkg}')
            return None
        return getattr(m,name,None)
    except: return None

def _to_dict(msg):
    if msg is None: return {}
    if isinstance(msg,dict): return msg
    out={}
    for s in getattr(msg,'__slots__',[]):
        v=getattr(msg,s); out[s]=_to_dict(v) if hasattr(v,'__slots__') else v
    return out


# ─────────────────────────────────────────────────────────────────────────────
# Node base
# ─────────────────────────────────────────────────────────────────────────────

class _N:
    def __init__(self,spec): self.id=spec['id']; self.spec=spec; self._v=None
    def compute(self,inp): raise NotImplementedError
    def val(self): return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Input
# ─────────────────────────────────────────────────────────────────────────────

class InputNode(_N):
    """Subscribe to a ROS topic and expose its fields each tick.

    stale_timeout (float, default 0.5 s)
        Seconds without a new message before compute() returns None.
        Set to 0 to disable.

    reconnect_blanking (float, default 0.3 s)
        When a topic transitions from stale back to active (i.e. the first
        message arrives after a stale window), suppress output (return None)
        for this many seconds.

        Why: VR headsets publish a burst of potentially incorrect button
        states when put on or taken off.  Returning None during the blanking
        window prevents downstream rising-edge / toggle nodes from seeing
        spurious transitions.  During the window _prev in LogicNodes is
        set to False (from None), so when real data resumes and the user is
        NOT pressing a button the system behaves correctly.  If the user IS
        holding a button when the window closes they will get one rising-edge
        detection — this is intentional (they are actively pressing).
    """

    def __init__(self, spec):
        super().__init__(spec)
        self._d                = None
        self._cls              = _msg_cls(spec['msg_type'])
        self._sub              = None
        self._got_first        = False
        self._last_msg_time    = None

        self._stale_timeout      = float(spec.get('stale_timeout',    0.5))
        self._reconnect_blanking = float(spec.get('reconnect_blanking', 0.3))

        # Set when the topic transitions stale → active.
        # Output stays None until rospy.Time.now() >= this value.
        self._reconnect_until = None
        self._was_stale       = False   # True while (or just after) stale

    def _on_msg(self, m):
        now = rospy.Time.now().to_sec()

        # Detect stale → active transition
        if (self._stale_timeout > 0
                and self._last_msg_time is not None
                and now - self._last_msg_time > self._stale_timeout):
            # Topic was stale; arm the blanking window
            if self._reconnect_blanking > 0:
                self._reconnect_until = now + self._reconnect_blanking

        self._d             = _to_dict(m)
        self._last_msg_time = now

        self._got_first = True

    def register(self, topic):
        if self._cls:
            self._sub = rospy.Subscriber(topic, self._cls, self._on_msg, queue_size=1)
            rospy.loginfo(f'[player] SUB  {topic}  ({self.spec["msg_type"]})')
        else:
            rospy.logwarn(
                f'[player] {self.id}: unknown msg_type "{self.spec["msg_type"]}" '
                f'— cannot subscribe to {topic}.  '
                f'Check msg_type in the mapping JSON.')

    def unregister(self):
        if self._sub is not None:
            try: self._sub.unregister()
            except Exception: pass

    @property
    def has_data(self):
        """True once at least one message has arrived on this input's topic."""
        return self._got_first

    def compute(self, _):
        now = rospy.Time.now().to_sec()

        # Staleness: no message received recently
        if (self._stale_timeout > 0
                and self._last_msg_time is not None
                and now - self._last_msg_time > self._stale_timeout):
            self._v = None
            return self._v

        # Reconnect blanking: suppress output to prevent spurious edges
        if self._reconnect_until is not None:
            if now < self._reconnect_until:
                self._v = None
                return self._v
            else:
                self._reconnect_until = None   # blanking window expired

        self._v = self._d
        return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Output
# ─────────────────────────────────────────────────────────────────────────────

class OutputNode(_N):
    def __init__(self,spec):
        super().__init__(spec)
        self._mt=spec['msg_type']; self._cls=_msg_cls(self._mt); self._pub=None
    def register(self,topic):
        if self._cls:
            self._pub=rospy.Publisher(topic,self._cls,queue_size=1)
            rospy.loginfo(f'[player] PUB  {topic}  ({self._mt})')
        else: rospy.logwarn(f'[player] {self.id} no cls ({topic})')
    def unregister(self):
        if self._pub is not None:
            try: self._pub.unregister()
            except Exception: pass
    def compute(self,inp):
        v=inp.get('in')
        if v is None or self._pub is None:
            rospy.logwarn_throttle(5,f'[player] {self.id}: not publishing — '
                                   f'{"no publisher" if self._pub is None else "input is None (check upstream input node)"}')
            return
        msg=self._build(v)
        if msg is not None: self._pub.publish(msg)
    def _build(self,v):
        _,n=self._mt.split('/',1)
        if n=='Bool':   m=Bool();   m.data=_b(v);           return m
        if n=='Float32':m=Float32();m.data=float(_f(v));     return m
        if n=='Float64':m=Float64();m.data=float(_f(v));     return m
        if n=='String': m=String(); m.data=str(v) if v else '';return m
        if n=='Pose':
            if not isinstance(v,dict): return None
            if 'pose' in v and isinstance(v['pose'],dict): v=v['pose']
            m=Pose()
            pos=v.get('position',{})
            if isinstance(pos,dict):
                m.position.x=_f(pos.get('x',0.));m.position.y=_f(pos.get('y',0.))
                m.position.z=_f(pos.get('z',0.))
            ori=v.get('orientation',{})
            if isinstance(ori,dict):
                m.orientation.w=_f(ori.get('w',1.));m.orientation.x=_f(ori.get('x',0.))
                m.orientation.y=_f(ori.get('y',0.));m.orientation.z=_f(ori.get('z',0.))
            else: m.orientation.w=1.
            rospy.logdebug_throttle(2,
                f'[player] {self.id}: publishing pose '
                f'pos=({m.position.x:.3f},{m.position.y:.3f},{m.position.z:.3f}) '
                f'ori=({m.orientation.w:.3f},{m.orientation.x:.3f},'
                f'{m.orientation.y:.3f},{m.orientation.z:.3f})')
            return m
        if n=='Twist':
            if not isinstance(v,dict): return None
            m=Twist()
            lin=v.get('linear',{});  ang=v.get('angular',{})
            if isinstance(lin,dict):
                m.linear.x=_f(lin.get('x',0.));m.linear.y=_f(lin.get('y',0.))
                m.linear.z=_f(lin.get('z',0.))
            if isinstance(ang,dict):
                m.angular.x=_f(ang.get('x',0.));m.angular.y=_f(ang.get('y',0.))
                m.angular.z=_f(ang.get('z',0.))
            return m
        rospy.logwarn_throttle(30,f'[player] unsupported type {self._mt}')
        return None


# ─────────────────────────────────────────────────────────────────────────────
# Constant
# ─────────────────────────────────────────────────────────────────────────────

class ConstantNode(_N):
    def __init__(self,spec):
        super().__init__(spec)
        vt=spec.get('value_type','float'); raw=spec.get('value',0)
        if vt=='bool': self._c=bool(raw)
        elif vt=='string':
            s=str(raw).strip(); self._c=_q2d(_QUAT_PRESETS[s]) if s in _QUAT_PRESETS else s
        else: self._c=_f(raw)
    def compute(self,_): self._v=self._c; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Extract
# ─────────────────────────────────────────────────────────────────────────────

class ExtractNode(_N):
    def __init__(self,spec): super().__init__(spec); self._field=spec.get('field','')
    def compute(self,inp):
        up=inp.get('in') or {}
        self._v=_deep_get(up,self._field) if isinstance(up,dict) else None
        return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Math
# ─────────────────────────────────────────────────────────────────────────────

class MathNode(_N):
    """Scalar math on a single 'in' port (and optional 'in2' for subtract).

    Operations
    ----------
    multiply        v *= factor
    add             v += addend
    subtract        v -= in2  (in2 port provides the second operand dynamically)
    negate          v = -v
    clamp           v = clip(v, min, max)
    abs             v = abs(v)
    deadzone        (pre-op) zero v when |v| < deadzone
    scale_deadzone  (pre-op) remap from deadzone..1 to 0..1 preserving sign
    """
    def __init__(self,spec): super().__init__(spec); self._op=spec.get('operation','multiply'); self._p=spec.get('params',{})
    def compute(self,inp):
        v=_f(inp.get('in',0.)); dz=_f(self._p.get('deadzone',0.))
        if dz>0:
            if abs(v)<dz: v=0.
            elif self._op=='scale_deadzone': v=(abs(v)-dz)/(1.-dz)*np.sign(v)
        op=self._op
        if op=='multiply':     v*=_f(self._p.get('factor',1.))
        elif op=='add':        v+=_f(self._p.get('addend',0.))
        elif op=='subtract':   v-=_f(inp.get('in2',0.))
        elif op=='negate':     v=-v
        elif op=='clamp':      v=float(np.clip(v,_f(self._p.get('min',-1.)),_f(self._p.get('max',1.))))
        elif op=='abs':        v=abs(v)
        self._v=v; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Logic
# ─────────────────────────────────────────────────────────────────────────────

class LogicNode(_N):
    def __init__(self,spec):
        super().__init__(spec)
        self._op=spec.get('operation','not'); self._prev=False; self._tog=False
        # debounce_ticks: for rising_edge/falling_edge, require the signal to
        # be True/False for this many consecutive ticks before firing.
        # Filters out 1-2 tick SDK glitches from OVR headset proximity transitions.
        # 0 = disabled (original rising_edge behaviour).
        self._debounce=int(spec.get('params',{}).get('debounce_ticks',0))
        self._count=0
    def compute(self,inp):
        a=_b(inp.get('in',False)); b=_b(inp.get('in2',False)); op=self._op
        # 'enable' port (default True): when False, edge/toggle ops track _prev
        # without firing.  _prev is ALWAYS updated from the live 'in' signal so
        # that when enable returns True the node has no accumulated
        # false-transition debt.
        enabled=_b(inp.get('enable',True))
        if op=='and':            r=a and b
        elif op=='or':           r=a or b
        elif op=='not':          r=not a
        elif op=='rising_edge':
            if self._debounce > 0:
                if a and enabled:
                    self._count+=1
                else:
                    self._count=0
                r=(self._count==self._debounce)
            else:
                r=(a and not self._prev) if enabled else False
        elif op=='falling_edge':
            if self._debounce > 0:
                if not a and enabled:
                    self._count+=1
                else:
                    self._count=0
                r=(self._count==self._debounce)
            else:
                r=(not a and self._prev) if enabled else False
        elif op=='toggle':
            if _b(inp.get('reset', False)):
                self._tog=False
            elif a and not self._prev and enabled:
                self._tog=not self._tog
            r=self._tog
        else: r=a
        self._prev=a; self._v=r; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Compare
# ─────────────────────────────────────────────────────────────────────────────

class CompareNode(_N):
    """Compare 'in' against a threshold.

    The threshold can be set statically via params.threshold, or dynamically
    via the 'in2' port.  When both are present, 'in2' takes priority.
    This allows floor_z or other runtime values to act as dynamic thresholds.
    """
    def __init__(self,spec): super().__init__(spec); self._op=spec.get('operation','gt'); self._p=spec.get('params',{})
    def compute(self,inp):
        raw=inp.get('in',0.)
        # Dynamic threshold via in2 port overrides static params.threshold
        thr = inp['in2'] if 'in2' in inp and inp['in2'] is not None else self._p.get('threshold',0.5)
        if self._op=='eq':
            if isinstance(raw,str) or isinstance(thr,str): r=str(raw).strip()==str(thr).strip()
            else: r=abs(_f(raw)-_f(thr))<_f(self._p.get('tolerance',1e-6))
        else:
            v=_f(raw); t=_f(thr)
            if self._op=='gt':  r=v>t
            elif self._op=='lt':r=v<t
            elif self._op=='gte':r=v>=t
            elif self._op=='lte':r=v<=t
            else: r=False
        self._v=r; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Gate
# ─────────────────────────────────────────────────────────────────────────────

class GateNode(_N):
    def __init__(self,spec):
        super().__init__(spec)
        raw=spec.get('params',{}).get('default',None)
        self._def=0. if raw=='zero' else (False if raw=='false' else raw)
    def compute(self,inp):
        self._v=inp.get('in') if _b(inp.get('enable',False)) else self._def
        return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Select
# ─────────────────────────────────────────────────────────────────────────────

class SelectNode(_N):
    def compute(self,inp):
        self._v=inp.get('b') if _b(inp.get('select',False)) else inp.get('a')
        return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Pose nodes
# ─────────────────────────────────────────────────────────────────────────────

class DecomposePoseNode(_N):
    def compute(self,inp):
        p=inp.get('in') or {}
        pos=p.get('position',{}) if isinstance(p,dict) else {}
        ori=p.get('orientation',{}) if isinstance(p,dict) else {}
        self._v={'pos_x':_f(pos.get('x',0.)),'pos_y':_f(pos.get('y',0.)),'pos_z':_f(pos.get('z',0.)),
                 'ori_w':_f(ori.get('w',1.)),'ori_x':_f(ori.get('x',0.)),'ori_y':_f(ori.get('y',0.)),'ori_z':_f(ori.get('z',0.))}
        return self._v

class BuildPoseNode(_N):
    def compute(self,inp):
        self._v={
            'position':    {'x':_f(inp.get('pos_x',0.)),'y':_f(inp.get('pos_y',0.)),'z':_f(inp.get('pos_z',0.))},
            'orientation': {'w':_f(inp.get('ori_w',1.)),'x':_f(inp.get('ori_x',0.)),'y':_f(inp.get('ori_y',0.)),'z':_f(inp.get('ori_z',0.))},
        }; return self._v

class BuildTwistNode(_N):
    """Assemble a Twist dict from named scalar ports.

    Input ports
    -----------
    linear_x / linear_y / linear_z    (default 0.0)
    angular_x / angular_y / angular_z (default 0.0)

    Output
    ------
    out  dict — consumed by OutputNode with msg_type geometry_msgs/Twist
    """
    def compute(self,inp):
        self._v={
            'linear':  {'x':_f(inp.get('linear_x', 0.)),'y':_f(inp.get('linear_y', 0.)),'z':_f(inp.get('linear_z', 0.))},
            'angular': {'x':_f(inp.get('angular_x',0.)),'y':_f(inp.get('angular_y',0.)),'z':_f(inp.get('angular_z',0.))},
        }; return self._v

class QuaternionMultNode(_N):
    def _rq(self,inp,pf):
        d=inp.get(pf)
        if isinstance(d,dict) and set('wxyz').issubset(d): return _d2q(d)
        w=_f(inp.get(f'{pf}_w',1. if pf=='a' else 0.))
        return np.array([w,_f(inp.get(f'{pf}_x',0.)),_f(inp.get(f'{pf}_y',0.)),_f(inp.get(f'{pf}_z',0.))])
    def compute(self,inp):
        self._v=_q2d(_qnorm(_qmul(self._rq(inp,'a'),self._rq(inp,'b')))); return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Integrators
# ─────────────────────────────────────────────────────────────────────────────

class IntegratePosNode(_N):
    def __init__(self,spec,dt):
        super().__init__(spec); self._dt=dt; p=spec.get('params',{})
        self._init=_f(p.get('init_value',0.))
        self._min=_f(p.get('min',-1e9)); self._max=_f(p.get('max',1e9))
        self._acc=self._init; self._seeded=False
    def compute(self,inp):
        seed=inp.get('seed', None)
        if not self._seeded and seed is not None:
            self._acc=_f(seed); self._seeded=True
        if _b(inp.get('reset',False)):
            self._acc=_f(seed) if seed is not None else self._init
            self._seeded=(seed is not None)
        self._acc+=_f(inp.get('in',0.))*self._dt
        self._acc=float(np.clip(self._acc,self._min,self._max))
        self._v=self._acc; return self._v


class IntegrateQuatNode(_N):
    """Integrate angular rates into a quaternion orientation.

    Input ports
    -----------
    roll_rate, pitch_rate, yaw_rate  (rad/s, default 0)
    reset                            rising bool: reset to seed (or identity)
    seed_w, seed_x, seed_y, seed_z  optional quaternion seed applied on reset.
                                     Connect to the current EE orientation so
                                     the integrator starts from the arm's actual
                                     pose instead of identity.  If the seed
                                     quaternion is all-zero or not provided,
                                     resets to identity [1,0,0,0].

    Outputs
    -------
    out  dict {w,x,y,z}
    w, x, y, z  individual scalar ports
    """
    def __init__(self, spec, dt):
        super().__init__(spec)
        self._dt = dt
        self._q  = np.array([1., 0., 0., 0.])

    def compute(self, inp):
        if _b(inp.get('reset', False)):
            sw = _f(inp.get('seed_w', 0.))
            sx = _f(inp.get('seed_x', 0.))
            sy = _f(inp.get('seed_y', 0.))
            sz = _f(inp.get('seed_z', 0.))
            seed_q = np.array([sw, sx, sy, sz])
            n = np.linalg.norm(seed_q)
            if n > 1e-9:
                self._q = seed_q / n
            else:
                self._q = np.array([1., 0., 0., 0.])

        rr = _f(inp.get('roll_rate',  0.)) * self._dt
        pr = _f(inp.get('pitch_rate', 0.)) * self._dt
        yr = _f(inp.get('yaw_rate',   0.)) * self._dt

        dq = _qmul(_qmul(_qaxis([0, 0, 1], yr),
                          _qaxis([0, 1, 0], pr)),
                   _qaxis([1, 0, 0], rr))
        self._q = _qnorm(_qmul(self._q, dq))

        d = _q2d(self._q)
        self._v = {'out': d, 'w': d['w'], 'x': d['x'], 'y': d['y'], 'z': d['z']}
        return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Floor constraint
# ─────────────────────────────────────────────────────────────────────────────

class FloorConstraintNode(_N):
    """Enforce a minimum-Z motion floor.

    Two modes controlled by the 'mode' field in the node spec:

    mode='velocity'  (default)
        Blocks negative 'in' (downward velocity / rate) when ee_z <= floor_z.
        Upward motion is always allowed so the arm can escape if it somehow
        ends up below the floor.
        Use this for joystick / rate-based control (integrate_pos pipeline).

        Inputs: in (velocity), ee_z (current EE height), floor_z (threshold)

    mode='position'
        Clamps 'in' so it is never less than floor_z: out = max(in, floor_z).
        Use this for direct pose control (pose mapping pipeline) where 'in'
        is a Z coordinate being sent directly to the arm.

        Inputs: in (z position), floor_z (threshold)
        (ee_z is ignored in position mode)

    In both modes 'floor_z' may be driven from a dynamic input node (e.g. a
    std_msgs/Float64 topic) so the floor can be changed at runtime without
    reloading the mapping.
    """

    def __init__(self, spec):
        super().__init__(spec)
        self._mode = spec.get('mode', 'velocity')

    def compute(self, inp):
        value   = _f(inp.get('in',      0.))
        # Default to -inf so an unconnected / not-yet-published floor_z port
        # never spuriously activates the constraint (max(v, -inf) == v).
        floor_z = _f(inp.get('floor_z'), float('-inf'))
        ee_z    = _f(inp.get('ee_z',    0.))   # only meaningful in velocity mode

        if self._mode == 'velocity':
            # Block downward velocity when already at / below floor
            if ee_z <= floor_z and value < 0:
                value = 0.
        else:  # position
            value = max(value, floor_z)

        self._v = value
        return self._v


# ─────────────────────────────────────────────────────────────────────────────
# State machine
# ─────────────────────────────────────────────────────────────────────────────

class StateMachineNode(_N):
    def __init__(self,spec):
        super().__init__(spec); self._state=spec.get('initial_state','idle')
        self._trans=spec.get('transitions',[]); self._prev={}
    def compute(self,inp):
        # 'ctrl_enable' port (default True): when False, _prev is updated but
        # no transitions fire.  Prevents spurious arm-state changes when
        # controllers go offline (headset on/off) and reconnect.
        enabled=_b(inp.get('ctrl_enable',True))
        for tr in self._trans:
            if not enabled: break
            if tr.get('from')!=self._state: continue
            port=tr.get('trigger_port','')
            if _b(inp.get(port,False)) and not self._prev.get(port,False):
                self._state=tr.get('to',self._state); break
        for t in self._trans: self._prev[t['trigger_port']]=_b(inp.get(t['trigger_port'],False))
        self._v=self._state; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Tracking mutex
# ─────────────────────────────────────────────────────────────────────────────

class TrackingMutexNode(_N):
    def __init__(self,spec):
        super().__init__(spec); self._owner='right'; self._pr=False; self._pl=False
    def compute(self,inp):
        r=_b(inp.get('right',False)); l=_b(inp.get('left',False))
        if r and not self._pr and self._owner!='right':
            self._owner='right'; rospy.loginfo('[player] mutex → right')
        elif l and not self._pl and self._owner!='left':
            self._owner='left'; rospy.loginfo('[player] mutex → left')
        self._pr=r; self._pl=l
        self._v={
            'right_btn':  r if self._owner=='right' else False,
            'left_btn':   l if self._owner=='left'  else False,
            'right_owns': self._owner=='right',
            'left_owns':  self._owner=='left',
        }; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# ROS service
# ─────────────────────────────────────────────────────────────────────────────

class RosServiceNode(_N):
    def __init__(self,spec):
        super().__init__(spec)
        self._st_spec=spec.get('service_type','Empty')
        self._st=self._st_spec
        self._cv=spec.get('call_value',True)
        self._cof=spec.get('call_on_false',False)
        self._prev=False; self._proxy=None; self._topic=None
    def register(self,topic):
        self._topic=topic
        try:
            import rosservice, roslib.message
            type_str=rosservice.get_service_type(topic)
            if type_str:
                cls=roslib.message.get_service_class(type_str)
                if cls:
                    self._proxy=rospy.ServiceProxy(topic,cls)
                    self._st=type_str
                    rospy.loginfo(f'[player] SVC  {topic}  ({type_str})')
                    return
        except Exception as ex:
            rospy.logwarn(f'[player] {self.id}: dynamic resolution failed ({ex})')
        rospy.logwarn(f'[player] {self.id}: {topic} not available yet — will retry on first trigger')
    def _ensure_proxy(self):
        if self._proxy or not self._topic: return
        try:
            import rosservice, roslib.message
            type_str=rosservice.get_service_type(self._topic)
            if type_str:
                cls=roslib.message.get_service_class(type_str)
                if cls:
                    self._proxy=rospy.ServiceProxy(self._topic,cls)
                    self._st=type_str
                    rospy.loginfo(f'[player] SVC (late)  {self._topic}  ({type_str})')
        except Exception: pass
    def _call(self,val=None):
        self._ensure_proxy()
        if not self._proxy:
            rospy.logwarn(f'[player] {self.id}: cannot call {self._topic} — proxy not ready')
            return
        try:
            rospy.loginfo(f'[player] {self.id}: calling {self._topic}')
            if 'SetBool' in self._st:
                self._proxy(bool(val) if val is not None else bool(self._cv))
            else:
                self._proxy()
        except Exception as ex: rospy.logwarn_throttle(5,f'[player] {self.id}: {ex}')
    def compute(self,inp):
        trig=_b(inp.get('trigger',False))
        if trig and not self._prev:  self._call(self._cv)
        elif not trig and self._prev and self._cof: self._call(not bool(self._cv))
        self._prev=trig; self._v=trig; return self._v


# ─────────────────────────────────────────────────────────────────────────────
# Pose selector
# ─────────────────────────────────────────────────────────────────────────────

class PoseSelectorNode(_N):
    """Scrollable pose menu that directly calls preset_poses services.

    Inputs
    ------
    open_menu   bool  rising edge toggles menu open/closed
    next        bool  rising edge advances to next pose (wraps)
    prev        bool  rising edge goes to previous pose (wraps)
    confirm     bool  rising edge executes selected pose and closes menu

    Outputs
    -------
    is_open      bool   menu is currently open
    selected_idx float  zero-based index of highlighted pose
    pose_name    str    service name of highlighted pose
    confirmed    bool   True for one tick immediately after confirm fires
    """

    def __init__(self, spec):
        super().__init__(spec)
        self._poses      = spec.get('poses', [])
        self._robot_name = ''
        self._idx        = 0
        self._is_open    = False
        self._prev_open    = False
        self._prev_next    = False
        self._prev_prev    = False
        self._prev_confirm = False
        self._pub_open  = None
        self._pub_idx   = None
        self._pub_name  = None
        self._pub_cmd   = None

    def register(self, robot_name):
        if not robot_name:
            return
        self._robot_name = robot_name
        self._pub_open = rospy.Publisher(
            f'/{robot_name}/preset_poses/menu_open',
            Bool, queue_size=1, latch=True)
        self._pub_idx = rospy.Publisher(
            f'/{robot_name}/preset_poses/selected_pose_index',
            Float64, queue_size=1, latch=True)
        self._pub_name = rospy.Publisher(
            f'/{robot_name}/preset_poses/selected_pose_name',
            String, queue_size=1, latch=True)
        self._pub_cmd = rospy.Publisher(
            '/preset_pose_command', String, queue_size=10)
        rospy.loginfo(f'[player] POSE_SEL  /{robot_name}  ({len(self._poses)} poses)')

    def unregister(self):
        for pub in (self._pub_open, self._pub_idx, self._pub_name, self._pub_cmd):
            if pub is not None:
                try: pub.unregister()
                except Exception: pass

    def compute(self, inp):
        open_in    = _b(inp.get('open_menu', False))
        next_in    = _b(inp.get('next',      False))
        prev_in    = _b(inp.get('prev',      False))
        confirm_in = _b(inp.get('confirm',   False))

        confirmed_pulse = False
        n = len(self._poses)

        if open_in and not self._prev_open:
            self._is_open = not self._is_open
            rospy.loginfo(
                f'[pose_sel] {self._robot_name}: menu '
                f'{"opened" if self._is_open else "closed"} '
                f'(pose: {self._poses[self._idx] if n else "-"})')

        if self._is_open and n > 0:
            if next_in and not self._prev_next:
                self._idx = (self._idx + 1) % n
                rospy.loginfo(f'[pose_sel] {self._robot_name}: → {self._poses[self._idx]}')
            if prev_in and not self._prev_prev:
                self._idx = (self._idx - 1) % n
                rospy.loginfo(f'[pose_sel] {self._robot_name}: → {self._poses[self._idx]}')
            if confirm_in and not self._prev_confirm:
                self._execute_pose()
                self._is_open   = False
                confirmed_pulse = True

        self._prev_open    = open_in
        self._prev_next    = next_in
        self._prev_prev    = prev_in
        self._prev_confirm = confirm_in

        pose_name = self._poses[self._idx] if n > 0 else ''

        if self._pub_open  is not None: self._pub_open.publish(self._is_open)
        if self._pub_idx   is not None: self._pub_idx.publish(float(self._idx))
        if self._pub_name  is not None: self._pub_name.publish(pose_name)

        self._v = {
            'is_open':      self._is_open,
            'selected_idx': float(self._idx),
            'pose_name':    pose_name,
            'confirmed':    confirmed_pulse,
        }
        return self._v

    def _execute_pose(self):
        """Hand the selected pose to the shared /preset_pose_command handler.

        Publishing rather than calling the services directly keeps the
        pause / move / resume sequence in one place, and that handler runs it on
        its own thread.  Calling the blocking pose service from compute() would
        stall the whole graph for the duration of the motion, freezing the base,
        the chest and the other arm.
        """

        if not self._poses or not self._robot_name or self._pub_cmd is None:
            return

        pose_name = self._poses[self._idx]
        rospy.loginfo(f'[pose_sel] {self._robot_name}: requesting {pose_name}')
        self._pub_cmd.publish(f'{self._robot_name}/{pose_name}')


# ─────────────────────────────────────────────────────────────────────────────
# Factory
# ─────────────────────────────────────────────────────────────────────────────

FACTORIES = {
    'input':             lambda s,dt: InputNode(s),
    'output':            lambda s,dt: OutputNode(s),
    'constant':          lambda s,dt: ConstantNode(s),
    'extract':           lambda s,dt: ExtractNode(s),
    'math':              lambda s,dt: MathNode(s),
    'logic':             lambda s,dt: LogicNode(s),
    'compare':           lambda s,dt: CompareNode(s),
    'gate':              lambda s,dt: GateNode(s),
    'select':            lambda s,dt: SelectNode(s),
    'decompose_pose':    lambda s,dt: DecomposePoseNode(s),
    'build_pose':        lambda s,dt: BuildPoseNode(s),
    'build_twist':       lambda s,dt: BuildTwistNode(s),
    'quaternion_mult':   lambda s,dt: QuaternionMultNode(s),
    'integrate_pos':     lambda s,dt: IntegratePosNode(s,dt),
    'integrate_quat':    lambda s,dt: IntegrateQuatNode(s,dt),
    'floor_constraint':  lambda s,dt: FloorConstraintNode(s),
    'state_machine':     lambda s,dt: StateMachineNode(s),
    'tracking_mutex':    lambda s,dt: TrackingMutexNode(s),
    'ros_service':       lambda s,dt: RosServiceNode(s),
    'pose_selector':     lambda s,dt: PoseSelectorNode(s),
}


# ─────────────────────────────────────────────────────────────────────────────
# Graph executor
# ─────────────────────────────────────────────────────────────────────────────

class MappingPlayer:

    def __init__(self, mapping_file, overrides, node_frequency):
        self._overrides      = dict(overrides)
        self._base_frequency = node_frequency
        self._lock = threading.Lock()
        self.current_file = None
        self._last_silent_report = rospy.Time.now().to_sec()
        self._load(mapping_file)

    def reload(self, mapping_file):
        if not os.path.isfile(mapping_file):
            rospy.logerr(f'[player] reload: file not found: {mapping_file}')
            return False
        with self._lock:
            old_nodes = dict(self._nodes)
            try:
                self._teardown(old_nodes)
                self._load(mapping_file)
                rospy.loginfo(f'\033[92m[player] reloaded: {mapping_file}\033[0m')
                return True
            except Exception as ex:
                rospy.logerr(f'[player] reload failed, file_path={mapping_file}: {ex}')
                return False

    def _teardown(self, node_dict):
        for node in node_dict.values():
            unreg = getattr(node, 'unregister', None)
            if unreg is not None:
                try: unreg()
                except Exception: pass

    def _load(self, mapping_file):
        with open(mapping_file) as f:
            spec = json.load(f)
        meta = spec.get('meta', {})

        self.node_frequency = self._base_frequency or meta.get('node_frequency', 100)
        dt = 1.0 / max(self.node_frequency, 1)

        robots_raw = meta.get('robots', None)
        if robots_raw is None:
            robots_raw = [{'robot_id':'robot',
                           'robot_name':      self._overrides.get('robot_name',      meta.get('robot_name','my_gen3')),
                           'controller_side': self._overrides.get('controller_side', meta.get('controller_side','right'))}]
        self._robots = list(robots_raw)
        if len(self._robots)==1:
            if 'robot_name'      in self._overrides: self._robots[0]['robot_name']      = self._overrides['robot_name']
            if 'controller_side' in self._overrides: self._robots[0]['controller_side'] = self._overrides['controller_side']

        self._gsubs={}
        for r in self._robots:
            if r.get('controller_side')=='left':  self._gsubs['left_robot']  = r['robot_name']
            elif r.get('controller_side')=='right':self._gsubs['right_robot'] = r['robot_name']

        rospy.loginfo(f'[player] {meta.get("name","unnamed")}  ({mapping_file})')
        for r in self._robots:
            rospy.loginfo(f'  {r.get("robot_id")}  name={r.get("robot_name")}  side={r.get("controller_side")}')

        self._nodes      = {}
        self._inputs_for = {}
        self._topo       = []

        self._build(spec, dt)
        self._topo = self._sort()
        self.current_file = mapping_file

    def _subs_for(self, robot_id):
        subs = dict(self._gsubs)
        for r in self._robots:
            if r.get('robot_id') == robot_id:
                subs.update(r); return subs
        if self._robots: subs.update(self._robots[0])
        return subs

    def _resolve(self, tmpl, spec):
        return _res(tmpl, self._subs_for(spec.get('robot_id','')))

    def _build(self, spec, dt):
        for s in spec.get('nodes', []):
            ntype = s.get('type')
            if not ntype:
                continue
            factory = FACTORIES.get(ntype)
            if factory is None:
                rospy.logwarn(f'[player] unknown node type: {ntype} ({s["id"]})')
                continue
            node = factory(s, dt)
            if ntype == 'input':
                node.register(self._resolve(s['topic'], s))
            elif ntype == 'output':
                node.register(self._resolve(s['topic'], s))
            elif ntype == 'ros_service':
                node.register(self._resolve(s.get('service_topic',''), s))
            elif ntype == 'pose_selector':
                direct = s.get('robot_name', '')
                if direct:
                    node.register(direct)
                else:
                    subs = self._subs_for(s.get('robot_id', ''))
                    node.register(subs.get('robot_name', ''))
            self._nodes[s['id']] = node

        self._inputs_for = {nid: {} for nid in self._nodes}
        for e in spec.get('connections', []):
            src=e.get('from'); dst=e.get('to')
            if not src or not dst:
                continue
            dp=e.get('to_port','in'); sp=e.get('from_port','out')
            if src in self._nodes and dst in self._nodes:
                self._inputs_for[dst][dp] = (src, sp)

    def _sort(self):
        indeg={n:0 for n in self._nodes}; adj={n:[] for n in self._nodes}
        for nid, ports in self._inputs_for.items():
            for _,(src,_sp) in ports.items():
                adj[src].append(nid); indeg[nid]+=1
        q=[n for n,d in indeg.items() if d==0]; order=[]
        while q:
            n=q.pop(0); order.append(n)
            for m in adj[n]:
                indeg[m]-=1
                if indeg[m]==0: q.append(m)
        missing = [n for n in self._nodes if n not in order]
        if missing:
            rospy.logwarn(f'[player] cycle detected — nodes may execute out of order: {missing}')
            order.extend(missing)
        return order

    def execute(self):
        with self._lock:
            computed = {}
            deferred_svc = []

            for nid in self._topo:
                node = self._nodes[nid]
                inputs = {}
                for dport, (src_id, sport) in self._inputs_for[nid].items():
                    sv = computed.get(src_id)
                    if isinstance(sv, dict) and sport != 'out':
                        inputs[dport] = sv.get(sport, sv)
                    else:
                        inputs[dport] = sv

                if isinstance(node, RosServiceNode):
                    deferred_svc.append((nid, inputs))
                    computed[nid] = node.val()
                else:
                    computed[nid] = node.compute(inputs)

            for nid, inputs in deferred_svc:
                computed[nid] = self._nodes[nid].compute(inputs)

            self._report_silent_inputs()

    def _report_silent_inputs(self):
        """Warn periodically about input topics that have never received a message.

        Emitted as one aggregated line rather than one line per input: a mapping's
        inputs mostly come from the same few sources (headset, controllers), so they
        go silent together and per-input warnings arrive in bursts that bury the rest
        of the log.  Caller must already hold self._lock.
        """

        now = rospy.Time.now().to_sec()
        if now - self._last_silent_report < SILENT_INPUT_REPORT_PERIOD_S:
            return
        self._last_silent_report = now

        inputs = [node for node in self._nodes.values() if isinstance(node, InputNode)]
        silent = [node.id for node in inputs if not node.has_data]
        if not silent:
            return

        rospy.logwarn(
            f'[player] no data yet on {len(silent)} of {len(inputs)} inputs: '
            f'{", ".join(silent)} '
            f'— check topic name and msg_type in the mapping JSON')


# ─────────────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    rospy.init_node('mapping_player', log_level=rospy.INFO)
    rospy.loginfo('\n\n\n\n\n')

    mf = rospy.get_param(f'{rospy.get_name()}/mapping_file', '')
    if not mf:
        rospy.logfatal(
            '[player] No mapping_file param set.\n'
            'Usage: rosrun <pkg> mapping_player.py _mapping_file:=<path/to/file.mapping.json>'
        )
        return
    if not os.path.isfile(mf):
        rospy.logfatal(f'[player] File not found: {mf}'); return

    player = MappingPlayer(mf, {}, 0)

    current_file_pub = rospy.Publisher(
        f'{rospy.get_name()}/current_file', String, queue_size=1, latch=True)
    current_file_pub.publish(player.current_file)

    def _on_file_path(msg):
        path = (msg.data or '').strip()
        if not path: return
        if player.reload(path):
            current_file_pub.publish(player.current_file)

    rospy.Subscriber(f'{rospy.get_name()}/file_path', String, _on_file_path, queue_size=1)

    from std_srvs.srv import Empty

    # Arms currently running a preset pose, so a second request for the same arm
    # is rejected instead of fighting the first.
    preset_pose_busy  = set()
    preset_pose_guard = threading.Lock()

    def _known_poses(arm_name):
        """List the preset-pose services this arm currently advertises."""
        prefix = f'/{arm_name}/preset_poses/'
        internal = ('get_loggers', 'set_logger_level', 'tf2_frames')
        try:
            import rosservice
            names = [name[len(prefix):] for name in rosservice.get_service_list()
                     if name.startswith(prefix)
                     and name[len(prefix):] not in internal]
        except Exception:
            return 'could not query the master'
        return ', '.join(sorted(names)) or 'none'

    def _call_preset_pose(pose_svc):
        """Call the blocking preset-pose service, giving up after a timeout.

        Returns None on success, else a message describing the failure.  A ROS
        service call cannot be cancelled, so on timeout the calling thread is
        abandoned (it is a daemon).  Leaking that thread is the lesser evil: the
        alternative is leaving relaxed_ik paused and the arm limp until someone
        restarts the launch file.
        """

        outcome = {}

        def _call():
            try:
                rospy.ServiceProxy(pose_svc, Empty)()
            except Exception as ex:
                outcome['error'] = str(ex)

        worker = threading.Thread(target=_call, daemon=True)
        worker.start()
        worker.join(PRESET_POSE_TIMEOUT_S)

        if worker.is_alive():
            return (f'timed out after {PRESET_POSE_TIMEOUT_S:.0f} s '
                    f'— move_group is not responding')
        return outcome.get('error')

    def _run_preset_pose(arm_name, pose_name):
        """Pause relaxed_ik, drive the arm to a preset pose, resume relaxed_ik.

        Runs on its own thread, one per arm, and deliberately does not take the
        player lock: a preset pose takes tens of seconds and the control graph
        has to keep ticking for the base, the chest and the other arm.
        """

        pose_svc   = f'/{arm_name}/preset_poses/{pose_name}'
        pause_svc  = f'/{arm_name}/teleoperation/pause_relaxed_ik'
        resume_svc = f'/{arm_name}/teleoperation/resume_relaxed_ik'

        try:
            try:
                rospy.wait_for_service(pose_svc,   timeout=2.0)
                rospy.wait_for_service(pause_svc,  timeout=2.0)
                rospy.wait_for_service(resume_svc, timeout=2.0)
            except Exception as ex:
                rospy.logwarn(
                    f'[player] preset_pose: {pose_svc} unavailable ({ex}).  '
                    f'Poses {arm_name} advertises: {_known_poses(arm_name)}')
                return

            try:
                rospy.ServiceProxy(pause_svc, Empty)()
                error = _call_preset_pose(pose_svc)
                if error:
                    rospy.logwarn(f'[player] preset_pose: {pose_svc}: {error}')
                else:
                    rospy.loginfo(f'[player] preset_pose: completed {pose_svc}')
            finally:
                try:
                    rospy.ServiceProxy(resume_svc, Empty)()
                except Exception as ex:
                    rospy.logwarn(f'[player] preset_pose: resume failed: {ex}')
        finally:
            with preset_pose_guard:
                preset_pose_busy.discard(arm_name)

    def _on_preset_pose(msg):
        key = (msg.data or '').strip()
        if not key or '/' not in key:
            rospy.logwarn(f'[player] preset_pose: invalid key "{key}" (expected arm_name/pose_name)')
            return
        arm_name, pose_name = key.split('/', 1)

        with preset_pose_guard:
            if arm_name in preset_pose_busy:
                rospy.logwarn(
                    f'[player] preset_pose: {arm_name} is already moving, '
                    f'ignoring {pose_name}')
                return
            preset_pose_busy.add(arm_name)

        threading.Thread(
            target=_run_preset_pose,
            args=(arm_name, pose_name),
            daemon=True,
        ).start()

    rospy.Subscriber('/preset_pose_command', String, _on_preset_pose, queue_size=10)

    last_freq = player.node_frequency
    rate = rospy.Rate(last_freq)
    rospy.loginfo('\033[92m[player] ready.\033[0m')
    while not rospy.is_shutdown():
        player.execute()
        if player.node_frequency != last_freq:
            last_freq = player.node_frequency
            rate = rospy.Rate(last_freq)
        rate.sleep()

if __name__ == '__main__':
    main()