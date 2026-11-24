"""Isaac Sim world of the aero_pursuit pursuit (docs/ISAAC_SIM.md).

Run with the Python of Isaac Sim 5.1 (isaac/run_world.sh). What it simulates and publishes:

- Pursuer ``x500_depth_0``: Pegasus multirotor (Iris geometry) with the x500 dynamics used in
  Gazebo (mass, inertia, motor/moment constants, ESC range 150..1000 rad/s), flown by PX4 SITL
  (MAVLink HIL, lockstep) with the x500 allocation parameters. PX4 talks uXRCE-DDS to the agent
  of the aero-isaac container (``/fmu/*`` unchanged).
- Fixed camera at the x500_depth mount: RGB 1920x1080, 69 deg (``/camera``, ``/camera/camera_info``)
  and depth, 32FC1 in metres (``/depth_camera``, ``/depth_camera/camera_info``). By default the
  depth comes from the RGB render (registered, 1920x1080, 69 deg): a second camera like Gazebo's
  OV7251 (640x480, 73 deg, ``--depth separate``) halves the real-time factor on two RTX A2000.
- Target drones (no PX4): kinematic bodies driven by the scenario runner through the ROS world
  (``/sim/*``, sim/ros_world.hpp): spawn/remove/teleport and heading-frame velocity commands,
  followed with a first-order lag like Gazebo's MulticopterVelocityControl.
- Ground truth: ``/tf`` ``<world> -> <model>`` for every model, ``/sim/poses`` and ``/clock``.

The loop paces itself to real time (``--rtf``), renders once every ``--render-every`` physics
steps (the cameras run at that rate: 250 Hz / 8 = 31.25 Hz) and prints the real-time factor.

Cost per physics step (--profile, docs/ISAAC_SIM.md "Desempenho"): the ROS executor spins in its
own thread, Pegasus reads the vehicle state without USD (fast_update_state), PhysX writes the
poses to Fabric (omni.physx.fabric) and the targets/gimbal touch USD only before a render.
"""
import argparse
import collections
import math
import signal
import threading
import os
import sys
import time

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--gui", action="store_true", help="janela do Isaac (padrão: headless)")
parser.add_argument("--world", default="lawn", help="frame do mundo (mesmo nome do Gazebo)")
parser.add_argument("--pursuer", default="x500_depth_0")
parser.add_argument("--spawn", type=float, nargs=3, default=[0.0, 0.0, 0.07], metavar=("X", "Y", "Z"))
parser.add_argument("--env", default="Flat Plane", help="ambiente do Pegasus (SIMULATION_ENVIRONMENTS)")
parser.add_argument("--camera", choices=["fixed", "gimbal", "none"], default="gimbal",
                    help="gimbal (default, as in Gazebo): camera on the x500_gimbal_depth joints "
                    "(/gimbal/joint_command; a fixed camera is the gimbal held still); fixed: old "
                    "body-fixed mount (no gimbal_node TF: use only with a matching launch)")
parser.add_argument("--rgb", type=int, nargs=2, default=[1920, 1080], metavar=("W", "H"))
parser.add_argument("--depth", choices=["aligned", "separate", "none"], default="aligned",
                    help="aligned: depth from the same render as the RGB (1 render/frame); separate: "
                         "own OV7251 camera 640x480, 73 degrees, as in Gazebo (2 renders, slower)")
parser.add_argument("--render-every", type=int, default=8, help="physics steps (250 Hz) per frame")
parser.add_argument("--single-gpu", action="store_true", help="render on a single GPU (default: all; ~2x slower with 2x A2000)")
parser.add_argument("--rtf", type=float, default=1.0, help="limite do fator de tempo real (0 = livre)")
parser.add_argument("--propeller-every", type=int, default=1, metavar="N",
                    help="spin the target propellers every N rendered frames (0 = still; visual only)")
parser.add_argument("--tf-every", type=int, default=5, help="physics steps per /tf and /sim/poses sample")
# Same base drones as the Gazebo world (launch_gz_and_drones.sh, scenario_runner base_drones):
# spawned at 1 m and enabled, so they hover ready for the joystick. --base-drones with no value: none.
BASE_DRONES = ["X4:x4:5:-2:1"]   # the others (X3, Tiltrotor, X5) can be added in the GCS Drones tab
parser.add_argument("--base-drones", nargs="*", default=BASE_DRONES,
                    help="drones present from the start, enabled: name:model:x:y:z")
parser.add_argument("--spare-targets", nargs="*", default=["x4", "x3", "x5", "tiltrotor"],
                    help="target models created hidden at startup for spawns while playing (DynamicDrone.park)")
parser.add_argument("--max-time", type=float, default=0.0, help="quit after this simulated time [s] (0 = never)")
parser.add_argument("--cprofile", type=int, default=0, metavar="N",
                    help="cProfile of N physics steps from t=20 s (top 40 in world.log)")
parser.add_argument("--usd-physics", action="store_true",
                    help="PhysX writes the poses to the USD (default: Fabric, omni.physx.fabric; ~0.7 ms/step less)")
parser.add_argument("--scenery", default="none",
                    help="cenografia: config/scenery/<nome>.json do aero_pursuit (a mesma do Gazebo, AERO_SCENERY)")
parser.add_argument("--scenery-shadows", action="store_true", help="trees cast shadows (slower)")
parser.add_argument("--kinematic-targets", action="store_true",
                    help="kinematic targets (no physics); default: rigid body + the Gazebo velocity controller")
parser.add_argument("--pegasus-state", action="store_true",
                    help="original Pegasus update_state (default: the fast version, fast_update_state)")
parser.add_argument("--profile", action="store_true",
                    help="wall time per loop phase (physics, render, ROS, wait) in the 5 s report")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "multi_gpu": not args.single_gpu})

# --- after SimulationApp ------------------------------------------------------------------
import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
if not args.usd_physics:
    # Rigid-body poses go to Fabric (what the renderer reads) instead of USD: no USD writes
    # and change notices per physics step. dynamic_control (Pegasus state) reads PhysX itself.
    enable_extension("omni.physx.fabric")
    import carb.settings
    for key in ("/physics/updateToUsd", "/physics/updateVelocitiesToUsd", "/physics/outputVelocitiesLocalSpace"):
        carb.settings.get_settings().set_bool(key, False)
app.update()

import omni.replicator.core as rep  # noqa: E402
import omni.syntheticdata  # noqa: E402
import omni.graph.core as og  # noqa: E402
import rclpy  # noqa: E402
import rclpy.executors  # noqa: E402
import rclpy.signals  # noqa: E402
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from geometry_msgs.msg import TransformStamped, TwistStamped  # noqa: E402
from isaacsim.ros2.bridge import read_camera_info  # noqa: E402
from omni.isaac.core.world import World  # noqa: E402
from pegasus.simulator.logic.backends.px4_mavlink_backend import (  # noqa: E402
    PX4MavlinkBackend, PX4MavlinkBackendConfig)
from pegasus.simulator.logic.dynamics import LinearDrag  # noqa: E402
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface  # noqa: E402
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve  # noqa: E402
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig  # noqa: E402
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from target_models import DYNAMICS, Propellers, build_model  # noqa: E402
from omni.isaac.dynamic_control import _dynamic_control  # noqa: E402
from pxr import Usd  # noqa: E402

GRAVITY = 9.81

PHYSICS_HZ = 250.0

# ---- x500 (PX4 Tools/simulation/gz/models/x500*, the same model as in Gazebo) -------------
X500_MASS = 2.0
X500_INERTIA = (0.02166666666666667, 0.02166666666666667, 0.04000000000000001)
X500_MOTOR_CONSTANT = 8.54858e-06              # thrust = k w^2 [N/(rad/s)^2]
X500_MOMENT_CONSTANT = 0.016                   # rotor drag torque = c * thrust
X500_ESC_MIN, X500_ESC_MAX = 150.0, 1000.0     # SIM_GZ_EC_MIN/MAX [rad/s]
# Rotor drag (rotorDragCoefficient 8.06e-5 * w * v) at hover (w ~ 760 rad/s), 4 rotors [N/(m/s)].
X500_LINEAR_DRAG = (0.25, 0.25, 0.0)
# Airframe 4001_gz_x500: allocation geometry and hover thrust (the 10015 airframe used by the
# MAVLink HIL bridge is Iris; these overrides make PX4 see the x500).
X500_PX4_PARAMS = {
    "CA_ROTOR0_PX": 0.13, "CA_ROTOR0_PY": 0.22, "CA_ROTOR0_KM": 0.05,
    "CA_ROTOR1_PX": -0.13, "CA_ROTOR1_PY": -0.2, "CA_ROTOR1_KM": 0.05,
    "CA_ROTOR2_PX": 0.13, "CA_ROTOR2_PY": -0.22, "CA_ROTOR2_KM": -0.05,
    "CA_ROTOR3_PX": -0.13, "CA_ROTOR3_PY": 0.2, "CA_ROTOR3_KM": -0.05,
    "MPC_THR_HOVER": 0.6,
    "NAV_DLL_ACT": 0,  # no GCS link required to arm (launch_gz_and_drones.sh does the same)
}

# ---- cameras (OakD-Lite of x500_depth: IMX214 RGB + OV7251 depth) --------------------------
CAMERA_LINK = (0.12, 0.03, 0.242)               # base_link -> camera_link (same static TF)
SENSOR_OFFSET = (0.01233, -0.03, 0.01878)       # camera_link -> sensor in the SDF
RGB_HFOV = 1.204
DEPTH_HFOV, DEPTH_SIZE, DEPTH_CLIP = 1.274, (640, 480), (0.2, 19.1)
RGB_CLIP = (0.1, 100.0)
# x500_gimbal_depth: base_link -> gimbal_mount (same static TF as the Gazebo launch), joints in the
# order of the SDF (yaw about z, roll about x, pitch about -y) and their limits [rad].
GIMBAL_MOUNT = (0.15, 0.03, 0.165)
GIMBAL_JOINTS = ("gimbal_yaw_joint", "gimbal_roll_joint", "gimbal_pitch_joint")
GIMBAL_LIMITS = ((-2.967, 2.967), (-0.785, 0.785), (-1.6, 0.55))
GIMBAL_TAU = 0.03   # [s] position loop of the joints (Gazebo JointPositionController, stiff)
# USD cameras look along -Z with +Y up; camera_link is x forward, z up (FLU):
# X_usd = -Y_link, Y_usd = Z_link, Z_usd = -X_link.
USD_CAM_IN_LINK = Rotation.from_matrix(np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float))


def stamp_of(t: float) -> TimeMsg:
    sec = int(math.floor(t))
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


def set_xform(prim, position, quat_xyzw):
    """Translate + orient ops on a prim (created once, updated in place)."""
    xf = UsdGeom.Xformable(prim)
    ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
    t = ops.get("xformOp:translate") or xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    o = ops.get("xformOp:orient") or xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    if not ops:
        xf.SetXformOpOrder([t, o])
    t.Set(Gf.Vec3d(*map(float, position)))
    x, y, z, w = map(float, quat_xyzw)
    o.Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))


class KinematicDrone:
    """Target drone without PX4: pose integrated from heading-frame velocity commands."""

    TAU = 0.2  # [s] first-order response of the velocity (Gazebo MulticopterVelocityControl ~)

    def __init__(self, stage, root, name, model, position, quat_xyzw):
        self.name = name
        self.model = model
        self.path = f"{root}/{name}"
        self.prim = build_model(stage, self.path, model)
        self.props = Propellers(self.prim)
        self.position = np.array(position, dtype=float)
        self.yaw = Rotation.from_quat(quat_xyzw).as_euler("ZYX")[0]
        self.velocity = np.zeros(3)       # world frame
        self.yaw_rate = 0.0
        self.cmd_heading = np.zeros(3)    # heading frame
        self.cmd_yaw_rate = 0.0
        self.enabled = False
        self.apply()

    def teleport(self, position, quat_xyzw):
        self.position = np.array(position, dtype=float)
        self.yaw = Rotation.from_quat(quat_xyzw).as_euler("ZYX")[0]
        self.velocity[:] = 0.0
        self.yaw_rate = 0.0
        self.apply()

    def step(self, dt):
        if self.enabled:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            v = self.cmd_heading
            target = np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])
            target_rate = self.cmd_yaw_rate
        else:
            target, target_rate = np.zeros(3), 0.0
        a = min(1.0, dt / self.TAU)
        self.velocity += a * (target - self.velocity)
        self.yaw_rate += a * (target_rate - self.yaw_rate)
        self.position += self.velocity * dt
        self.position[2] = max(self.position[2], 0.0)
        self.yaw = math.atan2(math.sin(self.yaw + self.yaw_rate * dt), math.cos(self.yaw + self.yaw_rate * dt))
        self.props.step(dt, 1.0 if self.enabled else 0.0)
        # USD pose only before a render (AeroWorld.run): the camera is what sees it, and each
        # write wakes the Kit USD listeners (250 Hz x targets for nothing).

    def quat(self):
        return Rotation.from_euler("z", self.yaw).as_quat()

    def apply(self):
        set_xform(self.prim, self.position, self.quat())


def quat_to_matrix(q):
    """x, y, z, w -> 3x3 (scipy's Rotation costs ~30 us per call; this runs per target per step)."""
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


class DynamicDrone:
    """Target drone with physics, as in Gazebo: a PhysX rigid body (mass and inertia of the SDF,
    gravity, a box collider) flown by the LeeVelocityController of gz-sim's
    MulticopterVelocityControl with the model's own gains (target_models.DYNAMICS). The command
    is a heading-frame velocity and a yaw rate; the controller turns it into a collective thrust
    along the body z axis and a torque, so the drone tilts to accelerate, lags like the Gazebo
    one and, disabled (motors off), falls and rests on the ground."""

    dc = None

    def __init__(self, stage, root, name, model, position, quat_xyzw):
        if DynamicDrone.dc is None:
            DynamicDrone.dc = _dynamic_control.acquire_dynamic_control_interface()
        self.name, self.model = name, model
        self.path = f"{root}/{name}"
        self.prim = build_model(stage, self.path, model)
        self.props = Propellers(self.prim)        # before the collider (not a rotor anyway)
        p = DYNAMICS.get(model, DYNAMICS["x4"])
        self.mass = p["mass"]
        self.inertia = np.array(p["inertia"])
        self.kv = np.array(p["vel_gain"]) / self.mass
        self.k_att, self.k_rate = np.array(p["att_gain"]), np.array(p["rate_gain"])
        self.max_acc, self.max_vel = np.array(p["max_acc"], float), np.array(p["max_vel"], float)
        # Centre of mass and collider from the visual's box (model frame = TF frame, as in Gazebo).
        box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeUntransformedBound(
            self.prim).ComputeAlignedRange()
        self.com = np.array(box.GetMidpoint()) if not box.IsEmpty() else np.zeros(3)
        size = np.array(box.GetSize()) if not box.IsEmpty() else np.array([0.5, 0.5, 0.15])
        col = UsdGeom.Cube.Define(stage, self.path + "/collider")
        col.CreateSizeAttr(1.0)
        col.AddTranslateOp().Set(Gf.Vec3d(*self.com))
        col.AddScaleOp().Set(Gf.Vec3f(*np.maximum(size * 0.9, 0.05)))
        col.CreatePurposeAttr(UsdGeom.Tokens.guide)       # not rendered
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(self.prim)
        mass = UsdPhysics.MassAPI.Apply(self.prim)
        mass.CreateMassAttr(self.mass)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*self.inertia))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*self.com))
        self.position = np.array(position, dtype=float)
        self.attitude = np.array(quat_xyzw, dtype=float)
        set_xform(self.prim, self.position, self.attitude)
        self.cmd_heading = np.zeros(3)
        self.cmd_yaw_rate = 0.0
        self.enabled = False
        self.parked = False
        self._gravity_off = False
        self._handle = None
        self._pending_pose = None

    def park(self, position):
        """Out of the scene, kept for reuse: a rigid body added while the timeline plays is moved
        by PhysX but never reaches Fabric, so the cameras would see it frozen where it spawned.
        Removing the prim is therefore avoided: a parked body is hidden, motors off, without
        gravity, far away, and the next spawn of the same model takes it back (unpark)."""
        self.parked, self.enabled = True, False
        self.cmd_heading, self.cmd_yaw_rate = np.zeros(3), 0.0
        UsdGeom.Imageable(self.prim).MakeInvisible()
        self.teleport(position, (0.0, 0.0, 0.0, 1.0))

    def unpark(self, name, position, quat_xyzw):
        self.name, self.parked = name, False
        UsdGeom.Imageable(self.prim).MakeVisible()
        self.teleport(position, quat_xyzw)

    def handle(self):
        if not self._handle:
            h = self.dc.get_rigid_body(self.path)
            self._handle = h if h != _dynamic_control.INVALID_HANDLE else None
        return self._handle

    def teleport(self, position, quat_xyzw):
        self.position = np.array(position, dtype=float)
        self.attitude = np.array(quat_xyzw, dtype=float)
        self._pending_pose = (self.position.copy(), self.attitude.copy())
        set_xform(self.prim, self.position, self.attitude)

    @property
    def yaw(self):
        x, y, z, w = self.attitude
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def quat(self):
        return self.attitude

    def step(self, dt):
        h = self.handle()
        if h is None:
            return
        dc = self.dc
        if self._pending_pose is not None:
            p, q = self._pending_pose
            dc.set_rigid_body_pose(h, _dynamic_control.Transform(p.tolist(), q.tolist()))
            dc.set_rigid_body_linear_velocity(h, (0.0, 0.0, 0.0))
            dc.set_rigid_body_angular_velocity(h, (0.0, 0.0, 0.0))
            self._pending_pose = None
            return
        if self._gravity_off != self.parked:
            dc.set_rigid_body_disable_gravity(h, self.parked)
            self._gravity_off = self.parked
        if self.parked:
            return
        pose = dc.get_rigid_body_pose(h)
        self.position = np.array((pose.p.x, pose.p.y, pose.p.z))
        self.attitude = np.array((pose.r.x, pose.r.y, pose.r.z, pose.r.w))
        if not self.enabled:
            self.props.step(dt, 0.0)
            return                                    # motors off
        v = dc.get_rigid_body_linear_velocity(h)
        w = dc.get_rigid_body_angular_velocity(h)     # world frame
        R = quat_to_matrix(self.attitude)
        yaw = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        # As gz-sim's LeeVelocityController: the command is clipped per axis and turned into the
        # world frame with the full body rotation R (the scenario runner sends R^-1 v_world).
        v_cmd = R @ np.clip(self.cmd_heading, -self.max_vel, self.max_vel)
        acc = np.clip(self.kv * (v_cmd - np.array((v.x, v.y, v.z))), -self.max_acc, self.max_acc)
        force = self.mass * (acc + np.array((0.0, 0.0, GRAVITY)))
        thrust = max(float(force @ R[:, 2]), 0.0)
        b3 = force / max(np.linalg.norm(force), 1e-6)
        b2 = np.cross(b3, np.array((c, s, 0.0)))
        b2 /= max(np.linalg.norm(b2), 1e-6)
        Rd = np.column_stack((np.cross(b2, b3), b2, b3))
        w_body = R.T @ np.array((w.x, w.y, w.z))
        e_R = 0.5 * _vee(Rd.T @ R - R.T @ Rd)
        e_w = w_body - R.T @ Rd @ np.array((0.0, 0.0, self.cmd_yaw_rate))
        torque = -self.k_att * e_R - self.k_rate * e_w + np.cross(w_body, self.inertia * w_body)
        dc.apply_body_force(h, (0.0, 0.0, thrust), tuple(self.com.tolist()), False)
        self.props.step(dt, thrust / (self.mass * GRAVITY))
        dc.apply_body_torque(h, tuple(torque.tolist()), False)

    def apply(self):
        pass                                          # PhysX moves it


class Gimbal:
    """Three revolute joints driven to the commanded positions (sensor_msgs/JointState on
    /gimbal/joint_command, the ros joint interface of gimbal_controller); camera_link turns
    as R = Rz(yaw) Rx(roll) Ry(-pitch) about gimbal_mount, the rotation gimbal_controller
    publishes from /gimbal/joint_states."""

    def __init__(self, mount_prim):
        self.link = UsdGeom.Xform.Define(mount_prim.GetStage(), str(mount_prim.GetPath()) + "/camera_link").GetPrim()
        self.q = np.zeros(3)
        self.cmd = np.zeros(3)
        self.apply()

    def on_command(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            for i, joint in enumerate(GIMBAL_JOINTS):
                if joint in name or name in ("yaw", "roll", "pitch")[i:i + 1]:
                    lo, hi = GIMBAL_LIMITS[i]
                    self.cmd[i] = min(max(float(pos), lo), hi)

    def step(self, dt):
        self.q += min(1.0, dt / GIMBAL_TAU) * (self.cmd - self.q)

    def rotation(self):
        yaw, roll, pitch = self.q
        return Rotation.from_euler("ZXY", [yaw, roll, -pitch])

    def apply(self):
        set_xform(self.link, (0.0, 0.0, 0.0), self.rotation().as_quat())

    def joint_state(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(GIMBAL_JOINTS)
        msg.position = [float(v) for v in self.q]
        return msg


def fast_update_state(vehicle):
    """Vehicle.update_state of Pegasus without the USD read of the attitude and the slow
    np.array(carb.Float3) conversions (~0.5 -> ~0.15 ms per physics step): the whole pose from
    dynamic_control (pose.r is x, y, z, w). AeroWorld.run compares it with the USD attitude."""
    dc = vehicle.get_dc_interface()
    path = vehicle._stage_prefix + "/body"
    state = vehicle._state

    def update_state(dt):
        body = dc.get_rigid_body(path)
        pose = dc.get_rigid_body_pose(body)
        p, r = pose.p, pose.r
        v = dc.get_rigid_body_linear_velocity(body)
        w = dc.get_rigid_body_angular_velocity(body)
        lin = np.array((v.x, v.y, v.z))
        att = np.array((r.x, r.y, r.z, r.w))
        inv = Rotation.from_quat(att).inv()
        state.linear_acceleration = (lin - state.linear_velocity) / dt
        state.position = np.array((p.x, p.y, p.z))
        state.attitude = att
        state.linear_velocity = lin
        state.linear_body_velocity = inv.apply(lin)
        state.angular_velocity = inv.apply(np.array((w.x, w.y, w.z)))

    # Pegasus subscribed the bound methods at construction (Vehicle.__init__); re-subscribe the
    # four vehicle callbacks in their original order (state before actuators, sensors, MAVLink).
    world, prefix = vehicle._world, vehicle._stage_prefix
    callbacks = [("/state", update_state), ("/update", vehicle.update),
                 ("/Sensors", vehicle.update_sensors), ("/mav_state", vehicle.update_sim_state)]
    for name, _ in callbacks:
        world.remove_physics_callback(prefix + name)
    for name, fn in callbacks:
        world.add_physics_callback(prefix + name, fn)
    vehicle.update_state = update_state


TARGET_CLASS = KinematicDrone if args.kinematic_targets else DynamicDrone


SPECTATOR_MODEL = "spectator_camera"
SPECTATOR_SIZE, SPECTATOR_HFOV, SPECTATOR_CLIP = (1280, 720), 1.15, (0.1, 600.0)   # gz_models/spectator_camera
SPECTATOR_EVERY = 1   # camera frames per free-camera frame (toggling the render per frame costs more)


class SpectatorCamera:
    """Free camera of the GCS ("Isaac" view): spawned by spectator_camera_node (world_backend ros)
    through /sim/spawn with the model spectator_camera, moved by /sim/set_pose (pose in the
    Gazebo camera convention: x forward, z up), published on /spectator/image like the Gazebo
    sensor. It renders only while someone subscribes to the image (a second camera costs
    render time every frame), checked once per second."""

    def __init__(self, world, name, position, quat_xyzw):
        self.world, self.name = world, name
        self.path = f"/World/{name}"
        self.prim = UsdGeom.Xform.Define(world.stage, self.path).GetPrim()
        self.move(position, quat_xyzw)
        cam = world._camera(self.path + "/cam", (0.0, 0.0, 0.0), SPECTATOR_HFOV, SPECTATOR_SIZE, SPECTATOR_CLIP)
        world._publish_camera(cam, SPECTATOR_SIZE, "spectator/image", "LdrColorSDROS2PublishImage", frame=name)
        self.rp = world.render_products.get("spectator/image")
        self.active = True
        self.set_active(False)
        print(f"[aero_world] free camera {name} (renders only with a viewer)", flush=True)

    def move(self, position, quat_xyzw):
        self.position = np.array(position, dtype=float)
        self.quat = np.array(quat_xyzw, dtype=float)
        set_xform(self.prim, self.position, self.quat)

    def set_active(self, on):
        if on == self.active or self.rp is None:
            return
        self.active = on
        self._enable(on)
        print(f"[aero_world] free camera {'on' if on else 'idle'}", flush=True)

    def _enable(self, on):
        try:
            self.rp.hydra_texture.set_updates_enabled(on)
        except AttributeError as e:
            carb.log_warn(f"free camera: no render update control ({e})")

    def before_render(self, frame):
        """With a viewer, render every SPECTATOR_EVERY-th camera frame (~15 Hz): enough to watch."""
        if self.active and SPECTATOR_EVERY > 1:
            self._enable(frame % SPECTATOR_EVERY == 0)

    def poll(self):
        self.set_active(self.world.node.count_subscribers("/spectator/image") > 0)


class AeroWorld:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        settings = dict(self.pg._world_settings)
        # rendering_dt = physics_dt: a step with render=True advances one physics step too
        # (render decimation is done by the loop).
        settings.update(physics_dt=1.0 / PHYSICS_HZ, rendering_dt=1.0 / PHYSICS_HZ)
        self.pg._world = World(**settings)
        self.world = self.pg.world
        self.stage = self.world.stage
        # GCS / host agent send names without spaces (Flat_Plane): same keys as Pegasus.
        self.pg.load_environment(SIMULATION_ENVIRONMENTS[args.env.replace('_', ' ')])

        self._ensure_light()

        self.render_products = {}
        self._add_scenery()
        self.models_root = "/World/models"
        UsdGeom.Xform.Define(self.stage, self.models_root)
        self.pursuer = self._spawn_pursuer()
        if not args.pegasus_state:
            fast_update_state(self.pursuer)
        self.gimbal = None
        if args.camera == "fixed":
            self._add_cameras(f"/World/{args.pursuer}/body/camera_link", CAMERA_LINK)
        elif args.camera == "gimbal":
            mount = f"/World/{args.pursuer}/body/gimbal_mount"
            set_xform(UsdGeom.Xform.Define(self.stage, mount).GetPrim(), GIMBAL_MOUNT, (0, 0, 0, 1))
            self.gimbal = Gimbal(self.stage.GetPrimAtPath(mount))
            self._add_cameras(f"{mount}/camera_link", (0.0, 0.0, 0.0))

        self.spectator = None
        self.targets = {}
        for entry in args.base_drones:
            name, model, x, y, z = entry.split(":")
            self.targets[name] = TARGET_CLASS(self.stage, self.models_root, name, model,
                                              (float(x), float(y), float(z)), (0, 0, 0, 1))
            self.targets[name].enabled = True
        # Rigid bodies must exist before the first reset to be drawn when they move (DynamicDrone.park):
        # one parked spare per model without a base drone, taken by the first spawn of that model.
        self.parked = []
        if TARGET_CLASS is DynamicDrone:
            have = {d.model for d in self.targets.values()}
            for i, model in enumerate(m for m in args.spare_targets if m not in have):
                self._park(DynamicDrone(self.stage, self.models_root, f"spare_{model}_{i}", model,
                                        (0.0, 0.0, 0.0), (0, 0, 0, 1)))
        self._init_ros()
        self.world.reset()
        if args.profile:
            self._print_physics_setup()

    def _print_physics_setup(self):
        """PhysX scene, rigid bodies and joints (what one physics step solves)."""
        bodies, joints, colliders = [], 0, 0
        for prim in self.stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.append(str(prim.GetPath()))
            if prim.IsA(UsdPhysics.Joint):
                joints += 1
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                colliders += 1
            if prim.GetTypeName() == "PhysicsScene":
                attrs = {a.GetName(): a.Get() for a in prim.GetAttributes() if a.HasAuthoredValue()}
                print(f"[aero_world] cena {prim.GetPath()}: {attrs}", flush=True)
        settings = carb.settings.get_settings()
        print(f"[aero_world] rigid bodies {len(bodies)} {bodies[:12]}, joints {joints}, colliders {colliders}, "
              f"threads physx {settings.get('/persistent/physics/numThreads')}", flush=True)

    # -- scene ------------------------------------------------------------------------------
    def _add_scenery(self):
        """Static scenery shared with Gazebo (tools/scenery_world.py): the same Fuel models, imported
        to assets/<model>.usd by import_gz_models.py, at the same poses. Visual only (no collider):
        the trees stay out of the flight area, as a cluttered background for the tracker."""
        if args.scenery in ("", "none"):
            return
        import json
        candidates = [os.path.join(HERE, "..", "src", "ros2_ws", "src", "aero_pursuit", "config", "scenery"),
                      os.path.join(HERE, "..", "ros2_ws", "src", "aero_pursuit", "config", "scenery")]
        path = next((os.path.join(d, args.scenery + ".json") for d in candidates
                     if os.path.isfile(os.path.join(d, args.scenery + ".json"))), None)
        if path is None:
            carb.log_error(f"scenery '{args.scenery}' not found in {candidates}")
            return
        doc = json.load(open(path))
        root = "/World/scenery"
        UsdGeom.Xform.Define(self.stage, root)
        missing = set()
        n = 0
        for t in doc["trees"]:
            asset = os.path.join(HERE, "assets", f"{t['model']}.usd")
            if not os.path.exists(asset):
                missing.add(t["model"])
                continue
            prim = UsdGeom.Xform.Define(self.stage, f"{root}/{t['name']}").GetPrim()
            prim.GetReferences().AddReference(asset)
            set_xform(prim, (t["x"], t["y"], 0.0), Rotation.from_euler("z", t["yaw"]).as_quat())
            if not args.scenery_shadows:
                # Shadows of alpha-cut foliage cost the most in the ray-traced render.
                UsdGeom.PrimvarsAPI(prim).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
            n += 1
        print(f"[aero_world] cenografia {args.scenery}: {n} modelos"
              + (f" (sem asset: {sorted(missing)}; rode import_gz_models.py)" if missing else ""), flush=True)

    def _ensure_light(self):
        has_light = any(p.HasAPI(UsdLux.LightAPI) or p.GetTypeName().endswith("Light")
                        for p in self.stage.Traverse())
        if not has_light:
            sun = UsdLux.DistantLight.Define(self.stage, "/World/sun")
            sun.CreateIntensityAttr(3000.0)
            set_xform(sun.GetPrim(), (0, 0, 10), Rotation.from_euler("xy", [-35, 20], degrees=True).as_quat())
            UsdLux.DomeLight.Define(self.stage, "/World/sky").CreateIntensityAttr(600.0)

    def _spawn_pursuer(self):
        for k, v in X500_PX4_PARAMS.items():
            os.environ[f"PX4_PARAM_{k}"] = str(v)   # inherited by the PX4 process Pegasus launches
        cfg = MultirotorConfig()
        cfg.thrust_curve = QuadraticThrustCurve({
            "num_rotors": 4,
            "rotor_constant": [X500_MOTOR_CONSTANT] * 4,
            "rolling_moment_coefficient": [X500_MOMENT_CONSTANT * X500_MOTOR_CONSTANT] * 4,
            "rot_dir": [-1, -1, 1, 1],
            "min_rotor_velocity": [0.0] * 4,
            "max_rotor_velocity": [X500_ESC_MAX] * 4,
        })
        cfg.drag = LinearDrag(list(X500_LINEAR_DRAG))
        cfg.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
            # w = 150 + 850 u (SIM_GZ_EC_MIN/MAX of the x500); 150 rad/s idle once armed
            "input_offset": [0.0] * 4,
            "input_scaling": [X500_ESC_MAX - X500_ESC_MIN] * 4,
            "zero_position_armed": [X500_ESC_MIN] * 4,
        }))]
        vehicle = Multirotor(f"/World/{args.pursuer}", ROBOTS["Iris"], 0, list(args.spawn),
                             Rotation.from_euler("XYZ", [0, 0, 0]).as_quat(), config=cfg)
        body = self.stage.GetPrimAtPath(f"/World/{args.pursuer}/body")
        mass = UsdPhysics.MassAPI.Apply(body)
        mass.CreateMassAttr(X500_MASS)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*X500_INERTIA))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        return vehicle

    def _camera(self, path, translation, hfov, size, clip):
        cam = UsdGeom.Camera.Define(self.stage, path)
        h_ap = 20.955
        cam.CreateHorizontalApertureAttr(h_ap)
        cam.CreateVerticalApertureAttr(h_ap * size[1] / size[0])
        cam.CreateFocalLengthAttr(h_ap / (2.0 * math.tan(hfov / 2.0)))
        cam.CreateClippingRangeAttr(Gf.Vec2f(*clip))
        set_xform(cam.GetPrim(), translation, USD_CAM_IN_LINK.as_quat())
        return path

    def _add_cameras(self, link, link_position):
        set_xform(UsdGeom.Xform.Define(self.stage, link).GetPrim(), link_position, (0, 0, 0, 1))
        self.camera_writers = getattr(self, "camera_writers", [])
        rgb = self._camera(f"{link}/rgb", SENSOR_OFFSET, RGB_HFOV, args.rgb, RGB_CLIP)
        rgb_rp = self._publish_camera(rgb, tuple(args.rgb), "camera", "LdrColorSDROS2PublishImage")
        if args.depth == "separate":
            depth = self._camera(f"{link}/depth", SENSOR_OFFSET, DEPTH_HFOV, DEPTH_SIZE, DEPTH_CLIP)
            self._publish_camera(depth, DEPTH_SIZE, "depth_camera", "DistanceToImagePlaneSDROS2PublishImage")
        elif args.depth == "aligned":
            # Same render product as the RGB: one render per frame, depth registered to the RGB
            # (its own camera_info carries the RGB intrinsics).
            self._publish_camera(rgb, tuple(args.rgb), "depth_camera",
                                 "DistanceToImagePlaneSDROS2PublishImage", rp_path=rgb_rp)

    def _publish_camera(self, cam_path, size, topic, image_writer, rp_path=None, frame="camera_optical"):
        if rp_path is None:
            rp = rep.create.render_product(cam_path, size)
            rp_path = rp.path if hasattr(rp, "path") else rp
            self.render_products[topic] = rp
        w = rep.writers.get(image_writer)
        w.initialize(nodeNamespace="", topicName=topic, frameId=frame, queueSize=1)
        w.attach([rp_path])
        info, _ = read_camera_info(render_product_path=rp_path)
        wi = rep.writers.get("ROS2PublishCameraInfo")
        wi.initialize(nodeNamespace="", topicName=f"{topic}/camera_info", frameId=frame, queueSize=1,
                      width=info.width, height=info.height, projectionType=info.distortion_model,
                      k=info.k.reshape([1, 9]), r=info.r.reshape([1, 9]), p=info.p.reshape([1, 12]),
                      physicalDistortionModel=info.distortion_model,
                      physicalDistortionCoefficients=info.d)
        wi.attach([rp_path])
        self.camera_writers += [w, wi]
        print(f"[aero_world] camera /{topic}: {size[0]}x{size[1]} fx={info.k[0]:.1f}", flush=True)
        return rp_path

    # -- ROS --------------------------------------------------------------------------------
    def _init_ros(self):
        """The executor spins in its own thread (a spin_once per physics step cost ~0.65 ms of
        wait-set setup at 250 Hz); the callbacks only queue the message, and the loop applies it
        in the main thread (USD and the models are not thread-safe). Publishing is thread-safe."""
        # Our own SIGINT/SIGTERM (kill_world.sh): rclpy's handler would invalidate the context in
        # the middle of a loop iteration (publish on a dead context); the loop ends on the flag.
        rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.stop_requested = False
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: setattr(self, "stop_requested", True))
        self.node = rclpy.create_node("aero_isaac_world")
        self.inbox = collections.deque()
        rel = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", 10)
        self.pub_tf = self.node.create_publisher(TFMessage, "/tf", 100)
        self.pub_poses = self.node.create_publisher(TFMessage, "/sim/poses", QoSProfile(depth=10))

        def sub(msg_type, topic, handler, qos):
            self.node.create_subscription(msg_type, topic, lambda m: self.inbox.append((handler, m)), qos)

        sub(TransformStamped, "/sim/spawn", self._on_spawn, rel)
        sub(String, "/sim/remove", self._on_remove, rel)
        sub(TransformStamped, "/sim/set_pose", self._on_set_pose, rel)
        sub(TwistStamped, "/sim/twist", self._on_twist, rel)
        sub(String, "/sim/enable", self._on_enable, rel)
        if self.gimbal:
            self.pub_joints = self.node.create_publisher(JointState, "/gimbal/joint_states", 10)
            sub(JointState, "/gimbal/joint_command", self.gimbal.on_command, 10)
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        def spin():
            try:
                self.executor.spin()
            except (rclpy.executors.ExternalShutdownException, KeyboardInterrupt):
                pass

        threading.Thread(target=spin, name="ros", daemon=True).start()

    def _drain_inbox(self):
        while self.inbox:
            handler, msg = self.inbox.popleft()
            handler(msg)

    @staticmethod
    def _pose_of(tf: TransformStamped):
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def _on_spawn(self, tf: TransformStamped):
        name, model = tf.child_frame_id, tf.header.frame_id
        pos, quat = self._pose_of(tf)
        if name == args.pursuer:
            return
        if model == SPECTATOR_MODEL:
            if self.spectator is None:
                self.spectator = SpectatorCamera(self, name, pos, quat)
            else:
                self.spectator.move(pos, quat)
            return
        if name in self.targets:
            self.targets[name].teleport(pos, quat)
            return
        spare = next((d for d in self.parked if d.model == model), None)
        if spare is not None:
            self.parked.remove(spare)
            spare.unpark(name, pos, quat)
            self.targets[name] = spare
            print(f"[aero_world] spawn {name} ({model}) at {np.round(pos, 2)} (reused {spare.path})", flush=True)
            return
        try:
            self.targets[name] = TARGET_CLASS(self.stage, self.models_root, name, model, pos, quat)
            print(f"[aero_world] spawn {name} ({model}) at {np.round(pos, 2)}", flush=True)
            if isinstance(self.targets[name], DynamicDrone) and self.timeline.is_playing():
                carb.log_warn(f"{name} ({model}) created while playing: PhysX moves it but the cameras may "
                              f"not (no Fabric entry); --spare-targets {model} creates one at startup")
        except Exception as e:  # unknown model: the runner retries/logs the timeout
            carb.log_error(f"spawn {name} ({model}): {e}")

    def _park(self, drone):
        drone.park((-1000.0 - 5.0 * len(self.parked), -1000.0, 20.0))
        self.parked.append(drone)

    def _on_remove(self, msg: String):
        drone = self.targets.pop(msg.data, None)
        if drone is None:
            return
        if isinstance(drone, DynamicDrone):
            self._park(drone)                         # see DynamicDrone.park
        else:
            self.stage.RemovePrim(drone.path)
        print(f"[aero_world] remove {msg.data}", flush=True)

    def _on_set_pose(self, tf: TransformStamped):
        if self.spectator is not None and tf.child_frame_id == self.spectator.name:
            self.spectator.move(*self._pose_of(tf))
            return
        drone = self.targets.get(tf.child_frame_id)
        if drone:
            drone.teleport(*self._pose_of(tf))

    def _on_twist(self, msg: TwistStamped):
        drone = self.targets.get(msg.header.frame_id)
        if drone:
            v = msg.twist.linear
            drone.cmd_heading = np.array([v.x, v.y, v.z])
            drone.cmd_yaw_rate = msg.twist.angular.z

    def _on_enable(self, msg: String):
        name, _, flag = msg.data.partition(" ")
        drone = self.targets.get(name)
        if drone:
            drone.enabled = flag.strip() != "0"

    def _publish_truth(self, t):
        stamp = stamp_of(t)
        tfs = []

        def add(name, pos, quat):
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = args.world
            tf.child_frame_id = name
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
            (tf.transform.rotation.x, tf.transform.rotation.y,
             tf.transform.rotation.z, tf.transform.rotation.w) = map(float, quat)
            tfs.append(tf)

        s = self.pursuer.state
        add(args.pursuer, s.position, s.attitude)
        for d in self.targets.values():
            add(d.name, d.position, d.quat())
        msg = TFMessage(transforms=tfs)
        self.pub_tf.publish(msg)
        if self.spectator is not None:
            # In /sim/poses only (the spectator node checks that its model exists and publishes
            # its own TF), as Gazebo lists the spectator_camera model among the world's models.
            add(self.spectator.name, self.spectator.position, self.spectator.quat)
            msg = TFMessage(transforms=tfs)
        self.pub_poses.publish(msg)

    # -- loop -------------------------------------------------------------------------------
    def run(self):
        self.timeline.play()
        dt = 1.0 / PHYSICS_HZ
        n = 0
        wall0 = time.time()
        report_wall, report_sim = wall0, 0.0
        prof = {k: 0.0 for k in ("models", "physics", "render", "ros", "sleep")}
        clock = time.perf_counter
        cprof, cprof_start = None, int(20 * PHYSICS_HZ)
        q_err = 0.0
        while app.is_running() and not self.stop_requested:
            if args.cprofile and n == cprof_start:
                import cProfile
                cprof = cProfile.Profile()
                cprof.enable()
            elif cprof and n == cprof_start + args.cprofile:
                import pstats
                cprof.disable()
                pstats.Stats(cprof, stream=sys.stdout).sort_stats("tottime").print_stats(40)
                pstats.Stats(cprof, stream=sys.stdout).sort_stats("cumulative").print_stats(40)
                sys.stdout.flush()
                cprof = None
            t0 = clock()
            self._drain_inbox()
            render = n % args.render_every == 0
            spin = render and args.propeller_every > 0 and (n // args.render_every) % args.propeller_every == 0
            for d in self.targets.values():
                d.step(dt)
                if render:
                    d.apply()
            if spin and self.targets:
                # One USD change notification for every rotor (each separate write wakes the Kit
                # listeners: ~130 us per rotor measured without the block).
                with Sdf.ChangeBlock():
                    for d in self.targets.values():
                        d.props.apply()
            if self.gimbal:
                self.gimbal.step(dt)
                if render:
                    self.gimbal.apply()
            if render and self.spectator is not None:
                self.spectator.before_render(n // args.render_every)
            t1 = clock()
            self.world.step(render=render)
            t2 = clock()
            n += 1
            t = self.world.current_time
            self.pub_clock.publish(Clock(clock=stamp_of(t)))
            if n % args.tf_every == 0:
                self._publish_truth(t)
            if self.gimbal and n % 2 == 0:   # 125 Hz (the controller uses up to 100 Hz)
                self.pub_joints.publish(self.gimbal.joint_state(stamp_of(t)))
            t3 = clock()
            if args.rtf > 0:
                ahead = t / args.rtf - (time.time() - wall0)
                if ahead > 0:
                    time.sleep(ahead)
            if args.profile:
                prof["models"] += t1 - t0
                prof["render" if render else "physics"] += t2 - t1
                prof["ros"] += t3 - t2
                prof["sleep"] += clock() - t3
            if not args.pegasus_state and args.usd_physics and n % 125 == 0:
                # fast_update_state vs the USD attitude Pegasus used (1 - |q1.q2|, 0 = same)
                prim = self.stage.GetPrimAtPath(f"/World/{args.pursuer}/body")
                gq = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractRotationQuat()
                q_usd = np.array([*gq.GetImaginary(), gq.GetReal()])
                q_err = max(q_err, 1.0 - abs(float(np.dot(q_usd, self.pursuer.state.attitude))))
            if self.spectator is not None and n % int(PHYSICS_HZ) == 0:
                self.spectator.poll()
            if n % int(5 * PHYSICS_HZ) == 0:
                now = time.time()
                rtf = (t - report_sim) / max(now - report_wall, 1e-6)
                report_wall, report_sim = now, t
                p = self.pursuer.state.position
                print(f"[aero_world] t={t:7.1f}s RTF={rtf:4.2f} {args.pursuer}=({p[0]:.1f},{p[1]:.1f},{p[2]:.1f})"
                      f" alvos={len(self.targets)}" + (f" q_err={q_err:.1e}" if q_err > 1e-6 else ""), flush=True)
                q_err = 0.0
                if args.profile:
                    # ms per physics step; render: per rendered step (1 in --render-every)
                    steps, renders = 5 * PHYSICS_HZ, 5 * PHYSICS_HZ / args.render_every
                    print("[aero_world] perfil ms/passo: " + " ".join(
                        f"{k}={v * 1e3 / (renders if k == 'render' else steps - renders if k == 'physics' else steps):.2f}"
                        for k, v in prof.items()), flush=True)
                    prof = dict.fromkeys(prof, 0.0)
            if args.max_time > 0 and t >= args.max_time:
                break
        self.timeline.stop()
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        rclpy.try_shutdown()
        app.close()


if __name__ == "__main__":
    AeroWorld().run()
