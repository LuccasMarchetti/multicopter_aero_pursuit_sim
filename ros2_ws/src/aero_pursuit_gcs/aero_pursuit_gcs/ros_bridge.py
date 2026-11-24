"""GCS ROS 2 bridge: telemetry, TF, parameters, generic topics/services,
joystick and target selection. The node runs on its own multi-threaded executor; asyncio
talks to it through futures and the last received values.
"""

import json
import array
import asyncio
import logging
import math
import re
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterEvent, ParameterType, ParameterValue
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.utilities import get_message, get_service

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState, Joy, RegionOfInterest
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

log = logging.getLogger('gcs.ros')

try:
    from aero_pursuit.msg import (PursuitMetrics, PursuitStatus,
                                  ScenarioStatus, SpectatorCommand, TargetDetection,
                                  TargetTfCmd, TrackingTruth)
    HAVE_AP = True
except ImportError:  # workspace without aero_pursuit built
    HAVE_AP = False

try:
    from px4_msgs.msg import VehicleStatus
except ImportError:
    VehicleStatus = None

# Heavy types do not go to the browser as JSON (use the video panel).
HEAVY_TYPES = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
               'sensor_msgs/msg/PointCloud2', 'sensor_msgs/msg/LaserScan'}

TYPE_NAMES = {
    ParameterType.PARAMETER_BOOL: 'bool', ParameterType.PARAMETER_INTEGER: 'int',
    ParameterType.PARAMETER_DOUBLE: 'double', ParameterType.PARAMETER_STRING: 'string',
    ParameterType.PARAMETER_BYTE_ARRAY: 'bytes', ParameterType.PARAMETER_BOOL_ARRAY: 'bool[]',
    ParameterType.PARAMETER_INTEGER_ARRAY: 'int[]',
    ParameterType.PARAMETER_DOUBLE_ARRAY: 'double[]',
    ParameterType.PARAMETER_STRING_ARRAY: 'string[]',
}


def to_jsonable(value, max_array=64):
    """ROS message -> JSON structures (NaN/inf become None: JSON.parse rejects them)."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, (bool, int, str)) or value is None:
        return value
    if isinstance(value, dict):
        return {k: to_jsonable(v, max_array) for k, v in value.items()}
    if isinstance(value, (bytes, bytearray)):
        return f'<{len(value)} bytes>' if len(value) > max_array else list(value)
    if isinstance(value, (list, tuple, array.array)) or type(value).__name__ == 'ndarray':
        seq = list(value[:max_array])
        out = [to_jsonable(v, max_array) for v in seq]
        if len(value) > max_array:
            out.append(f'... +{len(value) - max_array}')
        return out
    if hasattr(value, 'get_fields_and_field_types'):
        return {name: to_jsonable(getattr(value, name), max_array)
                for name in value.get_fields_and_field_types()}
    if type(value).__module__ == 'numpy' and hasattr(value, 'item'):
        # Elements of fixed-size ROS arrays (numpy.float32 etc.): plain numbers, NaN -> None.
        return to_jsonable(value.item(), max_array)
    return str(value)


def param_value_to_py(v: ParameterValue):
    t = v.type
    if t == ParameterType.PARAMETER_BOOL:
        return v.bool_value
    if t == ParameterType.PARAMETER_INTEGER:
        return v.integer_value
    if t == ParameterType.PARAMETER_DOUBLE:
        return v.double_value if math.isfinite(v.double_value) else None
    if t == ParameterType.PARAMETER_STRING:
        return v.string_value
    if t == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(v.byte_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(v.bool_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(v.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(v.double_array_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:
        return list(v.string_array_value)
    return None


def py_to_param_value(type_name: str, value) -> ParameterValue:
    """Converts respecting the declared type (2 in a double parameter becomes 2.0)."""
    pv = ParameterValue()
    if type_name == 'bool':
        pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, bool(value)
    elif type_name == 'int':
        pv.type, pv.integer_value = ParameterType.PARAMETER_INTEGER, int(round(float(value)))
    elif type_name == 'double':
        pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
    elif type_name == 'string':
        pv.type, pv.string_value = ParameterType.PARAMETER_STRING, str(value)
    elif type_name == 'bool[]':
        pv.type, pv.bool_array_value = ParameterType.PARAMETER_BOOL_ARRAY, [bool(x) for x in value]
    elif type_name == 'int[]':
        pv.type, pv.integer_array_value = (ParameterType.PARAMETER_INTEGER_ARRAY,
                                           [int(x) for x in value])
    elif type_name == 'double[]':
        pv.type, pv.double_array_value = (ParameterType.PARAMETER_DOUBLE_ARRAY,
                                          [float(x) for x in value])
    elif type_name == 'string[]':
        pv.type, pv.string_array_value = (ParameterType.PARAMETER_STRING_ARRAY,
                                          [str(x) for x in value])
    elif type_name == 'unset':
        pv.type = ParameterType.PARAMETER_NOT_SET
    else:
        raise ValueError(f'unsupported parameter type: {type_name}')
    return pv


async def wrap_future(fut, timeout=3.0):
    """rclpy future -> await in asyncio."""
    loop = asyncio.get_running_loop()
    afut = loop.create_future()

    def done(f):
        def settle():
            if afut.done():
                return
            exc = f.exception()
            if exc:
                afut.set_exception(exc)
            else:
                afut.set_result(f.result())
        loop.call_soon_threadsafe(settle)

    fut.add_done_callback(done)
    return await asyncio.wait_for(afut, timeout)


class GenericSub:
    def __init__(self, sub, type_name):
        self.sub = sub
        self.type_name = type_name
        self.msg = None
        self.count = 0
        self.refs = 0


class GcsNode(Node):
    def __init__(self, world_frame: str):
        super().__init__('aero_gcs')
        self.world_frame = world_frame
        self.lock = threading.Lock()
        self.latest = {}          # name -> (counter, msg)
        self.clock = None         # (sim_s, wall_s)
        self.rtf = None
        self._rtf_ref = None

        # Poses in the world frame, aggregated by gcs_telemetry (C++, in the container):
        # subscribing to /tf and /clock here cost ~40% CPU (hundreds of msgs/s in Python).
        self.poses = {}
        self.poses_t = 0.0

        be = qos_profile_sensor_data
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/gcs/poses', self._on_poses, be)
        if HAVE_AP:
            self._track('status', PursuitStatus, '/pursuit/status', 10)
            self._track('metrics', PursuitMetrics, '/pursuit/metrics', 10)
            self._track('detection', TargetDetection, '/target_detection', 10)
            self._track('scenario', ScenarioStatus, '/scenario/status', latched)
            self._track('truth', TrackingTruth, '/acquisition/truth', 10)
        self._track('estimate', Odometry, '/target_3d_state_filtered', 10)
        self._track('path', Path, '/pursuit/predicted_path', 10)
        self._track('teleop', String, '/drone_joy_teleop/status', latched)
        self._px4_topics = set()     # ver discover_px4()

        self.pub_joy = self.create_publisher(Joy, '/joy', 10)
        self.pub_selection = self.create_publisher(RegionOfInterest, '/target_selection', 10)
        # Journal of the state-changing actions taken through the GCS (session_recorder keeps it
        # in the bag; the replay re-injects it at the same simulation time).
        self.pub_input_log = self.create_publisher(String, '/gcs/input_log', 100)
        self.pub_tf_cmd = (self.create_publisher(TargetTfCmd, '/target_tf_cmd', 10)
                           if HAVE_AP else None)
        self.pub_spectator = (self.create_publisher(SpectatorCommand, '/spectator/command', 10)
                              if HAVE_AP else None)
        self.param_events = []    # consumed by asyncio
        self.create_subscription(ParameterEvent, '/parameter_events', self._on_param_event, 50)

        self.generic = {}         # topic -> GenericSub
        self.publishers_cache = {}
        self.clients_cache = {}
        self.param_clients = {}

    # -- assinaturas fixas -------------------------------------------------------
    def _track(self, name, cls, topic, qos):
        def cb(msg):
            with self.lock:
                count = self.latest.get(name, (0, None))[0] + 1
                self.latest[name] = (count, msg)
        self.create_subscription(cls, topic, cb, qos)

    def discover_px4(self):
        """Subscribes to vehicle_status in whatever version PX4 publishes (_v1, _v4, ...): the
        suffix changes between PX4 versions."""
        if VehicleStatus is None:
            return
        for name, types in self.get_topic_names_and_types():
            if (re.fullmatch(r'/fmu/out/vehicle_status(_v\d+)?', name) and
                    'px4_msgs/msg/VehicleStatus' in types and name not in self._px4_topics and
                    self.count_publishers(name) > 0):
                self._px4_topics.add(name)
                self._track('px4', VehicleStatus, name, qos_profile_sensor_data)
                log.info('PX4: assinando %s', name)

    def _on_poses(self, msg: TFMessage):
        poses = {}
        sim = 0.0
        for tr in msg.transforms:
            t, q = tr.transform.translation, tr.transform.rotation
            poses[tr.child_frame_id] = [round(t.x, 4), round(t.y, 4), round(t.z, 4),
                                        round(q.x, 5), round(q.y, 5), round(q.z, 5), round(q.w, 5)]
            sim = max(sim, tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
        wall = time.monotonic()
        with self.lock:
            if self.clock is not None and sim < self.clock[0] - 0.5:
                # Simulation restarted / world reset: discards the previous state.
                log.info('simulation time went back (%.1f -> %.1f s)', self.clock[0], sim)
                self.latest.clear()
                self._rtf_ref = None
            if self._rtf_ref is None:
                self._rtf_ref = (sim, wall)
            elif wall - self._rtf_ref[1] >= 1.0:
                self.rtf = (sim - self._rtf_ref[0]) / (wall - self._rtf_ref[1])
                self._rtf_ref = (sim, wall)
            self.clock = (sim, wall)
            self.poses = poses
            self.poses_t = wall

    def _on_param_event(self, ev: ParameterEvent):
        changes = [{'name': p.name, 'value': param_value_to_py(p.value)}
                   for p in list(ev.new_parameters) + list(ev.changed_parameters)]
        if changes:
            with self.lock:
                self.param_events.append({'node': ev.node, 'changes': changes})
                del self.param_events[:-200]

    def take(self, name):
        with self.lock:
            return self.latest.get(name, (0, None))

    def pop_param_events(self):
        with self.lock:
            ev, self.param_events = self.param_events, []
        return ev

    # -- TF ----------------------------------------------------------------------
    def transforms(self) -> dict:
        """Pose of each frame in the world frame (empty if gcs_telemetry is gone)."""
        with self.lock:
            if time.monotonic() - self.poses_t > 2.0:
                return {}
            return self.poses

    # -- comandos ----------------------------------------------------------------
    def publish_joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self.pub_joy.publish(msg)

    def log_input(self, entry: dict):
        self.pub_input_log.publish(String(data=json.dumps(entry, separators=(',', ':'))))

    def publish_selection(self, x, y, w, h):
        msg = RegionOfInterest()
        msg.x_offset, msg.y_offset = max(0, int(x)), max(0, int(y))
        msg.width, msg.height = max(0, int(w)), max(0, int(h))
        self.pub_selection.publish(msg)

    def publish_tf_cmd(self, frame: str, enable: bool):
        if self.pub_tf_cmd is None:
            raise RuntimeError('aero_pursuit not built')
        msg = TargetTfCmd()
        msg.target_frame = frame
        msg.enable_tf_tracking = bool(enable)
        self.pub_tf_cmd.publish(msg)

    def publish_spectator(self, cmd: dict):
        """Free camera: mode + pose (free/look) or follow/orbit parameters."""
        if self.pub_spectator is None:
            raise RuntimeError('aero_pursuit not built')
        msg = SpectatorCommand()
        msg.mode = str(cmd.get('mode', 'free'))
        msg.target = str(cmd.get('target', ''))
        p = cmd.get('position') or [0.0, 0.0, 10.0]
        msg.position.x, msg.position.y, msg.position.z = (float(v) for v in p[:3])
        for k in ('yaw', 'pitch', 'distance', 'azimuth', 'elevation', 'orbit_rate'):
            setattr(msg, k, float(cmd.get(k, 0.0)))
        self.pub_spectator.publish(msg)

    # -- grafo ---------------------------------------------------------------------
    # list_* names: Node already has the services/publishers/... properties used by the executor.
    def list_topics(self):
        return [{'name': n, 'types': t} for n, t in sorted(self.get_topic_names_and_types())]

    def list_services(self):
        return [{'name': n, 'types': t} for n, t in sorted(self.get_service_names_and_types())]

    def list_nodes(self):
        out = []
        for name, ns in self.get_node_names_and_namespaces():
            fqn = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
            if fqn != '/aero_gcs' and not name.startswith('_'):
                out.append(fqn)
        return sorted(set(out))

    def topic_type(self, topic):
        for n, types in self.get_topic_names_and_types():
            if n == topic and types:
                return types[0]
        return None

    def _qos_for(self, topic):
        infos = self.get_publishers_info_by_topic(topic)
        if not infos:
            return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = all(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
        latched = any(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if latched and reliable
            else DurabilityPolicy.VOLATILE)

    def subscribe(self, topic, type_name=None) -> GenericSub:
        with self.lock:
            g = self.generic.get(topic)
        if g:
            g.refs += 1
            return g
        type_name = type_name or self.topic_type(topic)
        if not type_name:
            raise ValueError(f'topic {topic} has no publishers (unknown type)')
        if type_name in HEAVY_TYPES:
            raise ValueError(f'{type_name} is too heavy for JSON: use the video panel')
        cls = get_message(type_name)
        holder = GenericSub(None, type_name)

        def cb(msg, h=holder):
            h.msg = msg
            h.count += 1
        holder.sub = self.create_subscription(cls, topic, cb, self._qos_for(topic))
        holder.refs = 1
        with self.lock:
            self.generic[topic] = holder
        return holder

    def unsubscribe(self, topic):
        with self.lock:
            g = self.generic.get(topic)
            if not g:
                return
            g.refs -= 1
            if g.refs > 0:
                return
            del self.generic[topic]
        self.destroy_subscription(g.sub)

    def publish(self, topic, type_name, data):
        key = (topic, type_name)
        pub = self.publishers_cache.get(key)
        if pub is None:
            pub = self.create_publisher(get_message(type_name), topic, 10)
            self.publishers_cache[key] = pub
        msg = get_message(type_name)()
        if data:
            set_message_fields(msg, data)
        if hasattr(msg, 'header') and not data.get('header'):
            msg.header.stamp = self.get_clock().now().to_msg()
        pub.publish(msg)

    async def call_service(self, name, type_name, request, timeout=5.0):
        if not type_name:
            for n, types in self.get_service_names_and_types():
                if n == name and types:
                    type_name = types[0]
                    break
        if not type_name:
            raise ValueError(f'service {name} unavailable')
        key = (name, type_name)
        client = self.clients_cache.get(key)
        if client is None:
            client = self.create_client(get_service(type_name), name)
            self.clients_cache[key] = client
        if not client.service_is_ready():
            await asyncio.get_running_loop().run_in_executor(
                None, client.wait_for_service, 1.0)
            if not client.service_is_ready():
                raise TimeoutError(f'service {name} did not answer')
        req = get_service(type_name).Request()
        if request:
            set_message_fields(req, request)
        resp = await wrap_future(client.call_async(req), timeout)
        return to_jsonable(resp)

    # -- parameters ----------------------------------------------------------------
    def _param_client(self, node):
        c = self.param_clients.get(node)
        if c is None:
            c = AsyncParameterClient(self, node)
            self.param_clients[node] = c
        return c

    async def list_params(self, node):
        c = self._param_client(node)
        ok = await asyncio.get_running_loop().run_in_executor(None, c.wait_for_services, 1.5)
        if not ok:
            raise TimeoutError(f'{node} does not answer the parameter services')
        names = (await wrap_future(c.list_parameters())).result.names
        names = sorted(n for n in names if n not in ('use_sim_time', 'qos_overrides') and
                       not n.startswith('qos_overrides.'))
        descs = (await wrap_future(c.describe_parameters(names))).descriptors
        values = (await wrap_future(c.get_parameters(names))).values
        out = []
        for name, d, v in zip(names, descs, values):
            entry = {'name': name, 'type': TYPE_NAMES.get(v.type, 'unset'),
                     'value': param_value_to_py(v), 'description': d.description,
                     'read_only': d.read_only, 'constraints': d.additional_constraints}
            if d.floating_point_range:
                r = d.floating_point_range[0]
                entry['range'] = [r.from_value, r.to_value, r.step]
            elif d.integer_range:
                r = d.integer_range[0]
                entry['range'] = [r.from_value, r.to_value, r.step]
            out.append(entry)
        return out

    async def set_param(self, node, name, type_name, value):
        c = self._param_client(node)
        p = ParameterMsg(name=name, value=py_to_param_value(type_name, value))
        res = (await wrap_future(c.set_parameters([p]))).results[0]
        return res.successful, res.reason


class RosBridge:
    """rclpy life cycle in its own thread."""

    def __init__(self, world_frame):
        rclpy.init(args=None)
        self.node = GcsNode(world_frame)
        # One thread: the callbacks only store values (nothing blocks) and the rclpy
        # multi-threaded executor costs much more CPU per message.
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True, name='ros-spin')
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:
            if rclpy.ok():
                log.exception('ROS executor stopped')

    def shutdown(self):
        try:
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
