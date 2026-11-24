"""A/B of the target acceleration: ground-truth metrics per run (TF), JSON lines out."""
import json, math, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from aero_pursuit.msg import PursuitStatus, ScenarioStatus
from aero_pursuit.srv import ScenarioCommand

def pset(node, name, value):
    v = str(value).lower() if isinstance(value, bool) else str(value)
    r = subprocess.run(['ros2', 'param', 'set', node, name, v], capture_output=True, text=True)
    if 'successful' not in r.stdout: print('param', node, name, r.stdout.strip(), r.stderr.strip()[:200], file=sys.stderr)

rclpy.init(); n = Node('accel_ab')
buf = Buffer(); TransformListener(buf, n)
st = {}
n.create_subscription(PursuitStatus, '/pursuit/status', lambda m: st.__setitem__('ps', m), 10)
n.create_subscription(ScenarioStatus, '/scenario/status', lambda m: st.__setitem__('sc', m),
                      QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cli = n.create_client(ScenarioCommand, '/scenario_runner/command')

def spin(sec):
    end = time.monotonic() + sec
    while time.monotonic() < end: rclpy.spin_once(n, timeout_sec=0.05)

def run(scn, ctrl, accel, standoff=5.0):
    C = '/pursuit_controller_node'
    typ, mode = ctrl.split(':') if ':' in ctrl else (ctrl, None)
    pset(C, 'controller_type', typ)
    if mode: pset(C, 'mpc.mode', mode)
    pset(C, 'target_distance', standoff); pset(C, 'max_vel', 4.0); pset(C, 'max_acc', 4.0)
    pset(C, 'vertical_mode', 'los'); pset(C, 'target_accel.enabled', accel)
    pset('/target_estimator_node', 'kf_model', 'singer' if accel else 'cv')
    cli.wait_for_service(timeout_sec=30)
    f = cli.call_async(ScenarioCommand.Request(command='start', path=scn)); rclpy.spin_until_future_complete(n, f, timeout_sec=30)
    st.pop('sc', None)
    t0 = time.monotonic(); seen_run = False
    rng, az, el, trk, cnt = [], [], [], 0, 0
    nxt = time.monotonic()
    while time.monotonic() - t0 < 600:
        rclpy.spin_once(n, timeout_sec=0.02)
        sc = st.get('sc')
        if sc is None: continue
        if sc.state_name == 'RUNNING': seen_run = True
        if seen_run and sc.state_name in ('FINISHED', 'IDLE', 'STOPPED'): break
        if not (sc.state_name == 'RUNNING' and sc.engaged) or time.monotonic() < nxt: continue
        nxt = time.monotonic() + 0.05
        try:
            t = buf.lookup_transform('camera_link', 'X4', rclpy.time.Time())
            a = buf.lookup_transform('lawn', 'X4', rclpy.time.Time()); b = buf.lookup_transform('lawn', 'x500_depth_0', rclpy.time.Time())
        except Exception:
            continue
        p = t.transform.translation; A = a.transform.translation; B = b.transform.translation
        d = math.dist((A.x, A.y, A.z), (B.x, B.y, B.z))
        rng.append(d - standoff); az.append(math.degrees(math.atan2(p.y, p.x))); el.append(math.degrees(math.atan2(p.z, math.hypot(p.x, p.y))))
        ps = st.get('ps'); cnt += 1; trk += 1 if ps and ps.mode == 1 else 0
    i0 = next((i for i, x in enumerate(rng) if abs(x) < 1.0), len(rng))
    rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')
    return dict(scenario=scn.split('/')[-1], ctrl=ctrl, accel=accel, n=len(rng), range_rms=rms(rng), range_max=max(map(abs, rng)) if rng else float('nan'),
                az_rms=rms([x for x in az if abs(x) < 90]), el_rms=rms([x for x in el if abs(x) < 90]), behind=sum(abs(x) >= 90 for x in az) / max(1, len(az)),
                tracking=trk / max(1, cnt), steady_from=i0 / 20.0, range_rms_ss=rms(rng[i0:]), range_max_ss=max(map(abs, rng[i0:])) if rng[i0:] else float('nan'),
                az_rms_ss=rms([x for x in az[i0:] if abs(x) < 90]), el_rms_ss=rms([x for x in el[i0:] if abs(x) < 90]))

if __name__ == '__main__':
    plan = json.loads(sys.argv[1]); out = sys.argv[2]
    for scn, ctrl, accel in plan:
        r = run(scn, ctrl, accel)
        print(json.dumps(r), flush=True)
        with open(out, 'a') as f: f.write(json.dumps(r) + '\n')
