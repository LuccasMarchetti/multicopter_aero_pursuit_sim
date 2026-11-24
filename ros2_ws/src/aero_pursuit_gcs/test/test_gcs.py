"""Tests of the GCS server logic (no running ROS or network)."""

import asyncio
import math
import struct

import pytest

from aero_pursuit_gcs import video_hub
from aero_pursuit_gcs.settings import Settings
from aero_pursuit_gcs.sim_control import SimControl


class FakeWS:
    closed = False

    def __init__(self):
        self.sent = []

    async def send_bytes(self, b):
        self.sent.append(b)

    async def send_str(self, s):
        pass


class FakeTransport:
    def __init__(self, buffered=0):
        self.buffered = buffered

    def get_write_buffer_size(self):
        return self.buffered


@pytest.fixture
def settings(tmp_path):
    return Settings(tmp_path / 'settings.json')


def packet(seq, key):
    head = video_hub.HEADER.pack(video_hub.MAGIC, 1, 1 if key else 0, 1, 64, 48, 64, 48, seq, 3,
                                 0, 0, 0, 0)
    return head + b'abc'


def test_header_matches_cpp_layout():
    assert video_hub.HEADER.size == 64
    f = video_hub.HEADER.unpack(packet(9, True)[:64])
    assert f[video_hub.F_SEQ] == 9 and f[video_hub.F_LEN] == 3


def test_settings_persist_and_token(settings, tmp_path):
    assert len(settings['token']) >= 20
    settings.update('stream', {'bitrate': 1234})
    again = Settings(tmp_path / 'settings.json')
    assert again['stream']['bitrate'] == 1234
    assert again['token'] == settings['token']
    assert again['stream']['codec'] == 'h264'   # defaults kept


def test_client_waits_for_keyframe_then_limits_inflight(settings):
    async def run():
        hub = video_hub.VideoHub(settings, lambda m: asyncio.sleep(0))
        ws = FakeWS()
        c = video_hub.VideoClient(hub, ws, FakeTransport(), 'test')
        c.enabled = True
        assert c.offer(packet(0, False), 0, False)      # no keyframe: skip, it is not congestion
        assert c.win_skip == 1 and c.win_drop == 0
        assert c.offer(packet(1, True), 1, True)
        await asyncio.sleep(0)
        assert c.offer(packet(2, False), 2, False)
        await asyncio.sleep(0)
        # max_inflight=2 without acks: the 3rd is dropped and the client waits for a new keyframe.
        assert not c.offer(packet(3, False), 3, False)
        assert c.need_key and c.win_drop == 1
        c.on_ack(1)
        c.on_ack(2)
        assert not c.inflight
        assert c.offer(packet(4, True), 4, True)
        await asyncio.sleep(0)
        assert len(ws.sent) == 3
        c.close()
    asyncio.run(run())


def test_client_drops_when_socket_buffer_full(settings):
    async def run():
        hub = video_hub.VideoHub(settings, lambda m: asyncio.sleep(0))
        c = video_hub.VideoClient(hub, FakeWS(), FakeTransport(buffered=10 << 20), 'test')
        c.enabled = True
        assert not c.offer(packet(1, True), 1, True)
        c.close()
    asyncio.run(run())


def test_adaptive_bitrate_backs_off_and_recovers(settings):
    hub = video_hub.VideoHub(settings, lambda m: asyncio.sleep(0))
    sent = []
    hub._send_line = sent.append

    class C:
        enabled = True
    congested = {'drop_fps': 5, 'ack_p95_ms': 900, 'goodput_kbps': 2000}
    hub._adapt([(C(), congested)])
    assert hub.effective_bitrate == 1500                      # 75% of the measured bandwidth
    clean = {'drop_fps': 0, 'ack_p95_ms': 20, 'goodput_kbps': 1500}
    for _ in range(30):
        hub._adapt([(C(), clean)])
    assert hub.effective_bitrate == settings['stream']['bitrate']   # back to the cap
    assert sent and sent[-1].startswith('cfg bitrate=')


def test_cfg_line_uses_dot_decimal(settings):
    hub = video_hub.VideoHub(settings, lambda m: asyncio.sleep(0))
    settings.update('stream', {'scale': 0.5, 'codec': 'mjpeg', 'jpeg_quality': 70})
    line = hub._cfg_line()
    assert 'scale=0.5000' in line and 'codec=mjpeg' in line and 'quality=70' in line
    assert line.endswith('\n') and ',' not in line


def test_to_jsonable_sanitizes_nan():
    ros_bridge = pytest.importorskip('aero_pursuit_gcs.ros_bridge')
    out = ros_bridge.to_jsonable({'a': math.nan, 'b': [1.0, math.inf], 'c': b'x' * 100})
    assert out['a'] is None and out['b'] == [1.0, None] and out['c'] == '<100 bytes>'


def test_param_value_respects_declared_type():
    ros_bridge = pytest.importorskip('aero_pursuit_gcs.ros_bridge')
    pv = ros_bridge.py_to_param_value('double', 2)
    assert ros_bridge.param_value_to_py(pv) == 2.0 and isinstance(ros_bridge.param_value_to_py(pv), float)
    assert ros_bridge.param_value_to_py(ros_bridge.py_to_param_value('int', 2.6)) == 3
    assert ros_bridge.param_value_to_py(ros_bridge.py_to_param_value('string[]', ['a'])) == ['a']


def test_launch_args_cover_form(settings, tmp_path):
    settings.set('workspace', str(tmp_path))
    sim = SimControl(settings, lambda m: asyncio.sleep(0))
    cfg = dict(settings['sim'], joystick='local', record=False, extra_args='foo:=bar')
    args = sim.launch_args(cfg)
    assert 'joystick:=true' in args and 'record:=false' in args and 'foo:=bar' in args
    env = sim.env(dict(cfg, foxglove=False, skip_px4_build=True))
    assert env['FOXGLOVE'] == '0' and env['SKIP_PX4_BUILD'] == '1'


def test_study_list_and_validation(settings, tmp_path):
    ws = tmp_path / 'ros2_ws'
    ws.mkdir()
    (tmp_path / 'scenarios/studies').mkdir(parents=True)
    (tmp_path / 'scenarios/tuning').mkdir(parents=True)
    (tmp_path / 'scenarios/studies/a.yaml').write_text('name: a')
    (tmp_path / 'scenarios/tuning/tune_x.yaml').write_text('name: x')
    (tmp_path / 'scenarios/tuning/s1.yaml').write_text('name: s1')  # scenario, not a tuning spec
    settings.set('workspace', str(ws))
    sim = SimControl(settings, lambda m: asyncio.sleep(0))
    assert sim.studies() == ['scenarios/studies/a.yaml', 'scenarios/tuning/tune_x.yaml']
    with pytest.raises(RuntimeError):
        asyncio.run(sim.run_study('../../etc/passwd'))
    with pytest.raises(RuntimeError):
        sim.cancel()


def test_migrate_drops_removed_controller_overrides():
    from aero_pursuit_gcs.param_memory import apply_order, migrate_overrides
    ov = {'/pursuit_controller_node': {
        'controller_type': {'type': 'string', 'value': 'lqr'},
        'mpc.lag.tau': {'type': 'double', 'value': 0.3},
        'ibvs.weight_image': {'type': 'double', 'value': 100.0},
        'mpc.params.horizon': {'type': 'double', 'value': 12.0},
        'output': {'type': 'string', 'value': 'motors'},
        'max_vel': {'type': 'double', 'value': 3.0}}}
    assert migrate_overrides(ov)
    assert set(ov['/pursuit_controller_node']) == {'mpc.params.horizon', 'max_vel'}  # output: removed
    assert [k for k, _ in apply_order(ov['/pursuit_controller_node'])] == list(ov['/pursuit_controller_node'])
    assert not migrate_overrides(ov)


def test_versioned_assets(tmp_path, monkeypatch):
    """The page loads its modules under /a/<version>/ and the version follows the files."""
    import asyncio
    import time
    from aiohttp import web
    from aiohttp.test_utils import TestClient, TestServer
    import aero_pursuit_gcs.main as m
    (tmp_path / 'js').mkdir()
    (tmp_path / 'css').mkdir()
    (tmp_path / 'js' / 'app.js').write_text('import "./b.js";')
    (tmp_path / 'js' / 'b.js').write_text('')
    (tmp_path / 'css' / 'app.css').write_text('')
    (tmp_path / 'secret.txt').write_text('x')
    (tmp_path / 'index.html').write_text('<link href="/css/app.css"><script src="/js/app.js"></script>')
    monkeypatch.setattr(m, 'web_dir', lambda: tmp_path)
    cls = next(v for v in vars(m).values() if isinstance(v, type) and hasattr(v, 'versioned_asset'))

    async def run():
        app = web.Application()
        app.router.add_get('/', lambda r: cls.index(None, r))
        app.router.add_get('/a/{ver}/{tail:.+}', lambda r: cls.versioned_asset(None, r))
        async with TestClient(TestServer(app)) as c:
            html = await (await c.get('/')).text()
            v1 = m.asset_version()
            assert f'/a/{v1}/js/app.js' in html and f'/a/{v1}/css/app.css' in html
            assert (await c.get(f'/a/{v1}/js/b.js')).status == 200
            assert (await c.get(f'/a/{v1}/../secret.txt')).status == 404
            time.sleep(0.01)
            (tmp_path / 'js' / 'b.js').write_text('// changed')
            assert m.asset_version() != v1
    asyncio.run(run())


def test_to_jsonable_numpy_scalars():
    """float32[3] fields arrive as numpy arrays: numbers (not strings), NaN as null."""
    import numpy as np
    from aero_pursuit_gcs.ros_bridge import to_jsonable
    out = to_jsonable(np.array([0.5, float('nan'), 2.0], dtype=np.float32))
    assert out == [0.5, None, 2.0] and all(v is None or isinstance(v, float) for v in out)
    assert to_jsonable(np.int32(3)) == 3


def test_isaac_backend_args_and_agent(settings, tmp_path):
    """Isaac backend: launch args of isaac_world.launch.xml, and the
    host agent client refusing shell syntax (isaac/host_agent.py)."""
    import subprocess
    import sys
    import time
    from pathlib import Path
    from aero_pursuit_gcs.sim_isaac import IsaacSimControl
    sim = IsaacSimControl(settings, None)
    cfg = dict(settings['sim'], record=False)
    args = sim.launch_args(cfg)
    assert 'record:=false' in args and not any(a.startswith(('controller:=', 'px4_output:=', 'camera_mount:=')) for a in args)
    assert not any(a.startswith('world:=') for a in args)
    assert sim.world_args(cfg) == ['--env', 'Flat_Plane']
    assert 'gz_headless' in sim.status()['unsupported']
    agent = Path(__file__).resolve().parents[4] / 'isaac' / 'host_agent.py'
    if not agent.exists():
        pytest.skip('isaac/host_agent.py fora do repositório')
    sock = tmp_path / 'h.sock'
    sim.agent = str(sock)
    proc = subprocess.Popen([sys.executable, str(agent), str(sock)])

    def run(*a):
        return subprocess.run(sim.host_cmd(*a), capture_output=True, text=True,
                              cwd=Path(__file__).parents[1])
    try:
        for _ in range(50):
            if sock.exists():
                break
            time.sleep(0.05)
        assert run('down;', 'rm').returncode == 2
        assert run('shell').returncode == 2
        assert oct(sock.stat().st_mode & 0o777) == '0o600'
    finally:
        proc.terminate()
