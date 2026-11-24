"""Persistent GCS settings (~/.config/aero_gcs/settings.json)."""

import copy
import json
import os
import secrets
import threading
from pathlib import Path

CONFIG_DIR = Path(os.environ.get('AERO_GCS_CONFIG', Path.home() / '.config' / 'aero_gcs'))

STREAM_DEFAULTS = {
    'topic': '/camera',
    'codec': 'h264',          # h264 | h265 | mjpeg
    'backend': 'auto',        # auto | hardware | software
    'scale': 1.0,             # fraction of the original resolution
    'fps': 30.0,              # 0: no limit
    'bitrate': 8000,          # kbit/s (target; cap of the adaptive mode)
    'rc': 'cbr',              # cbr | vbr | cqp
    'qp': 26,                 # QP in cqp mode
    'jpeg_quality': 85,
    'gop': 30,                # frames between keyframes
    'depth_min': 0.2,
    'depth_max': 20.0,
    'adaptive': True,         # bitrate follows the measured bandwidth of the worst client
    'min_bitrate': 500,
    'max_inflight': 2,        # unconfirmed frames before dropping (latency x smoothness)
}

# Gazebo free camera ("Gazebo" view): 1280x720, without the weight of the main video.
SPECTATOR_STREAM_DEFAULTS = dict(STREAM_DEFAULTS, topic='/spectator/image', bitrate=5000,
                                 min_bitrate=400)

SIM_DEFAULTS = {
    'world': 'lawn',
    'scenery': 'none',
    'pursuer_host': 'local',  # local | remote (docker/pursuer.sh on another computer, Gazebo only)
    'pursuer_remote': '',     # user@address of the remote pursuer (empty: AERO_PI_HOST / Pi default)
    'pursuer_max_rtt_ms': 0,  # stop the simulation above this network round trip (0 = off)
    'isaac_env': 'Flat_Plane',  # Pegasus/Isaac environment (Isaac Sim only; Flat_Plane = grey tiled floor)        # none | forest_edge (config/scenery/<name>.json, Gazebo and Isaac Sim)
    'dnn_target': 'cpu',
    'tf_source': 'gazebo',
    'record': True,
    'record_mode': 'session',
    'joystick': 'app',        # app (browser) | local (USB on the server) | none
    'joy_driver': 'game_controller_node',
    'log_level': 'warn',
    'gcs_stream': True,
    'foxglove': False,
    'skip_px4_build': True,
    'gz_headless': True,
    'gz_render_api': 'opengl',
    'px4_quiet': False,
    'debug': False,
    'extra_args': '',
    'scenario': '',           # Scenario run at start (empty: none)
}

DEFAULTS = {
    'token': '',
    'http_port': 8080,
    'https_port': 8443,
    'bind': '0.0.0.0',
    'local_no_auth': True,    # a browser on the same machine needs no token
    'telemetry_hz': 30.0,
    'world_frame': 'lawn',
    'workspace': str(Path.home() / 'aero_pursuit' / 'ros2_ws'),
    'stream': STREAM_DEFAULTS,
    'stream_spectator': SPECTATOR_STREAM_DEFAULTS,
    'sim': SIM_DEFAULTS,
    'backend': 'gazebo',      # gazebo | isaac (backends.py; AERO_GCS_SIM overrides)
    'tunnel': {'autostart': False, 'named_token': '', 'hostname': ''},
    # Parameter changes made through the interface, re-applied when the node reappears
    # (e.g. after restarting the simulation): {node: {parameter: {'type', 'value'}}}.
    'param_overrides': {},
    'remember_params': True,
}


def _merge(defaults, value):
    if not isinstance(defaults, dict) or not isinstance(value, dict):
        return copy.deepcopy(value if value is not None else defaults)
    out = copy.deepcopy(defaults)
    for k, v in value.items():
        out[k] = _merge(defaults.get(k), v) if k in defaults else v
    return out


class Settings:
    """Dictionary persisted as JSON; atomic and thread-safe writing."""

    def __init__(self, path: Path = CONFIG_DIR / 'settings.json'):
        self.path = path
        self._lock = threading.Lock()
        data = {}
        if path.exists():
            try:
                data = json.loads(path.read_text())
            except (OSError, ValueError):
                data = {}
        self.data = _merge(DEFAULTS, data)
        # The repository folder was renamed px4_docker -> aero_pursuit (2026-11).
        ws = Path(self.data['workspace'])
        if 'px4_docker' in ws.parts and not ws.exists():
            renamed = Path(*['aero_pursuit' if p == 'px4_docker' else p for p in ws.parts])
            if renamed.exists():
                self.data['workspace'] = str(renamed)
        if not self.data['token']:
            self.data['token'] = secrets.token_urlsafe(18)
        self.save()

    def __getitem__(self, key):
        return self.data[key]

    def update(self, section: str, values: dict):
        with self._lock:
            if isinstance(self.data.get(section), dict):
                self.data[section].update(values)
            else:
                self.data[section] = values
        self.save()

    def set(self, key, value):
        with self._lock:
            self.data[key] = value
        self.save()

    def save(self):
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_suffix('.tmp')
            tmp.write_text(json.dumps(self.data, indent=2, ensure_ascii=False))
            os.chmod(tmp, 0o600)   # contains the access token
            tmp.replace(self.path)
