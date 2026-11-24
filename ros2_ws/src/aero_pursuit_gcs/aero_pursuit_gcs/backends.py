"""Which simulator this GCS drives: Gazebo (sim_manager.sh) or Isaac Sim (isaac/host.sh).

Every GCS can drive both (docs/COMANDOS.md): settings['backend'], chosen in the Simulation tab
(AERO_GCS_SIM forces one). A change restarts the GCS process, because the ROS node is created
once per process and Isaac runs on its own DDS domain and transport profile (isaac/host.sh:
ROS_DOMAIN_ID 42, isaac/fastdds_shm.xml). Isaac needs an NVIDIA RTX GPU and the host install
(`aero isaac setup`); a GCS in a container reaches it through the host agent (AERO_ISAAC_AGENT,
isaac/host_agent.py)."""

import os
import shutil
import subprocess
from pathlib import Path

ISAAC_STATE = Path(os.environ.get('AERO_ISAAC_STATE', '~/.local/state/aero/isaac')).expanduser()
ISAACSIM = Path(os.environ.get('ISAACSIM_PATH', '~/isaacsim51')).expanduser()
ISAAC_DOMAIN = os.environ.get('AERO_ISAAC_DOMAIN', '42')
BACKENDS = ('gazebo', 'isaac')


def fixed() -> str:
    """Backend forced by the environment ('' = selectable)."""
    b = os.environ.get('AERO_GCS_SIM', '')
    return b if b in BACKENDS else ''


def selected(settings) -> str:
    b = fixed() or settings['backend']
    return b if b in BACKENDS else 'gazebo'


def _gpu() -> str:
    """Name of the first NVIDIA GPU ('' = none)."""
    if not shutil.which('nvidia-smi'):
        return ''
    try:
        out = subprocess.run(['nvidia-smi', '--query-gpu=name', '--format=csv,noheader'],
                             capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    return out.strip().splitlines()[0] if out.strip() else ''


def isaac_dir(workspace) -> Path:
    """isaac/ of the repository that holds this workspace."""
    return Path(workspace).expanduser().parent / 'isaac'


def availability(workspace) -> dict:
    """{backend: {'ok': bool, 'reason': str}} for the selector."""
    ws = Path(workspace).expanduser()
    gz = bool(shutil.which('gz')) and (ws / 'sim_manager.sh').is_file()
    out = {'gazebo': {'ok': gz, 'reason': '' if gz else 'Gazebo (gz) ou sim_manager.sh ausente'}}
    agent = os.environ.get('AERO_ISAAC_AGENT', '')
    if agent and Path(agent).exists():
        out['isaac'] = {'ok': True, 'reason': ''}
        return out
    gpu = _gpu()
    if 'RTX' not in gpu.upper():
        out['isaac'] = {'ok': False, 'reason': f'needs an NVIDIA RTX GPU (found: {gpu or "none"})'}
    elif not (ISAACSIM / 'python.sh').is_file():
        out['isaac'] = {'ok': False, 'reason': f'Isaac Sim not installed in {ISAACSIM} (aero isaac setup)'}
    else:
        out['isaac'] = {'ok': True, 'reason': gpu}
    return out


def apply_env(backend: str, workspace):
    """Process environment for the backend, before rclpy starts. Native Isaac joins the host's
    DDS domain and shared-memory profile; Gazebo gets back the domain the GCS started with."""
    if fixed():
        return
    base = os.environ.setdefault('AERO_GCS_BASE_DOMAIN', os.environ.get('ROS_DOMAIN_ID', ''))
    base_profile = os.environ.setdefault('AERO_GCS_BASE_PROFILE',
                                         os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', ''))
    if backend == 'isaac':
        os.environ['ROS_DOMAIN_ID'] = ISAAC_DOMAIN
        os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(isaac_dir(workspace) / 'fastdds_shm.xml')
        os.environ.setdefault('AERO_GCS_LOG_DIR', str(ISAAC_STATE / 'logs'))
        return
    for key, value in (('ROS_DOMAIN_ID', base), ('FASTRTPS_DEFAULT_PROFILES_FILE', base_profile)):
        if value:
            os.environ[key] = value
        else:
            os.environ.pop(key, None)
