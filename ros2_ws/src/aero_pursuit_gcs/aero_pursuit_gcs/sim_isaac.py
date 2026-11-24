"""Isaac Sim backend of the GCS. Two sides, like Gazebo has one:

- the Isaac world (Isaac Sim + Pegasus + PX4 SITL, isaac/aero_world.py) runs on the HOST with the
  Python of Isaac Sim: isaac/host.sh world/down, called directly (native GCS) or through the host
  agent (GCS in a container, docker mode);
- the ROS side (isaac_world.launch.xml) runs WHERE THE GCS RUNS, from the same workspace that
  `aero build` compiles (ros2_ws/isaac_ros.sh), exactly like the Gazebo launch (sim_manager.sh).

docs/ISAAC_SIM.md, backends.py."""

import os
import shlex
import sys
from pathlib import Path

from .sim_control import PROCESSES, SimControl, bool_arg

ISAAC_PROCESSES = {
    'isaac': (lambda c, a: 'aero_world.py' in a and c not in ('bash', 'python.sh') and
              not c.startswith('systemd'), 'Isaac Sim (aero_world.py)'),
    'px4': PROCESSES['px4'],
    'xrce': PROCESSES['xrce'],
    'container': (lambda c, a: 'component_container' in a and
                  '__node:=aero_isaac_container' in a, 'aero_pursuit (container)'),
    'launch': PROCESSES['launch'],
    'foxglove': PROCESSES['foxglove'],
}

# Launch arguments of isaac_world.launch.xml taken from the form (name in the form -> launch).
ISAAC_LAUNCH_ARGS = {
                     'dnn_target': 'dnn_target', 'record_mode': 'record_mode',
                     'log_level': 'log_level'}
ISAAC_BOOL_ARGS = ('record', 'gcs_stream', 'foxglove')


class IsaacSimControl(SimControl):
    """Isaac Sim + Pegasus + PX4 on this computer (docs/ISAAC_SIM.md). World: isaac/host.sh, directly
    (native GCS) or through the host agent (GCS in a container: isaac/host_agent.py, Unix socket
    AERO_ISAAC_AGENT; host_client.py). ROS side: ros2_ws/isaac_ros.sh here. Studies run here too
    (same DDS domain) with run_study.py --external-sim."""

    backend = 'isaac'
    processes = ISAAC_PROCESSES
    log_files = ['ros', 'world']
    unsupported = ('world', 'tf_source', 'joystick', 'joy_driver',
                   'skip_px4_build', 'gz_headless', 'gz_render_api', 'px4_quiet', 'debug',
                   # Remote pursuer: isaac_world.launch.xml has no remote mode yet, and the Pi
                   # (LAN) and the Isaac computer (Tailscale) share no DDS network.
                   'pursuer_host', 'pursuer_remote', 'pursuer_max_rtt_ms')

    @staticmethod
    def remote(cfg) -> bool:
        return False

    def __init__(self, settings, broadcast, scenarios=None):
        super().__init__(settings, broadcast, scenarios)
        from .backends import ISAAC_STATE
        self.log_dir = Path(os.environ.get('AERO_GCS_LOG_DIR', str(ISAAC_STATE / 'logs')))
        self.agent = os.environ.get('AERO_ISAAC_AGENT', '')

    def host_cmd(self, *args) -> list:
        """isaac/host.sh <args> on the host: through the agent from a GCS container (plain words
        only: the agent refuses the rest), or directly when the GCS runs natively (backends.py)."""
        if not (self.agent and Path(self.agent).exists()):
            from .backends import isaac_dir
            return ['bash', str(isaac_dir(self.ws) / 'host.sh'), *map(str, args)]
        return [sys.executable, '-m', 'aero_pursuit_gcs.host_client', self.agent, *map(str, args)]

    def launch_args(self, cfg) -> list:
        args = [f'{arg}:={cfg[k]}' for k, arg in ISAAC_LAUNCH_ARGS.items() if cfg.get(k) not in (None, '')]
        args += [f'{k}:={bool_arg(cfg.get(k))}' for k in ISAAC_BOOL_ARGS]
        return args + shlex.split(cfg.get('extra_args') or '')

    def world_args(self, cfg) -> list:
        args = []
        if cfg.get('isaac_env'):
            args += ['--env', cfg['isaac_env']]
        if cfg.get('scenery', 'none') not in ('', 'none'):
            args += ['--scenery', cfg['scenery']]
        return args

    def running(self) -> bool:
        return bool(self._procs.get('isaac') or self._procs.get('container'))

    async def start(self, cfg: dict):
        if self.busy:
            raise RuntimeError(f'busy: {self.busy}')
        scenario = cfg.pop('scenario', '') or ''
        self.settings.update('sim', cfg)
        cfg = dict(self.settings['sim'])
        if self._procs.get('container'):
            raise RuntimeError('simulation already running: stop it first')
        args = self.launch_args(cfg)
        if scenario:
            args.append(f'scenario:={scenario}')
        # The world runs on the host and outlives a GCS restart; the ROS side runs here and does
        # not (docker mode): with the world already up, only the ROS side is started again.
        code = 0
        if not (self._procs.get('isaac') or self._procs.get('px4')):
            code = await self._run('start', self.host_cmd('world', *self.world_args(cfg)), timeout=60)
        if code == 0:
            code = await self._run('start', [str(self.ws / 'isaac_ros.sh'), 'start', *args], timeout=30)
        return code

    async def stop(self):
        if self.busy and self.busy != 'stop':
            raise RuntimeError(f'busy: {self.busy}')
        await self._run('stop', [str(self.ws / 'isaac_ros.sh'), 'stop'], timeout=30)
        return await self._run('stop', self.host_cmd('down'), timeout=60)

    async def run_study(self, path, method: str = 'nm', max_evals: int = 0):
        if self.busy:
            raise RuntimeError(f'busy: {self.busy}')
        paths, tuning = self._study_paths(path)
        if not self.running():
            raise RuntimeError('start the simulation first (the study uses the Isaac one: --external-sim)')
        repo = self.ws.parent
        argv = ['ros2', 'run', 'aero_pursuit', 'tune_study.py' if tuning else 'run_study.py',
                *[str(repo / p) for p in paths], '--external-sim']
        if tuning:
            argv += ['--method', method if method in ('nm', 'bo') else 'nm']
            if max_evals > 0:
                argv += ['--max-evals', str(int(max_evals))]
        self.memory_logs['study'].clear()
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        old_ws, self.ws = self.ws, repo      # cwd: repository root (datasets/, scenarios/)
        try:
            return await self._run('study', argv, env=env, log_name='study', timeout=24 * 3600)
        finally:
            self.ws = old_ws

    async def px4(self, command: str):
        return await self._run('px4', self.host_cmd('px4', *shlex.split(command)), timeout=15)

    def run_dir(self) -> Path:
        return self.log_dir
