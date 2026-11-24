"""Simulation control from the GCS: start/stop (sim_manager.sh), process
state, live logs, workspace build and PX4 console.

Two backends: SimControl here (Gazebo, sim_manager.sh in the workspace) and IsaacSimControl
in sim_isaac.py (Isaac Sim through isaac/host.sh; backends.py picks one)."""

import asyncio
import collections
import logging
import os
import re
import shlex
import signal
import time
from pathlib import Path

from .remote_pursuer import RemotePursuer

log = logging.getLogger('gcs.sim')

ANSI = re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')
CLK_TCK = os.sysconf('SC_CLK_TCK')

# name -> (function(comm, cmdline) -> bool, label)
PROCESSES = {
    'gazebo': (lambda c, a: re.match(r'^(ruby\S* \S*/)?gz sim', a) is not None, 'Gazebo'),
    'px4': (lambda c, a: c == 'px4', 'PX4 SITL'),
    'xrce': (lambda c, a: c == 'MicroXRCEAgent', 'MicroXRCEAgent'),
    'container': (lambda c, a: 'component_container' in a and
                  '__node:=aero_pursuit_container' in a, 'aero_pursuit (container)'),
    'launch': (lambda c, a: re.search(r'ros2 launch aero_pursuit ', a) is not None, 'ros2 launch'),
    'joy': (lambda c, a: '__node:=joy_node' in a, 'joy (USB local)'),
    'foxglove': (lambda c, a: c == 'foxglove_bridge', 'Foxglove bridge'),
}

LOG_FILES = ['aero_pursuit', 'gazebo_px4', 'gazebo', 'px4', 'xrce_agent', 'foxglove',
             'gz_pose_tf']


def bool_arg(v):
    return 'true' if v else 'false'


class SimControl:
    backend = 'gazebo'
    processes = PROCESSES
    log_files = LOG_FILES
    # Form fields (settings['sim']) this backend ignores: hidden in the Simulation tab.
    unsupported = ('isaac_env',)
    choices = None                  # simulator selector (backends.py), set by the server

    def __init__(self, settings, broadcast, scenarios=None):
        self.settings = settings
        self.scenarios = scenarios
        self.broadcast = broadcast
        self.ws = Path(settings['workspace']).expanduser()
        self.log_root = self.ws.parent / 'logs'
        self.busy = ''                  # action in progress (start/stop/build)
        self.last_result = ''
        self._cpu_prev = {}
        self._procs = {}
        self.memory_logs = {'gcs': collections.deque(maxlen=2000),
                            'build': collections.deque(maxlen=4000),
                            'study': collections.deque(maxlen=4000)}
        self._proc = None               # process of the action in progress (cancellable)
        self.subscribers = {}           # ws -> log name
        self._offsets = {}
        self.memory_logs['pursuer'] = collections.deque(maxlen=4000)
        self.pursuer = RemotePursuer(self.ws.parent, self._log)   # pursuer_host: remote

    # -- comando -----------------------------------------------------------------
    def launch_args(self, cfg) -> list:
        args = [
            f"dnn_target:={cfg['dnn_target']}",
            f"tf_source:={cfg['tf_source']}", f"record:={bool_arg(cfg['record'])}",
            f"record_mode:={cfg['record_mode']}",
            f"joystick:={bool_arg(cfg['joystick'] == 'local')}",
            f"joy_driver:={cfg['joy_driver']}", f"gcs_stream:={bool_arg(cfg['gcs_stream'])}",
            f"debug:={bool_arg(cfg['debug'])}",
        ]
        return args + shlex.split(cfg.get('extra_args') or '')

    def env(self, cfg) -> dict:
        env = dict(os.environ)
        env.update({
            'WORLD_NAME': cfg['world'],
            'SKIP_PX4_BUILD': '1' if cfg['skip_px4_build'] else '',
            'GZ_HEADLESS': '1' if cfg['gz_headless'] else '',
            'GZ_RENDER_API': cfg['gz_render_api'],
            'PX4_QUIET': '1' if cfg['px4_quiet'] else '',
            'LOG_LEVEL': cfg['log_level'],
            'FOXGLOVE': '1' if cfg['foxglove'] else '0',
            'AERO_SCENERY': cfg.get('scenery', 'none'),     # launch_gz_and_drones.sh
            # remote: no pursuer nodes here, JPEG/PNG images for the pursuer on another computer.
            'AERO_PURSUER_HOST': 'remote' if self.remote(cfg) else 'local',
            # Recorded in each session's replay.yaml (session_recorder) for the replay.
            'AERO_SIM_BACKEND': self.backend,
            'AERO_GIT_REV': self.git_rev(),
        })
        if not env['PX4_QUIET']:
            env.pop('PX4_QUIET')
        return env

    async def _run(self, name, argv, env=None, log_name='gcs', timeout=None):
        self.busy = name
        await self.broadcast(self.status())
        self._log(log_name, f'$ {" ".join(shlex.quote(a) for a in argv)}')
        try:
            proc = await asyncio.create_subprocess_exec(
                *argv, cwd=str(self.ws if self.ws.is_dir() else Path.home()), env=env or dict(os.environ),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
                start_new_session=True)
            self._proc = proc

            async def pump():
                async for raw in proc.stdout:
                    self._log(log_name, raw.decode(errors='replace').rstrip())
            await asyncio.wait_for(asyncio.gather(pump(), proc.wait()), timeout)
            code = proc.returncode
        except asyncio.TimeoutError:
            proc.kill()
            code = -1
        except Exception as e:      # script ausente etc.
            self._log(log_name, f'error: {e}')
            code = -2
        self._proc = None
        self.last_result = f'{name}: {"ok" if code == 0 else f"failed ({code})"}'
        self._log(log_name, self.last_result)
        self.busy = ''
        await self.broadcast(self.status())
        return code

    async def start(self, cfg: dict):
        if self.busy:
            raise RuntimeError(f'busy: {self.busy}')
        # The scenario only applies to this start (it is not saved as the form default).
        scenario = cfg.pop('scenario', '') or ''
        self.settings.update('sim', cfg)
        cfg = dict(self.settings['sim'])
        if any(self._procs.get(k) for k in ('gazebo', 'px4', 'container')):
            raise RuntimeError('simulation already running: stop it first')
        args = self.launch_args(cfg)
        env = self.env(cfg)
        if scenario:
            # SCENARIO: sim_manager passes the file to the launch (scenario_runner loads and
            # starts it).
            env['SCENARIO'] = scenario
        code = await self._run('start', [str(self.ws / 'sim_manager.sh'), 'start'] + args,
                               env=env, timeout=90)
        if code == 0 and self.remote(cfg):
            code = await self._start_pursuer(cfg)
        return code

    async def start_replay(self, overrides: dict):
        """Simulator for a session replay (replay.py, world mode): the saved form with the
        recorded world/camera/scenery, no pursuer nodes (as with a remote pursuer, but none is
        started), no recording, and the world_replay node (launch arg replay:=true)."""
        if self.busy:
            raise RuntimeError(f'busy: {self.busy}')
        cfg = dict(self.settings['sim'], **{k: v for k, v in overrides.items() if v}, record=False)
        env = self.env(cfg)
        env['AERO_PURSUER_HOST'] = 'remote'
        env['AERO_REPLAY'] = '1'            # launch_gz_and_drones.sh: world without gravity
        return await self._run('start', [str(self.ws / 'sim_manager.sh'), 'start'] + self.launch_args(cfg)
                               + ['replay:=true'], env=env, timeout=90)

    @staticmethod
    def remote(cfg) -> bool:
        return cfg.get('pursuer_host', 'local') == 'remote'

    def git_rev(self) -> str:
        """Code revision of the repository ('<hash>+' with local changes; '' outside git)."""
        import subprocess
        repo = str(self.ws.parent)
        try:
            rev = subprocess.run(['git', '-C', repo, 'rev-parse', '--short', 'HEAD'],
                                 capture_output=True, text=True, timeout=3).stdout.strip()
            dirty = subprocess.run(['git', '-C', repo, 'status', '--porcelain', '--untracked-files=no'],
                                   capture_output=True, text=True, timeout=3).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return ''
        return rev + ('+' if rev and dirty else '')

    async def _start_pursuer(self, cfg):
        self.busy = 'start (remote pursuer)'
        await self.broadcast(self.status())
        try:
            code = await self.pursuer.start(cfg)
        finally:
            self.busy = ''
        self.last_result = f'remote pursuer: {"ok" if code == 0 else f"failed ({code}), see the pursuer log"}'
        await self.broadcast(self.status())
        return code

    async def stop_pursuer(self):
        cfg = self.settings['sim']
        if self.remote(cfg) or self.pursuer.state.get('running'):
            await self.pursuer.stop(cfg)

    async def stop(self):
        if self.busy and self.busy != 'stop':
            raise RuntimeError(f'busy: {self.busy}')
        await self.stop_pursuer()
        await self._run('stop', [str(self.ws / 'sim_manager.sh'), 'stop'], timeout=60)
        # Leftovers of runs from tmux/terminal or orphans.
        return await self._run('stop', [str(self.ws / 'stop_simulation.sh')], timeout=30)

    async def build(self, packages=('aero_pursuit', 'aero_pursuit_gcs')):
        if self.busy:
            raise RuntimeError(f'busy: {self.busy}')
        self.memory_logs['build'].clear()
        script = (f'source /opt/ros/{os.environ.get("ROS_DISTRO", "jazzy")}/setup.bash && '
                  f'colcon build --packages-select {" ".join(packages)} '
                  '--event-handlers console_direct+')
        return await self._run('build', ['bash', '-c', script], log_name='build', timeout=3600)

    # -- batch studies / tuning ------------------------------------------------------
    def studies(self) -> list:
        """Study and tuning files, relative to the repository root."""
        repo = self.ws.parent
        files = sorted((repo / 'scenarios/studies').glob('*.yaml')) + \
            sorted((repo / 'scenarios/tuning').glob('tune_*.yaml'))
        return [str(p.relative_to(repo)) for p in files]

    def _study_paths(self, path) -> tuple:
        """(paths, tuning): one tuning file, or one or more studies run in cascade."""
        paths = [path] if isinstance(path, str) else list(path)
        known = self.studies()
        for p in paths:
            if p not in known:
                raise RuntimeError(f'estudo desconhecido: {p}')
        if not paths:
            raise RuntimeError('no study chosen')
        tuning = any('/tuning/' in p for p in paths)
        if tuning and len(paths) > 1:
            raise RuntimeError('tuning (tune_*.yaml) runs on its own, outside the cascade')
        return paths, tuning

    async def run_study(self, path, method: str = 'nm', max_evals: int = 0):
        """run_study.py (one study or several in cascade) or tune_study.py (tune_*.yaml); log 'study'."""
        if self.busy:
            raise RuntimeError(f'busy: {self.busy}')
        paths, tuning = self._study_paths(path)
        repo = self.ws.parent
        argv = ['python3', str(self.ws / ('tune_study.py' if tuning else 'run_study.py')),
                *[str(repo / p) for p in paths], '--keep-running']
        if tuning:
            argv += ['--method', method if method in ('nm', 'bo') else 'nm']
            if max_evals > 0:
                argv += ['--max-evals', str(int(max_evals))]
        self.memory_logs['study'].clear()
        script = (f'source /opt/ros/{os.environ.get("ROS_DISTRO", "jazzy")}/setup.bash && '
                  f'source {shlex.quote(str(self.ws / "install/setup.bash"))} && '
                  f'PYTHONUNBUFFERED=1 exec {" ".join(shlex.quote(a) for a in argv)}')
        return await self._run('study', ['bash', '-c', script], log_name='study', timeout=24 * 3600)

    def cancel(self):
        """SIGINT to the running action (studies write their summary on interrupt)."""
        if self._proc is None or self.busy != 'study':
            raise RuntimeError('no study running')
        os.killpg(self._proc.pid, signal.SIGINT)

    async def px4(self, command: str):
        return await self._run('px4', [str(self.ws / 'sim_manager.sh'), 'px4', command],
                               timeout=10)

    # -- estado ------------------------------------------------------------------
    def scan(self):
        """Scans /proc (independent of how the simulation was launched). Only this user's
        processes: on a shared computer (Isaac host) others may run their own PX4."""
        found = {k: [] for k in self.processes}
        uid = os.getuid()
        now = time.monotonic()
        cpu_now = {}
        for entry in os.scandir('/proc'):
            if not entry.name.isdigit():
                continue
            try:
                if entry.stat().st_uid != uid:
                    continue
                with open(f'/proc/{entry.name}/cmdline', 'rb') as f:
                    argv = f.read().split(b'\0')
                args = b' '.join(a for a in argv if a).decode(errors='replace')
                if not args:
                    continue
                with open(f'/proc/{entry.name}/comm') as f:
                    comm = f.read().strip()
                matches = [k for k, (fn, _) in self.processes.items() if fn(comm, args)]
                if not matches:
                    continue
                with open(f'/proc/{entry.name}/stat') as f:
                    stat = f.read().rsplit(')', 1)[1].split()
                ticks = int(stat[11]) + int(stat[12])
                rss_mb = int(stat[21]) * os.sysconf('SC_PAGE_SIZE') / 2**20
            except (OSError, IndexError, ValueError):
                continue
            pid = int(entry.name)
            cpu_now[pid] = (ticks, now)
            prev = self._cpu_prev.get(pid)
            cpu = None
            if prev and now > prev[1]:
                cpu = 100.0 * (ticks - prev[0]) / CLK_TCK / (now - prev[1])
            for k in matches:
                found[k].append({'pid': pid, 'cpu': cpu, 'rss_mb': round(rss_mb, 1)})
        self._cpu_prev = cpu_now
        self._procs = found
        return found

    def status(self):
        return {
            'op': 'sim.status',
            'backend': self.backend,
            'unsupported': list(self.unsupported),
            'busy': self.busy,
            'last_result': self.last_result,
            'processes': {k: {'label': self.processes[k][1], 'instances': v}
                          for k, v in self._procs.items()},
            'running': self.running(),
            'cfg': self.settings['sim'],
            'logs': self.available_logs(),
            'pursuer': self.pursuer.state if self.remote(self.settings['sim']) else None,
            'backends': self.choices,
        }

    def running(self) -> bool:
        return bool(self._procs.get('gazebo') or self._procs.get('container'))

    # -- logs --------------------------------------------------------------------
    def run_dir(self) -> Path:
        return (self.log_root / 'latest').resolve()

    def available_logs(self):
        d = self.run_dir()
        files = [n for n in self.log_files if (d / f'{n}.log').exists()]
        mem = [n for n in self.memory_logs
               if n != 'pursuer' or self.remote(self.settings['sim']) or self.memory_logs[n]]
        return files + mem

    def _log(self, name, line):
        line = ANSI.sub('', line)
        self.memory_logs.setdefault(name, collections.deque(maxlen=2000)).append(line)
        for ws, sub in list(self.subscribers.items()):
            if sub == name and not ws.closed:
                asyncio.ensure_future(ws.send_json({'op': 'log', 'name': name, 'lines': [line]}))

    def _tail(self, path: Path, lines=300):
        try:
            with open(path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 256 * 1024))
                data = f.read().decode(errors='replace')
            return [ANSI.sub('', s) for s in data.splitlines()[-lines:]], size
        except OSError:
            return [], 0

    async def subscribe_log(self, ws, name):
        self.subscribers[ws] = name
        if name in self.memory_logs:
            lines = list(self.memory_logs[name])[-500:]
        else:
            path = self.run_dir() / f'{name}.log'
            lines, size = self._tail(path)
            self._offsets[ws] = (path, size)
        await ws.send_json({'op': 'log', 'name': name, 'lines': lines, 'reset': True})

    def unsubscribe_log(self, ws):
        self.subscribers.pop(ws, None)
        self._offsets.pop(ws, None)

    async def poll_logs(self):
        """Follows the subscribed log files (tail -f)."""
        for ws, name in list(self.subscribers.items()):
            if ws.closed:
                self.unsubscribe_log(ws)
                continue
            if name in self.memory_logs:
                continue
            path = self.run_dir() / f'{name}.log'
            old_path, offset = self._offsets.get(ws, (path, 0))
            if old_path != path:
                offset = 0          # new run (latest changed)
            try:
                size = path.stat().st_size
                if size < offset:
                    offset = 0      # arquivo truncado
                if size == offset:
                    self._offsets[ws] = (path, offset)
                    continue
                with open(path, 'rb') as f:
                    f.seek(max(offset, size - 512 * 1024))
                    data = f.read().decode(errors='replace')
                self._offsets[ws] = (path, size)
            except OSError:
                continue
            lines = [ANSI.sub('', s) for s in data.splitlines() if s.strip()]
            if lines:
                await ws.send_json({'op': 'log', 'name': name, 'lines': lines[-500:]})
