"""Pursuer on another computer (Raspberry Pi 5, docs/RASPBERRY.md), driven from the GCS.

The simulator side starts with AERO_PURSUER_HOST=remote (no pursuer nodes, JPEG/PNG images);
this module starts the aero-pursuer container on the remote host with docker/pursuer.sh, follows
its log, and polls the host over SSH: container state, CPU/memory, board temperature and the
network round trip (TCP connect to the SSH port, so no ping binary is needed)."""

import asyncio
import collections
import logging
import os
import shlex
import time
from pathlib import Path

log = logging.getLogger('gcs.pursuer')

DEFAULT_HOST = os.environ.get('AERO_PI_HOST', '')   # no built-in computer: the form or the env
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3']

# Form fields (settings['sim']) -> pursuer.launch.xml arguments.
LAUNCH_ARGS = {
               'dnn_target': 'dnn_target', 'log_level': 'log_level', 'world': 'world_name'}

# One SSH round trip: container state + stats + temperature, one line each.
PROBE = ("docker inspect -f '{{.State.Running}} {{.State.StartedAt}}' aero-pursuer 2>/dev/null || echo absent; "
         "docker stats --no-stream --format '{{.CPUPerc}} {{.MemUsage}}' aero-pursuer 2>/dev/null || echo -; "
         "cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo -")


class RemotePursuer:
    def __init__(self, repo: Path, log_fn):
        self.script = repo / 'docker' / 'pursuer.sh'
        self._log = log_fn                     # SimControl._log(name, line)
        self._follow = None                    # `pursuer.sh logs` process
        self._high = 0                         # consecutive polls over the latency limit
        self.state = {'host': '', 'reachable': None, 'rtt_ms': None, 'rtt_med_ms': None, 'running': False,
                      'started': '', 'cpu': '', 'mem': '', 'temp_c': None, 'error': '', 't': 0.0}
        self.rtt_hist = collections.deque(maxlen=12)

    @staticmethod
    def host(cfg) -> str:
        host = (cfg.get('pursuer_remote') or '').strip() or DEFAULT_HOST
        if not host:
            raise RuntimeError('remote pursuer: fill in the host (user@IP) in the "Remote host" field')
        return host

    def launch_args(self, cfg) -> list:
        args = [f'{arg}:={cfg[k]}' for k, arg in LAUNCH_ARGS.items() if cfg.get(k) not in (None, '')]
        return args

    def env(self, cfg) -> dict:
        return dict(os.environ)

    async def _exec(self, argv, timeout=20.0, env=None):
        proc = await asyncio.create_subprocess_exec(
            *argv, env=env, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
            start_new_session=True)
        try:
            out, _ = await asyncio.wait_for(proc.communicate(), timeout)
        except asyncio.TimeoutError:
            proc.kill()
            return -1, 'tempo esgotado'
        return proc.returncode, out.decode(errors='replace')

    # -- lifecycle ---------------------------------------------------------------------------
    async def start(self, cfg) -> int:
        host = self.host(cfg)
        argv = [str(self.script), 'run', host, *self.launch_args(cfg)]
        self._log('pursuer', f'$ {" ".join(shlex.quote(a) for a in argv)}')
        code, out = await self._exec(argv, timeout=40, env=self.env(cfg))
        for line in out.splitlines():
            self._log('pursuer', line)
        self._log('pursuer', f'remote pursuer on {host}: {"ok" if code == 0 else f"failed ({code})"}')
        if code == 0:
            self._start_follow(host)
        return code

    async def stop(self, cfg) -> int:
        self._stop_follow()
        host = self.host(cfg)
        code, out = await self._exec([str(self.script), 'stop', host], timeout=20)
        self._log('pursuer', f'remote pursuer stopped on {host} ({code}) {out.strip()}')
        self.state['running'] = False
        return code

    def _start_follow(self, host):
        self._stop_follow()

        async def follow():
            await asyncio.sleep(1.0)
            proc = await asyncio.create_subprocess_exec(
                str(self.script), 'logs', host, stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT, start_new_session=True)
            self._follow_proc = proc
            async for raw in proc.stdout:
                self._log('pursuer', raw.decode(errors='replace').rstrip())
            await proc.wait()
        self._follow_proc = None
        self._follow = asyncio.ensure_future(follow())

    def _stop_follow(self):
        proc = getattr(self, '_follow_proc', None)
        if proc and proc.returncode is None:
            proc.kill()
        if self._follow:
            self._follow.cancel()
        self._follow = None

    # -- monitoring --------------------------------------------------------------------------
    async def _rtt(self, host) -> float | None:
        addr = host.rsplit('@', 1)[-1]
        t0 = time.monotonic()
        try:
            _, w = await asyncio.wait_for(asyncio.open_connection(addr, 22), 2.0)
        except (OSError, asyncio.TimeoutError):
            return None
        rtt = (time.monotonic() - t0) * 1000.0
        w.close()
        return rtt

    async def poll(self, cfg) -> dict:
        host = self.host(cfg)
        st = self.state
        st['host'] = host
        rtt = await self._rtt(host)
        st['reachable'] = rtt is not None
        if rtt is not None:
            self.rtt_hist.append(rtt)
            st['rtt_ms'] = round(rtt, 1)
            st['rtt_med_ms'] = round(sorted(self.rtt_hist)[len(self.rtt_hist) // 2], 1)
        else:
            st['rtt_ms'] = st['rtt_med_ms'] = None
            st.update(running=False, error='host unreachable (port 22)')
            st['t'] = time.time()
            return st
        code, out = await self._exec([*SSH, host, PROBE], timeout=8)
        lines = out.strip().splitlines()
        if code != 0 or len(lines) < 3:
            st['error'] = (out.strip().splitlines() or ['ssh failed'])[-1][:160]
        else:
            run = lines[0].split()
            st['running'] = run[:1] == ['true']
            st['started'] = run[1] if len(run) > 1 and st['running'] else ''
            stats = lines[1].split(None, 1)
            st['cpu'], st['mem'] = (stats + [''])[:2] if stats != ['-'] else ('', '')
            st['temp_c'] = round(int(lines[2]) / 1000, 1) if lines[2].strip().isdigit() else None
            st['error'] = ''
        st['t'] = time.time()
        return st

    def over_limit(self, limit_ms: float, polls: int = 3) -> bool:
        """True after `polls` consecutive polls with the round trip above `limit_ms` (0 = off)."""
        rtt = self.state.get('rtt_ms')
        if limit_ms <= 0 or rtt is None:
            self._high = 0
            return False
        self._high = self._high + 1 if rtt > limit_ms else 0
        return self._high >= polls
