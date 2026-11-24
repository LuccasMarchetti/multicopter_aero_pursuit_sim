"""Operations of the Simulation and Scenarios tabs: start/stop/restart, simulator choice, remote
pursuer, studies (one or a cascade), scenario files and runs, bag playback and re-simulation.

Each handler takes the request message and returns the reply data; long actions run in the
background (`guard`) and report through sim.status."""

import asyncio
import logging

log = logging.getLogger('gcs.sim')


class SimOps:
    def __init__(self, server):
        self.server = server
        self.pursuer_polling = False

    @property
    def sim(self):
        return self.server.sim

    def routes(self) -> dict:
        r = {
            'sim.start': self.start, 'sim.save': self.save, 'sim.restart': self.restart,
            'sim.stop': self.stop, 'sim.build': self.build, 'sim.px4': self.px4,
            'sim.backend': self.switch_backend,
            'pursuer.start': self.pursuer, 'pursuer.stop': self.pursuer, 'pursuer.poll': self.pursuer,
            'study.list': lambda m: self.sim.studies(), 'study.run': self.run_study,
            'study.cancel': self.cancel_study, 'study.progress': self.study_progress,
            'replay.list': lambda m: self.server.replay.sessions(), 'replay.start': self.replay_start,
            'replay.stop': self.replay_stop, 'replay.cmd': self.replay_cmd,
        }
        for op in ('list', 'read', 'write', 'delete', 'preview', 'yaml', 'parse', 'run', 'sample',
                   'table'):
            r[f'scenario.{op}'] = getattr(self, f'scenario_{op}')
        return r

    async def guard(self, coro):
        """Runs a background action; its error goes to the gcs log and to sim.status."""
        try:
            await coro
        except Exception as e:
            self.sim._log('gcs', f'error: {e}')
            self.sim.last_result = str(e)
            await self.server.broadcast(self.sim.status())

    def background(self, coro):
        asyncio.ensure_future(self.guard(coro))

    def _no_replay(self):
        if self.server.replay.state['active']:
            raise RuntimeError('stop the replay before starting the simulation')

    # -- simulation --------------------------------------------------------------------------
    async def start(self, m):
        self._no_replay()
        self.background(self.sim.start(m.get('cfg', {})))

    async def save(self, m):
        self.server.settings.update('sim', m.get('cfg', {}))
        return self.server.settings['sim']

    async def restart(self, m):
        self._no_replay()
        self.background(self.restart_sim(dict(m.get('cfg') or {})))

    async def stop(self, m):
        self.background(self.sim.stop())

    async def build(self, m):
        self.background(self.sim.build())

    async def px4(self, m):
        return await self.sim.px4(m['cmd'])

    async def restart_sim(self, cfg):
        """Stops the simulation, waits for the processes to exit and starts with `cfg`."""
        await self.sim.stop()
        await self._wait_stopped()      # start() refuses while processes remain
        await self.sim.start(cfg)

    async def switch_backend(self, m):
        """Selects the simulator and restarts this process with it (backends.py)."""
        backend = str(m.get('backend', ''))
        ch = self.sim.choices
        if ch['fixed']:
            raise RuntimeError(f'simulador fixo nesta GCS (AERO_GCS_SIM={ch["fixed"]})')
        if backend == self.sim.backend:
            return {'restart': False}
        opt = ch['options'].get(backend)
        if not opt:
            raise RuntimeError(f'simulador desconhecido: {backend}')
        if not opt['ok']:
            raise RuntimeError(opt['reason'])
        self.sim.scan()
        if self.sim.running() or self.sim.busy:
            raise RuntimeError('stop the simulation before switching simulators')
        self.server.settings.set('backend', backend)
        log.info('simulador -> %s: reiniciando a GCS', backend)
        self.server.request_reexec()
        return {'restart': True}

    # -- remote pursuer ----------------------------------------------------------------------
    async def pursuer(self, m):
        op = m['op']
        cfg = dict(self.server.settings['sim'])
        if not self.sim.remote(cfg):
            raise RuntimeError('local pursuer: choose "remote" in the Simulation tab')
        if op == 'pursuer.start':
            if not self.sim.running():
                raise RuntimeError('start the simulation first')
            self.background(self.sim._start_pursuer(cfg))
        elif op == 'pursuer.stop':
            self.background(self.sim.pursuer.stop(cfg))
        elif not self.pursuer_polling:
            await self.poll_pursuer()
        return self.sim.pursuer.state

    async def poll_pursuer(self):
        """Remote pursuer state (SSH) and the latency guard: stops the simulation after three
        polls in a row above pursuer_max_rtt_ms (the pursuer flies on stale images otherwise)."""
        self.pursuer_polling = True
        try:
            cfg = self.server.settings['sim']
            await self.sim.pursuer.poll(cfg)
            limit = float(cfg.get('pursuer_max_rtt_ms') or 0)
            if self.sim.running() and not self.sim.busy and self.sim.pursuer.over_limit(limit):
                rtt = self.sim.pursuer.state['rtt_ms']
                self.sim._log('pursuer', f'latency {rtt} ms > {limit:g} ms in 3 readings: stopping the simulation')
                await self.sim.stop()
                self.sim.last_result = f'stopped: remote pursuer latency ({rtt} ms)'
        except Exception as e:
            log.warning('remote pursuer: %s', e)
        finally:
            self.pursuer_polling = False

    # -- studies -----------------------------------------------------------------------------
    async def run_study(self, m):
        paths = m.get('paths') or [str(m['path'])]      # paths: cascade (queue)
        self.background(self.sim.run_study([str(p) for p in paths], str(m.get('method', 'nm')),
                                           int(m.get('max_evals') or 0)))

    async def cancel_study(self, m):
        self.sim.cancel()

    async def study_progress(self, m):
        """datasets/study_progress.json (run_study.py) + study_sims.json (aero study --sims)."""
        import json
        import time
        out = {}
        datasets = self.sim.ws.parent / 'datasets'
        for key, name in (('progress', 'study_progress.json'), ('sims', 'study_sims.json')):
            f = datasets / name
            try:
                out[key] = json.loads(f.read_text())
                out[key]['age_s'] = time.time() - f.stat().st_mtime
            except (OSError, ValueError):
                out[key] = None
        return out

    # -- session replay (replay.py) ----------------------------------------------------------
    async def _replay_status(self):
        st = self.server.replay.state
        await self.server.broadcast({'op': 'replay.status', **st})
        return st

    async def replay_start(self, m):
        session = str(m.get('session', ''))
        self.server.replay.path(session)            # unknown session: error right away
        if m.get('world'):
            if self.sim.backend != 'gazebo':
                raise RuntimeError('replay in the simulator only with Gazebo; use "recording only"')
            self.background(self._replay_world_reported(session, m.get('rate', 1.0), m.get('loop', False)))
            return await self._replay_phase('preparando o simulador')
        self.sim.scan()
        if self.sim.running() or self.sim.busy:
            raise RuntimeError('stop the simulation before playing the recording only (both publish /clock and /tf)')
        await self.server.replay.start(self.server.node, session, m.get('rate', 1.0), m.get('loop', False))
        return await self._replay_status()

    async def _replay_phase(self, phase):
        self.server.replay.state['phase'] = phase
        self.sim._log('replay', f'[replay] {phase}')
        return await self._replay_status()

    async def _replay_world_reported(self, session, rate, loop):
        try:
            await self.replay_world(session, rate, loop)
        except Exception as e:
            await self._replay_phase(f'failed: {e}')
            raise

    async def replay_world(self, session, rate, loop):
        """Simulator without pursuer nodes and PX4, the session's drones spawned, then the bag
        with its poses going to world_replay (replay.py)."""
        replay = self.server.replay
        await replay.stop(self.server.node)
        rec = await asyncio.get_running_loop().run_in_executor(None, replay.environment, session)
        env = rec['env']
        self.sim.scan()
        if self.sim.running():
            await self._replay_phase('stopping the current simulation')
            await self.sim.stop()
            await self._wait_stopped()
        await self._replay_phase('starting the simulator (no pursuer, no PX4)')
        code = await self.sim.start_replay({'world': env.get('WORLD_NAME'),
                                            'scenery': env.get('AERO_SCENERY')})
        if code != 0:
            raise RuntimeError(f'the simulator did not start ({code})')
        await self._replay_phase('waiting for the nodes')
        for _ in range(180):
            node = self.server.node
            names = set(node.list_nodes()) if node else set()
            if {'/world_replay', '/scenario_runner'} <= names:
                break
            await asyncio.sleep(1.0)
        else:
            raise RuntimeError('world_replay did not appear in 3 min')
        # PX4 off: the x500 model stays in Gazebo, and nothing fights the teleports.
        await asyncio.sleep(5.0)
        await self.sim.px4('shutdown')
        if rec['scenario']:
            await self._replay_phase('inserting the scenario drones')
            res = await self.server.node.call_service(
                '/scenario_runner/command', 'aero_pursuit/srv/ScenarioCommand',
                {'command': 'load', 'yaml': rec['scenario']}, timeout=15.0)
            if not res.get('success'):
                self.sim._log('replay', f'[replay] scenario not loaded: {res.get("message")}')
            await asyncio.sleep(3.0)
        await replay.start(self.server.node, session, rate, loop, world=True)
        await self._replay_status()

    async def _wait_stopped(self):
        for _ in range(60):
            self.sim.scan()
            if not self.sim.running():
                return
            await asyncio.sleep(0.5)

    async def replay_stop(self, m):
        world = self.server.replay.state.get('world')
        await self.server.replay.stop(self.server.node)
        if world:
            self.background(self.sim.stop())      # the replay simulator has no other use
        return await self._replay_status()

    async def replay_cmd(self, m):
        await self.server.replay.control(self.server.node, str(m.get('cmd', '')), m.get('value'))
        return await self._replay_status()

    # -- scenarios ---------------------------------------------------------------------------
    @property
    def store(self):
        return self.server.scenarios

    async def scenario_list(self, m):
        return self.store.list()

    async def scenario_read(self, m):
        return self.store.read(m['file'])

    async def scenario_write(self, m):
        return self.store.write(m['file'], doc=m.get('doc'), text=m.get('yaml'))

    async def scenario_delete(self, m):
        self.store.delete(m['file'])

    async def scenario_preview(self, m):
        return await self.store.preview(doc=m.get('doc'), text=m.get('yaml'),
                                        horizon=float(m.get('horizon') or 0))

    async def scenario_sample(self, m):        # one trajectory -> samples (path editor)
        import json
        return await self.store.run_tool('sample', json.dumps(m['trajectory']), '--dt',
                                         str(float(m.get('dt') or 0.1)))

    async def scenario_table(self, m):         # CSV / TSV / Excel file -> rows (path editor)
        import base64
        from .tables import read_table
        return read_table(m.get('name', ''), base64.b64decode(m['data']))

    async def scenario_yaml(self, m):          # form document -> text (YAML tab)
        from .scenarios import dump_yaml
        return {'yaml': dump_yaml(m['doc'])}

    async def scenario_parse(self, m):         # text -> form document
        import yaml
        return {'doc': yaml.safe_load(m['yaml']) or {}}

    async def scenario_run(self, m):
        """Runs a saved scenario: in the running simulation (through the scenario_runner
        service) or starting the simulation with it."""
        info = self.store.read(m['file'])
        node = self.server.node
        if self.sim.status()['running'] and node is not None:
            res = await node.call_service(
                '/scenario_runner/command', 'aero_pursuit/srv/ScenarioCommand',
                {'command': 'start', 'path': info['path'], 'variant': m.get('variant', '')},
                timeout=8.0)
            if not res.get('success'):
                raise RuntimeError(res.get('message') or 'refused by the scenario_runner')
            return {'mode': 'live', 'message': res.get('message')}
        cfg = dict(m.get('cfg') or {}, scenario=info['path'])
        self.background(self.sim.start(cfg))
        return {'mode': 'start', 'message': 'starting the simulation with the scenario'}
