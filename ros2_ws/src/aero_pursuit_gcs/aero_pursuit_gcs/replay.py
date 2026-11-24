"""Session replay: a recorded session (session_recorder, datasets/<session>/) played back from its
rosbag, like a game replay. Two modes:

- world (Gazebo): the simulator runs with no pursuer nodes and no PX4 (sim_ops.replay_world) and
  the world_replay node teleports every model to its recorded pose and drives the gimbal to its
  recorded angles (/tf and /gimbal/state of the bag are remapped to /replay/*). The simulator
  renders exactly what happened: pursuer camera video, free camera, 3D; plots and status come
  from the bag. Exact by construction, unlike running the session again (the ROS nodes are not
  in lockstep with the physics, so a re-run drifts).
- bag only (any backend, no simulator): `ros2 bag play --clock` and a standalone gcs_telemetry
  feed the 3D view, plots and status. No video unless /camera was recorded.

Pause, speed and seek go through the player services (/rosbag2_player/*). Commands to the
vehicles are never replayed."""

import asyncio
import logging
import os
import re
import signal
from pathlib import Path

log = logging.getLogger('gcs.replay')

PLAYER = '/rosbag2_player'
# Never replay commands: if anything listens (a PX4 left running), it must not move.
EXCLUDE = r'^/fmu/in/.*|.*/command/.*|^/offboard/.*|^/joy.*|^/gimbal/cmd.*|^/gcs/input_log$'
# World mode: the live simulator owns /clock and /tf_static; the recorded poses go to world_replay.
WORLD_EXCLUDE = EXCLUDE + r'|^/clock$|^/tf_static$'
WORLD_REMAP = ['/tf:=/replay/tf', '/gimbal/state:=/replay/gimbal/state']
RATES = (0.25, 0.5, 1.0, 2.0, 4.0)


def _meta(path: Path) -> dict:
    """Duration and start of a bag from metadata.yaml (no yaml parser needed for two numbers)."""
    text = (path / 'metadata.yaml').read_text(errors='replace')
    dur = re.search(r'duration:\s*\n\s*nanoseconds:\s*(\d+)', text)
    start = re.search(r'starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)', text)
    count = re.search(r'^\s{2}message_count:\s*(\d+)', text, re.M)
    camera = bool(re.search(r'name:\s*/camera\s*$', text, re.M))
    return {'duration': int(dur.group(1)) / 1e9 if dur else 0.0,
            'start': int(start.group(1)) / 1e9 if start else 0.0,
            'messages': int(count.group(1)) if count else 0, 'video': camera}


def _scenario_source(path: Path) -> str:
    """Text of the scenario of a session (/scenario/source in the bag; '' if absent)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
               rosbag2_py.ConverterOptions('', ''))
        if '/scenario/source' not in {t.name for t in r.get_all_topics_and_types()}:
            return ''
        r.set_filter(rosbag2_py.StorageFilter(topics=['/scenario/source']))
        text = ''
        while r.has_next():
            text = deserialize_message(r.read_next()[1], String).data
        return text
    except Exception as e:      # older bag, other storage...
        log.warning('unreadable session scenario: %s', e)
        return ''


class Replay:
    def __init__(self, datasets: Path, world_frame: str, log_fn):
        self.root = datasets
        self.world_frame = world_frame
        self._log = log_fn                  # SimControl._log(name, line)
        self._procs = []
        self._clock = None                  # GenericSub of the topic that gives the position
        self._clock_topic = ''
        self._node = None
        self.state = {'active': False, 'session': '', 'duration': 0.0, 'start': 0.0, 'world': False,
                      'phase': '', 'position': 0.0, 'paused': False, 'rate': 1.0, 'loop': False,
                      'video': False}

    def sessions(self, limit=300) -> list:
        """Recorded sessions, newest first: [{'path', 'duration', 'messages'}]."""
        out = []
        for meta in self.root.glob('**/metadata.yaml'):
            d = meta.parent
            if len(d.relative_to(self.root).parts) > 3:
                continue
            try:
                m = _meta(d)
            except OSError:
                continue
            if m['messages'] == 0:
                continue
            out.append({'path': str(d.relative_to(self.root)), 'duration': round(m['duration'], 1),
                        'messages': m['messages'], 'video': m['video'], 'mtime': meta.stat().st_mtime})
        out.sort(key=lambda s: s['mtime'], reverse=True)
        return out[:limit]

    async def _spawn(self, name, argv):
        self._log('replay', f'$ {" ".join(argv)}')
        proc = await asyncio.create_subprocess_exec(
            *argv, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT,
            start_new_session=True)

        async def pump():
            async for raw in proc.stdout:
                self._log('replay', f'[{name}] {raw.decode(errors="replace").rstrip()}')
            await proc.wait()
            self._log('replay', f'[{name}] terminou ({proc.returncode})')
        asyncio.ensure_future(pump())
        self._procs.append(proc)
        return proc

    def path(self, session: str) -> Path:
        path = (self.root / session).resolve()
        if self.root.resolve() not in path.parents or not (path / 'metadata.yaml').is_file():
            raise RuntimeError(f'unknown session: {session}')
        return path

    def environment(self, session: str) -> dict:
        """Simulation environment of the recording (replay.yaml: world, camera, scenery) and the
        scenario text kept in the bag (its drones must exist in the world to be teleported)."""
        import yaml
        path = self.path(session)
        env = {}
        if (path / 'replay.yaml').is_file():
            env = (yaml.safe_load((path / 'replay.yaml').read_text()) or {}).get('env') or {}
        return {'env': env, 'scenario': _scenario_source(path)}

    async def start(self, node, session: str, rate=1.0, loop=False, world=False):
        path = self.path(session)
        await self.stop(node)
        self._log('replay', f'replay of {session} ({"in the simulator" if world else "recording only"})')
        m = _meta(path)
        rate = float(rate) if float(rate) in RATES else 1.0
        argv = ['ros2', 'bag', 'play', str(path), '--rate', str(rate), '--disable-keyboard-controls',
                '--delay', '1']
        if world:
            argv += ['--exclude-regex', WORLD_EXCLUDE, '--remap', *WORLD_REMAP]
            self._clock_topic = '/replay/tf'
        else:
            argv += ['--clock', '50', '--exclude-regex', EXCLUDE]
            self._clock_topic = '/clock'
            await self._spawn('telemetry', ['ros2', 'run', 'aero_pursuit', 'gcs_telemetry', '--ros-args',
                                            '-p', f'world_frame:={self.world_frame}', '-p', 'use_sim_time:=true'])
        if loop:
            argv.append('--loop')
        await self._spawn('bag', argv)
        self.state.update(active=True, session=session, duration=m['duration'], start=m['start'],
                          world=bool(world), phase='reproduzindo', position=0.0, paused=False,
                          rate=rate, loop=bool(loop), video=bool(world or m['video']))
        self._node = node
        if node is not None:
            try:
                ty = 'tf2_msgs/msg/TFMessage' if world else 'rosgraph_msgs/msg/Clock'
                self._clock = node.subscribe(self._clock_topic, ty)
            except Exception as e:
                log.warning('replay without %s: %s', self._clock_topic, e)

    async def stop(self, node=None):
        for proc in self._procs:
            if proc.returncode is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for proc in self._procs:
            try:
                await asyncio.wait_for(proc.wait(), 5)
            except asyncio.TimeoutError:
                os.killpg(proc.pid, signal.SIGKILL)
        self._procs = []
        if self._clock is not None and node is not None:
            node.unsubscribe(self._clock_topic)
        self._clock = None
        self.state.update(active=False, position=0.0, paused=False, phase='')

    def poll(self):
        """Position from /clock; ends the replay when the player exits (no --loop)."""
        if not self.state['active']:
            return self.state
        bag = self._procs[-1] if self._procs else None      # spawned last
        if bag is None or bag.returncode is not None:
            self.state.update(active=False)
            asyncio.ensure_future(self.stop(self._node))
            return self.state
        msg = self._clock.msg if self._clock is not None else None
        stamp = None
        if msg is not None and self.state['world']:
            stamp = msg.transforms[0].header.stamp if msg.transforms else None
        elif msg is not None:
            stamp = msg.clock
        if stamp is not None:
            t = stamp.sec + stamp.nanosec * 1e-9
            self.state['position'] = round(max(0.0, min(self.state['duration'], t - self.state['start'])), 2)
        return self.state

    async def control(self, node, cmd: str, value=None):
        if not self.state['active']:
            raise RuntimeError('no replay in progress')
        if node is None:
            raise RuntimeError('ROS 2 unavailable in the GCS')
        if cmd in ('pause', 'resume'):
            await node.call_service(f'{PLAYER}/{cmd}', f'rosbag2_interfaces/srv/{cmd.capitalize()}', {})
            self.state['paused'] = cmd == 'pause'
        elif cmd == 'rate':
            rate = float(value)
            if rate not in RATES:
                raise RuntimeError(f'invalid rate: {value}')
            await node.call_service(f'{PLAYER}/set_rate', 'rosbag2_interfaces/srv/SetRate', {'rate': rate})
            self.state['rate'] = rate
        elif cmd == 'seek':
            t = self.state['start'] + max(0.0, min(self.state['duration'], float(value)))
            sec = int(t)
            await node.call_service(f'{PLAYER}/seek', 'rosbag2_interfaces/srv/Seek',
                                    {'time': {'sec': sec, 'nanosec': int((t - sec) * 1e9)}})
            self.state['position'] = round(float(value), 2)
        else:
            raise RuntimeError(f'unknown replay command: {cmd}')
        return self.state
