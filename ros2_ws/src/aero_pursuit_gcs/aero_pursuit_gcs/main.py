"""GCS server: web interface, control and video WebSockets, ROS bridge and
simulation control.

    ./gcs.sh                       (or: ros2 run aero_pursuit_gcs gcs_server)
    http://localhost:8080          on the machine itself (no token)
    https://<lan-ip>:8443          on the local network (self-signed certificate)
    Cloudflare tunnel              from anywhere (Network tab)
"""

import argparse
import asyncio
import collections
import math
import hmac
import inspect
import json
import logging
import os
import secrets
import signal
import socket
import subprocess
import ssl
import sys
import time
from pathlib import Path

from aiohttp import WSMsgType, web

from . import backends
from .param_memory import ParamMemory
from .replay import Replay
from .scenarios import ScenarioStore
from .settings import CONFIG_DIR, Settings
from .sim_control import SimControl
from .sim_isaac import IsaacSimControl
from .sim_ops import SimOps
from .sysinfo import SysInfo
from .tunnel import Tunnel
from .video_hub import VideoHub

log = logging.getLogger('gcs')
COOKIE = 'gcs_token'
# Operator actions that change the simulation state: journaled on /gcs/input_log (kept in the session bag).
INPUT_OPS = ('spectator', 'select', 'select.stop', 'omni', 'param.set', 'pub', 'srv', 'scenario.cmd')


def asset_version() -> str:
    """Short hash of the interface files (names + mtimes). The page loads its modules under
    /a/<version>/, so any change gives new URLs and the browser can never run a stale cached copy
    (a no-cache header does not evict modules cached before it existed)."""
    import hashlib
    h = hashlib.sha1()
    root = web_dir()
    for sub in ('js', 'css'):
        for f in sorted((root / sub).rglob('*')):
            if f.is_file():
                h.update(f'{f.relative_to(root)}:{f.stat().st_mtime_ns}'.encode())
    return h.hexdigest()[:10]


def web_dir() -> Path:
    src = Path(__file__).resolve().parent.parent / 'web'
    if src.is_dir():
        return src          # running from the source code (gcs.sh): edits apply immediately
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('aero_pursuit_gcs')) / 'web'


def render_page(name):
    """An HTML page of web/ with its assets under /a/<version>/ (see asset_version)."""
    v = asset_version()
    html = (web_dir() / name).read_text()
    html = html.replace('href="/css/', f'href="/a/{v}/css/').replace('src="/js/', f'src="/a/{v}/js/')
    return web.Response(text=html, content_type='text/html', headers={'Cache-Control': 'no-cache'})


def lan_addresses():
    try:
        out = subprocess.run(['ip', '-o', '-4', 'addr', 'show'], capture_output=True,
                             text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    addrs = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) > 3 and parts[2] == 'inet' and not parts[1].startswith(
                ('docker', 'br-', 'veth', 'virbr')):
            ip = parts[3].split('/')[0]
            if not ip.startswith('127.'):
                addrs.append(ip)
    return addrs


def ensure_certificate(addrs):
    cert, key = CONFIG_DIR / 'cert.pem', CONFIG_DIR / 'key.pem'
    if cert.exists() and key.exists():
        return cert, key
    san = ','.join(['DNS:localhost', f'DNS:{socket.gethostname()}', 'IP:127.0.0.1'] +
                   [f'IP:{a}' for a in addrs])
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '3650',
                    '-subj', '/CN=aero-gcs', '-addext', f'subjectAltName={san}',
                    '-keyout', str(key), '-out', str(cert)],
                   check=True, capture_output=True)
    key.chmod(0o600)
    return cert, key


def is_local(request) -> bool:
    """Browser on the machine itself. cloudflared also connects from 127.0.0.1, but
    adds Cf-Connecting-Ip: those accesses are remote."""
    peer = request.remote or ''
    proxied = 'Cf-Connecting-Ip' in request.headers or 'X-Forwarded-For' in request.headers
    return peer in ('127.0.0.1', '::1') and not proxied


def dumps(obj) -> str:
    return json.dumps(obj, separators=(',', ':'), allow_nan=False, default=str)


class CtlClient:
    def __init__(self, ws, request):
        self.ws = ws
        self.transport = request.transport
        self.peer = request.headers.get('Cf-Connecting-Ip', request.remote)
        self.subs = {}          # id -> dict(topic, period, last_t, last_count)
        self.joy_t = None

    def buffered(self):
        try:
            return self.transport.get_write_buffer_size() if self.transport else 0
        except Exception:
            return 0


class GcsServer:
    def __init__(self, args):
        self.args = args
        self.settings = Settings()
        if args.workspace:
            self.settings.set('workspace', str(Path(args.workspace).resolve()))
        self.ctl_clients = set()
        self.hubs = {'main': VideoHub(self.settings, self.broadcast),
                     'spectator': VideoHub(self.settings, self.broadcast, 'spectator',
                                           'stream_spectator')}
        self.hub = self.hubs['main']
        backend = backends.selected(self.settings)
        backends.apply_env(backend, self.settings['workspace'])         # DDS domain/profile before rclpy starts
        sim_cls = IsaacSimControl if backend == 'isaac' else SimControl
        self.scenarios = ScenarioStore(self.settings['workspace'], sim_cls.backend)
        self.sim = sim_cls(self.settings, self.broadcast, self.scenarios)
        self.sim.choices = {'fixed': backends.fixed(),
                            'options': backends.availability(self.settings['workspace'])}
        self._reexec = False
        self.replay = Replay(Path(self.settings['workspace']).expanduser().parent / 'datasets',
                             self.settings['world_frame'], self.sim._log)
        self.sim.memory_logs['replay'] = collections.deque(maxlen=2000)
        self.spectator = None       # last free-camera command (re-sent to whoever joins)
        self.tunnel = Tunnel(self.settings, self.broadcast)
        self.sys = SysInfo()
        self.ros = None
        self.node = None
        self.addrs = lan_addresses()
        self._last_counts = {}
        self._last_path_t = 0.0
        self._joy_active = False
        self._tick = 0
        # Request routing: operations by domain (op -> async handler(message) -> reply data).
        self.params = ParamMemory(self)
        self.sim_ops = SimOps(self)
        self.routes = {**self.params.routes(), **self.sim_ops.routes(), **self._admin_routes()}

    # -- envio -------------------------------------------------------------------
    async def broadcast(self, msg, droppable=False):
        if not self.ctl_clients:
            return
        data = dumps(msg)
        for c in list(self.ctl_clients):
            if c.ws.closed:
                continue
            if droppable and c.buffered() > 256 * 1024:
                continue    # slow client: old telemetry is worthless
            asyncio.ensure_future(self._send(c.ws, data))

    @staticmethod
    async def _send(ws, data):
        try:
            await ws.send_str(data)
        except (ConnectionResetError, RuntimeError):
            pass

    # -- HTTP --------------------------------------------------------------------
    @staticmethod
    @web.middleware
    async def no_cache(request, handler):
        """The interface (HTML/JS/CSS) is revalidated on every load: after a build the browser
        gets the new modules instead of a stale cached copy (ETag/Last-Modified -> 304 if unchanged)."""
        resp = await handler(request)
        if not request.path.startswith('/ws/') and isinstance(resp, web.StreamResponse):
            resp.headers.setdefault('Cache-Control', 'no-cache')
        return resp

    @web.middleware
    async def auth(self, request, handler):
        path = request.path
        # Public: login, the launcher probe and the home-screen app files (Chrome fetches the
        # manifest without cookies).
        if path in ('/login', '/favicon.ico', '/api/info', '/manifest.webmanifest', '/icon-192.png',
                    '/icon-512.png') or path.startswith('/css/login'):
            return await handler(request)
        token = self.settings['token']
        if self.settings['local_no_auth'] and is_local(request):
            return await handler(request)
        q = request.query.get('token')
        if q and hmac.compare_digest(q, token):
            resp = web.HTTPFound(path)
            resp.set_cookie(COOKIE, token, max_age=30 * 86400, httponly=True, samesite='Lax')
            raise resp
        c = request.cookies.get(COOKIE, '')
        if c and hmac.compare_digest(c, token):
            return await handler(request)
        if path.startswith('/ws/'):
            raise web.HTTPUnauthorized()
        raise web.HTTPFound('/login')

    async def login_page(self, request):
        if request.method == 'POST':
            form = await request.post()
            t = str(form.get('token', ''))
            if hmac.compare_digest(t, self.settings['token']):
                resp = web.HTTPFound('/')
                resp.set_cookie(COOKIE, t, max_age=30 * 86400, httponly=True, samesite='Lax')
                raise resp
            await asyncio.sleep(1.0)    # freia tentativas
            return web.Response(text=LOGIN_HTML.replace('{{err}}', 'Invalid token'),
                                content_type='text/html')
        return web.Response(text=LOGIN_HTML.replace('{{err}}', ''), content_type='text/html')

    async def api_info(self, request):
        """Public (no token): what the launcher (launcher/aero_launcher.py) shows for this GCS."""
        return web.json_response(self.public_info(), headers={'Access-Control-Allow-Origin': '*'})

    def public_info(self) -> dict:
        s, status, node = self.settings, self.sim.status(), self.node
        scenario = None
        if node is not None:
            _, msg = node.take('scenario')
            if msg is not None and getattr(msg, 'state_name', '') not in ('', 'IDLE'):
                scenario = f'{msg.scenario} ({msg.state_name})'
        return {
            'kind': 'aero_gcs',
            'name': os.environ.get('AERO_GCS_NAME') or socket.gethostname(),
            'backend': self.sim.backend,
            'simulator': {'gazebo': 'Gazebo', 'isaac': 'Isaac Sim'}.get(self.sim.backend, self.sim.backend),
            'available': {b: o['ok'] for b, o in self.sim.choices['options'].items()},
            'addrs': self.addrs,
            'http_port': s['http_port'],
            'https_port': s['https_port'],
            'running': status['running'],
            'busy': status['busy'],
            'rtf': round(node.rtf, 2) if node is not None and node.rtf else None,
            'scenario': scenario,
            'tunnel': self.tunnel.url or None,   # Cloudflare (Network tab), when running
            'time': time.time(),
        }

    async def home(self, request):
        raise web.HTTPFound('/app' + (f'?{request.query_string}' if request.query_string else ''))

    async def index(self, request):
        return render_page('index.html')

    async def versioned_asset(self, request):
        """/a/<version>/<path>: a file of the interface; the URL changes with its content."""
        root = web_dir().resolve()
        f = (root / request.match_info['tail']).resolve()
        if root not in f.parents or not f.is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(f, headers={'Cache-Control': 'public, max-age=31536000, immutable'})

    # -- Video WebSocket --------------------------------------------------------
    async def ws_video(self, request):
        ws = web.WebSocketResponse(max_msg_size=1 << 20, compress=False)
        await ws.prepare(request)
        _set_nodelay(request.transport)
        hub = self.hubs.get(request.query.get('ch', 'main'), self.hub)
        client = hub.add_client(ws, request.transport, request.remote)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(msg.data)
                except ValueError:
                    continue
                op = m.get('op')
                if op == 'ack':
                    client.on_ack(int(m['seq']))
                elif op == 'enable':
                    hub.set_enabled(client, bool(m.get('on')))
                elif op == 'key':
                    client.need_key = True
                    hub.request_key(force=True)
        finally:
            hub.remove_client(client)
        return ws

    # -- Control WebSocket -----------------------------------------------------
    async def ws_ctl(self, request):
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4 << 20, compress=False)
        await ws.prepare(request)
        _set_nodelay(request.transport)
        client = CtlClient(ws, request)
        self.ctl_clients.add(client)
        log.info('client connected: %s', client.peer)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(msg.data)
                except ValueError:
                    continue
                asyncio.ensure_future(self._dispatch(client, m, request))
        finally:
            self.ctl_clients.discard(client)
            self.sim.unsubscribe_log(ws)
            for sub in client.subs.values():
                self.node and self.node.unsubscribe(sub['topic'])
            if client.joy_t is not None:
                self._neutral_joy()
            log.info('client disconnected: %s', client.peer)
        return ws

    async def _dispatch(self, c, m, request):
        op, rid = m.get('op'), m.get('id')
        try:
            data = await self._handle(c, op, m, request)
            if rid is not None:
                await c.ws.send_str(dumps({'op': 'result', 'id': rid, 'ok': True, 'data': data}))
        except Exception as e:  # error returned to the interface
            log.debug('op %s failed', op, exc_info=True)
            if rid is not None and not c.ws.closed:
                await c.ws.send_str(dumps({'op': 'result', 'id': rid, 'ok': False,
                                           'error': f'{type(e).__name__}: {e}'}))

    async def _handle(self, c, op, m, request):
        node = self.node
        if op == 'ping':
            await c.ws.send_str(dumps({'op': 'pong', 't': m.get('t'), 'ts': time.time() * 1000}))
            return None
        if op == 'joy':
            node.publish_joy(m.get('axes', []), m.get('buttons', []))
            c.joy_t = time.monotonic()
            self._joy_active = True
            if m.get('seq') is not None:
                await c.ws.send_str(dumps({'op': 'joy.ack', 'seq': m['seq'], 't': m.get('t')}))
            return None
        if op == 'hello':
            return self.hello(request)
        if op == 'stream.set':
            hub = self.hubs.get(m.get('channel', 'main'), self.hub)
            hub.set_config(m.get('cfg', {}))
            await self.broadcast(hub.state())
            return hub.cfg
        if op in INPUT_OPS:
            return await self.exec_input(op, m, journal=True)
        if op == 'sub':
            holder = node.subscribe(m['topic'], m.get('type'))
            rate = max(0.2, min(60.0, float(m.get('rate', 10))))
            c.subs[m['sid']] = {'topic': m['topic'], 'period': 1 / rate, 'last_t': 0,
                                'last_count': -1, 'holder': holder}
            return {'type': holder.type_name}
        if op == 'unsub':
            sub = c.subs.pop(m['sid'], None)
            if sub:
                node.unsubscribe(sub['topic'])
            return None
        if op == 'log.sub':
            await self.sim.subscribe_log(c.ws, m['name'])
            return None
        if op == 'log.unsub':
            self.sim.unsubscribe_log(c.ws)
            return None
        handler = self.routes.get(op)
        if handler is None:
            raise ValueError(f'unknown operation: {op}')
        result = handler(m)            # handlers may be plain functions (simple reads)
        return await result if inspect.isawaitable(result) else result

    def _admin_routes(self) -> dict:
        """ROS graph, tunnel, interface settings and token."""
        async def graph(m):
            return {'nodes': self.node.list_nodes, 'topics': self.node.list_topics,
                    'services': self.node.list_services}[m['op']]()

        async def tunnel(m):
            op = m['op']
            if op == 'tunnel.start':
                asyncio.ensure_future(self.tunnel.start(self.settings['http_port']))
            elif op == 'tunnel.stop':
                await self.tunnel.stop()
            elif op == 'tunnel.install':
                asyncio.ensure_future(self.tunnel.install())
            else:
                self.settings.update('tunnel', {k: m[k] for k in ('named_token', 'hostname', 'autostart') if k in m})
                await self.broadcast(self.tunnel.status())

        async def settings_set(m):
            for k in ('telemetry_hz', 'world_frame', 'local_no_auth'):
                if k in m:
                    self.settings.set(k, m[k])
            if 'world_frame' in m:
                self.node.world_frame = m['world_frame']
                await self.node.set_param('/gcs_telemetry', 'world_frame', 'string', m['world_frame'])

        async def token_regen(m):
            self.settings.set('token', secrets.token_urlsafe(18))
            return {'token': self.settings['token']}

        return {'nodes': graph, 'topics': graph, 'services': graph,
                'tunnel.start': tunnel, 'tunnel.stop': tunnel, 'tunnel.install': tunnel,
                'tunnel.config': tunnel, 'settings.set': settings_set, 'token.regen': token_regen}

    async def exec_input(self, op, m, journal=False):
        """Actions that change the simulation state (INPUT_OPS). journal=True also publishes them
        on /gcs/input_log, which the session recorder keeps in the bag (what the operator did)."""
        node = self.node
        if node is None:
            raise RuntimeError('ROS 2 unavailable in the GCS')
        args = {k: v for k, v in m.items() if k not in ('op', 'id')}
        result = None
        if journal and op != 'joy':
            # Logged at request time (simulation time of the operator's action).
            try:
                node.log_input({'op': op, **args})
            except Exception:
                log.debug('input journal unavailable', exc_info=True)
        if op == 'joy':
            node.publish_joy(args.get('axes', []), args.get('buttons', []))
            return None                 # /joy itself is recorded
        if op == 'spectator':
            self.spectator = args.get('cmd') or {}
            node.publish_spectator(self.spectator)
        elif op == 'select':
            node.publish_selection(args['x'], args['y'], args['w'], args['h'])
        elif op == 'select.stop':
            node.publish_selection(0, 0, 0, 0)
        elif op == 'omni':
            node.publish_tf_cmd(args.get('frame', ''), args.get('enable', True))
        elif op == 'param.set':
            ok, reason = await node.set_param(args['node'], args['name'], args['type'], args['value'])
            if not ok:
                raise RuntimeError(reason or 'refused by the node')
            if self.settings['remember_params'] and args.get('remember', True) and args['type'] != 'unset':
                await self.params.remember_change(args['node'], args['name'], args['type'], args['value'])
            result = {'ok': ok}
        elif op == 'pub':
            node.publish(args['topic'], args['type'], args.get('data') or {})
        elif op == 'srv':
            result = await node.call_service(args['service'], args.get('type'), args.get('request') or {})
        elif op == 'scenario.cmd':
            result = await node.call_service('/scenario_runner/command', 'aero_pursuit/srv/ScenarioCommand',
                                             {'command': args['command']}, timeout=8.0)
        else:
            raise ValueError(f'entrada desconhecida: {op}')
        return result

    def request_reexec(self):
        """Stops the server and restarts this process (simulator switch, backends.py)."""
        self._reexec = True
        asyncio.get_running_loop().call_later(0.5, self._stop_event.set)

    def hello(self, request):
        s = self.settings
        return {
            'world_frame': s['world_frame'],
            'telemetry_hz': s['telemetry_hz'],
            'local': is_local(request),
            'token': s['token'],
            'urls': {
                'local': f"http://localhost:{s['http_port']}",
                'lan': [f"https://{a}:{s['https_port']}" for a in self.addrs],
            },
            'local_no_auth': s['local_no_auth'],
            'stream': self.hub.state(),
            'streams': {ch: h.state() for ch, h in self.hubs.items()},
            'spectator': self.spectator,
            'scenario_dir': str(self.scenarios.dir),
            'sim': self.sim.status(),
            'tunnel': self.tunnel.status(),
            'ros': self.node is not None,
        }

    # -- loops -------------------------------------------------------------------
    async def telemetry_loop(self):
        from .ros_bridge import to_jsonable
        next_t = time.monotonic()
        while True:
            hz = max(1.0, min(60.0, float(self.settings['telemetry_hz'])))
            next_t += 1 / hz
            await asyncio.sleep(max(0.0, next_t - time.monotonic()))
            if time.monotonic() - next_t > 0.5:
                next_t = time.monotonic()       # fell far behind (suspend etc.)
            if not self.ctl_clients or self.node is None:
                continue
            n = self.node
            tel = {'op': 'tel', 't': time.time() * 1000, 'tf': n.transforms()}
            if n.clock:
                tel['sim'] = n.clock[0]
            tel['rtf'] = n.rtf
            for name in ('status', 'metrics', 'detection', 'teleop', 'px4',
                         'scenario', 'truth'):
                count, msg = n.take(name)
                if msg is not None and count != self._last_counts.get(name):
                    self._last_counts[name] = count
                    d = to_jsonable(msg)
                    d.pop('header', None)
                    tel[name] = d
            count, est = n.take('estimate')
            if est is not None and count != self._last_counts.get('estimate'):
                self._last_counts['estimate'] = count
                p, v = est.pose.pose.position, est.twist.twist.linear
                tel['estimate'] = {'frame': est.header.frame_id, 'p': [p.x, p.y, p.z],
                                   'v': [v.x, v.y, v.z]}
            now = time.monotonic()
            count, path = n.take('path')
            if (path is not None and count != self._last_counts.get('path') and
                    now - self._last_path_t > 0.2):
                self._last_counts['path'] = count
                self._last_path_t = now
                tel['path'] = {'frame': path.header.frame_id,
                               'points': [[round(ps.pose.position.x, 3),
                                           round(ps.pose.position.y, 3),
                                           round(ps.pose.position.z, 3)] for ps in path.poses]}
            await self.broadcast(to_jsonable(tel), droppable=True)
            await self._push_generic(now)

    async def _push_generic(self, now):
        from .ros_bridge import to_jsonable
        for c in list(self.ctl_clients):
            for sid, sub in list(c.subs.items()):
                h = sub['holder']
                if h.msg is None or h.count == sub['last_count'] or now - sub['last_t'] < sub['period']:
                    continue
                sub['last_count'], sub['last_t'] = h.count, now
                if c.buffered() > 256 * 1024:
                    continue
                asyncio.ensure_future(self._send(c.ws, dumps(
                    {'op': 'msg', 'sid': sid, 'topic': sub['topic'], 'count': h.count,
                     'data': to_jsonable(h.msg)})))

    async def slow_loop(self):
        while True:
            self.sim.scan()
            if self.node:
                self.node.discover_px4()
                if self._tick % 2 == 0:
                    asyncio.ensure_future(self.params.reapply())
            if self._tick % 5 == 0 and self.sim.remote(self.settings['sim']) and not self.sim_ops.pursuer_polling:
                asyncio.ensure_future(self.sim_ops.poll_pursuer())
            self._tick += 1
            if self.replay.state['active']:
                await self.broadcast({'op': 'replay.status', **self.replay.poll()})
            await self.broadcast(self.sim.status())
            await self.broadcast(self.sys.sample())
            await asyncio.sleep(1.0)

    async def fast_loop(self):
        while True:
            await asyncio.sleep(0.1)
            await self.sim.poll_logs()
            if self.node:
                for ev in self.node.pop_param_events():
                    await self.broadcast({'op': 'param.event', **ev})
            # Joystick watchdog: no packets for 0.5 s => neutral command (safety
            # against a connection drop with the target moving).
            if self._joy_active:
                now = time.monotonic()
                senders = [c for c in self.ctl_clients if c.joy_t and now - c.joy_t < 0.5]
                if not senders:
                    self._neutral_joy()

    def _neutral_joy(self):
        if self.node:
            self.node.publish_joy([0.0] * 6, [0] * 15)
        self._joy_active = False
        for c in self.ctl_clients:
            c.joy_t = None

    # -- life cycle ---------------------------------------------------------------
    async def start(self):
        try:
            from .ros_bridge import RosBridge
            self.ros = RosBridge(self.settings['world_frame'])
            self.node = self.ros.node
        except Exception:
            log.exception('ROS 2 unavailable: run with the workspace sourced (gcs.sh)')
        for hub in self.hubs.values():
            await hub.start()

        app = web.Application(middlewares=[self.no_cache, self.auth])
        app.router.add_route('*', '/login', self.login_page)
        app.router.add_get('/', self.home)
        app.router.add_get('/app', self.index)
        app.router.add_get('/api/info', self.api_info)
        app.router.add_get('/a/{ver}/{tail:.+}', self.versioned_asset)
        app.router.add_get('/ws/ctl', self.ws_ctl)
        app.router.add_get('/ws/video', self.ws_video)
        # /rede: the GCSs this computer reaches (launcher/aero_launcher.py inside the GCS), so a
        # phone needs only the browser.
        from .directory import Directory
        Directory(self.settings['workspace'], CONFIG_DIR).add_routes(app)
        app.router.add_static('/', web_dir(), show_index=False)
        self.runner = web.AppRunner(app, access_log=None)
        await self.runner.setup()
        bind, http_port = self.settings['bind'], self.args.port or self.settings['http_port']
        self.settings.set('http_port', http_port)
        await web.TCPSite(self.runner, bind, http_port).start()
        log.info('GCS at http://localhost:%d', http_port)
        if not self.args.no_https:
            try:
                cert, key = ensure_certificate(self.addrs)
                ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
                ctx.load_cert_chain(str(cert), str(key))
                https_port = self.args.https_port or self.settings['https_port']
                self.settings.set('https_port', https_port)
                await web.TCPSite(self.runner, bind, https_port, ssl_context=ctx).start()
                for a in self.addrs:
                    log.info('Local network: https://%s:%d  (token: %s)', a, https_port,
                             self.settings['token'])
            except Exception:
                log.exception('HTTPS disabled (openssl unavailable?)')
        self.tasks = [asyncio.ensure_future(c) for c in
                      (self.telemetry_loop(), self.slow_loop(), self.fast_loop())]
        if self.settings['tunnel'].get('autostart'):
            asyncio.ensure_future(self.tunnel.start(http_port))

    async def stop(self):
        for t in getattr(self, 'tasks', []):
            t.cancel()
        for c in list(self.ctl_clients):
            await c.ws.close()
        await self.tunnel.stop()
        await self.replay.stop(self.node)
        for hub in self.hubs.values():
            await hub.stop()
        await self.runner.cleanup()
        if self.ros:
            self.ros.shutdown()


def _set_nodelay(transport):
    sock = transport.get_extra_info('socket') if transport else None
    if sock is not None:
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Kernel queue with only what is needed: the GCS congestion control
            # sees the real delay instead of MBs hidden in the socket.
            sock.setsockopt(socket.IPPROTO_TCP, 25, 128 * 1024)  # TCP_NOTSENT_LOWAT
        except OSError:
            pass


class MemoryLogHandler(logging.Handler):
    def __init__(self, sim):
        super().__init__()
        self.sim = sim

    def emit(self, record):
        try:
            self.sim.memory_logs['gcs'].append(self.format(record))
        except Exception:
            pass


LOGIN_HTML = """<!doctype html><html lang="pt-br"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Aero GCS</title>
<link rel="stylesheet" href="/css/login.css"></head><body><form method="post" action="/login">
<h1>Aero GCS</h1><p>Access token (aero gcs token, or the Network tab).</p>
<input name="token" type="password" autofocus autocomplete="current-password">
<button>Sign in</button><p class="err">{{err}}</p></form></body></html>"""


def main():
    ap = argparse.ArgumentParser(description='aero_pursuit web ground station')
    ap.add_argument('--port', type=int, help='HTTP port (default 8080)')
    ap.add_argument('--https-port', type=int, help='HTTPS port (default 8443)')
    ap.add_argument('--no-https', action='store_true')
    ap.add_argument('--workspace', help='ROS workspace (default ~/aero_pursuit/ros2_ws)')
    ap.add_argument('--log-level', default='info')
    args = ap.parse_args()
    logging.basicConfig(level=args.log_level.upper(),
                        format='%(asctime)s %(name)s %(levelname)s %(message)s',
                        datefmt='%H:%M:%S')
    logging.getLogger('aiohttp').setLevel(logging.WARNING)

    server = GcsServer(args)
    server._stop_event = None
    handler = MemoryLogHandler(server.sim)
    handler.setFormatter(logging.Formatter('%(asctime)s %(name)s %(levelname)s %(message)s',
                                           '%H:%M:%S'))
    logging.getLogger().addHandler(handler)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    stop = asyncio.Event()
    server._stop_event = stop
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    async def run():
        await server.start()
        await stop.wait()
        log.info('encerrando...')
        await server.stop()

    try:
        loop.run_until_complete(run())
    finally:
        loop.close()
        if server._reexec:
            # Simulator switch (sim.backend): same command line, new backend from settings.
            os.execv(sys.executable, sys.orig_argv)
        os._exit(0)     # rclpy/executor threads do not hold the process


if __name__ == '__main__':
    main()
