"""GCS directory at /rede: the GCSs this computer reaches (LAN /24 + Tailscale + saved addresses),
served by every GCS so a phone or tablet needs only a browser (and a home-screen shortcut).

It is the launcher of launcher/aero_launcher.py (same discovery and page) running inside the
GCS: the GCSs stay independent (a card only opens the other GCS in the browser, which asks for
its own token). The scan only runs while someone has the page open (5 min after the last
request), so an idle GCS does not sweep the network.
"""

import importlib.util
import logging
import threading
import time
from pathlib import Path

from aiohttp import web

log = logging.getLogger('gcs.directory')

BASE = '/rede'
IDLE_STOP = 300.0      # [s] without requests: stop scanning
FULL_SCAN = 60.0       # [s] between full scans (refresh of the known ones in between)


class Directory:
    def __init__(self, workspace, config_dir: Path):
        self.reg = None
        self.mod = None
        self.error = ''
        path = Path(workspace).expanduser().parent / 'launcher' / 'aero_launcher.py'
        try:
            spec = importlib.util.spec_from_file_location('aero_launcher', path)
            self.mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.mod)
            self.mod.HOSTS_FILE = Path(config_dir) / 'launcher_hosts.txt'   # with the GCS settings
            self.reg = self.mod.Registry([])
        except Exception as e:        # noqa: BLE001 - the GCS works without the directory
            self.error = f'{path}: {e}'
            log.warning('GCS directory unavailable: %s', self.error)
        self._last_access = 0.0
        self._thread = None
        self._lock = threading.Lock()

    def _touch(self):
        """A request arrived: keep (or start) the background scan."""
        self._last_access = time.monotonic()
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._loop, name='gcs-directory', daemon=True)
                self._thread.start()

    def _loop(self):
        last_full = 0.0
        while time.monotonic() - self._last_access < IDLE_STOP:
            if time.monotonic() - last_full > FULL_SCAN:
                self.reg.scan()
                last_full = time.monotonic()
            else:
                self.reg.refresh()
            time.sleep(3)

    # -- routes -----------------------------------------------------------------------
    def add_routes(self, app: web.Application):
        app.router.add_get(BASE, self.page)
        app.router.add_get(BASE + '/api/list', self.api_list)
        app.router.add_post(BASE + '/api/scan', self.api_scan)
        app.router.add_post(BASE + '/api/hosts', self.api_hosts)

    async def page(self, request):
        if self.reg is None:
            return web.Response(text=f'GCS directory unavailable: {self.error}', status=503)
        self._touch()
        html = self.mod.PAGE.replace(
            '<script>\nconst BASE',
            f"<script>window.LAUNCHER_BASE = '{BASE}';</script>\n<script>\nconst BASE", 1)
        html = html.replace(
            '<title>Aero GCS</title>',
            '<title>Aero GCS</title>\n<link rel="manifest" href="/manifest.webmanifest">'
            '\n<meta name="theme-color" content="#0f1419">'
            '\n<link rel="apple-touch-icon" href="/icon-192.png">', 1)
        html = html.replace(
            '<button id="scan">Search again</button></header>',
            '<a href="/app" style="color:var(--acc)">← this GCS</a>'
            '<button id="scan">Search again</button></header>', 1)
        return web.Response(text=html, content_type='text/html')

    async def api_list(self, request):
        if self.reg is None:
            raise web.HTTPServiceUnavailable(text=self.error)
        self._touch()
        return web.json_response(self.reg.snapshot(local_url='/app'))

    async def api_scan(self, request):
        if self.reg is None:
            raise web.HTTPServiceUnavailable(text=self.error)
        self._touch()
        threading.Thread(target=self.reg.scan, daemon=True).start()
        return web.json_response({'ok': True})

    async def api_hosts(self, request):
        if self.reg is None:
            raise web.HTTPServiceUnavailable(text=self.error)
        body = await request.json()
        hosts = self.mod.saved_hosts()
        if body.get('add'):
            hosts.append(str(body['add']).strip().removeprefix('https://').rstrip('/'))
        if body.get('remove'):
            hosts = [h for h in hosts if h != body['remove']]
        self.mod.save_hosts(hosts)
        self._touch()
        threading.Thread(target=self.reg.scan, daemon=True).start()
        return web.json_response({'ok': True})
