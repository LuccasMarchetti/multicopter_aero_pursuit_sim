"""Remote access without a VPN or fixed IP: Cloudflare Tunnel.

- Quick (default): `cloudflared tunnel --url`, free and without an account; generates a
  https://<random>.trycloudflare.com URL at every start.
- Named (optional): token of a tunnel created in the Cloudflare dashboard (free account
  + own domain) => fixed URL.
Traffic reaches the server as local (127.0.0.1); the GCS token authentication
treats those accesses as remote through the Cf-Connecting-Ip header.
"""

import asyncio
import logging
import os
import re
import shutil
import stat
from pathlib import Path

import aiohttp

log = logging.getLogger('gcs.tunnel')

URL_RE = re.compile(r'https://[-a-z0-9]+\.trycloudflare\.com')
LOCAL_BIN = Path.home() / '.local' / 'bin' / 'cloudflared'
DOWNLOAD = {
    'x86_64': 'https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64',
    'aarch64': 'https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64',
}


class Tunnel:
    def __init__(self, settings, broadcast):
        self.settings = settings
        self.broadcast = broadcast
        self.proc = None
        self.state = 'stopped'      # stopped | installing | starting | running | error
        self.url = ''
        self.message = ''
        self._task = None

    def binary(self):
        found = shutil.which('cloudflared')
        if found:
            return found
        return str(LOCAL_BIN) if LOCAL_BIN.exists() else None

    def status(self):
        cfg = self.settings['tunnel']
        return {'op': 'tunnel.status', 'state': self.state, 'url': self.url,
                'message': self.message, 'installed': self.binary() is not None,
                'named': bool(cfg.get('named_token')), 'hostname': cfg.get('hostname', ''),
                'autostart': cfg.get('autostart', False)}

    async def _set(self, state, message='', url=None):
        self.state, self.message = state, message
        if url is not None:
            self.url = url
        await self.broadcast(self.status())

    async def install(self):
        arch = os.uname().machine
        src = DOWNLOAD.get(arch)
        if not src:
            await self._set('error', f'architecture {arch} has no known binary')
            return
        await self._set('installing', 'baixando cloudflared...')
        LOCAL_BIN.parent.mkdir(parents=True, exist_ok=True)
        tmp = LOCAL_BIN.with_suffix('.part')
        try:
            timeout = aiohttp.ClientTimeout(total=300)
            async with aiohttp.ClientSession(timeout=timeout) as s, s.get(src) as r:
                r.raise_for_status()
                with open(tmp, 'wb') as f:
                    async for chunk in r.content.iter_chunked(1 << 16):
                        f.write(chunk)
            tmp.chmod(tmp.stat().st_mode | stat.S_IXUSR)
            tmp.replace(LOCAL_BIN)
            await self._set('stopped', 'cloudflared instalado em ~/.local/bin')
        except Exception as e:
            tmp.unlink(missing_ok=True)
            await self._set('error', f'falha no download: {e}')

    async def start(self, http_port):
        if self.proc and self.proc.returncode is None:
            return
        binary = self.binary()
        if not binary:
            await self.install()
            binary = self.binary()
            if not binary:
                return
        cfg = self.settings['tunnel']
        if cfg.get('named_token'):
            argv = [binary, 'tunnel', '--no-autoupdate', 'run', '--token', cfg['named_token']]
            url = f"https://{cfg['hostname']}" if cfg.get('hostname') else ''
        else:
            argv = [binary, 'tunnel', '--no-autoupdate', '--url', f'http://127.0.0.1:{http_port}']
            url = ''
        await self._set('starting', 'connecting to Cloudflare...', url='')
        self.proc = await asyncio.create_subprocess_exec(
            *argv, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE,
            start_new_session=True)
        self._task = asyncio.ensure_future(self._watch(url))

    async def _watch(self, fixed_url):
        assert self.proc is not None
        async for raw in self.proc.stderr:
            line = raw.decode(errors='replace')
            m = URL_RE.search(line)
            if m and self.state != 'running':
                await self._set('running', 'tunnel active', url=m.group(0))
            elif fixed_url and 'Registered tunnel connection' in line and self.state != 'running':
                await self._set('running', 'named tunnel active', url=fixed_url)
            elif 'ERR' in line and self.state == 'starting':
                self.message = line.strip()[-200:]
        code = await self.proc.wait()
        if self.state != 'stopped':
            await self._set('error' if code else 'stopped', f'cloudflared exited (code {code})', url='')

    async def stop(self):
        if self.proc and self.proc.returncode is None:
            self.state = 'stopped'
            self.proc.terminate()
            try:
                await asyncio.wait_for(self.proc.wait(), 5)
            except asyncio.TimeoutError:
                self.proc.kill()
        await self._set('stopped', '', url='')
