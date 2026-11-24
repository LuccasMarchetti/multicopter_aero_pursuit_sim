#!/usr/bin/env python3
"""Aero GCS launcher: a local web page with every GCS this computer sees on the network.

Run it on the computer that only accesses the GCSs (docs/COMANDOS.md). It needs only Python 3.9+
(no packages, no ROS, no repository: this file alone is enough).

    python3 aero_launcher.py [--port 8090] [--no-browser] [--host ADDR[:PORT] ...]

Discovery, repeated every minute and on "Search again": the /24 networks of this computer's
interfaces (LAN, ZeroTier...), the online Tailscale peers (`tailscale status --json`), localhost
and the addresses saved on the page (~/.config/aero_launcher/hosts.txt). An address is a GCS when
https://<addr>:<port>/api/info (port 8443 unless saved with another) answers {"kind": "aero_gcs"}.
The GCSs know nothing about each other or about this launcher; a click only opens the chosen
GCS in the browser, which asks for its token on the first access (cookie for 30 days).
"""

import argparse
import concurrent.futures as futures
import ipaddress
import json
import os
import platform
import shutil
import socket
import ssl
import subprocess
import threading
import time
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

DEFAULT_GCS_PORT = 8443
HOSTS_FILE = Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'aero_launcher' / 'hosts.txt'
SKIP_IFACES = ('lo', 'docker', 'br-', 'veth', 'virbr', 'tailscale')   # tailscale: peers list instead

_ctx = ssl.create_default_context()
_ctx.check_hostname = False
_ctx.verify_mode = ssl.CERT_NONE          # the GCSs use self-signed certificates


# -- candidates ---------------------------------------------------------------------------
def local_networks():
    """(interface, IPv4Interface) of this computer, without loopback/containers/Tailscale."""
    out = []
    if shutil.which('ip'):
        try:
            data = json.loads(subprocess.run(['ip', '-j', '-4', 'addr'], capture_output=True,
                                             text=True, timeout=3).stdout or '[]')
        except (OSError, ValueError, subprocess.SubprocessError):
            data = []
        for itf in data:
            name = itf.get('ifname', '')
            if name.startswith(SKIP_IFACES) or 'UP' not in itf.get('flags', []):
                continue
            for a in itf.get('addr_info', []):
                if a.get('family') == 'inet':
                    out.append((name, ipaddress.ip_interface(f"{a['local']}/{a['prefixlen']}")))
    if not out:                           # other systems: the address of the default route
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(('192.0.2.1', 9))
                out.append(('default', ipaddress.ip_interface(f'{s.getsockname()[0]}/24')))
        except OSError:
            pass
    return out


def subnet_hosts(itf):
    """Hosts of the /24 around the interface address (or of its network, if smaller)."""
    net = itf.network if itf.network.prefixlen >= 24 else ipaddress.ip_network(f'{itf.ip}/24', strict=False)
    if net.is_loopback or ipaddress.ip_address(itf.ip) in ipaddress.ip_network('100.64.0.0/10'):
        return []
    return [str(h) for h in net.hosts()]


TAILSCALE_SOCKET = '/var/run/tailscale/tailscaled.sock'


def _tailscale_status():
    """`tailscale status --json`, or the same from the tailscaled local API through its Unix
    socket (a GCS container has the socket mounted but no tailscale CLI)."""
    exe = shutil.which('tailscale')
    if exe:
        try:
            return json.loads(subprocess.run([exe, 'status', '--json'], capture_output=True, text=True,
                                             timeout=5).stdout or '{}')
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
    if not os.path.exists(TAILSCALE_SOCKET):
        return {}
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect(TAILSCALE_SOCKET)
            s.sendall(b'GET /localapi/v0/status HTTP/1.0\r\nHost: local-tailscaled.sock\r\n\r\n')
            data = b''
            while chunk := s.recv(65536):
                data += chunk
        return json.loads(data.split(b'\r\n\r\n', 1)[1] or b'{}')
    except (OSError, ValueError, IndexError):
        return {}


def tailscale_peers():
    """{ip: name} of the online Tailscale peers (and this computer)."""
    st = _tailscale_status()
    out = {}
    for p in [st.get('Self') or {}] + list((st.get('Peer') or {}).values()):
        ips = [ip for ip in p.get('TailscaleIPs') or [] if ':' not in ip]
        if ips and (p.get('Online') or p is st.get('Self')):
            out[ips[0]] = p.get('HostName') or ips[0]
    return out


def saved_hosts():
    try:
        return [ln.strip() for ln in HOSTS_FILE.read_text().splitlines() if ln.strip() and not ln.startswith('#')]
    except OSError:
        return []


def save_hosts(hosts):
    HOSTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    HOSTS_FILE.write_text(''.join(h + '\n' for h in dict.fromkeys(hosts)))


def split_addr(addr):
    host, _, port = addr.rpartition(':') if addr.count(':') == 1 else (addr, '', '')
    return (host or addr), int(port) if port.isdigit() else DEFAULT_GCS_PORT


# -- probing ------------------------------------------------------------------------------
def port_open(host, port, timeout):
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def probe(host, port, timeout=2.5):
    """(info, rtt_ms) of the GCS at host:port, or (None, None)."""
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(f'https://{host}:{port}/api/info', timeout=timeout, context=_ctx) as r:
            info = json.loads(r.read(65536))
    except Exception:
        return None, None
    if not isinstance(info, dict) or (info.get('kind') != 'aero_gcs' and 'backend' not in info):
        return None, None
    return info, round((time.monotonic() - t0) * 1000)


class Registry:
    """GCSs found so far, by name; each with the addresses that answered."""

    def __init__(self, extra):
        self.extra = list(extra)
        self.lock = threading.Lock()
        self.gcs = {}                     # name -> {'info', 'routes': {addr: {...}}, 'seen'}
        self.scanning = False
        self.last_scan = 0.0
        self.scanned = 0
        self.pool = futures.ThreadPoolExecutor(max_workers=128)

    def candidates(self):
        """{(host, port): (label, origin)} to try."""
        c = {('127.0.0.1', DEFAULT_GCS_PORT): ('this computer', 'local')}
        ts_iface = next((n for n in os.listdir('/sys/class/net') if n.startswith('tailscale')), 'tailscale0') \
            if os.path.isdir('/sys/class/net') else 'tailscale'
        for ip, name in tailscale_peers().items():
            c[(ip, DEFAULT_GCS_PORT)] = (name, ts_iface)
        for name, itf in local_networks():
            for h in subnet_hosts(itf):
                c.setdefault((h, DEFAULT_GCS_PORT), (h, name))
        for a in saved_hosts() + self.extra:
            c[split_addr(a)] = (a, 'saved')
        return c

    def _record(self, host, port, label, origin, info, rtt):
        name = info.get('name') or host
        addr = f'{host}:{port}'
        with self.lock:
            g = self.gcs.setdefault(name, {'routes': {}})
            g['info'], g['seen'] = info, time.time()
            g['routes'][addr] = {'host': host, 'port': port, 'origin': origin, 'label': label,
                                 'rtt': rtt, 'ok': True, 'seen': time.time()}

    def scan(self):
        """Full discovery: quick TCP check of every candidate, /api/info on the open ones."""
        with self.lock:
            if self.scanning:
                return
            self.scanning = True
        try:
            cand = self.candidates()
            self.scanned = len(cand)
            opened = {k: v for k, v, ok in zip(cand, cand.values(), self.pool.map(
                lambda k: port_open(k[0], k[1], 1.0 if k in self._known() else 0.4), cand)) if ok}
            for (host, port), (label, origin), (info, rtt) in zip(
                    opened, opened.values(), self.pool.map(lambda k: probe(*k), opened)):
                if info:
                    self._record(host, port, label, origin, info, rtt)
            self.last_scan = time.time()
        finally:
            self.scanning = False

    def _known(self):
        with self.lock:
            return {(r['host'], r['port']) for g in self.gcs.values() for r in g['routes'].values()}

    def refresh(self):
        """State of the GCSs already found (every few seconds)."""
        with self.lock:
            routes = [(n, a, dict(r)) for n, g in self.gcs.items() for a, r in g['routes'].items()]
        for (name, addr, r), (info, rtt) in zip(routes, self.pool.map(
                lambda x: probe(x[2]['host'], x[2]['port'], 2.0), routes)):
            with self.lock:
                g = self.gcs.get(name)
                if g is None or addr not in g['routes']:
                    continue
                route = g['routes'][addr]
                route['ok'], route['rtt'] = info is not None, rtt
                if info:
                    route['seen'] = time.time()
                    g['info'], g['seen'] = info, time.time()

    def snapshot(self, local_url=None):
        """local_url: link to the GCS on 127.0.0.1 (the page served by a GCS: its own '/app', valid
        from any device; the launcher on a PC: http://localhost:<http_port>/app)."""
        now = time.time()
        with self.lock:
            items = []
            for name, g in sorted(self.gcs.items()):
                routes = sorted(g['routes'].values(), key=lambda r: (not r['ok'], r['rtt'] or 1e9))
                online = any(r['ok'] for r in routes)
                if not online and now - g['seen'] > 600:
                    continue              # gone for 10 min: drop it
                info = g['info']
                local = any(r['origin'] == 'local' for r in routes)
                urls = [{'url': f"https://{r['host']}:{r['port']}/app", 'origin': r['origin'],
                         'host': r['host'], 'rtt': r['rtt'], 'ok': r['ok']}
                        for r in routes if r['host'] != '127.0.0.1']
                if local:                 # same computer: plain http, no token
                    urls.insert(0, {'url': local_url or f"http://localhost:{info.get('http_port', 8080)}/app",
                                    'origin': 'local', 'host': 'esta GCS' if local_url else 'localhost',
                                    'rtt': 0, 'ok': online})
                items.append({'name': name, 'online': online, 'info': info, 'urls': urls,
                              'local': local, 'age': round(now - g['seen'])})
            return {'items': items, 'scanning': self.scanning, 'last_scan': self.last_scan,
                    'scanned': self.scanned, 'hosts': saved_hosts(),
                    'tailscale': bool(shutil.which('tailscale') or os.path.exists(TAILSCALE_SOCKET))}


# -- web page -----------------------------------------------------------------------------
PAGE = r'''<!doctype html>
<html lang="pt-BR"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aero GCS</title>
<link rel="icon" href="data:image/svg+xml,<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 16 16'><circle cx='8' cy='8' r='6' fill='%234fc3f7'/></svg>">
<style>
:root { --bg:#0f1419; --card:#172028; --line:#26323d; --fg:#e3e9ee; --muted:#8a99a6; --acc:#4fc3f7;
        --good:#5bd18b; --bad:#ef6a6a; --warn:#f2c14e; color-scheme: dark; }
@media (prefers-color-scheme: light) { :root { --bg:#f3f5f7; --card:#fff; --line:#d9dfe5; --fg:#1b2329;
        --muted:#5d6b77; --acc:#0277bd; --good:#1f9d55; --bad:#c62828; --warn:#a86b00; color-scheme: light; } }
* { box-sizing: border-box; }
body { margin:0; background:var(--bg); color:var(--fg); font:15px/1.45 system-ui,sans-serif; }
header { display:flex; align-items:center; gap:12px; flex-wrap:wrap; padding:14px 20px; border-bottom:1px solid var(--line); }
header h1 { font-size:18px; margin:0; } .muted { color:var(--muted); } .sp { flex:1; }
main { max-width:1000px; margin:0 auto; padding:20px 16px 40px; }
#cards { display:grid; grid-template-columns:repeat(auto-fill,minmax(300px,1fr)); gap:14px; }
.card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px; cursor:pointer;
        transition:border-color .15s, transform .15s; }
.card:hover { border-color:var(--acc); transform:translateY(-1px); }
.card.off { opacity:.55; cursor:default; } .card.off:hover { border-color:var(--line); transform:none; }
.card h2 { margin:0 0 8px; font-size:17px; display:flex; align-items:center; gap:8px; }
.dot { width:10px; height:10px; border-radius:50%; background:var(--muted); flex:none; }
.dot.good { background:var(--good); } .dot.bad { background:var(--bad); } .dot.warn { background:var(--warn); }
.tag { margin-left:auto; font-size:12px; padding:2px 8px; border-radius:99px; border:1px solid var(--line); }
.tag.isaac { border-color:#76b900; color:#76b900; }
.kv { display:grid; grid-template-columns:auto 1fr; gap:3px 12px; font-size:13px; }
.kv span:nth-child(odd) { color:var(--muted); }
.routes { margin-top:10px; display:flex; flex-wrap:wrap; gap:6px; }
.routes a { font-size:12px; color:var(--acc); text-decoration:none; border:1px solid var(--line); border-radius:6px; padding:2px 7px; }
.routes a.down { color:var(--muted); text-decoration:line-through; }
button { font:inherit; background:var(--card); color:var(--fg); border:1px solid var(--line); border-radius:6px; padding:6px 12px; cursor:pointer; }
button:hover { border-color:var(--acc); }
input { font:inherit; background:var(--card); color:var(--fg); border:1px solid var(--line); border-radius:6px; padding:6px 10px; min-width:0; }
section { margin-top:28px; } section h3 { font-size:14px; margin:0 0 8px; }
.row { display:flex; gap:8px; flex-wrap:wrap; align-items:center; }
.chip { font-size:13px; border:1px solid var(--line); border-radius:99px; padding:2px 4px 2px 10px; display:inline-flex; gap:6px; align-items:center; }
.chip button { padding:0 6px; border:none; }
#empty { padding:30px; text-align:center; border:1px dashed var(--line); border-radius:10px; }
</style></head><body>
<header><h1>Aero GCS</h1><span class="muted" id="sub">looking for GCSs on the network…</span><span class="sp"></span>
<button id="scan">Search again</button></header>
<main>
<div id="cards"></div>
<div id="empty" class="muted" hidden>No GCS found yet. Check that it is up (<code>aero status</code> on its computer)
and that this computer reaches its address (same network or Tailscale), or add the address below.</div>
<section><h3>Saved addresses</h3>
<p class="muted" style="margin:0 0 8px;font-size:13px">For GCSs outside the networks scanned automatically (local /24 network and Tailscale).
Format <code>host</code> or <code>host:port</code> (HTTPS, default 8443).</p>
<div class="row" id="hosts"></div>
<div class="row" style="margin-top:8px"><input id="addr" placeholder="e.g. 192.168.0.10 or my-pc:8443"><button id="add">Add</button></div>
</section>
<p class="muted" style="margin-top:28px;font-size:13px">The first time on each GCS the browser warns about the self-signed certificate (accept it) and the GCS asks
for the token (<code>aero gcs token</code> on its computer); it is then remembered for 30 days.</p>
</main>
<script>
const BASE = window.LAUNCHER_BASE || '';     // '' (launcher on a PC) or '/rede' (page served by a GCS)
const $ = (id) => document.getElementById(id);
const esc = (s) => String(s ?? '').replace(/[&<>"]/g, (c) => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
async function api(path, body) {
  const r = await fetch(BASE + path, body ? {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)} : {});
  return r.json();
}
function card(it) {
  const d = it.info || {}, best = it.urls.find((u) => u.ok) || it.urls[0];
  const state = !it.online ? ['bad', `no answer for ${it.age} s`] : d.busy ? ['warn', esc(d.busy) + '…']
    : d.running ? ['good', 'running' + (d.rtf ? ` · RTF ${d.rtf.toFixed(2)}` : '')] : ['', 'stopped'];
  const av = Object.entries(d.available || {}).map(([k, v]) => `${k === 'isaac' ? 'Isaac Sim' : 'Gazebo'}${v ? '' : ' (unavailable)'}`).join(', ');
  const el = document.createElement('div');
  el.className = 'card' + (it.online ? '' : ' off');
  el.title = it.online ? `Abrir ${best.url}` : '';
  el.innerHTML = `<h2><span class="dot ${state[0]}"></span>${esc(it.name)}${it.local ? ' <span class="muted" style="font-size:13px">(this computer)</span>' : ''}
      <span class="tag ${d.backend === 'isaac' ? 'isaac' : ''}">${esc(d.simulator || '?')}</span></h2>
    <div class="kv"><span>Simulation</span><span>${state[1]}</span>
      <span>Scenario</span><span>${esc(d.scenario || '—')}</span>
      <span>Simulators</span><span>${esc(av || '—')}</span>
      ${d.tunnel ? `<span>Cloudflare</span><span>${esc(d.tunnel)}</span>` : ''}</div>
    <div class="routes"></div>`;
  const routes = el.querySelector('.routes');
  for (const u of it.urls) {
    const a = document.createElement('a');
    a.href = u.url; a.className = u.ok ? '' : 'down';
    a.textContent = `${u.host} · ${u.origin}${u.ok && u.rtt ? ` · ${u.rtt} ms` : ''}`;
    a.onclick = (e) => e.stopPropagation();
    routes.append(a);
  }
  if (it.online) el.onclick = () => { location.href = best.url; };
  return el;
}
function chip(h) {
  const c = document.createElement('span'); c.className = 'chip'; c.textContent = h;
  const b = document.createElement('button'); b.textContent = '×'; b.title = 'remove';
  b.onclick = () => api('/api/hosts', {remove: h}).then(refresh);
  c.append(b); return c;
}
async function refresh() {
  try {
    const s = await api('/api/list');
    $('cards').replaceChildren(...s.items.map(card));
    $('empty').hidden = s.items.length > 0 || s.scanning;
    const ago = s.last_scan ? Math.round(Date.now() / 1000 - s.last_scan) : null;
    $('sub').textContent = s.scanning ? `searching… (${s.scanned} addresses)`
      : `${s.items.length} GCS · ${s.scanned} addresses scanned${ago !== null ? ` ${ago} s ago` : ''}`;
    $('scan').disabled = s.scanning;
    $('hosts').replaceChildren(...(s.hosts.length ? s.hosts.map(chip) : [Object.assign(document.createElement('span'), {className:'muted', textContent:'none'})]));
  } catch (e) { $('sub').textContent = 'launcher down: ' + e.message; }
}
$('scan').onclick = () => api('/api/scan', {}).then(refresh);
$('add').onclick = async () => { const v = $('addr').value.trim(); if (!v) return; $('addr').value = '';
  await api('/api/hosts', {add: v}); refresh(); };
$('addr').onkeydown = (e) => { if (e.key === 'Enter') $('add').click(); };
refresh(); setInterval(refresh, 2000);
</script></body></html>'''


def make_handler(reg):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype='application/json'):
            data = body.encode() if isinstance(body, str) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                return self._send(200, PAGE, 'text/html')
            if self.path == '/api/list':
                return self._send(200, reg.snapshot())
            self._send(404, {'error': 'not found'})

        def do_POST(self):
            try:
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length') or 0)) or b'{}')
            except ValueError:
                return self._send(400, {'error': 'json'})
            if self.path == '/api/scan':
                threading.Thread(target=reg.scan, daemon=True).start()
                return self._send(200, {'ok': True})
            if self.path == '/api/hosts':
                hosts = saved_hosts()
                if body.get('add'):
                    hosts.append(str(body['add']).strip().removeprefix('https://').rstrip('/'))
                if body.get('remove'):
                    hosts = [h for h in hosts if h != body['remove']]
                save_hosts(hosts)
                threading.Thread(target=reg.scan, daemon=True).start()
                return self._send(200, {'ok': True})
            self._send(404, {'error': 'not found'})
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--port', type=int, default=8090, help='port of this page (default 8090)')
    ap.add_argument('--no-browser', action='store_true', help='do not open the browser')
    ap.add_argument('--host', action='append', default=[], metavar='ADDR[:PORT]',
                    help='extra address to probe (besides the ones saved on the page)')
    args = ap.parse_args()

    reg = Registry(args.host)
    try:
        srv = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(reg))
    except OSError:                       # already running: just open it
        print(f'launcher already running: http://localhost:{args.port}/')
        if not args.no_browser:
            webbrowser.open(f'http://localhost:{args.port}/')
        return

    def loop():
        last_full = 0.0
        while True:
            if time.monotonic() - last_full > 60:
                reg.scan()
                last_full = time.monotonic()
            else:
                reg.refresh()
            time.sleep(3)
    threading.Thread(target=loop, daemon=True).start()

    url = f'http://localhost:{args.port}/'
    print(f'Aero GCS launcher: {url}  (Ctrl+C encerra; {platform.node()})')
    if not args.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
