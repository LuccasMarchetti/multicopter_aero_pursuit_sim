"""Video hub: receives frames from video_streamer (Unix socket) and distributes them to the
browsers over WebSocket with congestion control.

Minimum latency by construction: each client acknowledges (ack) every received frame;
with more than `max_inflight` unacknowledged frames, new ones are dropped instead
of queued (the queue is what caused the ~20 s in Foxglove) and the client waits for the
next keyframe, requested right away from the encoder. In adaptive mode the encoder
bitrate follows the measured bandwidth of the worst client.
"""

import asyncio
import collections
import json
import logging
import os
import struct
import time
from pathlib import Path

log = logging.getLogger('gcs.video')

# Ver aero_pursuit/streaming/frame_packet.hpp. Campos desempacotados:
#  0 magic 1 type 2 flags 3 codec 4 w 5 h 6 src_w 7 src_h 8 seq 9 len
# 10 stamp_ns 11 recv_ns 12 encoded_ns 13 sim_age_ns
HEADER = struct.Struct('<IBBBxIIIIIIqqqq')
F_SEQ, F_LEN, F_RECV, F_ENC = 8, 9, 11, 12
MAGIC = 0x31564741
TYPE_FRAME, TYPE_INFO, TYPE_ERROR = 1, 2, 3
CODEC_MJPEG = 3
assert HEADER.size == 64


def socket_path(channel: str = 'main') -> Path:
    """One socket per channel; the same name as the VideoStreamerNode (`channel` parameter)."""
    runtime = os.environ.get('XDG_RUNTIME_DIR')
    base = Path(runtime) / 'aero_gcs' if runtime else Path(f'/tmp/aero_gcs-{os.getuid()}')
    return base / ('video.sock' if channel == 'main' else f'video_{channel}.sock')


def parse_kv(text: str) -> dict:
    out = {}
    for tok in text.split():
        if '=' in tok:
            k, v = tok.split('=', 1)
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out


class VideoClient:
    """A browser connected to /ws/video."""

    def __init__(self, hub, ws, transport, peer):
        self.hub = hub
        self.ws = ws
        self.transport = transport
        self.peer = peer
        self.enabled = False
        self.need_key = True
        self.queue = collections.deque()
        self.event = asyncio.Event()
        self.inflight = collections.OrderedDict()  # seq -> (bytes, t_envio)
        self.rtt = collections.deque(maxlen=60)
        self.win_sent = self.win_bytes = self.win_drop = self.win_acked = 0
        self.win_skip = 0           # skipped waiting for a keyframe (config change etc.)
        self.total_drop = 0
        self.task = asyncio.ensure_future(self._sender())

    def buffered(self) -> int:
        try:
            return self.transport.get_write_buffer_size() if self.transport else 0
        except Exception:  # transporte fechado
            return 0

    def offer(self, packet: bytes, seq: int, key: bool) -> bool:
        """Delivers or drops a frame. Returns False if it had to drop it."""
        if not self.enabled:
            return True
        if self.need_key and not key:
            self.win_skip += 1
            return True     # already waiting for the requested keyframe
        now = time.monotonic()
        # Lost confirmations (reloaded tab etc.) must not lock the client.
        while self.inflight and now - next(iter(self.inflight.values()))[1] > 2.0:
            self.inflight.popitem(last=False)
        if (len(self.inflight) >= self.hub.max_inflight or self.queue or
                self.buffered() > 256 * 1024):
            self._drop()
            self.need_key = True
            return False
        self.need_key = False
        self.queue.append((seq, packet))
        self.event.set()
        return True

    def _drop(self):
        self.win_drop += 1
        self.total_drop += 1

    async def _sender(self):
        try:
            while not self.ws.closed:
                await self.event.wait()
                self.event.clear()
                while self.queue:
                    seq, packet = self.queue.popleft()
                    self.inflight[seq] = (len(packet), time.monotonic())
                    await self.ws.send_bytes(packet)
                    self.win_sent += 1
                    self.win_bytes += len(packet)
        except (ConnectionResetError, asyncio.CancelledError, RuntimeError):
            pass

    def on_ack(self, seq: int):
        entry = self.inflight.pop(seq, None)
        if entry is None:
            return
        size, t = entry
        self.rtt.append((time.monotonic() - t) * 1000.0)
        self.win_acked += size
        # Earlier frames without an ack (dropped by the browser) also go out.
        while self.inflight and next(iter(self.inflight)) < seq:
            self.inflight.popitem(last=False)

    def snapshot(self, dt: float) -> dict:
        rtt = sorted(self.rtt)
        snap = {
            'op': 'vstats',
            'sent_fps': self.win_sent / dt,
            'sent_kbps': self.win_bytes * 8 / 1000 / dt,
            'goodput_kbps': self.win_acked * 8 / 1000 / dt,
            'drop_fps': self.win_drop / dt,         # due to congestion
            'skip_fps': self.win_skip / dt,         # waiting for a keyframe
            'drops_total': self.total_drop,
            'ack_ms': rtt[len(rtt) // 2] if rtt else None,
            'ack_p95_ms': rtt[int(len(rtt) * 0.95)] if rtt else None,
            'inflight': len(self.inflight),
            'buffered': self.buffered(),
        }
        self.win_sent = self.win_bytes = self.win_drop = self.win_acked = self.win_skip = 0
        return snap

    def close(self):
        self.task.cancel()


class VideoHub:
    """A video channel (one video_streamer): `main` = drone camera, `spectator` =
    Gazebo free camera. Each channel's configuration lives in settings[`key`]."""

    def __init__(self, settings, broadcast, channel='main', key='stream'):
        self.settings = settings
        self.broadcast = broadcast      # async fn(dict) -> sends to every control client
        self.channel = channel
        self.key = key
        self.path = socket_path(channel)
        self.clients = set()
        self.writer = None
        self.info = {}
        self.last_error = ''
        self.effective_bitrate = int(settings['stream']['bitrate'])
        self._last_key_req = 0.0
        self._in_frames = 0
        self._in_bytes = 0
        self.in_fps = 0.0
        self.in_kbps = 0.0
        self.encode_ms = None
        self._enc_samples = collections.deque(maxlen=60)
        self._clean_secs = 0
        self._server = None

    # -- configuration ------------------------------------------------------------
    @property
    def cfg(self) -> dict:
        return self.settings[self.key]

    @property
    def max_inflight(self) -> int:
        return max(1, int(self.cfg.get('max_inflight', 2)))

    @property
    def active(self) -> bool:
        return any(c.enabled for c in self.clients)

    def _cfg_line(self, only_bitrate=False) -> str:
        c = self.cfg
        if only_bitrate:
            return f'cfg bitrate={self.effective_bitrate}\n'
        quality = c['jpeg_quality'] if c['codec'] == 'mjpeg' else c['qp']
        return (f"cfg active={1 if self.active else 0} topic={c['topic']} codec={c['codec']} "
                f"backend={c['backend']} scale={float(c['scale']):.4f} fps={float(c['fps']):.3f} "
                f"bitrate={self.effective_bitrate} rc={c['rc']} quality={int(quality)} "
                f"gop={int(c['gop'])} depth_min={float(c['depth_min']):.3f} "
                f"depth_max={float(c['depth_max']):.3f}\n")

    def _send_line(self, line: str):
        if self.writer is not None and not self.writer.is_closing():
            self.writer.write(line.encode())

    def push_config(self):
        self._send_line(self._cfg_line())

    def set_config(self, values: dict):
        allowed = {k: v for k, v in values.items() if k in self.cfg}
        old = dict(self.cfg)
        self.settings.update(self.key, allowed)
        if 'bitrate' in allowed or not self.cfg['adaptive']:
            self.effective_bitrate = int(self.cfg['bitrate'])
        self.push_config()
        structural = any(old.get(k) != self.cfg.get(k) for k in
                         ('topic', 'codec', 'backend', 'scale', 'rc', 'qp', 'jpeg_quality', 'gop'))
        if structural:
            for c in self.clients:
                c.need_key = True
        self.request_key(force=True)

    def request_key(self, force=False):
        now = time.monotonic()
        if force or now - self._last_key_req > 0.3:
            self._last_key_req = now
            self._send_line('key\n')

    def state(self) -> dict:
        return {
            'op': 'stream.state',
            'channel': self.channel,
            'connected': self.writer is not None,
            'info': self.info,
            'error': self.last_error,
            'cfg': self.cfg,
            'effective_bitrate': self.effective_bitrate,
            'in_fps': self.in_fps,
            'in_kbps': self.in_kbps,
            'encode_ms': self.encode_ms,
            'viewers': sum(1 for c in self.clients if c.enabled),
        }

    # -- clientes ----------------------------------------------------------------
    def add_client(self, ws, transport, peer) -> VideoClient:
        c = VideoClient(self, ws, transport, peer)
        self.clients.add(c)
        return c

    def remove_client(self, c: VideoClient):
        c.close()
        self.clients.discard(c)
        self.push_config()      # may disable the encoder

    def set_enabled(self, c: VideoClient, on: bool):
        was = self.active
        c.enabled = on
        c.need_key = True
        if self.active != was:
            self.push_config()
        self.request_key(force=True)

    # -- streamer ----------------------------------------------------------------
    async def start(self):
        self.path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if self.path.exists():
            self.path.unlink()
        self._server = await asyncio.start_unix_server(self._on_streamer, path=str(self.path))
        os.chmod(self.path, 0o600)
        asyncio.ensure_future(self._stats_loop())
        log.info('Channel %s: waiting for the video_streamer at %s', self.channel, self.path)

    async def stop(self):
        if self._server:
            self._server.close()
        if self.path.exists():
            self.path.unlink()

    async def _on_streamer(self, reader, writer):
        if self.writer is not None and not self.writer.is_closing():
            # One streamer at a time. Refusing the new one (instead of dropping the old one) prevents
            # two containers (two simulations) from knocking each other down in a loop.
            log.warning('Channel %s: second video_streamer refused (one is already connected)', self.channel)
            writer.close()
            return
        self.writer = writer
        self.info = {}
        self.last_error = ''
        log.info('Channel %s: video_streamer connected', self.channel)
        self.push_config()
        self.request_key(force=True)
        await self.broadcast(self.state())
        try:
            while True:
                head = await reader.readexactly(HEADER.size)
                f = HEADER.unpack(head)
                if f[0] != MAGIC:
                    log.error('Invalid packet from the streamer; reconnecting')
                    break
                payload = await reader.readexactly(f[F_LEN]) if f[F_LEN] else b''
                kind = f[1]
                if kind == TYPE_FRAME:
                    self._on_frame(head, payload, f)
                elif kind == TYPE_INFO:
                    self.info = parse_kv(payload.decode(errors='replace'))
                    self.last_error = ''
                    await self.broadcast(self.state())
                elif kind == TYPE_ERROR:
                    self.last_error = payload.decode(errors='replace')
                    await self.broadcast(self.state())
        except (asyncio.IncompleteReadError, ConnectionResetError):
            pass
        finally:
            if self.writer is writer:
                self.writer = None
                self.info = {}
                await self.broadcast(self.state())
            writer.close()
            log.info('Channel %s: video_streamer disconnected', self.channel)

    def _on_frame(self, head: bytes, payload: bytes, f):
        self._in_frames += 1
        self._in_bytes += len(payload)
        recv_ns, enc_ns = f[F_RECV], f[F_ENC]
        if recv_ns and enc_ns:
            self._enc_samples.append((enc_ns - recv_ns) / 1e6)
        key = bool(f[2] & 1) or f[3] == CODEC_MJPEG
        packet = head + payload
        need_key = False
        for c in self.clients:
            if not c.offer(packet, f[F_SEQ], key):
                need_key = True
        if need_key and f[3] != CODEC_MJPEG:
            self.request_key()

    async def _stats_loop(self):
        t_last = time.monotonic()
        while True:
            await asyncio.sleep(1.0)
            now = time.monotonic()
            dt = max(1e-3, now - t_last)
            t_last = now
            self.in_fps = self._in_frames / dt
            self.in_kbps = self._in_bytes * 8 / 1000 / dt
            self._in_frames = self._in_bytes = 0
            enc = sorted(self._enc_samples)
            self.encode_ms = enc[len(enc) // 2] if enc else None
            snaps = []
            for c in list(self.clients):
                snap = c.snapshot(dt)
                snaps.append((c, snap))
                if not c.ws.closed:
                    try:
                        await c.ws.send_str(json.dumps(snap))
                    except (ConnectionResetError, RuntimeError):
                        pass
            self._adapt(snaps)
            await self.broadcast(self.state())

    def _adapt(self, snaps):
        user = int(self.cfg['bitrate'])
        if not self.cfg.get('adaptive') or self.cfg['codec'] == 'mjpeg':
            if self.effective_bitrate != user:
                self.effective_bitrate = user
                self._send_line(self._cfg_line(only_bitrate=True))
            return
        active = [s for c, s in snaps if c.enabled]
        if not active:
            return
        congested = any(s['drop_fps'] > 0 or (s['ack_p95_ms'] or 0) > 400 for s in active)
        floor = int(self.cfg.get('min_bitrate', 500))
        new = self.effective_bitrate
        if congested:
            self._clean_secs = 0
            goodput = min(s['goodput_kbps'] for s in active)
            target = 0.75 * goodput if goodput > 0 else 0.5 * self.effective_bitrate
            new = max(floor, min(self.effective_bitrate, int(target)))
        else:
            self._clean_secs += 1
            if self._clean_secs >= 2 and self.effective_bitrate < user:
                new = min(user, int(self.effective_bitrate * 1.15) + 100)
        if abs(new - self.effective_bitrate) > 0.03 * self.effective_bitrate:
            self.effective_bitrate = new
            self._send_line(self._cfg_line(only_bitrate=True))
