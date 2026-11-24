#!/usr/bin/env python3
"""Host agent of a GCS that runs in a container (aero gcs in docker mode, docs/COMANDOS.md).

The GCS runs in a container and needs to start/stop the Isaac world, which runs natively on the
host. This agent (system python3, started and stopped with the GCS by `aero gcs`) listens on
a Unix socket in ~/.local/state/aero/isaac/run (mode 600, shared with the container) and runs only
`host.sh <subcommand> <words>` for the subcommands below, with plain words (no shell). No new
credential: it lives as long as the GCS, as the same user.

Protocol: one JSON line with the argument list; the answer is the command output line by line
and a last line "\\0EXIT <code>".  Client: aero_pursuit_gcs/host_client.py.
"""

import json
import os
import re
import signal
import socketserver
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
HOST_SH = HERE / 'host.sh'
ALLOWED = {'up', 'world', 'ros', 'down', 'status', 'build', 'px4'}
WORD = re.compile(r'^[A-Za-z0-9_.:=/,+-]*$')


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            args = json.loads(self.rfile.readline().decode())
            if not (isinstance(args, list) and args and args[0] in ALLOWED and
                    all(isinstance(a, str) and WORD.match(a) for a in args)):
                raise ValueError('command not allowed')
        except (ValueError, UnicodeDecodeError) as e:
            self.wfile.write(f'recusado: {e}\n\0EXIT 2\n'.encode())
            return
        proc = subprocess.Popen(['bash', str(HOST_SH), *args], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        try:
            for line in proc.stdout:
                self.wfile.write(line)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass                    # client gone (GCS timeout): the command goes on
        self.wfile.write(f'\0EXIT {proc.wait()}\n'.encode())


class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True


def main():
    path = Path(sys.argv[1])
    path.unlink(missing_ok=True)
    old = os.umask(0o177)           # socket 600: only this user (the containers run as it)
    with Server(str(path), Handler) as srv:
        os.umask(old)               # not for the commands (PX4 makes its rootfs in /tmp)
        signal.signal(signal.SIGTERM, lambda *_: (path.unlink(missing_ok=True), os._exit(0)))
        srv.serve_forever()


if __name__ == '__main__':
    main()
