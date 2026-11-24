"""Client of isaac/host_agent.py: `python3 -m aero_pursuit_gcs.host_client <socket> <args...>`
runs isaac/host.sh <args> on the Isaac host, prints its output and exits with its code."""

import json
import socket
import sys


def main():
    path, args = sys.argv[1], sys.argv[2:]
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(path)
    except OSError as e:
        print(f'host agent unavailable ({path}: {e}); restart the GCS: aero gcs restart')
        sys.exit(3)
    s.sendall((json.dumps(args) + '\n').encode())
    with s.makefile('rb') as f:
        for raw in f:
            line = raw.decode(errors='replace')
            if line.startswith('\0EXIT '):
                sys.exit(int(line.split()[1]))
            sys.stdout.write(line)
            sys.stdout.flush()
    sys.exit(4)                     # connection closed without an exit code


if __name__ == '__main__':
    main()
