#!/bin/bash
# Web ground station (aero_pursuit_gcs): low-latency video, 3D, parameters, joystick, logs and
# simulation control from the browser.
#
# Normally called by the `aero` command (native mode, docs/COMANDOS.md). Usage:
#   ./gcs.sh                     run in the foreground (Ctrl+C stops it)
#   ./gcs.sh open                start in the background (if not up) and open the home page
#   ./gcs.sh install-launcher    "Aero GCS" shortcut in the applications menu (same as open)
#   ./gcs.sh stop                stop the GCS started by open (or the service)
#   ./gcs.sh install-service     user service (systemd): starts by itself at login
#   ./gcs.sh uninstall-service
#   ./gcs.sh restart             restart the service (or the GCS of open)
#   ./gcs.sh status | logs       state / log of the service
#   ./gcs.sh token               show the remote access token
#
# Access: http://localhost:8080 (this machine) | https://<ip>:8443 (local network, self-signed
# certificate) | Cloudflare tunnel (Network tab, from anywhere).
# Extra arguments go to the server: --port, --https-port, --no-https, --log-level.

WS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO=${ROS_DISTRO:-jazzy}
UNIT=aero-gcs.service
UNIT_DIR="$HOME/.config/systemd/user"

run_server() {
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  if [ -f "$WS_PATH/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WS_PATH/install/setup.bash"
  else
    echo "Warning: workspace not built ($WS_PATH/install); run ./install.sh" >&2
  fi
  # Código-fonte na frente do instalado: mudanças na GCS valem sem recompilar.
  export PYTHONPATH="$WS_PATH/src/aero_pursuit_gcs:$PYTHONPATH"
  exec python3 -m aero_pursuit_gcs --workspace "$WS_PATH" "$@"
}

# HTTP port: GCS_PORT, else the one saved by the GCS (settings.json), else 8080.
GCS_PORT="${GCS_PORT:-$(python3 -c 'import json,os,sys; print(json.load(open(sys.argv[1])).get("http_port", 8080))' \
  "${AERO_GCS_CONFIG:-$HOME/.config/aero_gcs}/settings.json" 2>/dev/null || echo 8080)}"
gcs_up() { curl -s -o /dev/null --max-time 1 "http://localhost:$GCS_PORT/api/info"; }

case "${1:-}" in
  open)
    if ! gcs_up; then
      if systemctl --user is-enabled "$UNIT" >/dev/null 2>&1; then
        systemctl --user start "$UNIT"
      else
        mkdir -p "$HOME/.cache/aero_gcs"
        setsid nohup "$0" > "$HOME/.cache/aero_gcs/gcs.log" 2>&1 < /dev/null &
        echo $! > "$HOME/.cache/aero_gcs/gcs.pid"
      fi
      for _ in $(seq 40); do gcs_up && break; sleep 0.5; done
    fi
    gcs_up || { echo "The GCS did not start (see ~/.cache/aero_gcs/gcs.log)" >&2; exit 1; }
    [ -n "${GCS_NO_BROWSER:-}" ] || xdg-open "http://localhost:$GCS_PORT/" >/dev/null 2>&1 &
    ;;
  stop)
    if [ -f "$HOME/.cache/aero_gcs/gcs.pid" ]; then
      kill "$(cat "$HOME/.cache/aero_gcs/gcs.pid")" 2>/dev/null; rm -f "$HOME/.cache/aero_gcs/gcs.pid"
    fi
    systemctl --user stop "$UNIT" 2>/dev/null || true
    echo "GCS encerrada"
    ;;
  install-launcher)
    dir="$HOME/.local/share/applications"
    mkdir -p "$dir"
    printf '%s\n' '[Desktop Entry]' 'Type=Application' 'Name=Aero GCS' \
      'Comment=aero_pursuit ground station (Gazebo and Isaac Sim)' "Exec=$WS_PATH/../aero view" \
      'Icon=applications-science' 'Terminal=false' 'Categories=Science;Development;' > "$dir/aero-gcs.desktop"
    echo "Shortcut installed: $dir/aero-gcs.desktop (look for \"Aero GCS\" in the menu)"
    ;;
  install-service)
    mkdir -p "$UNIT_DIR"
    cat > "$UNIT_DIR/$UNIT" <<EOF
[Unit]
Description=Aero Pursuit GCS (web ground station)
After=network-online.target

[Service]
Type=simple
ExecStart=$WS_PATH/gcs.sh
Restart=on-failure
RestartSec=3
# A GCS lança a simulação: os processos dela sobrevivem a reinícios do serviço.
KillMode=process
Environment=ROS_DISTRO=$ROS_DISTRO
$( [ -n "${ROS_DOMAIN_ID:-}" ] && echo "Environment=ROS_DOMAIN_ID=$ROS_DOMAIN_ID" )

[Install]
WantedBy=default.target
EOF
    systemctl --user daemon-reload
    systemctl --user enable --now "$UNIT"
    echo "Service installed: $UNIT (systemctl --user status $UNIT)"
    echo "To start without a graphical login: sudo loginctl enable-linger $USER"
    ;;
  uninstall-service)
    systemctl --user disable --now "$UNIT" 2>/dev/null
    rm -f "$UNIT_DIR/$UNIT"
    systemctl --user daemon-reload
    echo "Service removed."
    ;;
  restart)
    if systemctl --user is-enabled "$UNIT" >/dev/null 2>&1; then
      systemctl --user restart "$UNIT" && echo "GCS reiniciada ($UNIT)"
    else
      "$0" stop >/dev/null; GCS_NO_BROWSER=1 "$0" open && echo "GCS reiniciada"
    fi
    ;;
  status) systemctl --user status "$UNIT" --no-pager ;;
  logs) journalctl --user -u "$UNIT" -f ;;
  token)
    python3 -c "import json,os;print(json.load(open(os.path.expanduser('~/.config/aero_gcs/settings.json')))['token'])"
    ;;
  -h|--help) sed -n 2,18p "$0" ;;
  *) run_server "$@" ;;
esac
