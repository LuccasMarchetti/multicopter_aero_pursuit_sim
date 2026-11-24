#!/usr/bin/env bash
# Installs the Aero GCS launcher on the computer that accesses the GCSs (docs/COMANDOS.md):
# ~/.local/share/aero_launcher/aero_launcher.py, the command ~/.local/bin/aero-launcher and the
# "Aero GCS" menu entry. Only Python 3.9+ is needed. Optional: --autostart (opens at login).
#   ./launcher/install.sh [--autostart]       |  ./launcher/install.sh --uninstall
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HOME/.local/share/aero_launcher"
BIN="$HOME/.local/bin/aero-launcher"
APP="$HOME/.local/share/applications/aero-launcher.desktop"
AUTO="$HOME/.config/autostart/aero-launcher.desktop"

if [ "${1:-}" = --uninstall ]; then
  rm -rf "$DEST" "$BIN" "$APP" "$AUTO"; echo "launcher removed"; exit 0
fi
mkdir -p "$DEST" "$HOME/.local/bin" "$(dirname "$APP")"
cp "$HERE/aero_launcher.py" "$DEST/"
printf '#!/bin/sh\nexec python3 "%s/aero_launcher.py" "$@"\n' "$DEST" > "$BIN"; chmod +x "$BIN"
entry() {
  printf '%s\n' '[Desktop Entry]' 'Type=Application' 'Name=Aero GCS' \
    'Comment=GCSs do aero_pursuit encontradas na rede' "Exec=$BIN $1" 'Icon=applications-science' \
    'Terminal=false' 'Categories=Science;Network;'
}
entry "" > "$APP"
if [ "${1:-}" = --autostart ]; then mkdir -p "$(dirname "$AUTO")"; entry "--no-browser" > "$AUTO"; fi
grep -qs 'HOME/.local/bin' "$HOME/.profile" "$HOME/.bashrc" || echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
echo "launcher installed: aero-launcher (or \"Aero GCS\" in the menu) -> http://localhost:8090/"
