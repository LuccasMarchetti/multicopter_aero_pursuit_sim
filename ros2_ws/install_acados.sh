#!/usr/bin/env bash
# acados (optimal control solvers: SQP/SQP-RTI + HPIPM) and its Python code generator, used by
# the MPC controllers of aero_pursuit (ros2_ws/src/aero_pursuit/acados/). No root needed.
#
#   ros2_ws/install_acados.sh [prefix]      default: $ACADOS_ROOT or ~/.local/share/acados
#
# Result: <prefix>/{include,lib} (C library), <prefix>/venv (acados_template + CasADi for the code
# generation at build time) and <prefix>/bin/t_renderer. CMake finds it through ACADOS_ROOT or
# the default prefix; install.sh and the Docker images call this script. Idempotent.
set -euo pipefail
ACADOS_REF="${ACADOS_REF:-v0.6.1}"
PREFIX="${1:-${ACADOS_ROOT:-$HOME/.local/share/acados}}"
TERA_VERSION=0.2.1
JOBS="${JOBS:-$(nproc)}"

case "$(uname -m)" in
  x86_64) BLASFEO_TARGET=X64_AUTOMATIC; TERA_ARCH=linux-amd64 ;;
  aarch64|arm64) BLASFEO_TARGET=ARMV8A_ARM_CORTEX_A76; TERA_ARCH=linux-arm64 ;;  # Raspberry Pi 5
  *) BLASFEO_TARGET=GENERIC; TERA_ARCH=linux-amd64 ;;
esac

if [ -f "$PREFIX/lib/libacados.so" ] && [ -x "$PREFIX/venv/bin/python" ] && [ -x "$PREFIX/bin/t_renderer" ] &&
  [ "$(cat "$PREFIX/.aero_ref" 2>/dev/null)" = "$ACADOS_REF" ]; then
  echo "acados $ACADOS_REF already installed in $PREFIX"; exit 0
fi

echo "== acados $ACADOS_REF in $PREFIX (BLASFEO $BLASFEO_TARGET)"
if [ ! -d "$PREFIX/.git" ]; then
  git clone -q --depth 1 --branch "$ACADOS_REF" https://github.com/acados/acados.git "$PREFIX"
else
  # Existing clone of another version: move it to $ACADOS_REF (an upgrade rebuilds everything).
  git -C "$PREFIX" fetch -q --depth 1 origin tag "$ACADOS_REF"
  git -C "$PREFIX" checkout -q -f "$ACADOS_REF"
  rm -rf "$PREFIX/build" "$PREFIX/lib" "$PREFIX/include"
fi
git -C "$PREFIX" submodule update -q --init --recursive --depth 1
cmake -S "$PREFIX" -B "$PREFIX/build" -DCMAKE_BUILD_TYPE=Release -DACADOS_INSTALL_DIR="$PREFIX" \
  -DBLASFEO_TARGET="$BLASFEO_TARGET" -DACADOS_WITH_QPOASES=OFF -DACADOS_WITH_OPENMP=OFF \
  -DACADOS_EXAMPLES=OFF -DACADOS_UNIT_TESTS=OFF > "$PREFIX/cmake.log"
# BLASFEO writes blasfeo_target.h into the build tree, but some ARMv8 kernels include it by the
# source path (../../include/blasfeo_target.h): the Raspberry Pi build failed there. Copy it over.
target_h=$(find "$PREFIX/build" -name blasfeo_target.h -print -quit)
if [ -n "$target_h" ] && [ ! -f "$PREFIX/external/blasfeo/include/blasfeo_target.h" ]; then
  cp "$target_h" "$PREFIX/external/blasfeo/include/"
fi
cmake --build "$PREFIX/build" -j "$JOBS" --target install > "$PREFIX/build.log"

# Code generator (build time only): acados_template + CasADi in their own venv.
[ -x "$PREFIX/venv/bin/python" ] || python3 -m venv "$PREFIX/venv"
"$PREFIX/venv/bin/pip" install -q --upgrade pip
"$PREFIX/venv/bin/pip" install -q -e "$PREFIX/interfaces/acados_template"

# Template renderer used by acados_template (downloaded here, not at the first build).
mkdir -p "$PREFIX/bin"
curl -fsSL -o "$PREFIX/bin/t_renderer" \
  "https://github.com/acados/tera_renderer/releases/download/v$TERA_VERSION/t_renderer-v$TERA_VERSION-$TERA_ARCH"
chmod +x "$PREFIX/bin/t_renderer"
echo "$ACADOS_REF" > "$PREFIX/.aero_ref"
echo "== acados ready: $PREFIX (CasADi $("$PREFIX/venv/bin/python" -c 'import casadi; print(casadi.__version__)'))"
