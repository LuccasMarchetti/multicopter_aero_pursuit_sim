#!/bin/bash
# Diagnóstico: o Gazebo está renderizando as câmeras na GPU?
#   ./check_gpu_rendering.sh          (com a simulação rodando)
# Lê /proc/<pid>/fdinfo dos processos do Gazebo: clientes DRM de hardware expõem
# `drm-driver` (amdgpu, i915, nvidia...) e o tempo acumulado do motor gráfico
# (`drm-engine-gfx`). Sem cliente DRM => renderização em software (llvmpipe/CPU).

echo "== OpenGL driver of this environment =="
if command -v glxinfo &> /dev/null && [ -n "$DISPLAY" ]; then
  glxinfo -B 2>/dev/null | grep -E "OpenGL (vendor|renderer)|Accelerated"
else
  echo "(glxinfo unavailable or no DISPLAY; install mesa-utils)"
fi
[ -n "$LIBGL_ALWAYS_SOFTWARE" ] && echo "Aviso: LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"

echo
echo "== Gazebo processes =="
PIDS=$(pgrep -f "^(ruby .*)?gz sim")
if [ -z "$PIDS" ]; then
  echo "No 'gz sim' running."
  exit 1
fi

gpu_ns() {  # soma de drm-engine-gfx (ns) de um pid
  cat /proc/"$1"/fdinfo/* 2>/dev/null | awk '/^drm-engine-gfx:/ {s += $2} END {print s + 0}'
}

found=0
for pid in $PIDS; do
  cmd=$(tr '\0' ' ' < /proc/"$pid"/cmdline 2>/dev/null | cut -c1-90)
  driver=$(cat /proc/"$pid"/fdinfo/* 2>/dev/null | awk '/^drm-driver:/ {print $2; exit}')
  if [ -n "$driver" ]; then
    t0=$(gpu_ns "$pid"); sleep 1; t1=$(gpu_ns "$pid")
    busy=$(awk -v a="$t0" -v b="$t1" 'BEGIN {printf "%.1f", (b - a) / 1e7}')
    echo "PID $pid [$driver] graphics engine use ~${busy}% | $cmd"
    found=1
  else
    echo "PID $pid [no DRM client] | $cmd"
  fi
done

echo
if [ $found -eq 1 ]; then
  echo "OK: Gazebo uses the GPU (OGRE2). Camera/depth sensors are rendered on it."
else
  echo "WARNING: no Gazebo process uses the GPU; check the driver (llvmpipe?)."
fi
