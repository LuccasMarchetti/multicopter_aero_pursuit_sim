#!/usr/bin/env bash
# Pursuer container on another computer (Raspberry Pi 5 by default): build, run and inspect it
# over SSH. The simulator side runs on this computer with AERO_PURSUER_HOST=remote.
#
#   docker/pursuer.sh build [host]            stage the sources on <host> and build aero-pursuer there
#                                             (detached: survives a dropped SSH session)
#   docker/pursuer.sh buildlog [host]         follow the build; returns when it ends
#   docker/pursuer.sh run   [host] [args...]  start the pursuer (args go to pursuer.launch.xml)
#   docker/pursuer.sh logs  [host]            follow the container log
#   docker/pursuer.sh stop  [host]            stop and remove the container
#   docker/pursuer.sh shell [host]            bash inside the running container
#   docker/pursuer.sh stats [host]            CPU / memory of the container and the board temperature
#
# host: user@address (or AERO_PI_HOST; no default). SSH by key. The build runs on
# the target itself (native arm64; ~40 min the first time on the Pi 5, mostly OpenCV), so no
# emulation is involved. The container uses the host network (DDS discovery on the LAN) and
# ROS_DOMAIN_ID / AERO_CAMERA_MOUNT from this shell.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE=aero-pursuer:latest
NAME=aero-pursuer
cmd="${1:-}"
host="${2:-${AERO_PI_HOST:-}}"
[ -n "$host" ] || [ -z "$cmd" ] || { echo "give the host: $0 $cmd user@IP (or AERO_PI_HOST)" >&2; exit 1; }
shift $(( $# > 2 ? 2 : $# )) || true
STAGE=aero_pursuer_build   # in the remote home

remote() { ssh -o BatchMode=yes "$host" "$@"; }

case "$cmd" in
  build)
    remote "mkdir -p ~/$STAGE/ros2_ws/src ~/$STAGE/docker"
    rsync -a --delete "$REPO/docker/Dockerfile.pursuer" "$host:$STAGE/docker/"
    rsync -a "$REPO/ros2_ws/install_acados.sh" "$host:$STAGE/ros2_ws/"
    rsync -a --delete --exclude '__pycache__' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
      "$REPO/ros2_ws/src/px4_msgs" "$REPO/ros2_ws/src/aero_pursuit" "$host:$STAGE/ros2_ws/src/"
    remote "cd ~/$STAGE; rm -f build.status; nohup sh -c 'docker build --progress=plain --target runtime -f docker/Dockerfile.pursuer -t $IMAGE . > build.log 2>&1; echo \$? > build.status' < /dev/null > /dev/null 2>&1 &"
    echo "build started on $host (~/$STAGE/build.log); follow it with: $0 buildlog $host"
    ;;
  buildlog)
    remote "cd ~/$STAGE && tail -n 5 -f build.log & pid=\$!; while [ ! -f ~/$STAGE/build.status ]; do sleep 5; done; kill \$pid; echo; echo build exit \$(cat ~/$STAGE/build.status)"
    ;;
  run)
    remote "docker rm -f $NAME >/dev/null 2>&1 || true; docker run -d --name $NAME --network host --ipc host \
      -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} -e AERO_CAMERA_MOUNT=${AERO_CAMERA_MOUNT:-fixed} \
      $IMAGE ros2 launch aero_pursuit pursuer.launch.xml $*"
    ;;
  logs)  remote "docker logs -f $NAME" ;;
  stop)  remote "docker rm -f $NAME" ;;
  shell) ssh -t "$host" "docker exec -it $NAME aero-entrypoint bash" ;;
  stats) remote "docker stats --no-stream $NAME; cat /sys/class/thermal/thermal_zone0/temp | awk '{print \"temp \" \$1/1000 \" C\"}'" ;;
  *) sed -n '2,16p' "$0"; exit 1 ;;
esac
