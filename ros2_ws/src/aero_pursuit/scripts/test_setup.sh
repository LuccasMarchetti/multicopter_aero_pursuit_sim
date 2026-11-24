#!/usr/bin/env bash
# Prepares a pursuit test in the simulation already airborne:
# raises the X4, turns on omniscient mode and fixes the controller parameters
# (overrides the param_overrides remembered by the GCS).
# Usage: test_setup.sh [distance=5.0]
set -e
D=${1:-5.0}
ros2 topic pub -1 /X4/enable std_msgs/msg/Bool "{data: true}" >/dev/null
timeout 3 ros2 topic pub -r 20 /X4/gazebo/command/twist geometry_msgs/msg/Twist "{linear: {z: 1.5}}" >/dev/null || true
ros2 topic pub -1 /X4/gazebo/command/twist geometry_msgs/msg/Twist "{}" >/dev/null
for p in "target_distance $D" "max_vel 2.0" "max_acc 3.0" "vertical_mode los" "mpc.mode kinematic"; do
  ros2 param set /pursuit_controller_node $p >/dev/null
done
ros2 topic pub -1 /target_tf_cmd aero_pursuit/msg/TargetTfCmd "{target_frame: X4, enable_tf_tracking: true}" >/dev/null
sleep 10
echo "ready: d=$D m, omniscient on"
