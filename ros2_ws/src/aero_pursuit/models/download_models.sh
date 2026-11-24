#!/usr/bin/env bash
# Downloads the class-agnostic tracker models used by vision_tracker_node
# (parameter tracker_model). All run on OpenCV DNN (OpenCV >= 4.9, see install_opencv.sh);
# no ONNX Runtime needed. Uses curl (present in the Docker image) or wget.
#   vit        ViTTrack (OpenCV Zoo)                     ~0.7 MB  (default)
#   nano       NanoTrack v2 (HonglinChu/SiamTrackers)    ~1.8 MB
#   dasiamrpn  DaSiamRPN (OpenCV samples)                ~160 MB  only with --dasiamrpn or DASIAMRPN=1
set -euo pipefail
cd "$(dirname "$0")"

fetch() {  # fetch <file> <url>
  if [ -s "$1" ]; then
    echo "$1 already exists."
    return
  fi
  echo "Downloading $1..."
  if command -v curl > /dev/null; then
    curl -fL --retry 3 -o "$1.part" "$2"
  elif command -v wget > /dev/null; then
    wget -O "$1.part" "$2"
  else
    echo "Install curl or wget to download $1." >&2
    exit 1
  fi
  # An HTML page (moved link, Git LFS pointer page) is not a model.
  if head -c 64 "$1.part" | grep -qi '<html\|<!doctype'; then
    rm -f "$1.part"
    echo "The address of $1 returned an HTML page, not the model: $2" >&2
    exit 1
  fi
  mv "$1.part" "$1"
}

fetch object_tracking_vittrack_2023sep.onnx \
  https://github.com/opencv/opencv_zoo/raw/main/models/object_tracking_vittrack/object_tracking_vittrack_2023sep.onnx
NANO=https://github.com/HonglinChu/SiamTrackers/raw/master/NanoTrack/models/nanotrackv2
fetch nanotrackv2_nanotrack_backbone_sim.onnx "$NANO/nanotrack_backbone_sim.onnx"
fetch nanotrackv2_nanotrack_head_sim.onnx "$NANO/nanotrack_head_sim.onnx"

if [ "${1:-}" = "--dasiamrpn" ] || [ "${DASIAMRPN:-0}" = "1" ]; then
  fetch dasiamrpn_model.onnx "https://www.dropbox.com/s/rr1lk9355vzolqv/dasiamrpn_model.onnx?dl=1"
  fetch dasiamrpn_kernel_r1.onnx "https://www.dropbox.com/s/999cqx5zrfi7w4p/dasiamrpn_kernel_r1.onnx?dl=1"
  fetch dasiamrpn_kernel_cls1.onnx "https://www.dropbox.com/s/qvmtszx5h339a0w/dasiamrpn_kernel_cls1.onnx?dl=1"
fi
