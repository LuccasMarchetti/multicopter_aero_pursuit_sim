#!/bin/bash
# Compila o OpenCV local (/usr/local) usado pelo aero_pursuit (cv::TrackerVit, >= 4.9).
# A inferência roda no motor nativo do OpenCV DNN (importador ONNX embutido, backend
# OpenCV com alvo CPU/OpenCL); ONNX Runtime/CUDA não são necessários.
#   OPENCV_VERSION=4.9.0  WITH_CONTRIB=1 (módulos extras; não usados pelo aero_pursuit)
set -e

OPENCV_VERSION=${OPENCV_VERSION:-4.9.0}
WITH_CONTRIB=${WITH_CONTRIB:-0}
SRC_DIR=${SRC_DIR:-$HOME}

# 1. Dependências do sistema
sudo apt update && sudo apt install -y build-essential cmake git pkg-config \
  libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev \
  libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran \
  ocl-icd-opencl-dev

# 2. Fontes
cd "$SRC_DIR"
rm -rf opencv opencv_contrib
git clone --depth 1 -b "$OPENCV_VERSION" https://github.com/opencv/opencv.git
EXTRA_ARGS=()
if [ "$WITH_CONTRIB" = "1" ]; then
  git clone --depth 1 -b "$OPENCV_VERSION" https://github.com/opencv/opencv_contrib.git
  EXTRA_ARGS+=(-D OPENCV_EXTRA_MODULES_PATH="$SRC_DIR/opencv_contrib/modules")
fi

# 3. Configuração (TrackerVit: módulos video + dnn; OpenCL para o backend GPU)
mkdir -p "$SRC_DIR/opencv/build" && cd "$SRC_DIR/opencv/build"
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_OPENCL=ON \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_EXAMPLES=OFF \
      "${EXTRA_ARGS[@]}" ..

# 4. Compilação e instalação
make -j"$(nproc)"
sudo make install
sudo ldconfig

echo "OpenCV $OPENCV_VERSION installed in /usr/local."
