# syntax=docker/dockerfile:1.7
# Project image (ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL + Micro-XRCE + OpenCV 4.9 + GCS).
#
# Stages:
#   base      ROS 2 base + Gazebo (ros_gz) + runtime and build libraries of the workspace
#   opencv    OpenCV with cv::TrackerVit (only the modules in use, bundled image libraries)
#   px4       PX4-Autopilot SITL at the tested commit; only what is needed to run is copied
#   xrce      Micro-XRCE-DDS-Agent
#   runtime   base + artifacts of the stages above (no PX4 toolchain, sources or objects)
#
# The repository code is NOT part of the image: it is mounted as a volume (docker/compose.yaml) and
# built inside the container in its own volumes (build/install/log), so editing the
# code only needs `colcon build`, never an image rebuild.

ARG ROS_DISTRO=jazzy

# ------------------------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8
# Mesa OpenCL (rusticl) on AMD/Intel GPUs: optional on old Mesa versions
ENV RUSTICL_ENABLE=radeonsi,iris
RUN apt-get update && apt-get install -y --no-install-recommends \
      # Gazebo Harmonic + bridges (Jazzy vendor)
      ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-rosbag2-storage-mcap ros-${ROS_DISTRO}-tf2-eigen \
      ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-eigen3-cmake-module \
      # Workspace build (mounted as a volume)
      build-essential cmake git python3-colcon-common-extensions libeigen3-dev libyaml-cpp-dev \
      # GStreamer (GCS video streamer): VA-API, x264/x265, JPEG
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
      # GPU (Mesa: OpenGL/EGL for OGRE2, VA-API) and OpenCL (ICD loader)
      libgl1-mesa-dri libegl1 libgl1 mesa-va-drivers vainfo ocl-icd-libopencl1 mesa-opencl-icd clinfo \
      # GCS e scripts
      python3-aiohttp python3-yaml python3-numpy python3-matplotlib openssl iproute2 procps curl ca-certificates sudo \
      # acados code generator (MPC solvers, generated at the workspace build)
      python3-venv \
 && if [ "$(dpkg --print-architecture)" = amd64 ]; then \
      apt-get install -y --no-install-recommends intel-media-va-driver; fi \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------------------------------
FROM base AS opencv
ARG OPENCV_VERSION=4.9.0
RUN git clone --depth 1 -b ${OPENCV_VERSION} https://github.com/opencv/opencv.git /src/opencv \
 && cmake -S /src/opencv -B /src/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DBUILD_LIST=core,imgproc,imgcodecs,video,dnn,calib3d,features2d,flann \
      -DWITH_OPENCL=ON -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_GTK=OFF -DWITH_QT=OFF \
      -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENEXR=OFF -DWITH_JASPER=OFF -DWITH_OPENJPEG=OFF \
      -DBUILD_JPEG=ON -DBUILD_PNG=ON -DBUILD_ZLIB=ON -DBUILD_PROTOBUF=ON \
      -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF \
      -DBUILD_JAVA=OFF -DBUILD_opencv_python3=OFF -DOPENCV_GENERATE_PKGCONFIG=ON \
 && cmake --build /src/build -j"$(nproc)" && cmake --install /src/build \
 && rm -rf /src

# ------------------------------------------------------------------------------------------
FROM base AS px4
ARG ROS_DISTRO
ARG PX4_REF=ff5b9484369b714763db9638517c08df0c242237
ARG PX4_TAG=v1.18.0-beta1
WORKDIR /opt/PX4-Autopilot
# Shallow clone of the tested commit; the local tag satisfies PX4's version `git describe`.
RUN git init -q . && git remote add origin https://github.com/PX4/PX4-Autopilot.git \
 && git fetch -q --depth 1 origin ${PX4_REF} && git checkout -q FETCH_HEAD && git tag ${PX4_TAG} \
 && git submodule update -q --init --recursive --depth 1 --jobs 8
# PIP_IGNORE_INSTALLED: Debian's numpy (python3-numpy) cannot be uninstalled by pip.
RUN apt-get update && PIP_IGNORE_INSTALLED=1 bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
 && rm -rf /var/lib/apt/lists/*
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && make px4_sitl -j$(nproc)"
# Only what SITL uses at runtime, at the same paths (gz_env.sh points to them).
RUN mkdir -p /out/opt/PX4-Autopilot/build/px4_sitl_default /out/opt/PX4-Autopilot/src/modules/simulation \
 && B=build/px4_sitl_default && O=/out/opt/PX4-Autopilot \
 && cp -a $B/bin $B/etc $O/$B/ \
 && mkdir -p $O/$B/rootfs && cp $B/rootfs/gz_env.sh $O/$B/rootfs/ \
 && mkdir -p $O/$B/src/modules/simulation/gz_plugins \
 && find $B/src/modules/simulation/gz_plugins -name '*.so*' -exec cp -a {} $O/$B/src/modules/simulation/gz_plugins/ \; \
 && mkdir -p $O/src/modules/simulation/gz_bridge && cp src/modules/simulation/gz_bridge/server.config $O/src/modules/simulation/gz_bridge/ \
 && mkdir -p $O/Tools/simulation && cp -a Tools/simulation/gz $O/Tools/simulation/ \
 && rm -rf $O/Tools/simulation/gz/.git

# ------------------------------------------------------------------------------------------
FROM base AS xrce
ARG XRCE_REF=v2.4.3
RUN git clone -q --depth 1 -b ${XRCE_REF} https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /src/xrce \
 && cmake -S /src/xrce -B /src/build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/xrce \
 && cmake --build /src/build -j"$(nproc)" && cmake --install /src/build && rm -rf /src

# ------------------------------------------------------------------------------------------
# acados: MPC solvers (SQP + HPIPM) and its CasADi code generator (ros2_ws/install_acados.sh)
FROM base AS acados
COPY ros2_ws/install_acados.sh /tmp/install_acados.sh
RUN bash /tmp/install_acados.sh /opt/acados && rm -rf /opt/acados/build /opt/acados/.git

# ------------------------------------------------------------------------------------------
FROM base AS runtime
ARG ROS_DISTRO
ARG USER_UID=1000
ARG USER_GID=1000
COPY --from=opencv /usr/local /usr/local
COPY --from=xrce /opt/xrce /usr/local
COPY --from=px4 /out/ /
COPY --from=acados /opt/acados /opt/acados
RUN ldconfig \
 # User with the host UID/GID: files created in the mounted repository stay yours.
 # UID/GID 0 (built as root) would clash with root: use 1000 instead.
 && uid=${USER_UID} && gid=${USER_GID} \
 && if [ "$uid" = 0 ]; then uid=1000; fi && if [ "$gid" = 0 ]; then gid=1000; fi \
 && (id -u ubuntu > /dev/null 2>&1 && userdel -r ubuntu 2> /dev/null || true) \
 && (getent group "$gid" > /dev/null || groupadd -g "$gid" aero) \
 && useradd -m -s /bin/bash -u "$uid" -g "$gid" -G video,sudo aero \
 && echo 'aero ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/aero \
 # PX4 logs/parameters (rootfs) and the Fuel model cache live in writable volumes.
 && chown -R "$uid:$gid" /opt/PX4-Autopilot/build/px4_sitl_default/rootfs \
 && mkdir -p /home/aero/.gz /home/aero/.config/aero_gcs /home/aero/.ros \
 && chown -R "$uid:$gid" /home/aero
COPY docker/entrypoint.sh /usr/local/bin/aero
ENV ROS_DISTRO=${ROS_DISTRO} PX4_DIR=/opt/PX4-Autopilot SKIP_PX4_BUILD=1 ACADOS_ROOT=/opt/acados \
    AERO_REPO=/home/aero/aero_pursuit GZ_HEADLESS=1 FOXGLOVE=0
USER aero
WORKDIR /home/aero/aero_pursuit
ENTRYPOINT ["/usr/local/bin/aero"]
CMD ["gcs"]
