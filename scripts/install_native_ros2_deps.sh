#!/usr/bin/env bash
# Install ROS2 Humble and runtime dependencies for native (non-Docker) Ultra-Fusion.
# Tested on Ubuntu 22.04. Mirrors Dockerfile.ros2.
#
# Usage:
#   ./scripts/install_native_ros2_deps.sh

set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  SUDO="sudo"
else
  SUDO=""
fi

if ! grep -q '22.04' /etc/os-release 2>/dev/null; then
  echo "Warning: Ultra-Fusion ROS2 is tested on Ubuntu 22.04 + ROS2 Humble." >&2
fi

echo "==> Adding ROS2 Humble apt source ..."
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg2 lsb-release

if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | $SUDO gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi

if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
  echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

echo "==> Installing ROS2 Humble and system libraries ..."
$SUDO apt-get update
$SUDO apt-get install -y --no-install-recommends \
  build-essential cmake git wget python3-pip python3-lz4 \
  libatlas-base-dev libboost-thread-dev libeigen3-dev libfmt-dev \
  libgflags-dev libgoogle-glog-dev libopencv-dev libpcl-dev \
  libsuitesparse-dev libtbb-dev \
  ros-humble-cv-bridge ros-humble-geometry-msgs ros-humble-nav-msgs \
  ros-humble-pcl-conversions ros-humble-rclcpp ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins ros-humble-rviz2 \
  ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-tf2 \
  ros-humble-tf2-ros ros-humble-visualization-msgs

echo "==> Installing rosbags (ROS1→ROS2 bag conversion helper) ..."
python3 -m pip install --user rosbags

echo "==> Building Ceres Solver 2.1.0 ..."
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"
TMP_CERES="$(mktemp -d)"
git clone --branch 2.1.0 --depth 1 https://github.com/ceres-solver/ceres-solver.git "$TMP_CERES"
cmake -S "$TMP_CERES" -B "$TMP_CERES/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=OFF -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DMINIGLOG=OFF
cmake --build "$TMP_CERES/build" --target install -- -j"$BUILD_JOBS"
rm -rf "$TMP_CERES"

echo "==> Building yaml-cpp 0.8.0 ..."
TMP_YAML="$(mktemp -d)"
git clone --branch 0.8.0 --depth 1 https://github.com/jbeder/yaml-cpp.git "$TMP_YAML"
cmake -S "$TMP_YAML" -B "$TMP_YAML/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=ON \
  -DYAML_CPP_BUILD_CONTRIB=OFF -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_BUILD_TOOLS=OFF
cmake --build "$TMP_YAML/build" --target install -- -j"$BUILD_JOBS"
rm -rf "$TMP_YAML"

$SUDO ldconfig

echo ""
echo "Native ROS2 dependencies installed."
echo "Next step: install the Ultra-Fusion ROS2 release package:"
echo "  ./scripts/install_ultrafusion_ros2_deb.sh"
echo ""
echo "Then source ROS2 in every new shell:"
echo "  source /opt/ros/humble/setup.bash"
