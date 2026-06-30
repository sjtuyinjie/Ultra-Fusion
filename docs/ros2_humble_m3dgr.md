# Ultra-Fusion ROS2/Humble Guide

This page documents the **ROS2 Humble runtime** (v0.2.0). Ultra-Fusion on ROS2 uses the same `uf_node` + YAML workflow as ROS1 — point profiles at your ROS2 topics and play any matching bag or live stream.

**M3DGR** below is the documented walkthrough (including ROS1→ROS2 bag conversion). The same steps apply to other datasets once topics match a profile.

## Runtime Image

Build locally:

```bash
docker build -f Dockerfile.ros2 -t ultrafusion-ros2:0.2.0 .
```

Or use the helper:

```bash
./scripts/build_push_ros2_docker.sh --version 0.2.0
```

Published tags:

```bash
docker pull maotiandocker/ultrafusion-ros2:0.2.0
docker pull registry.cn-hangzhou.aliyuncs.com/bit_robot_image/ultrafusion-ros2:0.2.0
```

Published digests:

```text
Docker Hub: sha256:ccc3b91f6781f0b63deca2c252ee14e28d75438ab5e563a3cef472bd6c6b6223
Alibaba Cloud ACR: sha256:9f452ca0bfb76236c21c555556b9f6f5f23bd240d884a77b1826c70668c815ac
```

Start the container:

```bash
xhost +local:docker

docker run --rm -it --net=host --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /media:/media:ro \
  -v "$(pwd)":/workspace \
  maotiandocker/ultrafusion-ros2:0.2.0
```

Install the ROS2 Ultra-Fusion package in the container:

```bash
cd /workspace
./scripts/install_ultrafusion_ros2_deb.sh
source /opt/ros/humble/setup.bash
```

The generated v0.2.0 package is:

```text
ultrafusion-ros2_0.2.0_amd64.deb
SHA256: 0a589e6f739038e3e2fa83d6d444d0d84b638854114bc8f8d7f7ac4b2a1dd225
```

## M3DGR ROS2 Bag Conversion

The ROS2 release consumes standard common topics. M3DGR LiDAR packets are
converted from Livox `CustomMsg` to `sensor_msgs/msg/PointCloud2`; IMU, wheel,
and visual topics are copied/remapped.

Install the Python converter dependency if it is not already available:

```bash
python3 -m pip install --user rosbags
```

Convert a short smoke-test segment:

```bash
python3 scripts/convert_m3dgr_ros1_to_ros2_common.py \
  --src /media/path/to/M3DGR/Grass01.bag \
  --dst /tmp/grass01_20s_ros2 \
  --duration 20 \
  --overwrite

ros2 bag info /tmp/grass01_20s_ros2
```

Convert a full bag by omitting `--duration`:

```bash
python3 scripts/convert_m3dgr_ros1_to_ros2_common.py \
  --src /media/path/to/M3DGR/Grass01.bag \
  --dst /media/path/to/M3DGR/ros2/Grass01_ros2 \
  --overwrite
```

## Run M3DGR LVWIO on ROS2 (example)

Use one terminal for the estimator and another for replay:

```bash
source /opt/ros/humble/setup.bash
uf_node /opt/ultrafusion/config/m3dgr/uf_m3dgr_ros2_lvwio.yaml
```

```bash
source /opt/ros/humble/setup.bash
ros2 bag play /tmp/grass01_20s_ros2 --clock
```

Open RViz2 with the ROS2 layout:

```bash
rviz2 -d /opt/ultrafusion/rviz/lio_ros2.rviz
```

Check the live visualization topics:

```bash
ros2 topic list -t | grep -E 'curr_cloud|result_lidar_path|odom_lidar'
ros2 topic echo /curr_cloud --once --field width
ros2 topic echo /result_lidar_path --once --field header.frame_id
```

The ROS2 LVWIO profile expects:

| Sensor | Topic |
| --- | --- |
| LiDAR | `/livox/mid360/lidar` (`sensor_msgs/msg/PointCloud2`) |
| IMU | `/livox/mid360/imu` |
| Wheel | `/odom` |
| Visual | `/camera/color/image_raw/compressed` |
| Depth image, if enabled | `/camera/aligned_depth_to_color/image_raw` |

## Release Checklist

1. Build the ROS2 runtime image with `Dockerfile.ros2`.
2. Build the ROS2 `.deb` with
   `scripts/package_ros2_deb_from_build.sh --source /path/to/Ultra-Fusion`.
   Run this inside the target ROS2 Humble image for the final public release so
   OpenCV/PCL/Ceres ABI versions match the runtime image.
3. Upload the ROS2 `.deb` to GitHub Releases under tag `v0.2.0`.
4. Set `ULTRAFUSION_ROS2_SHA256` in release notes or pass `--sha256` when
   installing.
5. Push Docker Hub and ACR images with `scripts/build_push_ros2_docker.sh`.
6. Run the 20s M3DGR smoke test and at least one full M3DGR sequence before
   marking the release stable.
