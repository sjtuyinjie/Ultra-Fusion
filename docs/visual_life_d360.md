# Computer Vision Life D360 Sensor Adaptation

This guide documents how to adapt the Computer Vision Life D360 dataset to
Ultra-Fusion's UF LVIO pipeline. The important addition is optional
multi-camera support in UF: one config can subscribe to three compressed camera
streams, load per-camera fisheye calibration and LiDAR-camera extrinsics, and
publish LiDAR-assisted visual debug clouds. The workflow targets the CMake/ROS1
runtime and does not require a launch file.

Version note: v0.1.0 is the paper release package. The D360 adaptation is
packaged separately as v0.1.1 so the paper release remains unchanged.

> Demo video: add the recording link here after it is published.

<p align="center">
  <img src="../images/gifs/d360_visual_life.gif" alt="Computer Vision Life D360 multi-camera UF LVIO demo" width="80%">
</p>

## Dataset And Goal

Validated local dataset layout:

```text
/media/big/tyh/rosbag/visual_life/
  calib.json
  2026-06-17_indoor_car1/
    slamibot_2026-06-17-10-44-18_0.bag
```

Topics used by this UF LVIO profile:

```text
/livox/imu              sensor_msgs/Imu                 about 200 Hz
/livox/lidar            livox_ros_driver2/CustomMsg      about 10 Hz
/SLB_CAM_A/compressed   sensor_msgs/CompressedImage      about 10 Hz
/SLB_CAM_B/compressed   sensor_msgs/CompressedImage      about 10 Hz
/SLB_CAM_C/compressed   sensor_msgs/CompressedImage      about 10 Hz
/rtk/raw                std_msgs/String                  not used by this LVIO profile
```

The adaptation target is LiDAR + visual + IMU (`lvio`) with three D360 cameras,
LiDAR-assisted visual feature depth, and RViz debug outputs:

```text
/result_path
/curr_cloud
/feature_reproject_cloud
/colored_lidar_cloud
```

## Adaptation Checklist

1. Put the UF config and three camera calibration files under
   `config/visual_life/`.
2. Convert the D360 fisheye camera intrinsics from `calib.json` to camodocal
   `KANNALA_BRANDT` YAML files.
3. Copy each dataset `T_lidar_to_cam` into the corresponding
   `multi_camera.modules[].extrinsic_TCL/RCL` field.
4. Point each camera module to the right compressed image topic and set
   `image_type: 1`.
5. Enable `use_multi_camera: true` and keep `wheel: 0`, `depth: 0` for the
   D360 LVIO profile.
6. Run `uf_node visual_life` from the installed package, or run
   `uf_node config/visual_life/config.yaml` from a source/build workspace, play
   the bag, and verify the RViz outputs listed below.

## Config Layout

Use `config/visual_life/config.yaml` as the UF entry config:

```text
config/visual_life/
  config.yaml
  cameraA.yaml
  cameraB.yaml
  cameraC.yaml
  vins_multi_config.yaml        # optional upstream VINS-Multi reference only
```

`vins_multi_config.yaml` is not the UF runtime entry. UF should be started with
`config.yaml`.

## Frame And Extrinsic Conventions

Use these conventions when filling `config.yaml`:

- LiDAR extrinsic: `T_I_L` from `mapping.extrinsic_T/R`.
- Dataset camera extrinsic: `T_lidar_to_cam`, represented in UF as `T_C_L`.
- `extrinsic_TCL/RCL` should therefore transform LiDAR points into the camera
  frame: `p_c = R_C_L * p_l + t_C_L`.
- `world` is the local SLAM world frame, not an absolute GNSS/ENU frame.
- `/colored_lidar_cloud` is only a debug/RViz output and is not optimized.

## Camera Calibration Conversion

The Computer Vision Life D360 `calib.json` uses fisheye cameras:

```text
intrinsics = [fx, fy, cx, cy]
distortion = [k2, k3, k4, k5]
model = fisheye
```

Convert each camera to camodocal `KANNALA_BRANDT`:

```yaml
model_type: KANNALA_BRANDT
image_width: 1920
image_height: 1200
projection_parameters:
  mu: fx
  mv: fy
  u0: cx
  v0: cy
  k2: distortion[0]
  k3: distortion[1]
  k4: distortion[2]
  k5: distortion[3]
```

Do not treat the D360 fisheye cameras as pinhole cameras. The UF config should
use the camodocal fisheye model for each D360 camera.

## Key config.yaml Fields

Sensor switches:

```yaml
imu: 1
wheel: 0
use_lidar: 1
use_image: 1
depth: 0
use_multi_camera: true
use_lidar_reproject: true
```

Topics and compressed image input:

```yaml
common:
  imu_topic: /livox/imu
  lid_topic: /livox/lidar
  image0_topic: "/SLB_CAM_A/compressed"
  image1_topic: "/SLB_CAM_B/compressed"
  img0_type: 1
  img1_type: 1
```

`image_type: 1` means `sensor_msgs/CompressedImage`. Use `image_type: 0` only
when images are raw `sensor_msgs/Image`.

Multi-camera modules:

```yaml
multi_camera:
  sync_tolerance: 0.002
  lidar_reproject_apply_to_backend: true
  modules:
    - camera_id: 0
      image_topic: "/SLB_CAM_A/compressed"
      image_type: 1
      time_offset: 0.0
      cam_calib: "cameraA.yaml"
      extrinsic_TCL: [...]
      extrinsic_RCL: [...]
    - camera_id: 1
      image_topic: "/SLB_CAM_B/compressed"
      image_type: 1
      time_offset: 0.0
      cam_calib: "cameraB.yaml"
      extrinsic_TCL: [...]
      extrinsic_RCL: [...]
    - camera_id: 2
      image_topic: "/SLB_CAM_C/compressed"
      image_type: 1
      time_offset: 0.0
      cam_calib: "cameraC.yaml"
      extrinsic_TCL: [...]
      extrinsic_RCL: [...]
```

Camera id convention:

| camera_id | Camera | Topic |
| --- | --- | --- |
| `0` | cameraA | `/SLB_CAM_A/compressed` |
| `1` | cameraB | `/SLB_CAM_B/compressed` |
| `2` | cameraC | `/SLB_CAM_C/compressed` |

`sync_tolerance: 0.002` allows up to 2 ms timestamp spread when grouping the
three corrected camera timestamps. Per-camera timing is:

```text
adjusted_stamp = image_header_stamp + img_time_offset + module.time_offset
```

## LiDAR Depth And Debug Options

Use these fields to control LiDAR-assisted visual depth:

```yaml
use_lidar_reproject: true
depth_threshold: 10
multi_camera:
  lidar_reproject_apply_to_backend: true
```

- `use_lidar_reproject: true` enables LiDAR-image depth projection.
- `depth_threshold` sets the accepted visual feature depth range.
- `lidar_reproject_apply_to_backend: true` writes accepted LiDAR depth back to
  visual features for backend use.
- Set `lidar_reproject_apply_to_backend: false` when you only want debug clouds
  and do not want LiDAR depth to affect the visual backend.

The colored LiDAR cloud is also debug-only. It is useful for checking
LiDAR-camera extrinsics and fisheye projection but is not used as an
optimization measurement.

## Run

Install the v0.1.1 D360 release package:

```bash
sudo dpkg -i ultrafusion_0.1.1_amd64.deb || sudo apt-get install -f -y
source /opt/ros/noetic/setup.bash
```

Build from the CMake workspace:

```bash
cd /path/to/Ultra-Fusion
cmake --build build -j4
```

Start ROS master:

```bash
export ROS_MASTER_URI=http://localhost:11311
roscore
```

Run UF:

```bash
cd /path/to/Ultra-Fusion
env ROS_MASTER_URI=http://localhost:11311 GLOG_logtostderr=1 \
  ./build/devel/lib/ultrafusion/uf_node config/visual_life/config.yaml
```

Installed runtime:

```bash
uf_node visual_life
```

This shortcut is available after installing the v0.1.1 D360 release package.

Custom config copy:

```bash
uf_node /path/to/config/visual_life/config.yaml
```

Play the bag:

```bash
env ROS_MASTER_URI=http://localhost:11311 rosbag play \
  /media/big/tyh/rosbag/visual_life/2026-06-17_indoor_car1/slamibot_2026-06-17-10-44-18_0.bag \
  --clock
```

Slower RViz inspection:

```bash
env ROS_MASTER_URI=http://localhost:11311 rosbag play \
  /media/big/tyh/rosbag/visual_life/2026-06-17_indoor_car1/slamibot_2026-06-17-10-44-18_0.bag \
  --clock -r 0.5
```

Expected startup logs include:

```text
use_multi_camera: 1 modules=3 sync_tolerance=0.002 lidar_reproject_apply_to_backend=1
Colored LiDAR camera context loaded camera_id=0 calib=...
Colored LiDAR camera context loaded camera_id=1 calib=...
Colored LiDAR camera context loaded camera_id=2 calib=...
Colored LiDAR cloud worker started
```

## RViz

Use `world` as the fixed frame.

| Topic | RViz type | Color |
| --- | --- | --- |
| `/result_path` | Path | default |
| `/curr_cloud` | PointCloud2 | Intensity |
| `/feature_reproject_cloud` | PointCloud2 | Intensity |
| `/colored_lidar_cloud` | PointCloud2 | RGB8 |

Checks:

```bash
env ROS_MASTER_URI=http://localhost:11311 rostopic hz /curr_cloud
env ROS_MASTER_URI=http://localhost:11311 rostopic hz /feature_reproject_cloud
env ROS_MASTER_URI=http://localhost:11311 rostopic hz /colored_lidar_cloud
env ROS_MASTER_URI=http://localhost:11311 rostopic echo -n1 /colored_lidar_cloud/header
```

`/feature_reproject_cloud` contains only visual feature 3D points whose LiDAR
reprojected depth was accepted. Point `intensity = camera_id + 1`.

`/colored_lidar_cloud` is generated in an independent latest-only worker. Slow
coloring drops old frames instead of blocking the SLAM main thread.

## Troubleshooting

`YAML::TypedBadConversion<int>`:

- Use `config/visual_life/config.yaml` as the UF entry config.
- Do not run `vins_multi_config.yaml` as the UF config.
- Check YAML integer fields, booleans, and indentation.

No `/feature_reproject_cloud`:

- Check `use_lidar_reproject: true`.
- Check three-camera synchronization.
- Check logs for `Multi-camera LiDAR feature depth updates total=...`.
- If `total` is 0, inspect extrinsic direction, visibility, and
  `depth_threshold`.

No `/colored_lidar_cloud`:

- Check startup logs for `Colored LiDAR cloud worker started`.
- Check `/SLB_CAM_A/compressed`, `/SLB_CAM_B/compressed`, and
  `/SLB_CAM_C/compressed`.
- This topic requires LiDAR, current pose, and at least one visible image.

Colored cloud has no color:

- Set RViz Color Transformer to `RGB8`.

Three-camera time alignment:

- Echo the three compressed image header stamps.
- Check merged-frame `max_dt` in UF logs.
- `max_dt` should be smaller than `multi_camera.sync_tolerance`.

## Change Boundary

The Computer Vision Life D360 adaptation is an optional multi-camera path:

- `use_multi_camera: true` enables multi-camera subscribers, feature merging,
  per-camera backend extrinsics, and multi-camera LiDAR reprojected depth.
- Existing M3DGR/M2DGR/LVIG profiles without `use_multi_camera`, or with it set
  to `false`, keep the original single-camera path.
- `/colored_lidar_cloud` is debug visualization only.
- `/rtk/raw` is not connected to the UF backend in this profile.
- No launch file is required.
