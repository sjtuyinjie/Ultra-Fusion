# Adapting Ultra-Fusion to Your Own Device

Copy the closest released profile, edit ROS topics, camera calibration, and extrinsics, then run `uf_node` with your YAML. **Computer Vision Life D360** below is the reference case (LiDAR + IMU + three fisheye cameras); replace its topics and calibration with yours.

> v0.1.0 = paper package. Multi-camera and `visual_life` profile require v0.1.1.

<p align="center">
  <img src="../images/gifs/d360_visual_life.gif" alt="Multi-camera LVIO on D360 data" width="80%">
</p>

## Workflow

| Step | Action | D360 |
| --- | --- | --- |
| 1 | Copy a full profile directory | `/opt/ultrafusion/config/visual_life/` or `config/visual_life/` |
| 2 | Map ROS topics | `/livox/imu`, `/livox/lidar`, `/SLB_CAM_*/compressed` |
| 3 | Add camodocal intrinsics per camera | `calib.json` → `KANNALA_BRANDT` YAML |
| 4 | Set LiDAR–camera extrinsics | `T_lidar_to_cam` → `multi_camera.modules[].extrinsic_TCL/RCL` |
| 5 | Set fusion switches | LVIO: `use_lidar: 1`, `use_image: 1`, `wheel: 0` |
| 6 | Multi-camera (if needed) | `use_multi_camera: true`, one module per stream |
| 7 | Run and check RViz / logs | `uf_node visual_life` |

YAML field reference: [§3 Custom Profiles](../README.md#3-custom-profiles).

**Single-camera** (M3DGR and most profiles): omit `use_multi_camera` or set `false`; use `common.image0_topic` and `cam0_calib`. **Multi-camera**: `use_multi_camera: true` and `multi_camera.modules[]`.

## D360 Example

**Bag layout:**

```text
/media/big/tyh/rosbag/visual_life/
  calib.json
  2026-06-17_indoor_car1/slamibot_2026-06-17-10-44-18_0.bag
```

**Topics:**

```text
/livox/imu              sensor_msgs/Imu                 ~200 Hz
/livox/lidar            livox_ros_driver2/CustomMsg      ~10 Hz
/SLB_CAM_A/compressed   sensor_msgs/CompressedImage      ~10 Hz
/SLB_CAM_B/compressed   sensor_msgs/CompressedImage      ~10 Hz
/SLB_CAM_C/compressed   sensor_msgs/CompressedImage      ~10 Hz
```

**Config layout** (calibration files stay next to the main YAML):

```text
config/visual_life/
  config.yaml          # UF entry — not vins_multi_config.yaml
  cameraA.yaml
  cameraB.yaml
  cameraC.yaml
```

**RViz outputs:** `/result_path`, `/curr_cloud`, `/feature_reproject_cloud`, `/colored_lidar_cloud` (fixed frame: `world`).

## Extrinsics

| Transform | YAML | Notes |
| --- | --- | --- |
| `T_I_L` | `mapping.extrinsic_T/R` | LiDAR → IMU/body |
| `T_C_L` | `multi_camera.modules[i].extrinsic_TCL/RCL` | LiDAR → camera *i*; `p_c = R_C_L * p_l + t_C_L` |

Dataset `T_lidar_to_cam` maps directly to `T_C_L`. `/colored_lidar_cloud` is debug-only.

## Camera Intrinsics

Per-camera files referenced from the main config — see [§3.2](../README.md#32-camera-intrinsics). D360 fisheye from `calib.json`:

```text
intrinsics = [fx, fy, cx, cy]
distortion = [k2, k3, k4, k5]
```

→ camodocal `KANNALA_BRANDT` (do not use `PINHOLE` for fisheye):

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

## Key Fields

```yaml
imu: 1
wheel: 0
use_lidar: 1
use_image: 1
depth: 0
use_multi_camera: true
use_lidar_reproject: true
```

```yaml
common:
  imu_topic: /livox/imu
  lid_topic: /livox/lidar
  image0_topic: "/SLB_CAM_A/compressed"
  image1_topic: "/SLB_CAM_B/compressed"
  img0_type: 1   # 0 = sensor_msgs/Image, 1 = CompressedImage
  img1_type: 1
```

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
    # camera_id 1, 2 → cameraB/C.yaml, /SLB_CAM_B/C/compressed
```

`sync_tolerance` is the max inter-camera timestamp spread (seconds). Adjusted stamp: `image_header_stamp + img_time_offset + module.time_offset`.

LiDAR-assisted depth (optional):

```yaml
use_lidar_reproject: true
depth_threshold: 10
multi_camera:
  lidar_reproject_apply_to_backend: true   # false = debug clouds only
```

## Run

```bash
sudo dpkg -i ultrafusion_0.1.1_amd64.deb || sudo apt-get install -f -y
source /opt/ros/noetic/setup.bash

roscore
rosbag play /path/to/your.bag --clock
uf_node visual_life
# uf_node /path/to/config/visual_life/config.yaml
```

Expected startup log:

```text
use_multi_camera: 1 modules=3 sync_tolerance=0.002 lidar_reproject_apply_to_backend=1
Colored LiDAR cloud worker started
```

## Troubleshooting

- **`YAML::TypedBadConversion<int>`** — use `config.yaml`; check YAML types and indentation.
- **No `/feature_reproject_cloud`** — check `use_lidar_reproject`, camera sync (`max_dt < sync_tolerance`), extrinsics, `depth_threshold`.
- **No `/colored_lidar_cloud`** — confirm worker started in logs; image topics publishing.
- **No color in cloud** — RViz Color Transformer → `RGB8`.
- **Drift / misalignment** — see [§3.4–3.5](../README.md#34-extrinsics); check `Opti_TIC`, `td`, LiDAR-IMU sync in startup logs.
