# Adapting Ultra-Fusion to Your Own Device

Ultra-Fusion ships with ready-made profiles for public benchmarks (M3DGR, KAIST, and others). To run on **your own hardware**, copy the closest profile, wire up ROS topics, camera calibration, and extrinsics, then launch `uf_node` with your YAML.

This guide walks through that workflow end to end. **Computer Vision Life D360** is used as a worked example: a LiDAR–visual–IMU rig with three fisheye cameras and compressed image streams. The same steps apply to any platform—swap in your topics, calibration files, and transforms.

> **Release note:** v0.1.0 is the paper package. Multi-camera support and the reference `visual_life` profile ship in **v0.1.1** as a separate `.deb`, leaving v0.1.0 unchanged.

> Demo video: add the recording link here after it is published.

<p align="center">
  <img src="../images/gifs/d360_visual_life.gif" alt="Multi-camera LVIO on a custom sensor rig (D360 example)" width="80%">
</p>
<p align="center"><em>Worked example: three-camera LVIO on Computer Vision Life D360 data.</em></p>

---

## Adaptation Workflow

| Step | What to do | D360 example |
| --- | --- | --- |
| 1 | Pick the closest released profile and **copy the whole config directory** | Start from `/opt/ultrafusion/config/visual_life/` (v0.1.1) or `config/visual_life/` in this repo |
| 2 | Map **ROS topics** to your bag or live drivers | `/livox/imu`, `/livox/lidar`, three `/SLB_CAM_*/compressed` topics |
| 3 | Prepare **per-camera intrinsics** as camodocal YAML | Convert D360 `calib.json` fisheye params → `KANNALA_BRANDT` |
| 4 | Fill **LiDAR–camera extrinsics** for each camera | Copy dataset `T_lidar_to_cam` into `multi_camera.modules[].extrinsic_TCL/RCL` |
| 5 | Set **fusion-mode switches** (`use_lidar`, `use_image`, `wheel`, …) | LVIO: `use_lidar: 1`, `use_image: 1`, `wheel: 0` |
| 6 | Enable **multi-camera** when you have more than one visual stream | `use_multi_camera: true` + one module per camera |
| 7 | Run `uf_node`, play data, and verify RViz / startup logs | `uf_node visual_life` or `uf_node /path/to/config.yaml` |

For field-level customization (fusion modes, GNSS, delays, online calibration), see [§3 Custom Profiles](../README.md#3-custom-profiles) in the README.

---

## Single-Camera vs Multi-Camera

| Path | When to use | Key YAML field |
| --- | --- | --- |
| **Single-camera** (default) | One RGB or RGB-D stream; matches M3DGR and most released profiles | `use_multi_camera: false` (or omit) — configure `common.image0_topic`, `cam0_calib` |
| **Multi-camera** | Two or more independent camera streams | `use_multi_camera: true` — configure `multi_camera.modules[]` |

Existing benchmark profiles without `use_multi_camera` keep the original single-camera path. Multi-camera is an **optional** extension; enabling it does not change behavior for other configs.

---

## Worked Example: Computer Vision Life D360

The sections below use D360 to illustrate each adaptation step. Replace topics, calibration, and extrinsics with your own sensor data.

### Dataset layout and target mode

Validated local layout:

```text
/media/big/tyh/rosbag/visual_life/
  calib.json
  2026-06-17_indoor_car1/
    slamibot_2026-06-17-10-44-18_0.bag
```

Topics in the reference profile:

```text
/livox/imu              sensor_msgs/Imu                 ~200 Hz
/livox/lidar            livox_ros_driver2/CustomMsg      ~10 Hz
/SLB_CAM_A/compressed   sensor_msgs/CompressedImage      ~10 Hz
/SLB_CAM_B/compressed   sensor_msgs/CompressedImage      ~10 Hz
/SLB_CAM_C/compressed   sensor_msgs/CompressedImage      ~10 Hz
/rtk/raw                std_msgs/String                  not used in this LVIO profile
```

**Target fusion mode:** LiDAR + visual + IMU (`lvio`), three cameras, LiDAR-assisted visual depth.

Expected debug / visualization topics:

```text
/result_path
/curr_cloud
/feature_reproject_cloud
/colored_lidar_cloud
```

### Config directory layout

Keep calibration files next to the main YAML (same rule as all UF profiles):

```text
config/visual_life/
  config.yaml              # UF entry config — always start here
  cameraA.yaml
  cameraB.yaml
  cameraC.yaml
  vins_multi_config.yaml   # optional upstream VINS-Multi reference only; not a UF entry
```

---

## Frame and Extrinsic Conventions

Use these conventions when filling any custom profile:

| Quantity | UF field | Meaning |
| --- | --- | --- |
| LiDAR → IMU/body | `mapping.extrinsic_T/R` | `T_I_L` |
| LiDAR → camera *i* | `multi_camera.modules[i].extrinsic_TCL/RCL` | `T_C_L`: `p_c = R_C_L * p_l + t_C_L` |
| SLAM world frame | RViz fixed frame `world` | Local frame, not absolute GNSS/ENU |

If your calibration tool exports `T_lidar_to_cam`, map it directly to `T_C_L` (`extrinsic_TCL/RCL`). `/colored_lidar_cloud` is debug-only and is not used in optimization.

---

## Camera Intrinsics

UF loads intrinsics from separate camodocal/OpenCV YAML files referenced by the main config—not inline in `config.yaml`. See [§3.2 Camera intrinsics](../README.md#32-camera-intrinsics) for the general format.

### D360 example: fisheye (`calib.json` → `KANNALA_BRANDT`)

D360 stores fisheye parameters as:

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

Do not treat fisheye cameras as pinhole (`PINHOLE`)—projection error will break visual tracking and LiDAR reprojection checks.

For pinhole or RGB-D rigs, use `PINHOLE` and the field layout shown in the README §3.2.

---

## Key `config.yaml` Fields

### Fusion and sensor switches

```yaml
imu: 1
wheel: 0
use_lidar: 1
use_image: 1
depth: 0
use_multi_camera: true
use_lidar_reproject: true
```

Adjust `wheel`, `depth`, and `use_lidar`/`use_image` for your target mode (see README §3.1).

### Topics and image encoding

```yaml
common:
  imu_topic: /livox/imu
  lid_topic: /livox/lidar
  image0_topic: "/SLB_CAM_A/compressed"
  image1_topic: "/SLB_CAM_B/compressed"
  img0_type: 1
  img1_type: 1
```

| `image_type` | ROS message |
| --- | --- |
| `0` | `sensor_msgs/Image` (raw) |
| `1` | `sensor_msgs/CompressedImage` |

Point every field at **your** driver topics. The D360 names above are illustrative.

### Multi-camera modules

One entry per camera:

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

D360 reference mapping:

| `camera_id` | Calibration file | Topic |
| --- | --- | --- |
| `0` | `cameraA.yaml` | `/SLB_CAM_A/compressed` |
| `1` | `cameraB.yaml` | `/SLB_CAM_B/compressed` |
| `2` | `cameraC.yaml` | `/SLB_CAM_C/compressed` |

**Synchronization:** `sync_tolerance` (seconds) is the max timestamp spread when grouping cameras. Per-camera adjusted stamp:

```text
adjusted_stamp = image_header_stamp + img_time_offset + module.time_offset
```

Tune `sync_tolerance` and per-module `time_offset` if your drivers stamp images inconsistently.

---

## LiDAR-Assisted Visual Depth

Optional but useful for checking extrinsics and enriching visual features:

```yaml
use_lidar_reproject: true
depth_threshold: 10
multi_camera:
  lidar_reproject_apply_to_backend: true
```

| Field | Effect |
| --- | --- |
| `use_lidar_reproject` | Project LiDAR points into images to estimate feature depth |
| `depth_threshold` | Accept feature depths within this range (meters) |
| `lidar_reproject_apply_to_backend` | `true`: depth feeds the visual backend; `false`: debug clouds only |

---

## Run and Verify

### Install v0.1.1 (multi-camera profile)

```bash
sudo dpkg -i ultrafusion_0.1.1_amd64.deb || sudo apt-get install -f -y
source /opt/ros/noetic/setup.bash
```

Or build from source:

```bash
cd /path/to/Ultra-Fusion
cmake --build build -j4
```

### Typical three-terminal workflow

```bash
# Terminal 1
roscore

# Terminal 2 — your bag or live drivers
rosbag play /path/to/your.bag --clock

# Terminal 3 — installed shortcut or custom config path
uf_node visual_life
# uf_node /path/to/config/visual_life/config.yaml
```

From a build tree:

```bash
env GLOG_logtostderr=1 ./build/devel/lib/ultrafusion/uf_node config/visual_life/config.yaml
```

### Startup checks

Look for lines like:

```text
use_multi_camera: 1 modules=3 sync_tolerance=0.002 lidar_reproject_apply_to_backend=1
Colored LiDAR camera context loaded camera_id=0 calib=...
Colored LiDAR cloud worker started
```

Module count should match your `multi_camera.modules` list.

### RViz

Fixed frame: `world`.

| Topic | Type | Notes |
| --- | --- | --- |
| `/result_path` | Path | Estimated trajectory |
| `/curr_cloud` | PointCloud2 | Intensity |
| `/feature_reproject_cloud` | PointCloud2 | Accepted LiDAR-depth features; intensity = `camera_id + 1` |
| `/colored_lidar_cloud` | PointCloud2 | RGB8 — set Color Transformer to **RGB8** |

Quick topic checks:

```bash
rostopic hz /curr_cloud
rostopic hz /feature_reproject_cloud
rostopic hz /colored_lidar_cloud
```

---

## Troubleshooting

| Symptom | Likely cause | What to check |
| --- | --- | --- |
| `YAML::TypedBadConversion<int>` | Wrong entry config or bad YAML typing | Use `config.yaml`, not `vins_multi_config.yaml`; verify booleans and indentation |
| No `/feature_reproject_cloud` | Reprojection off, sync failure, or bad extrinsics | `use_lidar_reproject: true`; three-camera `max_dt < sync_tolerance`; extrinsic direction and `depth_threshold` |
| No `/colored_lidar_cloud` | Worker not started or missing inputs | Startup log for `Colored LiDAR cloud worker started`; image topics publishing; valid pose |
| Colored cloud, no color | RViz setting | Color Transformer → `RGB8` |
| Drift or misaligned map | Frame / time error | Compare README §3.4 extrinsics and §3.5 delays; inspect startup logs for `Opti_TIC`, `td`, LiDAR-IMU sync |

For three-camera timing, echo compressed-image header stamps and compare merged-frame `max_dt` in UF logs against `multi_camera.sync_tolerance`.

---

## Summary

| Item | Takeaway |
| --- | --- |
| **General rule** | Copy a full profile directory; edit topics, intrinsics, extrinsics, and mode switches—do not build a minimal YAML from scratch |
| **Multi-camera** | Set `use_multi_camera: true` and one `multi_camera.modules[]` entry per stream |
| **D360 example** | Reference implementation under `config/visual_life/`; shortcut `uf_node visual_life` after v0.1.1 install |
| **Benchmark profiles** | Unchanged when `use_multi_camera` is absent or `false` |
| **Debug outputs** | `/colored_lidar_cloud` and optional reproject-only mode are for inspection, not core SLAM measurements |
