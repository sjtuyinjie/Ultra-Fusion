# Ultra-Fusion: A Resilient Tightly-Coupled Multi-Sensor Fusion SLAM Framework under Sensor Degradation and Spatiotemporal Perturbation for Intelligent Transportation Systems

<div align="center">

[![Website](https://img.shields.io/badge/Website-Ultra--Fusion-blue)](https://sjtuyinjie.github.io/ultrafusion-web/)
[![Hugging Face](https://img.shields.io/badge/Hugging%20Face-pre--built-ffcc4d)](https://huggingface.co/TianYihong/Ultra-Fusion)
[![arXiv](https://img.shields.io/badge/arXiv-2606.21223-b31b1b)](https://arxiv.org/abs/2606.21223)
[![Dataset](https://img.shields.io/badge/Dataset-M3DGR-orange)](https://github.com/sjtuyinjie/M3DGR)
[![Video](https://img.shields.io/badge/Video-Youtube-red)](https://www.youtube.com/watch?v=ekzD9ovd1SQ)
[![License](https://img.shields.io/badge/License-MIT-yellow)](#5-license--acknowledgements)
[![Platform](https://img.shields.io/badge/Platforms-Ground%20%7C%20Legged%20%7C%20Aerial-green)](#42-cross-platform-results)


</div>

**Core contributors:** [Yihong Tian](https://github.com/maotian123), [Junjie Zhang](https://github.com/Zjj587), [Liuyang Li](https://github.com/Lurvelly), and [Jie Yin](https://sjtuyinjie.github.io/)*

---

Ultra-Fusion is a tightly-coupled multi-sensor SLAM framework for intelligent transportation systems (ITS). It targets real-world deployment where **sensor degradation** (poor illumination, LiDAR degeneracy, wheel slippage, GNSS outage) and **spatiotemporal miscalibration** are common.

Within one configurable optimization framework, Ultra-Fusion supports **WIO, VIO, LIO, and LVIO**, with optional wheel/GNSS fusion and online calibration.

**Adaptation guide:** [Ultra-Fusion Computer Vision Life D360 Sensor Adaptation](docs/visual_life_d360.md)


## Contents

- [Overview](#overview)
  - [Highlights](#highlights)
  - [Method Overview](#method-overview)
  - [Why Ultra-Fusion](#why-ultra-fusion)
  - [Benchmarks and Findings](#benchmarks-and-findings)
- [1. Prerequisites & Installation](#1-prerequisites--installation)
  - [Option A — Docker (recommended)](#option-a--docker-install-recommended-)
  - [Option B — Native install](#option-b--native-install)
  - [Installed files](#installed-files)
- [2. Run on Five Benchmarks](#2-run-on-five-benchmarks)
  - [2.1 M3DGR](#21-m3dgr)
  - [2.2 Other datasets](#22-other-datasets)
  - [2.3 Computer Vision Life D360 Sensor Adaptation](#visual-life-d360)
- [3. Custom Profiles](#3-custom-profiles)
  - [3.1 Fusion modes](#31-fusion-modes)
  - [3.2 Camera intrinsics](#32-camera-intrinsics)
  - [3.3 GNSS fusion](#33-gnss-fusion)
  - [3.4 Extrinsics](#34-extrinsics)
  - [3.5 Calibration & delays](#35-calibration--delays)
- [4. Qualitative Results](#4-qualitative-results)
- [5. License & Acknowledgements](#5-license--acknowledgements)
- [6. Star History](#6-star-history)

---
## Overview

### Highlights

- Unified sliding-window estimator with timestamp-ordered heterogeneous factors.
- Observability-aware initialization for robust bootstrap under diverse motion/sensor conditions.
- Factor-wise reliability scheduling (FRS) to gate/down-weight degraded measurements.
- Online LiDAR-IMU spatiotemporal calibration during operation.
- Validated on wheeled, legged, and aerial platforms across multiple public benchmarks.

**See the [project website](https://sjtuyinjie.github.io/ultrafusion-web/) for videos, benchmark tables, and interactive demos!**

### Method Overview

Ultra-Fusion timestamp-orders asynchronous sensor streams and converts them into optional factors inside one sliding-window optimizer, sharing state representation, marginalization, and calibration logic.

<p align="center">
  <img src="images/pipeline.png" alt="Ultra-Fusion pipeline" width="92%">
</p>
<p align="center"><em>Unified pipeline: initialization, reliability scheduling, online calibration, and multi-modal fusion in one framework.</em></p>

---

### Why Ultra-Fusion

Compared with pipelines tied to a fixed sensor set, Ultra-Fusion focuses on:

1. **Configurability** — one framework for WIO / VIO / LIO / LVIO (+ wheel / GNSS).
2. **Reliability** — robust localization under corner-case sensor degradation.
3. **Deployability** — long-duration and high-speed ITS operation.
4. **Transferability** — validated on wheeled, legged, and aerial platforms.

---



### Benchmarks and Findings

Ultra-Fusion is evaluated on five public benchmarks:

- [**M3DGR**](https://github.com/sjtuyinjie/M3DGR) — wheeled, real + sim, sensor-degradation stress tests
- [**M2DGR-Plus**](https://github.com/SJTU-ViSYS/M2DGR-plus) — wheeled, multi-scenario ground SLAM
- [**KAIST Complex Urban**](https://sites.google.com/view/complex-urban-dataset) — autonomous driving, large-scale urban
- [**GrandTour**](https://github.com/leggedrobotics/grand_tour_dataset) — legged robots in the wild
- [**MARS-LVIG**](https://mars.hku.hk/dataset.html) — aerial LVIG fusion

Reported gains include competitive accuracy and improved localization availability under sensor degradation, calibration perturbation, and long-duration / high-speed operation.

> [!NOTE]
> This repository currently releases **executable binaries and demos**. Full source code will be released after paper acceptance. For related implementations, see [Ground-Fusion](https://github.com/SJTU-ViSYS/Ground-Fusion) and [Ground-Fusion++](https://github.com/sjtuyinjie/Ground-Fusion2).


---

## 1. Prerequisites & Installation

**Tested platform:** Ubuntu 20.04 + ROS Noetic.

The paper release is the prebuilt `.deb` v0.1.0. The D360 multi-camera adaptation is provided as the separate v0.1.1 release package, which keeps v0.1.0 unchanged and adds the `visual_life` profile under `/opt/ultrafusion/config/`.

> [!TIP]
> **Recommended: Option A (Docker).** It provides a clean, reproducible runtime and avoids dependency conflicts on your host. Choose Option B (native) only if you need a persistent host installation.

| Path | When to use | Summary |
| --- | --- | --- |
| **[Option A — Docker](#option-a--docker-install-recommended-)** ⭐ | First-time users, quick demos | Pull runtime image → start container → install `.deb` |
| **[Option B — Native](#option-b--native-install)** | Long-term host deployment | Install ROS + deps on host → install `.deb` |

---

### Option A — Docker install (recommended) ⭐

The image ships **only the ROS/runtime stack** (Noetic, RViz, Ceres, yaml-cpp, system libs).
You still need to install the Ultra-Fusion `.deb` inside the container.

**Step 1 — Clone this repository** (mounts into the container as `/workspace`)

```bash
git clone https://github.com/sjtuyinjie/Ultra-Fusion.git
cd Ultra-Fusion
```

**Step 2 — Pull the runtime image**

```bash
# Alibaba Cloud ACR (recommended in China):
docker pull registry.cn-hangzhou.aliyuncs.com/bit_robot_image/ultrafusion:0.1.0

# Docker Hub:
docker pull maotiandocker/ultrafusion:0.1.0

# Or build locally:
docker build -t ultrafusion:0.1.0 .
```

**Step 3 — Start a container** (with RViz GUI support)

Add `-v /media:/media:ro` if your rosbags are stored under `/media` on the host.

```bash
xhost +local:docker

docker run --rm -it --net=host --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /media:/media:ro \
  -v "$(pwd)":/workspace \
  registry.cn-hangzhou.aliyuncs.com/bit_robot_image/ultrafusion:0.1.0
```

**Step 4 — Install Ultra-Fusion inside the container**

```bash
cd /workspace
./scripts/install_ultrafusion_deb.sh   # add --mirror if GitHub is slow
source /opt/ros/noetic/setup.bash
```

**Step 5 — Verify**

```bash
which uf_node
rviz -d /opt/ultrafusion/rviz/lio.rviz
```

Proceed to [§2 Run on Five Benchmarks](#2-run-on-five-benchmarks).

---

### Option B — Native install

For users who prefer running directly on the host without Docker.

**Step 1 — Clone and install dependencies**

```bash
git clone https://github.com/sjtuyinjie/Ultra-Fusion.git
cd Ultra-Fusion
./scripts/install_native_deps.sh
```

Installs ROS Noetic, PCL/OpenCV/Eigen, and builds Ceres 2.1.0 and yaml-cpp 0.8.0.

**Step 2 — Install Ultra-Fusion**

```bash
./scripts/install_ultrafusion_deb.sh   # add --mirror if GitHub is slow
```

**Step 3 — Source ROS** (required in every new shell)

```bash
source /opt/ros/noetic/setup.bash
```

**Step 4 — Verify**

```bash
which uf_node
rviz -d /opt/ultrafusion/rviz/lio.rviz
```

---

### Installed files

The `.deb` installs:

| Path | Description |
| --- | --- |
| `/opt/ultrafusion/bin/uf_node` | Main executable |
| `/usr/bin/uf_node`, `/usr/bin/uf-node` | CLI shortcuts |
| `/opt/ultrafusion/config/m3dgr` | M3DGR profiles |
| `/opt/ultrafusion/config/m2p` | M2DGR-Plus profile |
| `/opt/ultrafusion/config/lvig` | MARS-LVIG profile |
| `/opt/ultrafusion/config/kaist` | KAIST profile |
| `/opt/ultrafusion/config/groundtour` | GrandTour profile |
| `/opt/ultrafusion/config/visual_life` | Computer Vision Life D360 multi-camera LVIO profile, available in v0.1.1 |
| `/opt/ultrafusion/rviz/lio.rviz` | Default RViz layout |

---
## 2. Run on Five Benchmarks

Each released `uf_node <shortcut>` command maps to a YAML profile under `/opt/ultrafusion/config/`.
You can also pass a config path directly: `uf_node /path/to/config.yaml`.

**Typical workflow** — open three terminals:

| Terminal | Command |
| --- | --- |
| 1 | `roscore` |
| 2 | `rosbag play /path/to/your.bag --clock` |
| 3 | `uf_node <shortcut>` |

Open RViz in a fourth terminal (optional): `rviz -d /opt/ultrafusion/rviz/lio.rviz`

### 2.1 M3DGR

Download [**M3DGR**](https://github.com/sjtuyinjie/M3DGR) rosbags and run:

```bash
# Terminal 1
roscore

# Terminal 2
rosbag play /media/path/to/your.bag --clock

# Terminal 3
uf_node m3dgr                  # default (= m3dgr_standard)
# uf_node m3dgr_standard       # standard LVWIO profile
# uf_node m3dgr_image_enhance  # stronger visual coupling
```

**Released profiles:**

| Command | Config | Recommended sequences |
| --- | --- | --- |
| `uf_node m3dgr` | `.../m3dgr/uf_m3dgr_standard.yaml` | Default M3DGR profile (alias of `m3dgr_standard`) |
| `uf_node m3dgr_standard` | `.../m3dgr/uf_m3dgr_standard.yaml` | Dynamic01, Varying-illu01, Dark01, Occlusion01 |
| `uf_node m3dgr_image_enhance` | `.../m3dgr/uf_m3dgr_image_enhance.yaml` | Corridor01, GNSS-denial01, Longtime01, Longtime02 |

> Legacy aliases `uf_node m3dgr_01` and `uf_node m3dgr_02` remain supported.

**Expected output (M3DGR)** — live LiDAR map + trajectory in RViz:

<table>
  <tr>
    <td width="33%" align="center">
      <img src="images/gifs/corridor_full.gif" alt="M3DGR Corridor demo" width="100%">
      <br>
      <strong>Corridor01</strong> · <code>uf_node m3dgr_image_enhance</code>
      <br>
      <em>Stable localization in an indoor corridor under vision challenge and LiDAR degeneration.</em>
    </td>
    <td width="33%" align="center">
      <img src="images/gifs/gnss_denial_full.gif" alt="M3DGR GNSS-denial demo" width="100%">
      <br>
      <strong>GNSS-denial01</strong> · <code>uf_node m3dgr_image_enhance</code>
      <br>
      <em>Continuous state estimation when GNSS measurements are unavailable.</em>
    </td>
    <td width="33%" align="center">
      <img src="images/gifs/longtime02_full.gif" alt="M3DGR Longtime02 demo" width="100%">
      <br>
      <strong>Longtime02</strong> · <code>uf_node m3dgr_image_enhance</code>
      <br>
      <em>Long-duration operation with consistent map and path alignment.</em>
    </td>
  </tr>
</table>

### 2.2 Other datasets

Additional shortcuts for cross-platform reproducibility. Sequences not listed may need parameter retuning.

| Command | Config | Recommended sequences |
| --- | --- | --- |
| `uf_node m2p` | `.../m2p/uf_m2p.yaml` | M2DGR-Plus bridge1-style LVWIO |
| `uf_node lvig` | `.../lvig/uf_lvig.yaml` | MARS-LVIG HKairport01 LVIO |
| `uf_node kaist` | `.../kaist/uf_kaist.yaml` | KAIST urban25, urban35 |
| `uf_node groundtour` | `.../groundtour/uf_groundtour.yaml` | GrandTour SPX-2, SNOW-2, EIG-1 |

**Expected output (other datasets)** — compare your RViz playback with:

<table>
  <tr>
    <td width="33%" align="center">
      <img src="images/gifs/arc2_full.gif" alt="GrandTour Arc2 demo" width="100%">
      <br>
      <strong>GrandTour Arc2</strong> · <code>uf_node groundtour</code>
      <br>
      <em>Legged-platform mapping and trajectory recovery through a large arc-shaped route.</em>
    </td>
    <td width="33%" align="center">
      <img src="images/gifs/kaist_full.gif" alt="KAIST urban driving demo" width="100%">
      <br>
      <strong>KAIST urban25/35</strong> · <code>uf_node kaist</code>
      <br>
      <em>Large-scale LiDAR mapping with high-speed vehicle trajectory visualization.</em>
    </td>
    <td width="33%" align="center">
      <img src="images/gifs/lvig_full.gif" alt="MARS-LVIG aerial demo" width="100%">
      <br>
      <strong>MARS-LVIG HKairport01</strong> · <code>uf_node lvig</code>
      <br>
      <em>Aerial LVIO reconstruction with dense point cloud and flight trajectory.</em>
    </td>
  </tr>
</table>

<a id="visual-life-d360"></a>

### 2.3 Computer Vision Life D360 Sensor Adaptation

A compact Computer Vision Life D360 adaptation note is provided here so the main README stays focused.

<p align="center">
  <img src="images/gifs/d360_visual_life.gif" alt="Computer Vision Life D360 multi-camera UF LVIO demo" width="70%">
</p>

- Detailed guide: [docs/visual_life_d360.md](docs/visual_life_d360.md)
- Release package: download `ultrafusion_0.1.1_amd64.deb` from GitHub Releases. v0.1.0 remains the paper package.
- Runtime entry: `uf_node visual_life` after installing the release package, or `uf_node /path/to/config/visual_life/config.yaml` for a custom config copy.
- Key outputs: `/result_path`, `/curr_cloud`, `/feature_reproject_cloud`, `/colored_lidar_cloud`
- Demo video: add the recording link to the detailed guide after publication.

This guide highlights the new optional multi-camera adaptation in UF (`use_multi_camera: true`). Use release package v0.1.1 for D360; release v0.1.0 remains the paper version. Existing profiles without `use_multi_camera` keep the original single-camera path.


## 3. Custom Profiles

Released shortcuts are aliases to YAML files under `/opt/ultrafusion/config/`.
To customize, **copy the closest profile directory** so camera-intrinsic files keep their relative paths. Avoid creating a minimal YAML from scratch — the runtime expects the full field set at startup.

```bash
WORK=/tmp/uf_config
mkdir -p "$WORK"
cp -a /opt/ultrafusion/config/m3dgr "$WORK"/

CFG="$WORK/m3dgr/uf_m3dgr_standard.yaml"
${EDITOR:-nano} "$CFG"

roscore &
sleep 3
uf_node "$CFG" &
rosbag play /media/path/to/your.bag --clock
```

> `uf_node` reads YAML only at startup — restart it after any config change.

### 3.1 Fusion modes

Keep `imu: 1` for the modes below. Visual sensing is selected by `use_image`.
`use_gf_standalone_vio` is not the UF visual switch: in the current runtime it
only selects the Ground-Fusion standalone backend for pure VIO
(`use_lidar: 0`, `use_image: 1`, `wheel: 0`). Ultra-Fusion also has its own
native VIO/VIWO/LVIO/LVWIO path; keep `use_gf_standalone_vio: false` for those
UF modes.

| Target mode | `use_lidar` | `use_image` | `wheel` | `use_gf_standalone_vio` | Runtime path |
| --- | --- | --- | --- | --- | --- |
| UF `lvwio` | `1` | `1` | `1` | `false` | Native UF LiDAR + visual + wheel |
| UF `lvio` | `1` | `1` | `0` | `false` | Native UF LiDAR + visual |
| UF `vio` | `0` | `1` | `0` | `false` | Native UF standalone VIO (`UFVIO`) |
| GF standalone `vio` | `0` | `1` | `0` | `true` | Ground-Fusion standalone VIO baseline |
| UF `viwo` | `0` | `1` | `1` | `false` | Native UF standalone visual + wheel |
| UF `wio` | `0` | `0` | `1` | ignored | Native UF standalone wheel + IMU |
| UF `lio` | `1` | `0` | `0` | ignored | LiDAR + IMU |
| UF `lwio` | `1` | `0` | `1` | ignored | LiDAR + wheel + IMU |

Set `depth: 1` only for RGB-D visual profiles that really provide the configured
depth image; use `depth: 0` for monocular RGB visual profiles. `use_lidar_reproject`
only matters for LiDAR+visual profiles, so keep the copied profile's value
unless you are intentionally evaluating that coupling. `use_planar_wheel_factor`
selects the planar wheel factor model; keep the released profile's value unless
you are intentionally comparing it with the legacy wheel-pose factor.

Sensor topics are configured in `common`:

```yaml
common:
  imu_topic: /camera/imu
  lid_topic: /livox/mid360/lidar
  wheel_topic: /odom
  image0_topic: /camera/color/image_raw/compressed
  image1_topic: /camera/aligned_depth_to_color/image_raw
```



### 3.2 Camera intrinsics

Camera intrinsics are not stored in the main UF YAML. The main YAML points to
camodocal/OpenCV calibration YAML files:

```yaml
cam0_calib: "color.yaml"
cam1_calib: "color.yaml"
```

`cam0_calib` is the primary visual camera file. The runtime loads it as
`<directory-of-main-config>/<cam0_calib>`, so keep the calibration file next to
the copied main YAML or preserve the released config directory layout as in the
copy example above. In the current runtime this field is treated as a path
relative to the main config directory; an absolute path will still be prefixed
by that directory.

The released visual profiles use `PINHOLE` or `KANNALA_BRANDT` camera models:

```yaml
%YAML:1.0
---
model_type: PINHOLE
camera_name: camera
image_width: 640
image_height: 480
distortion_parameters:
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0
projection_parameters:
  fx: 607.79772949218
  fy: 607.83526613281
  cx: 328.79772949218
  cy: 245.53321838378
```

For `KANNALA_BRANDT`, use `projection_parameters` fields
`mu`, `mv`, `u0`, `v0`, `k2`, `k3`, `k4`, and `k5`, following the released
fisheye-style calibration files. `cam1_calib` is only used when the runtime is
configured for the two-camera path; for the current single-camera/RGB-D public
profiles, keep it consistent with the released template. RGB-D depth input is
controlled by `depth: 1` and `common.image1_topic`, not by giving the depth image
its own camera-intrinsic YAML.

### 3.3 GNSS fusion

GNSS is independent of the LiDAR/visual/wheel mode switches. UF estimator paths
can add raw GNSS pseudorange/Doppler factors and position-only
`sensor_msgs/NavSatFix` factors when the bag provides the required topics. The
GF standalone VIO backend receives raw GNSS only; position-only GNSS fixes are
not consumed by that backend.

| Use case | Main fields | Notes |
| --- | --- | --- |
| Disable GNSS | `gnss_enable: 0` | No GNSS subscribers are started |
| Raw GNSS | `gnss_enable: 1`, `gnss_raw_enable: true`, `gnss_position_enable: false` | Requires range measurements plus ephemeris/iono topics |
| Position-only GNSS | `gnss_enable: 1`, `gnss_raw_enable: false`, `gnss_position_enable: true` | Uses `sensor_msgs/NavSatFix` in UF estimator paths |
| Raw + position GNSS | `gnss_enable: 1`, `gnss_raw_enable: true`, `gnss_position_enable: true` | Use only when both measurement types are available |

Typical GNSS topic and lever-arm fields:

```yaml
gnss_meas_topic: /ublox_driver/range_meas
gnss_position_topic: /ublox_driver/receiver_lla
gnss_ephem_topic: /ublox_driver/ephem
gnss_glo_ephem_topic: /ublox_driver/glo_ephem
gnss_iono_params_topic: /ublox_driver/iono_params

gnss_use_antenna_extrinsic: false
gnss_antenna_in_body: [0.0, 0.0, 0.0]
```

If `gnss_use_antenna_extrinsic` is true, `gnss_antenna_in_body` is the antenna
position in the estimator body/IMU frame. Do not enable raw GNSS without the
matching ephemeris topics; use position-only GNSS in a UF estimator profile or
leave GNSS off.

### 3.4 Extrinsics

All extrinsics are under `mapping`. Ultra-Fusion uses `T_A_B` to mean
"transform a point from frame `B` into frame `A`":

```text
p_A = R_A_B * p_B + t_A_B
```

Rotation arrays are row-major 3x3 matrices, and translations are in meters.

| YAML fields | Transform | Meaning |
| --- | --- | --- |
| `extrinsic_T`, `extrinsic_R` | `T_I_L` | LiDAR frame `L` to IMU/body frame `I` |
| `extrinsic_TIC`, `extrinsic_RIC` | `T_I_C` | Camera frame `C` to IMU/body frame `I` |
| `extrinsic_TCL`, `extrinsic_RCL` | `T_C_L` | LiDAR frame `L` to camera frame `C` |
| `extrinsic_TOL`, `extrinsic_ROL` | `T_O_L` | LiDAR frame `L` to wheel/odometer frame `O` |
| `extrinsic_TIO`, `extrinsic_RIO` | `T_I_O` | Wheel/odometer frame `O` to IMU/body frame `I` |

Runtime priority:

| Runtime transform | How UF obtains it |
| --- | --- |
| `T_I_L` | Always reads `mapping.extrinsic_T/R` |
| `T_I_C` | Uses `mapping.extrinsic_TIC/RIC` if present; otherwise computes `T_I_L * inverse(T_C_L)` from `extrinsic_TCL/RCL` |
| `T_C_L` | If `T_I_C` is present, UF also derives internal `T_C_L = inverse(T_I_C) * T_I_L` |
| `T_I_O` | Uses explicit `mapping.extrinsic_TIO/RIO` if present; otherwise computes `T_I_L * inverse(T_O_L)` from `extrinsic_TOL/ROL` |

There is no public YAML flag named `estimate_wheel_extrinsic`. To change the
wheel extrinsic, provide `extrinsic_TIO/RIO` directly or provide a correct
`extrinsic_TOL/ROL` so UF can derive `T_I_O`.

### 3.5 Calibration & delays

For fixed calibration, keep both visual online-calibration flags at zero:

```yaml
estimate_extrinsic: 0
estimate_td: 0
td: 0.0
```

In the current runtime, `estimate_extrinsic` and `estimate_td` are treated as a
joint online camera-IMU calibration request: any nonzero value starts the
`T_I_C + td` calibration state machine after the visual feature and motion
excitation gates pass. The state machine first commits `T_I_C`, then enters the
visual delay (`td`) stage. Therefore `estimate_td: 1` alone should not be read
as an isolated delay-only mode. This state machine is driven in the UF
LiDAR/visual processing path; pure UF standalone VIO/VIWO and GF standalone VIO
keep their `T_I_C` and `td` parameter blocks fixed in the solver.

| Field | Scope | Code behavior |
| --- | --- | --- |
| `estimate_extrinsic` | Camera-IMU | `0` does not request online visual calibration by itself; nonzero requests the joint `T_I_C + td` state machine |
| `estimate_td` | Visual timing | `0` does not request online visual calibration by itself; nonzero also requests the same `T_I_C + td` state machine |
| `td` | Visual timing | Visual state time uses `image_timestamp + td` |
| `common.img_time_offset` | ROS image stamp | Added to the ROS image timestamp before visual buffering; this is separate from `td` |
| `wheel_initial_td` | Wheel timing | Wheel state time uses `wheel_timestamp + wheel_initial_td` |
| `TimeSync.initial_lidar_to_imu_dt_sec` | LiDAR-IMU timing | Initial LiDAR-to-IMU time offset |
| `TimeSync.enable_lidar_imu_online_dt` | LiDAR-IMU timing | Enables online LiDAR-IMU time-offset estimation |

The current public configs use `wheel_initial_td` for wheel timing. Legacy
fields such as `estimate_td_wheel` and `td_wheel` are not the public switch for
wheel-delay calibration.

LiDAR-IMU online extrinsic calibration is configured separately:

```yaml
lidar_imu_calib:
  enable: false
  enable_lock_result: true
  freeze_after_locked_result: true
  apply_locked_result_to_slam: false
```

This LiDAR-IMU calibrator estimates the rotation part of `T_I_L`. Locked
rotation results affect SLAM only when `apply_locked_result_to_slam: true`; the
LiDAR-IMU translation used by SLAM remains the YAML translation unless you edit
the config.

When checking a new profile, inspect the startup log lines for `Opti_TIC`,
`Opti_TIO`, `td`, wheel `td`, GNSS status, and LiDAR-IMU time sync. A smooth but
biased trajectory is often a frame or time-offset error, not just solver tuning.





---

## 4. Qualitative Results

### 4.1 Robustness Under Degradation

<p align="center">
  <img src="images/degradation_qualitative.png" alt="Qualitative robustness under sensor degradation" width="88%">
</p>
<p align="center"><em>Representative stress cases: challenging perception conditions with consistent trajectory and map quality.</em></p>

### 4.2 Cross-Platform Results

<p align="center">
  <img src="images/trajs.png" alt="Trajectories across ground, legged, and aerial platforms" width="88%">
</p>
<p align="center"><em>Trajectory estimation examples on ground, legged, and UAV datasets.</em></p>

> For full-scene playback demos, see **Expected output** in [§2.1](#21-m3dgr) and [§2.2](#22-other-datasets).

---

## 5. License & Acknowledgements

This project is licensed under the MIT License. If you find this project useful, please cite:

```bibtex
@article{tian2026ultra,
  title={Ultra-Fusion: A Resilient Tightly-Coupled Multi-Sensor Fusion SLAM Framework under Sensor Degradation and Spatiotemporal Perturbation for Intelligent Transportation Systems},
  author={Tian, Yihong and Zhang, Junjie and Li, Liuyang and Zhang, Deteng and Zuo, Yunfei and Yin, Jie},
  journal={arXiv preprint arXiv:2606.21223},
  year={2026}
}
```

Please also consider citing our previous works related to this project:
```bibtex

@article{zhang2025towards,
  author={Zhang, Deteng and Zhang, Junjie and Sun, Yan and Li, Tao and Yin, Hao and Xie, Hongzhao and Yin, Jie},
  booktitle={2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework}, 
  year={2025},
  volume={},
  number={},
  pages={8894-8901},
  doi={10.1109/IROS60139.2025.11247507}}


@inproceedings{yin2024ground,
  title={Ground-fusion: A low-cost ground slam system robust to corner cases},
  author={Yin, Jie and Li, Ang and Xi, Wei and Yu, Wenxian and Zou, Danping},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8603--8609},
  year={2024},
  organization={IEEE}
}
@article{yin2021m2dgr,
  title={M2dgr: A multi-sensor and multi-scenario slam dataset for ground robots},
  author={Yin, Jie and Li, Ang and Li, Tao and Yu, Wenxian and Zou, Danping},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={2266--2273},
  year={2021},
  publisher={IEEE}
}
```
This work is self-funded. Thanks to Tianbao Zhang for providing computation resources. For maintenance, collaboration, or business inquiries, contact maotian616@gmail.com.

## 6. Star History

[![Star History Chart](https://api.star-history.com/svg?repos=sjtuyinjie/Ultra-Fusion&type=Timeline)](https://star-history.com/#Ashutosh00710/github-readme-activity-graph&Timeline)
