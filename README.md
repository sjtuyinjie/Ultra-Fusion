# Ultra-Fusion: A Resilient Tightly-Coupled Multi-Sensor Fusion SLAM Framework under Sensor Degradation and Spatiotemporal Perturbation for Intelligent Transportation Systems

<div align="center">

[![Website](https://img.shields.io/badge/Website-Ultra--Fusion-blue)](https://sjtuyinjie.github.io/ultrafusion-web/)
[![Platform](https://img.shields.io/badge/Platforms-Ground%20%7C%20Legged%20%7C%20Aerial-green)](#cross-platform-results)
[![License](https://img.shields.io/badge/License-MIT-orange)](#license)

</div>

Ultra-Fusion is a tightly-coupled multi-sensor SLAM/localization framework for intelligent transportation systems (ITS).
It is designed for real deployment where sensor degradation (illumination changes, LiDAR degeneracy, wheel slippage, GNSS outage) and spatiotemporal miscalibration are common.

The system unifies WIO, VIO, LIO, and LVIO in one configurable optimization framework, with optional wheel/GNSS fusion and online calibration.

> [!NOTE]
> We currently release executable binaries and demos. Full source code will be released after paper acceptance.

---

## Highlights

- Unified sliding-window estimator with timestamp-ordered heterogeneous factors.
- Observability-aware initialization for robust bootstrap under diverse motion/sensor conditions.
- Factor-wise reliability scheduling (FRS) to gate/down-weight degraded measurements.
- Online LiDAR-IMU spatiotemporal calibration during operation.
- Validated on wheeled, legged, and aerial platforms across multiple public benchmarks.

---

## Method Overview

Ultra-Fusion converts asynchronous sensor streams into optional factors in one optimization window, with shared state representation, marginalization, and calibration logic.

<p align="center">
  <img src="images/pipeline.png" alt="Ultra-Fusion pipeline" width="92%">
</p>
<p align="center"><em>Unified pipeline: initialization, reliability scheduling, online calibration, and multi-modal fusion in one framework.</em></p>

---

## Why Ultra-Fusion

Compared with conventional fusion pipelines that are heavily tied to a fixed sensor set, Ultra-Fusion emphasizes:

1. **Configurability**: one framework for WIO/VIO/LIO/LVIO (+ wheel/GNSS).
2. **Reliability**: robust localization under corner-case degradations.
3. **Deployability**: support for long-term and high-speed operation in real ITS scenarios.
4. **Transferability**: validated beyond wheeled robots to legged and aerial platforms.

---



## Benchmarks and Findings

Ultra-Fusion is evaluated on:

- [**M3DGR**](https://github.com/sjtuyinjie/M3DGR) (wheeled, real+sim, sensor degradation),
- [**M2DGR-Plus**](https://github.com/SJTU-ViSYS/M2DGR-plus)(wheeled),
- [**KAIST (Complex Urban Dataset)**](https://sites.google.com/view/complex-urban-dataset)(autonomous driving),
- [**GrandTour**](https://github.com/leggedrobotics/grand_tour_dataset) (legged),
- [**MARS-LVIG**](https://mars.hku.hk/dataset.html) (aerial).

Across these datasets, the paper reports competitive localization performance and improved availability under:

- sensor degradation (visual/LiDAR/wheel/GNSS),
- temporal/extrinsic perturbations,
- long-duration and high-speed operation.

---

## Quick Start

The public runtime is tested on Ubuntu 20.04 + ROS Noetic. The Docker image is
only the ROS/runtime environment: it contains ROS Noetic, rosbag, RViz, Ceres,
yaml-cpp, and the required system libraries. It does not contain the
Ultra-Fusion source tree or the release `.deb`.

### Pull the Docker Image for Environments



```bash
#Alibaba Cloud ACR:
docker pull registry.cn-hangzhou.aliyuncs.com/bit_robot_image/ultrafusion:0.1.0

#Docker Hub:
docker pull maotiandocker/ultrafusion:0.1.0

#Or build the public runtime image from the Dockerfile:
docker build -t ultrafusion:0.1.0 .
```

### Install the Release Deb of Ultra-Fusion

Start a container. The `/media` mount is optional, but convenient when rosbag
files are stored on the host under `/media`.

```bash
xhost +local:docker

docker run --rm -it --net=host --ipc=host \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /media:/media:ro \
  registry.cn-hangzhou.aliyuncs.com/bit_robot_image/ultrafusion:0.1.0
```

Inside the container, download and install the release package. The same deb is
available from GitHub Releases and the project mirror:

```bash
wget -O /tmp/ultrafusion.deb \
  https://github.com/sjtuyinjie/Ultra-Fusion/releases/download/v0.1.0/ultrafusion_0.1.0_amd64.deb

# Mirror:
# wget -O /tmp/ultrafusion.deb \
#   http://47.100.60.229:8088/loc_map/releases/ultrafusion/ultrafusion_0.1.0_amd64.deb

echo "c9a40d62df6100006431598d672c943f23f116e973e9c3b111d76d76c059196c  /tmp/ultrafusion.deb" | sha256sum -c -

dpkg -i /tmp/ultrafusion.deb
```

The deb installs:

- `/opt/ultrafusion/bin/uf_node`
- `/usr/bin/uf-node` and `/usr/bin/uf_node`
- `/opt/ultrafusion/config/m3dgr`
- `/opt/ultrafusion/config/m2p`
- `/opt/ultrafusion/config/lvig`
- `/opt/ultrafusion/config/kaist`
- `/opt/ultrafusion/config/groundtour`
- `/opt/ultrafusion/rviz/lio.rviz`

Open the included RViz layout with:

```bash
rviz -d /opt/ultrafusion/rviz/lio.rviz
```

### 🔥Run Ultra-Fusion on M3DGR
Download [**M3DGR**](https://github.com/sjtuyinjie/M3DGR) bags and give a star.
Start ROS and play your bag in the usual ROS way. Use one terminal for
`roscore`, one terminal for `rosbag play`, and one terminal for `uf_node`.

```bash
#play your bag
roscore & rosbag play /media/path/to/your.bag --clock
```

Run Ultra-Fusion in another shell:

```bash
#default setting
uf_node m3dgr

#standard LWIO
uf_node m3dgr_standard

#Stronger vision-coupling
uf_node m3dgr_image_enhance
```

M3DGR public release profiles:

| Command | Config | Recommended benchmark setting |
| --- | --- | --- |
| `uf_node m3dgr` | `/opt/ultrafusion/config/m3dgr/uf_m3dgr_standard.yaml` | Default M3DGR standard profile |
| `uf_node m3dgr_standard` | `/opt/ultrafusion/config/m3dgr/uf_m3dgr_standard.yaml` | Standard wheeled LVWIO profile: Dynamic01, Varying-illu01, Dark01, and Occlusion01 |
| `uf_node m3dgr_image_enhance` | `/opt/ultrafusion/config/m3dgr/uf_m3dgr_image_enhance.yaml` | Stronger visual-coupling profile: Corridor01, GNSS-denial01, Longtime01, and Longtime02 |

`uf_node m3dgr` is the default public M3DGR shortcut and is equivalent to
`uf_node m3dgr_standard`. Legacy aliases `uf_node m3dgr_01` and
`uf_node m3dgr_02` are still supported for backward compatibility.

## 🔥Run Ultra-Fusion on more datasets

These additional public shortcuts are included for reproducibility. The table
lists the dataset sequences/settings covered by the released profiles. Sequences
not listed here are outside the public release profile coverage and may require
separate parameter retuning.

| Command | Config | Recommended sequences/settings |
| --- | --- | --- |
| `uf_node m2p` | `/opt/ultrafusion/config/m2p/uf_m2p.yaml` | M2DGR-Plus bridge1-style LVWIO setting |
| `uf_node lvig` | `/opt/ultrafusion/config/lvig/uf_lvig.yaml` | MARS-LVIG HKairport01 LVIO setting |
| `uf_node kaist` | `/opt/ultrafusion/config/kaist/uf_kaist.yaml` | KAIST urban25 and urban35 |
| `uf_node groundtour` | `/opt/ultrafusion/config/groundtour/uf_groundtour.yaml` | GrandTour SPX-2, SNOW-2, and EIG-1 |

You can also pass a config path directly:

```bash
uf_node /opt/ultrafusion/config/m3dgr/uf_m3dgr_standard.yaml
```

---

## Qualitative Results

### Robustness Under Degradation

<p align="center">
  <img src="images/degradation_qualitative.png" alt="Qualitative robustness under sensor degradation" width="88%">
</p>
<p align="center"><em>Representative stress cases: challenging perception conditions with consistent trajectory and map quality.</em></p>

### Cross-Platform Results

<p align="center">
  <img src="images/trajs.png" alt="Trajectories across ground, legged, and aerial platforms" width="88%">
</p>
<p align="center"><em>Trajectory estimation examples on ground, legged, and UAV datasets.</em></p>





---

## License

This project is licensed under the MIT License. If you find this project useful, please cite:

```bibtex
@article{zhang2025towards,
  title={Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework},
  author={Zhang, Deteng and Zhang, Junjie and Sun, Yan and Li, Tao and Yin, Hao and Xie, Hongzhao and Yin, Jie},
  journal={arXiv preprint arXiv:2507.08364},
  year={2025}
}
```
