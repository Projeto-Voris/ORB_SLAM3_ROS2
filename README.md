# ORB_SLAM3_ROS2

A modernised, modular ROS 2 wrapper for **ORB-SLAM3**, optimised for high-performance intra-process communication (IPC) and robust feature tracking in challenging environments. This repository consolidates legacy monolithic tracking executables into a single unified architecture tailored for the **VORIS Project** at **LabMetro - UFSC**.

---

## Technical Highlights & Architecture

* **Unified SLAM Engine (`slam_node.cc`)**: A single, consolidated state machine manages all underlying tracking pipelines, eliminating code duplication across separate binaries.
* **Component-Based / Composable Nodes**: High-bandwidth pipelines like **Stereo** and **Stereo-Inertial** are engineered as ROS 2 Composable Nodes. By running them within a component container, they leverage true **Intra-Process Communication (IPC)**, allowing zero-copy pointer passing that completely bypasses network serialisation overhead.
* **Low-Bandwidth Resiliency**: **Monocular** and **Mono-Inertial** pipelines support native `image_transport` **compressed image streams**, enabling reliable tracking over tethered, high-latency, or narrow-bandwidth connections (e.g., ROV umbilical cables).
* **Adaptive Contrast Enhancement (CLAHE)**: Integrated Contrast Limited Adaptive Histogram Equalisation preprocessing addresses uneven illumination, backscatter, or low-visibility scenarios by sharpening structural features before they reach the ORB extractor.
* **Advanced TF & Reference Frame Control**:
* `ENU_publish`: Dynamically maps tracking output. Set to `True` to publish state estimates in standard ROS coordinate frames (East-North-Up), or `False` to output raw, right-handed camera coordinates.
* **Coordinate Tree Integration**: The node parses `parent_frame_id` (e.g., `base_link`) and `child_frame_id` (camera optical frame) parameters, automatically computing the camera-to-body spatial transformation and publishing tracking frames directly to the fixed `map` frame (`frame_id`).



---

## Supported Pipeline Matrix

| Tracking Mode | Node Type | Compressed Image Transport | CLAHE Preprocessing |
| --- | --- | --- | --- |
| **Monocular** | Standard / Component | ✅ Yes | ✅ Yes |
| **Mono-Inertial** | Standard / Component | ✅ Yes | ✅ Yes |
| **Stereo** | Composable (IPC Optimized) | ❌ No | ✅ Yes |
| **Stereo-Inertial** | Composable (IPC Optimized) | ❌ No | ✅ Yes |

---

## Prerequisites & Installation

### 1. Build the Core SLAM Engine

This wrapper requires the accompanying, custom-patched version of the core tracking system. Ensure you have successfully compiled the [VORIS ORB-SLAM3 Core Engine](https://github.com/Projeto-Voris/ORB-SLAM3) first.

### 2. Resolve ROS Dependencies

Use `rosdep` to download and link system dependencies (`vision-opencv`, `image-transport`, `message-filters`):

```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -y

```

### 3. Clone and Compile

Ensure you explicitly target the tracking development branch during cloning:

```bash
cd ~/colcon_ws/src
git clone https://github.com/Projeto-Voris/ORB_SLAM3_ROS2.git orbslam3_ros2

cd ~/colcon_ws
colcon build --symlink-install --packages-up-to orbslam3_ros2

```

---

## Configuration & Launch Parameters

Node configurations are managed directly through native ROS 2 parameter blocks rather than legacy command-line arguments.

### Parameter Definitions

| Parameter Name | Type | Description |
| --- | --- | --- |
| `voc_file` | `string` | Absolute path to the `ORBvoc.txt` vocabulary file. |
| `settings_file` | `string` | Absolute path to the sensor calibration `.yaml` configuration. |
| `resize_factor` | `double` | Rescale factor to input image of `ORB-SLAM3` pipeline. |
| `tf_publish` | `bool` | `True` for broadcasting tf in `/tf` topic. |
| `ENU_publish` | `bool` | `True` for standard ROS frames; `False` for raw camera-frame coordinates. |
| `parent_frame_id` | `string` | System base reference frame link (typically `base_link`). |
| `child_frame_id` | `string` | Origin optical coordinate frame link of the tracking camera sensor. |
| `frame_id` | `string` | Global fixed world coordinates tracking anchor (typically `map`). |

---

## Project Context

This software operates as a core state estimation module for the **VORIS Project** (*Veículo Operado Remotamente para Inspeção Submarina*) managed under **LabMetro - UFSC**. The pipeline is custom-tailored to provide deterministic localization data and robust feature tracking required for automated close-quarters visual inspection of maritime installations and structures.
