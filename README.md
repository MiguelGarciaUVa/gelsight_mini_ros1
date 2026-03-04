# gelsight_mini_ros1

ROS1 (Noetic) wrapper for **GelSight Mini** that publishes the tactile camera stream and (optionally) contact mask / depth proxy / point cloud for visualization in RViz.

This package is designed to work on Linux/Jetson where the official `gsrobotics` Python utilities work reliably (via `GelSightMini` class), and exposes the same data through ROS topics/services.

---

## Features

### Driver node (GelSight -> ROS Image)
Publishes:
- `image_raw` (`sensor_msgs/Image`) tactile RGB stream

Optional contact outputs (baseline + absdiff):
- `contact_mask` (`sensor_msgs/Image`, `mono8`)
- `contact` (`std_msgs/Bool`)
- `contact_area` (`std_msgs/Float32`)
- `contact_center` (`geometry_msgs/PointStamped`)

Service:
- `calibrate_baseline` (`std_srvs/Trigger`)

### PointCloud node (Image -> Mask/Depth/PointCloud)
Subscribes:
- `image_raw`

Publishes:
- `mask` (`sensor_msgs/Image`, `mono8`)
- `depth` (`sensor_msgs/Image`, `32FC1`) **(proxy by default)**
- `points` (`sensor_msgs/PointCloud2`)

Service:
- `calibrate_baseline` (`std_srvs/Trigger`)

> Note: `depth` is a **proxy** (from image difference) unless you plug in the real depth pipeline from GelSight/gsrobotics. The rest of the ROS interface (topics + PointCloud2) stays the same.

---

## Requirements

- ROS1 Noetic (Ubuntu 20.04)
- Python 3.8 (default on Noetic)
- `cv_bridge`
- `sensor_msgs`, `geometry_msgs`, `std_srvs`
- A working GelSight Mini + the Python utilities that provide `GelSightMini` (from GelSight/gsrobotics or equivalent)

### Install ROS dependencies
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-view \
  ros-noetic-rqt-image-view \
  ros-noetic-tf2-ros
```
### Build
Clone into your catkin workspace:
```
cd ~/catkin_workspace/src
git clone https://github.com/MiguelGarciaUVa/gelsight_mini_ros1.git
cd ..
catkin_make
source devel/setup.bash
```
---
### Quickstart
#### 1) Launch a single sensor

This starts the left GelSight in namespace /gelsight_left:
```
roslaunch gelsight_mini_ros1 gelsight_left.launch
```
Or right:
```
roslaunch gelsight_mini_ros1 gelsight_right.launch
```
Dual:
```
roslaunch gelsight_mini_ros1 gelsight_dual.launch
```

**Device selection (`device_index`)**

The GelSight Mini may appear as multiple V4L2 devices (e.g. `video-index0` and `video-index1`). In many setups, only one of them provides a valid video stream.  
Use the `device_index` launch argument to select the correct one, and note that the index can change depending on camera enumeration order (especially with multiple cameras).

Examples:
```bash
# Single sensor
roslaunch gelsight_mini_ros1 gelsight_left.launch device_index:=0

# Dual sensor
roslaunch gelsight_mini_ros1 gelsight_dual.launch left_device_index:=0 right_device_index:=1
```

#### 2) View the image
```
rqt_image_view
```
Select:

`/gelsight_left/image_raw` (or `/gelsight_right/image_raw`)

#### 3) Calibrate baseline (for contact mask)

Run this with the sensor not touching anything:
```
rosservice call /gelsight_left/calibrate_baseline "{}"
```

#### 4) Run point cloud node

In the same namespace as the driver:
```
rosrun gelsight_mini_ros1 gelsight_pointcloud_node.py __ns:=/gelsight_left _frame_id:=gelsight_left_frame
```

#### 5) Visualize point cloud in RViz (no TF)

If you don't publish TF yet, set **RViz → Fixed Frame** to:

- `gelsight_left_frame`

Then add these displays:

- **PointCloud2**: `/gelsight_left/points`
- **Image**: `/gelsight_left/mask`
- **Image**: `/gelsight_left/depth`


### TF (recommended)

To visualize in a global robot frame (e.g. `base_link`), publish a static transform:

```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link gelsight_left_frame
```
Then in RViz set:
- **Fixed Frame** = `base_link`

---

## Topics
### Driver node (namespace example: `/gelsight_left`)
- `/gelsight_left/image_raw` (`sensor_msgs/Image`)
- `/gelsight_left/contact_mask` (`sensor_msgs/Image`, `mono8`)
- `/gelsight_left/contact` (`std_msgs/Bool`)
- `/gelsight_left/contact_area` (`std_msgs/Float32`)
- `/gelsight_left/contact_center` (`geometry_msgs/PointStamped`)

### PointCloud node
- `/gelsight_left/mask` (`sensor_msgs/Image`, `mono8`)
- `/gelsight_left/depth` (`sensor_msgs/Image`, `32FC1`)
- `/gelsight_left/points` (`sensor_msgs/PointCloud2`)

---

## Parameters

### Common
- `~fps` (default: `15`)
- `~frame_id` (default: `gelsight_frame`)
- `~encoding` (`rgb8` or `bgr8`)

### GelSightMini acquisition
- `~device_index` (default: `1`) — index passed to `select_device()`
- `~target_width` (default: `640`)
- `~target_height` (default: `480`)
- `~border_fraction` (default: `0.15`)

### Contact mask
- `~mask_threshold` (default: `25.0`)
- `~min_area` (default: `400.0`)
- `~blur_ksize` (default: `7`)
- `~baseline_frames` (default: `20`)

### PointCloud (proxy)
- `~xy_scale` (default: `0.0002`) — meters per pixel
- `~z_scale` (default: `1e-5`) — meters per diff unit
- `~pc_step` (default: `2`) — point downsample step

---

## Notes

If you use a Python environment (conda/venv), make sure you run nodes with the ROS environment sourced:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_workspace/devel/setup.bash
```

`device_index` mapping can change depending on camera enumeration order. For stable multi-sensor setups, prefer selecting by serial/path when available.

---

## Acknowledgements


This ROS wrapper builds on the official **GelSight `gsrobotics`** Python SDK and its demo pipelines (live view, image-based depth/point cloud visualization). The goal is to expose similar outputs through a ROS1 interface for easier integration and RViz visualization.

- GelSight `gsrobotics` SDK (GitHub): https://github.com/gelsightinc/gsrobotics  
- GelSight (company / products): https://www.gelsight.com/

---

## License
This project is licensed under the GPL-3.0 License. See the `LICENSE` file for details.
