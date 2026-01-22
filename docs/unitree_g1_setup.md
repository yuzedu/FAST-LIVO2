# FAST-LIVO2 Setup for Unitree G1

## Prerequisites

- ROS2 Humble
- Livox ROS2 driver (`livox_ros_driver2`)
- Calibration completed with iKalibr

### System Dependencies

```bash
sudo apt-get install -y ros-humble-sophus libfmt-dev
```

## Source code


```bash
git clone https://github.com/yuzedu/FAST-LIVO2.git
git clone https://github.com/yuzedu/rpg_vikit.git
git clone https://github.com/yuzedu/livox_ros_driver2.git
```

```bash
cd ~/fast_wa/src/livox_ros_driver.git
cp package_ROS2.xml package.xml
```

## Build
```bash
cd /home/yuzedu/fast_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running FAST-LIVO2

### 1. With Rosbag (Offline)

**Terminal 1** - Launch FAST-LIVO2:
```bash
source install/setup.bash
ros2 launch fast_livo mapping_custom.launch.py use_rviz:=True
```

**Terminal 2** - Play rosbag (must source workspace for Livox message types):
```bash
source install/setup.bash
ros2 bag play /home/yuzedu/fast_ws/data/rosbag2_2025_11_14-15_20_03
```

### 2. With Real-time Sensor Data (Online)

**Terminal 1** - Launch Livox driver:
```bash
source install/setup.bash
ros2 launch livox_ros_driver2 livox_lidar_msg.launch.py
```

**Terminal 2** - Launch FAST-LIVO2:
```bash
source install/setup.bash
ros2 launch fast_livo mapping_custom.launch.py use_rviz:=True
```

## Configuration

Config files location: `src/FAST-LIVO2/config/`

- `custom_sensor.yaml` - Main sensor configuration (topics, extrinsics, IMU settings)
- `custom_camera.yaml` - Camera intrinsics

### Key Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `img_en` | 0 | Camera disabled (timestamp sync issue) |
| `lidar_en` | 1 | LiDAR enabled |
| `gravity_align_en` | true | Auto-align map with gravity |
| `extrinsic_R` | Identity | IMU-LiDAR rotation (Livox built-in IMU) |

## Topics

**Input:**
- `/livox/lidar` - Livox CustomMsg
- `/livox/imu` - sensor_msgs/Imu
- `/camera/camera/color/image_raw` - sensor_msgs/Image (currently disabled)

**Output:**
- `/aft_mapped_to_init` - Odometry
- `/path` - Trajectory
- `/cloud_registered` - Registered point cloud
- `/Laser_map` - Full map

## Build Fixes

The following CMake modifications were made to resolve Sophus dependency issues:

**vikit_ros/CMakeLists.txt** - Replace `find_package(Sophus REQUIRED)` with:
```cmake
# Sophus is header-only, directly add ROS2 include path
set(Sophus_INCLUDE_DIRS "/opt/ros/humble/include")
```

**vikit_common/CMakeModules/FindSophus.cmake** - Created to locate Sophus in ROS2 paths.

**FAST-LIVO2/CMakeLists.txt** - Same Sophus fix + added `fmt::fmt` to link libraries.

## Known Issues

1. **Camera timestamps**: The RealSense camera uses a different time epoch than LiDAR/IMU, causing sync failures. Camera is currently disabled.

2. **Livox message type**: Must source workspace before playing rosbag, otherwise `/livox/lidar` topic won't publish.
