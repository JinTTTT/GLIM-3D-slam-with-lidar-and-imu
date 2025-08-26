# 3D SLAM System

A three-dimensional SLAM system based on the GLIM algorithm, using Ouster OS1-128 LiDAR and built-in IMU for real-time mapping and localization.

## Features

- **Real-time SLAM**: GPU-accelerated LiDAR-inertial mapping based on GLIM algorithm
- **Hardware Support**: Specifically adapted for Ouster OS1-128 LiDAR + built-in 6-axis IMU
- **Data Processing**: Supports PCAP raw data playback and ROS2 bag format conversion
- **High-precision Mapping**: Achieves centimeter-level accuracy 3D point cloud maps using factor graph optimization

## Core Components

- **GLIM**: High-precision 3D mapping framework based on factor graphs
- **[Ouster ROS](https://github.com/ouster-lidar/ouster-ros/tree/ros2)**: Official Ouster sensor ROS2 driver, supporting real-time sensor connection, PCAP playback and data recording
- **GTSAM**: Graph optimization library and point cloud processing tools

## Runtime Environment

- **Operating System**: Ubuntu 22.04 LTS
- **ROS Version**: ROS2 Humble

## Installation Guide

This project uses **installation from source**. For complete installation instructions, please refer to the [GLIM Official Installation Guide](https://koide3.github.io/glim/installation.html).

### 1. Install GLIM Dependencies

Please follow the **"Common dependencies"** section in the [GLIM Official Installation Guide](https://koide3.github.io/glim/installation.html) to install all system-level dependencies (GTSAM, Iridescence, gtsam_points, etc.).

### 2. Create ROS2 Workspace and Install GLIM

```bash
# Create working directory
mkdir -p ros2_ws/src

# Clone GLIM related packages
cd ros2_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

# Build GLIM
cd ../
colcon build
```

### 3. Install Ouster Official ROS2 Driver

To support PCAP data processing and sensor communication, install the [Ouster Official ROS2 Driver](https://github.com/ouster-lidar/ouster-ros/tree/ros2):

```bash
# Install Ouster related dependencies
sudo apt install -y libpcap-dev libjsoncpp-dev libspdlog-dev \
    libcurl4-openssl-dev libeigen3-dev libtins-dev

# Clone Ouster official ROS2 driver
cd ros2_ws/src
git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

# Build Ouster driver (enable PCAP support)
cd ../
source /opt/ros/humble/setup.bash
colcon build --packages-select ouster_sensor_msgs ouster_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PCAP=ON

# Rebuild all packages to ensure correct dependencies
colcon build
source install/setup.bash
```

> **Note**: Ouster ROS driver supports all sensors with FW v2.0 or higher (OS0, OS1, OS2, OSDome). For more detailed information, please refer to the [official documentation](https://github.com/ouster-lidar/ouster-ros/tree/ros2).

## Configuration

### 1. Configure ROS Topics

Edit the `ros2_ws/src/glim/config/config_ros.json` file to set topic names and coordinate frames in the ROS bag:

```json
"imu_frame_id": "os_imu",
"lidar_frame_id": "os_lidar",
"imu_topic": "/ouster/imu",
"points_topic": "/ouster/points"
```

### 2. Set LiDAR-IMU Relative Position

LiDAR-IMU relative position information can be obtained from the `.json` file of the raw data. Find the `imu_intrinsics.imu_to_sensor_transform` parameter in the JSON file in the `data/os_pcaps/` directory.

**Getting information from raw JSON**:
```json
// Example: data/os_pcaps/ouster_20250604072442.json
"imu_intrinsics": {
    "imu_to_sensor_transform": [
        1, 0, 0, 6.253,      // 4x4 transformation matrix
        0, 1, 0, -11.775,    // Unit: millimeters (mm)
        0, 0, 1, 7.645,
        0, 0, 0, 1
    ]
}
```

**Convert to GLIM format**:
Edit the `ros2_ws/src/glim/config/config_sensors.json` file to convert the transformation information to the format expected by GLIM:

```json
"T_lidar_imu": [0.006253, -0.011775, 0.007645, 0, 0, 0, 1]
```

> **Important conversion rules**:
> 1. **Unit conversion**: Raw data units are in millimeters (mm), divide by 1000 to convert to meters (m)
> 2. **Format conversion**: Convert from 4x4 transformation matrix to `[x, y, z, qx, qy, qz, qw]` format
> 3. **Coordinate extraction**: Extract the translation part (4th, 8th, 12th elements) from the transformation matrix as x, y, z coordinates

### 3. GPU Acceleration Configuration

This system uses **GPU acceleration by default** for optimal performance:

**Hardware Configuration:**
- **CPU**: Intel Core i7-13650HX (13th Gen)
- **GPU**: NVIDIA GeForce RTX 4060 (8GB VRAM)
- **CUDA**: Version 12.8

**GPU Mode (Default):**
Edit `ros2_ws/src/glim/config/config.json`:
```json
{
  "config_odometry": "config_odometry_gpu.json",
  "config_sub_mapping": "config_sub_mapping_gpu.json",
  "config_global_mapping": "config_global_mapping_gpu.json"
}
```

**CPU-only Mode:**
```json
{
  "config_odometry": "config_odometry_cpu.json",
  "config_sub_mapping": "config_sub_mapping_cpu.json",
  "config_global_mapping": "config_global_mapping_cpu.json"
}
```

## Running

To start the 3D SLAM system, first start GLIM, then prepare and play the data.

### 1. Start GLIM Mapping Algorithm

```bash
cd ros2_ws
source install/setup.bash
ros2 run glim_ros glim_rosnode
```

### 2. Data Preparation and Playback

Choose the appropriate playback method according to the data type:

#### Method 1: Directly play existing ROS2 bag files
```bash
ros2 bag play bags/official_ouster_for_glim
```

#### Method 2: Directly play PCAP files (PCAP playback mode)

According to the PCAP playback mode in the [Ouster Official Documentation](https://github.com/ouster-lidar/ouster-ros/tree/ros2):

```bash
# Indoor data (loop playback)
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=data/os_pcaps/ouster_20250604072442.pcap \
    metadata:=data/os_pcaps/ouster_20250604072442.json \
    viz:=false loop:=true

# Outdoor data (loop playback)
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=data/os_pcaps/ouster_20250604074152.pcap \
    metadata:=data/os_pcaps/ouster_20250604074152.json \
    viz:=false loop:=true
```

> **Note**: Starting from package version 8.1, the metadata parameter is optional if the bag file already contains a metadata topic.

#### Method 3: Recording mode - Pre-convert PCAP to ROS2 bag (recommended for multiple uses)

Use the recording mode provided by Ouster official:

**Step 1**: Start PCAP playback (no loop)
```bash
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=data/os_pcaps/ouster_20250604072442.pcap \
    metadata:=data/os_pcaps/ouster_20250604072442.json \
    viz:=false loop:=false
```

**Step 2**: Record as ROS2 bag (in another terminal)
```bash
ros2 bag record -o bags/my_recording /ouster/points /ouster/imu /tf_static
```
> **Important**: The `/tf_static` topic must be recorded, otherwise GLIM will report errors due to missing coordinate transformation information.

**Step 3**: Play the converted bag file
```bash
ros2 bag play bags/my_recording
```

> **Tip**: You can also directly use Ouster's official recording launch file:
> ```bash
> ros2 launch ouster_ros record.launch.xml \
>     sensor_hostname:=<sensor_ip> \
>     bag_file:=bags/my_recording
> ```


## Project Structure

```
3D_SLAM/
├── ros2_ws/                  # ROS2 workspace
│   ├── src/
│   │   ├── glim/            # Core mapping algorithm
│   │   ├── glim_ros2/       # ROS2 interface
│   │   └── ouster-ros/      # Ouster driver
│   └── bags/                # ROS2 bag data
└── data/       # Raw data and conversion tools
```

## Hardware

- **LiDAR**: Ouster OS1-128
- **IMU**: Built-in 6-axis IMU

## Performance Benchmarks

The following performance statistics were measured during GLIM execution with GPU acceleration enabled:

### System Configuration
- **CPU**: Intel Core i7-13650HX (13th Gen)
- **GPU**: NVIDIA GeForce RTX 4060 (8GB VRAM)
- **CUDA**: Version 12.8
- **Configuration**: GPU mode with VGICP_GPU acceleration

### Performance Results
```
📊 GLIM Performance Statistics
==================================================================
Duration: 100s | Samples: 19

💻 CPU (i7-13650HX):
   Average: 5.1%
   Peak:    9.9%

🎮 GPU (RTX 4060):
   Average: 9.7%     Peak: 24%
   Memory:  3.6%     Peak: 3.8%
   Temp:    56.0°C    Peak: 57°C

📈 Performance Assessment:
   CPU: Efficient (5.1% avg)
   GPU: Efficient (9.7% avg)
```

### Key Findings
- **Efficient Resource Usage**: Both CPU and GPU operate well below capacity
- **Low Memory Footprint**: GPU memory usage remains minimal (< 4%)
- **Thermal Management**: GPU temperature stays within optimal range
- **Real-time Capability**: System has significant headroom for real-time processing

> **Note**: Performance may vary depending on point cloud density, mapping area size, and data playback speed. These benchmarks represent typical indoor mapping scenarios with moderate point cloud complexity.