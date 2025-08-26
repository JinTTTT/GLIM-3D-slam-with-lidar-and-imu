# 3D SLAM 系统

基于 GLIM 算法的三维 SLAM 系统，使用 Ouster OS1-128 激光雷达和内置 IMU 进行实时建图与定位。

## 功能

- **实时 SLAM**: 基于 GLIM 算法的 GPU 加速激光雷达-惯性建图
- **硬件支持**: 专门适配 Ouster OS1-128 激光雷达 + 内置 6 轴 IMU
- **数据处理**: 支持 PCAP 原始数据回放与 ROS2 bag 格式转换
- **高精度建图**: 利用因子图优化实现厘米级精度的 3D 点云地图

## 核心组件

- **GLIM**: 基于因子图的高精度 3D 建图框架
- **[Ouster ROS](https://github.com/ouster-lidar/ouster-ros/tree/ros2)**: Ouster 传感器官方 ROS2 驱动，支持实时传感器连接、PCAP 回放和数据录制
- **GTSAM**: 图优化库和点云处理工具

## 运行环境

- **操作系统**: Ubuntu 22.04 LTS
- **ROS 版本**: ROS2 Humble

## 安装指南

本项目使用 **从源码安装** 的方式。完整的安装说明请参考 [GLIM 官方安装指南](https://koide3.github.io/glim/installation.html)。

### 1. 安装 GLIM 依赖库

请按照 [GLIM 官方安装指南](https://koide3.github.io/glim/installation.html) 中的 **"Common dependencies"** 部分安装所有系统级依赖库（GTSAM、Iridescence、gtsam_points 等）。

### 2. 创建 ROS2 工作空间并安装 GLIM

```bash
# 创建工作目录
mkdir -p ros2_ws/src

# 克隆 GLIM 相关包
cd ros2_ws/src
git clone https://github.com/koide3/glim
git clone https://github.com/koide3/glim_ros2

# 编译 GLIM
cd ../
colcon build
```

### 3. 安装 Ouster 官方 ROS2 驱动

为了支持 PCAP 数据处理和传感器通信，需要安装 [Ouster 官方 ROS2 驱动](https://github.com/ouster-lidar/ouster-ros/tree/ros2)：

```bash
# 安装 Ouster 相关依赖
sudo apt install -y libpcap-dev libjsoncpp-dev libspdlog-dev \
    libcurl4-openssl-dev libeigen3-dev libtins-dev

# 克隆 Ouster 官方 ROS2 驱动
cd ros2_ws/src
git clone -b ros2 --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

# 编译 Ouster 驱动 (启用 PCAP 支持)
cd ../
source /opt/ros/humble/setup.bash
colcon build --packages-select ouster_sensor_msgs ouster_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PCAP=ON

# 重新编译所有包确保依赖正确
colcon build
source install/setup.bash
```

> **注意**: Ouster ROS 驱动支持所有 FW v2.0 或更高版本的传感器 (OS0, OS1, OS2, OSDome)。更多详细信息请参考 [官方文档](https://github.com/ouster-lidar/ouster-ros/tree/ros2)。

## Configuration

### 1. 配置 ROS 话题

编辑 `ros2_ws/src/glim/config/config_ros.json` 文件，设置 ROS bag 中的话题名称和坐标框架：

```json
"imu_frame_id": "os_imu",
"lidar_frame_id": "os_lidar",
"imu_topic": "/ouster/imu",
"points_topic": "/ouster/points"
```

### 2. 设置 LiDAR-IMU 相对位置

LiDAR-IMU 相对位置信息可以从原始数据的 `.json` 文件中获取。在 `data/os_pcaps/` 目录下的 JSON 文件中找到 `imu_intrinsics.imu_to_sensor_transform` 参数。

**从原始 JSON 获取信息**：
```json
// 例如：data/os_pcaps/ouster_20250604072442.json
"imu_intrinsics": {
    "imu_to_sensor_transform": [
        1, 0, 0, 6.253,      // 4x4 变换矩阵
        0, 1, 0, -11.775,    // 单位：毫米 (mm)
        0, 0, 1, 7.645,
        0, 0, 0, 1
    ]
}
```

**转换为 GLIM 格式**：
编辑 `ros2_ws/src/glim/config/config_sensors.json` 文件，将变换信息转换为 GLIM 期望的格式：

```json
"T_lidar_imu": [0.006253, -0.011775, 0.007645, 0, 0, 0, 1]
```

> **重要转换规则**：
> 1. **单位转换**：原始数据单位为毫米 (mm)，需要除以 1000 转换为米 (m)
> 2. **格式转换**：从 4x4 变换矩阵转换为 `[x, y, z, qx, qy, qz, qw]` 格式
> 3. **坐标提取**：提取变换矩阵中的平移部分 (第4、8、12个元素) 作为 x、y、z 坐标

## 运行

启动 3D SLAM 系统，需要先启动glim，然后准备并播放数据。

### 1. 启动 GLIM 建图算法

```bash
cd ros2_ws
source install/setup.bash
ros2 run glim_ros glim_rosnode
```

### 2. 数据准备与播放

根据数据类型选择合适的播放方式：

#### 方式1: 直接播放现有 ROS2 bag 文件
```bash
ros2 bag play bags/official_ouster_for_glim
```

#### 方式2: 直接播放 PCAP 文件 (PCAP 回放模式)

根据 [Ouster 官方文档](https://github.com/ouster-lidar/ouster-ros/tree/ros2) 的 PCAP 回放模式：

```bash
# 室内数据 (循环播放)
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=data/os_pcaps/ouster_20250604072442.pcap \
    metadata:=data/os_pcaps/ouster_20250604072442.json \
    viz:=false loop:=true

# 室外数据 (循环播放)
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=data/os_pcaps/ouster_20250604074152.pcap \
    metadata:=data/os_pcaps/ouster_20250604074152.json \
    viz:=false loop:=true
```

> **说明**: 从包版本 8.1 开始，如果 bag 文件已包含 metadata 话题，则 metadata 参数为可选。

#### 方式3: 录制模式 - 预转换 PCAP 到 ROS2 bag (推荐用于多次使用)

使用 Ouster 官方提供的录制模式：

**步骤1**: 启动 PCAP 回放 (不循环)
```bash
ros2 launch ouster_ros replay_pcap.launch.xml \
    pcap_file:=data/os_pcaps/ouster_20250604072442.pcap \
    metadata:=data/os_pcaps/ouster_20250604072442.json \
    viz:=false loop:=false
```

**步骤2**: 录制为 ROS2 bag (在另一个终端)
```bash
ros2 bag record -o bags/my_recording /ouster/points /ouster/imu /tf_static
```
> **重要**: 必须录制 `/tf_static` 话题，否则 GLIM 会因为缺少坐标变换信息而报错。

**步骤3**: 播放转换好的 bag 文件
```bash
ros2 bag play bags/my_recording
```

> **提示**: 也可以直接使用 Ouster 官方的录制启动文件：
> ```bash
> ros2 launch ouster_ros record.launch.xml \
>     sensor_hostname:=<sensor_ip> \
>     bag_file:=bags/my_recording
> ```


## 项目结构

```
3D_SLAM/
├── ros2_ws/                  # ROS2 工作空间
│   ├── src/
│   │   ├── glim/            # 核心建图算法
│   │   ├── glim_ros2/       # ROS2 接口
│   │   └── ouster-ros/      # Ouster 驱动
│   └── bags/                # ROS2 bag 数据
└── data/       # 原始数据和转换工具
```

## 硬件

- **LiDAR**: Ouster OS1-128
- **IMU**: 内置 6 轴 IMU