# 多轴车辆自动驾驶系统

Autonomous Driving System for Multi-Axle Vehicles

基于 **ROS** 的三轴多轴车辆自动驾驶系统,集成了 **感知 → 定位 → 规划 → 控制 → 底盘执行** 的完整自动驾驶功能栈,用于三轴六轮车辆(前、中、后三轴均可转向)的自主行驶。

---

## 系统架构

```
┌───────────────────────────────────────────────────────────────────────────┐
│                                传感器层                                    │
│   Robosense 激光雷达  ──►  rs_to_velodyne  ──►  ASENSING 组合惯导 (INS)     │
└──────────┬──────────────────────────────┬──────────────────┬───────────────┘
           │ /velodyne_points             │                  │ /IMU
           ▼                              ▼                  ▼
   ┌────────────────┐            ┌──────────────┐     ┌──────────────┐
   │ lidarObstac    │            │  FAST_LIO    │◄────┤  INS 驱动    │
   │ 障碍物检测与跟踪 │            │ 激光-惯性里程计│     └──────────────┘
   └───────┬────────┘            └──────┬───────┘
           │ /detection/.../objects     │ /state_to_control
           ▼                            ▼
   ┌───────────────────────────────────────────────────┐
   │                  Local_path (Hybrid A*)            │
   │     局部栅格地图 + 全局路径 + 车辆状态 ──► 局部路径    │
   └───────────────────────────┬───────────────────────┘
                               │ /local_path
                               ▼
   ┌───────────────────────────────────────────────────┐
   │          PurePursuit_control (纯跟踪控制)           │
   │   预瞄模型 + Pure Pursuit + 三轴阿克曼转向分配        │
   └───────────────────────────┬───────────────────────┘
                               │ /decision/steering_angle
                               ▼
   ┌───────────────────────────────────────────────────┐
   │                  can_ros (CAN 通信)                 │
   │              前/中/后轴转角、油门 ──► 底盘执行         │
   └───────────────────────────────────────────────────┘
```

**数据流:** 雷达点云与惯导数据经 FAST_LIO 融合输出车辆位姿;障碍物检测输出目标列表;二者结合全局路径,由 Hybrid A* 规划出满足车辆运动学约束的局部路径;纯跟踪控制器据此计算前轮转角,并通过三轴阿克曼转向几何分配到三个转向轴,最后经 CAN 总线下发底盘执行。

---

## 模块说明

| 模块 | 功能 | 核心算法 / 技术 |
|------|------|----------------|
| **FAST_LIO** | 激光-惯性里程计 / 建图 | 迭代扩展卡尔曼滤波 (IEKF) + ikd-Tree 增量建图 |
| **INS** | ASENSING 组合惯导驱动 | 串口协议解析,输出位置、姿态、质心侧偏角、横摆角速度 |
| **rs_to_velodyne** | 点云格式转换 | Robosense 点云 → Velodyne 格式 (XYZIRT / XYZI) |
| **lidarObstac** | 障碍物检测与跟踪 | Patchwork 同心区域地面分割 + 欧几里得聚类 + 包围盒拟合 + 匈牙利匹配 / 卡尔曼滤波跟踪 |
| **Local_path** | 局部路径规划 | Hybrid A* 搜索 + Reeds-Shepp 曲线解析扩展 |
| **PurePursuit_control** | 路径跟踪控制 | 预瞄 (Preview) 模型 + Pure Pursuit 纯跟踪 + 三轴阿克曼转向分配 |
| **can_ros** | CAN 总线通信 | Kvaser CANlib 驱动,发送转角 / 油门指令,读取底盘反馈 |
| **rviz** | 可视化配置 | 车辆模型、安全区域、局部路径、搜索树显示 |

---

## 硬件需求

- **车辆平台:** 三轴六轮车辆,前 / 中 / 后三轴均可转向(三轴阿克曼转向几何)
- **激光雷达:** Robosense 系列 (RS-16 / RS-32 / RS-Ruby / RS-Helios 等)
- **组合惯导:** ASENSING 系列(串口连接,默认 `/dev/ttyUSB0`)
- **CAN 总线:** Kvaser CAN 适配器(依赖 Kvaser CANlib)

---

## 软件依赖

| 依赖 | 版本要求 | 用途 |
|------|----------|------|
| Ubuntu | 16.04 / 18.04 | 系统 |
| ROS | Melodic / Noetic | 通信框架 |
| PCL | >= 1.8 | 点云处理 |
| Eigen | >= 3.3.4 | 矩阵运算 |
| autoware_msgs | - | `DetectedObjectArray` 等消息类型 |
| rslidar_sdk | - | Robosense 雷达驱动 |
| livox_ros_driver | - | FAST_LIO 依赖(需先 source) |
| Kvaser CANlib | - | CAN 通信 |

---

## 快速开始

### 1. 编译

将本仓库中的各 ROS 包放入 catkin 工作空间后编译:

```bash
cd ~/catkin_ws/src
# 将本仓库的 FAST_LIO-main、INS、Local_path、PurePursuit_control、can_ros、lidarObstac、rs_to_velodyne 复制到此目录
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> FAST_LIO 需要先安装并 source `livox_ros_driver`(即使使用 Velodyne 格式点云),并执行
> `git submodule update --init` 拉取 ikd-Tree 子模块。

### 2. 运行

按以下顺序启动各节点:

```bash
# ① 雷达驱动 + 点云格式转换
roslaunch rslidar_sdk start.launch
rosrun rs_to_velodyne rs_to_velodyne XYZIRT XYZIRT

# ② 组合惯导
roslaunch ins demo.launch    # 确认串口号 /dev/ttyUSB0

# ③ 激光-惯性里程计(定位)
roslaunch fast_lio mapping_velodyne.launch

# ④ 障碍物检测
roslaunch lidar_detection_track lidar_detection_track.launch

# ⑤ 局部路径规划(Hybrid A*)
rosrun hybrid_astar hybrid_astar

# ⑥ 纯跟踪控制
rosrun PurePursuit_control PurePursuit_control_node

# ⑦ CAN 底盘通信
rosrun can_ros can_all
```

---

## 核心算法说明

### Hybrid A* 局部路径规划 (`Local_path`)

在 **位置 + 航向角** 的连续状态空间上搜索,通过车辆运动学模型生成可行节点:

- **状态空间搜索:** 以车辆运动学模型 (`DynamicModel`) 扩展节点,转向角、路径段长度均离散化
- **代价设计:** 转向惩罚 `steering_penalty`、倒车惩罚 `reversing_penalty`、转向变化惩罚 `steering_change_penalty`
- **Reeds-Shepp 解析扩展:** 接近终点时直接生成满足运动学约束的 RS 曲线,显著加速收敛
- **碰撞检测:** 射线检测 + 车辆矩形轮廓离散化,可配置搜索树可视化 (`/searched_tree`)

### 纯跟踪控制 (`PurePursuit_control`)

基于前视点 (look-ahead point) 的经典路径跟踪算法,前轮转角由下式计算:

```
δ = atan2(2 · L · e_y, L_d²)
```

其中 `L` 为轴距,`L_d` 为前视距离,`e_y` 为横向偏差。控制器同时结合:

- **预瞄模型** (`Preview_error`):根据车速动态计算前视距离,获取理想横摆角速度与理想质心侧偏角
- **三轴阿克曼转向分配** (`Akm_get_delta`):由前轴转角推导中 / 后轴转角,满足多轴转向几何约束

### 障碍物检测 (`lidarObstac`)

- **Patchwork 地面分割:** 将点云按同心扇形分区,逐区域拟合地平面并自适应阈值,鲁棒地分离地面与非地面点
- **欧几里得聚类 + 包围盒:** 对非地面点聚类,生成障碍物包围盒
- **多目标跟踪:** 匈牙利算法数据关联 + 卡尔曼滤波,输出稳定的目标轨迹

---

## ROS 话题接口

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/rslidar_points` | `sensor_msgs/PointCloud2` | 输入 | Robosense 原始点云 |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | 输出 | Velodyne 格式点云 |
| `/state_to_control` | `fast_lio::Position_state` | 输出 | 定位状态(位置 / 姿态 / 质心侧偏角 / 横摆角速度) |
| `/detection/lidar_detector/objects` | `autoware_msgs::DetectedObjectArray` | 输出 | 障碍物目标列表 |
| `/local_path` | `nav_msgs/Path` | 输出 | 局部规划路径 |
| `/local_map` | `nav_msgs/OccupancyGrid` | 输出 | 局部栅格地图(障碍物投影) |
| `/global_path` | `nav_msgs/Path` | 输出 | 全局参考路径(由 txt 文件加载) |
| `/searched_tree` | `visualization_msgs/Marker` | 输出 | Hybrid A* 搜索树可视化 |
| `/decision/steering_angle` | `can_ros::send_can` | 输出 | 前 / 中 / 后轴转角、油门指令 |
| `/car_cube` | `visualization_msgs/Marker` | 输出 | 车辆模型可视化 |

> 自定义消息: `fast_lio::Position_state`、`can_ros::send_can` / `can_ros::read_can`。

---

## 目录结构

```
Autonomous-driving-of-multi-axle-vehicles-main
├── FAST_LIO-main/            # 激光-惯性里程计与建图
├── INS/                      # ASENSING 组合惯导驱动
├── Local_path/               # Hybrid A* 局部路径规划
│   └── src/hybrid_astar/     #   ├── hybrid_astar_searcher(搜索器)
│                            #   └── rs_path(Reeds-Shepp 曲线)
├── PurePursuit_control/      # 纯跟踪控制
│   └── src/PurePursuit_control/
│                            #   ├── purepursuit_controler(纯跟踪控制器)
│                            #   └── Preview_error(预瞄模型)
├── lidarObstac/              # 激光雷达障碍物检测与跟踪
│   └── src/lidarObstacleDetect-main/
│                            #   ├── ground_detector/patchwork
│                            #   ├── cluster/euclideanCluster
│                            #   └── bounding_box
├── can_ros/                  # CAN 总线通信
├── rs_to_velodyne/           # Robosense → Velodyne 点云转换
└── rviz/                     # rviz 可视化配置
```

---

## 注意事项

- 局部路径规划与全局路径加载的**文件路径为硬编码**(默认读取 `/home/kim/Documents/.../path/lla_path.txt` 或 `easy_path.txt`),部署时请按实际环境修改 [hybrid_astar_main.cpp](Local_path/src/hybrid_astar/src/hybrid_astar_main.cpp)。
- 控制节点运行于 **10 Hz** 控制周期,转角指令限幅为 ±20°。
- 实车运行前请确认 CAN 设备权限与底盘协议定义一致。

---

## 许可证

本项目各模块开源协议不尽相同:

- **FAST_LIO** 为第三方开源项目,遵循其原始许可证(见其仓库 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO))
- **lidarObstac**、**rs_to_velodyne** 等亦为第三方开源项目,遵循其各自原始许可证
- **本仓库其余自有代码**的许可证请自行选择后补充声明(当前 `package.xml` 中 `license` 字段为占位符)

> ⚠️ 发布前请务必核查并填写各 `package.xml` 的 `maintainer` 与 `license` 字段。

---

## 致谢

本系统集成了以下优秀开源项目:

- [FAST_LIO](https://github.com/hku-mars/FAST_LIO) — 快速激光-惯性里程计
- [lidarObstacleDetect](https://github.com/ORiN-Group/lidarObstacleDetect) — 激光雷达障碍物检测与跟踪
- [rs_to_velodyne](https://github.com/HViktorTsoi/rs_to_velodyne) — 点云格式转换工具
- [ASENSING INS Driver](https://github.com/ASENSING/ASENSING_INS_Driver) — 组合惯导驱动
