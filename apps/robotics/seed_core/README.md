# Seed Core

> ROS 机器人控制核心包 - 提供节点管理、地图管理、点位管理和文件操作等服务

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)](https://www.python.org/)

## 📋 目录

- [功能概述](#功能概述)
- [环境要求](#环境要求)
- [安装部署](#安装部署)
- [快速开始](#快速开始)
- [架构设计](#架构设计)
- [服务接口](#服务接口)
- [配置说明](#配置说明)
- [目录结构](#目录结构)
- [开发指南](#开发指南)
- [常见问题](#常见问题)

## 功能概述

Seed Core 是一个功能完整的 ROS 机器人控制核心包，为机器人导航系统提供基础服务支持。

### 主要功能

| 模块 | 功能 | 说明 |
|------|------|------|
| **服务处理器** | 节点管理 | 启动/停止/查询 ROS 节点和 launch 文件 |
| | 地图管理 | 加载/保存地图，切换导航/建图模式 |
| | 文件操作 | 读写 PGM、JSON、文本文件 |
| **TF 整合器** | TF 消息合并 | 将分散的 TF 消息整合为统一话题 |
| | 超时清理 | 自动清理过期的 TF 变换 |
| **点位管理器** | 点位 CRUD | 导航点位的增删改查 |
| | SQLite 持久化 | 使用 SQLite 数据库存储点位数据 |
| **角度控制器** | PID 控制 | 控制机器人旋转到指定角度 (C++) |
| **全局重定位** | 位姿搜索 | 在地图中搜索最佳机器人位置 (C++) |
| **位姿发布器** | TF 监听 | 发布机器人在地图中的实时位姿 (C++) |

## 环境要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic
- **Python**: 3.8+

### 依赖软件

```bash
# ROS 核心包
ros-noetic-rosbridge-suite
ros-noetic-navigation
ros-noetic-slam-toolbox
ros-noetic-teb-local-planner

# Python 依赖
python3-numpy
python3-pil
```

## 安装部署

### 1. 安装 ROS Noetic

```bash
# 使用 fishros 一键安装脚本
wget http://fishros.com/install -O fishros && . fishros

# 安装开发工具
sudo apt install -y python3 python3-venv python3-pip python3-rosdep ninja-build
sudo -E rosdep init && rosdep update
```

### 2. 安装依赖包

```bash
sudo apt install -y ros-noetic-rosbridge-suite
sudo apt install -y ros-noetic-navigation
sudo apt install -y ros-noetic-slam-toolbox
sudo apt install -y ros-noetic-teb-local-planner
```

### 3. 配置工作空间

```bash
# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 链接 seed_core 包
ln -s /path/to/seed_core ~/catkin_ws/src/seed_core

# 安装 ROS 依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

### 4. 编译

```bash
cd ~/catkin_ws

# 使用 catkin_make
catkin_make

# 或使用 catkin_make_isolated（推荐）
catkin_make_isolated --use-ninja
```

### 5. 初始化数据目录

```bash
# 创建点位数据库目录
mkdir -p ~/.seed/data

# 创建地图存储目录
mkdir -p ~/.seed/maps
```

## 快速开始

### 启动核心服务

```bash
# Source 工作空间
source ~/catkin_ws/devel/setup.bash

# 启动主服务（包含 rosbridge、TF 整合、服务处理器、点位管理）
roslaunch seed_core bringup.launch

# 可选：指定 WebSocket 端口
roslaunch seed_core bringup.launch port_rosbridge:=5002
```

### 启动导航模式

```bash
roslaunch seed_core navigation.launch
```

### 启动建图模式

```bash
roslaunch seed_core mapping.launch
```

### 仿真环境

```bash
# 需要安装 TurtleBot3 仿真包
export TURTLEBOT3_MODEL=burger
roslaunch seed_core simulation.launch
```

## 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                      前端 Web 应用                           │
└─────────────────────────┬───────────────────────────────────┘
                          │ WebSocket (port 5001)
┌─────────────────────────▼───────────────────────────────────┐
│                   rosbridge_websocket                        │
└─────────────────────────┬───────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
┌───────────────┐ ┌───────────────┐ ┌───────────────┐
│ service_handler│ │ topic_handler │ │ point_manager │
│               │ │               │ │               │
│ - 节点管理     │ │ - TF 整合     │ │ - 点位 CRUD   │
│ - 地图管理     │ │ - 超时清理    │ │ - SQLite 存储 │
│ - 文件操作     │ │               │ │               │
└───────┬───────┘ └───────────────┘ └───────────────┘
        │
        ▼
┌─────────────────────────────────────────────────────────────┐
│                    ROS Navigation Stack                      │
│  ┌─────────┐  ┌─────────────┐  ┌───────────────────────┐    │
│  │move_base│  │slam_toolbox │  │ teb_local_planner     │    │
│  └─────────┘  └─────────────┘  └───────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## 服务接口

### 节点管理服务

| 服务名称 | 类型 | 功能 |
|---------|------|------|
| `seed_core/node/start` | ManageNode | 启动节点或 launch 文件 |
| `seed_core/node/kill` | ManageNode | 终止节点或 launch 文件 |
| `seed_core/node/info` | ManageNode | 获取节点信息 |
| `seed_core/list_packages` | ListPackages | 列出所有 ROS 包 |
| `seed_core/list_executables` | ListExecutables | 列出包中的可执行文件 |
| `seed_core/roswtf` | Trigger | 运行 roswtf 诊断 |

### 地图管理服务

| 服务名称 | 类型 | 功能 |
|---------|------|------|
| `seed_core/load_map` | LoadMap | 加载地图文件 |
| `seed_core/save_map` | SaveMap | 保存当前地图 |
| `seed_core/list_maps` | ListMaps | 列出可用地图 |
| `seed_core/switch_to_navigation` | Trigger | 切换到导航模式 |
| `seed_core/switch_to_mapping` | Trigger | 切换到建图模式 |

### 文件操作服务

| 服务名称 | 类型 | 功能 |
|---------|------|------|
| `seed_core/read_pgm` | ReadPgm | 读取 PGM 地图文件 |
| `seed_core/update_pgm` | UpdatePgm | 更新 PGM 地图文件 |
| `seed_core/update_json` | UpdateJson | 更新 JSON 配置文件 |
| `seed_core/read_text_file` | ReadTextFile | 读取文本文件 |

### 点位管理服务

| 服务名称 | 类型 | 功能 |
|---------|------|------|
| `seed_core/point_manager/add_point` | AddPoint | 添加导航点位 |
| `seed_core/point_manager/delete_point` | DeletePoint | 删除点位 |
| `seed_core/point_manager/get_point` | GetPoint | 获取点位信息 |
| `seed_core/point_manager/list_points` | ListPoints | 列出点位列表 |

### 服务调用示例

```bash
# 启动导航 launch 文件
rosservice call /seed_core/node/start "node: 'roslaunch seed_core navigation.launch'"

# 保存地图
rosservice call /seed_core/save_map "{file_path: '~/.seed/maps/my_map', topic: '/map'}"

# 添加导航点位（注意：name 和 map_name 需要 Base64 编码）
rosservice call /seed_core/point_manager/add_point "{
  name: 'd29ya3N0YXRpb24x',
  point_type: 'navigation',
  x: 1.0, y: 2.0, z: 0.0,
  orientation_x: 0.0, orientation_y: 0.0, orientation_z: 0.0, orientation_w: 1.0,
  map_name: 'ZGVmYXVsdA==',
  description: '',
  metadata: '{}',
  tags: '[]',
  enabled: true
}"

# 列出所有点位
rosservice call /seed_core/point_manager/list_points "{
  point_type: 'navigation',
  map_name: '',
  enabled_only: true
}"
```

## 配置说明

### Launch 文件参数

**bringup.launch**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port_rosbridge` | 5001 | WebSocket 服务端口 |
| `retry_startup_delay` | 10 | 启动失败重试延迟(秒) |
| `fragment_timeout` | 30 | 消息分片超时(秒) |
| `database_dir` | ~/.seed/data | 点位数据库目录 |

### 配置文件

配置文件位于 `config/` 目录：

- `simulation.yaml` - 仿真环境配置
- `costmap/*.yaml` - 代价地图配置
- `navigation/*.yaml` - 导航参数配置
- `slam_toolbox/*.yaml` - SLAM 参数配置

## 目录结构

```
seed_core/
├── CMakeLists.txt          # CMake 构建配置
├── package.xml             # ROS 包描述文件
├── README.md               # 本文档
├── simulation.md           # 仿真部署文档
├── pyproject.toml          # Python 项目配置
├── requirements.txt        # Python 依赖
├── requirements-dev.txt    # 开发依赖
│
├── config/                 # 配置文件目录
│   ├── simulation.yaml     # 仿真配置
│   ├── costmap/            # 代价地图配置
│   ├── navigation/         # 导航参数
│   └── slam_toolbox/       # SLAM 配置
│
├── launch/                 # Launch 文件目录
│   ├── bringup.launch      # 主启动文件
│   ├── mapping.launch      # 建图模式
│   ├── navigation.launch   # 导航模式
│   ├── simulation.launch   # 仿真环境
│   ├── modules/            # 功能模块 launch
│   └── planners/           # 规划器 launch
│
├── scripts/                # Python 脚本
│   ├── __init__.py
│   ├── service_handler.py  # 服务处理器
│   ├── topic_handler.py    # TF 话题处理器
│   └── point_manager.py    # 点位管理器
│
├── src/                    # C++ 源代码
│   ├── angle_controller.cpp    # 角度控制器节点
│   ├── global_relocation.cpp   # 全局重定位节点
│   └── robot_map_pose.cpp      # 机器人位姿发布节点
│
├── srv/                    # 服务定义文件
│   ├── AddPoint.srv
│   ├── DeletePoint.srv
│   ├── GetPoint.srv
│   ├── ListPoints.srv
│   ├── LoadMap.srv
│   ├── SaveMap.srv
│   └── ...
│
└── public/                 # 公共资源
    └── simulation/         # 仿真资源
        ├── models/         # Gazebo 模型
        └── worlds/         # Gazebo 世界文件
```

## 开发指南

### 代码规范

本项目遵循以下规范：

- Python 代码遵循 PEP 8 规范
- 使用 Black 进行代码格式化
- 使用 isort 排序导入
- 使用 Pylint 进行静态检查

### 运行测试

```bash
# 安装开发依赖
pip install -r requirements-dev.txt

# 运行 lint 检查
pylint scripts/

# 运行类型检查
mypy scripts/
```

### 添加新服务

1. 在 `srv/` 目录创建 `.srv` 文件
2. 更新 `CMakeLists.txt` 添加新的 srv 文件
3. 重新编译包
4. 在对应的处理器中实现服务回调

## 常见问题

### Q: rosbridge 连接失败？

A: 检查以下项：
- 确认端口 5001 未被占用
- 检查防火墙设置
- 查看节点日志：`rosnode info /seed_core_rosbridge`

### Q: 地图加载失败？

A: 检查以下项：
- 确认地图文件路径正确
- 检查 map.yaml 和 map.pgm 文件是否存在
- 确认文件权限

### Q: 点位数据库错误？

A: 尝试以下操作：
- 确认数据库目录存在且可写
- 删除损坏的数据库文件重新创建
- 检查 SQLite 版本兼容性

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

## 贡献

欢迎提交 Issue 和 Pull Request！

---

**Seed Core** - Making robot navigation easier 🤖
