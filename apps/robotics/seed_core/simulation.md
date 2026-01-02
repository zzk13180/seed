# Seed Core 仿真环境部署文档

本文档描述如何在 Ubuntu 20.04 系统上部署 Seed Core 仿真环境。

## 环境要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic
- **图形界面**: 需要桌面环境支持 Gazebo 仿真

## 1. 安装 ROS Noetic

使用 fishros 一键安装脚本：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

安装 Python 开发工具：

```bash
sudo apt install -y python3 python3-venv python3-pip python3-rosdep ninja-build
sudo -E rosdep init && rosdep update
```

## 2. 安装依赖包

```bash
# ROS 通信桥接
sudo apt install -y ros-noetic-rosbridge-suite

# 导航功能包
sudo apt install -y ros-noetic-navigation

# SLAM 工具
sudo apt install -y ros-noetic-slam-toolbox

# TEB 局部规划器
sudo apt install -y ros-noetic-teb-local-planner

# TurtleBot3 仿真环境（可选）
sudo apt install -y ros-noetic-turtlebot3*

# Gazebo 仿真（如果没有安装）
sudo apt install -y ros-noetic-gazebo-ros-pkgs
```

## 3. 配置工作空间

```bash
# 进入代码目录
cd ~/your-project-path

# 创建 catkin 工作空间
mkdir -p ~/catkin_ws/src

# 链接 seed_core 包
ln -s ~/your-project-path/apps/ros/seed_core ~/catkin_ws/src/seed_core

# 安装 ROS 依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

## 4. 编译

```bash
cd ~/catkin_ws

# 使用 ninja 加速编译（推荐）
catkin_make_isolated --use-ninja

# 或使用标准 catkin_make
catkin_make
```

## 5. 初始化数据目录

```bash
# 创建数据存储目录
mkdir -p ~/.seed/data
mkdir -p ~/.seed/maps

# 如果有默认地图，可以复制到地图目录
# cp -r /path/to/default/map ~/.seed/maps/default
```

## 6. 运行仿真

### 方式一：使用 TurtleBot3 仿真

```bash
# 终端 1: Source 工作空间并启动仿真
source ~/catkin_ws/devel_isolated/setup.bash
export TURTLEBOT3_MODEL=burger

# 启动 Gazebo 仿真环境
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

```bash
# 终端 2: 启动 Seed Core 服务
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch seed_core bringup.launch
```

### 方式二：使用 Seed Core 内置仿真

```bash
# 终端 1: 启动仿真环境
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch seed_core simulation.launch

# 终端 2: 启动核心服务
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch seed_core bringup.launch
```

## 7. 验证服务

### 检查服务是否启动

```bash
# 列出所有 seed_core 服务
rosservice list | grep seed_core
```

预期输出：

```
/seed_core/list_executables
/seed_core/list_maps
/seed_core/list_packages
/seed_core/load_map
/seed_core/node/info
/seed_core/node/kill
/seed_core/node/start
/seed_core/point_manager/add_point
/seed_core/point_manager/delete_point
/seed_core/point_manager/get_point
/seed_core/point_manager/list_points
/seed_core/read_pgm
/seed_core/read_text_file
/seed_core/roswtf
/seed_core/save_map
/seed_core/switch_to_mapping
/seed_core/switch_to_navigation
/seed_core/update_json
/seed_core/update_pgm
```

### 测试 WebSocket 连接

打开浏览器控制台，测试 WebSocket 连接：

```javascript
const ws = new WebSocket('ws://localhost:5001');
ws.onopen = () => console.log('Connected!');
ws.onerror = (e) => console.error('Error:', e);
```

## 8. 常见问题

### Q: Gazebo 启动黑屏

A: 检查显卡驱动是否正确安装，尝试：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Q: rosbridge 连接超时

A: 检查端口是否被占用：

```bash
lsof -i :5001
```

### Q: slam_toolbox 启动失败

A: 确保激光雷达话题 `/scan` 正在发布：

```bash
rostopic echo /scan -n 1
```

---

> **注意**: 仿真环境需要图形界面支持。在无图形环境（如服务器）上，请使用 xvfb 虚拟帧缓冲。
