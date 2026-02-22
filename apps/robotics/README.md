# @seed/robotics

> ROS1 (Noetic) 机器人控制系统（Python + C++，独立于 pnpm workspace）

## AI 参考指引

- **此项目独立于 TypeScript 生态**：使用 ROS1 (catkin + Python)，不参与 pnpm workspace
- 仿真和硬件控制代码在 `seed_core/` 下
- `seed_core/simulation.md` 包含仿真环境搭建指南
- 如需新建 ROS 包，参考 `seed_core/CMakeLists.txt` 和 `package.xml` 的结构

## 技术栈

| 维度       | 技术                              |
| ---------- | --------------------------------- |
| 机器人框架 | ROS1 (Noetic)                     |
| 语言       | Python + C++                      |
| 构建       | CMake (catkin)                    |
| 依赖管理   | pyproject.toml + requirements.txt |

## 目录结构

```
seed_core/
├── CMakeLists.txt      # ROS 包构建配置（catkin）
├── config/             # 机器人参数配置
├── launch/             # ROS launch 文件
├── scripts/            # 运行脚本
├── simulation.md       # 仿真环境文档
├── src/                # Python + C++ 源码
├── srv/                # ROS 服务定义
└── public/             # 静态资源（Web 控制面板）
```

## 注意

- 不在 `pnpm-workspace.yaml` 中，Nx 仅管理其 project.json
- 无 Nx targets 定义，构建和运行通过 catkin 工具链
