# =============================================================================
# Seed Core CMakeLists.txt
# =============================================================================
# 
# 项目描述:
#   Seed Core 是一个 ROS 机器人控制核心包，提供:
#   - 节点管理服务
#   - 地图管理服务
#   - 点位管理服务
#   - 文件操作服务
#   - TF 话题整合
#
# 构建说明:
#   cd ~/catkin_ws
#   catkin_make -DCATKIN_WHITELIST_PACKAGES="seed_core"
#
# =============================================================================

# -----------------------------------------------------------------------------
# CMake 版本要求
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0.2)

# -----------------------------------------------------------------------------
# 项目名称
# -----------------------------------------------------------------------------
project(seed_core)

# -----------------------------------------------------------------------------
# 查找 catkin 及依赖包
# -----------------------------------------------------------------------------
# 说明:
#   - rospy: ROS Python 客户端库
#   - std_msgs: 标准消息类型
#   - geometry_msgs: 几何相关消息类型
#   - actionlib_msgs: 动作库消息类型
#   - move_base_msgs: 导航相关消息类型
#   - message_generation: 消息生成工具
#   - dynamic_reconfigure: 动态参数配置
# -----------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    actionlib_msgs
    move_base_msgs
    nav_msgs
    sensor_msgs
    tf
    tf2_ros
    message_generation
    dynamic_reconfigure
)

# -----------------------------------------------------------------------------
# 服务文件定义
# -----------------------------------------------------------------------------
# 说明:
#   定义本包提供的所有 ROS 服务接口
#   每个 .srv 文件定义了请求和响应的消息结构
# -----------------------------------------------------------------------------
add_service_files(
    FILES
    # --------------------- 节点管理服务 ---------------------
    ManageNode.srv          # 节点启动/停止/信息查询
    ListPackages.srv        # 列出所有 ROS 包
    ListExecutables.srv     # 列出包中的可执行文件
    
    # --------------------- 地图管理服务 ---------------------
    LoadMap.srv             # 加载地图
    SaveMap.srv             # 保存地图
    ListMaps.srv            # 列出可用地图
    
    # --------------------- 文件操作服务 ---------------------
    ReadPgm.srv             # 读取 PGM 地图文件
    UpdatePgm.srv           # 更新 PGM 地图文件
    UpdateJson.srv          # 更新 JSON 配置文件
    ReadTextFile.srv        # 读取文本文件
    
    # --------------------- 点位管理服务 ---------------------
    AddPoint.srv            # 添加导航点位
    DeletePoint.srv         # 删除点位
    GetPoint.srv            # 获取点位信息
    ListPoints.srv          # 列出点位
    
    # --------------------- 高级功能服务 ---------------------
    GetNodeParameters.srv   # 获取节点参数
    RecordRosbag.srv        # 录制 Rosbag
)

# -----------------------------------------------------------------------------
# 生成消息和服务
# -----------------------------------------------------------------------------
# 说明:
#   根据上面定义的消息和服务文件生成对应的代码
#   DEPENDENCIES 指定依赖的消息包
# -----------------------------------------------------------------------------
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

# -----------------------------------------------------------------------------
# catkin 包配置
# -----------------------------------------------------------------------------
# 说明:
#   声明本包的依赖关系，供其他包使用
# -----------------------------------------------------------------------------
catkin_package(
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        actionlib_msgs
        move_base_msgs
        nav_msgs
        sensor_msgs
        tf
        tf2_ros
        message_runtime
        dynamic_reconfigure
)

# -----------------------------------------------------------------------------
# 包含目录
# -----------------------------------------------------------------------------
include_directories(
    ${catkin_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------
# 安装 Python 脚本
# -----------------------------------------------------------------------------
# 说明:
#   查找 scripts 目录下所有 Python 脚本并安装
#   安装后的脚本可通过 rosrun seed_core <script_name>.py 运行
# -----------------------------------------------------------------------------
file(
    GLOB PYTHON_SCRIPTS_DIR "scripts/*.py"
)

catkin_install_python(
    PROGRAMS ${PYTHON_SCRIPTS_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# -----------------------------------------------------------------------------
# 安装其他资源文件
# -----------------------------------------------------------------------------
# 说明:
#   安装 launch 文件、配置文件和其他资源
# -----------------------------------------------------------------------------
install(
    DIRECTORY 
        launch
        config
        public
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".gitkeep" EXCLUDE
)

# -----------------------------------------------------------------------------
# C++ 可执行文件
# -----------------------------------------------------------------------------
# 说明:
#   编译 src 目录下的 C++ 源文件为可执行节点
#   - angle_controller_node: 角度控制器
#   - global_relocation_node: 全局重定位
#   - robot_map_pose_node: 机器人位姿发布
# -----------------------------------------------------------------------------

# 角度控制器节点
add_executable(angle_controller_node src/angle_controller.cpp)
add_dependencies(angle_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angle_controller_node ${catkin_LIBRARIES})

# 全局重定位节点
add_executable(global_relocation_node src/global_relocation.cpp)
add_dependencies(global_relocation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_relocation_node ${catkin_LIBRARIES})

# 机器人位姿发布节点
add_executable(robot_map_pose_node src/robot_map_pose.cpp)
add_dependencies(robot_map_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_map_pose_node ${catkin_LIBRARIES})

# 安装 C++ 可执行文件
install(
    TARGETS
        angle_controller_node
        global_relocation_node
        robot_map_pose_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
