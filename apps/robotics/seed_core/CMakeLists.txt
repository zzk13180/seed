# Seed Core CMakeLists.txt
# 构建: cd ~/catkin_ws && catkin_make -DCATKIN_WHITELIST_PACKAGES="seed_core"

cmake_minimum_required(VERSION 3.0.2)
project(seed_core)

# --- 依赖 ---
find_package(catkin REQUIRED COMPONENTS
    roscpp rospy
    std_msgs geometry_msgs actionlib_msgs move_base_msgs nav_msgs sensor_msgs
    tf tf2_ros
    message_generation dynamic_reconfigure
)

# --- 服务定义 ---
add_service_files(FILES
    ManageNode.srv ListPackages.srv ListExecutables.srv     # 节点管理
    LoadMap.srv SaveMap.srv ListMaps.srv                     # 地图管理
    ReadPgm.srv UpdatePgm.srv UpdateJson.srv ReadTextFile.srv  # 文件操作
    AddPoint.srv DeletePoint.srv GetPoint.srv ListPoints.srv   # 点位管理
    GetNodeParameters.srv RecordRosbag.srv                     # 高级功能
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# --- catkin 包 ---
catkin_package(CATKIN_DEPENDS
    roscpp rospy
    std_msgs geometry_msgs actionlib_msgs move_base_msgs nav_msgs sensor_msgs
    tf tf2_ros
    message_runtime dynamic_reconfigure
)

include_directories(${catkin_INCLUDE_DIRS})

# --- Python 脚本安装 ---
file(GLOB PYTHON_SCRIPTS_DIR "scripts/*.py")
catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS_DIR}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# --- 资源文件安装 ---
install(DIRECTORY launch config public
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN ".gitkeep" EXCLUDE
)

# --- C++ 节点 ---
# angle_controller_node  — 角度控制器
add_executable(angle_controller_node src/angle_controller.cpp)
add_dependencies(angle_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angle_controller_node ${catkin_LIBRARIES})

# global_relocation_node — 全局重定位
add_executable(global_relocation_node src/global_relocation.cpp)
add_dependencies(global_relocation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_relocation_node ${catkin_LIBRARIES})

# robot_map_pose_node    — 机器人位姿发布
add_executable(robot_map_pose_node src/robot_map_pose.cpp)
add_dependencies(robot_map_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_map_pose_node ${catkin_LIBRARIES})

install(TARGETS angle_controller_node global_relocation_node robot_map_pose_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
