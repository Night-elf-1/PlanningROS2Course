# 外层cmakelists文件
cmake_minimum_required(VERSION 3.20)
project(planning_process)

# 设置变量 依赖
set(DEPENDS
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2"
  "base_msgs"
)

# 添加节点   # ${CMAKE_CURRENT_SOURCE_DIR}获取到当前功能包目录
add_executable(planning_node 
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process.cpp
)
# 设置一个变量来代替planning_node
set(target planning_node)
# 添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)
target_link_libraries(${target}
  config_reader
  decision_center
  vehicle_info
  reference_line
  local_planner
)

# 安装动态库、节点和相关目录
install(TARGETS
  ${target}
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)