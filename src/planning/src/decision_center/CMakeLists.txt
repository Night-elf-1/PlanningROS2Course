# 外层cmakelists文件
cmake_minimum_required(VERSION 3.20)
project(decision_center)

# 设置依赖
set(DEPENDS
  "rclcpp"
)

# 添加动态链接库
add_library(decision_center SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/decision_center.cpp
)
set(decision_target
  decision_center
)

# 添加依赖
ament_target_dependencies(${decision_target}
  ${DEPENDS}
)
target_link_libraries(${decision_target}
  config_reader
  vehicle_info
)

# 安装动态库、节点和相关目录
install(TARGETS
  ${decision_target}
  LIBRARY DESTINATION lib
)