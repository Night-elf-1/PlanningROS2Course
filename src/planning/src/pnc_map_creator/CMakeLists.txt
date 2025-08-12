# 外层cmakelists文件
cmake_minimum_required(VERSION 3.20)
project(pnc_map_straight)

# 设置依赖
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "base_msgs"
)

# 添加动态链接库
add_library(pnc_map_creator SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_straight/pnc_map_creator_straight.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_s_turn/pnc_map_creator_sturn.cpp
)

# 添加节点
add_executable(pnc_map_server
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_server.cpp
) # 添加server的节点 因为只有server才需要编译成节点

set(project_targets
  pnc_map_creator
  pnc_map_server
)

# 添加依赖 使用foreach
foreach(targets ${project_targets})
  ament_target_dependencies(${targets}
    ${DEPENDS}
  )
endforeach()
# 添加自己写的依赖
target_link_libraries(pnc_map_creator
  config_reader
)
target_link_libraries(pnc_map_server
  pnc_map_creator
)

target_link_libraries(pnc_map_server
  config_reader
  pnc_map_creator
)

# 安装动态库、节点和相关目录
install(TARGETS
  pnc_map_creator
  LIBRARY DESTINATION lib
)
install(TARGETS
  pnc_map_server
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)