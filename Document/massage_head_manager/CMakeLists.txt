cmake_minimum_required(VERSION 3.8)
project(massage_head_manager)

# ================================
# 基本设置
# ================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================================
# 依赖包
# ================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(license_manager REQUIRED)


   # sqlite3-+
find_package(PkgConfig REQUIRED)  
pkg_check_modules(SQLITE3 REQUIRED sqlcipher)
add_definitions(-DSQLITE_HAS_CODEC)
find_package(nlohmann_json REQUIRED)

include_directories(include)
include_directories(${robot_interfaces_INCLUDE_DIRS})


# ================================
# 1. 基类库
# ================================
add_library(massage_head_base src/massage_head_base.cpp)
target_include_directories(massage_head_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(massage_head_base rclcpp std_msgs robot_interfaces)

# ================================
# 2. 协议库
# ================================
add_library(massage_head_protocol 
  src/protocol_v1.cpp
  src/protocol_v2.cpp
)
target_include_directories(massage_head_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(massage_head_protocol rclcpp robot_interfaces)

# ================================
# 3. 管理节点（可执行文件）
# ================================

add_executable(massage_head_manager src/massage_head_manager.cpp src/database_manager.cpp)

target_include_directories(massage_head_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # SQlite3
  ${SQLITE3_INCLUDE_DIRS})

target_link_libraries(massage_head_manager
  massage_head_base
  massage_head_protocol
  nlohmann_json::nlohmann_json
  # sqlite3
  ${SQLITE3_LIBRARIES}
  )
ament_target_dependencies(massage_head_manager
  rclcpp std_msgs serial_driver io_context robot_interfaces license_manager)

# ================================
# 4. 串口节点
# ================================
# add_executable(massage_head_serial_node src/massage_head_serial_node.cpp)
# target_include_directories(massage_head_serial_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(massage_head_serial_node
#   massage_head_base
#   massage_head_protocol)
# ament_target_dependencies(massage_head_serial_node
#   rclcpp std_msgs serial_driver io_context)

# ================================
# 5. 安装配置
# ================================
install(TARGETS
  massage_head_base
  massage_head_protocol
  massage_head_manager
  # massage_head_serial_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# 安装数据库配置文件目录
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  OPTIONAL
)

# 安装 launch 文件（如果存在）
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch/
# )


install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_dependencies(rclcpp std_msgs serial_driver io_context robot_interfaces)
ament_export_libraries(massage_head_base massage_head_protocol)
ament_export_include_directories(include)

# ================================
# 6. 测试依赖（保留）
# ================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
