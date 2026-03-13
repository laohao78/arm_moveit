cmake_minimum_required(VERSION 3.0.2)
project(arm_serial_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  serial  # ← 添加这一行！
  actionlib
)

# 消息定义
add_message_files(
  FILES
  ArmSerialCommand.msg
  ArmSerialFeedback.msg
)

# ✅ 修复1：在 generate_messages 中声明本包名（重要！）
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  # ✅ 修复2：CATKIN_DEPENDS 中加上 message_runtime（你已有），但确保顺序
  CATKIN_DEPENDS
    roscpp std_msgs sensor_msgs message_runtime serial  # ← 添加 serial
  # 可选：显式导出 include 路径（通常不需要）
)

# 编译选项
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 可执行文件
add_executable(arm_serial_driver_node
  src/arm_serial_driver_node.cpp
  src/arm_serial_driver.cpp
)
add_executable(moveit_to_serial_bridge src/moveit_to_serial_bridge.cpp)


# ✅ 修复3（关键）：让可执行文件依赖消息目标（确保先生成 .h）
add_dependencies(arm_serial_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(arm_serial_driver_node
  ${catkin_LIBRARIES}
)
target_link_libraries(moveit_to_serial_bridge ${catkin_LIBRARIES})

# 安装（可选）
install(TARGETS arm_serial_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)