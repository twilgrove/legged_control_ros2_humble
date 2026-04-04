cmake_minimum_required(VERSION 3.8)
project(dog_hardware)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 ")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(Threads REQUIRED)

# -----------------------------------------------------------------------------
# 1. 仿真驱动插件 (dog_gazebo_hw)
# -----------------------------------------------------------------------------
add_library(dog_gazebo_hw SHARED src/dog_gazebo_hw.cpp)
target_include_directories(
  dog_gazebo_hw PRIVATE include ${GAZEBO_INCLUDE_DIRS}
                        ${gazebo_ros2_control_INCLUDE_DIRS})
ament_target_dependencies(dog_gazebo_hw hardware_interface gazebo_ros2_control
                          gazebo_ros rclcpp pluginlib)
target_link_libraries(dog_gazebo_hw ${GAZEBO_LIBRARIES})

pluginlib_export_plugin_description_file(gazebo_ros2_control dog_hardware.xml)
# -----------------------------------------------------------------------------
# 1. 实物驱动插件 (J60_6_hw)
# -----------------------------------------------------------------------------
add_library(J60_6_hw SHARED src/J60_6/J60_6_hw.cpp src/IWT603/wit_c_sdk.c)

target_include_directories(J60_6_hw PRIVATE include)

ament_target_dependencies(J60_6_hw hardware_interface rclcpp pluginlib)

target_link_libraries(J60_6_hw Threads::Threads)

pluginlib_export_plugin_description_file(hardware_interface dog_hardware.xml)

# 安装规则
install(
  TARGETS dog_gazebo_hw J60_6_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES dog_hardware.xml DESTINATION share/dog_gazebo_hw)

ament_package()
