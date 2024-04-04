cmake_minimum_required(VERSION 3.8)
project(schunk_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(schunk_interface REQUIRED)

add_executable(schunk_gripper_node src/schunk_node.cpp src/schunk_eip_interface.cpp)
ament_target_dependencies(schunk_gripper_node rclcpp std_msgs schunk_interface)

# Adding EIPScanner library
include_directories(/usr/local/include/EIPScanner)
target_link_libraries(schunk_gripper_node EIPScanner)


install(TARGETS
  schunk_gripper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
