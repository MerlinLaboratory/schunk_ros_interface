cmake_minimum_required(VERSION 3.8)
project(schunk_ros_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_executable(my_node src/my_node.cpp)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(my_node ${Boost_LIBRARIES})

install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})

# add_subdirectory("CIPster/CIPster/source")

ament_package()
