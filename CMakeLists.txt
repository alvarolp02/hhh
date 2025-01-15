cmake_minimum_required(VERSION 3.8)
project(h)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(can_interface_exec src/can_interface_node.cpp)
ament_target_dependencies(can_interface_exec rclcpp 
                            std_msgs)

add_executable(dist_control_exec src/dist_control_node.cpp)
ament_target_dependencies(dist_control_exec rclcpp 
                            std_msgs)

install(TARGETS
  can_interface_exec
  dist_control_exec
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
