cmake_minimum_required(VERSION 3.8)
project(arm_collision_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_driver REQUIRED)

add_executable(collision_calib_node
  src/collision_calib_node.cpp
  src/arm_joint.cpp
  src/robot_arm.cpp
)
target_include_directories(collision_calib_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(collision_calib_node PRIVATE
  rclcpp::rclcpp
  robot_driver::robot_driver_pd42
)

install(TARGETS collision_calib_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
