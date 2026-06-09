cmake_minimum_required(VERSION 3.8)
project(robot_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(motor REQUIRED)
find_package(chassis REQUIRED)
find_package(scara_arm REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/platform.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  motor::robot_driver_pd42
  chassis::chassis
  scara_arm::scara_arm
)

add_executable(platform_node src/platform_node.cpp)
ament_target_dependencies(platform_node
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  robot_interfaces
  std_srvs
  tf2
  tf2_ros
)
target_link_libraries(platform_node ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS platform_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs sensor_msgs action_msgs motor chassis scara_arm robot_interfaces std_srvs tf2 tf2_ros)

ament_package()
