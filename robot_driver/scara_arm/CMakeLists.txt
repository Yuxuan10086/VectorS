cmake_minimum_required(VERSION 3.8)
project(scara_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(motor REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/arm_joint.cpp
  src/motion_playback.cpp
  src/robot_arm.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC motor::robot_driver_pd42)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  SCARA_ARM_RECORDINGS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/recordings")

find_package(rclcpp REQUIRED)

add_executable(scara_arm_joints_repl examples/scara_arm_joints_repl.cpp)
target_include_directories(scara_arm_joints_repl PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(scara_arm_joints_repl PRIVATE
  rclcpp::rclcpp
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS scara_arm_joints_repl
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY recordings/
  DESTINATION share/${PROJECT_NAME}/recordings
  USE_SOURCE_PERMISSIONS
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(motor)

ament_package()
