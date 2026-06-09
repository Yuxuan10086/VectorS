cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "srv/SetDriveMode.srv"
  "srv/GetMotorState.srv"
  "srv/GetMotorSpeedLoopPid.srv"
  "srv/SetMotorSpeedLoopPid.srv"
  "action/ChassisMove.action"
  "action/ChassisSpan.action"
  "action/ArmCalibrate.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime action_msgs builtin_interfaces)

ament_package()
