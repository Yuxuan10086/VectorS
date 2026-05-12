cmake_minimum_required(VERSION 3.8)
project(motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

add_library(robot_driver_pd42 STATIC
  src/can_interface.cpp
  src/pd42_motor.cpp
)
target_include_directories(robot_driver_pd42 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_driver_pd42 PUBLIC Threads::Threads)

add_executable(pd42_cycle_example example/test.cpp)
target_link_libraries(pd42_cycle_example PRIVATE robot_driver_pd42)

add_executable(pd42_test2 example/test2.cpp)
target_link_libraries(pd42_test2 PRIVATE robot_driver_pd42)

install(TARGETS pd42_cycle_example pd42_test2
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS robot_driver_pd42
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(Threads)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
