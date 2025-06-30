cmake_minimum_required(VERSION 3.8)
project(runtest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(test01 src/test01.cpp)
add_executable(test02 src/test02.cpp)
add_executable(final_controller src/final_controller.cpp)

ament_target_dependencies(test01 rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs)
ament_target_dependencies(test02 rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs)
ament_target_dependencies(final_controller rclcpp geometry_msgs)

install(TARGETS
  test01
  test02
  final_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
